// vehicle.c — raycast-suspension vehicle with discretized brush-model tires
// (docs/TIRE-MODEL.md, replacing the Pacejka MF of ADR-007) contacting the
// analytic C1 road surface of docs/ROAD-SURFACE.md.
//
// Chassis: one dynamic Box3D rigid body (box hull ~1.9 x 1.1 x 4.4 m; sprung
// mass 1262 kg with CoM + inertia tensor computed from the docs/SUSPENSION.md
// §4 mass layout). Wheels are NOT Box3D bodies: each is a 22 kg unsprung
// quarter-car corner with 1-DOF strut travel below the hardpoint
// (docs/SUSPENSION.md §1) — a 200 kN/m tire vertical spring against the
// analytic road (road_query(); mesh raycast only as the off-domain fallback)
// works against the suspension spring/damper to the chassis. Fz for the
// brush contact patch IS the tire spring force, so kerb/bump load spikes
// come from tire stiffness + unsprung dynamics. Kinematic camber/toe follow
// spring travel and camber feeds the patch as equivalent lateral slip
// (docs/SUSPENSION.md §2-3). The patch is N=16 bristle elements under a
// parabolic pressure distribution; grip, combined-slip budget sharing and
// the pneumatic-trail collapse all emerge from the per-element
// adhesion/sliding split. A two-node thermal layer (T_surf/T_core per tire,
// plus track conduction) scales friction with surface temperature. The
// per-wheel pipeline (strut -> slip -> brush -> thermal -> spin) runs 4
// fixed sub-steps per 400 Hz tick (ROAD-SURFACE §3).
//
// Coordinate convention (recorded in physics/NOTES.md):
//   chassis local: +X right, +Y up, +Z forward. Yaw about +Y; yaw = 0 faces +Z.
//   world: Y up (Box3D default gravity direction is -Y).
//
// Determinism notes: all transcendental math goes through Box3D's
// deterministic b3ComputeCosSin/b3Atan2; sqrtf lowers to the IEEE-exact
// hardware/wasm sqrt instruction. No time(), no rand(), no I/O.

#include "vehicle.h"

#include "box3d/math_functions.h"

#include <math.h> // sqrtf only (IEEE-754 exact, bit-stable in wasm)

// ---------------------------------------------------------------------------
// Tunables — every vehicle parameter lives in this one struct.
// ---------------------------------------------------------------------------

typedef struct VehicleTuning
{
	// Chassis. Mass properties (CoM + inertia tensor) are COMPUTED from the
	// docs/SUSPENSION.md §4 mass layout in vehicle_mass_data(), not hand-set.
	float half_extent_x; // half width (m)
	float half_extent_y; // half height (m)
	float half_extent_z; // half length (m)
	float mass;			 // TOTAL vehicle mass (kg); sprung = mass - 4*unsprung_mass

	// Aerodynamics: F = -drag_coef * v * |v|, applied at center of mass
	float drag_coef; // 0.5 * rho * Cd * A
	// Downforce: F = -cl * v^2 along body up, applied per axle. This is what
	// makes it a RACE car — grip that grows with speed (skidpad-diagnosed:
	// without it the ceiling is ~0.9 g, street-car territory).
	float aero_cl_front; // N/(m/s)^2 at the front axle
	float aero_cl_rear;	 // N/(m/s)^2 at the rear axle (rear-biased: stability)

	// Suspension (per wheel)
	float hardpoint_y;	  // chassis-local Y of the suspension hardpoints
	float track_half;	  // chassis-local |X| of the hardpoints
	float wheelbase_half; // chassis-local |Z| of the hardpoints
	float rest_length;	  // spring rest length (m)
	float max_travel;	  // max compression (m)
	float wheel_radius;	  // m
	float spring_k;		  // N/m
	float damper_bump;	  // N s/m (compressing)
	float damper_rebound; // N s/m (extending)

	// Quarter-car unsprung corner (docs/SUSPENSION.md §1)
	float unsprung_mass; // kg per corner (wheel + brake + upright)
	float tire_k;		 // tire vertical spring rate (N/m)
	float tire_c;		 // tire vertical damping (N s/m)

	// Kinematic camber/toe (docs/SUSPENSION.md §2). Conventional signs:
	// camber negative = top of the wheel toward the centerline; toe positive =
	// toe-IN (leading edge inward). Bump curves are linear in spring travel
	// about the per-axle static settle compression below.
	float camber_static_front;	// rad
	float camber_static_rear;	// rad
	float camber_bump_gain;		// rad per m of compression (both axles)
	float toe_in_front;			// rad per side (+ = in; front runs toe-OUT)
	float toe_in_rear;			// rad per side
	float toe_bump_gain_rear;	// rad per m of compression (rear only)
	float settle_comp_front;	// static settle spring compression, front (m)
	float settle_comp_rear;		// static settle spring compression, rear (m)
	float camber_thrust;		// sigma_y += camber_thrust * sin(gamma) (§3)

	// Tires — discretized brush contact patch (docs/TIRE-MODEL.md §1).
	// brush_mu_s is the BRISTLE-level static friction coefficient; the
	// patch-level peak grip that the car actually sees is lower because the
	// rear of the patch always slides at mu_k. Tuned so the emergent peak is
	// ~1.02 * (mu_s0 = 1.05) * Fz — see NOTES.md "Brush tire model".
	float brush_cp;			// bristle stiffness per unit length^2 (N/m^2)
	float brush_a0;			// patch half-length at reference load (m)
	float brush_fz0;		// reference load Fz0 (N)
	float brush_mu_s;		// bristle static friction at T_opt
	float brush_mu_k_ratio; // kinetic/static friction ratio
	float max_load;		  // Fz clamp (N) to bound force spikes
	float slip_v_min;	  // low-speed epsilon for slip computation (m/s)
	float wheel_inertia;  // kg m^2 per wheel

	// Tire thermal layer, two nodes per tire (docs/TIRE-MODEL.md §2,
	// docs/ROAD-SURFACE.md §2 for track conduction)
	float thermal_t_amb;   // ambient temperature (deg C)
	float thermal_t_opt;   // optimal surface temperature (deg C)
	float thermal_k_t;	   // grip falloff: mu_T = 1 - k_t * (T_surf - T_opt)^2
	float thermal_c_surf;  // surface-layer heat capacity (J/K)
	float thermal_h_conv;  // convective cooling rate (1/s, scaled by speed)
	float thermal_h_int;   // surface->core exchange as seen by the surface (1/s)
	float thermal_h_int2;  // surface->core exchange as seen by the core (1/s)
	float thermal_t_track; // asphalt temperature (deg C)
	float thermal_h_track; // surface->track conduction while in contact (1/s)

	// Drivetrain (RWD, fixed single ratio)
	float gear_ratio;		// engine:wheel
	float driveline_eff;	// torque efficiency
	float engine_rpm_pts[5];
	float engine_trq_pts[5]; // Nm at engine, piecewise-linear lookup

	// Brakes
	float brake_torque_front; // Nm per front wheel at full brake
	float brake_torque_rear;  // Nm per rear wheel at full brake
	float handbrake_torque;	  // Nm per rear wheel when handbrake held

	// Steering
	float max_steer; // rad (±30°)

	// Force-feedback signal (docs/TIRE-MODEL.md §3). Output-only; the
	// pneumatic trail is emergent from the brush patch, not a tunable.
	float ffb_caster_trail;	  // mechanical (caster) trail (m)
	float ffb_scrub_radius;	  // kingpin scrub radius (m)
	float ffb_steering_ratio; // rack:rim ratio
} VehicleTuning;

static const VehicleTuning kTuning = {
	.half_extent_x = 0.95f,
	.half_extent_y = 0.55f,
	.half_extent_z = 2.2f,
	.mass = 1350.0f,

	.drag_coef = 0.42f,
	.aero_cl_front = 1.1f,
	.aero_cl_rear = 1.4f,

	.hardpoint_y = -0.25f,
	.track_half = 0.80f,
	.wheelbase_half = 1.30f,
	.rest_length = 0.35f,
	.max_travel = 0.25f,
	.wheel_radius = 0.33f,
	.spring_k = 60000.0f,
	.damper_bump = 4500.0f,
	.damper_rebound = 5200.0f,

	.unsprung_mass = 22.0f,
	.tire_k = 200000.0f, // docs/SUSPENSION.md §1
	.tire_c = 300.0f,

	// docs/SUSPENSION.md §2: camber -1.5 deg F / -1.0 deg R; camber gain
	// -1.0 deg per 25 mm compression; toe +0.05 deg OUT front / +0.15 deg IN
	// rear; bump toe +0.10 deg per 25 mm on the rear only.
	.camber_static_front = -0.02617994f, // -1.5 deg
	.camber_static_rear = -0.01745329f,	 // -1.0 deg
	.camber_bump_gain = -0.69813170f,	 // -1.0 deg / 0.025 m
	.toe_in_front = -8.7266462e-4f,		 // 0.05 deg toe-out
	.toe_in_rear = 2.6179939e-3f,		 // 0.15 deg toe-in
	.toe_bump_gain_rear = 0.069813170f,	 // +0.10 deg / 0.025 m
	// Static settle spring compression per axle: sprung weight 1262*9.81 N on
	// the CoM at z = -0.13393 between hardpoints z = +/-1.3 gives per-wheel
	// front 2776.2 N -> 0.04627 m and rear 3414.0 N -> 0.05690 m at 60 kN/m.
	.settle_comp_front = 0.046270f,
	.settle_comp_rear = 0.056898f,
	.camber_thrust = 0.6f, // docs/SUSPENSION.md §3

	.brush_cp = 7.0e6f,
	.brush_a0 = 0.075f,
	.brush_fz0 = 3500.0f,
	.brush_mu_s = 1.48f,
	.brush_mu_k_ratio = 0.65f,
	.max_load = 9000.0f,
	.slip_v_min = 0.8f,
	.wheel_inertia = 1.2f,

	.thermal_t_amb = 25.0f,
	.thermal_t_opt = 85.0f,
	// Skidpad-retuned (sustained cornering drove the loaded front to 157 C and
	// a 20% grip cliff within ~90 s): gentler falloff, higher floor, 2x thermal
	// mass so the equilibrium arrives over minutes, not one lap.
	.thermal_k_t = 2.5e-5f, // 0.09 / 60^2: 25 C and 145 C both give 0.91
	.thermal_c_surf = 3000.0f,
	.thermal_h_conv = 0.008f,
	.thermal_h_int = 0.006f,
	.thermal_h_int2 = 0.002f,
	.thermal_t_track = 30.0f, // ROAD-SURFACE §2
	.thermal_h_track = 0.02f, // ROAD-SURFACE §2

	// Wheel-pipeline sub-stepping (docs/ROAD-SURFACE.md §3): fixed 4 inner
	// steps of SIM_DT/4 (1600 Hz) — see SIM_TIRE_SUBSTEPS below.

	.gear_ratio = 8.2f,
	.driveline_eff = 0.90f,
	.engine_rpm_pts = { 1000.0f, 3000.0f, 5000.0f, 6500.0f, 7500.0f },
	.engine_trq_pts = { 220.0f, 320.0f, 340.0f, 300.0f, 0.0f },

	.brake_torque_front = 1600.0f,
	.brake_torque_rear = 1050.0f,
	.handbrake_torque = 2500.0f,

	.max_steer = 0.5235988f, // 30 degrees

	.ffb_caster_trail = 0.025f,
	.ffb_scrub_radius = 0.008f,
	.ffb_steering_ratio = 13.0f,
};

#define TWO_PI_F 6.2831855f

// Wheel order: 0=FL 1=FR 2=RL 3=RR. Front wheels steer; rear wheels drive.
static b3Vec3 sVehicleHardpoint( int i )
{
	const VehicleTuning* t = &kTuning;
	float x = ( i == 0 || i == 2 ) ? -t->track_half : t->track_half;
	float z = ( i < 2 ) ? t->wheelbase_half : -t->wheelbase_half;
	return ( b3Vec3 ){ x, t->hardpoint_y, z };
}

static int sIsFront( int i )
{
	return i < 2;
}

// ---------------------------------------------------------------------------
// Mass layout -> composite CoM + inertia tensor (docs/SUSPENSION.md §4)
// ---------------------------------------------------------------------------

// Point masses of the layout table (the tub/body is the uniform box below;
// the 4 unsprung corners are appended from the hardpoint positions).
typedef struct MassPoint
{
	float m;
	b3Vec3 p;
} MassPoint;

#define SIM_BOX_MASS 900.0f // tub/body: uniform 1.9 x 1.1 x 4.4 box at origin

static const MassPoint kMassPoints[] = {
	{ 220.0f, { 0.0f, -0.10f, -0.90f } }, // engine + gearbox
	{ 80.0f, { 0.0f, 0.00f, 0.20f } },	  // driver
	{ 40.0f, { 0.0f, -0.20f, -0.30f } },  // fuel
	{ 22.0f, { 0.0f, -0.30f, 0.60f } },	  // ballast (remainder to 1350 total)
};
#define SIM_MASS_POINT_COUNT 4

// Composite CoM + principal (diagonal) inertia: box formula for the tub,
// parallel-axis for every point mass. The small Iyz product (~10 kg m^2,
// <1% of pitch) is dropped — principal axes assumed body-aligned. Fixed
// float math, fixed iteration order: bit-identical native/wasm.
b3MassData vehicle_mass_data( void )
{
	const VehicleTuning* t = &kTuning;

	// Gather all point masses: table + 4 unsprung corners at the hardpoints.
	MassPoint pts[SIM_MASS_POINT_COUNT + SIM_WHEEL_COUNT];
	for ( int i = 0; i < SIM_MASS_POINT_COUNT; ++i )
	{
		pts[i] = kMassPoints[i];
	}
	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		pts[SIM_MASS_POINT_COUNT + i].m = t->unsprung_mass;
		pts[SIM_MASS_POINT_COUNT + i].p = sVehicleHardpoint( i );
	}

	// Composite CoM (box sits at the origin, contributes no moment).
	float total = SIM_BOX_MASS;
	b3Vec3 moment = b3Vec3_zero;
	for ( int i = 0; i < SIM_MASS_POINT_COUNT + SIM_WHEEL_COUNT; ++i )
	{
		total += pts[i].m;
		moment = b3MulAdd( moment, pts[i].m, pts[i].p );
	}
	b3Vec3 com = b3MulSV( 1.0f / total, moment );

	// Uniform box about its own center, then parallel-axis to the CoM.
	float w = 2.0f * t->half_extent_x;
	float h = 2.0f * t->half_extent_y;
	float d = 2.0f * t->half_extent_z;
	float ixx = ( SIM_BOX_MASS / 12.0f ) * ( h * h + d * d ) + SIM_BOX_MASS * ( com.y * com.y + com.z * com.z );
	float iyy = ( SIM_BOX_MASS / 12.0f ) * ( w * w + d * d ) + SIM_BOX_MASS * ( com.x * com.x + com.z * com.z );
	float izz = ( SIM_BOX_MASS / 12.0f ) * ( w * w + h * h ) + SIM_BOX_MASS * ( com.x * com.x + com.y * com.y );

	for ( int i = 0; i < SIM_MASS_POINT_COUNT + SIM_WHEEL_COUNT; ++i )
	{
		float dx = pts[i].p.x - com.x;
		float dy = pts[i].p.y - com.y;
		float dz = pts[i].p.z - com.z;
		ixx += pts[i].m * ( dy * dy + dz * dz );
		iyy += pts[i].m * ( dx * dx + dz * dz );
		izz += pts[i].m * ( dx * dx + dy * dy );
	}

	// Translational mass is the SPRUNG mass (docs/SUSPENSION.md §1): the four
	// unsprung corners carry their own weight to the ground through the tire
	// springs. Rotationally the wheels move with the chassis (only their
	// strut DOF is separate), so the tensor keeps the full layout.
	b3MassData md;
	md.mass = total - 4.0f * t->unsprung_mass;
	md.center = com;
	md.inertia = ( b3Matrix3 ){
		.cx = { ixx, 0.0f, 0.0f },
		.cy = { 0.0f, iyy, 0.0f },
		.cz = { 0.0f, 0.0f, izz },
	};
	return md;
}

// ---------------------------------------------------------------------------
// Creation / reset
// ---------------------------------------------------------------------------

static b3Quat sYawQuat( float yaw )
{
	// Rotation about world +Y using Box3D's deterministic cos/sin.
	b3CosSin cs = b3ComputeCosSin( 0.5f * yaw );
	b3Quat q = { { 0.0f, cs.sine, 0.0f }, cs.cosine };
	return q;
}

void vehicle_create( b3WorldId world, Vehicle* v, b3Vec3 pos, float yaw )
{
	const VehicleTuning* t = &kTuning;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = pos;
	bodyDef.rotation = sYawQuat( yaw );
	bodyDef.enableSleep = false; // forces are ignored on sleeping bodies
	bodyDef.name = "chassis";

	v->chassis = b3CreateBody( world, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f; // overridden by explicit mass data below
	shapeDef.baseMaterial.friction = 0.5f;
	shapeDef.baseMaterial.restitution = 0.0f;
	shapeDef.filter.categoryBits = SIM_CAT_CHASSIS;
	shapeDef.filter.maskBits = SIM_CAT_TERRAIN;

	b3BoxHull box = b3MakeBoxHull( t->half_extent_x, t->half_extent_y, t->half_extent_z );
	b3CreateHullShape( v->chassis, &shapeDef, &box.base );

	// Mass properties computed from the docs/SUSPENSION.md §4 mass layout
	// (sprung translational mass, full-layout CoM + principal tensor).
	b3Body_SetMassData( v->chassis, vehicle_mass_data() );

	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		v->wheels[i] = ( WheelRuntime ){ 0 };
		v->wheels[i].travel = t->rest_length; // full droop: the car hangs
		v->wheels[i].t_surf = t->thermal_t_amb;
		v->wheels[i].t_core = t->thermal_t_amb;
	}
	v->valid = 1;
}

void vehicle_destroy( Vehicle* v )
{
	if ( v->valid && b3Body_IsValid( v->chassis ) )
	{
		b3DestroyBody( v->chassis );
	}
	v->valid = 0;
}

void vehicle_reset( b3WorldId world, Vehicle* v, b3Vec3 pos, float yaw )
{
	(void)world;
	b3Body_SetTransform( v->chassis, pos, sYawQuat( yaw ) );
	b3Body_SetLinearVelocity( v->chassis, b3Vec3_zero );
	b3Body_SetAngularVelocity( v->chassis, b3Vec3_zero );
	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		v->wheels[i] = ( WheelRuntime ){ 0 };
		v->wheels[i].travel = kTuning.rest_length; // full droop, as created
		// Tires back to ambient: a reset run must replay identically.
		v->wheels[i].t_surf = kTuning.thermal_t_amb;
		v->wheels[i].t_core = kTuning.thermal_t_amb;
	}
	v->rack_torque = 0.0f;
}

// ---------------------------------------------------------------------------
// Tire model — discretized brush contact patch (docs/TIRE-MODEL.md §1)
// ---------------------------------------------------------------------------

// Fixed bristle count: determinism requires a compile-time loop bound.
#define SIM_BRUSH_N 16

// Evaluate the brush patch for one tire. Inputs: slip vector sigma =
// (slip_ratio, tan(slip_angle)), vertical load fz (N), surface temperature
// t_surf (deg C). Outputs: fx along wheel forward, fy along wheel side (left),
// and the emergent pneumatic trail (m, distance of the lateral-force centroid
// BEHIND the patch center; positive at small slip, collapses to ~0 past the
// grip peak — the spec's t_p = -Mz/Fy under a forward-positive moment axis).
//
// Per element (position x from the leading edge, parabolic pressure q):
//   adhesion: f = cp * |sigma| * x       while cp*|sigma|*x <= mu_s(T)*q(x)
//   sliding:  f = mu_k(T) * q(x)         from the breakaway point rearward
// Both act along sigma-hat, so longitudinal and lateral demand share one
// friction budget with no ellipse hack. Non-static so test_tire can sweep it
// directly; NOT in the wasm export list (CONTRACTS §1.1 surface unchanged).
void vehicle_brush_patch( float sigma_x, float sigma_y, float fz, float t_surf, float* out_fx, float* out_fy,
						  float* out_trail )
{
	const VehicleTuning* t = &kTuning;

	float a = t->brush_a0 * sqrtf( fz > 0.0f ? fz / t->brush_fz0 : 0.0f );
	float sig = sqrtf( sigma_x * sigma_x + sigma_y * sigma_y );
	if ( sig < 1.0e-6f || fz <= 0.0f )
	{
		*out_fx = 0.0f;
		*out_fy = 0.0f;
		*out_trail = a * ( 1.0f / 3.0f ); // adhesion-only limit as |sigma| -> 0
		return;
	}
	float inv_sig = 1.0f / sig;
	float sx_hat = sigma_x * inv_sig;
	float sy_hat = sigma_y * inv_sig;

	// Thermal grip factor (docs/TIRE-MODEL.md §2)
	float dT = t_surf - t->thermal_t_opt;
	float mu_t = 1.0f - t->thermal_k_t * dT * dT;
	mu_t = b3ClampFloat( mu_t, 0.88f, 1.00f );
	float mu_s = t->brush_mu_s * mu_t;
	float mu_k = t->brush_mu_k_ratio * mu_s;

	float w = ( 2.0f * a ) / (float)SIM_BRUSH_N; // element width
	float q_scale = ( 3.0f * fz ) / ( 4.0f * a ); // parabolic pressure, N/m

	float f_sum = 0.0f; // total force magnitude along sigma-hat
	float m_sum = 0.0f; // first moment of that force about the patch center
	for ( int j = 0; j < SIM_BRUSH_N; ++j )
	{
		float x = ( (float)j + 0.5f ) * w; // element center, from leading edge
		float xi = x / a - 1.0f;		   // [-1, 1] across the patch
		float q = q_scale * ( 1.0f - xi * xi );
		float f_adh = t->brush_cp * sig * x;
		float f = ( f_adh <= mu_s * q ) ? f_adh : mu_k * q;
		float fe = f * w;
		f_sum += fe;
		m_sum += fe * ( x - a );
	}

	*out_fx = f_sum * sx_hat;
	*out_fy = f_sum * sy_hat;
	*out_trail = ( f_sum > 1.0e-3f ) ? ( m_sum / f_sum ) : a * ( 1.0f / 3.0f );
}

// Advance the two-node thermal state of one tire by dt (docs/TIRE-MODEL.md §2
// + docs/ROAD-SURFACE.md §2). power = friction power into the surface (W),
// speed = vehicle speed (m/s) for convective cooling; while in contact the
// surface also sheds heat into the asphalt it touches (track conduction).
// Plain float Euler integration — deterministic by construction, never hashed
// directly. Non-static so tests/test_road.c can drive the lockup flash-heat
// gate; NOT in the wasm export list.
void vehicle_tire_thermal( WheelRuntime* w, float power, float speed, int in_contact, float dt )
{
	const VehicleTuning* t = &kTuning;
	float d_surf = power / t->thermal_c_surf - t->thermal_h_conv * ( 1.0f + 0.05f * speed ) * ( w->t_surf - t->thermal_t_amb ) -
				   t->thermal_h_int * ( w->t_surf - w->t_core );
	if ( in_contact )
	{
		d_surf -= t->thermal_h_track * ( w->t_surf - t->thermal_t_track );
	}
	float d_core = t->thermal_h_int2 * ( w->t_surf - w->t_core );
	w->t_surf += d_surf * dt;
	w->t_core += d_core * dt;
}

// ---------------------------------------------------------------------------
// Quarter-car unsprung corner (docs/SUSPENSION.md §1)
// ---------------------------------------------------------------------------

// One 1600 Hz sub-step of the 1-DOF strut travel. travel is measured DOWN
// along the strut (-chassis up) from the hardpoint to the wheel center, so:
//   spring compression c = rest_length - travel,  c_dot = -travel_vel
//   tire penetration   p = travel + wheel_radius - hit_dist
// Forces on the unsprung mass along strut-down: +suspension (a compressed
// spring pushes the wheel away from the chassis), +gravity, -tire. Chassis
// pose is frozen within the tick, so hit_dist / hit_dist_dot are per-tick
// constants supplied by the caller. Semi-implicit Euler (velocity first):
// omega*dt ~ 0.07 at 1600 Hz, comfortably stable. mul/add/div/compare only.
void vehicle_suspension_step( WheelRuntime* w, float hit_dist, float hit_dist_dot, float g_along, float dt,
							  float* out_tire_force, float* out_susp_force )
{
	const VehicleTuning* t = &kTuning;

	// Suspension spring/damper between chassis and unsprung mass. Travel
	// clamps keep c in [0, max_travel]; the damper may pull (rebound), so the
	// strut force is clamped symmetrically, not at zero.
	float c = t->rest_length - w->travel;
	float c_dot = -w->travel_vel;
	float damper = c_dot >= 0.0f ? t->damper_bump : t->damper_rebound;
	float f_susp = t->spring_k * c + damper * c_dot;
	f_susp = b3ClampFloat( f_susp, -t->max_load, t->max_load );

	// Tire vertical spring against the road (Fz for the brush model). Pushes
	// only (clamped >= 0); force-free the instant the tread leaves the road.
	float pen = w->travel + t->wheel_radius - hit_dist;
	float f_tire = 0.0f;
	if ( pen > 0.0f )
	{
		f_tire = t->tire_k * pen + t->tire_c * ( w->travel_vel - hit_dist_dot );
		f_tire = b3ClampFloat( f_tire, 0.0f, t->max_load );
	}

	// m_u * dv = F_susp + m_u*g_along - F_tire  (strut-down positive; this is
	// the spec's m_u*dv = F_t - F_susp - m_u*g with v measured strut-up)
	float accel = ( f_susp - f_tire ) / t->unsprung_mass + g_along;
	w->travel_vel += accel * dt;
	w->travel += w->travel_vel * dt;

	// Travel clamps: kill velocity INTO the stop only.
	float travel_min = t->rest_length - t->max_travel;
	if ( w->travel > t->rest_length )
	{
		w->travel = t->rest_length;
		if ( w->travel_vel > 0.0f )
		{
			w->travel_vel = 0.0f;
		}
	}
	else if ( w->travel < travel_min )
	{
		w->travel = travel_min;
		if ( w->travel_vel < 0.0f )
		{
			w->travel_vel = 0.0f;
		}
	}

	*out_tire_force = f_tire;
	*out_susp_force = f_susp;
}

// ---------------------------------------------------------------------------
// Kinematic camber/toe + camber thrust (docs/SUSPENSION.md §2, §3)
// ---------------------------------------------------------------------------

void vehicle_wheel_kinematics( int wheel, float compression, float* out_camber, float* out_steer_add )
{
	const VehicleTuning* t = &kTuning;
	int front = wheel < 2;
	float bump = compression - ( front ? t->settle_comp_front : t->settle_comp_rear );

	float camber = ( front ? t->camber_static_front : t->camber_static_rear ) + t->camber_bump_gain * bump;
	float toe_in = front ? t->toe_in_front : ( t->toe_in_rear + t->toe_bump_gain_rear * bump );

	// Toe-in points the leading edge at the centerline: the LEFT wheel (x<0,
	// indices 0/2) steers right (-), the RIGHT wheel steers left (+).
	float mirror = ( wheel == 0 || wheel == 2 ) ? 1.0f : -1.0f; // +1 on left wheels
	*out_camber = camber;
	*out_steer_add = -mirror * toe_in;
}

float vehicle_camber_thrust_sigma( float gamma )
{
	b3CosSin cs = b3ComputeCosSin( gamma );
	return kTuning.camber_thrust * cs.sine;
}

static float sEngineTorque( float rpm )
{
	const VehicleTuning* t = &kTuning;
	if ( rpm <= t->engine_rpm_pts[0] )
	{
		return t->engine_trq_pts[0];
	}
	for ( int i = 1; i < 5; ++i )
	{
		if ( rpm <= t->engine_rpm_pts[i] )
		{
			float f = ( rpm - t->engine_rpm_pts[i - 1] ) / ( t->engine_rpm_pts[i] - t->engine_rpm_pts[i - 1] );
			return t->engine_trq_pts[i - 1] + f * ( t->engine_trq_pts[i] - t->engine_trq_pts[i - 1] );
		}
	}
	return 0.0f; // past redline
}

// Fixed sub-step count for the per-wheel pipeline (docs/ROAD-SURFACE.md §3):
// slip -> brush patch -> thermal -> wheel spin at 1600 Hz inside each 400 Hz
// tick. Compile-time constant — determinism requires a fixed loop bound.
#define SIM_TIRE_SUBSTEPS 4

// One sub-step of wheel spin dynamics: drive torque (RWD), tire reaction
// torque, brake clamp that cannot reverse the wheel within the step, and the
// handbrake hard-lock on the rears. Shared by the contact and airborne paths
// (the airborne path previously used an unclamped brake integration that
// could oscillate through zero — the clamped form is strictly better).
static void sWheelSpinStep( WheelRuntime* w, int front, int handbrake, float throttle, float brake,
							float reaction_torque, float dt )
{
	const VehicleTuning* t = &kTuning;

	float drive = 0.0f;
	if ( !front && !handbrake )
	{
		float rpm = w->omega * t->gear_ratio * ( 60.0f / TWO_PI_F );
		if ( rpm < 1000.0f )
		{
			rpm = 1000.0f;
		}
		drive = throttle * sEngineTorque( rpm ) * t->gear_ratio * t->driveline_eff * 0.5f;
	}

	float brake_cap = front ? t->brake_torque_front : t->brake_torque_rear;
	float brake_trq = brake * brake_cap;
	if ( handbrake && !front )
	{
		brake_trq += t->handbrake_torque;
	}

	float omega_new = w->omega + ( ( drive + reaction_torque ) / t->wheel_inertia ) * dt;
	float brake_dw = ( brake_trq / t->wheel_inertia ) * dt;
	if ( omega_new > 0.0f )
	{
		omega_new = omega_new > brake_dw ? omega_new - brake_dw : 0.0f;
	}
	else if ( omega_new < 0.0f )
	{
		omega_new = omega_new < -brake_dw ? omega_new + brake_dw : 0.0f;
	}

	// Handbrake locks the rears outright.
	if ( handbrake && !front )
	{
		omega_new = 0.0f;
	}

	w->omega = omega_new;
	w->spin_angle += w->omega * dt;
}

// Wrap the accumulated spin angle once per tick (|omega|·SIM_DT << pi).
static void sWrapSpin( WheelRuntime* w )
{
	if ( w->spin_angle > 3.1415927f )
	{
		w->spin_angle -= TWO_PI_F;
	}
	else if ( w->spin_angle < -3.1415927f )
	{
		w->spin_angle += TWO_PI_F;
	}
}

// ---------------------------------------------------------------------------
// Per-tick update
// ---------------------------------------------------------------------------

void vehicle_update( b3WorldId world, Vehicle* v, const Road* road, float steer, float throttle, float brake,
					 uint32_t flags )
{
	const VehicleTuning* t = &kTuning;
	b3BodyId chassis = v->chassis;

	b3WorldTransform xf = b3Body_GetTransform( chassis );
	b3Quat q = xf.q;
	b3Vec3 up = b3RotateVector( q, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } );
	b3Vec3 fwd = b3RotateVector( q, ( b3Vec3 ){ 0.0f, 0.0f, 1.0f } );

	int handbrake = ( flags & SIM_FLAG_HANDBRAKE ) != 0;

	// Vehicle speed for tire convective cooling (and reused for drag below).
	b3Vec3 chassis_vel = b3Body_GetLinearVelocity( chassis );
	float chassis_speed = b3Length( chassis_vel );

	// Steering: linear map, front axle only.
	float steer_angle = steer * t->max_steer;

	// FFB: front-axle rack torque accumulated over the wheel loop.
	float rack = 0.0f;

	b3QueryFilter rayFilter = b3DefaultQueryFilter();
	rayFilter.categoryBits = SIM_CAT_CHASSIS;
	rayFilter.maskBits = SIM_CAT_TERRAIN;

	const float ray_len = t->rest_length + t->wheel_radius;
	const float sub_dt = SIM_DT / (float)SIM_TIRE_SUBSTEPS;

	// Gravity component along strut-down (-up): the same for all four corners.
	const float g_along = 9.81f * up.y;

	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		WheelRuntime* w = &v->wheels[i];
		int front = sIsFront( i );

		// Kinematic camber/toe from the CURRENT spring compression
		// (docs/SUSPENSION.md §2). Travel clamps keep it in [0, max_travel].
		float camber_conv, toe_add;
		vehicle_wheel_kinematics( i, t->rest_length - w->travel, &camber_conv, &toe_add );
		w->steer_angle = ( front ? steer_angle : 0.0f ) + toe_add;

		b3Vec3 hp_local = sVehicleHardpoint( i );
		b3Pos origin = b3Body_GetWorldPoint( chassis, hp_local );

		// --- Wheel contact (docs/ROAD-SURFACE.md §1): analytic road query at
		// the hardpoint projected down; compression from the analytic height
		// along the suspension axis (-chassis up); contact normal from the
		// query. Mesh raycast only as the off-domain fallback — the chassis
		// body still collides with the mesh, only the WHEELS go analytic. ---
		int have_contact = 0;
		float hit_dist = 0.0f;
		b3Vec3 contact_point = origin;
		b3Vec3 contact_normal = up;
		int need_mesh_fallback = 1;

		if ( road != NULL && road->count > 0 )
		{
			if ( !w->road_hint_valid )
			{
				// One-time deterministic bootstrap after create/reset.
				w->road_hint = road_nearest_global( road, origin );
				w->road_hint_valid = 1;
			}
			RoadQuery rq;
			road_query( road, origin, w->road_hint, &rq );
			w->road_hint = rq.seg;

			if ( rq.on_road )
			{
				need_mesh_fallback = 0;
				// Suspension ray x(d) = origin - d·up against the local
				// tangent plane (point rq.point, normal rq.normal).
				float facing = b3Dot( up, rq.normal );
				if ( facing > 0.2f ) // surface must face the ray
				{
					float d = b3Dot( b3Sub( origin, rq.point ), rq.normal ) / facing;
					if ( d <= ray_len )
					{
						have_contact = 1;
						hit_dist = d > 0.0f ? d : 0.0f; // below surface → full compression
						contact_point = b3MulAdd( origin, -hit_dist, up );
						contact_normal = rq.normal;
					}
				}
			}
		}

		if ( need_mesh_fallback )
		{
			// Off the analytic domain (shoulders' outer void, rollover
			// recovery, ...): keep the original mesh raycast.
			b3Vec3 translation = b3MulSV( -ray_len, up );
			b3RayResult ray = b3World_CastRayClosest( world, origin, translation, rayFilter );
			if ( ray.hit )
			{
				have_contact = 1;
				hit_dist = ray.fraction * ray_len;
				contact_point = ray.point;
				contact_normal = ray.normal;
			}
		}

		if ( !have_contact )
		{
			// Airborne: no tire spring, but the strut still works the 22 kg
			// unsprung mass toward full droop (and its reaction still pulls
			// on the chassis), the tire cools (no friction power, no track
			// conduction) and the free wheel still sees engine/brake torque.
			float fsusp_sum = 0.0f;
			for ( int k = 0; k < SIM_TIRE_SUBSTEPS; ++k )
			{
				float f_tire, f_susp;
				vehicle_suspension_step( w, ray_len + 1.0f, 0.0f, g_along, sub_dt, &f_tire, &f_susp );
				fsusp_sum += f_susp;
				vehicle_tire_thermal( w, 0.0f, chassis_speed, 0, sub_dt );
				sWheelSpinStep( w, front, handbrake, throttle, brake, 0.0f, sub_dt );
			}
			sWrapSpin( w );

			const float inv_n_air = 1.0f / (float)SIM_TIRE_SUBSTEPS;
			b3Body_ApplyForce( chassis, b3MulSV( fsusp_sum * inv_n_air, up ), origin, true );

			w->in_contact = 0;
			w->compression = t->rest_length - w->travel;
			w->load = 0.0f;
			w->slip_ratio = 0.0f;
			w->slip_angle = 0.0f;
			w->wheel_center = b3MulAdd( origin, -w->travel, up );
			continue;
		}

		// Rate of change of the contact distance along the strut: the road is
		// static, so d = dot(origin - point, n)/dot(up, n) changes with the
		// hardpoint velocity only (per-tick constant; the ~0.03 deg/tick drift
		// of `up` itself is second order). Feeds the tire-spring damper.
		float hit_dist_dot = 0.0f;
		{
			float facing = b3Dot( up, contact_normal );
			if ( facing > 0.2f )
			{
				b3Vec3 v_hp = b3Body_GetWorldPointVelocity( chassis, origin );
				hit_dist_dot = b3Dot( v_hp, contact_normal ) / facing;
			}
		}

		w->in_contact = 1;
		w->contact_point = contact_point;

		// --- Contact patch basis, projected onto the contact plane ---
		// Wheel forward = chassis forward rotated by steer angle about chassis
		// up, then flattened onto the contact plane so the slip basis follows
		// the continuous analytic camber (banking transitions — the point).
		b3Vec3 wheel_fwd = fwd;
		if ( w->steer_angle != 0.0f )
		{
			b3Quat steer_q;
			{
				b3CosSin cs = b3ComputeCosSin( 0.5f * w->steer_angle );
				steer_q = ( b3Quat ){ { up.x * cs.sine, up.y * cs.sine, up.z * cs.sine }, cs.cosine };
			}
			wheel_fwd = b3RotateVector( steer_q, fwd );
		}
		{
			b3Vec3 proj = b3MulAdd( wheel_fwd, -b3Dot( wheel_fwd, contact_normal ), contact_normal );
			float len2 = b3Dot( proj, proj );
			if ( len2 > 1.0e-6f )
			{
				wheel_fwd = b3MulSV( 1.0f / sqrtf( len2 ), proj );
			}
			// else: degenerate normal — keep the chassis-basis direction.
		}
		b3Vec3 wheel_side = b3Cross( contact_normal, wheel_fwd ); // points left

		// --- Camber vs the ROAD normal (docs/SUSPENSION.md §3): chassis lean
		// about the wheel-forward axis (roll and banking both arrive through
		// the contact-basis projection) plus the kinematic camber, mirrored
		// so "negative camber" leans both wheel tops toward the centerline.
		// gamma > 0 = wheel top leaning LEFT (+side) => thrust to the left.
		float mirror = ( i == 0 || i == 2 ) ? 1.0f : -1.0f; // +1 on left wheels
		float lean = b3Atan2( b3Dot( up, wheel_side ), b3Dot( up, contact_normal ) );
		float gamma = lean + mirror * camber_conv;
		float sigma_camber = vehicle_camber_thrust_sigma( gamma );

		// --- Slip inputs from contact-patch velocity (constant across the
		// tick: the chassis integrates at tick level, only omega sub-steps) ---
		b3Vec3 vel = b3Body_GetWorldPointVelocity( chassis, contact_point );
		float v_long = b3Dot( vel, wheel_fwd );
		float v_lat = b3Dot( vel, wheel_side );

		float denom = b3AbsFloat( v_long );
		if ( denom < t->slip_v_min )
		{
			denom = t->slip_v_min;
		}

		// Slip vector: sigma_x = slip ratio, sigma_y = tan(slip angle) =
		// -v_lat/denom plus the camber-thrust equivalent slip — it flows
		// through the same bristle adhesion/sliding split, so camber thrust
		// saturates with everything else. The EXPORTED slip_angle stays the
		// kinematic one (SimStateV1 field meaning unchanged).
		float sigma_y = -v_lat / denom + sigma_camber;
		w->slip_angle = b3Atan2( -v_lat, denom );

		// --- Sub-stepped per-wheel pipeline (docs/ROAD-SURFACE.md §3 +
		// SUSPENSION.md §1): unsprung strut travel -> slip -> brush patch ->
		// thermal -> wheel spin at SIM_DT/4. Fz for the brush model is the
		// TIRE spring force from the quarter-car, per sub-step. Chassis
		// forces are the sub-step MEAN applied once per tick (Box3D
		// integrates F·SIM_DT, so the mean preserves the summed impulses):
		// strut reaction at the hardpoint, tire in-plane force at the patch.
		float fx_sum = 0.0f;
		float fy_sum = 0.0f;
		float rack_sum = 0.0f;
		float fsusp_sum = 0.0f;
		float fz = 0.0f;
		for ( int k = 0; k < SIM_TIRE_SUBSTEPS; ++k )
		{
			float f_susp;
			vehicle_suspension_step( w, hit_dist, hit_dist_dot, g_along, sub_dt, &fz, &f_susp );
			fsusp_sum += f_susp;

			float slip_ratio = ( w->omega * t->wheel_radius - v_long ) / denom;
			slip_ratio = b3ClampFloat( slip_ratio, -4.0f, 4.0f );
			w->slip_ratio = slip_ratio; // last sub-step's value is exported

			float fx, fy, trail;
			vehicle_brush_patch( slip_ratio, sigma_y, fz, w->t_surf, &fx, &fy, &trail );
			fx_sum += fx;
			fy_sum += fy;

			// FFB rack torque (front axle only, output-only): lateral force
			// behind the steering axis by the EMERGENT pneumatic trail plus
			// mechanical caster (docs/TIRE-MODEL.md §3).
			if ( front )
			{
				rack_sum += fy * ( trail + t->ffb_caster_trail ) + fx * t->ffb_scrub_radius;
			}

			// Thermal: friction power from the slip velocity at the patch,
			// with track conduction while the tread presses the road
			// (ROAD-SURFACE §2).
			float v_slip_x = w->omega * t->wheel_radius - v_long;
			float p_fric = b3AbsFloat( fx * v_slip_x ) + b3AbsFloat( fy * v_lat );
			vehicle_tire_thermal( w, p_fric, chassis_speed, fz > 0.0f, sub_dt );

			// Wheel spin (RWD drive, brakes on all four, tire reaction).
			sWheelSpinStep( w, front, handbrake, throttle, brake, -fx * t->wheel_radius, sub_dt );
		}
		sWrapSpin( w );

		w->load = fz; // last sub-step's tire spring force
		w->compression = t->rest_length - w->travel;
		w->wheel_center = b3MulAdd( origin, -w->travel, up );

		const float inv_n = 1.0f / (float)SIM_TIRE_SUBSTEPS;
		// Strut reaction on the chassis at the hardpoint, along the strut.
		b3Body_ApplyForce( chassis, b3MulSV( fsusp_sum * inv_n, up ), origin, true );
		// Tire in-plane force at the patch (the unsprung mass has no in-plane
		// DOF, so the patch force transfers rigidly to the chassis).
		b3Vec3 tire_force = b3Add( b3MulSV( fx_sum * inv_n, wheel_fwd ), b3MulSV( fy_sum * inv_n, wheel_side ) );
		b3Body_ApplyForce( chassis, tire_force, contact_point, true );

		if ( front )
		{
			rack += rack_sum * inv_n;
		}
	}

	v->rack_torque = rack / t->ffb_steering_ratio;

	// --- Aerodynamic drag at the center of mass ---
	// (chassis_vel/chassis_speed sampled before force accumulation; velocity
	// only changes at the subsequent b3World_Step, so this is the same value.)
	if ( chassis_speed > 0.01f )
	{
		b3Vec3 drag = b3MulSV( -t->drag_coef * chassis_speed, chassis_vel );
		b3Body_ApplyForceToCenter( chassis, drag, true );
	}

	// --- Aerodynamic downforce, per axle along -body-up ---
	{
		float v2 = chassis_speed * chassis_speed;
		b3Pos front_ax = b3Body_GetWorldPoint( chassis, ( b3Vec3 ){ 0.0f, t->hardpoint_y, t->wheelbase_half } );
		b3Pos rear_ax = b3Body_GetWorldPoint( chassis, ( b3Vec3 ){ 0.0f, t->hardpoint_y, -t->wheelbase_half } );
		b3Body_ApplyForce( chassis, b3MulSV( -t->aero_cl_front * v2, up ), front_ax, true );
		b3Body_ApplyForce( chassis, b3MulSV( -t->aero_cl_rear * v2, up ), rear_ax, true );
	}
}

void vehicle_export( const Vehicle* v, SimStateV1* state )
{
	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		const WheelRuntime* w = &v->wheels[i];
		SimWheelStateV1* ws = &state->wheels[i];
		ws->pos[0] = w->wheel_center.x;
		ws->pos[1] = w->wheel_center.y;
		ws->pos[2] = w->wheel_center.z;
		ws->spin_angle = w->spin_angle;
		ws->steer_angle = w->steer_angle;
		ws->susp_compression = w->compression;
		ws->slip_ratio = w->slip_ratio;
		ws->slip_angle = w->slip_angle;
	}
}

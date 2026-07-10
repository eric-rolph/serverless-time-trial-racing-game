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

	// Drivetrain (RWD, sequential 6-speed + reverse, docs/DRIVETRAIN.md §1-3)
	float gear_ratios[7];	 // overall engine:wheel ratios; [0] = reverse (< 0), [1..6] forward
	float driveline_eff;	 // torque efficiency
	float engine_rpm_pts[5];
	float engine_trq_pts[5]; // Nm at engine, piecewise-linear lookup
	float engine_inertia;	 // crank+flywheel inertia (kg m^2), free-spin during shift cuts
	float idle_omega;		 // 900 rpm floor (rad/s) — the engine never stalls
	float clutch_omega;		 // below this coupled speed the auto-clutch slips (1100 rpm, rad/s)
	float clutch_hold_omega; // slipping-clutch engine speed at full throttle (rad/s)
	float clutch_min;		 // launch floor of the clutch torque factor [0..1]
	float creep_throttle;	 // minimum effective throttle in the slip zone (idle creep)
	float limiter_omega;	 // hard fuel cut above this (7500 rpm, rad/s)
	float overrev_omega;	 // downshift protection ceiling (7800 rpm, rad/s)
	float eb_torque_idle;	 // engine braking at idle (Nm, negative)
	float eb_torque_red;	 // engine braking at redline (Nm, negative)
	float eb_fade_throttle;	 // engine braking fades to zero by this throttle
	float k_lsd;			 // viscous LSD coupling (Nm per rad/s of wheel speed difference)
	float lsd_clamp_frac;	 // LSD transfer clamp as a fraction of |transmitted torque|
	float shock_k;			 // downshift driveline shock: axle Nm per rad/s of rev-match snap
	float shock_max;		 // shock torque cap (Nm at the axle)

	// Anti-roll bars (docs/DRIVETRAIN.md §5): F = k_arb * (c_this - c_other)
	// added to the strut force per corner — pure roll-moment redistribution
	// (equal and opposite left/right), meaningful through tire load sensitivity.
	float arb_front; // N per m of left/right spring-compression difference
	float arb_rear;

	// Tire load sensitivity (docs/DRIVETRAIN.md §5):
	// mu_s(Fz) = mu_s0 * clamp(1 - k_load * (Fz - Fz0)/Fz0, lo, hi).
	float k_load;
	float load_clamp_lo;
	float load_clamp_hi;

	// Slip relaxation (docs/DRIVETRAIN.md §5): first-order lag on the slip
	// vector, tau = L/|v| clamped at low speed, at the 1600 Hz substep.
	float relax_len;	 // relaxation length L (m)
	float relax_tau_max; // tau clamp at low speed (s)

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

	// Effective aero coefficient tuned so the TOTAL resistance matches the
	// spec's 0.42 v^2 budget (docs/DRIVETRAIN.md §1): the quarter-car applies
	// strut forces along chassis-up, so rear squat under downforce leaks
	// ~0.05 v^2 of the support force into the horizontal (measured 185 N at
	// 63 m/s via a shift-cut coast). 0.37 + 0.05 = the spec's 0.42.
	.drag_coef = 0.37f,
	.aero_cl_front = 1.1f,
	.aero_cl_rear = 1.4f,

	.hardpoint_y = -0.25f,
	.track_half = 0.84f,
	.wheelbase_half = 1.30f,
	.rest_length = 0.31f,
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
	// 1.48 (patch peak 1.02 x mu_s0 x Fz0) until the drivetrain wave; the
	// docs/DRIVETRAIN.md §5 load sensitivity costs ~3-5% of in-situ grip
	// under transfer, so the §5 balance re-tune raises the bristle level to
	// keep the car in the mandated 1.05-1.10 g skidpad window. Patch peak is
	// now 1.071 x mu_s0 x Fz0 — still inside TIRE-MODEL.md §4.3's
	// [0.95, 1.10] gate (verified in test_tire).
	.brush_mu_s = 1.55f,
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

	// Ratios refined against the torque curve (docs/DRIVETRAIN.md §1): 1st tops
	// 95.2 km/h at the 7500 rpm limiter (785.4/9.8 * 0.33 m); 2nd is kept
	// close (8.2, tops 114 km/h) so the 0-100 sprint stays in the meat of the
	// curve; ~0.79 steps down to 6th = 2.95, whose drag-limited top lands at
	// ~271 km/h (redline speed 316 km/h is never reached: past ~6500 rpm the
	// curve dives under the 0.42 v^2 resistance budget). Reverse = -1st.
	.gear_ratios = { -9.8f, 9.8f, 7.9f, 6.06f, 4.77f, 3.75f, 2.95f },
	.driveline_eff = 0.90f,
	// Torque curve: low/mid end raised in the drivetrain wave so drive force
	// EXCEEDS rear grip through 1st (spec §1: launch wheelspin available) —
	// the old 220/320 low end was tuned for the single 8.2:1 ratio and left
	// the geared car torque-limited below 4500 rpm. Peak power is unchanged
	// (~204 kW at 6500), so the drag-limited top speed is intact.
	.engine_rpm_pts = { 1000.0f, 3000.0f, 5000.0f, 6500.0f, 7500.0f },
	.engine_trq_pts = { 300.0f, 350.0f, 348.0f, 310.0f, 0.0f },
	.engine_inertia = 0.15f,			// docs/DRIVETRAIN.md §2
	.idle_omega = 94.24778f,			// 900 rpm
	.clutch_omega = 115.19173f,			// 1100 rpm (docs/DRIVETRAIN.md §1)
	.clutch_hold_omega = 471.23890f,	// 4500 rpm: launch-flare rpm at full throttle
	.clutch_min = 0.9f,					// clutch torque-factor floor at standstill
	.creep_throttle = 0.04f,			// idle creep through the slipping clutch
	.limiter_omega = 785.39816f,		// 7500 rpm
	.overrev_omega = 816.81409f,		// 7800 rpm
	.eb_torque_idle = -20.0f,
	.eb_torque_red = -60.0f,
	.eb_fade_throttle = 0.10f,
	.k_lsd = 25.0f,			// skidpad-tuned (docs/DRIVETRAIN.md §3)
	.lsd_clamp_frac = 0.40f,
	.shock_k = 1.0f,
	.shock_max = 400.0f,

	.arb_front = 20000.0f, // skidpad-tuned: front-stiffer = mild limit understeer
	.arb_rear = 14000.0f,  // (1.06 g steady state, front slips first by ~1 deg)

	.k_load = 0.07f, // docs/DRIVETRAIN.md §5
	.load_clamp_lo = 0.70f,
	.load_clamp_hi = 1.15f,

	.relax_len = 0.3f,	   // docs/DRIVETRAIN.md §5
	.relax_tau_max = 0.05f,

	.brake_torque_front = 2200.0f, // docs/DRIVETRAIN.md §5: front-biased,
	.brake_torque_rear = 950.0f,   // fronts lock first (stable failure mode)
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

#define SIM_BOX_MASS 900.0f // tub/body: uniform 1.9 x 1.1 x 4.4 box
// Rollover fix (skidpad-derived): with the tub at the origin the composite
// CoM sat ~0.83 m high — rollover threshold 0.97 g vs a ~1.15 g grip ceiling,
// so the car TIPPED at the limit instead of sliding. Race cars mount all the
// heavy bits on the floor: tub center lowered, every point mass dropped.
// Result (with wider track + lower ride in kTuning): threshold ~1.3 g.
#define SIM_BOX_CENTER_Y ( -0.12f ) // floor/undertray-heavy monocoque

static const MassPoint kMassPoints[] = {
	{ 220.0f, { 0.0f, -0.35f, -0.90f } }, // engine + gearbox (dry sump, low)
	{ 80.0f, { 0.0f, -0.15f, 0.20f } },	  // driver (reclined)
	{ 40.0f, { 0.0f, -0.40f, -0.30f } },  // fuel (floor cell)
	{ 22.0f, { 0.0f, -0.55f, 0.60f } },	  // ballast (bolted to the floor)
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

	// Composite CoM (box center contributes its moment at SIM_BOX_CENTER_Y).
	float total = SIM_BOX_MASS;
	b3Vec3 moment = { 0.0f, SIM_BOX_MASS * SIM_BOX_CENTER_Y, 0.0f };
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
	float box_dy = SIM_BOX_CENTER_Y - com.y;
	float ixx = ( SIM_BOX_MASS / 12.0f ) * ( h * h + d * d ) + SIM_BOX_MASS * ( box_dy * box_dy + com.z * com.z );
	float iyy = ( SIM_BOX_MASS / 12.0f ) * ( w * w + d * d ) + SIM_BOX_MASS * ( com.x * com.x + com.z * com.z );
	float izz = ( SIM_BOX_MASS / 12.0f ) * ( w * w + h * h ) + SIM_BOX_MASS * ( com.x * com.x + box_dy * box_dy );

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
	// Crash damage (docs/SOFTBODY.md Phase 2): hit events on the chassis feed
	// the deterministic damage model. Output-only telemetry that also drives
	// physics — the events themselves never alter the step, only what we read.
	shapeDef.enableHitEvents = true;

	b3BoxHull box = b3MakeBoxHull( t->half_extent_x, t->half_extent_y, t->half_extent_z );
	v->chassis_shape = b3CreateHullShape( v->chassis, &shapeDef, &box.base );

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
	v->rack_torque = 0.0f;
	// Drivetrain at spawn: 1st gear engaged, engine at idle, no shift pending.
	v->engine_omega = t->idle_omega;
	v->gear = 1;
	v->shift_ticks = 0;
	v->shift_target = 1;
	v->shift_is_down = 0;
	v->shock_ticks = 0;
	v->shock_torque = 0.0f;
	v->prev_flags = 0;
	v->damage_overall = 0.0f;
	v->damage_steer = 0.0f;
	v->damage_toe_front = 0.0f;
	v->damage_toe_rear = 0.0f;
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
	// Drivetrain back to the spawn state (1st gear, idle, no shift/shock
	// pending, edge-detector cleared) — a reset run must replay identically.
	v->engine_omega = kTuning.idle_omega;
	v->gear = 1;
	v->shift_ticks = 0;
	v->shift_target = 1;
	v->shift_is_down = 0;
	v->shock_ticks = 0;
	v->shock_torque = 0.0f;
	v->prev_flags = 0;
	// A reset run must replay identically: crash damage back to pristine.
	v->damage_overall = 0.0f;
	v->damage_steer = 0.0f;
	v->damage_toe_front = 0.0f;
	v->damage_toe_rear = 0.0f;
}

// ---------------------------------------------------------------------------
// Crash damage (docs/SOFTBODY.md Phase 2)
// ---------------------------------------------------------------------------

// Severity per (m/s) of approach speed above SIM_DAMAGE_THRESHOLD, mapped to a
// normalized [0,1] impact severity. At 0.045: a hit ~22 m/s over threshold is
// "destroyed" (sev=1); a solid ~15 m/s total wall strike (~9 over) lands ~0.4.
#define SIM_DAMAGE_SEV_PER_MS 0.045f
// Per-component gains from one impact's severity.
#define SIM_DAMAGE_K_OVERALL 0.60f	  // damage_overall increment at sev=1
#define SIM_DAMAGE_K_STEER 0.55f	  // damage_steer increment (front impacts)
#define SIM_DAMAGE_K_TOE 0.015f		  // toe bend (rad) increment at sev=1
#define SIM_DAMAGE_TOE_CLAMP 0.030f	  // |toe damage| clamp (rad) ~1.7 deg
// Effect magnitudes felt in vehicle_update.
#define SIM_DAMAGE_AERO_LOSS 0.40f	  // cl *= (1 - LOSS*damage_overall)
#define SIM_DAMAGE_STEER_LOSS 0.30f	  // max_steer *= (1 - LOSS*damage_steer)

static float sClampMag( float x, float m )
{
	return b3ClampFloat( x, -m, m );
}

// ---------------------------------------------------------------------------
// Grass surface (docs/ROAD-SURFACE.md §6) — off-corridor wheels ride the flat
// grass plane: reduced bristle friction plus a rolling drag, so going off
// costs time but is recoverable. Both effects are completely inert for
// on-corridor wheels (mu scale is exactly 1.0f, the drag branch is untaken),
// which keeps v1 replay hashes byte-identical.
// ---------------------------------------------------------------------------

#define SIM_GRASS_MU_SCALE 0.55f  // bristle-friction multiplier on grass
#define SIM_GRASS_DRAG_COEF 30.0f // N per (m/s) of in-plane patch velocity
								  // (~600 N per wheel at 20 m/s, linear in speed)

// Off-corridor terrain raycast (docs/TERRAIN.md §2, v2 tracks only): one
// straight-down world-space ray per off-corridor wheel per TICK against the
// landscape mesh (SIM_CAT_MESH — prop boxes excluded, you cannot drive on a
// tire stack). Origin SIM_TERRAIN_RAY_UP above the hardpoint (immune to a
// hardpoint momentarily below a slope after a hard landing), total length
// SIM_TERRAIN_RAY_LEN (reaches RAY_LEN − RAY_UP below the hardpoint — anything
// farther is far out of suspension reach anyway). Ray miss falls back to the
// flat ground_y grass plane from the road query — exactly the pre-raycast
// surface. One ray per tick, not per substep: the ray origin is the hardpoint,
// which is FROZEN within the tick (the chassis integrates at tick level), so a
// per-substep ray against static geometry would return the identical hit 4x —
// the tick-level hit plane is exact, not an approximation.
#define SIM_TERRAIN_RAY_UP 5.0f
#define SIM_TERRAIN_RAY_LEN 10.0f

void vehicle_apply_impact( Vehicle* v, float approachSpeed, b3Vec3 localPoint )
{
	if ( approachSpeed <= SIM_DAMAGE_THRESHOLD )
	{
		return; // below threshold: pristine, no damage (clean-lap guarantee)
	}
	float sev = ( approachSpeed - SIM_DAMAGE_THRESHOLD ) * SIM_DAMAGE_SEV_PER_MS;
	sev = b3ClampFloat( sev, 0.0f, 1.0f );

	// Overall damage accrues from every impact (HUD + aero loss).
	v->damage_overall = b3ClampFloat( v->damage_overall + sev * SIM_DAMAGE_K_OVERALL, 0.0f, 1.0f );

	// Region: front (+Z) vs rear (-Z); side sign from local X (+ = right).
	int front = localPoint.z >= 0.0f;
	float side = ( localPoint.x >= 0.0f ) ? 1.0f : -1.0f;

	if ( front )
	{
		// Front-end hits bend the steering and the front toe.
		v->damage_steer = b3ClampFloat( v->damage_steer + sev * SIM_DAMAGE_K_STEER, 0.0f, 1.0f );
		v->damage_toe_front = sClampMag( v->damage_toe_front + side * sev * SIM_DAMAGE_K_TOE, SIM_DAMAGE_TOE_CLAMP );
	}
	else
	{
		// Rear hits bend the rear toe.
		v->damage_toe_rear = sClampMag( v->damage_toe_rear + side * sev * SIM_DAMAGE_K_TOE, SIM_DAMAGE_TOE_CLAMP );
	}
}

void vehicle_accumulate_damage( b3WorldId world, Vehicle* v )
{
	b3ContactEvents ev = b3World_GetContactEvents( world );
	for ( int i = 0; i < ev.hitCount; ++i )
	{
		const b3ContactHitEvent* h = &ev.hitEvents[i];
		int is_chassis =
			B3_ID_EQUALS( h->shapeIdA, v->chassis_shape ) || B3_ID_EQUALS( h->shapeIdB, v->chassis_shape );
		if ( !is_chassis )
		{
			continue;
		}
		// Hit point into the chassis-local frame for region routing.
		b3Vec3 local = b3Body_GetLocalPoint( v->chassis, h->point );
		vehicle_apply_impact( v, h->approachSpeed, local );
	}
}

float vehicle_effect_aero_front( const Vehicle* v )
{
	return kTuning.aero_cl_front * ( 1.0f - SIM_DAMAGE_AERO_LOSS * v->damage_overall );
}

float vehicle_effect_max_steer( const Vehicle* v )
{
	return kTuning.max_steer * ( 1.0f - SIM_DAMAGE_STEER_LOSS * v->damage_steer );
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
//
// vehicle_brush_patch_mu additionally scales the bristle friction by mu_scale
// (grass, docs/ROAD-SURFACE.md §6). mu_scale = 1.0f multiplies the thermal
// mu_s by exactly 1.0f — an IEEE identity — so the asphalt path is
// bit-identical to the pre-grass kernel (v1 replay hashes unchanged).
void vehicle_brush_patch_mu( float sigma_x, float sigma_y, float fz, float t_surf, float mu_scale, float* out_fx,
							 float* out_fy, float* out_trail )
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

	// Thermal grip factor (docs/TIRE-MODEL.md §2), then the surface scale
	// (grass ~0.55, asphalt exactly 1.0f — IEEE identity, bit-identical).
	float dT = t_surf - t->thermal_t_opt;
	float mu_t = 1.0f - t->thermal_k_t * dT * dT;
	mu_t = b3ClampFloat( mu_t, 0.88f, 1.00f );
	// Load sensitivity (docs/DRIVETRAIN.md §5): friction falls with load above
	// the reference Fz0, so lateral load transfer costs an axle net grip —
	// roll stiffness becomes a real balance lever. At Fz == Fz0 the factor is
	// exactly 1.0f (IEEE identity), so reference-load sweeps are unchanged.
	float mu_load = 1.0f - t->k_load * ( ( fz - t->brush_fz0 ) / t->brush_fz0 );
	mu_load = b3ClampFloat( mu_load, t->load_clamp_lo, t->load_clamp_hi );
	float mu_s = t->brush_mu_s * mu_t * mu_load * mu_scale;
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

// Asphalt-only wrapper (the historical signature, kept for the test sweeps).
void vehicle_brush_patch( float sigma_x, float sigma_y, float fz, float t_surf, float* out_fx, float* out_fy,
						  float* out_trail )
{
	vehicle_brush_patch_mu( sigma_x, sigma_y, fz, t_surf, 1.0f, out_fx, out_fy, out_trail );
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
void vehicle_suspension_step( WheelRuntime* w, float hit_dist, float hit_dist_dot, float g_along, float f_arb,
							  float dt, float* out_tire_force, float* out_susp_force )
{
	const VehicleTuning* t = &kTuning;

	// Suspension spring/damper between chassis and unsprung mass, plus the
	// caller-computed anti-roll-bar share (docs/DRIVETRAIN.md §5). Travel
	// clamps keep c in [0, max_travel]; the damper may pull (rebound), so the
	// strut force is clamped symmetrically, not at zero.
	float c = t->rest_length - w->travel;
	float c_dot = -w->travel_vel;
	float damper = c_dot >= 0.0f ? t->damper_bump : t->damper_rebound;
	float f_susp = t->spring_k * c + damper * c_dot + f_arb;
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

// rad/s <-> rpm
#define W_TO_RPM 9.5492966f

// Shift timing (400 Hz ticks, docs/DRIVETRAIN.md §1): upshift ignition cut,
// downshift engagement delay, downshift driveline-shock duration.
#define SIM_SHIFT_UP_TICKS 28	// 70 ms
#define SIM_SHIFT_DOWN_TICKS 48 // 120 ms
#define SIM_SHIFT_SHOCK_TICKS 8 // 20 ms
// Reverse engages from 1st only below this forward speed (m/s).
#define SIM_REVERSE_MAX_SPEED 1.0f

float vehicle_gear_ratio( int gear )
{
	if ( gear < 0 || gear > 6 )
	{
		return 0.0f;
	}
	return kTuning.gear_ratios[gear];
}

// Viscous LSD (docs/DRIVETRAIN.md §3): base 50/50 split of the transmitted
// axle torque plus k_lsd * (omega_l - omega_r) moved FROM the faster wheel TO
// the slower one, clamped to +/- lsd_clamp_frac of |t_axle|. Power and coast
// both flow through the same coupling; with no transmitted torque (shift cut)
// the clamp collapses to zero and the diff is open.
void vehicle_lsd_split( float t_axle, float omega_l, float omega_r, float* out_t_left, float* out_t_right )
{
	const VehicleTuning* t = &kTuning;
	float transfer = t->k_lsd * ( omega_l - omega_r );
	float cap = t->lsd_clamp_frac * b3AbsFloat( t_axle );
	transfer = b3ClampFloat( transfer, -cap, cap );
	*out_t_left = 0.5f * t_axle - transfer;
	*out_t_right = 0.5f * t_axle + transfer;
}

// Engine braking (docs/DRIVETRAIN.md §2): closed throttle => negative crank
// torque, linear from eb_torque_idle at idle to eb_torque_red at the limiter
// (clamped there for the over-rev bounce), fading to zero by 10% throttle.
float vehicle_engine_brake_torque( float engine_omega, float throttle )
{
	const VehicleTuning* t = &kTuning;
	float fade = 1.0f - throttle / t->eb_fade_throttle;
	if ( fade <= 0.0f )
	{
		return 0.0f;
	}
	if ( fade > 1.0f )
	{
		fade = 1.0f;
	}
	float u = ( engine_omega - t->idle_omega ) / ( t->limiter_omega - t->idle_omega );
	u = b3ClampFloat( u, 0.0f, 1.0f );
	return ( t->eb_torque_idle + u * ( t->eb_torque_red - t->eb_torque_idle ) ) * fade;
}

// Slip relaxation (docs/DRIVETRAIN.md §5): first-order lag on the slip vector
// fed to the brush patch, tau = L_relax/|v| clamped to tau_max at low speed.
// One 1600 Hz sub-step; plain float math, deterministic.
void vehicle_slip_relax( WheelRuntime* w, float target_x, float target_y, float speed, float dt )
{
	const VehicleTuning* t = &kTuning;
	float v = speed > 0.001f ? speed : 0.001f;
	float tau = t->relax_len / v;
	if ( tau > t->relax_tau_max )
	{
		tau = t->relax_tau_max;
	}
	float alpha = dt / tau;
	if ( alpha > 1.0f )
	{
		alpha = 1.0f;
	}
	w->sigma_x_rel += alpha * ( target_x - w->sigma_x_rel );
	w->sigma_y_rel += alpha * ( target_y - w->sigma_y_rel );
}

// ---------------------------------------------------------------------------
// Gearbox state machine (docs/DRIVETRAIN.md §1) — one call per 400 Hz tick.
// Edge-detects shift bits (rising edge = one request; both bits rising in the
// same tick: up wins), runs the shift timers, and performs the engagement:
// upshift engages after the 28-tick ignition cut (engine re-couples to the
// wheel-matched speed on the next sub-step); downshift engages after the
// 48-tick delay with the engine snapped to the wheel-matched speed (auto-blip
// illusion) plus a brief driveline shock torque sized by the snap delta.
// ---------------------------------------------------------------------------
static void sShiftMachine( Vehicle* v, uint32_t flags, float v_fwd )
{
	const VehicleTuning* t = &kTuning;
	uint32_t rising = flags & ~v->prev_flags;
	v->prev_flags = flags;

	// Downshift-shock decay runs independently of the shift timer.
	if ( v->shock_ticks > 0 )
	{
		v->shock_ticks--;
	}

	if ( v->shift_ticks > 0 )
	{
		// Shift in progress: requests are dropped (spec §1).
		v->shift_ticks--;
		if ( v->shift_ticks == 0 )
		{
			v->gear = v->shift_target;
			if ( v->shift_is_down )
			{
				float w_mean = 0.5f * ( v->wheels[2].omega + v->wheels[3].omega );
				float w_matched = w_mean * t->gear_ratios[v->gear];
				w_matched = b3ClampFloat( w_matched, t->idle_omega, t->overrev_omega );
				float delta = w_matched - v->engine_omega;
				if ( delta > 0.0f )
				{
					float shock = t->shock_k * delta;
					v->shock_torque = shock > t->shock_max ? t->shock_max : shock;
					v->shock_ticks = SIM_SHIFT_SHOCK_TICKS;
				}
				v->engine_omega = w_matched; // auto-blip illusion
			}
		}
		return;
	}

	if ( rising & SIM_FLAG_SHIFT_UP )
	{
		if ( v->gear == 0 )
		{
			// R -> 1: forward-safe at any speed (spec §1 reverse rule).
			v->shift_target = 1;
			v->shift_ticks = SIM_SHIFT_UP_TICKS;
			v->shift_is_down = 0;
		}
		else if ( v->gear < 6 )
		{
			v->shift_target = v->gear + 1;
			v->shift_ticks = SIM_SHIFT_UP_TICKS;
			v->shift_is_down = 0;
		}
	}
	else if ( rising & SIM_FLAG_SHIFT_DOWN )
	{
		if ( v->gear == 1 )
		{
			// 1 -> R engages only when |v| < 1 m/s (spec §1 reverse rule).
			if ( b3AbsFloat( v_fwd ) < SIM_REVERSE_MAX_SPEED )
			{
				v->shift_target = 0;
				v->shift_ticks = SIM_SHIFT_DOWN_TICKS;
				v->shift_is_down = 1;
			}
		}
		else if ( v->gear >= 2 )
		{
			// Downshift protection: refuse any downshift that would put the
			// engine above 7800 rpm at the current wheel speed.
			float w_mean = 0.5f * ( v->wheels[2].omega + v->wheels[3].omega );
			float w_pred = w_mean * t->gear_ratios[v->gear - 1];
			if ( w_pred <= t->overrev_omega )
			{
				v->shift_target = v->gear - 1;
				v->shift_ticks = SIM_SHIFT_DOWN_TICKS;
				v->shift_is_down = 1;
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Drivetrain sub-step (docs/DRIVETRAIN.md §1-3): one engine state coupled to
// the mean rear wheel speed through the engaged ratio. Computes the per-rear-
// wheel drive torques (LSD split + downshift shock) and the effective rear
// wheel inertia (reflected engine inertia when rigidly coupled). Advances
// v->engine_omega (slaved when engaged; free integration under engine braking
// during shift cuts).
// ---------------------------------------------------------------------------
static void sDrivetrainSubstep( Vehicle* v, int handbrake, float throttle, float dt, float* out_t_rl,
								float* out_t_rr, float* out_i_rear )
{
	const VehicleTuning* t = &kTuning;
	float w_mean = 0.5f * ( v->wheels[2].omega + v->wheels[3].omega );
	float ratio = t->gear_ratios[v->gear];
	float t_axle = 0.0f;
	float i_rear = t->wheel_inertia;

	if ( v->shift_ticks > 0 )
	{
		// Shift cut: no torque path (upshift ignition cut / downshift neutral
		// delay). The engine integrates freely under its own inertia + engine
		// braking (combustion is cut, so the closed-throttle curve applies).
		float w_e = v->engine_omega + ( vehicle_engine_brake_torque( v->engine_omega, 0.0f ) / t->engine_inertia ) * dt;
		v->engine_omega = w_e > t->idle_omega ? w_e : t->idle_omega;
	}
	else if ( handbrake )
	{
		// Handbrake overrides the drive path (rears are hard-locked in the
		// spin step); the engine sits at the throttle flare rpm, declutched.
		float w_hold = t->idle_omega + throttle * ( t->clutch_hold_omega - t->idle_omega );
		v->engine_omega = w_hold;
	}
	else
	{
		float w_c = w_mean * ratio; // crank speed if rigidly coupled
		if ( w_c >= t->clutch_omega )
		{
			// Rigid coupling: engine slaved to the wheels. Limiter = hard fuel
			// cut above 7500 rpm; engine braking applies through the same path
			// (the "natural bounce"). Reflected crank inertia stiffens the
			// rear wheels' spin dynamics by I_e * ratio^2 (split across two).
			v->engine_omega = w_c;
			float t_comb = 0.0f;
			if ( w_c <= t->limiter_omega )
			{
				t_comb = throttle * sEngineTorque( w_c * W_TO_RPM );
			}
			float t_eb = vehicle_engine_brake_torque( w_c, throttle );
			t_axle = ( t_comb + t_eb ) * ratio * t->driveline_eff;
			i_rear = t->wheel_inertia + 0.5f * t->engine_inertia * ratio * ratio;
		}
		else
		{
			// Implicit auto-clutch (spec §1): below the coupled speed that
			// would drag the engine under 1100 rpm the clutch slips. The
			// engine flares to a throttle-scaled launch rpm (never below
			// idle — it cannot stall); transmitted torque scales with the
			// rpm headroom of the coupled speed toward lockup, floored so a
			// standing start pulls away, with a small creep throttle so the
			// car creeps at idle. No engine braking through a slipping clutch.
			float w_hold = t->idle_omega + throttle * ( t->clutch_hold_omega - t->idle_omega );
			float w_e = w_c > w_hold ? w_c : w_hold;
			v->engine_omega = w_e;
			float eff_th = throttle > t->creep_throttle ? throttle : t->creep_throttle;
			float frac = b3ClampFloat( w_c / t->clutch_omega, 0.0f, 1.0f );
			float factor = t->clutch_min + ( 1.0f - t->clutch_min ) * frac;
			t_axle = eff_th * sEngineTorque( w_e * W_TO_RPM ) * factor * ratio * t->driveline_eff;
		}
	}

	vehicle_lsd_split( t_axle, v->wheels[2].omega, v->wheels[3].omega, out_t_rl, out_t_rr );

	// Downshift driveline shock (spec §1): brief deterministic retarding
	// torque on the rear axle right after the rev-match snap.
	if ( v->shock_ticks > 0 )
	{
		float sgn = w_mean >= 0.0f ? 1.0f : -1.0f;
		*out_t_rl -= 0.5f * v->shock_torque * sgn;
		*out_t_rr -= 0.5f * v->shock_torque * sgn;
	}

	*out_i_rear = i_rear;
}

// One sub-step of wheel spin integration: drive torque (rears, from the
// drivetrain), tire reaction torque, brake clamp that cannot reverse the
// wheel within the step, handbrake hard-lock on the rears.
static void sWheelSpinIntegrate( WheelRuntime* w, float inertia, float drive, float reaction_torque,
								 float brake_trq, int hard_lock, float dt )
{
	float omega_new = w->omega + ( ( drive + reaction_torque ) / inertia ) * dt;
	float brake_dw = ( brake_trq / inertia ) * dt;
	if ( omega_new > 0.0f )
	{
		omega_new = omega_new > brake_dw ? omega_new - brake_dw : 0.0f;
	}
	else if ( omega_new < 0.0f )
	{
		omega_new = omega_new < -brake_dw ? omega_new + brake_dw : 0.0f;
	}
	if ( hard_lock )
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

// Per-wheel per-tick context: contact, patch basis and slip inputs are
// per-tick constants (the chassis pose/velocity integrate at tick level).
// Filled by phase A of vehicle_update; consumed by the 1600 Hz phase-B loop,
// which couples the rear wheels through the drivetrain and left/right
// through the anti-roll bars.
typedef struct WheelTick
{
	b3Vec3 origin; // hardpoint, world
	int contact;
	int grass;
	float hit_dist;
	float hit_dist_dot;
	b3Vec3 contact_point;
	b3Vec3 wheel_fwd;
	b3Vec3 wheel_side;
	float v_long;
	float v_lat;
	float denom;
	float sigma_y; // lateral slip target incl. camber thrust
	float mirror;
	// Sub-step accumulators / last values
	float fx_sum;
	float fy_sum;
	float rack_sum;
	float fsusp_sum;
	float fz; // last sub-step's tire spring force
} WheelTick;

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

	// Gearbox state machine (docs/DRIVETRAIN.md §1): edge-detected shift bits,
	// shift timers, engagement. Once per 400 Hz tick, BEFORE the wheel
	// pipeline so cut timing is exact in ticks. Forward speed gates reverse.
	sShiftMachine( v, flags, b3Dot( chassis_vel, fwd ) );

	// Steering: linear map, front axle only. Front-end crash damage shrinks the
	// lock (docs/SOFTBODY.md Phase 2). At damage_steer = 0 the factor is exactly
	// 1.0, so a clean lap is bit-identical to an undamaged car.
	float damaged_max_steer = t->max_steer * ( 1.0f - SIM_DAMAGE_STEER_LOSS * v->damage_steer );
	float steer_angle = steer * damaged_max_steer;

	// FFB: front-axle rack torque accumulated over the wheel loop.
	float rack = 0.0f;

	b3QueryFilter rayFilter = b3DefaultQueryFilter();
	rayFilter.categoryBits = SIM_CAT_CHASSIS;
	rayFilter.maskBits = SIM_CAT_TERRAIN;

	// Off-corridor terrain ray (docs/TERRAIN.md §2): landscape mesh ONLY —
	// prop boxes are not wheel-ridable. The legacy fallback ray above keeps
	// masking SIM_CAT_TERRAIN (mesh + props), bit-identical to before.
	b3QueryFilter meshRayFilter = b3DefaultQueryFilter();
	meshRayFilter.categoryBits = SIM_CAT_CHASSIS;
	meshRayFilter.maskBits = SIM_CAT_MESH;

	const float ray_len = t->rest_length + t->wheel_radius;
	const float sub_dt = SIM_DT / (float)SIM_TIRE_SUBSTEPS;

	// Gravity component along strut-down (-up): the same for all four corners.
	const float g_along = 9.81f * up.y;

	WheelTick wt[SIM_WHEEL_COUNT];

	// --- Phase A: per-wheel kinematics, contact and slip inputs -------------
	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		WheelRuntime* w = &v->wheels[i];
		WheelTick* c = &wt[i];
		int front = sIsFront( i );
		*c = ( WheelTick ){ 0 };

		// Kinematic camber/toe from the CURRENT spring compression
		// (docs/SUSPENSION.md §2). Travel clamps keep it in [0, max_travel].
		float camber_conv, toe_add;
		vehicle_wheel_kinematics( i, t->rest_length - w->travel, &camber_conv, &toe_add );
		// Crash toe damage bends the axle (docs/SOFTBODY.md Phase 2); 0 when
		// undamaged, so a clean lap adds exactly 0.0f (bit-identical).
		float damage_toe = front ? v->damage_toe_front : v->damage_toe_rear;
		w->steer_angle = ( front ? steer_angle : 0.0f ) + toe_add + damage_toe;

		b3Vec3 hp_local = sVehicleHardpoint( i );
		b3Pos origin = b3Body_GetWorldPoint( chassis, hp_local );
		c->origin = origin;

		// --- Wheel contact (docs/ROAD-SURFACE.md §1): analytic road query at
		// the hardpoint projected down; compression from the analytic height
		// along the suspension axis (-chassis up); contact normal from the
		// query. Mesh raycast only as the off-domain fallback — the chassis
		// body still collides with the mesh, only the WHEELS go analytic. ---
		int have_contact = 0;
		int grass = 0; // contact is the off-corridor grass plane (§6)
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

			// On v2 tracks the query is total (docs/ROAD-SURFACE.md §6):
			// corridor answers are bit-identical to before, off-corridor is
			// kind GRASS — the wheel then rides the real landscape mesh via
			// the terrain raycast below (docs/TERRAIN.md §2), with the flat
			// ground_y plane in rq as the ray-miss fallback. On v1 tracks
			// off-corridor queries still report !on_road with kind ASPHALT,
			// so this condition — and everything downstream — reduces exactly
			// to the legacy `if (rq.on_road)` control flow (bit-identical
			// replays). A surface facing away from the ray (rolled car) still
			// ends up airborne, exactly as before; chassis mesh collision
			// handles it.
			if ( rq.on_road || rq.kind == ROAD_KIND_GRASS )
			{
				need_mesh_fallback = 0;
				b3Vec3 surf_point = rq.point;
				b3Vec3 surf_normal = rq.normal;

				if ( rq.kind == ROAD_KIND_GRASS )
				{
					// Off-corridor on a v2 track (GRASS never occurs on v1):
					// ride the REAL landscape (docs/TERRAIN.md §2) — cast
					// straight down from above the hardpoint against the
					// static mesh only (SIM_CAT_MESH; props excluded) and use
					// the hit's tangent plane as the wheel surface. The plane
					// is a per-tick constant reused across the 4 substeps,
					// exactly like the analytic corridor surface. A miss
					// keeps the flat ground_y plane already in rq — the
					// pre-raycast behavior, bit-identical.
					b3Pos ray_origin = origin;
					ray_origin.y += SIM_TERRAIN_RAY_UP;
					b3Vec3 ray_translation = { 0.0f, -SIM_TERRAIN_RAY_LEN, 0.0f };
					b3RayResult tray = b3World_CastRayClosest( world, ray_origin, ray_translation, meshRayFilter );
					if ( tray.hit )
					{
						surf_point = tray.point;
						surf_normal = tray.normal; // unit, up-facing (the mesh
												   // raycast is back-face culled)
					}
				}

				// Suspension ray x(d) = origin - d·up against the local
				// tangent plane (point surf_point, normal surf_normal).
				float facing = b3Dot( up, surf_normal );
				if ( facing > 0.2f ) // surface must face the ray
				{
					float d = b3Dot( b3Sub( origin, surf_point ), surf_normal ) / facing;
					if ( d <= ray_len )
					{
						have_contact = 1;
						hit_dist = d > 0.0f ? d : 0.0f; // below surface → full compression
						contact_point = b3MulAdd( origin, -hit_dist, up );
						contact_normal = surf_normal;
						grass = ( rq.kind == ROAD_KIND_GRASS );
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
			// Airborne: no tire spring; phase B still works the strut toward
			// full droop, cools the tire and spins the free wheel under
			// drivetrain/brake torque. hit_dist beyond reach = no tire force.
			c->contact = 0;
			c->hit_dist = ray_len + 1.0f;
			c->hit_dist_dot = 0.0f;
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
		w->slip_angle = b3Atan2( -v_lat, denom );

		c->contact = 1;
		c->grass = grass;
		c->hit_dist = hit_dist;
		c->hit_dist_dot = hit_dist_dot;
		c->contact_point = contact_point;
		c->wheel_fwd = wheel_fwd;
		c->wheel_side = wheel_side;
		c->v_long = v_long;
		c->v_lat = v_lat;
		c->denom = denom;
		c->sigma_y = -v_lat / denom + sigma_camber;
		c->mirror = mirror;
	}

	// --- Phase B: 1600 Hz sub-stepped pipeline (docs/ROAD-SURFACE.md §3 +
	// SUSPENSION.md §1 + DRIVETRAIN.md §1-3, §5): per sub-step — anti-roll
	// bars from the current left/right compressions, unsprung strut travel,
	// slip relaxation, brush patch (with load-sensitive friction), thermal,
	// then ONE drivetrain evaluation coupling the rear axle (engine + LSD)
	// and wheel spin integration for all four corners. Fz for the brush
	// model is the tire spring force from the quarter-car, per sub-step.
	// Grass wheels see reduced bristle friction (§6): asphalt multiplies by
	// exactly 1.0f — an IEEE identity. ---
	for ( int k = 0; k < SIM_TIRE_SUBSTEPS; ++k )
	{
		// Anti-roll bars (docs/DRIVETRAIN.md §5): F = k_arb * (c_this -
		// c_other) per axle, equal and opposite left/right — pure roll-moment
		// redistribution, made meaningful by tire load sensitivity.
		float dc_front = v->wheels[1].travel - v->wheels[0].travel; // = c_FL - c_FR
		float dc_rear = v->wheels[3].travel - v->wheels[2].travel;	// = c_RL - c_RR
		float f_arb[SIM_WHEEL_COUNT];
		f_arb[0] = t->arb_front * dc_front;
		f_arb[1] = -f_arb[0];
		f_arb[2] = t->arb_rear * dc_rear;
		f_arb[3] = -f_arb[2];

		float fx_k[SIM_WHEEL_COUNT]; // this sub-step's patch fx per wheel

		for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
		{
			WheelRuntime* w = &v->wheels[i];
			WheelTick* c = &wt[i];
			int front = sIsFront( i );

			float fz, f_susp;
			vehicle_suspension_step( w, c->hit_dist, c->hit_dist_dot, g_along, f_arb[i], sub_dt, &fz, &f_susp );
			c->fsusp_sum += f_susp;
			c->fz = fz;

			fx_k[i] = 0.0f;
			if ( c->contact )
			{
				float slip_ratio = ( w->omega * t->wheel_radius - c->v_long ) / c->denom;
				slip_ratio = b3ClampFloat( slip_ratio, -4.0f, 4.0f );
				w->slip_ratio = slip_ratio; // last sub-step's RAW value is exported

				// Slip relaxation (docs/DRIVETRAIN.md §5), then the brush
				// patch on the RELAXED slip vector: progressive force
				// build-up over ~L/v instead of instantaneous response.
				vehicle_slip_relax( w, slip_ratio, c->sigma_y, c->denom, sub_dt );

				const float mu_scale = c->grass ? SIM_GRASS_MU_SCALE : 1.0f;
				float fx, fy, trail;
				vehicle_brush_patch_mu( w->sigma_x_rel, w->sigma_y_rel, fz, w->t_surf, mu_scale, &fx, &fy, &trail );
				c->fx_sum += fx;
				c->fy_sum += fy;
				fx_k[i] = fx;

				// FFB rack torque (front axle only, output-only): lateral
				// force behind the steering axis by the EMERGENT pneumatic
				// trail plus mechanical caster (docs/TIRE-MODEL.md §3). The
				// scrub moment arm mirrors about the kingpin, so the fx term
				// carries the per-wheel mirror sign: symmetric braking fx on
				// both fronts cancels at the rack instead of doubling.
				if ( front )
				{
					c->rack_sum += fy * ( trail + t->ffb_caster_trail ) + c->mirror * fx * t->ffb_scrub_radius;
				}

				// Thermal: friction power from the slip velocity at the
				// patch, with track conduction while the tread presses the
				// road (ROAD-SURFACE §2).
				float v_slip_x = w->omega * t->wheel_radius - c->v_long;
				float p_fric = b3AbsFloat( fx * v_slip_x ) + b3AbsFloat( fy * c->v_lat );
				vehicle_tire_thermal( w, p_fric, chassis_speed, fz > 0.0f, sub_dt );
			}
			else
			{
				// Airborne: slips relax toward zero, the tire cools in the
				// airflow (no friction power, no track conduction).
				vehicle_slip_relax( w, 0.0f, 0.0f, 0.0f, sub_dt );
				vehicle_tire_thermal( w, 0.0f, chassis_speed, 0, sub_dt );
			}
		}

		// Drivetrain (docs/DRIVETRAIN.md §1-3): one engine coupled to the
		// mean rear wheel speed; LSD splits the axle torque; then integrate
		// wheel spin for all four corners (reflected crank inertia on the
		// rears while rigidly coupled).
		float t_rl, t_rr, i_rear;
		sDrivetrainSubstep( v, handbrake, throttle, sub_dt, &t_rl, &t_rr, &i_rear );

		for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
		{
			WheelRuntime* w = &v->wheels[i];
			int front = sIsFront( i );
			float drive = front ? 0.0f : ( i == 2 ? t_rl : t_rr );
			float inertia = front ? t->wheel_inertia : i_rear;
			float brake_cap = front ? t->brake_torque_front : t->brake_torque_rear;
			float brake_trq = brake * brake_cap;
			if ( handbrake && !front )
			{
				brake_trq += t->handbrake_torque;
			}
			int hard_lock = handbrake && !front;
			sWheelSpinIntegrate( w, inertia, drive, -fx_k[i] * t->wheel_radius, brake_trq, hard_lock, sub_dt );
		}
	}

	// --- Per-wheel epilogue: exports + chassis forces (sub-step MEANS applied
	// once per tick: Box3D integrates F·SIM_DT, so the mean preserves the
	// summed sub-step impulses) ---
	const float inv_n = 1.0f / (float)SIM_TIRE_SUBSTEPS;
	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		WheelRuntime* w = &v->wheels[i];
		WheelTick* c = &wt[i];
		int front = sIsFront( i );
		sWrapSpin( w );

		w->compression = t->rest_length - w->travel;
		w->wheel_center = b3MulAdd( c->origin, -w->travel, up );

		// Strut reaction on the chassis at the hardpoint, along the strut
		// (applies airborne too: droop extension damping).
		b3Body_ApplyForce( chassis, b3MulSV( c->fsusp_sum * inv_n, up ), c->origin, true );

		if ( !c->contact )
		{
			w->in_contact = 0;
			w->load = 0.0f;
			w->slip_ratio = 0.0f;
			w->slip_angle = 0.0f;
			continue;
		}

		w->load = c->fz; // last sub-step's tire spring force

		// Tire in-plane force at the patch (the unsprung mass has no in-plane
		// DOF, so the patch force transfers rigidly to the chassis).
		b3Vec3 tire_force =
			b3Add( b3MulSV( c->fx_sum * inv_n, c->wheel_fwd ), b3MulSV( c->fy_sum * inv_n, c->wheel_side ) );
		b3Body_ApplyForce( chassis, tire_force, c->contact_point, true );

		// Grass rolling drag (§6): linear in the in-plane patch velocity,
		// ~600 N per wheel at 20 m/s — grass is drivable but slow. This
		// branch is untaken on the corridor (completely inert on asphalt).
		if ( c->grass )
		{
			b3Vec3 grass_drag = b3Add( b3MulSV( -SIM_GRASS_DRAG_COEF * c->v_long, c->wheel_fwd ),
									   b3MulSV( -SIM_GRASS_DRAG_COEF * c->v_lat, c->wheel_side ) );
			b3Body_ApplyForce( chassis, grass_drag, c->contact_point, true );
		}

		if ( front )
		{
			rack += c->rack_sum * inv_n;
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
	// Crash damage sheds downforce (docs/SOFTBODY.md Phase 2): a wrecked wing/
	// floor makes less grip. At damage_overall = 0 the factor is exactly 1.0,
	// keeping a clean lap bit-identical to the undamaged car.
	{
		float v2 = chassis_speed * chassis_speed;
		float aero_factor = 1.0f - SIM_DAMAGE_AERO_LOSS * v->damage_overall;
		float cl_front = t->aero_cl_front * aero_factor;
		float cl_rear = t->aero_cl_rear * aero_factor;
		b3Pos front_ax = b3Body_GetWorldPoint( chassis, ( b3Vec3 ){ 0.0f, t->hardpoint_y, t->wheelbase_half } );
		b3Pos rear_ax = b3Body_GetWorldPoint( chassis, ( b3Vec3 ){ 0.0f, t->hardpoint_y, -t->wheelbase_half } );
		b3Body_ApplyForce( chassis, b3MulSV( -cl_front * v2, up ), front_ax, true );
		b3Body_ApplyForce( chassis, b3MulSV( -cl_rear * v2, up ), rear_ax, true );
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

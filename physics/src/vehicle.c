// vehicle.c — raycast-suspension vehicle with discretized brush-model tires
// (docs/TIRE-MODEL.md, replacing the Pacejka MF of ADR-007).
//
// Chassis: one dynamic Box3D rigid body (box hull ~1.9 x 1.1 x 4.4 m, 1350 kg,
// explicit inertia). Wheels are NOT rigid bodies: each is a fixed hardpoint
// raycast down the chassis-local -Y axis, a spring/damper along chassis up,
// and brush contact-patch tire forces applied at the contact point. The patch
// is N=16 bristle elements under a parabolic pressure distribution; grip,
// combined-slip budget sharing and the pneumatic-trail collapse all emerge
// from the per-element adhesion/sliding split. A two-node thermal layer
// (T_surf/T_core per tire) scales friction with surface temperature.
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
	// Chassis
	float half_extent_x; // half width (m)
	float half_extent_y; // half height (m)
	float half_extent_z; // half length (m)
	float mass;			 // kg
	float com_drop;		 // center of mass lowered by this much (m)
	float inertia_pitch; // about local X (kg m^2)
	float inertia_yaw;	 // about local Y
	float inertia_roll;	 // about local Z

	// Aerodynamics: F = -drag_coef * v * |v|, applied at center of mass
	float drag_coef; // 0.5 * rho * Cd * A

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

	// Tire thermal layer, two nodes per tire (docs/TIRE-MODEL.md §2)
	float thermal_t_amb;  // ambient temperature (deg C)
	float thermal_t_opt;  // optimal surface temperature (deg C)
	float thermal_k_t;	  // grip falloff: mu_T = 1 - k_t * (T_surf - T_opt)^2
	float thermal_c_surf; // surface-layer heat capacity (J/K)
	float thermal_h_conv; // convective cooling rate (1/s, scaled by speed)
	float thermal_h_int;  // surface->core exchange as seen by the surface (1/s)
	float thermal_h_int2; // surface->core exchange as seen by the core (1/s)

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
	.com_drop = 0.15f,
	.inertia_pitch = 2400.0f,
	.inertia_yaw = 2600.0f,
	.inertia_roll = 550.0f,

	.drag_coef = 0.42f,

	.hardpoint_y = -0.25f,
	.track_half = 0.80f,
	.wheelbase_half = 1.30f,
	.rest_length = 0.35f,
	.max_travel = 0.25f,
	.wheel_radius = 0.33f,
	.spring_k = 60000.0f,
	.damper_bump = 4500.0f,
	.damper_rebound = 5200.0f,

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
	.thermal_k_t = 4.1666668e-5f, // 0.15 / 60^2: 25 C and 145 C both give 0.85
	.thermal_c_surf = 1500.0f,
	.thermal_h_conv = 0.008f,
	.thermal_h_int = 0.006f,
	.thermal_h_int2 = 0.002f,

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

	// Explicit, realistic mass properties (a uniform box hull of these
	// dimensions would be far too top-heavy in roll).
	b3MassData md;
	md.mass = t->mass;
	md.center = ( b3Vec3 ){ 0.0f, -t->com_drop, 0.0f };
	md.inertia = ( b3Matrix3 ){
		.cx = { t->inertia_pitch, 0.0f, 0.0f },
		.cy = { 0.0f, t->inertia_yaw, 0.0f },
		.cz = { 0.0f, 0.0f, t->inertia_roll },
	};
	b3Body_SetMassData( v->chassis, md );

	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		v->wheels[i] = ( WheelRuntime ){ 0 };
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
	mu_t = b3ClampFloat( mu_t, 0.80f, 1.00f );
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

// Advance the two-node thermal state of one tire by SIM_DT
// (docs/TIRE-MODEL.md §2). power = friction power into the surface (W),
// speed = vehicle speed (m/s) for convective cooling. Plain float Euler
// integration — deterministic by construction, never hashed directly.
static void sTireThermal( WheelRuntime* w, float power, float speed )
{
	const VehicleTuning* t = &kTuning;
	float d_surf = power / t->thermal_c_surf - t->thermal_h_conv * ( 1.0f + 0.05f * speed ) * ( w->t_surf - t->thermal_t_amb ) -
				   t->thermal_h_int * ( w->t_surf - w->t_core );
	float d_core = t->thermal_h_int2 * ( w->t_surf - w->t_core );
	w->t_surf += d_surf * SIM_DT;
	w->t_core += d_core * SIM_DT;
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

// ---------------------------------------------------------------------------
// Per-tick update
// ---------------------------------------------------------------------------

void vehicle_update( b3WorldId world, Vehicle* v, float steer, float throttle, float brake, uint32_t flags )
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

	float ray_len = t->rest_length + t->wheel_radius;

	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		WheelRuntime* w = &v->wheels[i];
		w->steer_angle = sIsFront( i ) ? steer_angle : 0.0f;

		// --- Suspension raycast: from hardpoint, down chassis-local -Y ---
		b3Vec3 hp_local = sVehicleHardpoint( i );
		b3Pos origin = b3Body_GetWorldPoint( chassis, hp_local );
		b3Vec3 translation = b3MulSV( -ray_len, up );

		b3RayResult ray = b3World_CastRayClosest( world, origin, translation, rayFilter );

		float prev_compression = w->compression;

		if ( !ray.hit )
		{
			w->in_contact = 0;
			w->compression = 0.0f;
			w->prev_compression = prev_compression;
			w->load = 0.0f;
			w->slip_ratio = 0.0f;
			w->slip_angle = 0.0f;
			w->wheel_center = b3MulAdd( origin, -t->rest_length, up );

			// Airborne tire still cools (no friction power).
			sTireThermal( w, 0.0f, chassis_speed );

			// Free-spinning wheel: engine/brake still act on it.
			float drive = 0.0f;
			if ( !sIsFront( i ) && !handbrake )
			{
				float rpm = w->omega * t->gear_ratio * ( 60.0f / TWO_PI_F );
				drive = throttle * sEngineTorque( rpm < 1000.0f ? 1000.0f : rpm ) * t->gear_ratio * t->driveline_eff * 0.5f;
			}
			float brake_cap = sIsFront( i ) ? t->brake_torque_front : t->brake_torque_rear;
			float brake_trq = brake * brake_cap;
			if ( handbrake && !sIsFront( i ) )
			{
				brake_trq += t->handbrake_torque;
			}
			float resist = ( w->omega > 0.0f ? -brake_trq : ( w->omega < 0.0f ? brake_trq : 0.0f ) );
			float alpha = ( drive + resist ) / t->wheel_inertia;
			w->omega += alpha * SIM_DT;
			w->spin_angle += w->omega * SIM_DT;
			if ( w->spin_angle > 3.1415927f )
			{
				w->spin_angle -= TWO_PI_F;
			}
			else if ( w->spin_angle < -3.1415927f )
			{
				w->spin_angle += TWO_PI_F;
			}
			continue;
		}

		// Compression: distance the spring is shorter than rest length.
		float hit_dist = ray.fraction * ray_len;
		float compression = ray_len - hit_dist; // 0 (full droop) .. ray_len
		compression = b3ClampFloat( compression, 0.0f, t->max_travel );

		w->in_contact = 1;
		w->compression = compression;
		w->contact_point = ray.point;
		w->wheel_center = b3MulAdd( origin, -( hit_dist - t->wheel_radius ), up );

		// --- Spring + damper along chassis up ---
		float comp_vel = ( compression - prev_compression ) / SIM_DT;
		float damper_c = comp_vel >= 0.0f ? t->damper_bump : t->damper_rebound;
		float fz = t->spring_k * compression + damper_c * comp_vel;
		fz = b3ClampFloat( fz, 0.0f, t->max_load );
		w->load = fz;
		w->prev_compression = prev_compression;

		b3Vec3 susp_force = b3MulSV( fz, up );
		b3Body_ApplyForce( chassis, susp_force, ray.point, true );

		if ( fz <= 0.0f )
		{
			w->slip_ratio = 0.0f;
			w->slip_angle = 0.0f;
			sTireThermal( w, 0.0f, chassis_speed );
			continue;
		}

		// --- Contact patch basis ---
		// Wheel forward = chassis forward rotated by steer angle about chassis up.
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
		b3Vec3 wheel_side = b3Cross( up, wheel_fwd ); // points left

		// --- Slips from contact-patch velocity ---
		b3Vec3 vel = b3Body_GetWorldPointVelocity( chassis, ray.point );
		float v_long = b3Dot( vel, wheel_fwd );
		float v_lat = b3Dot( vel, wheel_side );

		float denom = b3AbsFloat( v_long );
		if ( denom < t->slip_v_min )
		{
			denom = t->slip_v_min;
		}

		float slip_ratio = ( w->omega * t->wheel_radius - v_long ) / denom;
		slip_ratio = b3ClampFloat( slip_ratio, -4.0f, 4.0f );
		float slip_angle = b3Atan2( -v_lat, denom );

		w->slip_ratio = slip_ratio;
		w->slip_angle = slip_angle;

		// --- Brush contact-patch forces (docs/TIRE-MODEL.md §1) ---
		// Slip vector: sigma_x = slip ratio, sigma_y = tan(slip angle) =
		// -v_lat/denom (exactly what slip_angle's atan2 argument already is).
		// Combined slip shares one friction budget inside the patch model.
		float sigma_y = -v_lat / denom;
		float fx, fy, trail;
		vehicle_brush_patch( slip_ratio, sigma_y, fz, w->t_surf, &fx, &fy, &trail );

		b3Vec3 tire_force = b3Add( b3MulSV( fx, wheel_fwd ), b3MulSV( fy, wheel_side ) );
		b3Body_ApplyForce( chassis, tire_force, ray.point, true );

		// --- FFB rack torque (front axle only, output-only) ---
		// Lateral force acts behind the steering axis by the EMERGENT
		// pneumatic trail plus mechanical caster (docs/TIRE-MODEL.md §3).
		// The trail collapse past the grip peak — the "light wheel"
		// understeer cue — comes out of the patch moment, not a heuristic.
		if ( sIsFront( i ) )
		{
			rack += fy * ( trail + t->ffb_caster_trail ) + fx * t->ffb_scrub_radius;
		}

		// --- Tire thermal state (docs/TIRE-MODEL.md §2) ---
		// Friction power from the slip velocity at the patch.
		{
			float v_slip_x = w->omega * t->wheel_radius - v_long;
			float p_fric = b3AbsFloat( fx * v_slip_x ) + b3AbsFloat( fy * v_lat );
			sTireThermal( w, p_fric, chassis_speed );
		}

		// --- Wheel spin dynamics (RWD, brakes on all four) ---
		float drive = 0.0f;
		if ( !sIsFront( i ) && !handbrake )
		{
			float rpm = w->omega * t->gear_ratio * ( 60.0f / TWO_PI_F );
			if ( rpm < 1000.0f )
			{
				rpm = 1000.0f;
			}
			drive = throttle * sEngineTorque( rpm ) * t->gear_ratio * t->driveline_eff * 0.5f;
		}

		float brake_cap = sIsFront( i ) ? t->brake_torque_front : t->brake_torque_rear;
		float brake_trq = brake * brake_cap;
		if ( handbrake && !sIsFront( i ) )
		{
			brake_trq += t->handbrake_torque;
		}

		float reaction = -fx * t->wheel_radius;
		float net = drive + reaction;

		// Brake torque opposes spin; do not let it reverse the wheel in one tick.
		float omega_new = w->omega + ( net / t->wheel_inertia ) * SIM_DT;
		float brake_dw = ( brake_trq / t->wheel_inertia ) * SIM_DT;
		if ( omega_new > 0.0f )
		{
			omega_new = omega_new > brake_dw ? omega_new - brake_dw : 0.0f;
		}
		else if ( omega_new < 0.0f )
		{
			omega_new = omega_new < -brake_dw ? omega_new + brake_dw : 0.0f;
		}

		// Handbrake locks the rears outright.
		if ( handbrake && !sIsFront( i ) )
		{
			omega_new = 0.0f;
		}

		w->omega = omega_new;
		w->spin_angle += w->omega * SIM_DT;
		if ( w->spin_angle > 3.1415927f )
		{
			w->spin_angle -= TWO_PI_F;
		}
		else if ( w->spin_angle < -3.1415927f )
		{
			w->spin_angle += TWO_PI_F;
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

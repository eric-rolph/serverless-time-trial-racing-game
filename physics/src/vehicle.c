// vehicle.c — raycast-suspension vehicle with Pacejka MF-lite tires (ADR-007).
//
// Chassis: one dynamic Box3D rigid body (box hull ~1.9 x 1.1 x 4.4 m, 1350 kg,
// explicit inertia). Wheels are NOT rigid bodies: each is a fixed hardpoint
// raycast down the chassis-local -Y axis, a spring/damper along chassis up,
// and Pacejka Magic Formula tire forces applied at the contact patch.
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

typedef struct PacejkaAxis
{
	float B; // stiffness factor
	float C; // shape factor
	float mu; // peak friction coefficient: D = mu * Fz
	float E; // curvature factor
} PacejkaAxis;

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

	// Tires
	PacejkaAxis longitudinal;
	PacejkaAxis lateral;
	float max_load;		  // Fz clamp (N) to bound force spikes
	float slip_v_min;	  // low-speed epsilon for slip computation (m/s)
	float wheel_inertia;  // kg m^2 per wheel

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

	// Force-feedback signal (docs/FFB.md). Output-only; no simulation effect.
	float ffb_trail0;		  // pneumatic trail at zero slip (m)
	float ffb_trail_falloff;  // trail collapse rate per rad of slip angle
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

	.longitudinal = { .B = 12.0f, .C = 1.65f, .mu = 1.15f, .E = 0.97f },
	.lateral = { .B = 10.0f, .C = 1.30f, .mu = 1.00f, .E = 0.97f },
	.max_load = 9000.0f,
	.slip_v_min = 0.8f,
	.wheel_inertia = 1.2f,

	.gear_ratio = 8.2f,
	.driveline_eff = 0.90f,
	.engine_rpm_pts = { 1000.0f, 3000.0f, 5000.0f, 6500.0f, 7500.0f },
	.engine_trq_pts = { 220.0f, 320.0f, 340.0f, 300.0f, 0.0f },

	.brake_torque_front = 1600.0f,
	.brake_torque_rear = 1050.0f,
	.handbrake_torque = 2500.0f,

	.max_steer = 0.5235988f, // 30 degrees

	.ffb_trail0 = 0.030f,
	.ffb_trail_falloff = 8.3f, // trail gone by ~7° slip (past the Pacejka peak)
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
	}
	v->rack_torque = 0.0f;
}

// ---------------------------------------------------------------------------
// Tire model
// ---------------------------------------------------------------------------

// Pacejka Magic Formula: F = D sin(C atan(B s − E (B s − atan(B s))))
static float sPacejka( const PacejkaAxis* a, float slip, float fz )
{
	float D = a->mu * fz;
	float Bs = a->B * slip;
	float inner = Bs - a->E * ( Bs - b3Atan2( Bs, 1.0f ) );
	float phi = a->C * b3Atan2( inner, 1.0f );
	b3CosSin cs = b3ComputeCosSin( phi );
	return D * cs.sine;
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

		// --- Pacejka forces + friction-ellipse combining ---
		float fx = sPacejka( &t->longitudinal, slip_ratio, fz );
		float fy = sPacejka( &t->lateral, slip_angle, fz );

		float dx = t->longitudinal.mu * fz;
		float dy = t->lateral.mu * fz;
		float ex = fx / dx;
		float ey = fy / dy;
		float rho2 = ex * ex + ey * ey;
		if ( rho2 > 1.0f )
		{
			float inv = 1.0f / sqrtf( rho2 );
			fx *= inv;
			fy *= inv;
		}

		b3Vec3 tire_force = b3Add( b3MulSV( fx, wheel_fwd ), b3MulSV( fy, wheel_side ) );
		b3Body_ApplyForce( chassis, tire_force, ray.point, true );

		// --- FFB rack torque (front axle only, output-only) ---
		// Lateral force acts behind the steering axis by pneumatic + caster
		// trail; the pneumatic component collapses as slip passes the Pacejka
		// peak, which is the natural "light wheel" understeer cue.
		if ( sIsFront( i ) )
		{
			float trail_scale = 1.0f - b3AbsFloat( slip_angle ) * t->ffb_trail_falloff;
			if ( trail_scale < 0.0f )
			{
				trail_scale = 0.0f;
			}
			float trail = t->ffb_trail0 * trail_scale + t->ffb_caster_trail;
			rack += fy * trail + fx * t->ffb_scrub_radius;
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
	b3Vec3 vel = b3Body_GetLinearVelocity( chassis );
	float speed = b3Length( vel );
	if ( speed > 0.01f )
	{
		b3Vec3 drag = b3MulSV( -t->drag_coef * speed, vel );
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

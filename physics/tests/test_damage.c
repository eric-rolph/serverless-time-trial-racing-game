// test_damage.c — deterministic crash-damage model (docs/SOFTBODY.md Phase 2).
//
//   1. CLEAN LAP ZERO   — a normal scripted lap on the flat oval accrues
//                          EXACTLY 0.0 damage (the leaderboard-integrity gate).
//   2. DETERMINISM      — the same impact sequence yields bit-identical damage.
//   3. MONOTONICITY     — aero and max_steer fall (never rise) as damage grows
//                          and stay strictly positive at full damage.
//   4. RESET            — vehicle_reset zeros every damage scalar.
//
// The deterministic test contact is created via the internal, non-exported
// helper vehicle_apply_impact(v, approachSpeed, localPoint) (the documented
// option B in the mission) — no world contact geometry required, so the
// impact is perfectly reproducible.

#include "make_test_track.h"
#include "sim/sim.h"
#include "vehicle.h" // internal Vehicle + damage helpers (src/, not public)

#include "box3d/box3d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int g_failures = 0;

#define CHECK( cond, msg )                                                                                              \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( !( cond ) )                                                                                              \
		{                                                                                                              \
			fprintf( stderr, "FAIL: %s (line %d)\n", msg, __LINE__ );                                                \
			g_failures++;                                                                                              \
		}                                                                                                              \
		else                                                                                                           \
		{                                                                                                              \
			printf( "ok: %s\n", msg );                                                                               \
		}                                                                                                              \
	} while ( 0 )

// Heading = chassis forward (+Z local) in world, from the state quaternion.
static void heading( const SimStateV1* s, float* fx, float* fz )
{
	float x = s->quat[0], y = s->quat[1], z = s->quat[2], w = s->quat[3];
	*fx = 2.0f * ( x * z + w * y );
	*fz = 1.0f - 2.0f * ( x * x + y * y );
}

// ---------------------------------------------------------------------------
// 1. CLEAN LAP ZERO — pursuit-drive the flat oval, assert no damage.
// ---------------------------------------------------------------------------
static void test_clean_lap_zero( const uint8_t* blob, size_t blob_len )
{
	int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)blob_len );
	if ( rc != 0 )
	{
		fprintf( stderr, "FAIL: sim_load_track %d\n", rc );
		g_failures++;
		return;
	}
	const SimStateV1* s = sim_state();

	// Oval centerline (same params as make_test_track.c).
	enum { S = 256 };
	static float cx[S], cz[S];
	for ( int i = 0; i < S; ++i )
	{
		double th = ( 2.0 * 3.14159265358979323846 * (double)i ) / (double)S;
		cx[i] = (float)( 60.0 * sin( th ) );
		cz[i] = (float)( 40.0 * cos( th ) );
	}

	sim_reset();
	int completed = 0;
	int invalid_seen = 0;
	float max_dmg = 0.0f;

	for ( int tick = 0; tick < 24000 && !completed; ++tick )
	{
		float bx = s->pos[0], bz = s->pos[2];
		int best = 0;
		float best_d = 1e30f;
		for ( int i = 0; i < S; ++i )
		{
			float dx = cx[i] - bx, dz = cz[i] - bz;
			float d = dx * dx + dz * dz;
			if ( d < best_d )
			{
				best_d = d;
				best = i;
			}
		}
		int target = ( best + 8 ) % S;
		float dx = cx[target] - bx, dz = cz[target] - bz;
		float dl = sqrtf( dx * dx + dz * dz );
		dx /= dl;
		dz /= dl;
		float fx, fz;
		heading( s, &fx, &fz );
		float err = fz * dx - fx * dz;
		float dot = fx * dx + fz * dz;
		float ang = atan2f( err, dot );
		int32_t steer = (int32_t)( 3.0f * ang / 0.5235988f * 32767.0f );
		if ( steer > 32767 )
		{
			steer = 32767;
		}
		if ( steer < -32767 )
		{
			steer = -32767;
		}
		uint32_t throttle = s->speed < 11.0f ? 45000u : 0u;

		uint32_t status = sim_step( steer, throttle, 0, 0 );
		if ( status & SIM_STATUS_ERROR )
		{
			fprintf( stderr, "FAIL: ERROR during clean lap\n" );
			g_failures++;
			break;
		}
		if ( status & SIM_STATUS_LAP_INVALID )
		{
			invalid_seen = 1;
		}
		// Damage must never budge off zero at ANY point of a clean lap.
		float d = sim_damage( 0 );
		if ( d > max_dmg )
		{
			max_dmg = d;
		}
		if ( status & SIM_STATUS_LAP_COMPLETE )
		{
			completed = 1;
		}
	}

	printf( "clean lap: completed=%d invalid=%d max_damage_seen=%.9g\n", completed, invalid_seen, (double)max_dmg );
	CHECK( completed, "clean lap completes" );
	CHECK( !invalid_seen, "clean lap never flagged LAP_INVALID" );
	// THE linchpin gate: a lap that does not crash is physically identical to
	// today — zero damage, exactly.
	CHECK( max_dmg == 0.0f, "damage stays EXACTLY 0 through the whole clean lap" );
	CHECK( sim_damage( 0 ) == 0.0f, "final overall damage == 0" );
	CHECK( sim_damage( 1 ) == 0.0f, "final steer damage == 0" );
	CHECK( sim_damage( 2 ) == 0.0f, "final front-toe damage == 0" );
	CHECK( sim_damage( 3 ) == 0.0f, "final rear-toe damage == 0" );

	// Out-of-range component reads 0.
	CHECK( sim_damage( 4 ) == 0.0f, "sim_damage(4) out-of-range → 0" );
	CHECK( sim_damage( 99 ) == 0.0f, "sim_damage(99) out-of-range → 0" );
}

// ---------------------------------------------------------------------------
// 2. DETERMINISM — identical impacts → identical damage.
// ---------------------------------------------------------------------------
static void apply_seq( Vehicle* v )
{
	// A mix of front/rear, left/right, and one below-threshold (no-op) impact.
	vehicle_apply_impact( v, 14.0f, ( b3Vec3 ){ 0.8f, 0.0f, 2.0f } );	// front-right
	vehicle_apply_impact( v, 9.0f, ( b3Vec3 ){ -0.5f, 0.1f, -2.0f } );	// rear-left
	vehicle_apply_impact( v, 25.0f, ( b3Vec3 ){ 0.0f, 0.0f, 2.2f } );	// front, hard
	vehicle_apply_impact( v, 5.0f, ( b3Vec3 ){ 0.0f, 0.0f, 2.0f } );	// below threshold
	vehicle_apply_impact( v, 11.5f, ( b3Vec3 ){ 0.6f, 0.0f, -2.1f } );	// rear-right
}

static void test_determinism( void )
{
	Vehicle a = { 0 };
	Vehicle b = { 0 };
	apply_seq( &a );
	apply_seq( &b );

	CHECK( a.damage_overall == b.damage_overall, "determinism: damage_overall identical" );
	CHECK( a.damage_steer == b.damage_steer, "determinism: damage_steer identical" );
	CHECK( a.damage_toe_front == b.damage_toe_front, "determinism: damage_toe_front identical" );
	CHECK( a.damage_toe_rear == b.damage_toe_rear, "determinism: damage_toe_rear identical" );

	// Sanity: the impacts actually registered (not a trivial 0==0 pass).
	CHECK( a.damage_overall > 0.0f, "impacts accrue overall damage" );
	CHECK( a.damage_steer > 0.0f, "front impacts accrue steer damage" );
	CHECK( a.damage_toe_front != 0.0f, "front impacts bend front toe" );
	CHECK( a.damage_toe_rear != 0.0f, "rear impacts bend rear toe" );

	// Below-threshold impact alone must be a pure no-op.
	Vehicle c = { 0 };
	vehicle_apply_impact( &c, SIM_DAMAGE_THRESHOLD, ( b3Vec3 ){ 0.0f, 0.0f, 2.0f } );
	vehicle_apply_impact( &c, SIM_DAMAGE_THRESHOLD - 1.0f, ( b3Vec3 ){ 0.0f, 0.0f, 2.0f } );
	CHECK( c.damage_overall == 0.0f && c.damage_steer == 0.0f && c.damage_toe_front == 0.0f &&
			   c.damage_toe_rear == 0.0f,
		   "at/below-threshold impacts accrue no damage" );
}

// ---------------------------------------------------------------------------
// 3. MONOTONICITY — effects fall as damage rises, positive at full damage.
// ---------------------------------------------------------------------------
static void test_monotonicity( void )
{
	Vehicle v = { 0 };
	float base_aero = vehicle_effect_aero_front( &v );
	float base_steer = vehicle_effect_max_steer( &v );
	CHECK( base_aero > 0.0f && base_steer > 0.0f, "undamaged effects positive" );

	float prev_aero = base_aero;
	float prev_steer = base_steer;
	int mono = 1;
	for ( int k = 0; k < 12; ++k )
	{
		// Rising-severity FRONT impacts drive both overall and steer damage.
		vehicle_apply_impact( &v, 8.0f + (float)k * 2.0f, ( b3Vec3 ){ 0.0f, 0.0f, 2.0f } );
		float aero = vehicle_effect_aero_front( &v );
		float steer = vehicle_effect_max_steer( &v );
		if ( aero > prev_aero || steer > prev_steer )
		{
			mono = 0;
		}
		prev_aero = aero;
		prev_steer = steer;
	}
	CHECK( mono, "aero and max_steer never increase as damage rises" );
	CHECK( prev_aero < base_aero, "aero strictly reduced by damage" );
	CHECK( prev_steer < base_steer, "max_steer strictly reduced by damage" );

	// Saturate to full damage and confirm the car still has grip and steering.
	for ( int k = 0; k < 20; ++k )
	{
		vehicle_apply_impact( &v, 60.0f, ( b3Vec3 ){ 0.0f, 0.0f, 2.0f } );
	}
	CHECK( v.damage_overall == 1.0f, "damage_overall saturates at 1.0" );
	CHECK( v.damage_steer == 1.0f, "damage_steer saturates at 1.0" );
	CHECK( vehicle_effect_aero_front( &v ) > 0.0f, "aero stays positive at full damage (limps, not dead)" );
	CHECK( vehicle_effect_max_steer( &v ) > 0.0f, "max_steer stays positive at full damage" );
}

// ---------------------------------------------------------------------------
// 4. RESET — vehicle_reset zeros the damage scalars.
// ---------------------------------------------------------------------------
static void test_reset( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	wd.enableSleep = false;
	wd.workerCount = 0;
	wd.enqueueTask = NULL;
	wd.finishTask = NULL;
	b3WorldId world = b3CreateWorld( &wd );

	Vehicle v;
	vehicle_create( world, &v, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f }, 0.0f );
	CHECK( v.damage_overall == 0.0f && v.damage_steer == 0.0f && v.damage_toe_front == 0.0f &&
			   v.damage_toe_rear == 0.0f,
		   "vehicle_create starts pristine" );

	vehicle_apply_impact( &v, 30.0f, ( b3Vec3 ){ 0.7f, 0.0f, 2.0f } );	// front-right
	vehicle_apply_impact( &v, 22.0f, ( b3Vec3 ){ -0.6f, 0.0f, -2.0f } ); // rear-left
	CHECK( v.damage_overall > 0.0f && v.damage_steer > 0.0f && v.damage_toe_front != 0.0f &&
			   v.damage_toe_rear != 0.0f,
		   "impacts populate all damage channels" );

	vehicle_reset( world, &v, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f }, 0.0f );
	CHECK( v.damage_overall == 0.0f, "reset zeros damage_overall" );
	CHECK( v.damage_steer == 0.0f, "reset zeros damage_steer" );
	CHECK( v.damage_toe_front == 0.0f, "reset zeros damage_toe_front" );
	CHECK( v.damage_toe_rear == 0.0f, "reset zeros damage_toe_rear" );

	vehicle_destroy( &v );
	b3DestroyWorld( world );
}

int main( void )
{
	size_t blob_len = 0;
	uint8_t* blob = make_test_track( &blob_len );
	if ( blob == NULL )
	{
		fprintf( stderr, "make_test_track failed\n" );
		return 1;
	}

	test_clean_lap_zero( blob, blob_len );
	test_determinism();
	test_monotonicity();
	test_reset();

	free( blob );

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_damage: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_damage PASS\n" );
	return 0;
}

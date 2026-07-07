// test_vehicle.c — vehicle sanity checks on the flat oval:
//   1. suspension settles near static load, car does not fall through terrain
//   2. throttle accelerates forward
//   3. brakes stop the car
//   4. steering yaws the chassis
//   5. handbrake locks the rear wheels

#include "make_test_track.h"
#include "sim/sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int g_failures = 0;

#define CHECK( cond, msg )                                                                                             \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( !( cond ) )                                                                                               \
		{                                                                                                              \
			fprintf( stderr, "FAIL: %s (line %d)\n", msg, __LINE__ );                                                  \
			g_failures++;                                                                                              \
		}                                                                                                              \
		else                                                                                                           \
		{                                                                                                              \
			printf( "ok: %s\n", msg );                                                                                 \
		}                                                                                                              \
	} while ( 0 )

static void run_ticks( int n, int32_t steer, uint32_t throttle, uint32_t brake, uint32_t flags )
{
	for ( int i = 0; i < n; ++i )
	{
		uint32_t status = sim_step( steer, throttle, brake, flags );
		if ( status & SIM_STATUS_ERROR )
		{
			fprintf( stderr, "FAIL: sim_step ERROR\n" );
			exit( 1 );
		}
	}
}

// Heading = chassis forward (+Z local) in world, from the state quaternion.
static void heading( const SimStateV1* s, float* fx, float* fz )
{
	float x = s->quat[0], y = s->quat[1], z = s->quat[2], w = s->quat[3];
	// R * (0,0,1)
	*fx = 2.0f * ( x * z + w * y );
	*fz = 1.0f - 2.0f * ( x * x + y * y );
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

	int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)blob_len );
	if ( rc != 0 )
	{
		fprintf( stderr, "sim_load_track failed: %d\n", rc );
		return 1;
	}

	const SimStateV1* s = sim_state();

	// --- 1. Settle: 1.5 s idle ---
	sim_reset();
	run_ticks( 600, 0, 0, 0, 0 );

	CHECK( s->pos[1] > 0.2f && s->pos[1] < 1.5f, "car rests above terrain (no fall-through)" );
	CHECK( s->speed < 0.6f, "car nearly at rest after settling" );

	// Static load per wheel ~ mg/4 = 1350*9.81/4 = 3311 N; k = 60000 N/m
	// → expected compression ≈ 0.055 m. Allow generous tolerance for
	// damping transients and front/rear balance.
	for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
	{
		float c = s->wheels[i].susp_compression;
		char msg[64];
		snprintf( msg, sizeof( msg ), "wheel %d compression near static load (%.4f m)", i, (double)c );
		CHECK( c > 0.025f && c < 0.10f, msg );
	}

	// --- 2. Throttle accelerates forward: 2 s full throttle ---
	sim_reset();
	run_ticks( 400, 0, 0, 0, 0 ); // settle first
	float speed0 = s->speed;
	run_ticks( 800, 0, 65535, 0, 0 );
	printf( "speed after 2 s full throttle: %.2f m/s\n", (double)s->speed );
	CHECK( s->speed > speed0 + 5.0f, "throttle accelerates the car (>5 m/s gained in 2 s)" );

	// Moving forward, not backward: velocity aligned with heading.
	{
		float fx, fz;
		heading( s, &fx, &fz );
		float along = fx * s->lin_vel[0] + fz * s->lin_vel[2];
		CHECK( along > 5.0f, "car moves along its forward axis" );
	}

	// --- 3. Brakes stop it ---
	run_ticks( 1600, 0, 0, 65535, 0 ); // up to 4 s of braking
	printf( "speed after braking: %.2f m/s\n", (double)s->speed );
	CHECK( s->speed < 0.8f, "brakes stop the car" );

	// --- 4. Steering yaws ---
	sim_reset();
	run_ticks( 400, 0, 0, 0, 0 );
	run_ticks( 800, 0, 65535, 0, 0 ); // get moving straight
	float fx_before, fz_before;
	heading( s, &fx_before, &fz_before );
	run_ticks( 400, 20000, 39321, 0, 0 ); // steer right 1 s
	float fx_after, fz_after;
	heading( s, &fx_after, &fz_after );
	// Signed turn angle about +Y: cross(before, after).y = fz_before*fx_after - fx_before*fz_after
	float turn = fz_before * fx_after - fx_before * fz_after;
	printf( "turn cross-product after 1 s of steering: %.3f\n", (double)turn );
	CHECK( fabsf( turn ) > 0.05f, "steering yaws the chassis" );
	CHECK( fabsf( s->wheels[0].steer_angle - 20000.0f / 32767.0f * 0.5235988f ) < 1e-3f,
		   "front wheel steer angle matches linear map" );
	CHECK( s->wheels[2].steer_angle == 0.0f, "rear wheels do not steer" );

	// --- 5. Handbrake locks rears ---
	sim_reset();
	run_ticks( 400, 0, 0, 0, 0 );
	run_ticks( 800, 0, 65535, 0, 0 );
	run_ticks( 200, 0, 0, 0, 1 ); // handbrake 0.5 s
	CHECK( s->wheels[2].slip_ratio < -0.5f || s->wheels[3].slip_ratio < -0.5f,
		   "handbrake locks rear wheels (deep negative slip while moving)" );

	// --- 6. Full lap with a simple pursuit controller → LAP_COMPLETE ---
	// Rebuild the oval centerline (same params as make_test_track.c).
	{
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
		uint32_t lap_ticks = 0;

		for ( int tick = 0; tick < 24000 && !completed; ++tick )
		{
			// Nearest centerline sample (full scan, host-side only)
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
			// Pursue a look-ahead point
			int target = ( best + 8 ) % S;
			float dx = cx[target] - bx, dz = cz[target] - bz;
			float dl = sqrtf( dx * dx + dz * dz );
			dx /= dl;
			dz /= dl;
			float fx, fz;
			heading( s, &fx, &fz );
			float err = fz * dx - fx * dz;			 // cross(fwd, desired).y
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
				fprintf( stderr, "FAIL: ERROR during lap drive\n" );
				g_failures++;
				break;
			}
			if ( status & SIM_STATUS_LAP_INVALID )
			{
				invalid_seen = 1;
			}
			if ( status & SIM_STATUS_LAP_COMPLETE )
			{
				completed = 1;
				lap_ticks = sim_lap_time_ticks();
			}
		}

		printf( "lap: completed=%d invalid=%d lap_ticks=%u (%.2f s)\n", completed, invalid_seen, lap_ticks,
				(double)lap_ticks / SIM_TICK_RATE );
		CHECK( completed, "driven lap sets LAP_COMPLETE" );
		CHECK( !invalid_seen, "driven lap never flagged LAP_INVALID" );
		CHECK( lap_ticks > 2000 && lap_ticks < 24000, "lap_time_ticks is plausible" );
	}

	free( blob );

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_vehicle: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_vehicle PASS\n" );
	return 0;
}

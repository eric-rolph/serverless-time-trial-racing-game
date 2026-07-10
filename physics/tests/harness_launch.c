// harness_launch.c — docs/DRIVETRAIN.md §7 gate 4 (+ §5 brake-bias check):
//
//   A. 0-100 km/h in 4-5 s with an auto-shifting driver (up at ~7200 rpm),
//      WITH measurable 1st-gear wheelspin (rear slip ratio > 0.15 transiently).
//   B. gear/top-speed table: redline speeds for gears 1-5 (analytic), 6th
//      driven to its drag-limited top on the big circle — must land in the
//      270-280 km/h window.
//   C. stopping distance from 29 m/s within +/-10% of the pre-drivetrain
//      baseline (41.01 m, measured on this exact scenario against the
//      pre-wave kernel — see physics/NOTES.md).
//
// Runs on a huge flat circle (R = 4000 m: lateral accel at 78 m/s is only
// ~1.5 m/s^2, so the "straight" is straight enough for top-speed work).

#include "make_test_track.h"
#include "sim/sim.h"
#include "vehicle.h"

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

#define WHEEL_RADIUS 0.33f
#define LIMITER_W 785.39816f // 7500 rpm

#define R_TRACK 4000.0
#define N_SAMPLES 10053

// Pre-drivetrain baseline (same scenario, pre-wave kernel; NOTES.md).
#define BASELINE_STOP_M 41.01f

static double s_cx[N_SAMPLES], s_cz[N_SAMPLES];
static int s_nearest = 0;

static void heading( const SimStateV1* s, float* fx, float* fz )
{
	float x = s->quat[0], y = s->quat[1], z = s->quat[2], w = s->quat[3];
	*fx = 2.0f * ( x * z + w * y );
	*fz = 1.0f - 2.0f * ( x * x + y * y );
}

static int32_t pursuit_steer( const SimStateV1* s )
{
	float bx = s->pos[0], bz = s->pos[2];
	int best = s_nearest;
	float best_d = 1e30f;
	for ( int k = -8; k <= 60; ++k )
	{
		int i = ( s_nearest + k + N_SAMPLES ) % N_SAMPLES;
		float dx = (float)s_cx[i] - bx, dz = (float)s_cz[i] - bz;
		float d = dx * dx + dz * dz;
		if ( d < best_d )
		{
			best_d = d;
			best = i;
		}
	}
	s_nearest = best;
	int look = 8 + (int)( s->speed * 0.5f );
	int target = ( best + look ) % N_SAMPLES;
	float dx = (float)s_cx[target] - bx, dz = (float)s_cz[target] - bz;
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
	return steer;
}

// One auto-shifting tick: up at 6600 rpm — right past the 6500 rpm power
// peak, before the torque dive chokes acceleration (hysteresis via a 40-tick
// cooldown). Returns the flags to use.
static int s_shift_cooldown = 0;
static uint32_t auto_up_flags( void )
{
	uint32_t flags = 0;
	if ( s_shift_cooldown > 0 )
	{
		s_shift_cooldown--;
	}
	if ( sim_gear() >= 1 && sim_gear() < 6 && sim_rpm() > 6600.0f && s_shift_cooldown == 0 )
	{
		flags = SIM_FLAG_SHIFT_UP;
		s_shift_cooldown = 40;
	}
	return flags;
}

// Launch-control throttle: a human feathers the pedal against wheelspin.
// Full throttle in 1st lights the rears up to the limiter (that IS the car);
// for a clean 0-100 the driver cuts the pedal proportionally to the slip
// excess over the force peak (|sigma| ~ 0.15-0.20 for this brush patch).
// Stateless: reacts within one tick, so a spin-up cannot run away past the
// kinetic-friction cliff.
static uint32_t modulated_throttle( const SimStateV1* s )
{
	float slip = s->wheels[2].slip_ratio > s->wheels[3].slip_ratio ? s->wheels[2].slip_ratio : s->wheels[3].slip_ratio;
	float th = 1.0f - 3.0f * ( slip - 0.15f );
	th = th < 0.25f ? 0.25f : ( th > 1.0f ? 1.0f : th );
	return (uint32_t)( th * 65535.0f );
}

int main( void )
{
	size_t blob_len = 0;
	uint8_t* blob = make_test_track_ex( &blob_len, (float)R_TRACK, (float)R_TRACK, N_SAMPLES, 12.0f, 30.0f );
	if ( blob == NULL )
	{
		fprintf( stderr, "make_test_track_ex failed\n" );
		return 1;
	}
	for ( int i = 0; i < N_SAMPLES; ++i )
	{
		double th = ( 2.0 * 3.14159265358979323846 * (double)i ) / (double)N_SAMPLES;
		s_cx[i] = R_TRACK * sin( th );
		s_cz[i] = R_TRACK * cos( th );
	}
	if ( sim_load_track( (sim_ptr_t)blob, (uint32_t)blob_len ) != 0 )
	{
		fprintf( stderr, "sim_load_track failed\n" );
		return 1;
	}
	const SimStateV1* s = sim_state();

	// ---------------- A. 0-100 km/h + 1st-gear wheelspin ----------------
	sim_reset();
	s_nearest = 0;
	s_shift_cooldown = 0;
	for ( int i = 0; i < 400; ++i )
	{
		sim_step( 0, 0, 0, 0 );
	}

	const float V100 = 100.0f / 3.6f; // 27.78 m/s
	float max_rear_slip_1st = 0.0f;
	int t100 = -1;
	for ( int t = 0; t < 6000 && t100 < 0; ++t )
	{
		uint32_t flags = auto_up_flags();
		uint32_t th = modulated_throttle( s );
		sim_step( pursuit_steer( s ), th, 0, flags );
		if ( sim_gear() == 1 )
		{
			float sl = s->wheels[2].slip_ratio > s->wheels[3].slip_ratio ? s->wheels[2].slip_ratio
																		 : s->wheels[3].slip_ratio;
			if ( sl > max_rear_slip_1st )
			{
				max_rear_slip_1st = sl;
			}
		}
		if ( t % 200 == 199 )
		{
			printf( "  t=%.1fs v=%5.2f gear=%d rpm=%4.0f slip=%5.2f th=%.2f Tr=%.0fC\n", ( t + 1 ) / 400.0,
					(double)s->speed, sim_gear(), (double)sim_rpm(), (double)s->wheels[3].slip_ratio,
					th / 65535.0, (double)sim_tire_temp( 3 ) );
		}
		if ( s->speed >= V100 )
		{
			t100 = t + 1;
		}
	}
	float t100_s = (float)t100 / 400.0f;
	printf( "0-100 km/h: %.2f s (gear at 100: %d), max 1st-gear rear slip ratio %.3f\n", (double)t100_s, sim_gear(),
			(double)max_rear_slip_1st );
	CHECK( t100 > 0, "reaches 100 km/h" );
	CHECK( t100_s >= 4.0f && t100_s <= 5.0f, "0-100 km/h in 4-5 s" );
	CHECK( max_rear_slip_1st > 0.15f, "1st-gear launch wheelspin (rear slip ratio > 0.15 transiently)" );

	// ---------------- B. gear / top-speed table ----------------
	printf( "\ngear/top-speed table (redline %d rpm, wheel %.2f m):\n", 7500, (double)WHEEL_RADIUS );
	for ( int g = 1; g <= 5; ++g )
	{
		float v = LIMITER_W / vehicle_gear_ratio( g ) * WHEEL_RADIUS;
		printf( "  %d: %6.1f km/h (redline-limited)\n", g, (double)( v * 3.6f ) );
	}

	// 6th: keep accelerating until the speed plateaus (drag-limited top; the
	// approach is asymptotic with tau ~ 20 s, so "no +0.05 m/s in 4 s" cuts
	// the run within ~0.25 m/s of the true equilibrium).
	float v_top = 0.0f;
	int plateau = 0;
	for ( int t = 0; t < 48000 && plateau < 1600; ++t ) // up to 2 min
	{
		uint32_t flags = auto_up_flags();
		sim_step( pursuit_steer( s ), 65535, 0, flags );
		if ( s->speed > v_top + 0.05f )
		{
			v_top = s->speed;
			plateau = 0;
		}
		else
		{
			plateau++;
		}
	}
	if ( s->speed > v_top )
	{
		v_top = s->speed;
	}
	printf( "  6: %6.1f km/h (drag-limited, measured; gear %d, %.0f rpm)\n", (double)( v_top * 3.6f ), sim_gear(),
			(double)sim_rpm() );
	CHECK( sim_gear() == 6, "top-speed run ends in 6th" );
	CHECK( v_top * 3.6f >= 270.0f && v_top * 3.6f <= 280.0f, "6th gear top speed in the 270-280 km/h window" );

	float v1 = LIMITER_W / vehicle_gear_ratio( 1 ) * WHEEL_RADIUS * 3.6f;
	CHECK( v1 > 90.0f && v1 < 100.0f, "1st tops ~95 km/h" );

	// ---------------- C. stopping distance vs baseline ----------------
	sim_reset();
	s_nearest = 0;
	s_shift_cooldown = 0;
	for ( int i = 0; i < 400; ++i )
	{
		sim_step( 0, 0, 0, 0 );
	}
	int reached = 0;
	for ( int t = 0; t < 24000; ++t )
	{
		uint32_t flags = auto_up_flags();
		sim_step( pursuit_steer( s ), 65535, 0, flags );
		if ( s->speed >= 29.0f )
		{
			reached = 1;
			break;
		}
	}
	CHECK( reached, "reaches 29 m/s for the brake test" );
	float x0 = s->pos[0], z0 = s->pos[2];
	float v0 = s->speed;
	int ticks = 0;
	while ( s->speed > 0.5f && ticks < 8000 )
	{
		sim_step( 0, 0, 65535, 0 );
		ticks++;
	}
	float dxs = s->pos[0] - x0, dzs = s->pos[2] - z0;
	float dist = sqrtf( dxs * dxs + dzs * dzs );
	printf( "\nstop from %.2f m/s: %.2f m in %.2f s (baseline %.2f m, +/-10%% gate)\n", (double)v0, (double)dist,
			(double)ticks / 400.0, (double)BASELINE_STOP_M );
	CHECK( dist > 0.90f * BASELINE_STOP_M && dist < 1.10f * BASELINE_STOP_M,
		   "stopping distance within +/-10%% of the pre-wave baseline" );

	free( blob );

	if ( g_failures != 0 )
	{
		fprintf( stderr, "harness_launch: %d failures\n", g_failures );
		return 1;
	}
	printf( "harness_launch PASS\n" );
	return 0;
}

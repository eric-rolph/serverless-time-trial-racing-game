// test_drivetrain.c — gearbox / engine / LSD validation gates
// (docs/DRIVETRAIN.md §7 gate 2).
//
//   1. ratio table sanity: 6 forward gears strictly decreasing, reverse ≈ -1st,
//      1st redline speed ~95 km/h, 6th redline speed high enough that drag
//      (not the limiter) sets the 270-280 km/h top speed.
//   2. edge-triggered shifts: holding the bit = exactly ONE shift.
//   3. cut timing in ticks: upshift engages after exactly 28 ticks, downshift
//      after exactly 48, with the auto-blip rev-match snap on engage.
//   4. rev limiter: rpm ceilings at ~7500 in 1st, speed plateaus at the 1st
//      redline speed.
//   5. engine braking: closed-throttle decel in 3rd is real (0.4-1.5 m/s^2)
//      and clearly stronger than in 6th (ratio-scaled crank drag).
//   6. LSD split: direction (faster wheel sheds torque), conservation, 40%
//      clamp, coast behavior; engine-brake torque curve endpoints.
//   7. reverse rule: 1 -> R only below 1 m/s, R -> 1 at any speed.
//   8. downshift over-rev protection: refuse when the target gear would spin
//      the engine past 7800 rpm, accept once slowed.
//   9. CROWN GATE: a shift-heavy closed-loop log (launch, 1->6 upshifts,
//      brake with downshifts, reverse shuffle) recorded once, then re-run
//      via sim_step and via sim_replay from sim_reset — all three final
//      hashes bit-identical.
//
// Runs on a big flat circle (R = 2000 m) so top-gear speeds fit on the road.

#include "make_test_track.h"
#include "sim/sim.h"
#include "vehicle.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

// Mirrors of kTuning constants (deliberately static in src/vehicle.c).
#define WHEEL_RADIUS 0.33f
#define LIMITER_W 785.39816f // 7500 rpm in rad/s
#define IDLE_RPM 900.0f

// --- Big circle track + pursuit controller --------------------------------

#define R_TRACK 2000.0
#define N_SAMPLES 5027

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
	for ( int k = -8; k <= 40; ++k )
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

static void reset_car( void )
{
	sim_reset();
	s_nearest = 0;
	for ( int i = 0; i < 400; ++i ) // 1 s settle
	{
		sim_step( 0, 0, 0, 0 );
	}
}

// Drive with pursuit steering at the given pedals/flags for n ticks.
static void drive( int n, uint32_t throttle, uint32_t brake, uint32_t flags )
{
	const SimStateV1* s = sim_state();
	for ( int i = 0; i < n; ++i )
	{
		uint32_t st = sim_step( pursuit_steer( s ), throttle, brake, flags );
		if ( st & SIM_STATUS_ERROR )
		{
			fprintf( stderr, "FAIL: sim_step ERROR\n" );
			exit( 1 );
		}
	}
}

// Drive until the speed condition holds (dir > 0: speed >= v, else <= v).
static int drive_until_speed( float v, int dir, uint32_t throttle, uint32_t brake, int budget )
{
	const SimStateV1* s = sim_state();
	for ( int i = 0; i < budget; ++i )
	{
		if ( ( dir > 0 && s->speed >= v ) || ( dir < 0 && s->speed <= v ) )
		{
			return 1;
		}
		sim_step( pursuit_steer( s ), throttle, brake, 0 );
	}
	return 0;
}

// One momentary shift request (bit held for exactly one tick), then n-1 idle
// pursuit ticks at the given pedals.
static void request_shift( uint32_t bit, int n, uint32_t throttle, uint32_t brake )
{
	const SimStateV1* s = sim_state();
	sim_step( pursuit_steer( s ), throttle, brake, bit );
	for ( int i = 1; i < n; ++i )
	{
		sim_step( pursuit_steer( s ), throttle, brake, 0 );
	}
}

// --- 1. ratio table --------------------------------------------------------

static void test_ratios( void )
{
	float r[7];
	for ( int g = 0; g <= 6; ++g )
	{
		r[g] = vehicle_gear_ratio( g );
	}
	printf( "ratios: R=%.2f 1=%.2f 2=%.2f 3=%.2f 4=%.2f 5=%.2f 6=%.2f\n", (double)r[0], (double)r[1], (double)r[2],
			(double)r[3], (double)r[4], (double)r[5], (double)r[6] );

	int decreasing = 1;
	for ( int g = 1; g < 6; ++g )
	{
		if ( r[g + 1] >= r[g] )
		{
			decreasing = 0;
		}
	}
	CHECK( decreasing && r[6] > 0.0f, "forward ratios strictly decreasing, all positive" );
	CHECK( r[0] < 0.0f && fabsf( r[0] + r[1] ) < 0.05f * r[1], "reverse ratio ~ -1st" );
	CHECK( vehicle_gear_ratio( 7 ) == 0.0f && vehicle_gear_ratio( -1 ) == 0.0f, "out-of-range gear ratio = 0" );

	float v1 = LIMITER_W / r[1] * WHEEL_RADIUS; // 1st redline speed (m/s)
	float v6 = LIMITER_W / r[6] * WHEEL_RADIUS;
	printf( "redline speeds: 1st %.1f km/h, 6th %.1f km/h\n", (double)( v1 * 3.6f ), (double)( v6 * 3.6f ) );
	CHECK( v1 * 3.6f > 90.0f && v1 * 3.6f < 100.0f, "1st tops ~95 km/h at the limiter" );
	CHECK( v6 * 3.6f > 280.0f, "6th redline above the 270-280 km/h drag-limited top (power-limited, not gear-limited)" );
}

// --- 2+3. edge-triggered shifts + cut timing -------------------------------

static void test_shift_edges_and_timing( void )
{
	const SimStateV1* s = sim_state();
	reset_car();

	// Get moving in 1st, well below its redline.
	CHECK( drive_until_speed( 12.0f, 1, 55000, 0, 4000 ), "reaches 12 m/s in 1st" );
	CHECK( sim_gear() == 1, "still in 1st before any request" );

	// HOLD the up bit for 300 ticks: exactly one shift.
	for ( int i = 0; i < 300; ++i )
	{
		sim_step( pursuit_steer( s ), 39321, 0, SIM_FLAG_SHIFT_UP );
	}
	CHECK( sim_gear() == 2, "holding shift-up for 300 ticks = exactly ONE shift (edge-triggered)" );

	// Release (edge reset), then measure the upshift cut timing exactly.
	drive( 10, 39321, 0, 0 );
	int gear_before = sim_gear();
	sim_step( pursuit_steer( s ), 39321, 0, SIM_FLAG_SHIFT_UP ); // request tick (tick 0 of the cut)
	int ticks_in_old = 0;
	while ( sim_gear() == gear_before && ticks_in_old < 100 )
	{
		sim_step( pursuit_steer( s ), 39321, 0, 0 );
		ticks_in_old++;
	}
	printf( "upshift engage: gear %d -> %d after %d ticks in the old gear\n", gear_before, sim_gear(), ticks_in_old );
	CHECK( sim_gear() == gear_before + 1, "upshift engages the next gear" );
	CHECK( ticks_in_old == 28, "upshift cut lasts exactly 28 ticks (70 ms)" );

	// Downshift timing + auto-blip: speed up a little in 3rd, then coast and
	// request a downshift; the engine must snap UP to the wheel-matched rpm.
	CHECK( drive_until_speed( 18.0f, 1, 55000, 0, 4000 ), "reaches 18 m/s in 3rd" );
	drive( 40, 20000, 0, 0 );
	gear_before = sim_gear();
	CHECK( gear_before == 3, "in 3rd before the downshift" );
	sim_step( pursuit_steer( s ), 0, 0, SIM_FLAG_SHIFT_DOWN ); // request tick
	int down_ticks = 0;
	float rpm_before_engage = 0.0f;
	while ( sim_gear() == gear_before && down_ticks < 100 )
	{
		rpm_before_engage = sim_rpm();
		sim_step( pursuit_steer( s ), 0, 0, 0 );
		down_ticks++;
	}
	float rpm_after_engage = sim_rpm();
	printf( "downshift engage: gear %d -> %d after %d ticks, rpm %.0f -> %.0f (blip snap)\n", gear_before, sim_gear(),
			down_ticks, (double)rpm_before_engage, (double)rpm_after_engage );
	CHECK( sim_gear() == gear_before - 1, "downshift engages the next gear down" );
	CHECK( down_ticks == 48, "downshift delay lasts exactly 48 ticks (120 ms)" );
	// Wheel-matched prediction in the NEW gear: v/r * ratio (rpm), vs the
	// snapped engine speed one tick after engagement.
	float rpm_matched = s->speed / WHEEL_RADIUS * vehicle_gear_ratio( sim_gear() ) * 9.5492966f;
	printf( "wheel-matched rpm %.0f, snapped rpm %.0f\n", (double)rpm_matched, (double)rpm_after_engage );
	CHECK( rpm_after_engage > rpm_before_engage + 100.0f, "engage snaps the engine UP (auto-blip illusion)" );
	CHECK( fabsf( rpm_after_engage - rpm_matched ) < 0.05f * rpm_matched, "snap lands on the wheel-matched rpm (5%)" );
}

// --- 4. rev limiter ---------------------------------------------------------

static void test_limiter( void )
{
	const SimStateV1* s = sim_state();
	reset_car();

	float max_rpm = 0.0f;
	float max_speed = 0.0f;
	for ( int i = 0; i < 8000; ++i ) // 20 s of full throttle, stuck in 1st
	{
		sim_step( pursuit_steer( s ), 65535, 0, 0 );
		if ( sim_rpm() > max_rpm )
		{
			max_rpm = sim_rpm();
		}
		if ( s->speed > max_speed )
		{
			max_speed = s->speed;
		}
	}
	float v1_redline = LIMITER_W / vehicle_gear_ratio( 1 ) * WHEEL_RADIUS;
	printf( "limiter: max rpm %.0f, max speed %.2f m/s (1st redline %.2f m/s)\n", (double)max_rpm, (double)max_speed,
			(double)v1_redline );
	CHECK( max_rpm >= 7350.0f, "limiter actually reached in 1st" );
	CHECK( max_rpm <= 7650.0f, "hard fuel cut holds the engine at ~7500 rpm" );
	CHECK( fabsf( max_speed - v1_redline ) < 1.5f, "speed plateaus at the 1st-gear redline speed" );
}

// --- 5. engine braking ------------------------------------------------------

static float coast_decel( int gear_expect )
{
	const SimStateV1* s = sim_state();
	// 0.25 s to let the throttle-lift transient decay, then 1 s measurement.
	drive( 100, 0, 0, 0 );
	CHECK( sim_gear() == gear_expect, "coasting in the expected gear" );
	float v0 = s->speed;
	drive( 400, 0, 0, 0 );
	float v1 = s->speed;
	return ( v0 - v1 ) / 1.0f;
}

static void test_engine_braking( void )
{
	const SimStateV1* s = sim_state();
	reset_car();

	// Up to 3rd at ~21 m/s.
	CHECK( drive_until_speed( 12.0f, 1, 65535, 0, 4000 ), "launch for the coast test" );
	request_shift( SIM_FLAG_SHIFT_UP, 40, 55000, 0 ); // -> 2
	CHECK( drive_until_speed( 17.0f, 1, 55000, 0, 4000 ), "2nd leg" );
	request_shift( SIM_FLAG_SHIFT_UP, 40, 55000, 0 ); // -> 3
	CHECK( drive_until_speed( 21.0f, 1, 55000, 0, 4000 ), "3rd leg to 21 m/s" );

	float decel3 = coast_decel( 3 );
	float v_after3 = s->speed;
	printf( "closed-throttle decel in 3rd from ~20 m/s: %.3f m/s^2 (v now %.1f)\n", (double)decel3, (double)v_after3 );
	CHECK( decel3 > 0.4f && decel3 < 1.5f, "engine braking in 3rd decelerates 0.4-1.5 m/s^2 (sign + magnitude)" );

	// Same speed in 6th: crank drag scales with the ratio, so decel drops.
	request_shift( SIM_FLAG_SHIFT_UP, 60, 45000, 0 ); // -> 4
	request_shift( SIM_FLAG_SHIFT_UP, 60, 45000, 0 ); // -> 5
	request_shift( SIM_FLAG_SHIFT_UP, 60, 45000, 0 ); // -> 6
	CHECK( sim_gear() == 6, "shifted up to 6th for the comparison coast" );
	CHECK( drive_until_speed( 21.0f, 1, 65535, 0, 6000 ), "back to 21 m/s in 6th" );
	float decel6 = coast_decel( 6 );
	printf( "closed-throttle decel in 6th from ~20 m/s: %.3f m/s^2\n", (double)decel6 );
	CHECK( decel3 > decel6 + 0.15f, "engine braking is ratio-scaled: 3rd coasts much harder than 6th" );
}

// --- 6. LSD split + engine-brake curve (unit level) -------------------------

static void test_lsd_and_eb_units( void )
{
	float tl, tr;

	// Power, small speed difference: faster LEFT wheel sheds torque to the right.
	vehicle_lsd_split( 2000.0f, 100.0f, 90.0f, &tl, &tr );
	printf( "LSD power split (dw=10): L=%.0f R=%.0f\n", (double)tl, (double)tr );
	CHECK( tl < tr, "LSD moves torque from the faster to the slower wheel" );
	CHECK( fabsf( ( tl + tr ) - 2000.0f ) < 0.01f, "LSD conserves the axle torque" );

	// Clamp at 40% of |t_axle|.
	vehicle_lsd_split( 2000.0f, 200.0f, 100.0f, &tl, &tr );
	CHECK( fabsf( tl - ( 1000.0f - 800.0f ) ) < 0.01f && fabsf( tr - 1800.0f ) < 0.01f,
		   "LSD transfer clamps at 40%% of the transmitted torque" );

	// Coast (negative axle torque): same coupling, faster wheel dragged harder.
	vehicle_lsd_split( -300.0f, 50.0f, 40.0f, &tl, &tr );
	printf( "LSD coast split (dw=10): L=%.0f R=%.0f\n", (double)tl, (double)tr );
	CHECK( tl < tr, "LSD acts on coast too (engine braking splits through the coupling)" );
	CHECK( fabsf( ( tl + tr ) + 300.0f ) < 0.01f, "coast split conserves the axle torque" );

	// No transmitted torque (shift cut): clamp collapses, diff open.
	vehicle_lsd_split( 0.0f, 80.0f, 20.0f, &tl, &tr );
	CHECK( tl == 0.0f && tr == 0.0f, "zero transmitted torque = open diff (shift cut)" );

	// Symmetric wheels: pure 50/50.
	vehicle_lsd_split( 1000.0f, 70.0f, 70.0f, &tl, &tr );
	CHECK( tl == 500.0f && tr == 500.0f, "no speed difference = pure 50/50 split" );

	// Engine-brake curve endpoints and fade (docs/DRIVETRAIN.md §2).
	float idle_w = IDLE_RPM / 9.5492966f;
	float eb_idle = vehicle_engine_brake_torque( idle_w, 0.0f );
	float eb_red = vehicle_engine_brake_torque( LIMITER_W, 0.0f );
	float eb_half = vehicle_engine_brake_torque( LIMITER_W, 0.05f );
	float eb_off = vehicle_engine_brake_torque( LIMITER_W, 0.12f );
	printf( "engine braking: idle %.1f Nm, redline %.1f Nm, half-fade %.1f Nm, 12%% throttle %.1f Nm\n",
			(double)eb_idle, (double)eb_red, (double)eb_half, (double)eb_off );
	CHECK( fabsf( eb_idle + 20.0f ) < 0.5f, "engine braking -20 Nm at idle" );
	CHECK( fabsf( eb_red + 60.0f ) < 0.5f, "engine braking -60 Nm at redline" );
	CHECK( fabsf( eb_half + 30.0f ) < 0.5f, "engine braking fades linearly (half at 5%% throttle)" );
	CHECK( eb_off == 0.0f, "engine braking gone by 10%% throttle" );
}

// --- 7. reverse rule ---------------------------------------------------------

static void test_reverse_rule( void )
{
	const SimStateV1* s = sim_state();
	reset_car();

	CHECK( sim_gear() == 1, "spawns in 1st" );

	// At rest: 1 -> R engages (48-tick delay).
	request_shift( SIM_FLAG_SHIFT_DOWN, 60, 0, 0 );
	CHECK( sim_gear() == 0, "downshift from 1st at rest engages reverse" );

	// Reverse drive: the car moves backward.
	float fx, fz;
	for ( int i = 0; i < 600; ++i )
	{
		sim_step( 0, 26214, 0, 0 ); // 40% throttle, no steering
	}
	heading( s, &fx, &fz );
	float v_fwd = fx * s->lin_vel[0] + fz * s->lin_vel[2];
	printf( "reverse drive: v_fwd = %.2f m/s\n", (double)v_fwd );
	CHECK( v_fwd < -0.5f, "reverse gear drives the car backward" );

	// R -> 1 while still rolling backward: allowed at any speed.
	request_shift( SIM_FLAG_SHIFT_UP, 40, 0, 0 );
	CHECK( sim_gear() == 1, "upshift from R engages 1st while rolling backward (forward-safe)" );

	// Forward again, then a moving downshift request must be REFUSED.
	CHECK( drive_until_speed( 5.0f, 1, 55000, 0, 4000 ), "forward to 5 m/s in 1st" );
	request_shift( SIM_FLAG_SHIFT_DOWN, 100, 30000, 0 );
	CHECK( sim_gear() == 1, "downshift from 1st at 5 m/s is refused (|v| >= 1)" );
}

// --- 8. downshift over-rev protection ----------------------------------------

static void test_overrev_protection( void )
{
	const SimStateV1* s = sim_state();
	reset_car();

	// Up to 2nd, then to ~30 m/s (1st would spin ~8500 rpm: refused).
	CHECK( drive_until_speed( 15.0f, 1, 65535, 0, 4000 ), "launch" );
	request_shift( SIM_FLAG_SHIFT_UP, 40, 65535, 0 ); // -> 2
	CHECK( sim_gear() == 2, "in 2nd" );
	CHECK( drive_until_speed( 30.0f, 1, 65535, 0, 8000 ), "2nd to 30 m/s" );

	float rpm_pred_1st = s->speed / WHEEL_RADIUS * vehicle_gear_ratio( 1 ) * 9.5492966f;
	printf( "at %.1f m/s, predicted 1st-gear rpm = %.0f\n", (double)s->speed, (double)rpm_pred_1st );
	CHECK( rpm_pred_1st > 7800.0f, "test premise: 1st would over-rev here" );
	request_shift( SIM_FLAG_SHIFT_DOWN, 100, 30000, 0 );
	CHECK( sim_gear() == 2, "over-revving downshift is refused" );

	// Brake to 22 m/s (1st ~6250 rpm): accepted.
	CHECK( drive_until_speed( 22.0f, -1, 0, 40000, 4000 ), "brake to 22 m/s" );
	rpm_pred_1st = s->speed / WHEEL_RADIUS * vehicle_gear_ratio( 1 ) * 9.5492966f;
	CHECK( rpm_pred_1st < 7800.0f, "test premise: 1st is now safe" );
	request_shift( SIM_FLAG_SHIFT_DOWN, 100, 0, 0 );
	CHECK( sim_gear() == 1, "safe downshift is accepted" );
}

// --- 9. CROWN GATE: shift-heavy log, step==step==replay ----------------------

#define CROWN_MAX_TICKS 16000

typedef struct Rec
{
	int16_t steer;
	uint16_t throttle;
	uint16_t brake;
	uint16_t flags;
} Rec;

static Rec s_log[CROWN_MAX_TICKS];
static int s_log_len = 0;

static uint64_t state_hash( void )
{
	return ( (uint64_t)sim_state_hash_hi() << 32 ) | (uint64_t)sim_state_hash_lo();
}

// Closed-loop recording driver: launch, shift 1->6 at ~7200 rpm, brake with
// downshifts at ~3500 rpm to 2nd, stop, shuffle into reverse and back, pull
// away again. Every quantized input is recorded for the replay passes.
static void crown_record( int* saw_gear6, int* saw_reverse, float* min_rpm )
{
	const SimStateV1* s = sim_state();
	sim_reset();
	s_nearest = 0;
	s_log_len = 0;
	*saw_gear6 = 0;
	*saw_reverse = 0;
	*min_rpm = 1.0e9f;

	int phase = 0; // 0 settle, 1 launch+upshifts, 2 brake+downshifts, 3 to-rest,
				   // 4 engage R, 5 reverse leg, 6 back to 1st, 7 forward again
	int cooldown = 0;
	int phase_ticks = 0;

	for ( int t = 0; t < CROWN_MAX_TICKS; ++t )
	{
		int32_t steer = 0;
		uint32_t throttle = 0, brake = 0, flags = 0;
		int gear = sim_gear();
		float rpm = sim_rpm();
		if ( rpm < *min_rpm )
		{
			*min_rpm = rpm;
		}
		if ( cooldown > 0 )
		{
			cooldown--;
		}

		switch ( phase )
		{
			case 0: // settle 0.5 s
				if ( ++phase_ticks >= 200 )
				{
					phase = 1;
					phase_ticks = 0;
				}
				break;
			case 1: // launch, short-shift at 6000 rpm all the way to 6th
				steer = pursuit_steer( s );
				throttle = 65535;
				if ( gear < 6 && rpm > 6000.0f && cooldown == 0 )
				{
					flags |= SIM_FLAG_SHIFT_UP;
					cooldown = 40;
				}
				if ( gear == 6 )
				{
					*saw_gear6 = 1;
					phase = 2;
					phase_ticks = 0;
				}
				break;
			case 2: // hard brake, down whenever rpm < 3500, until 2nd and slow
				steer = pursuit_steer( s );
				brake = 60000;
				if ( gear > 2 && rpm < 3500.0f && cooldown == 0 )
				{
					flags |= SIM_FLAG_SHIFT_DOWN;
					cooldown = 60;
				}
				if ( gear == 2 && s->speed < 6.0f )
				{
					phase = 3;
					phase_ticks = 0;
				}
				break;
			case 3: // roll to a stop, drop into 1st
				brake = 30000;
				if ( gear == 2 && cooldown == 0 && s->speed < 4.0f )
				{
					flags |= SIM_FLAG_SHIFT_DOWN;
					cooldown = 60;
				}
				if ( gear == 1 && s->speed < 0.4f )
				{
					phase = 4;
					phase_ticks = 0;
				}
				break;
			case 4: // engage reverse at rest
				if ( cooldown == 0 )
				{
					flags |= SIM_FLAG_SHIFT_DOWN;
					cooldown = 60;
				}
				if ( gear == 0 )
				{
					*saw_reverse = 1;
					phase = 5;
					phase_ticks = 0;
				}
				break;
			case 5: // back up for 1.5 s
				throttle = 26214;
				if ( ++phase_ticks >= 600 )
				{
					phase = 6;
					phase_ticks = 0;
				}
				break;
			case 6: // back to 1st (upshift from R, any speed)
				if ( cooldown == 0 )
				{
					flags |= SIM_FLAG_SHIFT_UP;
					cooldown = 60;
				}
				if ( gear == 1 )
				{
					phase = 7;
					phase_ticks = 0;
				}
				break;
			case 7: // pull away again for 2 s, then hold to the end of the log
				steer = pursuit_steer( s );
				throttle = 52428;
				break;
		}

		s_log[s_log_len++] = ( Rec ){ (int16_t)steer, (uint16_t)throttle, (uint16_t)brake, (uint16_t)flags };
		uint32_t st = sim_step( steer, throttle, brake, flags );
		if ( st & SIM_STATUS_ERROR )
		{
			fprintf( stderr, "FAIL: ERROR during crown recording\n" );
			exit( 1 );
		}
	}
}

static void test_crown_gate( void )
{
	int saw6 = 0, sawR = 0;
	float min_rpm = 0.0f;

	// Pass 1: closed-loop recording (this IS the first sim_step run).
	crown_record( &saw6, &sawR, &min_rpm );
	uint64_t h1 = state_hash();
	printf( "crown pass 1 (record, sim_step): %d ticks, hash %016llx, saw6=%d sawR=%d min_rpm=%.0f\n", s_log_len,
			(unsigned long long)h1, saw6, sawR, (double)min_rpm );
	CHECK( saw6, "crown log reaches 6th gear" );
	CHECK( sawR, "crown log shuffles through reverse" );
	CHECK( min_rpm >= 899.0f, "engine never stalls (900 rpm idle floor)" );

	// Pass 2: identical inputs re-fed through sim_step from a fresh reset.
	sim_reset();
	for ( int t = 0; t < s_log_len; ++t )
	{
		sim_step( s_log[t].steer, s_log[t].throttle, s_log[t].brake, s_log[t].flags );
	}
	uint64_t h2 = state_hash();
	printf( "crown pass 2 (sim_step):   hash %016llx\n", (unsigned long long)h2 );

	// Pass 3: one sim_replay call on the packed 8-byte records.
	unsigned char* buf = (unsigned char*)malloc( (size_t)s_log_len * 8 );
	for ( int t = 0; t < s_log_len; ++t )
	{
		memcpy( buf + 8 * (size_t)t + 0, &s_log[t].steer, 2 );
		memcpy( buf + 8 * (size_t)t + 2, &s_log[t].throttle, 2 );
		memcpy( buf + 8 * (size_t)t + 4, &s_log[t].brake, 2 );
		memcpy( buf + 8 * (size_t)t + 6, &s_log[t].flags, 2 );
	}
	sim_reset();
	uint32_t st = sim_replay( (sim_ptr_t)buf, (uint32_t)s_log_len );
	CHECK( !( st & SIM_STATUS_ERROR ), "sim_replay of the crown log runs clean" );
	uint64_t h3 = state_hash();
	printf( "crown pass 3 (sim_replay): hash %016llx\n", (unsigned long long)h3 );
	free( buf );

	CHECK( h1 == h2, "crown gate: recording and sim_step playback hashes identical" );
	CHECK( h2 == h3, "crown gate: sim_step and sim_replay hashes identical" );
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

	test_ratios();
	test_shift_edges_and_timing();
	test_limiter();
	test_engine_braking();
	test_lsd_and_eb_units();
	test_reverse_rule();
	test_overrev_protection();
	test_crown_gate();

	free( blob );

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_drivetrain: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_drivetrain PASS\n" );
	return 0;
}

// harness_skidpad.c — docs/DRIVETRAIN.md §5/§7 gate 5: steady-state balance.
//
//   1. steady-state lateral grip on a R = 60 m skidpad lands in 1.05-1.10 g
//      (measured as v^2/r over a sustained window at the limit).
//   2. mild limit understeer: the FRONT axle runs the larger slip angle at
//      the limit (front slips first — stable).
//   3. rollover threshold still > 1.25 g: quasi-static tipping limit from
//      the measured CoM height (with the downforce term at the limit speed),
//      plus an empirical no-tip check (bounded roll, inside wheels loaded).
//
// The driver ramps a target speed slowly upward with an auto-shifter until
// the car can no longer hold the radius; the "limit" is the best 2 s window
// where the car still tracks the circle within 2.5 m of lateral error.

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

#define R_TRACK 60.0
#define N_SAMPLES 151
#define G 9.81f

// Mirrors of kTuning constants (deliberately static in src/vehicle.c).
#define TRACK_HALF 0.84f
#define WHEEL_RADIUS 0.33f
#define AERO_CL_TOTAL ( 1.1f + 1.4f )
#define MASS_TOTAL 1350.0f

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
	for ( int k = -5; k <= 20; ++k )
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
	int look = 3 + (int)( s->speed * 0.18f );
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

int main( void )
{
	size_t blob_len = 0;
	// Wide corridor (16 m) so working the limit doesn't leave the road model.
	uint8_t* blob = make_test_track_ex( &blob_len, (float)R_TRACK, (float)R_TRACK, N_SAMPLES, 16.0f, 30.0f );
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
	sim_reset();
	for ( int i = 0; i < 400; ++i )
	{
		sim_step( 0, 0, 0, 0 );
	}

	// CoM height above the road for the quasi-static rollover threshold:
	// chassis center height at settle plus the (negative) CoM offset.
	float settle_y = s->pos[1];
	b3MassData md = vehicle_mass_data();
	float h_com = settle_y + md.center.y; // road is at y ~ 0 (crown ~ -0.6 mm)
	printf( "settle: chassis y %.3f m, CoM offset %.4f -> h_com %.3f m\n", (double)settle_y, (double)md.center.y,
			(double)h_com );

	// --- skidpad protocol: 60 s warmup at ~0.75 g brings the tires to
	// operating temperature (the thermal layer moves grip ~10% cold->warm),
	// then a slow staged ramp probes the limit. ---
	int shift_cd = 0;
	float steer_scale = 1.0f;
	const int WIN = 800; // 2 s sustained-window length

	// Ring buffers over the measurement phase.
	enum { N_MEAS = 32000 }; // 80 s
	static float rb_alat[N_MEAS];
	static float rb_err[N_MEAS];
	static float rb_slip_f[N_MEAS];
	static float rb_slip_r[N_MEAS];
	static float rb_comp_in[N_MEAS];
	static float rb_roll[N_MEAS];

	int n = 0;
	const int WARM = 24000; // 60 s warmup
	for ( int t = 0; t < WARM + N_MEAS; ++t )
	{
		float target = t < WARM ? 21.0f : 23.0f + 0.25f * (float)( ( t - WARM ) / 800 );
		if ( shift_cd > 0 )
		{
			shift_cd--;
		}
		uint32_t flags = 0;
		if ( sim_gear() < 6 && sim_rpm() > 7200.0f && shift_cd == 0 )
		{
			flags = SIM_FLAG_SHIFT_UP;
			shift_cd = 40;
		}
		else if ( sim_gear() > 1 && sim_rpm() < 3000.0f && shift_cd == 0 )
		{
			flags = SIM_FLAG_SHIFT_DOWN;
			shift_cd = 60;
		}
		// Proportional throttle (bang-bang upsets the rear axle at the limit).
		float th = 0.45f + 0.6f * ( target - s->speed );
		th = th < 0.0f ? 0.0f : ( th > 0.85f ? 0.85f : th );
		uint32_t throttle = (uint32_t)( th * 65535.0f );

		// Slip-aware steering: a driver at the limit holds the front axle
		// near its peak slip angle (~6.6 deg) instead of plowing past it.
		float front_slip = 0.5f * ( fabsf( s->wheels[0].slip_angle ) + fabsf( s->wheels[1].slip_angle ) );
		if ( front_slip > 0.125f )
		{
			steer_scale -= 0.002f;
		}
		else if ( front_slip < 0.115f )
		{
			steer_scale += 0.002f;
		}
		steer_scale = steer_scale < 0.6f ? 0.6f : ( steer_scale > 1.0f ? 1.0f : steer_scale );
		int32_t steer = (int32_t)( (float)pursuit_steer( s ) * steer_scale );

		uint32_t st = sim_step( steer, throttle, 0, flags );
		if ( st & SIM_STATUS_ERROR )
		{
			fprintf( stderr, "sim ERROR at tick %d\n", t );
			return 1;
		}
		if ( t < WARM )
		{
			continue;
		}

		float r = sqrtf( s->pos[0] * s->pos[0] + s->pos[2] * s->pos[2] );
		rb_alat[n] = s->speed * s->speed / ( r > 1.0f ? r : 1.0f );
		rb_err[n] = fabsf( r - (float)R_TRACK );
		// Mean |slip angle| per axle (state export, kinematic).
		rb_slip_f[n] = front_slip;
		rb_slip_r[n] = 0.5f * ( fabsf( s->wheels[2].slip_angle ) + fabsf( s->wheels[3].slip_angle ) );
		// Inside wheels on this circle: the pair carrying less load — minimum.
		float ci = s->wheels[0].susp_compression < s->wheels[1].susp_compression ? s->wheels[0].susp_compression
																				 : s->wheels[1].susp_compression;
		float cir = s->wheels[2].susp_compression < s->wheels[3].susp_compression ? s->wheels[2].susp_compression
																				  : s->wheels[3].susp_compression;
		rb_comp_in[n] = ci < cir ? ci : cir;
		// Lean angle from the chassis up vector's world-Y component.
		float x = s->quat[0], z = s->quat[2];
		float uy = 1.0f - 2.0f * ( x * x + z * z ); // (R*(0,1,0)).y
		rb_roll[n] = acosf( uy > 1.0f ? 1.0f : ( uy < -1.0f ? -1.0f : uy ) );
		n++;
	}

	// Best sustained 2 s window: max mean a_lat with the car actually ON the
	// circle (mean lateral error < 2.5 m) and NOT plowing (mean front slip
	// below 9 deg — past that the "grip" number is a drift, not steady state).
	float best_alat = 0.0f;
	int best_i = -1;
	{
		// Prefix sums for O(n) windows.
		double sum_a = 0.0, sum_e = 0.0, sum_sf = 0.0;
		static double pa[N_MEAS + 1], pe[N_MEAS + 1], psf[N_MEAS + 1];
		pa[0] = pe[0] = psf[0] = 0.0;
		for ( int i = 0; i < n; ++i )
		{
			sum_a += rb_alat[i];
			sum_e += rb_err[i];
			sum_sf += rb_slip_f[i];
			pa[i + 1] = sum_a;
			pe[i + 1] = sum_e;
			psf[i + 1] = sum_sf;
		}
		for ( int i = 0; i + WIN <= n; ++i )
		{
			float mean_a = (float)( ( pa[i + WIN] - pa[i] ) / WIN );
			float mean_e = (float)( ( pe[i + WIN] - pe[i] ) / WIN );
			float mean_sf = (float)( ( psf[i + WIN] - psf[i] ) / WIN );
			if ( mean_e < 2.5f && mean_sf < 0.157f && mean_a > best_alat )
			{
				best_alat = mean_a;
				best_i = i;
			}
		}
	}
	if ( best_i < 0 )
	{
		fprintf( stderr, "no sustained window found\n" );
		return 1;
	}

	float mean_slip_f = 0.0f, mean_slip_r = 0.0f, min_comp = 1.0f, max_roll = 0.0f, mean_v = 0.0f;
	for ( int i = best_i; i < best_i + WIN; ++i )
	{
		mean_slip_f += rb_slip_f[i];
		mean_slip_r += rb_slip_r[i];
		if ( rb_comp_in[i] < min_comp )
		{
			min_comp = rb_comp_in[i];
		}
		if ( rb_roll[i] > max_roll )
		{
			max_roll = rb_roll[i];
		}
		mean_v += sqrtf( rb_alat[i] * (float)R_TRACK );
	}
	mean_slip_f /= (float)WIN;
	mean_slip_r /= (float)WIN;
	mean_v /= (float)WIN;

	printf( "skidpad limit window: a_lat %.3f m/s^2 = %.3f g at ~%.1f m/s\n", (double)best_alat,
			(double)( best_alat / G ), (double)mean_v );
	printf( "  slip angles: front %.2f deg, rear %.2f deg (delta %.2f)\n", (double)( mean_slip_f * 57.29578f ),
			(double)( mean_slip_r * 57.29578f ), (double)( ( mean_slip_f - mean_slip_r ) * 57.29578f ) );
	printf( "  min inside compression %.4f m, max roll %.2f deg\n", (double)min_comp,
			(double)( max_roll * 57.29578f ) );

	CHECK( best_alat / G >= 1.05f && best_alat / G <= 1.10f, "steady-state 1.05-1.10 g" );
	CHECK( mean_slip_f > mean_slip_r + 0.002f, "mild limit understeer: front axle runs the larger slip angle" );
	CHECK( mean_slip_f - mean_slip_r < 0.09f, "understeer stays MILD (< ~5 deg of push)" );

	// Quasi-static rollover threshold with the downforce term at limit speed:
	// a_roll = (g + F_down/m) * track_half / h_com. Downforce adds restoring
	// moment (applied at axle height ~ near the roll axis) without lateral
	// force, so it raises the tipping limit; grip (1.05-1.10 g) must stay
	// clearly below it.
	float f_down = AERO_CL_TOTAL * mean_v * mean_v;
	float a_roll = ( G + f_down / MASS_TOTAL ) * TRACK_HALF / h_com;
	float a_roll_static = G * TRACK_HALF / h_com;
	printf( "rollover threshold: %.3f g (with downforce at %.1f m/s; %.3f g static)\n", (double)( a_roll / G ),
			(double)mean_v, (double)( a_roll_static / G ) );
	CHECK( a_roll / G > 1.25f, "rollover threshold > 1.25 g" );
	CHECK( min_comp > 0.002f, "inside wheels stay loaded at the limit (slides, does not tip)" );
	CHECK( max_roll < 0.12f, "roll angle bounded at the limit (< ~7 deg)" );

	free( blob );

	if ( g_failures != 0 )
	{
		fprintf( stderr, "harness_skidpad: %d failures\n", g_failures );
		return 1;
	}
	printf( "harness_skidpad PASS\n" );
	return 0;
}

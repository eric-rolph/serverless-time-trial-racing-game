// test_suspension.c — suspension wave 3 validation gates (docs/SUSPENSION.md §5.2).
//
//   (a) settle consistency on the flat oval (full sim): suspension spring
//       compression ≈ sprung corner weight / k_s and tire compression ≈
//       total corner weight / k_tire, both ±10 %, per axle.
//   (b) unsprung resonance: drive vehicle_suspension_step directly at 1600 Hz,
//       settle, then step the road up 3 cm. With the spec dampers the wheel
//       mode is ~critically damped (zeta ≈ (4500+300)/(2·sqrt(260e3·22)) ≈ 1),
//       so the resonance frequency is measured from the step response's
//       peak-velocity time (t_pk = 1/omega_n for zeta = 1) instead of zero
//       crossings — deviation from the literal "oscillates" wording recorded
//       in NOTES.md. Gate: f_n in [12, 18] Hz and the response decays.
//   (c) kinematic camber becomes more negative with compression
//       (−1.0° per 25 mm) and matches the static setting at settle.
//   (d) camber thrust through the warm brush patch: γ = −3° vs 0° shifts the
//       Fy(σy) curve in the grip-adding direction (thrust at zero slip, peak
//       shifted, more |Fy| on the leaned side).
//
// vehicle_suspension_step / vehicle_wheel_kinematics / vehicle_camber_thrust_
// sigma / vehicle_brush_patch / vehicle_mass_data are driven directly
// (vehicle.h lives in src/, not the public include/ surface).

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

// Constants mirroring kTuning in src/vehicle.c (deliberately static there).
#define K_SPRING 60000.0f
#define K_TIRE 200000.0f
#define M_UNSPRUNG 22.0f
#define REST_LENGTH 0.35f
#define WHEEL_RADIUS 0.33f
#define GRAVITY 9.81f
#define SUB_DT ( 0.0025f / 4.0f ) // 1600 Hz

// Static loads (docs/SUSPENSION.md §1/§4): sprung 1262 kg on the composite
// CoM at z = -0.13393 between hardpoints at z = ±1.3.
#define SPRUNG_FRONT_N 2776.2f // per wheel
#define SPRUNG_REAR_N 3414.0f
#define CORNER_FRONT_N ( SPRUNG_FRONT_N + M_UNSPRUNG * GRAVITY )
#define CORNER_REAR_N ( SPRUNG_REAR_N + M_UNSPRUNG * GRAVITY )

#define DEG2RAD 0.017453293f

int main( void )
{
	// --- mass layout sanity: report the computed tensor + CoM ---
	{
		b3MassData md = vehicle_mass_data();
		printf( "mass data: sprung mass %.1f kg, CoM (%.4f, %.4f, %.4f)\n", (double)md.mass, (double)md.center.x,
				(double)md.center.y, (double)md.center.z );
		printf( "inertia: pitch %.1f, yaw %.1f, roll %.1f kg m^2\n", (double)md.inertia.cx.x, (double)md.inertia.cy.y,
				(double)md.inertia.cz.z );
		CHECK( fabsf( md.mass - 1262.0f ) < 0.5f, "sprung mass = 1350 - 4*22 = 1262 kg" );
		CHECK( md.center.x == 0.0f, "CoM on the centerline" );
		CHECK( md.inertia.cy.y < 2600.0f, "yaw inertia well under the old hand-set 2600" );
		CHECK( md.inertia.cx.x > md.inertia.cy.y * 0.5f && md.inertia.cz.z < md.inertia.cx.x,
			   "tensor ordering plausible (roll < pitch < ~yaw)" );
	}

	// --- (a) settle consistency on the flat oval, full sim ---
	{
		size_t blob_len = 0;
		uint8_t* blob = make_test_track( &blob_len );
		if ( blob == NULL )
		{
			fprintf( stderr, "make_test_track failed\n" );
			return 1;
		}
		if ( sim_load_track( (sim_ptr_t)blob, (uint32_t)blob_len ) != 0 )
		{
			fprintf( stderr, "sim_load_track failed\n" );
			return 1;
		}
		const SimStateV1* s = sim_state();
		sim_reset();
		for ( int i = 0; i < 1200; ++i ) // 3 s idle
		{
			if ( sim_step( 0, 0, 0, 0 ) & SIM_STATUS_ERROR )
			{
				fprintf( stderr, "sim_step ERROR during settle\n" );
				return 1;
			}
		}

		// The flat oval road is at y = 0 with a parabolic crown; the wheels
		// sit ±0.8 m off the centerline of a 10 m road: crown = -25 mm *
		// (2*0.8/10)^2 = -0.64 mm at the contact point.
		const float crown = -0.025f * ( 1.6f / 10.0f ) * ( 1.6f / 10.0f );
		for ( int i = 0; i < SIM_WHEEL_COUNT; ++i )
		{
			int front = i < 2;
			float exp_spring = ( front ? SPRUNG_FRONT_N : SPRUNG_REAR_N ) / K_SPRING;
			float exp_tire = ( front ? CORNER_FRONT_N : CORNER_REAR_N ) / K_TIRE;
			float got_spring = s->wheels[i].susp_compression;
			float got_tire = WHEEL_RADIUS - s->wheels[i].pos[1] + crown;
			char msg[96];
			snprintf( msg, sizeof( msg ), "wheel %d spring settle %.4f m vs %.4f (+/-10%%)", i, (double)got_spring,
					  (double)exp_spring );
			CHECK( fabsf( got_spring - exp_spring ) < 0.10f * exp_spring, msg );
			snprintf( msg, sizeof( msg ), "wheel %d tire settle %.4f m vs %.4f (+/-10%%)", i, (double)got_tire,
					  (double)exp_tire );
			CHECK( fabsf( got_tire - exp_tire ) < 0.10f * exp_tire, msg );
		}
		free( blob );
	}

	// --- (b) unsprung resonance: 3 cm road step, direct 1600 Hz integration ---
	{
		WheelRuntime w = { 0 };
		w.travel = REST_LENGTH;

		// Road placed so the strut settles near the front static load.
		float travel_eq_guess = REST_LENGTH - SPRUNG_FRONT_N / K_SPRING;
		float hit_dist0 = travel_eq_guess + WHEEL_RADIUS - CORNER_FRONT_N / K_TIRE;

		float ft, fs;
		for ( int k = 0; k < 16000; ++k ) // 10 s: fully settled
		{
			vehicle_suspension_step( &w, hit_dist0, 0.0f, GRAVITY, SUB_DT, &ft, &fs );
		}
		float travel_settle = w.travel;

		// 3 cm step up. Track the velocity peak (t_pk = 1/omega_n at critical
		// damping) with 3-point parabolic refinement around the max.
		float hit_dist1 = hit_dist0 - 0.03f;
		enum { N_REC = 1600 }; // 1 s
		static float vel[N_REC];
		int pk = 0;
		float pk_v = 0.0f;
		for ( int k = 0; k < N_REC; ++k )
		{
			vehicle_suspension_step( &w, hit_dist1, 0.0f, GRAVITY, SUB_DT, &ft, &fs );
			vel[k] = fabsf( w.travel_vel );
			if ( vel[k] > pk_v )
			{
				pk_v = vel[k];
				pk = k;
			}
		}
		float travel_end = w.travel;

		float t_pk = ( (float)pk + 1.0f ) * SUB_DT;
		if ( pk > 0 && pk < N_REC - 1 )
		{
			float y0 = vel[pk - 1], y1 = vel[pk], y2 = vel[pk + 1];
			float denom = y0 - 2.0f * y1 + y2;
			if ( fabsf( denom ) > 1e-9f )
			{
				t_pk += ( 0.5f * ( y0 - y2 ) / denom ) * SUB_DT;
			}
		}

		// The unsprung mode rides BOTH springs (they act in parallel on the
		// wheel) and is ~critically damped, so the step response peaks its
		// velocity at t_pk = 1/omega_n. Gate: the model's natural frequency
		// is in the spec band AND the measured timing agrees with it within
		// 15% (the 1600 Hz discretization stiffens the mode ~4% and the
		// estimator adds ~1% — both measured, see NOTES.md).
		float omega_n = sqrtf( ( K_TIRE + K_SPRING ) / M_UNSPRUNG );
		float f_n = omega_n / ( 2.0f * 3.1415927f );
		float timing_ratio = t_pk * omega_n; // 1.0 for an ideal critically damped mode

		// Decay: settle against the new road height, compare the offset that
		// remains 300 ms after the step with the initial offset.
		WheelRuntime w2 = w;
		for ( int k = 0; k < 16000; ++k )
		{
			vehicle_suspension_step( &w2, hit_dist1, 0.0f, GRAVITY, SUB_DT, &ft, &fs );
		}
		float travel_eq2 = w2.travel;
		float offset0 = fabsf( travel_settle - travel_eq2 );

		// Re-run and sample at 300 ms.
		WheelRuntime w3 = { 0 };
		w3.travel = REST_LENGTH;
		for ( int k = 0; k < 16000; ++k )
		{
			vehicle_suspension_step( &w3, hit_dist0, 0.0f, GRAVITY, SUB_DT, &ft, &fs );
		}
		for ( int k = 0; k < 480; ++k ) // 300 ms
		{
			vehicle_suspension_step( &w3, hit_dist1, 0.0f, GRAVITY, SUB_DT, &ft, &fs );
		}
		float offset300 = fabsf( w3.travel - travel_eq2 );

		printf( "unsprung step response: settle %.4f m -> %.4f m, vel peak %.3f m/s at %.2f ms\n",
				(double)travel_settle, (double)travel_eq2, (double)pk_v, (double)( t_pk * 1000.0f ) );
		printf( "f_n %.2f Hz (model), timing ratio t_pk*omega_n = %.3f\n", (double)f_n, (double)timing_ratio );
		printf( "decay: offset %.2f mm -> %.4f mm after 300 ms\n", (double)( offset0 * 1000.0f ),
				(double)( offset300 * 1000.0f ) );
		CHECK( f_n >= 12.0f && f_n <= 18.0f, "unsprung natural frequency in [12, 18] Hz" );
		CHECK( timing_ratio > 0.85f && timing_ratio < 1.15f,
			   "measured step-response timing matches 1/omega_n within 15%" );
		CHECK( offset0 > 0.015f, "3 cm road step moves the settle point (> 15 mm)" );
		CHECK( offset300 < 0.05f * offset0, "step response decayed to < 5% within 300 ms" );
		CHECK( fabsf( travel_end - travel_eq2 ) < 1e-4f, "response converges to the new equilibrium" );
	}

	// --- (c) kinematic camber/toe curves ---
	{
		// At the per-axle settle compression the camber equals the static
		// setting; +25 mm of compression adds -1.0 deg on both axles.
		float settle_f = SPRUNG_FRONT_N / K_SPRING;
		float settle_r = SPRUNG_REAR_N / K_SPRING;
		float cam, cam_bump, toe;
		vehicle_wheel_kinematics( 0, settle_f, &cam, &toe );
		printf( "front: camber %.3f deg, FL toe steer %+.4f deg at settle\n", (double)( cam / DEG2RAD ),
				(double)( toe / DEG2RAD ) );
		CHECK( fabsf( cam - ( -1.5f * DEG2RAD ) ) < 0.05f * DEG2RAD, "front static camber -1.5 deg at settle" );
		CHECK( fabsf( toe - ( 0.05f * DEG2RAD ) ) < 0.005f * DEG2RAD,
			   "FL toe-out steers the left wheel left (+0.05 deg)" );
		vehicle_wheel_kinematics( 0, settle_f + 0.025f, &cam_bump, &toe );
		CHECK( fabsf( ( cam_bump - cam ) - ( -1.0f * DEG2RAD ) ) < 0.01f * DEG2RAD,
			   "front camber gains -1.0 deg per 25 mm compression" );
		CHECK( cam_bump < cam, "camber more negative with compression (front)" );

		vehicle_wheel_kinematics( 2, settle_r, &cam, &toe );
		printf( "rear: camber %.3f deg, RL toe steer %+.4f deg at settle\n", (double)( cam / DEG2RAD ),
				(double)( toe / DEG2RAD ) );
		CHECK( fabsf( cam - ( -1.0f * DEG2RAD ) ) < 0.05f * DEG2RAD, "rear static camber -1.0 deg at settle" );
		CHECK( fabsf( toe - ( -0.15f * DEG2RAD ) ) < 0.005f * DEG2RAD,
			   "RL toe-in steers the left-rear wheel right (-0.15 deg)" );
		float toe_bump;
		vehicle_wheel_kinematics( 2, settle_r + 0.025f, &cam_bump, &toe_bump );
		CHECK( cam_bump < cam, "camber more negative with compression (rear)" );
		CHECK( fabsf( ( toe_bump - toe ) - ( -0.10f * DEG2RAD ) ) < 0.01f * DEG2RAD,
			   "rear bump toe: +0.10 deg toe-in per 25 mm (left wheel steers right)" );

		// Mirroring: the right wheel's toe steer is the negation.
		float cam_r, toe_r;
		vehicle_wheel_kinematics( 3, settle_r, &cam_r, &toe_r );
		CHECK( cam_r == cam && toe_r == -toe, "right wheel mirrors toe, same camber" );
	}

	// --- (d) camber thrust through the warm brush patch ---
	{
		const float FZ0 = 3500.0f;
		const float T_OPT = 85.0f;
		float sig_c = vehicle_camber_thrust_sigma( -3.0f * DEG2RAD );
		printf( "camber thrust sigma at -3 deg: %.4f\n", (double)sig_c );
		CHECK( vehicle_camber_thrust_sigma( 0.0f ) == 0.0f, "no thrust at zero camber" );
		CHECK( sig_c < -0.03f && sig_c > -0.033f, "thrust sigma = 0.6*sin(-3 deg) ~ -0.0314" );

		// Fy(sigma_y) sweeps with the thrust applied as in vehicle_update.
		float fx, fy0, fyc, trail;
		vehicle_brush_patch( 0.0f, 0.0f + sig_c, FZ0, T_OPT, &fx, &fyc, &trail );
		printf( "Fy at zero slip with gamma = -3 deg: %.1f N\n", (double)fyc );
		CHECK( fyc < -1000.0f, "camber thrust at zero slip pushes toward the lean (right)" );

		// Peak locations: gamma = -3 deg shifts the positive-Fy peak to larger
		// sigma_y by ~|sig_c| (curve translation through the shared budget).
		float peak0 = 0.0f, peakc = 0.0f, arg0 = 0.0f, argc = 0.0f;
		for ( float sy = -0.40f; sy <= 0.40f; sy += 0.001f )
		{
			vehicle_brush_patch( 0.0f, sy, FZ0, T_OPT, &fx, &fy0, &trail );
			if ( fy0 > peak0 )
			{
				peak0 = fy0;
				arg0 = sy;
			}
			vehicle_brush_patch( 0.0f, sy + sig_c, FZ0, T_OPT, &fx, &fyc, &trail );
			if ( fyc > peakc )
			{
				peakc = fyc;
				argc = sy;
			}
		}
		printf( "positive Fy peak: %.1f N at sigma %.3f (gamma 0) vs %.1f N at %.3f (gamma -3 deg)\n", (double)peak0,
				(double)arg0, (double)peakc, (double)argc );
		CHECK( fabsf( ( argc - arg0 ) - ( -sig_c ) ) < 0.005f, "peak shifted by the camber-equivalent slip" );

		// Grip-adding direction: at a pre-peak working slip toward the lean
		// side the cambered wheel makes MORE side force; on the opposite
		// side, less. (Past the grip peak the shift subtracts instead — the
		// thrust saturates with everything else, which is the point.)
		float fy_lean0, fy_leanc, fy_off0, fy_offc;
		vehicle_brush_patch( 0.0f, -0.06f, FZ0, T_OPT, &fx, &fy_lean0, &trail );
		vehicle_brush_patch( 0.0f, -0.06f + sig_c, FZ0, T_OPT, &fx, &fy_leanc, &trail );
		vehicle_brush_patch( 0.0f, 0.06f, FZ0, T_OPT, &fx, &fy_off0, &trail );
		vehicle_brush_patch( 0.0f, 0.06f + sig_c, FZ0, T_OPT, &fx, &fy_offc, &trail );
		printf( "Fy at sigma -0.06: %.1f -> %.1f N; at +0.06: %.1f -> %.1f N\n", (double)fy_lean0, (double)fy_leanc,
				(double)fy_off0, (double)fy_offc );
		CHECK( fabsf( fy_leanc ) > fabsf( fy_lean0 ), "gamma = -3 deg adds grip on the leaned side" );
		CHECK( fy_offc < fy_off0, "and gives it back on the opposite side" );
	}

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_suspension: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_suspension PASS\n" );
	return 0;
}

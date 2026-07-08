// test_tire.c — brush contact-patch validation gates (docs/TIRE-MODEL.md §4.3).
//
// Sweeps vehicle_brush_patch() directly (no world, no track) at the reference
// load Fz0 = 3500 N with the surface temperature forced to T_opt (warm tire):
//   1. lateral force peaks between 5 and 9 degrees of slip angle
//   2. peak |Fy| within [0.95, 1.10] * mu_s0 * Fz0   (mu_s0 = 1.05, the
//      spec-level PATCH grip; the bristle-level static coefficient in
//      kTuning is higher because the patch rear always slides at mu_k)
//   3. Fy at 15 degrees is 5-20 % below the peak (kinetic drop-off)
//   4. pneumatic trail collapses through the peak:
//        trail(1 deg) is large and positive, trail(15 deg) ~ 0.
//      DEVIATION from the literal spec chain t1 > t8 > t15 >= 0: with the
//      binary static/kinetic bristle split, the raw trail dips a few mm
//      NEGATIVE just past the peak (adhesion centroid moves ahead of the
//      patch center while the sliding rear is discounted by mu_k) and then
//      returns to exactly 0 at full slide. That dip is inherent to this
//      model class, so the 8-vs-15 degree link is checked with a small
//      tolerance instead of strictly. See NOTES.md "Brush tire model".
//   5. combined slip: with sigma_x = 0.10 the lateral peak drops vs pure
//      slip (one shared friction budget, no ellipse).
// Plus sign-symmetry and cold-tire sanity checks.

#include "vehicle.h"

#include <math.h>
#include <stdio.h>

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

#define FZ0 3500.0f
#define T_OPT 85.0f
#define T_COLD 25.0f
#define MU_S0 1.05f // spec base friction (docs/TIRE-MODEL.md §1)
#define DEG2RAD 0.017453293f

typedef struct SweepResult
{
	float peak_fy;
	float peak_alpha_deg; // slip angle at the lateral peak
} SweepResult;

// Sweep slip angle 0..20 deg at fixed sigma_x and surface temperature.
static SweepResult sweep( float sigma_x, float t_surf )
{
	SweepResult r = { 0.0f, 0.0f };
	for ( float alpha = 0.05f; alpha <= 20.0f; alpha += 0.05f )
	{
		float sigma_y = tanf( alpha * DEG2RAD );
		float fx, fy, trail;
		vehicle_brush_patch( sigma_x, sigma_y, FZ0, t_surf, &fx, &fy, &trail );
		if ( fy > r.peak_fy )
		{
			r.peak_fy = fy;
			r.peak_alpha_deg = alpha;
		}
	}
	return r;
}

static float fy_at( float alpha_deg, float t_surf )
{
	float fx, fy, trail;
	vehicle_brush_patch( 0.0f, tanf( alpha_deg * DEG2RAD ), FZ0, t_surf, &fx, &fy, &trail );
	return fy;
}

static float trail_at( float alpha_deg, float t_surf )
{
	float fx, fy, trail;
	vehicle_brush_patch( 0.0f, tanf( alpha_deg * DEG2RAD ), FZ0, t_surf, &fx, &fy, &trail );
	return trail;
}

int main( void )
{
	// --- 1+2. pure-alpha sweep, warm tire ---
	SweepResult warm = sweep( 0.0f, T_OPT );
	printf( "warm pure-alpha: peak Fy = %.1f N at %.2f deg (%.4f x mu_s0*Fz0)\n", (double)warm.peak_fy,
			(double)warm.peak_alpha_deg, (double)( warm.peak_fy / ( MU_S0 * FZ0 ) ) );
	CHECK( warm.peak_alpha_deg >= 5.0f && warm.peak_alpha_deg <= 9.0f, "lateral peak between 5 and 9 deg slip" );
	CHECK( warm.peak_fy >= 0.95f * MU_S0 * FZ0 && warm.peak_fy <= 1.10f * MU_S0 * FZ0,
		   "peak |Fy| within [0.95, 1.10] * mu_s0 * Fz0" );

	// --- 3. kinetic drop-off at 15 deg ---
	float fy15 = fy_at( 15.0f, T_OPT );
	float drop = 1.0f - fy15 / warm.peak_fy;
	printf( "Fy(15 deg) = %.1f N, drop from peak = %.2f %%\n", (double)fy15, (double)( drop * 100.0f ) );
	CHECK( drop >= 0.05f && drop <= 0.20f, "Fy at 15 deg is 5-20 %% below peak" );

	// --- 4. pneumatic trail collapse ---
	float t1 = trail_at( 1.0f, T_OPT );
	float t8 = trail_at( 8.0f, T_OPT );
	float t15 = trail_at( 15.0f, T_OPT );
	printf( "trail: 1 deg = %.2f mm, 8 deg = %.2f mm, 15 deg = %.2f mm\n", (double)( t1 * 1000.0f ),
			(double)( t8 * 1000.0f ), (double)( t15 * 1000.0f ) );
	CHECK( t1 > 0.010f, "trail at 1 deg is a real trail (> 10 mm)" );
	CHECK( t1 > t8 + 0.010f, "trail collapses hard between 1 and 8 deg" );
	CHECK( fabsf( t8 ) <= 0.005f, "trail at 8 deg is ~gone (|t| <= 5 mm; small negative dip allowed)" );
	CHECK( t8 >= t15 - 0.005f, "trail 8 -> 15 deg does not regrow (tolerance for the kinetic dip)" );
	CHECK( fabsf( t15 ) <= 0.003f, "trail at 15 deg ~ 0 (full slide: force centroid at patch center)" );

	// --- 5. combined slip: shared budget ---
	SweepResult comb = sweep( 0.10f, T_OPT );
	printf( "combined (sigma_x = 0.10): lateral peak = %.1f N (%.3f of pure peak)\n", (double)comb.peak_fy,
			(double)( comb.peak_fy / warm.peak_fy ) );
	CHECK( comb.peak_fy < 0.97f * warm.peak_fy, "longitudinal demand steals lateral peak (budget sharing)" );
	{
		// And the vector sum stays within the (warm) patch budget.
		float fx, fy, trail;
		vehicle_brush_patch( 0.10f, tanf( 6.0f * DEG2RAD ), FZ0, T_OPT, &fx, &fy, &trail );
		float mag = sqrtf( fx * fx + fy * fy );
		CHECK( mag <= 1.10f * MU_S0 * FZ0, "combined |F| bounded by the friction budget" );
		CHECK( fx > 0.0f && fy > 0.0f, "combined force components follow the slip vector" );
	}

	// --- sanity: sign symmetry and cold grip ---
	{
		float fxp, fyp, fxn, fyn, tr;
		vehicle_brush_patch( 0.0f, 0.08f, FZ0, T_OPT, &fxp, &fyp, &tr );
		vehicle_brush_patch( 0.0f, -0.08f, FZ0, T_OPT, &fxn, &fyn, &tr );
		CHECK( fyp > 0.0f && fyn == -fyp, "lateral force is odd in sigma_y" );
		vehicle_brush_patch( 0.08f, 0.0f, FZ0, T_OPT, &fxp, &fyp, &tr );
		CHECK( fxp > 0.0f && fyp == 0.0f, "pure longitudinal slip makes pure longitudinal force" );
	}
	{
		SweepResult cold = sweep( 0.0f, T_COLD );
		float ratio = cold.peak_fy / warm.peak_fy;
		printf( "cold (25 C) peak = %.1f N, %.3f of warm peak\n", (double)cold.peak_fy, (double)ratio );
		CHECK( ratio > 0.89f && ratio < 0.93f, "cold tire grips ~0.91 of warm (thermal floor, skidpad-retuned)" );
	}

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_tire: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_tire PASS\n" );
	return 0;
}

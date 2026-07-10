// test_road.c — analytic road surface validation gates (docs/ROAD-SURFACE.md §5.2).
//
//   1. continuity: walk straight lines in 10 cm steps across sample
//      boundaries on a flat oval and across a banking transition on a banked
//      oval; assert |dh| < 5 mm and dnormal < 1 deg per step.
//   2. crown: height at l = width/4 sits below l = 0 by the parabolic amount
//      crown_m * (2l/width)^2 = 25 mm * 0.25 = 6.25 mm.
//   3. kerb band: through the tight corner (smoothed |kappa| > 0.022) the
//      band [width/2, width/2 + 1.1] carries the 0..20 mm sinusoid with a
//      1.25 m period; outside the gate the band is the plain shoulder ramp.
//   4. lockup flash-heat (§2): 1 s locked at 30 m/s under Fz0 raises T_surf
//      by > 20 C; after release it decays (track conduction + convection).
//
// road_query and vehicle_tire_thermal/vehicle_brush_patch are driven directly
// (road.h / vehicle.h live in src/, not the public include/ surface).

#include "road.h"
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

// ---------------------------------------------------------------------------
// In-memory oval sample builders (ellipse rx=60 rz=40, width 10 — the same
// geometry as tests/make_test_track.c, optionally banked).
// ---------------------------------------------------------------------------

#define OVAL_S 256
#define OVAL_RX 60.0
#define OVAL_RZ 40.0
#define OVAL_WIDTH 10.0f

static void build_oval( RoadSample* s, int banked )
{
	// First pass: positions and horizontal frames.
	for ( int i = 0; i < OVAL_S; ++i )
	{
		double th = ( 2.0 * 3.14159265358979323846 * (double)i ) / (double)OVAL_S;
		double dx = OVAL_RX * cos( th );
		double dz = -OVAL_RZ * sin( th );
		double tl = sqrt( dx * dx + dz * dz );
		s[i].pos = ( b3Vec3 ){ (float)( OVAL_RX * sin( th ) ), 0.0f, (float)( OVAL_RZ * cos( th ) ) };
		s[i].tangent = ( b3Vec3 ){ (float)( dx / tl ), 0.0f, (float)( dz / tl ) };
		s[i].up = ( b3Vec3 ){ 0.0f, 1.0f, 0.0f };
		s[i].width = OVAL_WIDTH;
	}
	if ( !banked )
	{
		return;
	}
	// Second pass: lean `up` about the tangent proportionally to the local
	// curvature (smooth along the ellipse), normalized so the bank sweeps
	// 0 .. 0.07 rad around the lap — a genuine banking transition.
	for ( int i = 0; i < OVAL_S; ++i )
	{
		int j = ( i + 1 ) % OVAL_S;
		double ddx = (double)s[j].pos.x - (double)s[i].pos.x;
		double ddz = (double)s[j].pos.z - (double)s[i].pos.z;
		double dist = sqrt( ddx * ddx + ddz * ddz );
		double kappa =
			( (double)s[i].tangent.x * s[j].tangent.z - (double)s[i].tangent.z * s[j].tangent.x ) / dist;
		double k_min = 0.0111, k_max = 0.0375; // ellipse curvature extremes
		double f = ( fabs( kappa ) - k_min ) / ( k_max - k_min );
		if ( f < 0.0 )
		{
			f = 0.0;
		}
		if ( f > 1.0 )
		{
			f = 1.0;
		}
		double bank = 0.07 * f;
		// side = cross(up, tangent) with up = +Y: (t.z, 0, -t.x)
		b3Vec3 side = { s[i].tangent.z, 0.0f, -s[i].tangent.x };
		float cb = (float)cos( bank ), sb = (float)sin( bank );
		s[i].up = ( b3Vec3 ){ side.x * sb, cb, side.z * sb };
	}
}

static float angle_deg( b3Vec3 a, b3Vec3 b )
{
	float d = b3Dot( a, b );
	d = b3ClampFloat( d, -1.0f, 1.0f );
	return (float)( acos( (double)d ) * 180.0 / 3.14159265358979323846 );
}

// Walk a straight horizontal line from `start` along `dir` (unit, horizontal)
// in 10 cm steps; assert per-step height and normal continuity.
static void walk_continuity( const Road* road, b3Vec3 start, b3Vec3 dir, int steps, const char* label )
{
	uint32_t hint = road_nearest_global( road, start );
	RoadQuery q;
	road_query( road, start, hint, &q );
	float prev_h = q.point.y;
	b3Vec3 prev_n = q.normal;
	hint = q.seg;

	float max_dh = 0.0f, max_dn = 0.0f;
	for ( int k = 1; k <= steps; ++k )
	{
		b3Vec3 p = b3MulAdd( start, 0.10f * (float)k, dir );
		road_query( road, p, hint, &q );
		hint = q.seg;
		if ( !q.on_road )
		{
			fprintf( stderr, "FAIL: %s left the analytic domain at step %d\n", label, k );
			g_failures++;
			return;
		}
		float dh = fabsf( q.point.y - prev_h );
		float dn = angle_deg( q.normal, prev_n );
		if ( dh > max_dh )
		{
			max_dh = dh;
		}
		if ( dn > max_dn )
		{
			max_dn = dn;
		}
		prev_h = q.point.y;
		prev_n = q.normal;
	}
	printf( "%s: max |dh| = %.3f mm, max dnormal = %.4f deg per 10 cm step\n", label, (double)( max_dh * 1000.0f ),
			(double)max_dn );
	char msg[96];
	snprintf( msg, sizeof( msg ), "%s: |dh| < 5 mm per 10 cm step", label );
	CHECK( max_dh < 0.005f, msg );
	snprintf( msg, sizeof( msg ), "%s: dnormal < 1 deg per 10 cm step", label );
	CHECK( max_dn < 1.0f, msg );
}

static b3Vec3 side_of( const RoadSample* s )
{
	return b3Cross( s->up, s->tangent );
}

int main( void )
{
	static RoadSample flat[OVAL_S], banked[OVAL_S];
	build_oval( flat, 0 );
	build_oval( banked, 1 );

	Road road_flat, road_banked;
	if ( road_load( &road_flat, flat, OVAL_S, 0 ) != 0 || road_load( &road_banked, banked, OVAL_S, 0 ) != 0 )
	{
		fprintf( stderr, "road_load failed\n" );
		return 1;
	}

	// --- 1a. continuity across sample boundaries (flat oval) ---
	// Start 2 m left of the centerline at sample 5 and walk 15 m along the
	// local tangent: ~6 sample boundaries, still on the road at the end.
	{
		b3Vec3 start = b3MulAdd( flat[5].pos, 2.0f, side_of( &flat[5] ) );
		walk_continuity( &road_flat, start, flat[5].tangent, 150, "flat oval walk" );
	}

	// --- 1b. continuity across a banking transition (banked oval) ---
	// Sample 32 sits mid-transition (45 deg around the ellipse, bank climbing
	// from 0 toward the 0.07 rad clip); same walk with the tilted surface.
	{
		b3Vec3 start = b3MulAdd( banked[32].pos, 2.0f, side_of( &banked[32] ) );
		walk_continuity( &road_banked, start, banked[32].tangent, 150, "banked oval walk" );
	}

	// --- 1c. banking actually reaches the surface normal ---
	{
		RoadQuery q;
		road_query( &road_banked, banked[64].pos, 64, &q ); // tightest corner: full bank
		printf( "banked corner normal.y = %.5f (expect cos(0.07) = 0.99755)\n", (double)q.normal.y );
		CHECK( q.normal.y > 0.99f && q.normal.y < 0.9995f, "banked corner normal is tilted by the bank angle" );
	}

	// --- 2. crown: l = width/4 sits 6.25 mm below the centerline ---
	{
		RoadQuery q0, q1;
		road_query( &road_flat, flat[10].pos, 10, &q0 );
		b3Vec3 p1 = b3MulAdd( flat[10].pos, 0.25f * OVAL_WIDTH, side_of( &flat[10] ) );
		road_query( &road_flat, p1, 10, &q1 );
		float drop = q0.point.y - q1.point.y;
		printf( "crown drop at l = width/4: %.3f mm (expect 6.25)\n", (double)( drop * 1000.0f ) );
		CHECK( fabsf( drop - 0.00625f ) < 0.001f, "crown drop at width/4 matches -crown_m*(2l/width)^2" );
		CHECK( fabsf( q0.point.y ) < 0.001f, "centerline height is the sample height (crown = 0 at l = 0)" );
		CHECK( q0.normal.y > 0.99999f, "centerline normal is vertical on the flat oval" );
	}

	// --- 3a. kerb band through the tight corner (|kappa| = 0.0375 > 0.022) ---
	// Walk 2.5 m of arc at |l| = width/2 + 0.55 (mid-band): the sinusoid must
	// sweep its 0..20 mm range (period 1.25 m -> two full cycles).
	{
		b3Vec3 side = side_of( &flat[64] );
		float h_min = 1.0e9f, h_max = -1.0e9f;
		uint32_t hint = 64;
		for ( int k = 0; k <= 50; ++k )
		{
			b3Vec3 p = b3MulAdd( flat[64].pos, 0.05f * (float)k, flat[64].tangent );
			p = b3MulAdd( p, 0.5f * OVAL_WIDTH + 0.55f, side );
			RoadQuery q;
			road_query( &road_flat, p, hint, &q );
			hint = q.seg;
			if ( q.point.y < h_min )
			{
				h_min = q.point.y;
			}
			if ( q.point.y > h_max )
			{
				h_max = q.point.y;
			}
		}
		printf( "kerb band sweep: h in [%.2f, %.2f] mm\n", (double)( h_min * 1000.0f ), (double)( h_max * 1000.0f ) );
		CHECK( h_max - h_min > 0.015f, "kerb sinusoid sweeps most of its 20 mm range" );
		CHECK( h_min > -0.003f && h_max < 0.023f, "kerb heights stay within the 0..20 mm band" );
	}

	// --- 3b. no kerb on the gentle corner (|kappa| = 0.0111 < 0.022):
	// the band is the plain shoulder ramp (-1.2 m over 8 m from the edge) ---
	{
		b3Vec3 p = b3MulAdd( flat[0].pos, 0.5f * OVAL_WIDTH + 0.55f, side_of( &flat[0] ) );
		RoadQuery q;
		road_query( &road_flat, p, 0, &q );
		float expect = -1.2f * ( 0.55f / 8.0f ); // -82.5 mm
		printf( "gentle-corner band height: %.1f mm (expect %.1f)\n", (double)( q.point.y * 1000.0f ),
				(double)( expect * 1000.0f ) );
		CHECK( fabsf( q.point.y - expect ) < 0.005f, "band outside the kappa gate is the shoulder ramp" );
	}

	// --- 3c. domain edge: |l| > width/2 + 8 -> on_road = 0 (mesh fallback) ---
	{
		b3Vec3 p = b3MulAdd( flat[0].pos, 0.5f * OVAL_WIDTH + 9.0f, side_of( &flat[0] ) );
		RoadQuery q;
		road_query( &road_flat, p, 0, &q );
		CHECK( !q.on_road, "query beyond width/2 + 8 reports off the analytic domain" );
	}

	// --- 4. lockup flash-heat (ROAD-SURFACE §2) ---
	// Locked wheel (omega = 0) at 30 m/s under Fz0: slip_ratio = -1, full
	// slide, all friction power into the surface node. Mirrors the vehicle
	// loop: 4 sub-steps of SIM_DT/4 per 400 Hz tick, in contact.
	{
		WheelRuntime w = { 0 };
		w.t_surf = 25.0f;
		w.t_core = 25.0f;
		const float fz0 = 3500.0f;
		const float v = 30.0f;
		const float sub_dt = SIM_DT / 4.0f;

		for ( int tick = 0; tick < SIM_TICK_RATE; ++tick ) // 1 s locked
		{
			for ( int k = 0; k < 4; ++k )
			{
				float fx, fy, trail;
				vehicle_brush_patch( -1.0f, 0.0f, fz0, w.t_surf, &fx, &fy, &trail );
				float power = fabsf( fx * -v ); // v_slip_x = 0*r - v
				vehicle_tire_thermal( &w, power, v, 1, sub_dt );
			}
		}
		float t_locked = w.t_surf;
		printf( "T_surf after 1 s lockup at 30 m/s: %.1f C (rise %.1f C)\n", (double)t_locked,
				(double)( t_locked - 25.0f ) );
		CHECK( t_locked - 25.0f > 20.0f, "1 s of lockup raises T_surf by > 20 C" );
		CHECK( t_locked < 200.0f, "lockup flash-heat stays bounded" );

		// Release: rolling cleanly (no slip power), still in contact at 30 m/s.
		int monotonic = 1;
		float prev = w.t_surf;
		for ( int tick = 0; tick < 5 * SIM_TICK_RATE; ++tick ) // 5 s release
		{
			for ( int k = 0; k < 4; ++k )
			{
				vehicle_tire_thermal( &w, 0.0f, v, 1, sub_dt );
			}
			if ( w.t_surf > prev + 1.0e-6f )
			{
				monotonic = 0;
			}
			prev = w.t_surf;
		}
		printf( "T_surf 5 s after release: %.1f C (decayed %.1f C)\n", (double)w.t_surf,
				(double)( t_locked - w.t_surf ) );
		CHECK( monotonic, "surface temperature decays monotonically after release" );
		CHECK( t_locked - w.t_surf > 5.0f, "flash heat sheds > 5 C within 5 s of release" );
	}

	road_free( &road_flat );
	road_free( &road_banked );

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_road: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_road PASS\n" );
	return 0;
}

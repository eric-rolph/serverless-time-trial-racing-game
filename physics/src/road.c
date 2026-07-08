// road.c — analytic C1 road surface (docs/ROAD-SURFACE.md §1).
//
// Longitudinal: uniform Catmull-Rom through the centerline pos[i] (closed
// loop), parameter found by projecting onto the polyline near the last-known
// index (±6 window) and refining with 2 fixed Newton steps on the CR curve.
// Frames: up/tangent interpolated with the same CR weights, re-orthonormalized.
// Lateral profile: parabolic crown on the road proper, sinusoidal kerb band
// where the smoothed |κ| exceeds the trackgen gate, linear shoulder ramp
// matching the mesh outside. Normal = normalize(up − (∂h/∂l)·side) —
// continuous camber through banking transitions, which is the point.
//
// Determinism: fixed iteration counts, mul/add/div + sqrtf + b3ComputeCosSin
// only (the kerb sinusoid reduces to sin(4πu) because the 2.5 m sample
// spacing is exactly two 1.25 m rumble periods, so the integer sample index
// contributes whole periods — no floor/fmod needed).

#include "road.h"

#include <stdlib.h>

// ---------------------------------------------------------------------------
// Load / free — precompute the smoothed curvature array (spec §1: "kappa from
// tangents, window 15 — mirror trackgen").
// ---------------------------------------------------------------------------

int road_load( Road* r, const RoadSample* samples, uint32_t count )
{
	r->samples = samples;
	r->count = count;
	r->kappa = (float*)malloc( sizeof( float ) * count );
	float* raw = (float*)malloc( sizeof( float ) * count );
	if ( r->kappa == NULL || raw == NULL )
	{
		free( raw );
		free( r->kappa );
		r->kappa = NULL;
		return -1;
	}

	// Signed horizontal curvature between consecutive tangents — the trackgen
	// formula (t.x * t_next.z − t.z * t_next.x) / spacing, using the ACTUAL
	// per-segment distance instead of the nominal 2.5 m: identical on real
	// arc-length-resampled TRK1 tracks, and correct on test tracks whose
	// samples are not exactly 2.5 m apart (e.g. the uniform-angle flat oval).
	for ( uint32_t i = 0; i < count; ++i )
	{
		uint32_t j = ( i + 1 == count ) ? 0 : i + 1;
		const b3Vec3 t0 = samples[i].tangent;
		const b3Vec3 t1 = samples[j].tangent;
		float dist = b3Distance( samples[i].pos, samples[j].pos );
		if ( dist < 0.001f )
		{
			dist = 0.001f;
		}
		raw[i] = ( t0.x * t1.z - t0.z * t1.x ) / dist;
	}

	// Circular box smoothing, window 15 (±7), mirroring trackgen's convolve.
	const int hw = ROAD_KAPPA_WINDOW / 2;
	const float inv_w = 1.0f / (float)ROAD_KAPPA_WINDOW;
	for ( uint32_t i = 0; i < count; ++i )
	{
		float sum = 0.0f;
		for ( int k = -hw; k <= hw; ++k )
		{
			uint32_t idx = (uint32_t)( ( (int)i + k + (int)count ) % (int)count );
			sum += raw[idx];
		}
		r->kappa[i] = sum * inv_w;
	}

	free( raw );
	return 0;
}

void road_free( Road* r )
{
	free( r->kappa );
	r->kappa = NULL;
	r->samples = NULL;
	r->count = 0;
}

// ---------------------------------------------------------------------------
// Catmull-Rom evaluation on a closed sample loop
// ---------------------------------------------------------------------------

static float sDistSq( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 d = b3Sub( a, b );
	return b3Dot( d, d );
}

typedef struct CrWeights
{
	float w0, w1, w2, w3;	 // position weights
	float d0, d1, d2, d3;	 // first-derivative weights (d/du)
	float dd0, dd1, dd2, dd3; // second-derivative weights
} CrWeights;

static CrWeights sCrWeights( float u )
{
	float u2 = u * u;
	float u3 = u2 * u;
	CrWeights w;
	w.w0 = 0.5f * ( -u + 2.0f * u2 - u3 );
	w.w1 = 0.5f * ( 2.0f - 5.0f * u2 + 3.0f * u3 );
	w.w2 = 0.5f * ( u + 4.0f * u2 - 3.0f * u3 );
	w.w3 = 0.5f * ( -u2 + u3 );
	w.d0 = 0.5f * ( -1.0f + 4.0f * u - 3.0f * u2 );
	w.d1 = 0.5f * ( -10.0f * u + 9.0f * u2 );
	w.d2 = 0.5f * ( 1.0f + 8.0f * u - 9.0f * u2 );
	w.d3 = 0.5f * ( -2.0f * u + 3.0f * u2 );
	w.dd0 = 0.5f * ( 4.0f - 6.0f * u );
	w.dd1 = 0.5f * ( -10.0f + 18.0f * u );
	w.dd2 = 0.5f * ( 8.0f - 18.0f * u );
	w.dd3 = 0.5f * ( -2.0f + 6.0f * u );
	return w;
}

static b3Vec3 sCrCombine( b3Vec3 p0, b3Vec3 p1, b3Vec3 p2, b3Vec3 p3, float c0, float c1, float c2, float c3 )
{
	b3Vec3 v;
	v.x = c0 * p0.x + c1 * p1.x + c2 * p2.x + c3 * p3.x;
	v.y = c0 * p0.y + c1 * p1.y + c2 * p2.y + c3 * p3.y;
	v.z = c0 * p0.z + c1 * p1.z + c2 * p2.z + c3 * p3.z;
	return v;
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

uint32_t road_nearest_global( const Road* r, b3Vec3 p )
{
	uint32_t best = 0;
	float best_d = sDistSq( p, r->samples[0].pos );
	for ( uint32_t i = 1; i < r->count; ++i )
	{
		float d = sDistSq( p, r->samples[i].pos );
		if ( d < best_d )
		{
			best_d = d;
			best = i;
		}
	}
	return best;
}

// Clamped projection of p onto polyline segment a→b; returns t in [0,1] and
// writes the squared distance to the projected point.
static float sSegProject( b3Vec3 p, b3Vec3 a, b3Vec3 b, float* out_dsq )
{
	b3Vec3 ab = b3Sub( b, a );
	float len2 = b3Dot( ab, ab );
	float t = len2 > 1.0e-8f ? b3Dot( b3Sub( p, a ), ab ) / len2 : 0.0f;
	t = b3ClampFloat( t, 0.0f, 1.0f );
	b3Vec3 q = b3MulAdd( a, t, ab );
	*out_dsq = sDistSq( p, q );
	return t;
}

void road_query( const Road* r, b3Vec3 p, uint32_t hint, RoadQuery* out )
{
	const RoadSample* s = r->samples;
	const uint32_t n = r->count;
	if ( hint >= n )
	{
		hint = 0;
	}

	// --- 1. Nearest sample in a fixed ±6 window around the hint (spec §1;
	// same incremental trick as the lap logic, strict '<' for determinism) ---
	uint32_t best = hint;
	float best_d = sDistSq( p, s[hint].pos );
	for ( int k = -ROAD_SEARCH_WINDOW; k <= ROAD_SEARCH_WINDOW; ++k )
	{
		if ( k == 0 )
		{
			continue;
		}
		uint32_t idx = (uint32_t)( ( (int)hint + k + (int)n ) % (int)n );
		float d = sDistSq( p, s[idx].pos );
		if ( d < best_d )
		{
			best_d = d;
			best = idx;
		}
	}

	// --- 2. Segment choice: project onto the two polyline segments adjacent
	// to the nearest sample, keep the closer one ---
	uint32_t prev = ( best + n - 1 ) % n;
	uint32_t next = ( best + 1 == n ) ? 0 : best + 1;
	float d_prev, d_next;
	float t_prev = sSegProject( p, s[prev].pos, s[best].pos, &d_prev );
	float t_next = sSegProject( p, s[best].pos, s[next].pos, &d_next );

	uint32_t i;   // segment index (between samples i and i+1)
	float u;	  // CR parameter in [0,1] within that segment
	if ( d_prev < d_next )
	{
		i = prev;
		u = t_prev;
	}
	else
	{
		i = best;
		u = t_next;
	}

	// Control points for segment i (closed loop).
	uint32_t i0 = ( i + n - 1 ) % n;
	uint32_t i1 = i;
	uint32_t i2 = ( i + 1 == n ) ? 0 : i + 1;
	uint32_t i3 = ( i2 + 1 == n ) ? 0 : i2 + 1;
	b3Vec3 p0 = s[i0].pos, p1 = s[i1].pos, p2 = s[i2].pos, p3 = s[i3].pos;

	// --- 3. Two fixed Newton steps on f(u) = (c(u)−p)·c'(u) = 0 ---
	for ( int step = 0; step < ROAD_NEWTON_STEPS; ++step )
	{
		CrWeights w = sCrWeights( u );
		b3Vec3 c = sCrCombine( p0, p1, p2, p3, w.w0, w.w1, w.w2, w.w3 );
		b3Vec3 dc = sCrCombine( p0, p1, p2, p3, w.d0, w.d1, w.d2, w.d3 );
		b3Vec3 ddc = sCrCombine( p0, p1, p2, p3, w.dd0, w.dd1, w.dd2, w.dd3 );
		b3Vec3 e = b3Sub( c, p );
		float f = b3Dot( e, dc );
		float fp = b3Dot( dc, dc ) + b3Dot( e, ddc );
		if ( fp > 1.0e-8f )
		{
			u -= f / fp;
		}
		u = b3ClampFloat( u, 0.0f, 1.0f );
	}

	// --- 4. Frames at u: same CR weights, re-orthonormalized (spec §1) ---
	CrWeights w = sCrWeights( u );
	b3Vec3 c = sCrCombine( p0, p1, p2, p3, w.w0, w.w1, w.w2, w.w3 );
	b3Vec3 tan_i = sCrCombine( s[i0].tangent, s[i1].tangent, s[i2].tangent, s[i3].tangent, w.w0, w.w1, w.w2, w.w3 );
	b3Vec3 up_i = sCrCombine( s[i0].up, s[i1].up, s[i2].up, s[i3].up, w.w0, w.w1, w.w2, w.w3 );

	b3Vec3 tangent = b3Normalize( tan_i );
	b3Vec3 up = b3Normalize( b3MulAdd( up_i, -b3Dot( up_i, tangent ), tangent ) );
	b3Vec3 side = b3Cross( up, tangent ); // unit: up ⟂ tangent by construction

	float width = ( 1.0f - u ) * s[i1].width + u * s[i2].width; // linear (spec)
	float half = 0.5f * width;
	float kappa = ( 1.0f - u ) * r->kappa[i1] + u * r->kappa[i2];

	// --- 5. Lateral profile: h(l) and ∂h/∂l (spec §1) ---
	b3Vec3 rel = b3Sub( p, c );
	float l = b3Dot( rel, side );
	float al = b3AbsFloat( l );
	float sign_l = l >= 0.0f ? 1.0f : -1.0f;

	float h, dh;
	if ( al <= half )
	{
		// Crown: h = −crown_m · (2l/width)², 25 mm center-to-edge cross-fall.
		float ratio = l / half;
		h = -ROAD_CROWN_M * ratio * ratio;
		dh = -2.0f * ROAD_CROWN_M * l / ( half * half );
	}
	else
	{
		// Shoulder ramp: linear drop from the road edge to −1.2 m at
		// width/2 + 8 — matches the mesh ribbon (trackgen rows are a straight
		// line from edge height to edge − SHOULDER_DROP over SHOULDER_W).
		float over = al - half;
		h = -ROAD_SHOULDER_DROP * ( over / ROAD_SHOULDER_W );
		dh = -( ROAD_SHOULDER_DROP / ROAD_SHOULDER_W ) * sign_l;

		if ( over <= ROAD_KERB_WIDTH && b3AbsFloat( kappa ) > ROAD_KERB_KAPPA )
		{
			// Kerb band: smooth sinusoidal rumble replacing the mesh teeth
			// (tires only). h = 0.02·(0.5 + 0.5·sin(2π·s/1.25)) with
			// s = (i + u)·2.5 m; 2.5/1.25 = 2 exactly, so the phase reduces
			// to 4π·u (b3ComputeCosSin unwinds the angle internally).
			b3CosSin cs = b3ComputeCosSin( 2.0f * B3_PI * 2.0f * u );
			h = ROAD_KERB_HEIGHT * ( 0.5f + 0.5f * cs.sine );
			dh = 0.0f; // laterally flat band, like the mesh strip
		}
	}

	out->point = b3MulAdd( b3MulAdd( c, l, side ), h, up );
	out->normal = b3Normalize( b3MulAdd( up, -dh, side ) );
	out->lateral = l;
	out->on_road = al <= half + ROAD_SHOULDER_W;
	out->seg = i;
}

// road.h — analytic C1 road surface interpolated from the TRK1 centerline
// (docs/ROAD-SURFACE.md §1, binding). The wheels query this smooth surface
// instead of raycasting the low-poly collision mesh; the chassis body still
// collides with the mesh, and wheels fall back to the mesh raycast when the
// query is outside the analytic domain (on_road == 0).
//
// Determinism: fixed iteration counts everywhere (±ROAD_SEARCH_WINDOW sample
// window, ROAD_NEWTON_STEPS refinement steps), math limited to mul/add/div,
// sqrtf and b3ComputeCosSin (kerb sinusoid). No rand/time/IO.

#ifndef SIM_ROAD_H
#define SIM_ROAD_H

#include "box3d/math_functions.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constants mirroring trackgen/generate_track.py (keep in sync — the analytic
// surface must match the visual/collision mesh within tolerance).
#define ROAD_SPACING 2.5f		// TRK1 centerline sample spacing (CONTRACTS §8)
#define ROAD_CROWN_M 0.025f		// parabolic cross-fall, center to edge (m)
#define ROAD_KERB_KAPPA 0.022f	// KERB_KAPPA: |κ| gate for kerb bands (1/m)
#define ROAD_KERB_WIDTH 1.1f	// KERB_WIDTH_M (m)
#define ROAD_KERB_HEIGHT 0.02f	// KERB_TOOTH_M: sinusoid peak height (m)
#define ROAD_KERB_PERIOD 1.25f	// rumble wavelength (m) — exactly 2 per sample
#define ROAD_SHOULDER_W 8.0f	// SHOULDER_WIDTH_M (m)
#define ROAD_SHOULDER_DROP 1.2f // SHOULDER_DROP_M at the outer edge (m)
#define ROAD_KAPPA_WINDOW 15	// CURVATURE_SMOOTH_WINDOW (samples)

#define ROAD_SEARCH_WINDOW 6 // ± samples around the hint (fixed, spec §1)
#define ROAD_NEWTON_STEPS 2	 // CR projection refinement steps (fixed, spec §1)

// One TRK1 centerline sample. Field-for-field the layout sim.c parses
// (pos/up/tangent/width); sim.c stores its Track samples as this type.
typedef struct RoadSample
{
	b3Vec3 pos;
	b3Vec3 up;
	b3Vec3 tangent;
	float width;
} RoadSample;

typedef struct Road
{
	const RoadSample* samples; // borrowed — owned by the Track, must outlive us
	uint32_t count;
	float* kappa; // owned: smoothed signed horizontal curvature per sample (1/m)
} Road;

typedef struct RoadQuery
{
	b3Vec3 point;  // analytic surface point at the query's (u, lateral)
	b3Vec3 normal; // unit surface normal (up-facing), continuous through banking
	float lateral; // signed lateral offset from the centerline (m, + = left)
	int on_road;   // |lateral| <= width/2 + ROAD_SHOULDER_W: analytic answer valid
	uint32_t seg;  // segment (sample) index used — feed back as the next hint
} RoadQuery;

// Precompute the smoothed curvature array (S floats, malloc'd like the other
// track arrays). Borrows `samples`. Returns 0 on success, <0 on alloc failure.
int road_load( Road* r, const RoadSample* samples, uint32_t count );
void road_free( Road* r );

// Full deterministic O(S) scan for the nearest centerline sample. Used once
// to bootstrap a wheel's incremental hint after create/reset.
uint32_t road_nearest_global( const Road* r, b3Vec3 p );

// Analytic surface query near `hint` (window ±6 samples, 2 Newton steps —
// all iteration counts fixed). Fills *out; out->seg is the next hint.
void road_query( const Road* r, b3Vec3 p, uint32_t hint, RoadQuery* out );

#ifdef __cplusplus
}
#endif

#endif // SIM_ROAD_H

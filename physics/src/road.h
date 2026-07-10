// road.h — analytic C1 road surface interpolated from the TRK1 centerline
// (docs/ROAD-SURFACE.md §1, binding). The wheels query this smooth surface
// instead of raycasting the low-poly collision mesh; the chassis body still
// collides with the mesh. The query is TOTAL (docs/ROAD-SURFACE.md §6):
// outside the corridor it returns the flat grass plane at ground_y, so the
// wheels always find a surface — the mesh raycast remains only for callers
// with no road at all.
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

// Off-corridor grass plane (docs/ROAD-SURFACE.md §6, shared formula):
//   ground_y = (min over centerline samples of pos.y) - ROAD_GROUND_DROP
// trackgen bakes its collision-mesh ground quad at this height and the client
// ground disc (worker/public/js/ambience.js) uses the same formula — all
// three must agree.
#define ROAD_GROUND_DROP 1.35f

// RoadQuery surface kinds
#define ROAD_KIND_ASPHALT 0 // road proper / kerb / shoulder (corridor)
#define ROAD_KIND_GRASS 1	// off-corridor flat grass plane at ground_y

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
	float* kappa;	// owned: smoothed signed horizontal curvature per sample (1/m)
	float ground_y; // off-corridor grass height: min sample pos.y - ROAD_GROUND_DROP
	// Off-corridor grass fallback enabled (TRK1 v2 tracks only). v1 tracks
	// keep the legacy behavior — off-corridor queries report on_road = 0 with
	// the extrapolated shoulder profile and the wheels mesh-raycast — because
	// archived v1 replays MUST stay bit-identical forever (the wasm smoke
	// script provably dangles a wheel past the corridor edge; an ungated
	// grass plane there changes its hash).
	int grass_fallback;
} Road;

typedef struct RoadQuery
{
	b3Vec3 point;  // analytic surface point at the query's (u, lateral)
	b3Vec3 normal; // unit surface normal (up-facing), continuous through banking
	float lateral; // signed lateral offset from the centerline (m, + = left)
	int on_road;   // |lateral| <= width/2 + ROAD_SHOULDER_W (kind == ASPHALT)
	int kind;	   // ROAD_KIND_ASPHALT or ROAD_KIND_GRASS (query is total)
	uint32_t seg;  // segment (sample) index used — feed back as the next hint
} RoadQuery;

// Precompute the smoothed curvature array (S floats, malloc'd like the other
// track arrays) and ground_y. Borrows `samples`. grass_fallback enables the
// §6 off-corridor grass plane (pass track version >= 2; 0 preserves legacy v1
// behavior bit-exactly). Returns 0 on success, <0 on alloc failure.
int road_load( Road* r, const RoadSample* samples, uint32_t count, int grass_fallback );
void road_free( Road* r );

// Full deterministic O(S) scan for the nearest centerline sample. Used once
// to bootstrap a wheel's incremental hint after create/reset.
uint32_t road_nearest_global( const Road* r, b3Vec3 p );

// Analytic surface query near `hint` (window ±6 samples, 2 Newton steps —
// all iteration counts fixed). Fills *out; out->seg is the next hint.
// Inside the corridor (|lateral| <= width/2 + ROAD_SHOULDER_W) the answer is
// bit-identical to the pre-§6 behavior with kind = ASPHALT. Outside:
//   grass_fallback ON  (v2 tracks) — the query is TOTAL: flat grass plane
//     (p.x, ground_y, p.z), normal +Y, zero curvature, kind = GRASS.
//   grass_fallback OFF (v1 tracks) — legacy: extrapolated shoulder profile
//     with on_road = 0, kind = ASPHALT (callers mesh-raycast, bit-identical
//     to the pre-§6 kernel).
void road_query( const Road* r, b3Vec3 p, uint32_t hint, RoadQuery* out );

#ifdef __cplusplus
}
#endif

#endif // SIM_ROAD_H

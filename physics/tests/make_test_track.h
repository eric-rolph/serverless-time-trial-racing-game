// make_test_track.h — builds a flat oval TRK1 blob in memory for tests.
#ifndef SIM_MAKE_TEST_TRACK_H
#define SIM_MAKE_TEST_TRACK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Allocates and fills a TRK1 blob (CONTRACTS §8): a flat oval with a genuine
// triangle terrain strip, a centerline, and 3 checkpoints. Caller frees.
// Returns NULL on allocation failure. *out_len receives the byte length.
uint8_t* make_test_track( size_t* out_len );

// Parametric variant for harnesses (drivetrain launch strip, skidpad):
// flat ellipse rx x rz with `samples` centerline samples, drivable `width`,
// terrain strip half-width `terrain_half`. Same format/checkpoints/spawn
// conventions as make_test_track (which equals rx=60 rz=40 samples=256
// width=10 terrain_half=7). Caller frees.
uint8_t* make_test_track_ex( size_t* out_len, float rx, float rz, uint32_t samples, float width,
							 float terrain_half );

// Sloped-terrain variant (docs/TERRAIN.md §2 raycast test): the standard flat
// oval PLUS a large sloped rectangle in the collision mesh, placed OFF the
// corridor on the outside of the spawn straight (where the grass-excursion
// drive exits). Height is linear in x:
//   y(x) = TT_SLOPE_BASE_Y + TT_SLOPE_K * (x - TT_SLOPE_X0)
// TT_SLOPE_BASE_Y equals the oval's ground_y (min sample y - 1.35), so the
// slope takes over exactly where the flat grass plane would be. Caller frees.
#define TT_SLOPE_X0 44.0f
#define TT_SLOPE_X1 120.0f
#define TT_SLOPE_Z0 30.0f
#define TT_SLOPE_Z1 60.0f
#define TT_SLOPE_K 0.08f		   // rise per meter of x (~4.6 deg, drivable)
#define TT_SLOPE_BASE_Y ( -1.35f ) // == ground_y of the flat oval
uint8_t* make_test_track_sloped( size_t* out_len );

#ifdef __cplusplus
}
#endif

#endif

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

#ifdef __cplusplus
}
#endif

#endif

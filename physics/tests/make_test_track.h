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

#ifdef __cplusplus
}
#endif

#endif

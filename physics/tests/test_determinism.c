// test_determinism.c — run the same scripted 8000-tick session twice from
// fresh sim instances and assert bit-identical state hashes at every 500-tick
// checkpoint (CONTRACTS §1.4).

#include "make_test_track.h"
#include "sim/sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICKS 8000
#define CHECK_EVERY 500
#define CHECKPOINTS ( TICKS / CHECK_EVERY )

// Deterministic scripted inputs: accelerate, steer sweep, brake, accelerate.
static void script( uint32_t tick, int32_t* steer, uint32_t* throttle, uint32_t* brake, uint32_t* flags )
{
	*flags = 0;
	if ( tick < 2000 )
	{
		// launch: full throttle, straight
		*steer = 0;
		*throttle = 65535;
		*brake = 0;
	}
	else if ( tick < 5000 )
	{
		// steer sweep at 60% throttle
		double phase = ( (double)( tick - 2000 ) / 3000.0 ) * 2.0 * 3.14159265358979323846;
		*steer = (int32_t)( 20000.0 * sin( phase ) );
		*throttle = 39321;
		*brake = 0;
	}
	else if ( tick < 6500 )
	{
		// hard brake
		*steer = 0;
		*throttle = 0;
		*brake = 65535;
	}
	else
	{
		// pull away again with slight steer, tap of handbrake at the end
		*steer = -8000;
		*throttle = 52428;
		*brake = 0;
		if ( tick > 7800 )
		{
			*flags = 1;
		}
	}
}

static int run_session( const uint8_t* blob, size_t blob_len, uint32_t hashes_lo[CHECKPOINTS + 1],
						uint32_t hashes_hi[CHECKPOINTS + 1] )
{
	int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)blob_len );
	if ( rc != 0 )
	{
		fprintf( stderr, "sim_load_track failed: %d\n", rc );
		return 1;
	}
	sim_reset();

	int ci = 0;
	for ( uint32_t tick = 1; tick <= TICKS; ++tick )
	{
		int32_t steer;
		uint32_t throttle, brake, flags;
		script( tick - 1, &steer, &throttle, &brake, &flags );

		uint32_t status = sim_step( steer, throttle, brake, flags );
		if ( status & SIM_STATUS_ERROR )
		{
			fprintf( stderr, "sim_step returned ERROR at tick %u\n", tick );
			return 1;
		}
		if ( tick % CHECK_EVERY == 0 )
		{
			hashes_lo[ci] = sim_state_hash_lo();
			hashes_hi[ci] = sim_state_hash_hi();
			ci++;
		}
	}
	hashes_lo[CHECKPOINTS] = sim_state_hash_lo();
	hashes_hi[CHECKPOINTS] = sim_state_hash_hi();
	return 0;
}

int main( void )
{
	size_t blob_len = 0;
	uint8_t* blob = make_test_track( &blob_len );
	if ( blob == NULL )
	{
		fprintf( stderr, "make_test_track failed\n" );
		return 1;
	}

	static uint32_t lo_a[CHECKPOINTS + 1], hi_a[CHECKPOINTS + 1];
	static uint32_t lo_b[CHECKPOINTS + 1], hi_b[CHECKPOINTS + 1];

	if ( run_session( blob, blob_len, lo_a, hi_a ) != 0 )
	{
		return 1;
	}
	if ( run_session( blob, blob_len, lo_b, hi_b ) != 0 )
	{
		return 1;
	}

	int failures = 0;
	for ( int i = 0; i <= CHECKPOINTS; ++i )
	{
		if ( lo_a[i] != lo_b[i] || hi_a[i] != hi_b[i] )
		{
			fprintf( stderr, "hash mismatch at checkpoint %d (tick %d): %08x%08x vs %08x%08x\n", i,
					 ( i + 1 ) * CHECK_EVERY, hi_a[i], lo_a[i], hi_b[i], lo_b[i] );
			failures++;
		}
	}

	free( blob );

	if ( failures != 0 )
	{
		fprintf( stderr, "FAIL: %d hash mismatches\n", failures );
		return 1;
	}

	printf( "test_determinism PASS — final hash %08x%08x over %d ticks\n", hi_a[CHECKPOINTS], lo_a[CHECKPOINTS], TICKS );
	return 0;
}

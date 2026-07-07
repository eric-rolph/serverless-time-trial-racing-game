// wasm_api.c — the exact export surface from CONTRACTS §1.1.
//
// sim_load_track / sim_reset / sim_step / sim_state_* / sim_lap_time_ticks are
// implemented in sim.c; this file adds sim_abi_version, sim_alloc, sim_replay
// and (for the Emscripten build) libc stubs that keep WASI imports off the hot
// path. No printf/IO anywhere in the kernel.

#include "sim/sim.h"

#include <stdlib.h>
#include <string.h>

uint32_t sim_abi_version( void )
{
	return SIM_ABI_VERSION;
}

sim_ptr_t sim_alloc( uint32_t len )
{
	if ( len == 0 )
	{
		return 0;
	}
	void* p = malloc( len );
	return (sim_ptr_t)p;
}

// LAPLOG tick record (CONTRACTS §3): { i16 steer, u16 throttle, u16 brake,
// u16 flags } — 8 bytes, little-endian. Runs at max speed; the host calls
// sim_reset() first. Lap bits (LAP_COMPLETE / LAP_INVALID) are accumulated
// so a lap completed mid-replay is visible in the returned status.
uint32_t sim_replay( sim_ptr_t log_ptr, uint32_t tick_count )
{
	const unsigned char* p = (const unsigned char*)log_ptr;
	uint32_t status = 0;
	uint32_t sticky = 0;

	for ( uint32_t i = 0; i < tick_count; ++i )
	{
		int16_t steer;
		uint16_t throttle;
		uint16_t brake;
		uint16_t flags;
		memcpy( &steer, p + 8 * (size_t)i + 0, 2 );
		memcpy( &throttle, p + 8 * (size_t)i + 2, 2 );
		memcpy( &brake, p + 8 * (size_t)i + 4, 2 );
		memcpy( &flags, p + 8 * (size_t)i + 6, 2 );

		status = sim_step( (int32_t)steer, (uint32_t)throttle, (uint32_t)brake, (uint32_t)flags );
		if ( status & SIM_STATUS_ERROR )
		{
			return status;
		}
		sticky |= status & ( SIM_STATUS_LAP_COMPLETE | SIM_STATUS_LAP_INVALID );
	}

	return status | sticky;
}

#ifdef __EMSCRIPTEN__
// ---------------------------------------------------------------------------
// Deterministic libc overrides.
//
// Box3D calls b3GetTicks() (→ clock_gettime) every b3World_Step for its
// profile counters. Left alone that emits a wasi clock_time_get import that
// IS on the hot path, which ADR-008 forbids. Defining these symbols here
// shadows the libc versions at link time, killing the imports entirely.
// The values only feed b3Profile timings — never simulation state — so a
// constant clock is deterministic and harmless.
// ---------------------------------------------------------------------------

#include <time.h>
#include <sched.h>

int clock_gettime( clockid_t clock_id, struct timespec* tp )
{
	(void)clock_id;
	if ( tp != NULL )
	{
		tp->tv_sec = 0;
		tp->tv_nsec = 0;
	}
	return 0;
}

int nanosleep( const struct timespec* req, struct timespec* rem )
{
	(void)req;
	(void)rem;
	return 0;
}

int sched_yield( void )
{
	return 0;
}
#endif // __EMSCRIPTEN__

// sim.h — public C API of the deterministic physics kernel.
//
// This is the single authority for SIM_TICK_RATE / SIM_DT and the SimStateV1
// layout. See docs/CONTRACTS.md §1 (binding).

#ifndef SIM_SIM_H
#define SIM_SIM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Constants (CONTRACTS §2)
// ---------------------------------------------------------------------------

#define SIM_TICK_RATE 400

// A single float literal, defined once. Never computed as 1.0/400.0 at runtime.
#define SIM_DT 0.0025f

#define SIM_ABI_VERSION 1u

// ---------------------------------------------------------------------------
// Status bits (CONTRACTS §1.3)
// ---------------------------------------------------------------------------

#define SIM_STATUS_RUNNING 0x00000001u
#define SIM_STATUS_LAP_COMPLETE 0x00000002u
#define SIM_STATUS_OFF_TRACK 0x00000004u
#define SIM_STATUS_LAP_INVALID 0x00000008u
#define SIM_STATUS_ERROR 0x80000000u

// Input flag bits (CONTRACTS §2)
#define SIM_FLAG_HANDBRAKE 0x0001u

// ---------------------------------------------------------------------------
// SimStateV1 (CONTRACTS §1.2) — packed little-endian, fixed offsets, 200 bytes
// ---------------------------------------------------------------------------

#define SIM_WHEEL_COUNT 4

// Per-wheel state block: 8 floats = 32 bytes.
typedef struct SimWheelStateV1
{
	float pos[3];			 // wheel center, world space (m)
	float spin_angle;		 // accumulated spin (rad), wrapped
	float steer_angle;		 // steering angle (rad)
	float susp_compression;	 // suspension compression (m), 0 = fully extended
	float slip_ratio;		 // longitudinal slip ratio
	float slip_angle;		 // lateral slip angle (rad)
} SimWheelStateV1;

typedef struct SimStateV1
{
	uint32_t tick;								  // offset 0
	float pos[3];								  // offset 4   chassis position (m)
	float quat[4];								  // offset 16  chassis orientation (x,y,z,w)
	float lin_vel[3];							  // offset 32  linear velocity (m/s)
	float ang_vel[3];							  // offset 44  angular velocity (rad/s)
	SimWheelStateV1 wheels[SIM_WHEEL_COUNT];	  // offset 56  4 * 32 = 128 bytes
	float speed;								  // offset 184 speed (m/s)
	float lap_progress;							  // offset 188 [0,1]
	uint32_t checkpoint_mask;					  // offset 192
	uint32_t lap_time_ticks;					  // offset 196 valid on LAP_COMPLETE
} SimStateV1;

_Static_assert( sizeof( SimWheelStateV1 ) == 32, "SimWheelStateV1 must be 32 bytes" );
_Static_assert( offsetof( SimStateV1, tick ) == 0, "tick offset" );
_Static_assert( offsetof( SimStateV1, pos ) == 4, "pos offset" );
_Static_assert( offsetof( SimStateV1, quat ) == 16, "quat offset" );
_Static_assert( offsetof( SimStateV1, lin_vel ) == 32, "lin_vel offset" );
_Static_assert( offsetof( SimStateV1, ang_vel ) == 44, "ang_vel offset" );
_Static_assert( offsetof( SimStateV1, wheels ) == 56, "wheels offset" );
_Static_assert( offsetof( SimStateV1, speed ) == 184, "speed offset" );
_Static_assert( offsetof( SimStateV1, lap_progress ) == 188, "lap_progress offset" );
_Static_assert( offsetof( SimStateV1, checkpoint_mask ) == 192, "checkpoint_mask offset" );
_Static_assert( offsetof( SimStateV1, lap_time_ticks ) == 196, "lap_time_ticks offset" );
_Static_assert( sizeof( SimStateV1 ) == 200, "SimStateV1 must be 200 bytes" );

// ---------------------------------------------------------------------------
// Kernel API (CONTRACTS §1.1). On wasm these are the module exports
// (via src/wasm_api.c); natively they are called directly by the tests.
// ---------------------------------------------------------------------------

// Pointer-sized integer for buffer addresses. On wasm32 uintptr_t IS uint32_t,
// so the wasm export signatures match CONTRACTS §1.1 exactly (i32 params and
// results). Native (x64) test builds get full-width pointers.
typedef uintptr_t sim_ptr_t;

// = SIM_ABI_VERSION
uint32_t sim_abi_version( void );

// Allocate a buffer inside the kernel heap for host→kernel data (track blob,
// input log). Returns 0 on failure. Never freed individually in v1.
sim_ptr_t sim_alloc( uint32_t len );

// Parse a TRK1 blob (CONTRACTS §8), build the static collision world and the
// vehicle. 0 = ok, <0 = error (see SIM_ERR_*). Implicitly resets.
int32_t sim_load_track( sim_ptr_t ptr, uint32_t len );

#define SIM_ERR_BAD_MAGIC ( -1 )
#define SIM_ERR_BAD_VERSION ( -2 )
#define SIM_ERR_TRUNCATED ( -3 )
#define SIM_ERR_BAD_COUNTS ( -4 )
#define SIM_ERR_ALLOC ( -5 )
#define SIM_ERR_BAD_GEOMETRY ( -6 )

// Car to spawn pose, tick = 0, lap state cleared.
void sim_reset( void );

// One 400 Hz tick with quantized inputs (CONTRACTS §2):
//   steer:    i16 range −32767..32767 → −1..1 (passed as i32)
//   throttle: u16 0..65535 → 0..1
//   brake:    u16 0..65535 → 0..1
//   flags:    u16 bitfield, bit 0 = handbrake
// Returns status bits (CONTRACTS §1.3).
uint32_t sim_step( int32_t steer, uint32_t throttle, uint32_t brake, uint32_t flags );

// Run tick_count 8-byte records {i16 steer, u16 throttle, u16 brake, u16 flags}
// at max speed. Does NOT reset first — the host calls sim_reset() before this.
// Returns the final status bits (ORed lap-completion bits are sticky, see NOTES).
uint32_t sim_replay( sim_ptr_t log_ptr, uint32_t tick_count );

// Pointer (wasm linear-memory offset / native address) to the SimStateV1 block.
sim_ptr_t sim_state_ptr( void );
uint32_t sim_state_size( void );

// FNV-1a-64 over: tick, chassis pos/quat/vel/angvel, all wheel state,
// checkpoint mask (CONTRACTS §1.4). Recomputed on demand, split for JS.
uint32_t sim_state_hash_lo( void );
uint32_t sim_state_hash_hi( void );

// Lap time in ticks; valid when the last sim_step returned LAP_COMPLETE.
uint32_t sim_lap_time_ticks( void );

// Steering rack torque (Nm at the rim) for force feedback. Output-only —
// reads never affect simulation or hashes. Added in ABI 1.1; hosts may ignore.
float sim_ffb_torque( void );

// Tire tread surface temperature (deg C) for wheel 0-3 (FL FR RL RR); out of
// range (or no loaded world) returns 0. Output-only — reads never affect
// simulation or hashes. Added in ABI 1.2 (additive); hosts may ignore.
float sim_tire_temp( uint32_t wheel );

// Native-test helper (not a wasm export): direct state access.
const SimStateV1* sim_state( void );

#ifdef __cplusplus
}
#endif

#endif // SIM_SIM_H

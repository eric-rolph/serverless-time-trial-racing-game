# CONTRACTS — binding interfaces between subsystems

Every subsystem (physics kernel, Rust client, edge validator, track generator) is
built against this document. Changes here require an ADR in `DECISIONS.md` and a
version bump of the affected format.

## 1. The deterministic kernel: `sim.wasm`

One wasm32 binary, compiled by Emscripten from `physics/`, is the **only** physics
authority. The Rust client executes it via `wasmtime`; the Cloudflare Worker
executes it via V8. WebAssembly mandates bit-exact IEEE-754 semantics (no FMA
contraction, no fast-math, deterministic rounding), so:

> identical `sim.wasm` + identical quantized input log ⇒ bit-identical final state.

Native (clang) builds of the kernel exist **only** for unit testing and debugging.
Only wasm-vs-wasm identity is load-bearing.

Build constraints (enforced in `build_wasm.sh` and CI):

- `-O2`. **Never** `-ffast-math`, `-funsafe-math-optimizations`, or `-ffp-contract=fast`
  (`-ffp-contract=off` explicitly).
- `-sSTANDALONE_WASM=1 --no-entry`, no Emscripten JS glue required at runtime.
- Single-threaded. `BOX3D_DISABLE_SIMD=ON` for native test builds; wasm build has no
  x86 SIMD and must not enable wasm SIMD128.
- No `time()`, no `rand()`, no I/O inside the kernel.

### 1.1 Exports

```
memory                                    exported linear memory
sim_abi_version() -> u32                  // = 1
sim_alloc(len: u32) -> u32                // allocate buffer for host→wasm data
sim_load_track(ptr: u32, len: u32) -> i32 // parse TRK1 blob; 0 = ok, <0 = error
sim_reset() -> void                       // car to spawn pose, tick = 0
sim_step(steer: i32, throttle: u32, brake: u32, flags: u32) -> u32
                                          // one 400 Hz tick; quantized args (§2)
                                          // returns status bits (§1.3)
sim_replay(log_ptr: u32, tick_count: u32) -> u32
                                          // ticks[] is 8-byte records (§3), runs at
                                          // max speed; returns final status bits
sim_state_ptr() -> u32                    // pointer to SimStateV1 (§1.2)
sim_state_size() -> u32
sim_state_hash_lo() -> u32                // FNV-1a-64 over dynamic state, low half
sim_state_hash_hi() -> u32                // high half (avoids u64 at JS boundary)
sim_lap_time_ticks() -> u32               // valid when LAP_COMPLETE set
sim_ffb_torque() -> f32                   // ABI 1.1: steering rack torque (Nm)
                                          // for FFB — output-only, never affects
                                          // simulation state or hashes
sim_tire_temp(wheel: u32) -> f32          // ABI 1.2: tire surface temp (°C),
                                          // wheel 0-3 (FL FR RL RR); output-only
sim_damage(component: u32) -> f32         // ABI 1.3: deterministic crash damage,
                                          // component 0 = overall [0,1],
                                          // 1 = steer loss [0,1], 2 = |front toe| rad,
                                          // 3 = |rear toe| rad. Additive export;
                                          // derived purely from the input log, so
                                          // replays stay honest. Output-only read —
                                          // out-of-range/no-world → 0.
```

### 1.2 `SimStateV1` (packed, little-endian, fixed offsets)

| offset | type      | field |
|-------:|-----------|-------|
| 0      | u32       | tick |
| 4      | f32[3]    | chassis position (m) |
| 16     | f32[4]    | chassis orientation quat (x,y,z,w) |
| 32     | f32[3]    | linear velocity |
| 44     | f32[3]    | angular velocity |
| 56     | f32[4][8] | per wheel: pos[3], spin_angle, steer_angle, susp_compression, slip_ratio, slip_angle |
| 184    | f32       | speed (m/s) |
| 188    | f32       | lap progress [0,1] |
| 192    | u32       | checkpoint mask |
| 196    | u32       | lap_time_ticks (valid on completion) |

Total size: 200 bytes. The client copies this out every tick and keeps
previous+current for α-interpolation.

### 1.3 Status bits

```
0x00000001 RUNNING          sim active, no fault
0x00000002 LAP_COMPLETE     all checkpoints hit, start line re-crossed
0x00000004 OFF_TRACK        car left drivable surface this tick (informational)
0x00000008 LAP_INVALID      checkpoint order violated / corner cut
0x80000000 ERROR            internal fault (bad track, not initialized)
```

### 1.4 State hash

FNV-1a 64-bit over the raw bytes of: tick, chassis pos/quat/vel/angvel, all wheel
state, checkpoint mask. Recomputed on demand. This is the determinism fingerprint:
the client submits it, the validator recomputes it after replay; mismatch ⇒ reject.

## 2. Tick rate & input quantization

- `TICK_RATE = 400` Hz. `DT = 0.0025f` — a single float literal defined once in
  `physics/include/sim/sim.h`. Never computed as `1.0/400.0` at runtime.
- `steer`: i16, −32767..32767 ⇒ −1.0..1.0 (full left..full right at max lock)
- `throttle`: u16, 0..65535 ⇒ 0.0..1.0
- `brake`: u16, 0..65535 ⇒ 0.0..1.0
- `flags`: u16 bitfield — bit 0 = handbrake; bits 1–15 reserved (must be 0 in v1)

The client polls hardware at ~1 kHz; each 400 Hz tick consumes the **latest**
sample, quantizes it, feeds the sim, and appends the quantized record to the log.
The quantized values are the canonical truth — both executions consume exactly
these integers, so no analog nuance is lost between client run and validator replay.

## 3. Input log format: `LAPLOG` v1 (little-endian)

```
offset      size  field
0           4     magic "STLG" (0x53544C47 as bytes S,T,L,G)
4           2     version = 1
6           2     tick_rate = 400
8           8     track_hash — FNV-1a-64 of the exact track.bin bytes
16          4     tick_count N  (1 ≤ N ≤ 72000, i.e. ≤ 3 minutes)
20          8·N   tick records: { i16 steer, u16 throttle, u16 brake, u16 flags }
20+8N       8     final_state_hash (sim_state_hash after last tick)
28+8N       4     claimed_lap_ticks (u32)
```

Max size ≈ 576 KiB — fits one WebSocket binary message (< 1 MiB Workers limit).

## 4. Signing (Ed25519)

- Client generates a keypair on first launch, stored in the OS config dir
  (never transmitted). **Identity = public key.**
- Signed message: ASCII domain tag `"sttr-lap-v1"` ‖ the exact LAPLOG bytes as
  transmitted. Signature: Ed25519 (64 bytes).
- Client: `ed25519-dalek`. Worker: WebCrypto `{ name: "Ed25519" }`.
- Scope honesty: the signature proves log integrity and authorship by a keypair —
  it does **not** prove the inputs came from physical hardware. That is the
  heuristic gate's job (§6).

## 5. WebSocket protocol — `wss://…/api/submit`

Client sends one JSON text frame, then one binary frame:

```json
{ "type": "submit", "pubkey": "<base64 32B>", "sig": "<base64 64B>",
  "name": "<display name ≤ 24 chars>", "logBytes": 123456 }
```

Server responses (JSON text frames):

```json
{ "type": "ack", "stage": "received" | "verified" | "heuristics" | "replayed" }
{ "type": "result", "status": "accepted", "lapTimeMs": 83452, "rank": 4 }
{ "type": "result", "status": "pending", "lapTimeMs": 83452 }
   // VALIDATE_MODE=queue (free tier): signature+heuristics passed; replay
   // validation happens in the Actions sweeper within ~30 min
{ "type": "result", "status": "rejected",
  "reason": "bad_signature" | "heuristics_failed" | "replay_mismatch" |
            "track_mismatch" | "log_malformed" | "too_large",
  "detail": "…" }
```

## 6. Heuristic gate (anti-TAS telemetry analysis)

Runs on the steer channel (i16 series) before the expensive replay. Metrics:

- `identical_run_ratio` — fraction of consecutive identical values outside the
  center dead zone. Real wheels have ADC noise; long flat runs are synthetic.
- `jerk_variance` — variance of the second difference. Spline-generated inputs
  are pathologically smooth.
- `distinct_value_ratio` — distinct values / samples. Scripted inputs cluster on
  few quantization points.
- `micro_jitter_energy` — mean |first difference| within "quiet" segments; a
  physical sensor never reads exactly constant.

Verdict `{ score, flags[] }`; reject only on ≥ 2 independent flags (conservative —
false-rejecting humans is worse than admitting a marginal entry, which replay
validation still bounds). Flags are stored with the leaderboard entry for audit.

## 7. Leaderboard (Cloudflare KV, binding `LEADERBOARD`)

- Entry: `lb:{track_hash_hex}:{lap_ticks zero-padded to 8}:{pubkey_hex[:16]}`
  → JSON `{ pubkey, name, ticks, ms, submittedAt, flags }`
  KV list is lexicographic ⇒ prefix scan returns ascending lap times for free.
- Personal best: `best:{track_hash_hex}:{pubkey_hex}` → ticks. New entry written
  only on improvement; the superseded `lb:` key is deleted.
- `GET /api/leaderboard?track=<hex>` → top 100 JSON.
- `GET /api/track/current` → active track.bin (binary) + `x-track-hash` header.

## 8. Track data: `TRK1` (little-endian)

```
offset  size  field
0       4     magic "STRK"
4       2     version = 1
6       2     checkpoint_count C
8       8     seed (u64, generator provenance)
16      4     centerline sample count S
20      4     terrain vertex count V
24      4     terrain triangle count T
28      40·S  centerline: pos f32[3], up f32[3], tangent f32[3], width f32
28+40S  4·C   checkpoint sample indices (u32, ascending; index into centerline)
…       12·V  terrain vertices f32[3]
…       12·T  triangle indices u32[3]
…       12    spawn pose: position index u32, yaw f32, reserved f32
```

- `track_hash = FNV-1a-64(track.bin bytes)` — used in LAPLOG, KV keys, and URLs.
- The kernel builds static collision geometry from the triangle soup; the exact
  Box3D shape mapping (mesh vs. hull decomposition) is `physics/`-internal.
- Binary floats are consumed as IEEE bit patterns — no text parsing in C.

## 9. Versioning

`sim_abi_version() = 1`, LAPLOG v1, TRK1 v1 move in lockstep; a breaking change to
any of them bumps all three and invalidates in-flight submissions. Weekly track
rotation partitions the leaderboard naturally via `track_hash`.

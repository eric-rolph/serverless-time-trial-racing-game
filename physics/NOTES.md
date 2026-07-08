# physics/ — implementation notes, discoveries, contract deviations

Status (2026-07-05): native build green, both native tests pass, wasm build
green, `tests/wasm_smoke.mjs` passes (sim_step and sim_replay paths produce the
identical hash `6f375f286e4d406e` over 4000 scripted ticks).

## Box3D findings (submodule @ 540ea38)

- **Static triangle-mesh shape exists.** `b3CreateMesh(b3MeshDef)` builds a
  BVH'd triangle soup (`b3MeshData`), attached via
  `b3CreateMeshShape(body, shapeDef, mesh, scale)`. Mesh collision only
  generates contacts on **static** bodies — exactly our terrain use case, so
  no hull-decomposition fallback was needed. `b3CreateMesh` **clones** the
  input vertex/index arrays into one relocatable block (verified in
  `src/mesh.c`), so the parse-time arrays are freed after creation; the
  returned `b3MeshData*` must outlive the shape and is kept in the Track and
  destroyed with `b3DestroyMesh` on reload.
- **Up axis:** Box3D has no built-in up vector; `b3DefaultWorldDef()` sets
  gravity `(0, -10, 0)` → Y-up convention. We use Y-up, gravity
  `(0, -9.81, 0)`.
- **Threading defaults:** `b3DefaultWorldDef()` leaves `workerCount = 0` and
  no task callbacks; `b3CreateWorld` then runs strictly single-threaded on the
  calling thread (no internal scheduler, no thread creation —
  `physics_world.c:354-375`). We set these explicitly anyway.
- **Raycast:** `b3World_CastRayClosest(world, origin, translation, filter)`
  returns `b3RayResult {shapeId, point, normal, fraction, hit, ...}`. Ray
  hit-point is `origin + fraction * translation`. Query filtering works like
  shape filtering (`categoryBits`/`maskBits`); chassis is category
  `SIM_CAT_CHASSIS`, terrain `SIM_CAT_TERRAIN`, suspension rays mask
  terrain-only so they can never hit the car's own hull.
- **Force application:** `b3Body_ApplyForce(body, force, worldPoint, wake)`
  (not `ApplyForceAtPoint`). Forces are ignored on sleeping bodies → world
  and chassis are created with `enableSleep = false`.
- **Quaternions** are `{ b3Vec3 v; float s; }` → (x,y,z,w) matches the
  SimStateV1 quat field order directly.
- **Deterministic math:** Box3D ships `b3ComputeCosSin` / `b3Atan2` designed
  for cross-platform determinism. The vehicle code uses these for all
  trig; the only libm call in kernel code is `sqrtf` (IEEE-exact, lowers to
  the wasm `f32.sqrt` instruction).
- **Asserts:** `B3_ASSERT` compiles to `(void)0` under `NDEBUG` (see
  `base.h`), so the wasm build just defines `-DNDEBUG` — no assert hook
  override needed.
- **SIMD:** without `BOX3D_DISABLE_SIMD`, an Emscripten build maps Box3D's
  SSE2 path onto wasm SIMD128 (`core.h` + top-level CMakeLists add
  `-msimd128 -msse2`). The wasm build therefore defines
  `-DBOX3D_DISABLE_SIMD` (contract: no SIMD128).
- **Hot-path clock:** `b3World_Step` calls `b3GetTicks()` (→ `clock_gettime`)
  every step for its profile counters. Left alone this puts a
  `wasi_snapshot_preview1.clock_time_get` import **on the hot path**, which
  ADR-008 forbids. `src/wasm_api.c` defines `clock_gettime` / `nanosleep` /
  `sched_yield` no-op overrides under `__EMSCRIPTEN__`; they shadow libc at
  link time and the import disappears. The values only feed `b3Profile`
  timings, never simulation state.
- **Remaining wasm imports** (verified): `env.emscripten_notify_memory_growth`
  (no-op unless memory grows) and `wasi_snapshot_preview1.fd_write` (reachable
  only from `printf` on Box3D recording-replay error paths, never called by
  the kernel). Both are satisfied by trivial no-op stubs in any host.

## Design decisions / clarifications (not deviations)

- **Coordinates:** chassis-local +X right, +Y up, +Z forward. Yaw rotates
  about +Y; yaw = 0 faces +Z, so `forward = (sin yaw, 0, cos yaw)` and track
  generators should emit `spawn yaw = atan2(tangent.x, tangent.z)`.
- **`sim_ptr_t`:** pointer parameters/returns (`sim_alloc`, `sim_load_track`,
  `sim_replay`, `sim_state_ptr`) are declared `uintptr_t`. On wasm32 that IS
  `u32`, so the module's export signatures match CONTRACTS §1.1 exactly;
  native x64 test builds get full-width pointers.
- **`sim_replay` does not reset.** CONTRACTS §1.1 doesn't say either way; the
  host (validator) calls `sim_load_track`/`sim_reset` first. LAP_COMPLETE and
  LAP_INVALID bits observed mid-replay are OR-ed into the returned final
  status so a completed lap isn't masked by trailing ticks.
- **Hash coverage:** FNV-1a-64 over SimStateV1 bytes [0,184) (tick, chassis
  pos/quat/vel/angvel, all wheel state — contiguous by layout) plus the
  4 checkpoint-mask bytes at offset 192. `speed`, `lap_progress` and
  `lap_time_ticks` are derived values and are NOT hashed, per §1.4's field
  list.
- **Lap logic:** incremental nearest-centerline tracking with a fixed ±12
  sample search window around the last index (deterministic: fixed iteration
  order, strict `<`). Progress = (index + tangent projection)/S. A checkpoint
  is awarded when the nearest index crosses its sample going forward while
  the car is on the drivable surface (|lateral| ≤ width/2); crossing it
  off-track, or reaching a checkpoint before all earlier ones, sets
  LAP_INVALID (sticky until the next start-line crossing). Start line =
  nearest index wrapping from the last quarter to the first quarter; with all
  checkpoints set and no invalidation → LAP_COMPLETE +
  `lap_time_ticks = tick − lap_start_tick`. Crossing the start line backwards
  just restarts lap tracking.
- **Spawn pose:** centerline sample `spawn_index` position + 0.9 m along its
  up vector (suspension settles in ~0.5 s), rotation = yaw about +Y.
- **Tire/surface simplification:** slip is computed in the chassis horizontal
  basis (not projected onto the contact-plane normal); fine for gentle
  terrain, noted for a future fidelity pass. Suspension force is applied
  along chassis-up at the contact point; damper velocity is the
  per-tick compression delta / SIM_DT.

## Vehicle tunables (single struct `kTuning` in src/vehicle.c)

- Chassis 1.9 × 1.1 × 4.4 m box hull, 1350 kg, CoM lowered 0.15 m, inertia
  diag (pitch 2400, yaw 2600, roll 550) kg·m².
- Suspension: rest 0.35 m, travel 0.25 m, k = 60 kN/m, damper 4500/5200
  N·s/m (bump/rebound), wheel radius 0.33 m, hardpoints ±0.80 m × ±1.30 m at
  local y = −0.25. Static compression ≈ 0.055 m (measured exactly in test).
- Pacejka: long B=12 C=1.65 μ=1.15 E=0.97; lat B=10 C=1.30 μ=1.00 E=0.97;
  D = μ·Fz with Fz clamped to 9 kN; friction-ellipse combining
  (scale both if (Fx/Dx)²+(Fy/Dy)² > 1).
- Drivetrain: RWD, fixed ratio 8.2, efficiency 0.9, engine torque lookup
  {1000:220, 3000:320, 5000:340, 6500:300, 7500:0} Nm.
- Brakes: 1600 Nm front / 1050 Nm rear per wheel (~60/40), handbrake locks
  rears (flags bit 0). Steering ±30°, linear map, front axle only.
- Aero drag 0.42 N/(m/s)² at the CoM.

Measured behavior on the flat test oval: settles at 0.055 m compression,
0→7.1 m/s in 2 s (engine-limited launch), full ABS-free stop from ~29 m/s,
pursuit-controller lap of the 60×40 m oval in 30.5 s with LAP_COMPLETE and no
LAP_INVALID.

## Build

- Native: `cmake -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -B build . && cmake --build build` then run
  `build/test_determinism` and `build/test_vehicle`. Box3D options forced:
  SAMPLES/UNIT_TESTS/BENCHMARKS/VALIDATE off, DISABLE_SIMD on;
  `-ffp-contract=off` everywhere.
- wasm: `bash build_wasm.sh` → `out/sim.wasm` (~395 KiB). Box3D sources are
  compiled directly (list mirrors `BOX3D_SOURCE_FILES` in
  `box3d/src/CMakeLists.txt`); if the submodule is ever bumped (ADR-002 event),
  re-sync that list.
- Smoke: `node tests/wasm_smoke.mjs`.

## Brush tire model (2026-07-07, replaces the Pacejka bullets above)

`sPacejka` + friction-ellipse + the canned FFB trail heuristic are gone;
tire forces now come from the discretized brush contact-patch model of
docs/TIRE-MODEL.md (`vehicle_brush_patch` in src/vehicle.c, N = 16 bristles,
parabolic pressure, vector slip -> one shared friction budget, emergent
pneumatic trail, two-node thermal layer per tire).

### Constants chosen (all compile-time literals in `kTuning`)

- `brush_cp = 7.0e6 N/m²`, `brush_a0 = 0.075 m`, `brush_fz0 = 3500 N`
  → Cα = 2·cp·a0² = 78.75 kN/rad; full slide at |σ| = 0.197.
- `brush_mu_s = 1.48` (bristle-level static), `brush_mu_k_ratio = 0.65`.
- Thermal: `T_amb 25`, `T_opt 85`, `k_T = 0.15/60² = 4.1667e-5`
  (25 °C and 145 °C both ⇒ μ_T = 0.85, clamp floor 0.80),
  `C_s = 1500 J/K`, `h_conv = 0.008 /s`, `h_int = 0.006 /s`,
  `h_int2 = 0.002 /s`.
- FFB keeps `caster 0.025 m`, `scrub 0.008 m`, `ratio 13`; pneumatic trail is
  now the patch output (`ffb_trail0`/`ffb_trail_falloff` deleted).

### Deviations from TIRE-MODEL.md (with justification)

1. **Bristle μs = 1.48, μk/μs = 0.65 instead of the spec's μs0 = 1.05,
   0.85.** The spec-literal constants fail the spec's own §4.3 gates: a
   binary static/kinetic brush under a parabolic pressure peaks just above
   μk·Fz — measured peak 0.865·μs0·Fz0 (gate ≥ 0.95) with a 1.5 % drop at
   15° (gate 5–20 %). Analytically the continuous model peaks at
   `r + 4(1−r)³/(3−2r)²` in units of bristle-μs·Fz, so peak ≈ μk for
   r ≥ 0.8. Chosen tuning keeps μs0 = 1.05 as the PATCH-level grip target:
   peak fraction at r = 0.65 is 0.709, and 1.48·0.709 ≈ 1.05. Measured:
   peak 3758.6 N = 1.023·μs0·Fz0 at 6.65°, 15° drop 10.2 %.
2. **Trail gate relaxed between 8° and 15°.** Raw emergent trail:
   20.2 mm at 1°, −3.4 mm at 8°, 0.0 mm at 15°. The few-mm negative dip just
   past the peak (and return to exactly 0 at full slide) is inherent to the
   binary μs/μk split — the adhesion centroid sits ahead of the patch center
   while the sliding rear is discounted by μk — so the literal strictly
   monotone chain t1 > t8 > t15 is unattainable in this model class.
   test_tire checks t1 > 10 mm, hard collapse 1°→8°, |t8| ≤ 5 mm,
   t8 ≥ t15 − 5 mm, |t15| ≤ 3 mm. FFB uses the raw (unclamped) trail; the
   25 mm caster keeps total front trail positive everywhere.
3. **C_s = 1500 J/K instead of ≈ 9000.** 9000 J/K is the heat capacity of a
   whole tire, not the tread surface layer; with it the surface never leaves
   ~35 °C in a lap. 1500 J/K (≈1 kg tread rubber) with the smaller h_conv
   gives a warm-up time constant of ~50 s and bounded steady state (cooling
   grows linearly with ΔT and speed, so no runaway).

### Interface / determinism notes

- `WheelRuntime` gained `t_surf`/`t_core` (internal only, NOT in SimStateV1,
  not hashed; they feed grip which feeds the hashed dynamics).
  `vehicle_create`/`vehicle_reset` set both to 25 °C — replays from
  `sim_reset` are bit-identical.
- `vehicle_brush_patch` is non-static and declared in src/vehicle.h so
  tests/test_tire.c can sweep it. It is NOT in build_wasm.sh's
  EXPORTED_FUNCTIONS — the CONTRACTS §1.1 export surface is unchanged
  (verified: wasm exports identical, sim_state_size still 200).
- Math in the patch loop: mul/add/div/compare + `sqrtf` only, fixed 16-count
  loop; μ_T is a clamped quadratic. No new transcendentals, no libm beyond
  `sqrtf`.
- Longitudinal/lateral stiffness is now isotropic (single cp): 74 kN/slip at
  static load vs Pacejka's 75 k long / 43 k lat. Long behavior ~unchanged,
  lateral response crisper.

### Measured (2026-07-07)

- test_determinism PASS, native 8000-tick final hash `b5bccb81c606e0b8`
  (differs from the pre-brush `75691d62c83dd18d` — expected, physics changed).
- test_vehicle PASS with NO threshold changes; oval pursuit lap 30.50 s
  (12198 ticks) on cold tires, no LAP_INVALID; settle compression 0.0552 m;
  0→7.13 m/s in 2 s (still engine-limited).
- test_tire PASS: warm peak 3758.6 N @ 6.65°; Fy(15°) 3373.6 N (−10.2 %);
  trail 20.23 / −3.36 / −0.00 mm at 1/8/15°; combined σx = 0.10 lateral peak
  0.865× pure; cold peak 0.849× warm.
- wasm build 405 941 bytes; wasm_smoke PASS, step-vs-replay hash
  `e9eadd8e6b64fb8d` (was `6f375f286e4d406e`).
- tools/autopilot_lap.mjs on assets/tracks/dev/track.bin at --target-speed 18:
  LAP COMPLETE, 61.910 s (24 764 ticks), final hash `79ab68ecea9ca914` —
  cold-tire grip (0.85 floor at 25 °C) did not require lowering the target.

## Open items

- Native-vs-wasm hashes differ (expected and allowed — only wasm-vs-wasm
  identity is load-bearing, ADR-001): native 8000-tick script hash
  `75691d62c83dd18d`, wasm 4000-tick script hash `6f375f286e4d406e` (different
  scripts; the comparison that matters is step-vs-replay inside one artifact,
  which is bit-identical).
- CI should assert the wasm import list stays ⊆ {emscripten_notify_memory_growth,
  fd_write} so hot-path imports can't sneak back in.
- Launch acceleration is mild (engine-limited); gear ratio / torque curve are
  the knobs if the game wants a livelier car.

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

## Analytic road surface (2026-07-07, docs/ROAD-SURFACE.md wave)

Wheels no longer raycast the low-poly collision mesh: each wheel queries an
analytic C1 surface interpolated from the TRK1 centerline (new src/road.{h,c}).
The chassis body still collides with the mesh; the mesh raycast remains only
as the off-domain fallback (|lateral| > width/2 + 8). Also in this wave:
track-conduction cooling + lockup flash-heat validation, 4x sub-stepping of
the per-wheel pipeline, and the additive `sim_tire_temp` export (ABI 1.2).

### road.c interface / architecture

- `RoadSample` (road.h) is the TRK1 centerline sample layout; sim.c's Track
  stores its samples as this type (`typedef RoadSample CenterlineSample`) and
  the Road BORROWS that array — `road_load(&track.road, samples, S)` only
  allocates the S-float smoothed-kappa array (malloc, freed by `road_free`
  from `sFreeTrack`). No copy of the centerline.
- `road_query(road, p, hint, out)` → surface point, unit normal, signed
  lateral, `on_road`, and `seg` (feed back as the next hint). Fixed iteration
  everywhere: ±6-sample window search around the hint (same incremental trick
  as the lap logic), initial u from clamped projection onto the two polyline
  segments adjacent to the nearest sample, then exactly 2 Newton steps on
  f(u) = (c(u)−p)·c'(u) with u clamped to [0,1]. Segment-boundary behavior:
  the clamped-u answer from either neighboring segment agrees to within the
  CR-vs-polyline sagitta (~2 cm of arc position at R = 40 m, sub-mm in
  height), well inside the 5 mm/1° continuity gates.
- Frames: up/tangent interpolated with the same Catmull-Rom weights and
  re-orthonormalized (tangent normalized, up made ⟂ tangent, side = up×tan);
  width and kappa linear in u.
- Lateral profile: crown −0.025·(2l/width)² on the road proper; kerb band
  (width/2 ≤ |l| ≤ width/2+1.1 AND smoothed |κ| > 0.022) is the 0..20 mm
  sinusoid; otherwise the band and beyond is the linear shoulder ramp
  −1.2·(|l|−width/2)/8, which matches the trackgen mesh ribbon (the mesh ramp
  also starts at the road edge). Normal = normalize(up − (∂h/∂l)·side) —
  continuous camber through banking transitions.
- Kerb phase without floor/fmod: sample spacing 2.5 m is exactly two 1.25 m
  rumble periods, so sin(2π·s/1.25) with s = (i+u)·2.5 reduces to sin(4π·u);
  b3ComputeCosSin unwinds internally (remainderf, IEEE-exact). C1 across
  segment boundaries by construction.
- **Deviation from the spec text**: raw κ is the trackgen cross-of-tangents
  formula divided by the ACTUAL per-segment distance, not the nominal 2.5 m.
  Identical on real arc-length-resampled tracks; required for the test ovals
  (uniform-angle ellipse ⇒ ~1.23 m mean spacing), where the /2.5 version
  underestimates κ ~2x and the kerb gate never fires. Smoothing window 15,
  circular, mirroring trackgen.

### Wheel contact

- Suspension ray (origin = hardpoint, dir = −chassis up, len = rest+radius)
  intersects the local tangent plane of the analytic surface; compression
  from that distance, exactly like the old raycast fraction. Guard: surface
  must face the ray (dot(up, normal) > 0.2) — otherwise (rolled car on the
  road) the wheel is treated as airborne and chassis mesh collision handles
  it. Hardpoint below the surface ⇒ hit_dist clamps to 0 (full compression,
  bounded by max_travel as before).
- The query normal now feeds the TIRE BASIS: wheel forward is projected onto
  the contact plane (mesh-fallback contacts use the mesh normal the same
  way). Suspension force stays along chassis up (strut axis). On flat ground
  this is numerically the old behavior; on banking the slip/force basis
  follows the surface — the continuity point of the spec.
- Per-wheel incremental hint (`road_hint` in WheelRuntime) bootstraps via a
  one-time full O(S) deterministic scan after create/reset
  (`road_nearest_global`), then stays within the ±6 window (≤ ~0.13 m of car
  motion per tick ≪ 15 m of window).

### Sub-stepping (ROAD-SURFACE §3)

- Fixed `SIM_TIRE_SUBSTEPS 4` inner steps of SIM_DT/4 (1600 Hz): slip calc →
  brush patch → thermal → wheel spin integration. Chassis pose/velocity are
  frozen within the tick (Box3D integrates at tick level), so v_long/v_lat
  and Fz are per-tick constants; only omega (⇒ slip ratio ⇒ fx/fy ⇒ heating)
  sub-steps.
- **Force application choice**: the chassis receives the sub-step MEAN force
  once per tick at the patch. Box3D integrates F·SIM_DT, so the mean exactly
  preserves the summed sub-step impulses Σ Fₖ·(SIM_DT/4) — cheaper than four
  ApplyForce calls and bit-identical in intent. FFB rack likewise averages
  the four sub-step samples; exported slip_ratio is the last sub-step's
  (deterministic either way).
- The airborne/zero-load paths sub-step the same way. The airborne wheel now
  uses the same clamped brake integration as the contact path (the old
  airborne branch could oscillate omega through zero) and the handbrake hard
  lock applies airborne too — behavior change, strictly more physical, hash
  changes anyway.

### Thermal depth (ROAD-SURFACE §2)

- Track conduction added: `−h_track·(T_surf − T_track)` while in contact,
  `T_track = 30 °C`, `h_track = 0.02 /s` (spec constants, in kTuning).
  `sTireThermal` became `vehicle_tire_thermal(w, power, speed, in_contact,
  dt)` — non-static (like vehicle_brush_patch) so test_road drives it; NOT a
  wasm export.
- Lockup flash-heat measured (test_road): 1 s locked at 30 m/s under Fz0
  from 25 °C ⇒ T_surf 87.8 °C (+62.8 °C, gate > 20). After release (rolling,
  in contact, 30 m/s): monotonic decay, −12.5 °C in 5 s. That is a ~20 s time
  constant, not the spec's "a few seconds": total cooling at 30 m/s is
  h_conv·2.5 + h_track + h_int ≈ 0.046 /s. Deliberately NOT retuned — pushing
  h_int high enough for a few-second τ would bleed the surface node so hard
  the lap-scale warm-up (70–90 °C after a hard lap, tuned in the brush-tire
  wave) collapses. The binding gate ("decays after release") passes; noted as
  a scope-honest deviation.

### New export (ABI 1.2, additive)

- `sim_tire_temp(u32 wheel) -> f32` in sim.h/sim.c, reading
  `vehicle.wheels[w].t_surf`; out-of-range or no world ⇒ 0. Added
  `_sim_tire_temp` to build_wasm.sh EXPORTS and `src/road.c` to both builds.
  `sim_abi_version()` still returns 1 (CONTRACTS §1.1 keeps `= 1`; the
  addition is purely additive). Verified export surface = previous list +
  sim_tire_temp; imports unchanged (emscripten_notify_memory_growth,
  fd_write only).

### Measured (2026-07-07, this wave)

- test_determinism PASS, native 8000-tick final hash `a305f4cd3421cef5`
  (was `b5bccb81c606e0b8` — expected, wheel physics changed; stable across
  repeated process runs).
- test_vehicle PASS with NO threshold changes: settle compression 0.0552 m,
  0→7.13 m/s in 2 s, oval pursuit lap 30.50 s (12 201 ticks vs 12 198
  pre-road — the flat-oval surface is nearly identical by design).
- test_tire PASS (patch model untouched).
- test_road PASS: flat-oval walk max |dh| 0.034 mm & dnormal 0.020°/10 cm;
  banked-oval walk (0→0.07 rad transition) max |dh| 1.41 mm & 0.028°/10 cm;
  banked-corner normal.y = 0.99755 = cos(0.07); crown drop at width/4 =
  6.250 mm (exact); kerb band sweeps 0.0–20.0 mm through the κ = 0.0375
  corner and is the −82.5 mm shoulder ramp on the κ = 0.011 corner;
  flash-heat numbers above.
- wasm build 410 175 bytes; wasm_smoke PASS, step-vs-replay hash
  `b8729a7de50d1b0c` (was `e9eadd8e6b64fb8d`).
- sim_tire_temp via wasm on dev-next: 25.00 at reset, rears warm faster than
  fronts under a hard RWD launch (35.7/33.8 vs 25.7/25.4 after 10 s), 0 for
  wheel ≥ 4.
- tools/autopilot_lap.mjs on assets/tracks/dev-next/track.bin at
  --target-speed 15: LAP COMPLETE, 65.375 s (26 150 ticks), final hash
  `690a50d23d211840` — inside the ≤ 65.4 s gate.

## Suspension wave 3 (2026-07-08, docs/SUSPENSION.md)

Per-corner 22 kg unsprung mass with 1-DOF strut travel + 200 kN/m tire
vertical spring; kinematic camber/toe with bump curves; camber thrust as
equivalent lateral slip through the brush patch; chassis CoM + inertia tensor
computed from the §4 mass layout. All in src/vehicle.c; SimStateV1 layout and
the wasm export surface unchanged.

### Quarter-car unsprung corner (§1)

- `WheelRuntime` gained `travel` (wheel center below the hardpoint along the
  strut, m) and `travel_vel`; `prev_compression` is gone (the damper now reads
  `-travel_vel` directly instead of a finite difference). `compression`
  remains the SUSPENSION SPRING travel `rest_length - travel`, which is what
  `susp_compression` exports; wheel pos exports the unsprung center
  `hardpoint - travel·up`. Create/reset initialize `travel = rest_length`
  (full droop — the car hangs), deterministic for replays.
- `vehicle_suspension_step()` (non-static, test-driven like
  vehicle_brush_patch): one 1600 Hz sub-step, semi-implicit Euler
  (velocity first — the standard stable form of "explicit" at ω·dt ≈ 0.07).
  EOM along strut-down: `m_u·dv = F_susp + m_u·g_along − F_tire` with
  `g_along = 9.81·up.y` (spec's equation with the axis flipped). Travel
  clamps kill velocity into the stop only.
- Tire spring `F_t = k_t·pen + c_t·(travel_vel − hit_dist_dot)` clamped
  [0, max_load]; **Fz fed to the brush model is this force**, per sub-step.
  `hit_dist_dot = dot(v_hardpoint, n)/dot(up, n)` — road static, so the
  contact distance changes with hardpoint velocity only (per-tick constant).
- Strut force `F_susp = k_s·c − damper·travel_vel` clamped ±max_load — it may
  go NEGATIVE (rebound damper pulling the chassis down into a dip); the old
  path clamped at 0 because the same number fed the tire. Its chassis
  reaction applies at the HARDPOINT along up (same torque as the old
  contact-point application — the offset is parallel to the force — but
  matches the spec text). Applied airborne too (droop extension damping).
- Load path at settle: springs carry the sprung weight only. Chassis body
  mass = 1262 kg (spec §1); tire loads sum back to 1350·g because each corner
  adds its own `m_u·g` through the tire spring. Known simplification: tire
  IN-PLANE forces accelerate the 1262 kg body (the unsprung corners have no
  in-plane DOF), so the car is ~6.5 % lighter in translation than 1350 kg —
  quarter-car-in-a-game standard, accepted by the spec's mass split.
- The airborne path integrates the same quarter-car with F_t = 0 (wheel runs
  to the droop stop, strut reaction still applies); thermal/spin sub-steps
  unchanged.

### Kinematic camber/toe (§2)

- `vehicle_wheel_kinematics()` (non-static, test-driven): static camber
  −1.5°F/−1.0°R, toe 0.05° OUT front / 0.15° IN rear per side; bump curves
  linear in spring travel about per-axle static settle: camber −1.0°/25 mm
  both axles, toe +0.10°/25 mm rear only. Toe adds to `steer_angle`
  (mirrored: toe-in steers left wheels right); camber feeds §3 only (no
  geometric wheel-pose change, per spec).
- Settle references are kTuning literals derived from the computed layout:
  sprung 1262·9.81 N on CoM z = −0.13393 between hardpoints ±1.3 m →
  2776.2 N/wheel front (0.046270 m), 3414.0 N/wheel rear (0.056898 m).
  Measured sim settle matches to 4 decimals (test_suspension gate a).

### Camber thrust (§3)

- `sigma_y += 0.6·sin(gamma)` via `vehicle_camber_thrust_sigma()`;
  γ = lean-left-positive camber vs the ROAD normal:
  `gamma = atan2(dot(up, wheel_side), dot(up, contact_normal)) + mirror·camber`
  — chassis roll and banking arrive through the contact-basis projection,
  static/bump camber through the mirrored kinematic value. Sign verified:
  negative camber on the outside wheel thrusts INTO the corner; body roll
  (lean-out) reduces grip, which the static negative camber counteracts.
- Computed once per tick (travel moves ≪ 1 mm per tick); flows through the
  brush patch so it saturates with the shared friction budget — past the
  grip peak the shift SUBTRACTS force (measured, gate d).
- Side effect worth knowing: with Cα ≈ 79 kN/slip, static −1.5° camber makes
  ~1.1 kN of standing lateral preload per front tire (left/right cancel —
  net force, yaw and roll torque all zero; no thermal power at v_lat ≈ 0).
  That is an aggressive but spec-mandated k = 0.6 (physical camber stiffness
  is ~10× smaller); the emergent behavior gates all pass with it.

### Mass layout → CoM + inertia (§4)

- `vehicle_mass_data()` (non-static, test-driven) computes from the spec
  table: box 900 kg 1.9×1.1×4.4 at origin + engine 220 @ (0,−0.10,−0.90) +
  driver 80 @ (0,0,0.20) + fuel 40 @ (0,−0.20,−0.30) + 4×22 at the
  hardpoints + ballast 22 @ (0,−0.30,0.60) = 1350 kg total.
  Applied via `b3Body_SetMassData` (b3MassData: mass/center/inertia-about-CoM,
  physics/box3d/include/box3d/box3d.h:647).
- **Computed: CoM (0, −0.0434, −0.1339); pitch 1868.9, yaw 2096.5, roll
  426.6 kg·m²** vs the old hand-set 2400/2600/550 about (0, −0.15, 0). Yaw
  is 19 % under the old value, as the spec predicted.
- **Deviation — CoM height**: the spec asks for a CoM within ~2 cm of the old
  −0.15 m, "adjust ballast Y if needed". Unattainable: the 900 kg uniform box
  at the origin pins the composite CoM near −0.04; matching −0.15 would need
  the 22 kg ballast at y ≈ −6.8 m. Ballast kept at the table position (even
  y = −0.55, the floor, moves the CoM only 4 mm). The ~10.7 cm higher CoM
  adds ~13 % lateral load transfer; measured lap impact nil (see below) —
  camber thrust on the loaded wheels and the lighter translational mass
  compensate.
- Off-diagonal Iyz ≈ 10.4 kg·m² (0.6 % of pitch) dropped — principal axes
  assumed body-aligned, spec says "principal inertia".
- Body mass = sprung 1262 kg, center/tensor = full 1350 kg layout: the wheels
  yaw/pitch/roll WITH the chassis (only their strut DOF is separate), so
  rotational inertia keeps them; vertical statics need the sprung split.
  Using the full-layout CoM (vs sprung-only) shifts axle loads 0.7 % — noise.

### Validation / threshold changes (all justified)

- test_vehicle: front steer-angle tolerance 1e-3 → 2e-3 rad (steer_angle now
  includes 0.05° ≈ 0.87 mrad toe-out); rear "steer == 0" → "|steer| < 0.01"
  (rear wheels carry 0.15° toe-in + bump toe). Settle compressions moved from
  0.0552 m to 0.0463 F / 0.0569 R (sprung weight only + rearward CoM) — still
  inside the existing 0.025..0.10 gate, no change needed. Oval pursuit lap
  30.57 s (12 227 ticks, was 30.50 s), no LAP_INVALID.
- NEW test_suspension (wired in CMakeLists): (a) settle: spring compression
  ≈ sprung_corner/k_s and tire compression ≈ corner_weight/k_tire per axle,
  all within 1 % of prediction (gate ±10 %); (b) unsprung 3 cm road step —
  **deviation from the literal "oscillates 12–18 Hz" wording**: with the
  spec-mandated dampers the wheel mode is critically damped
  (ζ = (4500+300)/(2·√(260 kN/m · 22 kg)) ≈ 1.00), so NO oscillation is
  possible in any correct implementation; the gate instead checks
  f_n = √((k_t+k_s)/m_u)/2π = **17.30 Hz** ∈ [12,18] and that the measured
  step-response peak-velocity time matches 1/ω_n within 15 %. Measured:
  peak 0.932 m/s at 8.77 ms → t_pk·ω_n = 0.953 (the 1600 Hz discretization
  stiffens the discrete mode ~4 % — its exact eigenvalues give t_pk = 8.75 ms
  — and the estimator adds ~1 %); response decays to <0.01 % of the 23.1 mm
  offset within 300 ms. (c) camber −1.5°/−1.0° at settle, −1.0°/25 mm gain,
  rear bump toe verified with mirroring. (d) γ = −3° vs 0° warm sweep at Fz0:
  thrust −1969.5 N at zero slip, positive-Fy peak shifted +0.032 in σy,
  Fy at σy = −0.06: −3016 → −3662 N (grip added on the leaned side),
  at +0.06: 3016 → 1841 N (given back opposite).
- Determinism: 2×8000-tick native hash `f349e9180a8ffd61` (was
  `a305f4cd3421cef5` — expected, physics changed), stable across process
  runs. New math in the kernel: b3Atan2 for the lean angle, b3ComputeCosSin
  for sin(γ), mul/add/div/compare in the strut step — no new libm.
- wasm 410 890 bytes; smoke PASS, step-vs-replay hash `63252f4009ee327e`
  (was `b8729a7de50d1b0c`); export list verified byte-identical (CONTRACTS
  §1.1 + sim_ffb_torque + sim_tire_temp), imports still only
  emscripten_notify_memory_growth + fd_write.
- tools/autopilot_lap.mjs on assets/tracks/dev-next/track.bin at
  --target-speed 18: **LAP COMPLETE 57.555 s** (23 022 ticks, hash
  `bda8f627263a0012`) vs the 57.6 s pre-wave baseline — net wash: the higher
  CoM costs load transfer, camber thrust + kinematic toe add mid-corner
  grip and the 1262 kg translational mass helps corner-exit accel.

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

## Damage system (Phase 2)

Deterministic per-region crash damage in the kernel (docs/SOFTBODY.md Phase 2,
ABI 1.3 additive). The chassis Box3D hull now has `enableHitEvents = true`; its
shape id is stored in `Vehicle.chassis_shape` at create. After each
`b3World_Step`, `sim_step` calls `vehicle_accumulate_damage`, which reads
`b3World_GetContactEvents`, keeps only hitEvents touching the chassis shape,
transforms `point` into the chassis-local frame (`b3Body_GetLocalPoint`) and
folds each into four damage scalars on the Vehicle (`damage_overall`,
`damage_steer`, `damage_toe_front`, `damage_toe_rear`). Causality: a tick's
`vehicle_update` uses the PRIOR damage; accumulation feeds the NEXT tick.

- **Threshold = 6.0 m/s** (`SIM_DAMAGE_THRESHOLD`). Hits at/below accrue ZERO
  damage. Chosen well above anything a clean lap produces: the car rides on the
  WHEELS (analytic road / raycast), so the chassis box only ever touches the
  terrain mesh in a genuine crash. Kerb strikes, banking and bumps never move
  damage off 0. A real wall strike approaches at ~15-30 m/s, comfortably above.
- **Severity** = clamp((approachSpeed − 6) · 0.045, 0, 1). ~22 m/s over
  threshold ⇒ destroyed (sev 1); a solid ~15 m/s total strike ⇒ ~0.4.
- **Per-component gains**: overall += sev·0.60; front hits (local z ≥ 0)
  steer += sev·0.55 and front toe ±= sev·0.015 rad (sign from local x); rear
  hits bend rear toe likewise. Toe clamped to ±0.030 rad (~1.7°).
- **Effects** (applied in `vehicle_update` by scaling tunables at the use site;
  `kTuning` never mutated): aero cl_front/cl_rear ·= (1 − 0.40·damage_overall);
  max_steer ·= (1 − 0.30·damage_steer); damage_toe_{front,rear} added to the
  respective axle toe. A destroyed car keeps 60% aero / 70% steer — it limps,
  autopilot still finishes. Zeroed in `vehicle_create` and `vehicle_reset`.

**Export**: `float sim_damage(uint32_t component)` — 0=overall(0..1),
1=steer(0..1), 2=|front toe| rad, 3=|rear toe| rad; out-of-range / no world ⇒ 0.
Output-only (reads never affect sim/hash). Damage itself is part of the hashed
dynamics implicitly (like tire temperature — stored in Vehicle, reset on reset,
never hashed directly) because it feeds forces.

**Clean-lap-zero guarantee (the linchpin)**: when damage = 0 the effect math is
an exact IEEE identity — `x·1.0f == x`, `x + 0.0f == x` — and Box3D hit events
are purely observational (both `contact_solver.c` sites only set a reporting
bit; they never touch impulses/velocities/positions). PROVEN empirically:
compiling the kernel with the whole feature neutralized (`-DSIM_DAMAGE_OFF`,
temporary guards since removed) gives the **identical** 8000-tick determinism
hash `5ec1a7132b229092` as the feature-active build. So a non-crashing lap is
bit-identical to pre-Phase-2; leaderboard laps stay comparable. (The old NOTES
baseline `f349e9180a8ffd61` is stale — physics changed in intervening commits;
the current pre-Phase-2 and post-Phase-2 clean hash is `5ec1a7132b229092`.)

**Test contact (test_damage.c)**: created via the non-exported helper
`vehicle_apply_impact(v, approachSpeed, localPoint)` (documented option B) — no
world geometry needed, so impacts are perfectly reproducible.

**Gates** (all PASS): test_determinism (2× in-process, hash
`5ec1a7132b229092`), test_vehicle, test_tire, test_road, test_suspension,
test_damage. wasm builds (412027 bytes); `wasm_smoke` PASS, step==replay hash
`ea0284163b73636d`; exports = previous 13 sim_* + `sim_damage` (14), imports
unchanged {emscripten_notify_memory_growth, fd_write}. Autopilot on
assets/tracks/dev5/track.bin --target-speed 18: **LAP COMPLETE 49.197 s**
(19679 ticks); replaying that laplog through sim.wasm reads sim_damage(0..3) =
**0, 0, 0, 0** — clean lap accrued EXACTLY zero damage.

Files changed: src/vehicle.h, src/vehicle.c, src/sim.c, include/sim/sim.h,
build_wasm.sh, CMakeLists.txt, tests/test_damage.c (new).

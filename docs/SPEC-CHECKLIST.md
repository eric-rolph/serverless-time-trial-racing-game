# Spec Checklist — verified every 5 iterations

Status: ☐ planned · ◐ implemented · ✅ verified (evidence linked)

Iteration counter and verification log at the bottom.

## 1. Physics engine (Box3D & C17)

- ✅ Box3D (Erin Catto, C17) incorporated — submodule pinned `540ea38`
- ✅ Headless 3D environment with vehicle — native tests drive a full closed-loop lap
- ✅ Pacejka Magic Formula tire model — B/C/D/E per axis, load-dependent D, friction ellipse (vehicle.c)
- ✅ Independent suspension geometry — per-corner raycast spring/damper, travel clamps
- ✅ Strict 400 Hz fixed timestep — `SIM_DT 0.0025f` single literal; test_determinism: 2×8000 ticks, hashes identical at all 17 checkpoints
- ✅ No determinism-breaking flags — `-ffp-contract=off`, no fast-math (CMakeLists + build_wasm.sh); Box3D scalar, single-threaded, hot-path clock import removed

## 2. Client (Rust & WebAssembly)

- ✅ Standalone Rust client — `sttr-client` (race/replay/submit), 29/29 unit tests
- ✅ Decoupled loop with accumulator — integer-nanosecond accumulator (DT_NANOS=2.5e6), exact tick counts under drift test
- ✅ α-blend state interpolation — pos lerp + hemisphere-corrected nlerp, tested
- ✅ Aspect-native projection — fov_y fixed, aspect from live surface (32:9 native); PresentMode::AutoNoVsync (uncapped)
- ◐→✅ High-frequency input polling — 1 kHz gilrs thread + keyboard fallback (hardware polling untested without a physical wheel on this machine)
- ✅ Cryptographic signing — ed25519-dalek over "sttr-lap-v1"‖LAPLOG, keypair in OS config dir, roundtrip tested

## 3. Edge validator (Cloudflare Workers)

- ✅ TypeScript Worker, WebSocket endpoint — DEPLOYED: sttr-referee.ericrolph.workers.dev
- ✅ 1. Ed25519 WebCrypto verification — live (bad sigs rejected in unit tests; live accept path proven)
- ✅ 2. Telemetry heuristics — 4 metrics; step-TAS rejected (3 flags), human-like noise passes, jitter-injected autopilot passed live with 0 flags
- ✅ 3. Wasm Box3D sim instantiated in memory — same binary as client (ADR-001)
- ✅ 4. Headless max-speed replay — 24,814 ticks replayed at edge; state hash + lap ticks matched claim (74 ms in wasmtime locally, 838× real-time)
- ✅ 5. KV leaderboard — live entry: autopilot-e2e 62.035 s rank 1, key scheme sorts by time lexicographically

## 4. Automated Game Master (GitHub Actions)

- ✅ CI/CD YAML — 5 jobs (worker, physics-native, physics-wasm, client, trackgen determinism)
- ✅ Python procedural generator — seeded Catmull-Rom spline + terrain ribbon; same seed ⇒ byte-identical track.bin (CI-enforced)
- ✅ Weekly cron — Mon 00:00 UTC: generate → Emscripten build (Box3D C17) → wrangler deploy globally → provenance commit; secret CLOUDFLARE_API_TOKEN set (cron unexercised until next Monday; workflow_dispatch available)

## Iteration log

| # | milestone | notes |
|---|-----------|-------|
| 1 | Repo scaffold, Box3D pinned, contracts + ADRs written | commit 3c21e46 |
| 2 | trackgen: seeded spline+terrain generator, TRK1 packer | dev track 935 m, hash 3fbe91d5a2ca5851 |
| 3 | worker: full referee pipeline + 17 unit tests green | tsc clean, vitest 17/17 |
| 4 | CI + weekly Game Master workflows; KV namespace provisioned | 427a0d7959b741f3bda621d63196711b |
| 5 | **VERIFICATION PASS #1** | see below |
| 6 | Physics kernel merged: native determinism + vehicle tests green, wasm smoke green | hash 6f375f286e4d406e stable |
| 7 | Client merged: fixed float-drift accumulator (→ integer ns), ribbon closure; 29/29 tests | cargo check + test clean |
| 8 | Worker deployed to edge; Blob binary-frame normalization fix | sttr-referee.ericrolph.workers.dev |
| 9 | LIVE E2E ACCEPTED: autopilot lap 62.035 s, edge replay hash match, KV rank 1 | + wasmtime replay = same hash (3-host determinism) |
| 10 | **VERIFICATION PASS #2** | see below |
| 11 | Web client shipped: browser game on Workers static assets (free), Gamepad API wheel support + calibration wizard, minimap, three.js renderer | in-browser autopilot lap ACCEPTED via full browser stack (noble-ed25519 + WS) |
| 12 | FFB research + integration design | docs/FFB.md — rack-torque model, DirectInput plan, ffb.toml schema |
| 14 | Track design wave: banking (4° max — 8° flip-tested and rejected; sign bug caught by autopilot regression) + rumble kerbs (2 cm alternating teeth, both edges, tight corners) + striped visuals; replay viewer `/api/replay` + ▶ race buttons; rollover auto-respawn | banked lap 56.530s < flat 57.407s (banking = grip, as physics demands); prod e2e accepted; replay endpoint 200/180 KB |
| 13 | Must-have wave 1 (docs/MUST-HAVES.md): ghost+delta+sectors, countdown, controller zero-config, WebHID FFB, audio, cameras, `sim_ffb_torque` ABI 1.1, VALIDATE_MODE=queue + sweeper | native hashes unchanged (75691d62…, 6f375f28…); prod e2e re-accepted with new binary |

### Verification passes (every 5 iterations)

**Pass #1 (iteration 5, 2026-07-05).** Re-read the full spec against the tree:

- §1 physics: Box3D pinned ✅; kernel in progress (physics subagent). 400 Hz +
  no-fast-math constraints encoded in CONTRACTS §1 and CI wasm job — enforcement
  pending kernel merge. Pacejka + suspension specified in ADR-007, pending code.
- §2 client: in progress (client subagent). Accumulator/α-blend, 1 kHz polling,
  signing all specified in CONTRACTS §§2–5.
- §3 worker: implemented ◐ — all 5 referee stages coded; verify/heuristics/
  leaderboard/protocol unit-tested (17/17). Replay stage needs real sim.wasm to
  be exercised (blocked on physics kernel). WebSocket + KV per contract.
- §4 game master: implemented ◐ — ci.yml (4 jobs incl. trackgen determinism
  check: same seed ⇒ byte-identical track.bin) + weekly-track.yml (cron Mon
  00:00 UTC: generate → emcc build → wrangler deploy → provenance commit).
  Unexercised until first CI run after push.
- Deviation check vs spec: none found. Gap list: end-to-end replay test
  (client-produced log accepted by worker) — planned as integration step;
  KV id now real; GitHub secret CLOUDFLARE_API_TOKEN set.

**Pass #2 (iteration 10, 2026-07-07).** Full spec re-read against evidence:

- Every §1–§4 item now ✅ (details inline above). Strongest evidence: the same
  24,814-tick lap log produces state hash `a816338aa9b6c534` on (a) node/V8
  sim_step-by-step, (b) Cloudflare edge V8 `sim_replay`, (c) Rust wasmtime —
  ADR-001's cross-platform determinism claim holds in practice.
- Live artifacts: Worker `sttr-referee.ericrolph.workers.dev` (rev 4), KV entry
  rank 1, GET /api/leaderboard returns it, GET /api/track/current serves TRK1,
  GET /api/sim/current serves the canonical binary (hash 5a45a4cbe4a24f45).
- Live negative test: tampered lap-time claim with a VALID signature was
  rejected `replay_mismatch/time_mismatch` — the edge replay, not crypto, is
  what catches lying clients.
- Honest residuals: (1) physical wheel hardware not exercised (no wheel attached
  during build; gilrs path is code-complete + keyboard fallback tested);
  (2) weekly cron fires next Monday — same steps as the manual deploy performed
  today, plus workflow_dispatch for on-demand runs; (3) interactive wgpu window
  not run headlessly here — mesh/camera/interpolation logic unit-tested;
  (4) Workers Paid plan required for full-length replay CPU budget.

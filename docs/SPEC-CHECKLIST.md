# Spec Checklist — verified every 5 iterations

Status: ☐ planned · ◐ implemented · ✅ verified (evidence linked)

Iteration counter and verification log at the bottom.

## 1. Physics engine (Box3D & C17)

- ☐ Box3D (Erin Catto, C17) incorporated — submodule pinned `540ea38`
- ☐ Headless 3D environment with vehicle
- ☐ Pacejka Magic Formula tire model
- ☐ Independent suspension geometry (per-corner raycast spring/damper)
- ☐ Strict 400 Hz fixed timestep (`DT = 0.0025f`, defined once)
- ☐ No determinism-breaking compiler flags (no `-ffast-math`; `-ffp-contract=off`)

## 2. Client (Rust & WebAssembly)

- ☐ Standalone Rust client
- ☐ Decoupled loop with accumulator; physics 400 Hz, render uncapped
- ☐ α-blend state interpolation for rendering
- ☐ Aspect-native projection (32:9 supported natively)
- ☐ High-frequency raw input polling (steering/throttle/brake), ~1 kHz thread
- ☐ Cryptographic signing module: hash+sign input log before transmission

## 3. Edge validator (Cloudflare Workers)

- ☐ TypeScript Worker, WebSocket submission endpoint
- ☐ 1. Verify cryptographic signature (Ed25519 WebCrypto)
- ☐ 2. Telemetry heuristics: micro-jitter / hardware-noise analysis, TAS flagging
- ☐ 3. Instantiate wasm-compiled Box3D sim in memory
- ☐ 4. Headless max-speed replay validating final lap time
- ☐ 5. Post validated results to global leaderboard via Cloudflare KV

## 4. Automated Game Master (GitHub Actions)

- ☐ CI/CD YAML workflow
- ☐ Python procedural track generator (3D spline + terrain mesh)
- ☐ Weekly cron: generate → compile into Box3D env → Emscripten wasm build → deploy Worker globally

## Iteration log

| # | milestone | notes |
|---|-----------|-------|
| 1 | Repo scaffold, Box3D pinned, contracts + ADRs written | commit 3c21e46 |
| 2 | trackgen: seeded spline+terrain generator, TRK1 packer | dev track 935 m, hash 3fbe91d5a2ca5851 |
| 3 | worker: full referee pipeline + 17 unit tests green | tsc clean, vitest 17/17 |
| 4 | CI + weekly Game Master workflows; KV namespace provisioned | 427a0d7959b741f3bda621d63196711b |
| 5 | **VERIFICATION PASS #1** | see below |

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

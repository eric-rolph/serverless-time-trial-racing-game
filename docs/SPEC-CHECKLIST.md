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
| 1 | Repo scaffold, Box3D pinned, contracts + ADRs written | this commit |

### Verification passes (every 5 iterations)

_None yet — first pass due at iteration 5._

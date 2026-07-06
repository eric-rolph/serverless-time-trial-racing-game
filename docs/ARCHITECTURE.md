# Architecture — Serverless Time-Trial Racing

Edge-authoritative asynchronous sim racing: players race alone against the clock;
a Cloudflare Worker at the edge is the referee that decides whether a lap is real.

```
┌────────────────────────────── PLAYER MACHINE ─────────────────────────────┐
│  USB wheel/pedals ──1 kHz poll──► input thread ──latest sample──┐         │
│                                                                 ▼         │
│  ┌───────────────────── Rust client (native) ────────────────────────┐    │
│  │  accumulator loop:                                                │    │
│  │    while acc ≥ 2.5 ms:  quantize input → sim.wasm step (wasmtime) │    │
│  │                         append tick to LAPLOG                     │    │
│  │    α = acc/DT → render interpolated state (wgpu, uncapped fps,    │    │
│  │                 aspect-native projection → 32:9 just works)       │    │
│  │  on lap complete: Ed25519-sign LAPLOG → WebSocket submit          │    │
│  └───────────────────────────────────────────────────────────────────┘    │
└───────────────────────────────────┬────────────────────────────────────────┘
                                    │ wss (signed LAPLOG, ≤576 KiB)
                                    ▼
┌────────────────────────── CLOUDFLARE EDGE ────────────────────────────────┐
│  Worker (TypeScript)                                                      │
│   1. verify Ed25519 signature (WebCrypto)          → reject: bad_signature│
│   2. telemetry heuristics (jitter/smoothness/grid) → reject: heuristics   │
│   3. instantiate sim.wasm (same binary as client)                         │
│   4. sim_replay(log) at max compute speed                                 │
│      compare final state hash + lap ticks          → reject: mismatch     │
│   5. write KV: lb:{track}:{ticks}:{player}         → accepted + rank      │
│                                                                           │
│  KV LEADERBOARD ◄── GET /api/leaderboard   GET /api/track/current         │
└───────────────────────────────────▲────────────────────────────────────────┘
                                    │ deploy weekly
┌────────────────────────── GITHUB ACTIONS ─────────────────────────────────┐
│  cron (Mon 00:00 UTC): trackgen.py --seed $(date +%G%V)                   │
│    → track.json + track.bin → emcc build sim.wasm (Box3D C17)             │
│    → wrangler deploy (wasm + track bundled) → commit assets back          │
└────────────────────────────────────────────────────────────────────────────┘
```

## Subsystems

| dir | language | role |
|-----|----------|------|
| `physics/` | C17 (Box3D submodule) | deterministic 400 Hz kernel: rigid body, raycast suspension, Pacejka MF tires, track collision, state hash |
| `client/` | Rust | wasmtime host, decoupled game loop, wgpu renderer, HID polling, signing, submission |
| `worker/` | TypeScript | edge referee: verify → heuristics → replay → KV leaderboard |
| `trackgen/` | Python | seeded procedural spline + terrain → TRK1 binary |
| `.github/workflows/` | YAML | CI + weekly Game Master pipeline |

## Why this is cheat-resistant (and where it isn't)

Accepted lap ⇒ there exists an input sequence that, fed through the real physics,
produces that time — **speed hacks, teleports, and clipped checkpoints are
impossible** regardless of client modification, because the client is untrusted;
only the log is.

Remaining attack surface: tool-assisted input crafting (a bot that plays through
the real physics). Mitigated (not solved) by the telemetry heuristic gate —
see ADR-006 for the honest threat model.

## Determinism invariants (checked in CI)

1. Same LAPLOG replayed twice through `sim.wasm` ⇒ identical state hash.
2. `sim.wasm` built with `-ffp-contract=off`, no fast-math, no SIMD, no threads.
3. `DT = 0.0025f` literal; tick counts are integers; no wall-clock anywhere in kernel.
4. Kernel has no source of entropy: no `rand`, no `time`, no uninitialized reads
   (validated by ASan/UBSan native test builds).

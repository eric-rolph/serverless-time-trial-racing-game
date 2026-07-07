# Serverless Time-Trial Racing

Edge-authoritative asynchronous sim racing. You race alone against the clock on
your own machine; a Cloudflare Worker at the edge is the referee that decides
whether your lap was physically real — by re-simulating it.

**The core trick:** the physics engine (Erin Catto's [Box3D](https://github.com/erincatto/box3d),
C17) is compiled once to WebAssembly. Your client runs that exact binary via
`wasmtime` at a strict 400 Hz fixed timestep; the Worker runs the same bytes in
V8 at maximum compute speed. WebAssembly's bit-exact IEEE-754 semantics mean the
replay is deterministic down to the last bit — so the server never needs to trust
your client, only your input log.

```
wheel/pedals → Rust client (400 Hz sim + uncapped α-interpolated render)
             → Ed25519-signed input log (~400 KB)
             → wss://…/api/submit
             → Worker: verify sig → anti-TAS telemetry heuristics
                       → replay through the same sim.wasm → compare state hash
             → Cloudflare KV global leaderboard
```

Every Monday a GitHub Actions cron generates a brand-new procedural track
(spline + terrain, seeded by ISO week), compiles it against the physics kernel,
and redeploys the referee worldwide. Fresh leaderboard every week.

## Layout

| dir | what |
|-----|------|
| `physics/` | C17 deterministic kernel: Box3D (submodule) + Pacejka MF tires + raycast suspension, 400 Hz, wasm export surface |
| `client/` | Rust: wasmtime host, accumulator game loop, wgpu renderer (native 32:9), 1 kHz input polling, signing, submission |
| `worker/` | TypeScript referee: signature → heuristics → wasm replay → KV leaderboard |
| `trackgen/` | Python: seeded Catmull-Rom spline + terrain ribbon → TRK1 binary |
| `docs/` | ARCHITECTURE, CONTRACTS (binding formats/ABI), DECISIONS (ADRs), SPEC-CHECKLIST |

## Live deployment

- Referee: `https://sttr-referee.ericrolph.workers.dev` (`/api/leaderboard`,
  `/api/track/current`, `/api/sim/current`, `/api/submit` via WebSocket)
- Always race with the **canonical** binaries the referee validates against:
  `curl -O https://sttr-referee.ericrolph.workers.dev/api/sim/current` — a
  self-built `sim.wasm` from a different compiler version will replay-mismatch.

## Quick start

```sh
# 1. Generate a track
cd trackgen && pip install -r requirements.txt
python generate_track.py --seed 202627 --out-dir ../assets/tracks/dev
python pack_track.py ../assets/tracks/dev/track.json

# 2. Build the physics kernel to wasm (needs emsdk on PATH)
bash physics/build_wasm.sh && node physics/tests/wasm_smoke.mjs

# 3. Race
cd client
cargo run --release -- race --track ../assets/tracks/dev/track.bin \
  --sim-wasm ../physics/out/sim.wasm

# 4. Submit (after a completed lap writes lap.laplog)
cargo run --release -- submit lap.laplog --to wss://sttr-referee.<your>.workers.dev/api/submit
```

Worker dev/deploy:

```sh
cd worker && npm ci
npm run check && npm test
npx wrangler deploy   # needs CLOUDFLARE_API_TOKEN; assets/ staged by CI
```

## Honest threat model

- ✅ Impossible: speed hacks, teleports, checkpoint clipping, modified clients —
  an accepted lap *is* a physics-valid input sequence, re-simulated at the edge.
- ⚠️ Arms race: tool-assisted input crafting through the real physics. The
  telemetry gate (micro-jitter, jerk variance, value-grid analysis) raises the
  cost; flags are stored with every leaderboard entry. See ADR-006.
- Keyboard play works but will carry heuristic flags — this game is built for
  real wheels.

## Notes

- Replay validation takes seconds of CPU per submission → requires Workers Paid
  (30 s CPU budget). See `worker/wrangler.toml`.
- Determinism invariants and the full formats (LAPLOG, TRK1, SimStateV1, wasm
  ABI) live in [docs/CONTRACTS.md](docs/CONTRACTS.md).

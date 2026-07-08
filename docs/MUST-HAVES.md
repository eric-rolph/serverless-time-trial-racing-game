# Racing simulator must-haves

Status: ✅ shipped · 🚧 this iteration · ▢ backlog. Web-first: everything must
work in the browser client unless marked native-only.

## Input

| # | feature | status |
|---|---------|--------|
| 1 | Keyboard (slewed steering, drivable) | ✅ |
| 2 | Wheel + pedals via Gamepad API, per-channel calibration wizard | ✅ |
| 3 | Controller zero-config (standard gamepad mapping: stick + analog triggers) | ✅ |
| 4 | Wheel torque FFB in the browser — WebHID, Fanatec protocol (experimental) | ✅ shipped, awaiting hardware test |
| 5 | FFB in native client (DirectInput constant force, ffb.toml) | ▢ (docs/FFB.md design done) |
| 6 | Steering sensitivity / rotation-range setting + invert | ✅ |
| 6b | Physically-based tire model: discretized brush patch, emergent pneumatic trail (SAT/FFB), 2-node thermal + track conduction + lockup flash-heat, combined-slip budget (docs/TIRE-MODEL.md) | ✅ |
| 6c | Analytic C1 road surface for tires — decoupled from the low-poly visual/collision mesh: CR-interpolated frames, crown, smooth kerb profile, banking-continuous camber; 1600 Hz tire substepping (docs/ROAD-SURFACE.md) | ✅ |
| 6d | Tire temps on HUD (sim_tire_temp, ABI 1.2) | ✅ |
| 6e | Physics wave 3: unsprung mass (quarter-car), kinematic camber/toe curves + camber thrust, inertia tensor from mass layout | ▢ next |

## Driving & timing core

| # | feature | status |
|---|---------|--------|
| 7 | Deterministic 400 Hz physics, replay-validated laps | ✅ |
| 8 | Lap timer, checkpoint validation, corner-cut detection | ✅ |
| 9 | Standing start with 3-2-1 countdown | ✅ |
| 10 | Ghost car of your personal best (local, per track) | ✅ |
| 11 | Live delta-to-best readout | ✅ |
| 12 | Sector times (from checkpoints) with best comparison | ✅ |
| 13 | Off-track + rollover auto-respawn | ✅ |
| 14 | Setup/tuning options (tire pressure, wing, gearing) | ▢ (needs setup hash in LAPLOG to stay fair) |

## Presentation

| # | feature | status |
|---|---------|--------|
| 15 | 3D track + car rendering, chase camera, minimap | ✅ |
| 16 | Camera toggle (chase / hood) | ✅ |
| 17 | Engine + tire audio (WebAudio synth) | ✅ |
| 18 | Aspect-native rendering (21:9/32:9) | ✅ (native + web) |
| 19 | Proper car model / track-side objects | ▢ |
| 19b | Track banking + rumble kerbs (physical geometry + striped visuals) | ✅ |
| 20 | Replay viewer (any leaderboard lap as ghost — logs are public by design) | ✅ |

## Competition & infrastructure

| # | feature | status |
|---|---------|--------|
| 21 | Global leaderboard (KV), weekly track rotation | ✅ |
| 22 | Edge anti-cheat: signature + telemetry heuristics + wasm replay | ✅ |
| 23 | Cost middle-ground: `VALIDATE_MODE=queue` — free-plan Worker defers replay to a GitHub Actions sweeper (results in ≤30 min) | ✅ |
| 24 | Personal best persistence (local) | ✅ |
| 25 | Named seasons / multiple concurrent tracks | ▢ |

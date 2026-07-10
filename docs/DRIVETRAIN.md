# Drivetrain & Handling wave — binding spec

Two kernel waves executed as ONE pass (one hash-discipline cycle). This is a
**breaking physics change**: existing lap logs will not reproduce old times.
It deploys together with a track rotation so the leaderboard cut is clean.

## 1. Gearbox (sequential, no clutch pedal)

- **6 forward gears + reverse.** Overall ratios (incl. final drive), tuned so:
  1st tops ~95 km/h @ redline with launch wheelspin available (drive force >
  rear grip in 1st/2nd), 6th tops ~270–280 km/h (power-limited: 178 kW vs
  0.42·v² drag ⇒ v_max ≈ 75 m/s). Starting point: 11.8 / 8.9 / 7.0 / 5.6 /
  4.4 / 3.5, reverse ≈ 1st. Refine against the torque curve.
- **Shift inputs are part of the deterministic input log** (CONTRACTS §2):
  flags bit 1 = shift_up, bit 2 = shift_down. The kernel **edge-detects**
  (rising edge = one shift request; previous-flags state lives in Vehicle).
  Replays are bit-honest by construction.
- **Shift dynamics**: upshift = ignition cut, torque 0 for a fixed 28 ticks
  (70 ms), then engaged in the next gear. Downshift = 48-tick (120 ms)
  engagement delay; on engage, engine speed snaps to the wheel-matched value
  (auto-blip illusion) with a brief deterministic driveline shock torque.
  **Downshift protection**: refuse any downshift that would put the engine
  above 7800 rpm. Shift requests during a shift in progress are dropped.
- **Reverse rule**: downshift from 1st engages R only when |v| < 1 m/s.
  Upshift from R goes to 1st (any speed; it's forward-safe).
- **Rev limiter**: hard fuel cut above 7500 rpm (drive torque = 0 while
  over), natural bounce through engine braking.
- **Implicit auto-clutch at low speed only** (no clutch modeling otherwise):
  below the speed where 1st-gear rigid coupling would drag the engine under
  1100 rpm, drive torque scales with rpm headroom so the car creeps and
  cannot stall. Deterministic, few lines.

## 2. Engine

- **One engine state ω_e** (rad/s), rigidly coupled to the mean rear wheel
  speed through the engaged ratio (except during shift cuts, where ω_e
  integrates freely under its own inertia ~0.15 kg·m² + engine braking).
  Kills the current "each rear wheel runs a private engine" model.
- **Engine braking**: closed throttle ⇒ negative crank torque, linear from
  −20 Nm @ idle to −60 Nm @ redline, fading to zero by 10% throttle.
  Applied through the drivetrain to the rear axle. Lift-off finally does
  something; coast decel stops being aero-only.
- **Idle 900 rpm floor** (engine never stalls; see auto-clutch).

## 3. Differential (rear, viscous LSD)

- Torque transfer T = k_lsd · (ω_left − ω_right), clamped to ±40% of the
  currently transmitted torque; applies on power AND coast (engine braking
  splits through the same coupling). Base split 50/50. k_lsd is a kTuning
  knob; tune for mild inside-wheel control on corner exit without locked-
  diff push.

## 4. New exports (ABI 1.4, additive)

- `float sim_rpm(void)` — engine rpm. Output-only, unhashed.
- `int32_t sim_gear(void)` — 0 = reverse, 1..6. Output-only, unhashed.
- Gear/ω_e are dynamics state like tire temps and damage: not in SimStateV1
  (its 200-byte layout is frozen), not hashed directly — divergence would
  surface in the chassis hash within ticks because gearing scales forces.

## 5. Handling wave (same kernel pass)

- **Brake bias → front-biased**: brake_torque_front 2200 Nm, rear 950 Nm
  (fronts lock first = stable failure mode; today the rears lock at ~70%
  pedal and fronts never can — snap oversteer built into every stop).
  Verify stopping distance stays within ±10% of current.
- **Tire load sensitivity**: μ_s(Fz) = μ_s0 · (1 − k_load·(Fz−Fz0)/Fz0),
  k_load ≈ 0.07, clamped to [0.7, 1.15]·μ_s0. Lateral load transfer now
  costs an axle grip, which makes roll stiffness a real balance lever.
- **Anti-roll bars**: add ARB rates (front/rear) to kTuning acting on the
  suspension's roll moment distribution — now meaningful thanks to load
  sensitivity. Tune front-stiffer for mild limit understeer, throttle-on
  oversteer available.
- **Slip relaxation length**: per-wheel first-order lag on slip quantities,
  L_relax ≈ 0.3 m (τ = L/|v|, clamped ≤ 50 ms at low speed), integrated at
  the 1600 Hz substep. Progressive on-center build-up instead of
  instantaneous force = the "buzzy-stiff" FFB becomes progressive.
- **Balance re-tune after all of the above** using the skidpad harness:
  target ~1.05–1.10 g steady-state with mild understeer at the limit;
  re-verify rollover threshold still > 1.25 g.

## 6. Client

- **input.js**: keyboard Q = down / E = up; gamepad LB(4) = down / RB(5) =
  up; wheel: shift paddles/levers are HID buttons — add a capture-to-map
  flow ("press your upshift now") in the calibration panel, persisted with
  the existing calibration in localStorage. Shift bits are momentary
  (rising-edge semantics live in the kernel; the client just reports state).
- **Auto-gearbox toggle** (default ON): client-side auto-shifter emits
  shift bits from sim_rpm() thresholds (up ≈ 7200, down ≈ 3800, with
  hysteresis + no shifting mid-corner above 0.6 lateral g if cheap to
  detect from state). It's still just input bits in the log — fully
  deterministic and replay-honest. Wheel users switch to MANUAL in config;
  the choice is announced in the status line.
- **HUD (Apex/Tufte, unboxed)**: gear numeral next to speed, thin RPM bar
  with redline tick, shift light (cyan pulse ≥ 7200 rpm).
- **audio.js**: update() consumes real sim_rpm() (null-guarded: falls back
  to the current speed-fake on old wasm). Shift cut = 70 ms gain dip +
  clunk one-shot; limiter = hard stutter at redline.
- **tools/autopilot_lap.mjs**: MUST learn to shift (read sim_rpm each tick,
  emit up/down bits at 7000/3000 with hysteresis) — the weekly-track CI
  raceability gate depends on the autopilot completing laps.

## 7. Verification gates

1. All existing native suites green (test_damage etc. unchanged).
2. New: test_drivetrain (ratio table, edge-triggered shifts, cut timing,
   limiter, engine braking sign, LSD transfer, reverse rule, shift-heavy
   log → step==replay bit-identical), load-sensitivity + relaxation tests.
3. wasm smoke: step==replay equality (the old golden hash is retired —
   breaking change; record the new one in NOTES.md).
4. Launch harness: 0–100 km/h ≈ 4–5 s with visible 1st-gear wheelspin;
   gear-speed table printed.
5. Skidpad: balance + rollover targets above.
6. Autopilot with auto-shift completes the current circuit; lap time drops
   vs single-gear baseline (it must — more speed on straights).
7. Live e2e submit accepted post-deploy.

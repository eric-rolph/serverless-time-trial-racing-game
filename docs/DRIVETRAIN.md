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

## 6.5 IMPLEMENTATION STATUS (2026-07-09, kernel wave — READ FIRST)

Sections **1–5 are implemented** in `physics/src/vehicle.c` (+ `sim.h`/`sim.c`
for the §4 exports) and gated by `tests/test_drivetrain.c`,
`tests/harness_launch.c`, `tests/harness_skidpad.c` and the extended
`tests/test_tire.c`. Section 6 (client) and the §7 autopilot **auto-shift**
gate are NOT part of this kernel wave; gate 7's fallback was verified instead:
the unshifted autopilot (stuck in 1st) still completes circuit1. Numbers and
hashes live in `physics/NOTES.md` ("Drivetrain & handling wave"). The
autopilot auto-shift has SINCE been implemented — see §6.7.

**Deviations from the letter of this spec (all deliberate, all measured):**

1. **Engine torque curve raised** to `{1000: 300, 3000: 350, 5000: 348,
   6500: 310, 7500: 0}` Nm (was 220/320/340/300/0). The old low end was tuned
   for the single 8.2:1 ratio; with the geared box it left the car
   torque-limited below ~4500 rpm, which contradicts this spec's own §1
   requirement (drive force > rear grip in 1st) and made the §7 0–100 gate
   unreachable (best 5.35 s). Peak power ~211 kW at 6500 (was ~204) — the §1
   power-limit arithmetic (v_max ≈ 75 m/s drag-limited) is preserved.
2. **Ratios**: −9.8 / 9.8 / 7.9 / 6.06 / 4.77 / 3.75 / 2.95 (vs the §1
   starting point). 1st = 9.8 tops 95.2 km/h at the limiter; 2nd = 7.9 is
   kept short so 100 km/h arrives without a second shift; 6th = 2.95 puts the
   drag-limited top at 272 km/h (the §1 set's 3.5 topped out at ~240 because
   the torque dive past 6500 cannot hold 0.42·v² at 7500-rpm gearing).
3. **`drag_coef` 0.42 → 0.37**: the quarter-car applies strut forces along
   chassis-up, so rear squat under downforce leaks ~0.05·v² of the support
   force into the horizontal (measured 185 N @ 63 m/s via a shift-cut coast).
   0.37 + 0.05 ≈ the §1 0.42·v² *total* resistance budget.
4. **Tire patch grip re-tuned up** (`brush_mu_s` 1.48 → 1.55, patch peak now
   1.071·μ_s0·Fz0, still inside TIRE-MODEL.md §4.3's [0.95, 1.10] gate): the
   §5 load sensitivity costs ~3–5% of in-situ grip under transfer; this is
   the §5-mandated "balance re-tune" that puts the skidpad at 1.08 g.
5. **Auto-clutch shape** (delegated by §1 "deterministic, few lines"): slip
   zone = coupled speed < 1100 rpm as specified; there the engine flares to
   `idle + throttle·(4500 rpm − idle)`, transmitted torque scales with the
   coupled speed's headroom toward lockup floored at 0.9 (a standing start
   pulls away at grip level), and a 4% creep throttle keeps the car creeping
   at idle. No engine braking through a slipping clutch (it would fight the
   creep and could push the car backward at rest).
6. **Downshift shock** (§1 "brief deterministic driveline shock torque"):
   retarding axle torque = 1.0 Nm per rad/s of rev-match snap delta, capped
   at 400 Nm, for 8 ticks (20 ms), split 50/50 across the rear axle opposing
   the rolling direction.
7. **LSD clamp** (§3 "±40% of the currently transmitted torque"): the
   transfer is clamped to 0.4·|T_axle| (total transmitted), so the wheel
   split can range 10%/90%. Zero transmitted torque (shift cut) = open diff.
8. **Simultaneous shift bits**: both bits rising in the same tick → up wins
   (deterministic tie-break). Requests during a shift in progress are
   dropped, per §1.
9. **Reflected crank inertia**: while rigidly coupled the rear wheels carry
   `I_wheel + ½·I_e·ratio²` each — spinning the engine up consumes real
   torque (~800 N equivalent in 1st), which is why the car short-shifts well.
10. **`sim_gear()` reports the outgoing gear during a shift** and returns 1
    (the spawn gear) with no world loaded.
11. **ARB implementation** (§5): `F = k_arb·(c_left − c_right)` added
    equal-and-opposite to the two strut forces of an axle inside the 1600 Hz
    substep — pure roll-moment redistribution. Front 20 kN/m, rear 14 kN/m
    (front-stiffer), k_lsd = 25 Nm/(rad/s): skidpad 1.083 g, front axle
    saturates first by ~2°, rollover threshold 1.47 g.

## 6.6 CLIENT PRESENTATION STATUS + shift-mode contract (2026-07-09)

The §6 presentation pieces owned by the HUD/audio builder are implemented in
`worker/public/js/sim.js` (`rpm()`/`gear()` accessors, null on pre-1.4
binaries), `worker/public/js/app.js` + `index.html` (gear numeral, RPM bar,
shift light, AUTO/MANUAL tag, keys hints) and `worker/public/js/audio.js`
(real-rpm engine, shift cut + clunk, limiter stutter — all falling back to the
old speed-fake when `sim_rpm` is absent). input.js (Q/E/G, paddle capture,
auto-shifter) and the autopilot auto-shift are a DIFFERENT builder's scope.

**Binding contract between input.js and the HUD's AUTO/MANUAL tag:**

- `localStorage["sttr-shift-mode"]` = `"auto"` | `"manual"`. Missing key ⇒
  `"auto"` (this spec's default-ON). input.js owns all writes (the G toggle);
  it should also announce changes on the status line per §6.
- If the live `Input` instance exposes a `.shiftMode` property
  (`"auto"`|`"manual"`), the HUD prefers it over storage every frame, so a
  toggle shows instantly without a storage read/write race. Absent property ⇒
  the HUD polls the localStorage key.

## 6.7 AUTOPILOT AUTO-SHIFT STATUS (2026-07-09, `tools/autopilot_lap.mjs`)

The §6 last bullet is **IMPLEMENTED**: the autopilot reads `sim_rpm()` every
tick and emits input-flag bit 1 (shift up, 0x2) at ≥ 7000 rpm / bit 2 (shift
down, 0x4) at ≤ 3000 rpm, with a 300 rpm re-arm hysteresis band, a minimum
200-tick gap between requests, and never both bits in one tick. The bits are
written into the LOGGED tick records before `sim_step` consumes them — the
log IS the input, replay-honest by construction. Null-guarded: on a
pre-ABI-1.4 wasm (`sim_rpm` absent) the flags stay 0 and the produced LAPLOG
was verified **byte-identical** (cmp) to the pre-shift tool's output.

**Deviation from the bare 7000/3000 threshold rule (deliberate, required):**

1. **A down-request is only emitted in gear ≥ 2.** The bare rule engages
   REVERSE on the very first tick of every run: at standstill the engine
   idles at 900 rpm (≤ 3000) in 1st with |v| < 1 m/s — exactly the §1
   reverse-rule window. Read via `sim_gear()`; if that export were ever
   absent the guard fails safe (assumes 1st, no downshifts).
2. Upshift requests are suppressed in 6th (a no-op request would burn the
   200-tick request budget for nothing).

Additive CLI flag `--verify-replay` re-feeds the produced log through
`sim_replay()` on a reset sim and demands the same final hash + lap ticks.
CLI defaults and the `LAP COMPLETE:` line are unchanged; at the weekly-CI
default `--target-speed 22` the rpm never reaches 7000 in 1st (22 m/s ≈
6240 rpm), so the gate's LAPLOG stays byte-identical to before this change.

Verified on circuit1 + staged `worker/assets/sim.wasm` (§7 gate 6):
**134.162 s** at `--target-speed 25` (gears 1st 15.7% / 2nd 84.3%, 3 up /
2 down, hash `ce4329864f1a0682`, step == sim_replay) vs the **171.363 s**
single-gear gate-7 baseline. The same target-speed with shifting disabled
does not even finish — the car rides 1st near the limiter, goes off at 56%
progress and wedges against a wall at 79%.

## 6.8 CLIENT INPUT-SIDE STATUS (2026-07-09, input wave — READ FIRST)

§6's input items are **IMPLEMENTED** in `worker/public/js/input.js`,
`worker/public/js/hid-input.js` and the calibration-panel region of
`worker/public/index.html`:

- **Shift buttons**: keyboard `Q` down / `E` up (all input paths),
  standard-mapped gamepad LB(4) down / RB(5) up, and mapped wheel-shifter HID
  buttons. The client reports **held** state into flags bits 1/2 via
  `quantize()`; edge detection stays kernel-side per §1.
- **Wheel shifter capture-to-map**: "MAP UPSHIFT: press it now" flow in the
  calibration panel. hid-input.js keeps the latest raw payload of every HID
  input report and reuses the axis detector's two-phase pattern — a 700 ms
  hold-still phase blacklists self-toggling bits (axis bytes, counters), then
  the first rising bit among quiet bits wins. Binding = `{id, reportId, byte,
  bit}`, persisted in the existing `sttr-input-cal-v2` calibration blob,
  shown in the panel with a re-map button.
- **AUTO gearbox, default ON**: pulses the same flag bits from `sim_rpm()` /
  `sim_gear()` — up ≥ 7200, down ≤ 3800, one ~20 ms pulse then a 250 ms
  cooldown (longer than any shift in progress) as hysteresis. Toggled by `G`
  and the panel's `#autoGear` checkbox, persisted per the §6.6 contract
  (`sttr-shift-mode`, live `input.shiftMode` getter), announced on the status
  line. Mapping a wheel shifter button switches the profile to MANUAL.

**Deviations / cross-file resolutions (all deliberate):**

1. **`worker/src/protocol.ts` flag mask widened `~0x0001` → `~0x0007`** (+
   tests in `worker/test/protocol.test.ts`; suite green). The referee's log
   parser still rejected any flag beyond bit 0 as "reserved flag bits set" —
   every lap driven with the §1 shift bits would have failed submission.
   CONTRACTS §2 already declares bits 1/2 valid (kernel wave); this is the
   referee catching up to the binding contract, not a new bit grant. Bits
   3–15 remain reserved-must-be-zero.
2. **`worker/public/js/app.js`, one line**: `input.attachSim(sim)` after
   `new Input()` — the auto-shifter needs an rpm/gear source and app.js owns
   the sim instance. `Input` also falls back to `globalThis.__sttr?.sim`, and
   stays inert (manual bits still work; kernel 1st-gear fallback drives) on a
   pre-1.4 wasm or when never attached.
3. **Downshift refuse-rule mirror**: the auto-shifter predicts post-downshift
   rpm with a mirrored ratio table (§6.5 item 2) and skips requests whose
   matched rpm exceeds **7600** (margin under the kernel's 7800 refusal)
   rather than spamming doomed log bits. Auto never selects reverse; shifting
   out of R is always manual.
4. **Mid-corner shift hold (§6 "if cheap to detect") NOT implemented**:
   lateral g isn't exported to the client; the wide 7200/3800 band plus the
   250 ms cooldown already prevents shift chatter. Revisit if a lateral-g
   export lands.
5. **Rig-path activation narrowed**: a calibration blob containing ONLY
   shifter-button bindings no longer activates the multi-device axis path
   (which would have dropped a standard pad's axes to keyboard fallback) —
   only bound AXIS channels do.

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

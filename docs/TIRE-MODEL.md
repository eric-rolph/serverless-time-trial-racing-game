# Physically-based tire model — discretized brush / contact-patch model

Replaces the Pacejka Magic Formula (ADR-007) with a first-principles model in
the spirit of rFactor 2's CPM and iRacing's NTM: instead of telling the tire
how to behave via a pre-recorded curve, we build the contact patch out of
mathematical bristles and let grip — and its loss — emerge from the forces.
(Scope honesty: this is a quasi-static brush model with a two-node thermal
layer, not a carcass FEM or polymer-chemistry simulation. It reproduces the
*behaviors* that matter for feel: load-dependent patch growth, combined-slip
friction sharing, the pneumatic-trail collapse that makes steering go light at
the limit, and temperature-dependent grip. Wear is out of scope for 1–3 minute
time trials.)

## 1. The brush model (per wheel, per 400 Hz tick)

The contact patch is a strip of `N = 16` bristle elements (fixed count —
determinism) spanning length `2a` along the rolling direction.

- **Patch half-length** grows with load: `a = a0 * sqrt(Fz / Fz0)`
  with `a0 = 0.075 m`, `Fz0 = 3500 N`.
- **Pressure distribution** is parabolic (standard brush assumption):
  `q(ξ) = (3 Fz / 4a) * (1 − ξ²)`, ξ ∈ [−1, 1] across the patch.
- **Slip vector** (dimensionless): `σ = (σx, σy)` with `σx = slip_ratio`
  (already computed from ω·r vs v_long) and `σy = tan(slip_angle)`
  (use `v_lat/denom` directly — it already is that tangent).
- A bristle entering the patch sticks to the road; by the time it reaches
  position `x` from the leading edge its deflection is `δ = σ · x`, giving
  force density `f_adh(x) = cp · |σ| · x` along σ̂ (bristle stiffness
  `cp` per unit length², tuned below).
- The bristle **breaks away** where adhesion exceeds the local friction
  budget: `cp·|σ|·x > μs(T)·q(x)`. From there to the trailing edge it
  **slides**: `f_slide(x) = μk(T)·q(x)` along σ̂, with `μk = 0.85·μs` —
  this static/kinetic gap is what makes the force *fall* past the peak
  rather than plateau.
- Sum the N elements: `Fx = Σ f·σ̂x·w`, `Fy = Σ f·σ̂y·w`, and the moment
  about the patch center `Mz = Σ f·σ̂y·(x − a)·w`.

**Pneumatic trail is not a formula — it's the output**:
`t_p = −Mz / Fy` (guard Fy ≈ 0). At small slip the adhesion force ramps
linearly toward the rear of the patch → centroid sits behind center → large
trail. As slip grows, the rear of the patch slides (force capped, centroid
marches forward) → trail shrinks → **Self-Aligning Torque collapses exactly
when front grip peaks**. That collapse IS the understeer cliff in the wheel.

Combined slip needs no friction-ellipse hack: longitudinal and lateral demand
share one budget through the vector treatment.

**Base friction**: `μs0 = 1.05` (scaled by the thermal factor below).
**Stiffness tuning**: `cp` such that cornering stiffness `Cα = 2·cp·a₀²`
puts the lateral peak near 6–8° slip at Fz0: `cp ≈ 7.5e6 N/m²` as a starting
point; validate with the sweep test (§4) and adjust.

## 2. Thermal layer (two nodes per tire)

State per tire: `T_surf`, `T_core` (°C), init at ambient `T_amb = 25`.

- Friction power into the surface:
  `P = |Fx·v_slip_x| + |Fy·v_slip_y|` where `v_slip_x = ω·r − v_long`,
  `v_slip_y = v_lat`.
- `dT_surf = [P/C_s − h_conv·(1 + 0.05·v)·(T_surf−T_amb) − h_int·(T_surf−T_core)] · dt`
- `dT_core = [h_int2·(T_surf − T_core)] · dt`
- Grip factor: `μ_T = clamp(1 − k_T·(T_surf − T_opt)², 0.80, 1.00)` with
  `T_opt = 85 °C`, `k_T` chosen so 25 °C (cold) ⇒ ≈ 0.85 and 145 °C ⇒ ≈ 0.85.
  Cold tires grip less and snap earlier; a lap of work brings them in.
- Constants (`C_s ≈ 9000 J/K`, `h_conv ≈ 0.04 /s`, `h_int ≈ 0.02 /s`,
  `h_int2 ≈ 0.006 /s`) are starting points — tune so the surface reaches
  ~70–90 °C after one hard lap (~60 s) and doesn't run away by lap three.

All state is plain floats advanced with `SIM_DT` — deterministic by
construction. Same input log ⇒ same temperatures ⇒ same grip ⇒ same hash.

## 3. FFB: SAT from the model, not a heuristic

Rack torque (existing `sim_ffb_torque()` export, same signature):

`rack = Σ_front [ Fy·(t_p_emergent + caster_trail) + s_i·Fx·scrub_radius ] / steering_ratio`

where `s_i` is the per-wheel mirror sign (+1 left, −1 right): the scrub moment
arm mirrors about the kingpin between the two front wheels, so symmetric
braking Fx produces equal and opposite kingpin moments that cancel at the rack
(only left/right brake-force asymmetry tugs the wheel).

Delete the old canned trail-collapse tunables (`ffb_trail0`,
`ffb_trail_falloff`); keep `caster_trail = 0.025 m`, `scrub_radius = 0.008 m`,
`steering_ratio = 13`. Kerb strikes already arrive through suspension load
spikes → patch load → force → rack. No canned effects anywhere.

## 4. Validation gates (all must pass)

1. Existing `test_determinism` (2×8000 ticks, identical hashes) — unchanged.
2. Existing `test_vehicle` sanity + closed-loop oval lap — retune the test's
   expectations only if physically justified (cold tires are slower).
3. NEW `test_tire`: pure-α sweep at Fz0, warm tire (force T_surf = T_opt):
   - `Fy` peaks between 5° and 9° slip angle;
   - peak `|Fy|` within [0.95, 1.10]·μs0·Fz0;
   - `Fy` at 15° is 5–20 % below peak (kinetic drop-off);
   - pneumatic trail at 1° > trail at 8° > trail at 15° ≥ 0-ish
     (monotonic collapse through the peak);
   - combined test: at σx = 0.10, lateral peak drops vs pure-α (budget sharing).
4. Wasm smoke + `tools/autopilot_lap.mjs` completes a lap on
   `assets/tracks/dev/track.bin` (use `--target-speed 18`; cold tires).

## 4b. Skidpad retuning (2026-07-08, after real-rig feedback)

Player report: "skids out too quickly above 75 km/h." Verified on a synthetic
60 m constant-radius skidpad (scratchpad harness, pure-pursuit radius hold,
0.125 m/s² speed ramp): breakaway at 83 km/h (0.88 g), terminal understeer,
**outside-front tire at 157 °C** — sustained cornering drove the loaded front
past the thermal cliff (μ floor 0.80) in ~90 s. Two root causes, two fixes:

1. **Thermal balance**: `C_s 1500 → 3000 J/K` (equilibrium arrives over
   minutes, not one lap), `k_T → 2.5e-5` with floor `0.88` (cold ≈ 0.91,
   worst-case hot ≈ 0.88 — a warning, not a cliff).
2. **Aerodynamic downforce** (it's a race car): `F = cl·v²` per axle,
   `cl_front = 1.1`, `cl_rear = 1.4 N/(m/s)²` (rear-biased for stability).
   ≈ +10 % grip at 23 m/s, +17 % at 30 m/s.

After: breakaway 89.9 km/h (1.04 g) cold or warm, outside-front peaks 125 °C,
still front-limited at the absolute limit (stable). Autopilot lap on the
archetype track: 65.4 s → 57.6 s at its original target pace.

## 5. Notes

- `sPacejka` and its tunables are removed with the ADR-007 force path;
  wheel-spin dynamics, brakes, drivetrain, suspension raycast stay as-is.
- SimStateV1 layout and all existing exports are UNCHANGED (ABI stays 1.x).
  Tire temperatures are internal state for now (HUD export is future work).
- Slip-angle/-ratio fields in SimStateV1 keep their meaning.
- Physics change ⇒ new binary ⇒ new lap times; the leaderboard rotates with
  the next track seed at deploy.

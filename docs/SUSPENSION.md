# Physics Wave 3 — unsprung mass, kinematics, camber thrust, inertia tensor

Binding spec. Completes the "suspension kinematics and mass distribution"
factor: isolate the unsprung corners from the sprung chassis, let geometry
change wheel angles with travel, and derive the inertia tensor from an actual
mass layout instead of hand-set principal values.

## 1. Quarter-car unsprung mass (per corner)

Add per-wheel unsprung mass `m_u = 22 kg` (wheel + brake + upright) with 1-DOF
travel along the strut axis (chassis-local −Y at the hardpoint):

- State per wheel: `travel` (m, wheel center offset below hardpoint measured
  along the strut) and `travel_vel` (m/s). Integrated at the existing 1600 Hz
  sub-step (explicit; ω_tire ≈ 15 Hz ⇒ ω·dt ≈ 0.06 — stable).
- **Tire vertical spring** replaces the direct kinematic contact: penetration
  `p = (road contact height along strut) − (wheel bottom)`;
  tire force `F_t = k_tire·p + c_tire·ṗ` clamped ≥ 0, with
  `k_tire = 200 kN/m`, `c_tire = 300 N·s/m`. `Fz` fed to the brush model is
  **this tire force** (not the suspension force) — kerb/bump load spikes now
  come from tire stiffness + unsprung dynamics, which is the entire point.
- Suspension spring/damper (existing k, dampers) acts between chassis and the
  unsprung mass; its reaction applies to the chassis at the hardpoint (as
  today). Unsprung equation of motion along strut:
  `m_u·(dv) = F_t − F_susp − m_u·g_along_strut`.
- Sprung mass drops to `1350 − 4·22 = 1262 kg`; total stays 1350.
- Travel clamps as today (max_travel); on clamp, kill travel_vel toward the stop.

## 2. Kinematic camber and toe

- Static setup: camber `−1.5°` front / `−1.0°` rear; toe `+0.05°` out front /
  `+0.15°` in rear (per side, conventional signs: toe-in = leading edge inward).
- Bump curves (linear in travel about static settle):
  camber gain `−1.0°` per 25 mm compression (both axles);
  bump toe `+0.10°` per 25 mm on the rear only (toe-in under load = stability).
- Toe adds to each wheel's steer angle (mind left/right sign); camber feeds §3.

## 3. Camber thrust in the brush model

Camber acts as an equivalent lateral slip contribution at the patch:
`σy_eff = σy + k_camber_thrust · sin(γ)` with `k_camber_thrust = 0.6`
(γ = wheel camber relative to the ROAD normal, i.e. chassis roll and road
banking included via the contact-basis projection). It flows through the same
bristle adhesion/sliding split — thrust saturates with everything else, no
bolt-on force. Sign such that negative camber on the outside wheel adds grip
INTO the corner.

## 4. Inertia tensor + CoM from a mass layout

Replace hand-set inertia with values computed from:

| item | mass | local position (x,y,z) |
|---|---|---|
| tub/body (uniform box 1.9×1.1×4.4) | 900 kg | (0, 0, 0) |
| engine+gearbox | 220 kg | (0, −0.10, −0.90) |
| driver | 80 kg | (0, 0.00, +0.20) |
| fuel | 40 kg | (0, −0.20, −0.30) |
| 4 × unsprung (as points at hardpoints) | 4×22 | hardpoint positions |
| ballast (remainder to 1350 total) | 22 kg | (0, −0.30, +0.60) |

Compute composite CoM + principal inertia (box formula + parallel axis for
points) at compile time or in vehicle_create (fixed float math). Apply via the
Box3D mass-data API (keep the existing com_drop intent: aim for a CoM height
within ~2 cm of current so grip balance carries over; adjust ballast Y if
needed and note it). Log/report the computed tensor in NOTES.md — expect yaw
inertia well under the old 2600 kg·m² (mass concentrated near center).

## 5. Validation gates

1. Existing suites PASS. Justified threshold updates expected: test_vehicle
   settle (suspension now carries sprung weight only ⇒ ~mg_sprung/4k) and any
   lap-time drift. Determinism 2-run identity mandatory.
2. NEW test_suspension: (a) settle consistency: tire compression ≈
   corner_weight/k_tire and spring ≈ sprung_corner/k_s (±10 %);
   (b) unsprung resonance: after a 3 cm road step (use the analytic kerb or an
   initial travel offset), the wheel oscillates ~12–18 Hz and decays;
   (c) camber becomes more negative with compression (sign + magnitude);
   (d) warm brush sweep with γ = −3° vs 0°: Fy asymmetry/peak shift in the
   grip-adding direction.
3. wasm + smoke PASS; export surface unchanged; autopilot dev-next lap
   COMPLETES at --target-speed 18 (report time; some drift acceptable —
   justify direction: more grip from camber on loaded wheels, livelier yaw).
4. SimStateV1 unchanged (susp_compression now reports suspension-spring travel;
   wheel pos from unsprung state — keep fields meaningful).

## Addendum (2026-07-09, drivetrain & handling wave — anti-roll bars)

`vehicle_suspension_step` gained an `f_arb` input (docs/DRIVETRAIN.md §5):
per 1600 Hz substep the caller computes `F = k_arb · (c_this − c_other)` for
each axle pair from the CURRENT spring compressions and adds it to the strut
force (before the ±max_load clamp). Left/right forces are equal and opposite:
no net vertical force, pure roll-moment redistribution between the axles,
made meaningful by the tire load sensitivity of the same wave. Rates live in
kTuning (`arb_front = 20 kN/m`, `arb_rear = 14 kN/m`, front-stiffer per
DRIVETRAIN.md §5); tests/test_suspension.c passes `f_arb = 0` to keep the §5.2
single-corner gates unchanged.

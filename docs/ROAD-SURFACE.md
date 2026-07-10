# Physics Wave 2 — analytic road surface, thermal depth, tire sub-stepping

Binding spec. Companion to docs/TIRE-MODEL.md. Motivation: the tire model is
only as good as the surface it reads. Today the wheel raycasts hit the same
low-poly triangle mesh the renderer draws — hard edges at every 2.5 m sample
boundary, worst at banking transitions (the classic launched-into-orbit spike
risk). Real sims decouple: a smooth, high-continuity physics surface underneath
a low-poly visual layer.

## 1. Analytic road query (replaces wheel raycasts ON the road)

The TRK1 centerline (pos/up/tangent/width every 2.5 m) already defines the
road; interpolate it instead of meshing it.

`road_query(world_point) -> { height_point, normal, on_road, lateral, seg }`

- **Longitudinal**: Catmull-Rom through centerline `pos[i]` (the 4 samples
  around the segment), parameter found by projecting the query point onto the
  polyline near the last-known index (window search ±6, per wheel — same
  incremental trick as the lap logic; deterministic fixed iteration).
  Refine u by 2 fixed Newton/bisection steps on the CR curve (fixed count).
- **Frames**: `up(u)`, `tangent(u)` interpolated with the same CR weights,
  re-orthonormalized; `side = normalize(cross(up, tangent))`;
  `width(u)` linear.
- **Lateral profile** (l = signed lateral offset from centerline):
  - base: the CR surface point `c(u) + side·l`
  - **crown** (drainage camber): `h_crown = -0.020 · (l / (width/2))² · (width/2) · 0.02`
    — simplified: `h_crown(l) = -crown_m · (2l/width)²` with `crown_m = 0.025`
    (25 mm center-to-edge parabolic cross-fall).
  - **kerb band**: for `width/2 ≤ |l| ≤ width/2 + 1.1` AND smoothed |κ(u)| >
    0.022 (mirror trackgen constants): `h_kerb = 0.02 · (0.5 + 0.5·sin(2π·s/1.25))`
    where `s` = arc length (use `u · 2.5 m` per segment accumulated by index —
    sample index × 2.5 is fine). A smooth sinusoidal rumble (≈13 Hz at 17 m/s)
    replaces the stepped mesh teeth for the TIRES; the visual/chassis mesh
    keeps its geometry.
  - **shoulders**: `width/2 + 1.1 < |l| ≤ width/2 + 8`: linear drop to
    −1.2 m at the outer edge (matches the mesh within tolerance).
- **Normal**: analytic from the lateral profile derivative crossed with
  tangent (continuous camber through banking transitions — this is the point).
- `on_road = |l| ≤ width/2 + 8`. Curvature κ(u): reuse the per-sample smoothed
  κ computed once at track load (precompute an S-length float array in
  sim_load_track; kappa from tangents, window 15 — mirror trackgen).

**Wheel contact** in vehicle_update: replace `b3World_CastRayClosest` with
`road_query` at the hardpoint projected down; compression from the analytic
height along the suspension axis; contact normal from the query.
**Fallback**: if `on_road` is false → keep the existing mesh raycast (car on
shoulders' outer void, rollover recovery, etc.). Chassis body still collides
with the Box3D mesh; only WHEELS go analytic.

Precompute at sim_load_track: kappa array (S floats) via sim arena alloc.
All math fixed-iteration, no transcendentals beyond b3ComputeCosSin/b3Atan2 and
sqrtf (sinf for the kerb: implement via b3ComputeCosSin on wrapped phase).

## 2. Thermal depth (extends TIRE-MODEL §2)

- **Track conduction**: contact adds `h_track·(T_surf − T_track)·dt` cooling
  (T_track = 30 °C, h_track = 0.02/s, only while in contact) — surface sheds
  heat into the asphalt it touches, not just the air.
- **Lockup flash-heat sanity**: with a locked wheel at 30 m/s (σx → −1, full
  slide) friction power all lands on the surface node: verify in the new test
  that 1 s of lockup at Fz0 raises T_surf by > 20 °C and that it decays with a
  time constant of a few seconds after release. Tune C_s/h if needed (justify
  in NOTES.md).

## 3. Tire/wheel sub-stepping (1600 Hz inner loop)

Inside each 400 Hz tick, run the per-wheel force pipeline (slip calc → brush
patch → wheel spin integration → thermal) **4 sub-steps** at `SIM_DT/4`,
accumulating chassis force/torque applications (apply the summed force once
per tick at the patch, or apply per-substep — pick one, justify; forces on the
chassis are integrated by Box3D at the tick level either way). Purpose: wheel
rotational dynamics (lockup, spin-up) are the stiffest ODE in the system;
sub-stepping tames the explicit-Euler oscillation at low speed / high brake
torque. Fixed 4 iterations — determinism unaffected.

## 4. New export (ABI 1.2, additive)

`sim_tire_temp(uint32_t wheel) -> f32` — surface temperature °C (wheel 0-3,
out of range → 0). Add to build_wasm.sh EXPORTS (`_sim_tire_temp`) and a
declaration in sim.h. Everything else unchanged.

## 5. Validation gates

1. test_determinism, test_vehicle, test_tire: PASS (thresholds may shift with
   the smoother surface + substepping — justify any change in NOTES.md).
2. NEW test_road: (a) continuity — walk a straight line crossing sample
   boundaries and a banking transition on the loaded flat-oval test track;
   assert height and normal change smoothly (|Δh| < 5 mm, Δnormal < 1° per
   10 cm step); (b) crown — height at l=width/4 below l=0 by the parabolic
   amount; (c) lockup flash-heat per §2.
3. wasm build + smoke PASS; autopilot lap on assets/tracks/dev-next/track.bin
   COMPLETES (--target-speed 15; report time — expect equal or faster than
   65.4 s thanks to the smoother surface).
4. Export surface = CONTRACTS §1.1 + sim_ffb_torque + sim_tire_temp, nothing else.

## 6. Off-corridor fallback (addendum, 2026-07-09)

On **TRK1 v2 tracks** `road_query` is now TOTAL: outside the corridor
(|lateral| > width/2 + 8) it no longer reports an unusable domain — it returns
a flat **grass** plane: `point = (p.x, ground_y, p.z)`, normal exactly +Y,
zero curvature, and a new `surface_kind` output = 1 (grass; 0 = asphalt /
kerb / shoulder as before). `ground_y = (min over centerline samples of
pos.y) − 1.35`, computed once in `road_load` — the same shared formula
trackgen uses for the ground quad baked into the collision mesh and
`ambience.js` uses for the visual ground disc (CONTRACTS §8).

**Superseded in part (2026-07-09 later wave, docs/TERRAIN.md §2)**: on v2
tracks the flat grass plane is no longer what the wheels ride — it is only
the RAY-MISS fallback. When `road_query` reports kind = grass, the wheel
(vehicle.c) casts one straight-down ray per TICK against the static Box3D
collision world, landscape mesh only (`SIM_CAT_MESH`; prop boxes are excluded
— you cannot drive on a tire stack), and uses the hit point/normal as its
surface plane for all 4 substeps. A miss keeps the ground_y plane above,
bit-identically. The `road_query` contract in this section is UNCHANGED —
the query still returns the plane; the raycast lives in the wheel-contact
layer. Grass classification and forces below are also unchanged.

Wheels on grass (vehicle.c): the brush-model bristle friction is scaled by
**0.55** and a rolling drag of **30 N per (m/s)** of in-plane patch velocity
(~600 N per wheel at 20 m/s, linear in speed) is applied at the patch — grass
is drivable but slow and slippery; going off costs time but is recoverable
instead of a void fall. The transition at the corridor skirt is a physical
drop from the shoulder edge to ground_y (intended). OFF_TRACK / LAP_INVALID
logic is untouched: grass is off the drivable surface, so checkpoints crossed
on grass still invalidate the lap — grass shortcuts cannot produce valid laps.

**Compatibility (binding)**: on **v1 tracks the fallback is disabled** and
off-corridor behavior is bit-identical to the pre-addendum kernel (query
reports `on_road = 0` with the extrapolated shoulder profile; wheels
mesh-raycast). This is not optional caution: the wasm smoke script provably
dangles a wheel ~0.6 m past the corridor edge around tick 3741, so an ungated
grass plane changes v1 hashes (measured). For on-corridor wheels the math is
bit-identical on every version (the grass mu multiplier is exactly 1.0f on
asphalt — an IEEE identity — and the drag branch is untaken); verified by the
byte-identical smoke hash and a bit-exact pre-change query capture in
tests/test_track_v2.c.

# Terrain apron — binding spec (kills the cliff)

Problem: the terrain strip ends at the shoulders; beyond it a vertical skirt
drops to the flat ground plane at ground_y = min(sample y) − 1.35. Any track
elevation reads as a mesa with cliffs. The car physically falls off an
invisible ledge at the shoulder edge.

## Design

**1. Apron geometry (trackgen).** Replace the skirt walls with apron rows:
extend each cross-section beyond the shoulder with 4 extra vertex rows per
side that blend shoulder-edge height → ground_y using a C1 cosine falloff
  h(d) = ground_y + (h_edge − ground_y) · ½(1 + cos(π·d/W))   d ∈ [0, W]
where W is a **per-sample, per-side safe apron width**: W = clamp(
(distance from this sample's shoulder edge to the nearest NON-ADJACENT
segment's shoulder edge) / 2, 10 m, 45 m). This prevents aprons overlapping
in tight loop interiors (hairpin bowl forms a valley instead). The far-field
ground quad stays as the catch-all. The apron triangles are part of the
COLLISION mesh (that is the point).

**2. Physics = the mesh, off-corridor (kernel).** Wheels keep the analytic
road inside the corridor (bit-identical, as today). OFF-corridor on v2
tracks, instead of the flat ground_y plane, each wheel raycasts DOWN against
the static Box3D collision world (the same mesh the player sees — apron,
ground quad, everything). Grass classification (kind=1, μ×0.55, drag) keys
off the corridor test exactly as today. Ray miss (shouldn't happen — ground
quad) falls back to the ground_y plane. Version-gated identically to the
existing grass fallback: v1 behavior stays bit-identical (smoke hash
unchanged). Determinism: Box3D raycasts on static geometry are pure float
math — same mesh + same query = same answer everywhere.

**3. Client (ambience.js).** On v2 tracks skip the skirt walls (the apron
replaces them). Trees/props near the track must sit ON the terrain: sample
height by raycasting the terrain mesh once at placement time (three.js
Raycaster, build-time only) instead of assuming the flat ground plane.

## Consequences

- Regenerating circuit1 with aprons changes its bytes → new hash → board
  rotation. **Bundle with the drivetrain deploy** (one rotation, not two).
- Visual-physics match is by construction (wheels ride the rendered mesh),
  so no shared-formula drift risk beyond the corridor test itself.
- The skirt-color wedge artifact disappears; lighting shades a real slope.

## Gates

1. v1 smoke hash byte-identical (version gate holds).
2. circuit1 regen: lint gate passes; apron self-intersection check (no apron
   vertex above any other segment's road surface); autopilot completes.
3. Grass excursion harness: drive off at speed on a high section — car
   descends the apron slope (no ledge drop > 0.4 m), returns to track.
4. step==replay equality on the new track with an excursion-heavy log.
5. Visual: no skirts on v2, trees grounded on slopes, no z-fighting at the
   apron/ground-quad seam (share the exact ground_y).

## Status (2026-07-09)

- **§2 physics = the mesh: DONE** (kernel wave, bundled with the drivetrain
  deploy — one board rotation). Off-corridor wheels on v2 tracks cast one
  straight-down ray per wheel per TICK against the static collision world,
  filtered to the landscape mesh only (`SIM_CAT_MESH` category bit on the
  mesh shape; prop boxes deliberately excluded — wheels cannot ride a tire
  stack, the chassis still crashes into props as before). The hit's tangent
  plane is the wheel surface for all 4 substeps (the hardpoint is frozen
  within a tick, so a per-substep ray would return the identical hit — the
  tick-level ray is exact, not an approximation). Grass classification
  (kind = 1, μ×0.55 + rolling drag) keys off the corridor test exactly as
  before; ray miss falls back to the flat ground_y plane bit-identically.
  Version gate proven: wasm smoke hash `7acf8c978fae724b` unchanged (the
  smoke oval is v1 WITH an off-corridor wheel excursion). Sloped-mesh
  follow + on-corridor capture-compare + v2 excursion step==replay live in
  physics/tests/test_track_v2.c §5. Measured cost: ≈0.5 µs/tick native with
  all four wheels raycasting (~0.02 % of the 2.5 ms tick budget); ray API
  caveats in physics/NOTES.md.
- **§1 apron geometry (trackgen): DONE** (trackgen wave, bundled with the
  same drivetrain deploy). `assemble_circuit` (v2/circuit path ONLY — the v1
  `generate()` ribbon is frozen; dev5 regen byte-identity `162adead4bdf1931`
  re-proven post-change) appends 4 apron rows per side at geometric spacing
  (gaps W·(1,2,4,8)/15), heights on the C1 cosine falloff, W = clamp(gap/2,
  10, 45) from the nearest non-adjacent (index distance > 40) shoulder edge,
  circularly smoothed over 15 samples. **Fold cap (spec refinement)**: on the
  inside of a corner the offset rays converge at the local curvature center;
  W is additionally capped at 0.8·(1/|κ_raw,max-in-window| − lat_edge) on the
  converging side (re-applied after smoothing), and this cap may undercut the
  10 m floor — without it the ruled surface folds over itself (284 flipped,
  back-face-culled triangles the wheel ray would miss; measured before the
  fix). Hairpin inside bottoms out at W = 1.61 m (a steep funnel — still
  strictly better than the vertical skirt it replaces). Vertex layout: apron
  block at [4S, 12S), v(i,side,row) = 4S + i·8 + side·4 + row; kerbs then
  ground quad follow. Z-fight choice: far apron row at ground_y − 0.001, quad
  exactly at ground_y (1 mm resolves in-depth out to ~40 m at near=0.1;
  beyond that the seam sliver is dark-green-on-dark-green). circuit1
  regenerated in place: 555 276 bytes, track_hash `79a40d1c2b08fbe0` (was
  `99f5ea59bd905065`) — centerline/props/ground_y bit-identical, mesh
  +8 verts/+16 tris per sample (V 14 856, T 26 952). Gates live in
  trackgen/check_apron.py (mesh integrity/normals, corridor intersection,
  prop clearance, apron stats); W min/max/mean = 1.61/45.00/38.91 m, outer
  edge step 0.001 m. Autopilot on the staged wasm completes (142.977 s,
  target-speed 22) with a final state hash bit-identical to the pre-apron
  mesh — on-corridor physics untouched. track.js mottling extended to the
  apron rows (stepped slightly darker toward the plain).
- **§3 client (ambience.js): DONE** (same bundle) — skirt walls skipped on
  v2 tracks (apron replaces them; v1 path keeps them), trees and stands
  grounded by a single build-time raycast against the real terrain mesh
  (aprons included automatically), trees on slopes > 20° skipped.

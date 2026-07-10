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

# Soft-body (BeamNG-style) crash physics — engineering assessment

BeamNG simulates the vehicle as a node/beam lattice (mass points + spring
beams) at ~2000 Hz; deformation IS the physics. Verdict for us: **a full
soft-body chassis is the wrong trade for this game, but a bounded node/beam
DAMAGE LAYER on top of the rigid core is a great fit.** Reasons and plan:

## Why not full soft-body

1. **The validated stack is the product.** Tires, suspension, FFB and the
   replay referee all hang off one rigid chassis + quarter-car corners. A
   node/beam chassis replaces all of it — months of re-validation for crash
   fidelity in a *time-trial* game where crashing means your lap is dead
   anyway.
2. **Determinism cost is fine, CPU is not.** ~60 nodes × ~200 beams at the
   1600 Hz substep is feasible in wasm (BeamNG uses ~4k beams/car), but the
   edge replay budget (~100 ms/lap today) would grow ~10×.
3. Time-trial competition needs identical cars; persistent deformation
   changing handling mid-lap makes leaderboard laps incomparable unless
   damage hard-invalidates the lap — which checkpoint logic already does
   for the crashes that matter.

## What we take from BeamNG: a damage lattice (phased)

**Phase 1 — visual deformation (web client, no kernel change). — DONE.**
Shipped as `worker/public/js/crumple.js` (45-node 3×3×5 lattice, 160 beams,
plastic-yield rest-length drag); player car only, ghost stays pristine, reset on
new lap/respawn. Render-only — no determinism/replay impact.
~40-node lattice mapped to the car-mesh vertices. On chassis contact events
(detectable today: chassis mesh collision ⇒ large Δvelocity spikes at the
body), apply impulse to the nearest lattice nodes; beams (neighbor springs
with plastic yield) relax each frame; skin the render mesh to the lattice.
Crumpled nose after a wall hit, permanent, cheap ((~200 spring evals/frame,
JS). Cosmetic ⇒ no determinism/replay impact.

**Phase 2 — deterministic damage state (kernel, ABI-additive). — DONE.**
Shipped as the `sim_damage(component)` export (ABI 1.3, CONTRACTS §1.1): overall +
steer + front/rear toe scalars, zero on a clean lap, driving both HUD and the
crumple severity so the visible dent matches the referee-scored damage.
Kernel tracks per-corner damage scalars from contact impulses (deterministic:
same log ⇒ same damage): aero_cl loss, toe offset, max_steer reduction.
Exported for HUD; affects physics ⇒ replays stay honest because it derives
purely from the input log. Lap validity rules unchanged.

**Phase 3 (only if a sandbox mode ever exists): true node/beam chassis** as a
separate non-competitive mode with its own binary. Not planned.

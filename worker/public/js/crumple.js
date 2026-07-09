// Cosmetic crash deformation for the player car (SOFTBODY.md Phase 1).
//
// A coarse control lattice (node/beam grid) encloses the car body. Each body
// mesh vertex is bound to its nearest lattice nodes by inverse-distance weights;
// moving nodes re-skins the mesh via displacement interpolation. An impact
// shoves the nodes on the struck face inward; each frame the beams relax toward
// their rest length, but a beam strained past a yield fraction PERMANENTLY
// updates its rest length — so the plastic core of the dent stays while the
// elastic ripple springs back.
//
// This is render-only. It never touches the sim, the input quantiser, the lap
// log, or determinism/replay. Only the player car owns a Crumple; the ghost is
// left pristine (separate geometries, no instance).
import * as THREE from "../vendor/three.module.js";

// ---- lattice resolution: 3 × 3 × 5 = 45 nodes -----------------------------
const NX = 3, NY = 3, NZ = 5; // z is the long car axis (+Z forward)
const PAD = 0.04;             // grow bbox a touch so the shell sits inside

// ---- plastic-beam tuning --------------------------------------------------
const STIFFNESS = 0.55;   // elastic pull toward rest, per relaxation pass
const ITERS = 3;          // relaxation passes per frame while settling
const YIELD = 0.09;       // |strain| beyond this yields (permanent set)
const PLASTIC = 0.55;     // fraction of the over-strain baked into rest length
const SETTLE_EPS = 1.5e-3; // per-frame max node move below which we call it done
const STABLE_FRAMES = 4;   // consecutive quiet frames before going idle
const MAX_SETTLE = 90;     // hard cap on settling frames (safety)

// ---- impact shaping -------------------------------------------------------
const DENT_SCALE = 0.42;  // metres of inward crush at the face for severity 1
const BIND_K = 3;         // lattice nodes each vertex blends between

const _v = new THREE.Vector3();
const _d = new THREE.Vector3();

export class Crumple {
  /** carGroup must have been built with buildCar(..., deformable=true) so its
   *  body meshes are baked into group space and listed in userData.bodyMeshes. */
  constructor(carGroup) {
    this.meshes = carGroup.userData.bodyMeshes ?? carGroup.children.filter((c) => c.isMesh);

    // Per-mesh original (pristine) vertex positions in group space.
    this.orig = this.meshes.map((m) => Float32Array.from(m.geometry.attributes.position.array));

    // Bounding box over every body vertex → lattice extent.
    const min = new THREE.Vector3(Infinity, Infinity, Infinity);
    const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
    for (const o of this.orig)
      for (let i = 0; i < o.length; i += 3) {
        _v.set(o[i], o[i + 1], o[i + 2]);
        min.min(_v);
        max.max(_v);
      }
    min.subScalar(PAD);
    max.addScalar(PAD);
    this.center = min.clone().add(max).multiplyScalar(0.5);

    // ---- build nodes (rest = pristine position, pos = live position) ----
    this.nodes = [];
    const at = (t, a, b) => a + (b - a) * t;
    const idx = (ix, iy, iz) => (ix * NY + iy) * NZ + iz;
    for (let ix = 0; ix < NX; ix++)
      for (let iy = 0; iy < NY; iy++)
        for (let iz = 0; iz < NZ; iz++) {
          const rest = new THREE.Vector3(
            at(NX > 1 ? ix / (NX - 1) : 0.5, min.x, max.x),
            at(NY > 1 ? iy / (NY - 1) : 0.5, min.y, max.y),
            at(NZ > 1 ? iz / (NZ - 1) : 0.5, min.z, max.z),
          );
          this.nodes.push({ rest, pos: rest.clone() });
        }

    // ---- beams: axis neighbours + the 4 space diagonals of every cell ----
    this.beams = [];
    const addBeam = (a, b) => {
      const rest = this.nodes[a].rest.distanceTo(this.nodes[b].rest);
      this.beams.push({ a, b, rest, rest0: rest });
    };
    for (let ix = 0; ix < NX; ix++)
      for (let iy = 0; iy < NY; iy++)
        for (let iz = 0; iz < NZ; iz++) {
          if (ix + 1 < NX) addBeam(idx(ix, iy, iz), idx(ix + 1, iy, iz));
          if (iy + 1 < NY) addBeam(idx(ix, iy, iz), idx(ix, iy + 1, iz));
          if (iz + 1 < NZ) addBeam(idx(ix, iy, iz), idx(ix, iy, iz + 1));
        }
    for (let ix = 0; ix + 1 < NX; ix++)
      for (let iy = 0; iy + 1 < NY; iy++)
        for (let iz = 0; iz + 1 < NZ; iz++) {
          addBeam(idx(ix, iy, iz), idx(ix + 1, iy + 1, iz + 1));
          addBeam(idx(ix + 1, iy, iz), idx(ix, iy + 1, iz + 1));
          addBeam(idx(ix, iy + 1, iz), idx(ix + 1, iy, iz + 1));
          addBeam(idx(ix, iy, iz + 1), idx(ix + 1, iy + 1, iz));
        }

    // ---- bind each vertex to its BIND_K nearest nodes (inverse distance) ----
    // Skin rule: vertexNew = vertexOrig + Σ w·(node.pos − node.rest). With the
    // weights summing to 1 this is exactly pristine when no node has moved.
    this.bind = this.orig.map((o) => {
      const n = o.length / 3;
      const nodeIdx = new Int32Array(n * BIND_K);
      const weight = new Float32Array(n * BIND_K);
      const near = new Array(BIND_K);
      for (let vi = 0; vi < n; vi++) {
        _v.set(o[vi * 3], o[vi * 3 + 1], o[vi * 3 + 2]);
        for (let k = 0; k < BIND_K; k++) near[k] = { i: -1, d: Infinity };
        for (let ni = 0; ni < this.nodes.length; ni++) {
          const dist = _v.distanceTo(this.nodes[ni].rest);
          // insertion into the small nearest-K list
          if (dist < near[BIND_K - 1].d) {
            let k = BIND_K - 1;
            while (k > 0 && dist < near[k - 1].d) { near[k] = near[k - 1]; k--; }
            near[k] = { i: ni, d: dist };
          }
        }
        let sum = 0;
        for (let k = 0; k < BIND_K; k++) {
          const w = 1 / (near[k].d + 1e-4);
          weight[vi * BIND_K + k] = w;
          nodeIdx[vi * BIND_K + k] = near[k].i;
          sum += w;
        }
        for (let k = 0; k < BIND_K; k++) weight[vi * BIND_K + k] /= sum;
      }
      return { nodeIdx, weight };
    });

    this.active = false;   // true while settling (relaxation runs)
    this._quiet = 0;
    this._frames = 0;
    this.skin(); // establish pristine positions (no-op displacement)
  }

  /** Displace the lattice nodes on the struck face inward. localDir is the
   *  inward crush direction in car-local space (e.g. a nose hit → (0,0,-1));
   *  severity is a rough 0..~1.5 magnitude. Wakes the settling loop. */
  applyImpact(localDir, severity) {
    _d.set(
      localDir.x ?? localDir[0] ?? 0,
      localDir.y ?? localDir[1] ?? 0,
      localDir.z ?? localDir[2] ?? 0,
    );
    if (_d.lengthSq() < 1e-9) return;
    _d.normalize();
    severity = Math.max(0, Math.min(1.5, severity));
    if (severity <= 0) return;
    const reach = severity * DENT_SCALE;

    // Struck face = the extreme in the −localDir (outward-normal) direction.
    // Score nodes by projection onto the outward normal, normalise to [0,1].
    const nrm = _d.clone().negate();
    let lo = Infinity, hi = -Infinity;
    const proj = this.nodes.map((nd) => {
      const p = _v.copy(nd.rest).sub(this.center).dot(nrm);
      if (p < lo) lo = p;
      if (p > hi) hi = p;
      return p;
    });
    const span = hi - lo || 1;
    for (let i = 0; i < this.nodes.length; i++) {
      const t = (proj[i] - lo) / span; // 0 far side … 1 struck face
      const w = t * t;                 // crush concentrated at the face
      this.nodes[i].pos.addScaledVector(_d, reach * w);
    }
    this.active = true;
    this._quiet = 0;
    this._frames = 0;
  }

  /** Advance the plastic relaxation one frame and re-skin. No-op when idle.
   *  Quiet-detection tracks the NET node movement this frame (start→end), not
   *  raw beam corrections: an over-constrained lattice settles into a tiny
   *  Gauss-Seidel limit cycle whose per-beam corrections never vanish, but
   *  whose net node motion does once the dent has set. */
  update() {
    if (!this.active) return;
    for (const nd of this.nodes) nd.prev = nd.prev ? nd.prev.copy(nd.pos) : nd.pos.clone();
    for (let it = 0; it < ITERS; it++) {
      for (const beam of this.beams) {
        const a = this.nodes[beam.a].pos, b = this.nodes[beam.b].pos;
        _d.copy(b).sub(a);
        const len = _d.length();
        if (len < 1e-6) continue;
        // Plastic yield: strain past YIELD permanently drags rest → current.
        const strain = Math.abs(len - beam.rest) / beam.rest;
        if (strain > YIELD) beam.rest += (len - beam.rest) * PLASTIC;
        // Elastic correction toward the (possibly updated) rest length.
        const corr = ((len - beam.rest) * STIFFNESS * 0.5) / len;
        a.addScaledVector(_d, corr);
        b.addScaledVector(_d, -corr);
      }
    }
    let maxMove = 0;
    for (const nd of this.nodes) maxMove = Math.max(maxMove, nd.pos.distanceTo(nd.prev));
    this.skin();
    this._frames++;
    this._quiet = maxMove < SETTLE_EPS ? this._quiet + 1 : 0;
    if (this._quiet >= STABLE_FRAMES || this._frames >= MAX_SETTLE) this.active = false;
  }

  /** Re-skin every bound vertex from current node displacements. */
  skin() {
    for (let mi = 0; mi < this.meshes.length; mi++) {
      const o = this.orig[mi];
      const { nodeIdx, weight } = this.bind[mi];
      const attr = this.meshes[mi].geometry.attributes.position;
      const arr = attr.array;
      const n = o.length / 3;
      for (let vi = 0; vi < n; vi++) {
        let dx = 0, dy = 0, dz = 0;
        for (let k = 0; k < BIND_K; k++) {
          const ni = nodeIdx[vi * BIND_K + k];
          const w = weight[vi * BIND_K + k];
          const nd = this.nodes[ni];
          dx += w * (nd.pos.x - nd.rest.x);
          dy += w * (nd.pos.y - nd.rest.y);
          dz += w * (nd.pos.z - nd.rest.z);
        }
        arr[vi * 3] = o[vi * 3] + dx;
        arr[vi * 3 + 1] = o[vi * 3 + 1] + dy;
        arr[vi * 3 + 2] = o[vi * 3 + 2] + dz;
      }
      attr.needsUpdate = true;
      this.meshes[mi].geometry.computeVertexNormals();
    }
  }

  /** Restore the car to pristine (undented) — called by resetRun / new lap. */
  reset() {
    for (const nd of this.nodes) nd.pos.copy(nd.rest);
    for (const beam of this.beams) beam.rest = beam.rest0;
    this.active = false;
    this._quiet = 0;
    this._frames = 0;
    this.skin();
  }
}

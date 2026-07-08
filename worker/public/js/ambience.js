// Trackside ambience — procedural, seeded, cheap. Everything is derived
// deterministically from the track bytes (fnv1a64 hash → LCG), so a given
// track always dresses itself the same way and no assets are shipped:
//   • low-poly trees (InstancedMesh trunk + canopy) on both sides, outside
//     the shoulder zone (lateral offset 12–25 m past width/2), every 10–20 m,
//     skipped where they'd land on another part of the loop;
//   • red/white tire-stack barriers lining the OUTSIDE of the tightest
//     corners (same smoothed-curvature rule the kerb paint uses);
//   • a start/finish gantry at sample 0 with an emissive strip + banner
//     (materials exposed via group.userData.glowMaterials so the time-of-day
//     lighting can boost them at night);
//   • 3 braking marker boards at 150/100/50 m (60/40/20 samples) before the
//     tightest corner, striped 3/2/1.
import * as THREE from "../vendor/three.module.js";
import { fnv1a64, smoothedCurvature, KERB_KAPPA } from "./track.js";

const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

/** @returns THREE.Group (userData.glowMaterials = materials the night
 *  lighting may brighten). hash: precomputed fnv1a64 of track.bytes. */
export function buildAmbience(track, hash = null) {
  const group = new THREE.Group();
  const glowMaterials = [];
  group.userData.glowMaterials = glowMaterials;
  const n = track.S;

  // Deterministic PRNG seeded from the track identity.
  let seed = (Number((hash ?? fnv1a64(track.bytes)) & 0xffffffffn) >>> 0) || 1;
  const rand = () => (seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 2 ** 32;

  // Local frame at sample i: side = up × tangent (lateral), all normalized.
  const frame = (i) => {
    const c = v3(track.center[i]);
    const u = v3(track.up[i]).normalize();
    const t = v3(track.tangent[i]).normalize();
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    return { c, u, t, side };
  };
  // The curvature vector (tangent[i+1] − tangent[i]) points INTO the turn;
  // its sign along `side` tells us which lateral direction is inside.
  const insideSign = (i) => {
    const f = frame(i);
    const t2 = v3(track.tangent[(i + 1) % n]);
    return Math.sign(t2.sub(f.t).dot(f.side)) || 1;
  };

  // ------------------------------------------------------------- ground
  // The world beyond the 8 m shoulders is void — give it a ground plane at
  // shoulder-base level so trees stand on something and the horizon reads as
  // land, not outer space. Trees sit ON this plane.
  let minY = Infinity, cx = 0, cz = 0, maxR = 0;
  for (const c of track.center) {
    minY = Math.min(minY, c[1]);
    cx += c[0] / n; cz += c[2] / n;
  }
  for (const c of track.center) maxR = Math.max(maxR, Math.hypot(c[0] - cx, c[2] - cz));
  const groundY = minY - 1.35; // just under the shoulder drop (-1.2)
  const groundGeo = new THREE.CircleGeometry(maxR + 120, 40);
  groundGeo.rotateX(-Math.PI / 2);
  const ground = new THREE.Mesh(groundGeo, new THREE.MeshLambertMaterial({ color: 0x2d3b31 }));
  ground.position.set(cx, groundY, cz);
  group.add(ground);

  // Skirts: the terrain strip ends at the shoulders' outer edges, which float
  // above the ground plane wherever the track climbs — connect each outer
  // edge down to the plane with a wall strip so road and land read as one
  // world. Terrain vertex layout (trackgen): v(i,row) = i*4+row, rows 0/3 are
  // the outer shoulder edges.
  {
    const skirtMat = new THREE.MeshLambertMaterial({ color: 0x2d3b31, side: THREE.DoubleSide });
    for (const row of [0, 3]) {
      const pos = new Float32Array(n * 2 * 3);
      for (let i = 0; i < n; i++) {
        const base = (i * 4 + row) * 3;
        const x = track.verts[base], y = track.verts[base + 1], z = track.verts[base + 2];
        pos.set([x, y + 0.02, z], i * 6);
        pos.set([x, groundY, z], i * 6 + 3);
      }
      const idx = [];
      for (let i = 0; i < n; i++) {
        const j = (i + 1) % n;
        idx.push(2 * i, 2 * i + 1, 2 * j, 2 * j, 2 * i + 1, 2 * j + 1);
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      g.setIndex(idx);
      g.computeVertexNormals();
      group.add(new THREE.Mesh(g, skirtMat));
    }
  }

  // ------------------------------------------------------------- trees
  // A candidate 12–25 m off one segment can still sit ON another segment of
  // the loop (hairpins, straights folding back) — reject anything within
  // width/2 + 7 m of any centerline sample.
  const clearOfTrack = (p) => {
    for (let i = 0; i < n; i += 2) {
      const c = track.center[i];
      const dx = p.x - c[0], dz = p.z - c[2];
      const r = track.width[i] / 2 + 7;
      if (dx * dx + dz * dz < r * r) return false;
    }
    return true;
  };
  const trees = [];
  for (let i = 0; i < n; i += 4 + ((rand() * 4) | 0)) { // every 10–20 m
    for (const s of [-1, 1]) {
      if (rand() < 0.25) continue; // density jitter
      const f = frame(i);
      const lat = track.width[i] / 2 + 12 + rand() * 13; // 12–25 m past edge
      const p = f.c.clone()
        .addScaledVector(f.side, s * lat)
        .addScaledVector(f.t, (rand() - 0.5) * 4);
      p.y = groundY; // stand on the ground plane (world beyond shoulders is flat)
      if (!clearOfTrack(p)) continue;
      trees.push({ p, scale: [0.8, 1.0, 1.35][(rand() * 3) | 0], rot: rand() * Math.PI * 2 });
    }
  }
  if (trees.length) {
    const trunkGeo = new THREE.CylinderGeometry(0.15, 0.22, 1.2, 6);
    trunkGeo.translate(0, 0.6, 0); // base at y=0
    const canopyGeo = new THREE.ConeGeometry(1.2, 3.2, 7);
    canopyGeo.translate(0, 2.6, 0); // sits on the trunk
    const trunks = new THREE.InstancedMesh(
      trunkGeo, new THREE.MeshLambertMaterial({ color: 0x3a2f26 }), trees.length);
    const canopies = new THREE.InstancedMesh(
      canopyGeo, new THREE.MeshLambertMaterial({ color: 0xffffff }), trees.length);
    const greens = [0x1e3227, 0x24402c, 0x1a2b22].map((c) => new THREE.Color(c));
    const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), up = new THREE.Vector3(0, 1, 0);
    trees.forEach((tr, k) => {
      q.setFromAxisAngle(up, tr.rot);
      m4.compose(tr.p, q, new THREE.Vector3(tr.scale, tr.scale, tr.scale));
      trunks.setMatrixAt(k, m4);
      canopies.setMatrixAt(k, m4);
      canopies.setColorAt(k, greens[(rand() * greens.length) | 0]);
    });
    group.add(trunks, canopies);
  }

  // ------------------------------------------- tire-stack barriers
  // Outside of every corner the kerb painter flags (|smoothed κ| > 0.022),
  // one stack (2 cylinders high, alternating red/white) every ~5 m.
  const kappa = smoothedCurvature(track);
  const stacks = [];
  for (let i = 0; i < n; i += 2) {
    if (Math.abs(kappa[i]) <= KERB_KAPPA) continue;
    const f = frame(i);
    const lat = track.width[i] / 2 + 2.4 + rand() * 0.8;
    const p = f.c.clone().addScaledVector(f.side, -insideSign(i) * lat)
      .addScaledVector(f.t, (rand() - 0.5) * 1.2);
    stacks.push({ p, u: f.u });
  }
  if (stacks.length) {
    const tireGeo = new THREE.CylinderGeometry(0.55, 0.55, 0.55, 10);
    const barrier = new THREE.InstancedMesh(
      tireGeo, new THREE.MeshLambertMaterial({ color: 0xffffff }), stacks.length * 2);
    const red = new THREE.Color(0xb8352b), white = new THREE.Color(0xd9dde2);
    const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
    const pos = new THREE.Vector3();
    stacks.forEach((st, k) => {
      for (const level of [0, 1]) {
        pos.copy(st.p).addScaledVector(st.u, 0.28 + level * 0.56);
        m4.compose(pos, q, one);
        barrier.setMatrixAt(2 * k + level, m4);
        barrier.setColorAt(2 * k + level, (k + level) % 2 ? red : white);
      }
    });
    group.add(barrier);
  }

  // ------------------------------------------- start/finish gantry (sample 0)
  {
    const f = frame(0);
    const gantry = new THREE.Group();
    gantry.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
    gantry.position.copy(f.c);
    const dark = new THREE.MeshLambertMaterial({ color: 0x1b2129 });
    const postX = track.width[0] / 2 + 1.5;
    for (const s of [-1, 1]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.35, 5.4, 0.35), dark);
      post.position.set(s * postX, 2.7, 0);
      gantry.add(post);
    }
    const beam = new THREE.Mesh(new THREE.BoxGeometry(2 * postX + 0.35, 0.5, 0.5), dark);
    beam.position.set(0, 5.55, 0);
    gantry.add(beam);
    // Emissive strip + banner — glowMaterials so night mode can crank them.
    const stripMat = new THREE.MeshLambertMaterial({
      color: 0x0c0f14, emissive: 0xfff0c8, emissiveIntensity: 0.6 });
    const strip = new THREE.Mesh(new THREE.BoxGeometry(2 * postX, 0.09, 0.09), stripMat);
    strip.position.set(0, 5.26, 0);
    gantry.add(strip);
    const bannerMat = new THREE.MeshLambertMaterial({
      color: 0x0e1522, emissive: 0x8fd3ff, emissiveIntensity: 0.5 });
    const banner = new THREE.Mesh(new THREE.BoxGeometry(3.2, 1.1, 0.12), bannerMat);
    banner.position.set(0, 4.7, 0);
    gantry.add(banner);
    glowMaterials.push(stripMat, bannerMat);
    group.add(gantry);
  }

  // ------------------------------------------- braking marker boards
  // 150/100/50 m (60/40/20 samples @ 2.5 m) before the tightest corner, on
  // the corner's outside — that's the braking-zone side. MeshBasicMaterial so
  // they stay readable at night. Stripes count down 3 → 2 → 1.
  {
    let maxIdx = 0;
    for (let i = 1; i < n; i++) if (Math.abs(kappa[i]) > Math.abs(kappa[maxIdx])) maxIdx = i;
    const out = -insideSign(maxIdx);
    const boardMat = new THREE.MeshBasicMaterial({ color: 0xf2f4f7 });
    const barMat = new THREE.MeshBasicMaterial({ color: 0xd23b2f });
    const postMat = new THREE.MeshLambertMaterial({ color: 0x1b2129 });
    [[60, 3], [40, 2], [20, 1]].forEach(([back, bars]) => {
      const bi = (maxIdx - back + n) % n;
      const f = frame(bi);
      const board = new THREE.Group();
      board.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
      board.position.copy(f.c).addScaledVector(f.side, out * (track.width[bi] / 2 + 3));
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.0, 0.08), postMat);
      post.position.y = 0.5;
      board.add(post);
      const face = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.7, 0.06), boardMat);
      face.position.y = 1.35;
      board.add(face);
      for (let k = 0; k < bars; k++) {
        const bar = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.56, 0.02), barMat);
        // Cars travel toward +tangent (increasing sample index), so the
        // legible face points back down the track (local −z).
        bar.position.set((k - (bars - 1) / 2) * 0.28, 1.35, -0.045);
        board.add(bar);
      }
      group.add(board);
    });
  }

  return group;
}

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
// APEX "Build the world" layer order, read from the white line outward:
// white line → kerb → run-off (gravel beds on the two tightest corners,
// mottled grass elsewhere — see track.js) → barrier (tire stacks on the
// tightest corners, instanced Armco on the medium ones) → grandstand + crowd
// (one main stand on the start straight, smaller stands at signature corners).
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

  // ------------------------------------------- curvature + corner zones
  // Shared by gravel traps, Armco, tire stacks, brake boards and stands.
  const kappa = smoothedCurvature(track);
  // Contiguous |κ| > KERB_KAPPA runs (wrap-aware), ranked tightest-first.
  const zones = (() => {
    let a = 0;
    while (a < n && Math.abs(kappa[a]) > KERB_KAPPA) a++;
    if (a >= n) return [];
    const out = [];
    let runStart = -1;
    for (let s = 1; s <= n; s++) {
      const inz = s < n && Math.abs(kappa[(a + s) % n]) > KERB_KAPPA;
      if (inz && runStart < 0) runStart = s;
      else if (!inz && runStart >= 0) {
        let peak = 0, peakIdx = (a + runStart) % n;
        for (let k = runStart; k < s; k++) {
          const ii = (a + k) % n;
          if (Math.abs(kappa[ii]) > peak) { peak = Math.abs(kappa[ii]); peakIdx = ii; }
        }
        out.push({ start: (a + runStart) % n, len: s - runStart, peak, peakIdx });
        runStart = -1;
      }
    }
    out.sort((p, q) => q.peak - p.peak);
    return out;
  })();

  // A point 10–25 m off one segment can still sit ON another segment of the
  // loop (hairpins, straights folding back) — reject anything within
  // width/2 + 7 m of any centerline sample. Used by trees and stands.
  const clearOfTrack = (p) => {
    for (let i = 0; i < n; i += 2) {
      const c = track.center[i];
      const dx = p.x - c[0], dz = p.z - c[2];
      const r = track.width[i] / 2 + 7;
      if (dx * dx + dz * dz < r * r) return false;
    }
    return true;
  };

  // ------------------------------------------- grandstands + crowd
  // Placed BEFORE the trees so tree candidates inside a stand footprint can
  // be culled. One main tiered stand along the start straight (sample 0) +
  // up to 3 smaller stands at the signature (tightest) corners. All boxes go
  // through two InstancedMeshes (grey structure, carbon-dark roofs); the
  // crowd is one merged quad mesh with a shared random-pixel canvas texture.
  const standFootprints = []; // {x, z, r} — tree exclusion circles
  {
    const upY = new THREE.Vector3(0, 1, 0);
    const STEP_H = 0.55, STEP_D = 1.3;
    const standDefs = [];
    const tryStand = (i0, out, L, tiers, dist) => {
      const f = frame(i0);
      const th = f.t.clone().setY(0).normalize();
      const sh = new THREE.Vector3().crossVectors(upY, th).normalize();
      const base = f.c.clone().addScaledVector(sh, out * dist);
      base.y = groundY;
      const depth = tiers * STEP_D + 2.5;
      for (const [dl, dd] of [[0, 0], [-L / 2, 0], [L / 2, 0], [0, depth]]) {
        const p = base.clone().addScaledVector(th, dl).addScaledVector(sh, out * dd);
        if (!clearOfTrack(p)) return false;
      }
      standDefs.push({ base, th, sh, out, L, tiers });
      standFootprints.push({
        x: base.x + sh.x * out * depth / 2, z: base.z + sh.z * out * depth / 2,
        r: Math.hypot(L / 2, depth / 2) + 2,
      });
      return true;
    };
    // Main grandstand: sized to the flat run around sample 0 (start straight).
    let run = 1;
    while (run < 15 && Math.abs(kappa[run % n]) < 0.008 && Math.abs(kappa[(n - run) % n]) < 0.008) run++;
    const mainL = Math.max(30, Math.min(72, run * 2 * 2.5 * 0.8));
    const d0 = track.width[0] / 2 + 11;
    tryStand(0, 1, mainL, 5, d0) || tryStand(0, -1, mainL, 5, d0);
    // Smaller stands at the tightest corners, on the outside past the run-off.
    let small = 0;
    for (const z of zones) {
      if (small >= 3) break;
      if (z.len < 4) continue;
      if (tryStand(z.peakIdx, -insideSign(z.peakIdx), 16, 3, track.width[z.peakIdx] / 2 + 15)) small++;
    }
    if (standDefs.length) {
      const structXf = [], roofXf = [];
      const crowdPos = [], crowdUv = [], crowdIdx = [];
      for (const sd of standDefs) {
        const { base, th, sh, out, L, tiers } = sd;
        const q = new THREE.Quaternion().setFromRotationMatrix(
          new THREE.Matrix4().makeBasis(th, upY, new THREE.Vector3().crossVectors(th, upY)));
        const at = (dAlong, dOut, y) =>
          base.clone().addScaledVector(th, dAlong).addScaledVector(sh, out * dOut).setY(base.y + y);
        for (let k = 0; k < tiers; k++) { // solid steps rising away from track
          const h = (k + 1) * STEP_H;
          structXf.push({ p: at(0, k * STEP_D + STEP_D / 2, h / 2), q, s: new THREE.Vector3(L, h, STEP_D) });
        }
        const wallH = tiers * STEP_H + 1.4; // back wall
        structXf.push({ p: at(0, tiers * STEP_D + 0.25, wallH / 2), q, s: new THREE.Vector3(L, wallH, 0.5) });
        const roofY = tiers * STEP_H + 2.6; // front roof posts
        for (const dl of [-L / 2 + 0.4, L / 2 - 0.4]) {
          structXf.push({ p: at(dl, 0.3, roofY / 2), q, s: new THREE.Vector3(0.22, roofY, 0.22) });
        }
        roofXf.push({ // carbon-dark slab with a slight front overhang
          p: at(0, tiers * STEP_D / 2 - 0.4, roofY + 0.09), q,
          s: new THREE.Vector3(L + 1.2, 0.18, tiers * STEP_D + 2.0),
        });
        // crowd band: one inclined quad laid over the steps
        const vbase = crowdPos.length / 3;
        for (const [dOut, y] of [[0.25, STEP_H + 0.35], [(tiers - 1) * STEP_D + 0.9, tiers * STEP_H + 0.4]]) {
          for (const dl of [-L / 2, L / 2]) {
            const p = at(dl, dOut, y);
            crowdPos.push(p.x, p.y, p.z);
          }
        }
        const rep = L / 24; // one texture repeat / 24 m of seating
        crowdUv.push(0, 0, rep, 0, 0, 1, rep, 1);
        crowdIdx.push(vbase, vbase + 1, vbase + 2, vbase + 1, vbase + 3, vbase + 2);
      }
      const boxGeo = new THREE.BoxGeometry(1, 1, 1);
      const m4 = new THREE.Matrix4();
      const structMesh = new THREE.InstancedMesh(boxGeo,
        new THREE.MeshStandardMaterial({ color: 0x767e88, roughness: 0.85, metalness: 0.1 }),
        structXf.length);
      structXf.forEach((xf, k) => { m4.compose(xf.p, xf.q, xf.s); structMesh.setMatrixAt(k, m4); });
      const roofMesh = new THREE.InstancedMesh(boxGeo,
        new THREE.MeshStandardMaterial({ color: 0x14171c, roughness: 0.55, metalness: 0.35 }),
        roofXf.length);
      roofXf.forEach((xf, k) => { m4.compose(xf.p, xf.q, xf.s); roofMesh.setMatrixAt(k, m4); });
      // Crowd texture: random colored pixel blobs in seat rows — reads as
      // people from track distance. Shared by every stand. Emissive: none.
      const cvs = document.createElement("canvas");
      cvs.width = 192; cvs.height = 64;
      const g2d = cvs.getContext("2d");
      g2d.fillStyle = "#171b21";
      g2d.fillRect(0, 0, 192, 64);
      const cols = ["#c8ccd4", "#8f3d3d", "#3d5c8f", "#c2a23a", "#3f7a55",
        "#7a4f8f", "#22e0d0", "#b8352b", "#d9dde2", "#37516e"];
      for (let y = 2; y < 62; y += 4) {
        for (let x = 1; x < 190; x += 3) {
          if (rand() < 0.82) {
            g2d.fillStyle = cols[(rand() * cols.length) | 0];
            g2d.fillRect(x, y, 2, 3);
          }
        }
      }
      const crowdTex = new THREE.CanvasTexture(cvs);
      crowdTex.colorSpace = THREE.SRGBColorSpace;
      crowdTex.wrapS = THREE.RepeatWrapping;
      crowdTex.magFilter = THREE.NearestFilter;
      const cg = new THREE.BufferGeometry();
      cg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(crowdPos), 3));
      cg.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(crowdUv), 2));
      cg.setIndex(crowdIdx);
      cg.computeVertexNormals();
      const crowdMesh = new THREE.Mesh(cg,
        new THREE.MeshLambertMaterial({ map: crowdTex, side: THREE.DoubleSide }));
      group.add(structMesh, roofMesh, crowdMesh);
    }
  }

  // ------------------------------------------------------------- trees
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
      if (standFootprints.some((fp) =>
        (p.x - fp.x) * (p.x - fp.x) + (p.z - fp.z) * (p.z - fp.z) < fp.r * fp.r)) continue;
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

  // ------------------------------------------- gravel-trap run-off beds
  // Sandy beds on the OUTSIDE of the two tightest corners. The physics
  // terrain mesh is untouched — this is a thin visual lid draped over the
  // shoulder using the terrain's own vertices (rows 0/1 = −side shoulder,
  // rows 2/3 = +side; trackgen layout v(i,row) = i*4+row), lifted 5 cm.
  {
    const pos = [], col = [], gidx = [];
    const sand = new THREE.Color(0x8a7f63);
    for (const z of zones.slice(0, 2)) {
      const out = -insideSign(z.peakIdx);
      const rIn = out > 0 ? 2 : 1, rOut = out > 0 ? 3 : 0;
      const i0 = (z.start - 3 + n) % n;
      const len = Math.min(n, z.len + 6);
      const vbase = pos.length / 3;
      const A = new THREE.Vector3(), B = new THREE.Vector3(), u = new THREE.Vector3();
      for (let s = 0; s < len; s++) {
        const i = (i0 + s) % n;
        u.set(track.up[i][0], track.up[i][1], track.up[i][2]);
        A.fromArray(track.verts, (i * 4 + rIn) * 3);
        B.fromArray(track.verts, (i * 4 + rOut) * 3);
        for (const f of [0.05, 0.5, 0.92]) { // 3 columns → mottle across width
          const p = A.clone().lerp(B, f).addScaledVector(u, 0.05);
          pos.push(p.x, p.y, p.z);
          const m = 1 + (rand() - 0.5) * 0.22; // vertex-color grain
          col.push(sand.r * m, sand.g * m, sand.b * (m + (rand() - 0.5) * 0.1));
        }
        if (s) {
          const a = vbase + (s - 1) * 3, b = vbase + s * 3;
          for (let k = 0; k < 2; k++) gidx.push(a + k, b + k, a + k + 1, a + k + 1, b + k, b + k + 1);
        }
      }
    }
    if (pos.length) {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pos), 3));
      g.setAttribute("color", new THREE.BufferAttribute(new Float32Array(col), 3));
      g.setIndex(gidx);
      g.computeVertexNormals();
      group.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({
        vertexColors: true, roughness: 1.0, metalness: 0.0, side: THREE.DoubleSide })));
    }
  }

  // ------------------------------------------- Armco guardrails
  // Medium-speed corners (0.011 < |κ| ≤ KERB_KAPPA) get instanced W-profile
  // steel rail where the tire stacks are NOT (stacks own the tightest
  // corners). One 2.55 m rail per sample (2.5 m spacing → continuous run) +
  // one post per rail, 10–14 m off the centerline, following the shoulder
  // slope. Capped at ~380 rails.
  {
    const MED_LO = 0.011;
    const med = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const ak = Math.abs(kappa[i]);
      med[i] = ak > MED_LO && ak <= KERB_KAPPA ? 1 : 0;
    }
    // Rails 10–14 m off one segment can clip another part of the loop —
    // reject against samples more than 14 indices away (own zone excluded).
    const clearOfOtherLoop = (p, i) => {
      for (let k = 0; k < n; k += 2) {
        const raw = Math.abs(k - i);
        if (Math.min(raw, n - raw) <= 14) continue;
        const c = track.center[k];
        const dx = p.x - c[0], dz = p.z - c[2];
        const r = track.width[k] / 2 + 5;
        if (dx * dx + dz * dz < r * r) return false;
      }
      return true;
    };
    const rails = [];
    for (let i = 0; i < n; i++) {
      if (!med[i]) continue;
      // skip blips: require ≥5 consecutive medium samples around i
      let runLen = 1;
      for (let d = 1; d < 6 && med[(i + d) % n]; d++) runLen++;
      for (let d = 1; d < 6 && med[(i - d + n) % n]; d++) runLen++;
      if (runLen < 5) continue;
      const out = -insideSign(i);
      const half = track.width[i] / 2;
      const d = Math.min(14, Math.max(10, half + 4.5));
      const f = frame(i);
      const p = f.c.clone().addScaledVector(f.side, out * d);
      p.y -= 1.2 * Math.min(1, Math.max(0, (d - half) / 8)); // shoulder drop
      if (!clearOfOtherLoop(p, i)) continue;
      rails.push({ p, t: f.t, u: f.u, out });
    }
    if (rails.length) {
      const stride = Math.max(1, Math.ceil(rails.length / 380));
      const use = stride > 1 ? rails.filter((_, k) => k % stride === 0) : rails;
      // W-profile cross-section (y up, z toward the track), extruded along x.
      const prof = [[0.78, 0.0], [0.72, 0.075], [0.655, 0.02], [0.59, 0.075], [0.53, 0.0]];
      const rp = new Float32Array(prof.length * 2 * 3);
      prof.forEach(([y, zz], k) => {
        rp.set([-1.275, y, zz], k * 6);
        rp.set([1.275, y, zz], k * 6 + 3);
      });
      const ridx = [];
      for (let k = 0; k < prof.length - 1; k++) {
        ridx.push(2 * k, 2 * k + 1, 2 * k + 2, 2 * k + 2, 2 * k + 1, 2 * k + 3);
      }
      const railGeo = new THREE.BufferGeometry();
      railGeo.setAttribute("position", new THREE.BufferAttribute(rp, 3));
      railGeo.setIndex(ridx);
      railGeo.computeVertexNormals();
      const railMesh = new THREE.InstancedMesh(railGeo, new THREE.MeshStandardMaterial({
        color: 0x9ca4ad, metalness: 0.75, roughness: 0.4, side: THREE.DoubleSide }), use.length);
      const postGeo = new THREE.BoxGeometry(0.14, 0.95, 0.1);
      postGeo.translate(0, 0.32, 0); // sunk slightly, top just under the rail
      const postMesh = new THREE.InstancedMesh(postGeo, new THREE.MeshStandardMaterial({
        color: 0x565d66, metalness: 0.4, roughness: 0.6 }), use.length);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
      const x = new THREE.Vector3(), zed = new THREE.Vector3(), po = new THREE.Vector3();
      use.forEach((st, k) => {
        x.copy(st.t).multiplyScalar(-st.out); // right-handed: x×u = out·side
        zed.crossVectors(x, st.u);
        q.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, st.u, zed));
        m4.compose(st.p, q, one);
        railMesh.setMatrixAt(k, m4);
        po.copy(st.p).addScaledVector(zed, -0.06); // post tucked behind rail
        m4.compose(po, q, one);
        postMesh.setMatrixAt(k, m4);
      });
      group.add(railMesh, postMesh);
    }
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

// Trackside ambience — procedural, seeded, PBR-upgraded circuit world.
// Deterministic from track bytes (fnv1a64 → LCG), zero external assets:
//   • Textured ground plain + distant multi-harmonic alpine mountain horizon ring;
//   • Multi-tier evergreen conifers + broadleaf clusters (InstancedMesh PBR),
//     terrain-grounded on v2 tracks with apron slope rejection;
//   • Collision-exact tire stacks, corrugated W-profile Armco & concrete walls
//     on v2 tracks (heuristic fallback on v1);
//   • Gravel-trap run-off beds + apex chevron sightline boards on tight corners;
//   • FIA start/finish gantry with live 5-red-light F1 start sequence synced to
//     countdown via group.userData.setStartLights(litCount, isGo);
//   • High-contrast 150 / 100 / 50 m braking boards before ALL major heavy
//     braking zones (up to 4 distinct corners around the circuit).
import * as THREE from "../vendor/three.module.js";
import { fnv1a64, smoothedCurvature, KERB_KAPPA, grassTexture } from "./track.js";

const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

/** Merge an array of non-indexed or indexed BufferGeometries (positions + normals). */
function mergeSimpleGeos(geos) {
  const expanded = geos.map((g) => (g.index ? g.toNonIndexed() : g));
  let totalVerts = 0;
  for (const g of expanded) totalVerts += g.attributes.position.count;
  const pos = new Float32Array(totalVerts * 3);
  const nrm = new Float32Array(totalVerts * 3);
  let off = 0;
  for (const g of expanded) {
    if (!g.attributes.normal) g.computeVertexNormals();
    pos.set(g.attributes.position.array, off * 3);
    nrm.set(g.attributes.normal.array, off * 3);
    off += g.attributes.position.count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  out.setAttribute("normal", new THREE.BufferAttribute(nrm, 3));
  return out;
}

/** Procedural crisp FIA distance board face ("150", "100", "50" with stripes). */
function brakeBoardTexture(meters, bars) {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 176;
  const g = c.getContext("2d");
  g.fillStyle = "#f2f4f7";
  g.fillRect(0, 0, 256, 176);
  g.strokeStyle = "#1a1f28";
  g.lineWidth = 8;
  g.strokeRect(4, 4, 248, 168);
  // Diagonal FIA red countdown stripes on left third
  g.fillStyle = "#d9342b";
  for (let k = 0; k < bars; k++) {
    g.fillRect(18 + k * 24, 22, 14, 132);
  }
  // Bold distance numeral on right
  g.fillStyle = "#11161f";
  g.font = "900 84px sans-serif";
  g.textAlign = "right";
  g.textBaseline = "middle";
  g.fillText(String(meters), 240, 94);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

/** Procedural high-visibility yellow/black apex chevron board. */
function chevronTexture(dirSign) {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 128;
  const g = c.getContext("2d");
  g.fillStyle = "#f0c824";
  g.fillRect(0, 0, 256, 128);
  g.fillStyle = "#14181f";
  for (let k = -1; k < 4; k++) {
    const x0 = k * 72 + 12;
    g.beginPath();
    if (dirSign >= 0) {
      g.moveTo(x0, 0); g.lineTo(x0 + 36, 0); g.lineTo(x0 + 68, 64);
      g.lineTo(x0 + 36, 128); g.lineTo(x0, 128); g.lineTo(x0 + 32, 64);
    } else {
      g.moveTo(x0 + 68, 0); g.lineTo(x0 + 32, 0); g.lineTo(x0, 64);
      g.lineTo(x0 + 32, 128); g.lineTo(x0 + 68, 128); g.lineTo(x0 + 36, 64);
    }
    g.closePath();
    g.fill();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/** Start gantry header banner texture. */
function gantryBannerTexture() {
  const c = document.createElement("canvas");
  c.width = 512; c.height = 128;
  const g = c.getContext("2d");
  g.fillStyle = "#0b1019";
  g.fillRect(0, 0, 512, 128);
  g.fillStyle = "#22e0d0";
  g.fillRect(0, 0, 512, 8);
  g.fillRect(0, 120, 512, 8);
  g.fillStyle = "#eef4fb";
  g.font = "900 48px sans-serif";
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText("STTR · APEX CIRCUIT", 256, 56);
  g.fillStyle = "#22e0d0";
  g.font = "700 20px monospace";
  g.fillText("400 HZ DETERMINISTIC TIME TRIAL", 256, 98);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

/** @returns THREE.Group (userData.glowMaterials + userData.setStartLights). */
export function buildAmbience(track, hash = null) {
  const group = new THREE.Group();
  const glowMaterials = [];
  group.userData.glowMaterials = glowMaterials;
  const n = track.S;
  const props = track.props && track.props.length ? track.props : null;

  let seed = (Number((hash ?? fnv1a64(track.bytes)) & 0xffffffffn) >>> 0) || 1;
  const rand = () => (seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 2 ** 32;

  const frame = (i) => {
    const c = v3(track.center[i]);
    const u = v3(track.up[i]).normalize();
    const t = v3(track.tangent[i]).normalize();
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    return { c, u, t, side };
  };
  const insideSign = (i) => {
    const f = frame(i);
    const t2 = v3(track.tangent[(i + 1) % n]);
    return Math.sign(t2.sub(f.t).dot(f.side)) || 1;
  };

  // ------------------------------------------------------------- ground + mountain horizon
  let minY = Infinity, maxCY = -Infinity, cx = 0, cz = 0, maxR = 0;
  for (const c of track.center) {
    minY = Math.min(minY, c[1]);
    maxCY = Math.max(maxCY, c[1]);
    cx += c[0] / n; cz += c[2] / n;
  }
  for (const c of track.center) maxR = Math.max(maxR, Math.hypot(c[0] - cx, c[2] - cz));
  const groundY = minY - 1.35;

  const groundTex = grassTexture();
  groundTex.repeat.set(36, 36);
  const groundGeo = new THREE.CircleGeometry(maxR + 360, 48);
  groundGeo.rotateX(-Math.PI / 2);
  const ground = new THREE.Mesh(
    groundGeo,
    new THREE.MeshStandardMaterial({ color: 0x2c3c30, map: groundTex, roughness: 0.96, metalness: 0.0, envMapIntensity: 0.3 }),
  );
  ground.position.set(cx, groundY - 0.02, cz);
  group.add(ground);

  // Distant rolling mountain silhouette ring — encircles the horizon past the
  // circuit so the world feels like a real alpine valley circuit (Spa/Red Bull Ring).
  {
    const segs = 72;
    const rIn = maxR + 135;
    const rPeak = maxR + 235;
    const rOut = maxR + 340;
    const p1 = rand() * 6.28, p2 = rand() * 6.28, p3 = rand() * 6.28;
    const mPos = new Float32Array(segs * 3 * 3);
    const mCol = new Float32Array(segs * 3 * 3);
    const mIdx = [];
    const baseCol = new THREE.Color(0x28382f);
    const ridgeCol = new THREE.Color(0x3a4952);
    const peakCol = new THREE.Color(0x566470);
    for (let k = 0; k < segs; k++) {
      const a = (k / segs) * Math.PI * 2;
      const ca = Math.cos(a), sa = Math.sin(a);
      const ridgeH = 26
        + 18 * Math.sin(3 * a + p1)
        + 11 * Math.sin(7 * a + p2)
        + 6 * Math.cos(13 * a + p3)
        + 4 * Math.sin(19 * a + p1 * 1.7);
      const h = Math.max(10, ridgeH);
      const b0 = k * 9;
      mPos.set([cx + ca * rIn, groundY - 1.5, cz + sa * rIn], b0);
      mPos.set([cx + ca * rPeak, groundY + h, cz + sa * rPeak], b0 + 3);
      mPos.set([cx + ca * rOut, groundY - 4.0, cz + sa * rOut], b0 + 6);
      const peakMix = Math.min(1, Math.max(0, (h - 24) / 30));
      const rc = ridgeCol.clone().lerp(peakCol, peakMix);
      mCol.set([baseCol.r, baseCol.g, baseCol.b], b0);
      mCol.set([rc.r, rc.g, rc.b], b0 + 3);
      mCol.set([baseCol.r * 0.7, baseCol.g * 0.75, baseCol.b * 0.85], b0 + 6);
      const nk = (k + 1) % segs;
      const i0 = k * 3, i1 = nk * 3;
      mIdx.push(i0, i1, i0 + 1, i0 + 1, i1, i1 + 1);
      mIdx.push(i0 + 1, i1 + 1, i0 + 2, i0 + 2, i1 + 1, i1 + 2);
    }
    const mg = new THREE.BufferGeometry();
    mg.setAttribute("position", new THREE.BufferAttribute(mPos, 3));
    mg.setAttribute("color", new THREE.BufferAttribute(mCol, 3));
    mg.setIndex(mIdx);
    mg.computeVertexNormals();
    const mountains = new THREE.Mesh(mg, new THREE.MeshStandardMaterial({
      vertexColors: true, roughness: 0.96, metalness: 0.0, envMapIntensity: 0.25,
    }));
    group.add(mountains);
  }

  // Skirts (v1 ONLY)
  if (!props) {
    const skirtMat = new THREE.MeshStandardMaterial({ color: 0x2d3b31, roughness: 0.94, metalness: 0.0, side: THREE.DoubleSide });
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

  // ---------------------------------------------- terrain height sampling (v2)
  const APRON_COS = Math.cos((20 * Math.PI) / 180);
  const groundHit = (() => {
    if (!props) return () => null;
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(track.verts, 3));
    g.setIndex(new THREE.BufferAttribute(track.tris, 1));
    const mesh = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ side: THREE.DoubleSide }));
    const ray = new THREE.Raycaster();
    ray.ray.direction.set(0, -1, 0);
    return (x, z) => {
      ray.ray.origin.set(x, maxCY + 25, z);
      const hits = ray.intersectObject(mesh, false);
      return hits.length ? hits[0] : null;
    };
  })();

  // ------------------------------------------- curvature + corner zones
  const kappa = smoothedCurvature(track);
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
  const standFootprints = [];
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
      const hit = groundHit(base.x, base.z);
      if (hit) {
        if (Math.abs(hit.face.normal.y) < APRON_COS) return false;
        base.y = hit.point.y;
      }
      standDefs.push({ base, th, sh, out, L, tiers });
      standFootprints.push({
        x: base.x + sh.x * out * depth / 2, z: base.z + sh.z * out * depth / 2,
        r: Math.hypot(L / 2, depth / 2) + 2,
      });
      return true;
    };
    let run = 1;
    while (run < 15 && Math.abs(kappa[run % n]) < 0.008 && Math.abs(kappa[(n - run) % n]) < 0.008) run++;
    const mainL = Math.max(30, Math.min(72, run * 2 * 2.5 * 0.8));
    const d0 = track.width[0] / 2 + 11;
    tryStand(0, 1, mainL, 5, d0) || tryStand(0, -1, mainL, 5, d0);
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
        for (let k = 0; k < tiers; k++) {
          const h = (k + 1) * STEP_H;
          structXf.push({ p: at(0, k * STEP_D + STEP_D / 2, h / 2), q, s: new THREE.Vector3(L, h, STEP_D) });
        }
        const wallH = tiers * STEP_H + 1.4;
        structXf.push({ p: at(0, tiers * STEP_D + 0.25, wallH / 2), q, s: new THREE.Vector3(L, wallH, 0.5) });
        const roofY = tiers * STEP_H + 2.6;
        for (const dl of [-L / 2 + 0.4, L / 2 - 0.4]) {
          structXf.push({ p: at(dl, 0.3, roofY / 2), q, s: new THREE.Vector3(0.22, roofY, 0.22) });
        }
        roofXf.push({
          p: at(0, tiers * STEP_D / 2 - 0.4, roofY + 0.09), q,
          s: new THREE.Vector3(L + 1.2, 0.18, tiers * STEP_D + 2.0),
        });
        const vbase = crowdPos.length / 3;
        for (const [dOut, y] of [[0.25, STEP_H + 0.35], [(tiers - 1) * STEP_D + 0.9, tiers * STEP_H + 0.4]]) {
          for (const dl of [-L / 2, L / 2]) {
            const p = at(dl, dOut, y);
            crowdPos.push(p.x, p.y, p.z);
          }
        }
        const rep = L / 24;
        crowdUv.push(0, 0, rep, 0, 0, 1, rep, 1);
        crowdIdx.push(vbase, vbase + 1, vbase + 2, vbase + 1, vbase + 3, vbase + 2);
      }
      const boxGeo = new THREE.BoxGeometry(1, 1, 1);
      const m4 = new THREE.Matrix4();
      const structMesh = new THREE.InstancedMesh(boxGeo,
        new THREE.MeshStandardMaterial({ color: 0x767e88, roughness: 0.85, metalness: 0.12 }),
        structXf.length);
      structXf.forEach((xf, k) => { m4.compose(xf.p, xf.q, xf.s); structMesh.setMatrixAt(k, m4); });
      const roofMesh = new THREE.InstancedMesh(boxGeo,
        new THREE.MeshStandardMaterial({ color: 0x161a21, roughness: 0.48, metalness: 0.45 }),
        roofXf.length);
      roofXf.forEach((xf, k) => { m4.compose(xf.p, xf.q, xf.s); roofMesh.setMatrixAt(k, m4); });

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
        new THREE.MeshStandardMaterial({ map: crowdTex, roughness: 0.9, metalness: 0.0, side: THREE.DoubleSide }));
      group.add(structMesh, roofMesh, crowdMesh);
    }
  }

  // ------------------------------------------------------------- multi-tier PBR trees
  const conifers = [], broads = [];
  for (let i = 0; i < n; i += 4 + ((rand() * 3) | 0)) {
    for (const s of [-1, 1]) {
      if (rand() < 0.22) continue;
      const f = frame(i);
      const lat = track.width[i] / 2 + 12 + rand() * 16;
      const p = f.c.clone()
        .addScaledVector(f.side, s * lat)
        .addScaledVector(f.t, (rand() - 0.5) * 4.5);
      p.y = groundY;
      if (!clearOfTrack(p)) continue;
      if (standFootprints.some((fp) =>
        (p.x - fp.x) * (p.x - fp.x) + (p.z - fp.z) * (p.z - fp.z) < fp.r * fp.r)) continue;
      const hit = groundHit(p.x, p.z);
      if (hit) {
        if (Math.abs(hit.face.normal.y) < APRON_COS) continue;
        p.y = hit.point.y;
      }
      const entry = { p, scale: 0.82 + rand() * 0.72, rot: rand() * Math.PI * 2 };
      (rand() < 0.68 ? conifers : broads).push(entry);
    }
  }
  const barkMat = new THREE.MeshStandardMaterial({ color: 0x3b2e24, roughness: 0.95, metalness: 0.0 });
  const leafMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.88, metalness: 0.0, envMapIntensity: 0.4 });
  const m4t = new THREE.Matrix4(), qt = new THREE.Quaternion(), upY = new THREE.Vector3(0, 1, 0);

  if (conifers.length) {
    const trunkGeo = new THREE.CylinderGeometry(0.14, 0.24, 1.4, 6);
    trunkGeo.translate(0, 0.7, 0);
    // 3-tier layered evergreen crown
    const c1 = new THREE.ConeGeometry(1.45, 2.2, 7); c1.translate(0, 2.1, 0);
    const c2 = new THREE.ConeGeometry(1.15, 2.0, 7); c2.translate(0, 3.25, 0);
    const c3 = new THREE.ConeGeometry(0.75, 1.7, 7); c3.translate(0, 4.35, 0);
    const canopyGeo = mergeSimpleGeos([c1, c2, c3]);
    const trunks = new THREE.InstancedMesh(trunkGeo, barkMat, conifers.length);
    const canopies = new THREE.InstancedMesh(canopyGeo, leafMat, conifers.length);
    const pineCols = [0x1c3426, 0x22422d, 0x182d22, 0x284930].map((c) => new THREE.Color(c));
    conifers.forEach((tr, k) => {
      qt.setFromAxisAngle(upY, tr.rot);
      m4t.compose(tr.p, qt, new THREE.Vector3(tr.scale, tr.scale * (0.95 + rand() * 0.2), tr.scale));
      trunks.setMatrixAt(k, m4t);
      canopies.setMatrixAt(k, m4t);
      canopies.setColorAt(k, pineCols[(rand() * pineCols.length) | 0]);
    });
    group.add(trunks, canopies);
  }

  if (broads.length) {
    const trunkGeo = new THREE.CylinderGeometry(0.16, 0.26, 1.8, 6);
    trunkGeo.translate(0, 0.9, 0);
    const b1 = new THREE.DodecahedronGeometry(1.25, 0); b1.translate(0, 2.8, 0);
    const b2 = new THREE.DodecahedronGeometry(0.88, 0); b2.translate(0.65, 2.4, 0.35);
    const b3 = new THREE.DodecahedronGeometry(0.82, 0); b3.translate(-0.55, 2.55, -0.4);
    const crownGeo = mergeSimpleGeos([b1, b2, b3]);
    const trunks = new THREE.InstancedMesh(trunkGeo, barkMat, broads.length);
    const crowns = new THREE.InstancedMesh(crownGeo, leafMat, broads.length);
    const broadCols = [0x2c4b2b, 0x3b592e, 0x4b5c2d, 0x5c532a].map((c) => new THREE.Color(c));
    broads.forEach((tr, k) => {
      qt.setFromAxisAngle(upY, tr.rot);
      m4t.compose(tr.p, qt, new THREE.Vector3(tr.scale, tr.scale, tr.scale));
      trunks.setMatrixAt(k, m4t);
      crowns.setMatrixAt(k, m4t);
      crowns.setColorAt(k, broadCols[(rand() * broadCols.length) | 0]);
    });
    group.add(trunks, crowns);
  }

  // ------------------------------------------- collision-prop barriers (v2) / heuristics (v1)
  const upY2 = new THREE.Vector3(0, 1, 0);
  const tireMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.78, metalness: 0.04 });

  if (props) {
    const items = [];
    for (const pr of props) {
      if (pr.type !== 0) continue;
      const [hx, hy, hz] = pr.half;
      const alongX = hx >= hz;
      const halfLong = Math.max(alongX ? hx : hz, 0.05);
      const r = Math.max(0.05, Math.min(hx, hz));
      const cols = Math.max(1, Math.min(96, Math.round(halfLong / r)));
      const rLong = halfLong / cols;
      const levels = Math.max(1, Math.min(8, Math.round((2 * hy) / 0.55)));
      const levelH = Math.max(0.1, (2 * hy) / levels);
      items.push({ pr, alongX, halfLong, r, cols, rLong, levels, levelH });
    }
    const total = items.reduce((s, it) => s + it.cols * it.levels, 0);
    if (total) {
      const tireGeo = new THREE.CylinderGeometry(1, 1, 1, 12);
      const barrier = new THREE.InstancedMesh(tireGeo, tireMat, total);
      const red = new THREE.Color(0xc43127), white = new THREE.Color(0xe1e5ea);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const dir = new THREE.Vector3(), p = new THREE.Vector3(), s = new THREE.Vector3();
      let k = 0;
      for (const it of items) {
        const { pr } = it;
        q.setFromAxisAngle(upY2, pr.yaw);
        dir.set(it.alongX ? 1 : 0, 0, it.alongX ? 0 : 1).applyQuaternion(q);
        s.set(it.alongX ? it.rLong : it.r, it.levelH, it.alongX ? it.r : it.rLong);
        const bottomY = pr.pos[1] - pr.half[1];
        for (let c = 0; c < it.cols; c++) {
          for (let lev = 0; lev < it.levels; lev++) {
            p.set(pr.pos[0], 0, pr.pos[2])
              .addScaledVector(dir, -it.halfLong + (2 * c + 1) * it.rLong);
            p.y = bottomY + (lev + 0.5) * it.levelH;
            m4.compose(p, q, s);
            barrier.setMatrixAt(k, m4);
            barrier.setColorAt(k, (c + lev) % 2 ? red : white);
            k++;
          }
        }
      }
      group.add(barrier);
    }
  } else {
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
      const tireGeo = new THREE.CylinderGeometry(0.55, 0.55, 0.55, 12);
      const barrier = new THREE.InstancedMesh(tireGeo, tireMat, stacks.length * 2);
      const red = new THREE.Color(0xc43127), white = new THREE.Color(0xe1e5ea);
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
  }

  // ------------------------------------------- gravel-trap run-off beds
  {
    const pos = [], col = [], gidx = [];
    const sand = new THREE.Color(0x8e8264);
    for (const z of zones.slice(0, 3)) {
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
        for (const f of [0.05, 0.5, 0.92]) {
          const p = A.clone().lerp(B, f).addScaledVector(u, 0.05);
          pos.push(p.x, p.y, p.z);
          const m = 1 + (rand() - 0.5) * 0.24;
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
        vertexColors: true, roughness: 0.98, metalness: 0.0, side: THREE.DoubleSide,
      })));
    }
  }

  // ------------------------------------------- Armco guardrails + concrete walls
  if (props) {
    const armco = props.filter((p) => p.type === 1);
    if (armco.length) {
      const loop = [
        [1, 0], [0.52, 1], [0, 0.267], [-0.52, 1], [-1, 0],
        [-0.52, -1], [0, -0.267], [0.52, -1],
      ];
      const m = loop.length;
      const rp = new Float32Array(m * 2 * 3);
      loop.forEach(([y, z], k) => {
        rp.set([-1, y, z], k * 6);
        rp.set([1, y, z], k * 6 + 3);
      });
      const ridx = [];
      for (let k = 0; k < m; k++) {
        const j = (k + 1) % m;
        ridx.push(2 * k, 2 * k + 1, 2 * j, 2 * j, 2 * k + 1, 2 * j + 1);
      }
      const railGeo = new THREE.BufferGeometry();
      railGeo.setAttribute("position", new THREE.BufferAttribute(rp, 3));
      railGeo.setIndex(ridx);
      railGeo.computeVertexNormals();
      const railMesh = new THREE.InstancedMesh(railGeo, new THREE.MeshStandardMaterial({
        color: 0xa4adb8, metalness: 0.82, roughness: 0.32, side: THREE.DoubleSide,
      }), armco.length);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const p = new THREE.Vector3(), s = new THREE.Vector3();
      armco.forEach((pr, k) => {
        q.setFromAxisAngle(upY2, pr.yaw);
        p.set(pr.pos[0], pr.pos[1], pr.pos[2]);
        s.set(pr.half[0], pr.half[1], pr.half[2]);
        m4.compose(p, q, s);
        railMesh.setMatrixAt(k, m4);
      });
      group.add(railMesh);
    }
    const walls = props.filter((p) => p.type === 2);
    if (walls.length) {
      const wallGeo = new THREE.BoxGeometry(2, 2, 2);
      const wallMesh = new THREE.InstancedMesh(wallGeo, new THREE.MeshStandardMaterial({
        color: 0x3b4048, roughness: 0.78, metalness: 0.14,
      }), walls.length);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const p = new THREE.Vector3(), s = new THREE.Vector3();
      walls.forEach((pr, k) => {
        q.setFromAxisAngle(upY2, pr.yaw);
        p.set(pr.pos[0], pr.pos[1], pr.pos[2]);
        s.set(pr.half[0], pr.half[1], pr.half[2]);
        m4.compose(p, q, s);
        wallMesh.setMatrixAt(k, m4);
      });
      group.add(wallMesh);
    }
  } else {
    const MED_LO = 0.011;
    const med = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const ak = Math.abs(kappa[i]);
      med[i] = ak > MED_LO && ak <= KERB_KAPPA ? 1 : 0;
    }
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
      let runLen = 1;
      for (let d = 1; d < 6 && med[(i + d) % n]; d++) runLen++;
      for (let d = 1; d < 6 && med[(i - d + n) % n]; d++) runLen++;
      if (runLen < 5) continue;
      const out = -insideSign(i);
      const half = track.width[i] / 2;
      const d = Math.min(14, Math.max(10, half + 4.5));
      const f = frame(i);
      const p = f.c.clone().addScaledVector(f.side, out * d);
      p.y -= 1.2 * Math.min(1, Math.max(0, (d - half) / 8));
      if (!clearOfOtherLoop(p, i)) continue;
      rails.push({ p, t: f.t, u: f.u, out });
    }
    if (rails.length) {
      const stride = Math.max(1, Math.ceil(rails.length / 380));
      const use = stride > 1 ? rails.filter((_, k) => k % stride === 0) : rails;
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
        color: 0xa4adb8, metalness: 0.82, roughness: 0.32, side: THREE.DoubleSide,
      }), use.length);
      const postGeo = new THREE.BoxGeometry(0.14, 0.95, 0.1);
      postGeo.translate(0, 0.32, 0);
      const postMesh = new THREE.InstancedMesh(postGeo, new THREE.MeshStandardMaterial({
        color: 0x565d66, metalness: 0.45, roughness: 0.55,
      }), use.length);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
      const x = new THREE.Vector3(), zed = new THREE.Vector3(), po = new THREE.Vector3();
      use.forEach((st, k) => {
        x.copy(st.t).multiplyScalar(-st.out);
        zed.crossVectors(x, st.u);
        q.setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, st.u, zed));
        m4.compose(st.p, q, one);
        railMesh.setMatrixAt(k, m4);
        po.copy(st.p).addScaledVector(zed, -0.06);
        m4.compose(po, q, one);
        postMesh.setMatrixAt(k, m4);
      });
      group.add(railMesh, postMesh);
    }
  }

  // ------------------------------------------- start/finish gantry + 5-light FIA start rig
  const startLightMats = [];
  {
    const f = frame(0);
    const gantry = new THREE.Group();
    gantry.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
    gantry.position.copy(f.c);
    const steel = new THREE.MeshStandardMaterial({ color: 0x1e242d, metalness: 0.65, roughness: 0.38 });
    const postX = track.width[0] / 2 + 1.65;
    for (const s of [-1, 1]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.42, 5.8, 0.48), steel);
      post.position.set(s * postX, 2.9, 0);
      gantry.add(post);
      // Concrete plinth base
      const plinth = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.8, 0.85),
        new THREE.MeshStandardMaterial({ color: 0x626973, roughness: 0.85 }));
      plinth.position.set(s * postX, 0.3, 0);
      gantry.add(plinth);
    }
    const beam = new THREE.Mesh(new THREE.BoxGeometry(2 * postX + 0.5, 0.55, 0.6), steel);
    beam.position.set(0, 5.85, 0);
    gantry.add(beam);

    // Overhead timing bridge banner (textured)
    const bannerTex = gantryBannerTexture();
    const bannerMat = new THREE.MeshStandardMaterial({
      map: bannerTex, emissiveMap: bannerTex, emissive: 0xffffff, emissiveIntensity: 0.55,
      roughness: 0.4, metalness: 0.1,
    });
    const banner = new THREE.Mesh(new THREE.BoxGeometry(6.2, 1.35, 0.24), bannerMat);
    banner.position.set(0, 5.85, -0.22);
    gantry.add(banner);

    // Underside warm floodlight strip
    const stripMat = new THREE.MeshStandardMaterial({
      color: 0x0c0f14, emissive: 0xfff0c8, emissiveIntensity: 0.65, roughness: 0.3,
    });
    const strip = new THREE.Mesh(new THREE.BoxGeometry(2 * postX - 0.6, 0.08, 0.12), stripMat);
    strip.position.set(0, 5.54, 0);
    gantry.add(strip);
    glowMaterials.push(stripMat, bannerMat);

    // 5 FIA start-light pods suspended beneath the bridge facing approaching cars (-Z)
    const podHousing = new THREE.Mesh(new THREE.BoxGeometry(3.5, 0.48, 0.22), steel);
    podHousing.position.set(0, 4.78, -0.35);
    gantry.add(podHousing);
    const podGeo = new THREE.CircleGeometry(0.15, 16);
    for (let k = 0; k < 5; k++) {
      const lm = new THREE.MeshStandardMaterial({
        color: 0x1a0808, emissive: 0xff2218, emissiveIntensity: 0.08, roughness: 0.2, metalness: 0.1,
      });
      const lamp = new THREE.Mesh(podGeo, lm);
      lamp.position.set((k - 2) * 0.62, 4.78, -0.465);
      lamp.rotation.y = Math.PI; // face approaching driver (-Z)
      gantry.add(lamp);
      startLightMats.push(lm);
    }
    group.add(gantry);
  }

  /** Sync 5 FIA start gantry pods with countdown: litCount 0..5 red, or isGo = green flash. */
  group.userData.setStartLights = (litCount, isGo = false) => {
    for (let k = 0; k < startLightMats.length; k++) {
      const m = startLightMats[k];
      if (isGo) {
        m.emissive.setHex(0x22e078);
        m.emissiveIntensity = 3.2;
      } else if (k < litCount) {
        m.emissive.setHex(0xff241a);
        m.emissiveIntensity = 3.6;
      } else {
        m.emissive.setHex(0x330808);
        m.emissiveIntensity = 0.06;
      }
    }
  };

  // ------------------------------------------- braking boards (top 4 heavy corners) + apex chevrons
  {
    const tex150 = brakeBoardTexture(150, 3);
    const tex100 = brakeBoardTexture(100, 2);
    const tex50 = brakeBoardTexture(50, 1);
    const boardMats = [tex150, tex100, tex50].map((tex) => new THREE.MeshBasicMaterial({ map: tex }));
    const backMat = new THREE.MeshStandardMaterial({ color: 0x1e232b, roughness: 0.7 });
    const postMat = new THREE.MeshStandardMaterial({ color: 0x202630, roughness: 0.6, metalness: 0.5 });

    // Pick up to 4 well-separated heavy braking zones
    const chosenZones = [];
    for (const z of zones) {
      if (chosenZones.length >= 4) break;
      const farEnough = chosenZones.every((oz) => {
        const d = Math.abs(z.start - oz.start);
        return Math.min(d, n - d) > 55;
      });
      if (farEnough) chosenZones.push(z);
    }

    for (const z of chosenZones) {
      const out = -insideSign(z.peakIdx);
      const specs = [[60, boardMats[0]], [40, boardMats[1]], [20, boardMats[2]]];
      for (const [back, faceMat] of specs) {
        const bi = (z.start - back + n) % n;
        // Skip placing a board inside another tight turn
        if (Math.abs(kappa[bi]) > 0.018) continue;
        const f = frame(bi);
        const pos = f.c.clone().addScaledVector(f.side, out * (track.width[bi] / 2 + 2.8));
        const board = new THREE.Group();
        board.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
        board.position.copy(pos);
        const post = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.05, 0.08), postMat);
        post.position.y = 0.52;
        board.add(post);
        // Box faces: +x,-x,+y,-y,+z,-z — local -z faces approaching cars
        const face = new THREE.Mesh(new THREE.BoxGeometry(1.18, 0.82, 0.06), [
          backMat, backMat, backMat, backMat, backMat, faceMat,
        ]);
        face.position.y = 1.42;
        board.add(face);
        group.add(board);
      }

      // Apex chevron sightline board on the outside of each signature corner
      const fApex = frame(z.peakIdx);
      const cTex = chevronTexture(insideSign(z.peakIdx));
      const cFaceMat = new THREE.MeshBasicMaterial({ map: cTex });
      const chev = new THREE.Group();
      chev.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(fApex.side, fApex.u, fApex.t));
      chev.position.copy(fApex.c).addScaledVector(fApex.side, out * (track.width[z.peakIdx] / 2 + 3.6));
      const cPostL = new THREE.Mesh(new THREE.BoxGeometry(0.08, 1.1, 0.08), postMat);
      cPostL.position.set(-0.75, 0.55, 0);
      const cPostR = cPostL.clone();
      cPostR.position.x = 0.75;
      const cFace = new THREE.Mesh(new THREE.BoxGeometry(2.1, 0.95, 0.08), [
        backMat, backMat, backMat, backMat, backMat, cFaceMat,
      ]);
      cFace.position.y = 1.45;
      chev.add(cPostL, cPostR, cFace);
      group.add(chev);
    }
  }

  return group;
}

// TRK1 parser (CONTRACTS §8) + PBR three.js track geometry + minimap rendering.
// Procedural PBR asphalt (albedo + Sobel micro-relief normal map + aggregate/
// groove roughness map), world-UV textured turf shoulders/aprons, 3D white edge
// ribbons (zero 1px WebGL aliasing), FIA checkered start line + staggered grid
// boxes, painted bevel kerbs, rubbered-in racing groove, and ghost-aware minimap.
import * as THREE from "../vendor/three.module.js";

export function parseTrack(buf) {
  const bytes = new Uint8Array(buf);
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== 0x4b525453) throw new Error("bad TRK1 magic");
  const version = dv.getUint16(4, true);
  if (version !== 1 && version !== 2) throw new Error("bad TRK1 version");
  const checkpointCount = dv.getUint16(6, true);
  const S = dv.getUint32(16, true);
  const V = dv.getUint32(20, true);
  const T = dv.getUint32(24, true);

  const center = [], up = [], tangent = [], width = [];
  for (let i = 0; i < S; i++) {
    const o = 28 + 40 * i;
    center.push([dv.getFloat32(o, true), dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true)]);
    up.push([dv.getFloat32(o + 12, true), dv.getFloat32(o + 16, true), dv.getFloat32(o + 20, true)]);
    tangent.push([dv.getFloat32(o + 24, true), dv.getFloat32(o + 28, true), dv.getFloat32(o + 32, true)]);
    width.push(dv.getFloat32(o + 36, true));
  }
  let o = 28 + 40 * S;
  const checkpoints = [];
  for (let c = 0; c < checkpointCount; c++) checkpoints.push(dv.getUint32(o + 4 * c, true));
  o += 4 * checkpointCount;
  const verts = new Float32Array(buf, o, V * 3); o += 12 * V;
  const tris = new Uint32Array(buf, o, T * 3); o += 12 * T;
  const spawnIndex = dv.getUint32(o, true);
  o += 12;

  const props = [];
  if (version === 2) {
    const propCount = dv.getUint32(o, true);
    o += 4;
    if (propCount > 4096) throw new Error("bad TRK1 prop count");
    for (let p = 0; p < propCount; p++) {
      props.push({
        type: dv.getUint16(o, true),
        pos: [dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true), dv.getFloat32(o + 12, true)],
        yaw: dv.getFloat32(o + 16, true),
        half: [dv.getFloat32(o + 20, true), dv.getFloat32(o + 24, true), dv.getFloat32(o + 28, true)],
      });
      o += 36;
    }
  }

  return { bytes, center, up, tangent, width, checkpoints, verts, tris, spawnIndex, S, props };
}

export function fnv1a64(bytes) {
  let h = 0xcbf29ce484222325n;
  for (let i = 0; i < bytes.length; i++) h = ((h ^ BigInt(bytes[i])) * 0x100000001b3n) & 0xffffffffffffffffn;
  return h;
}

const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

export const KERB_KAPPA = 0.022;
export const KERB_SMOOTH = 15;

/** Smoothed signed curvature per centerline sample (sign = turn direction). */
export function smoothedCurvature(track, smooth = KERB_SMOOTH) {
  const n = track.S;
  const raw = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const t = track.tangent[i];
    const t2 = track.tangent[(i + 1) % n];
    raw[i] = (t[0] * t2[2] - t[2] * t2[0]) / 2.5;
  }
  const kappa = new Float32Array(n);
  const half = (smooth / 2) | 0;
  for (let i = 0; i < n; i++) {
    let sum = 0;
    for (let k = -half; k <= half; k++) sum += raw[(i + k + n) % n];
    kappa[i] = sum / smooth;
  }
  return kappa;
}

/** Procedural PBR asphalt texture suite: albedo + tangent-space normal map +
 *  roughness map. Coarse stone aggregate + fine micro-grit + longitudinal tire-
 *  polish bands + tar seams. Tiled smoothly along arc length & width. */
function asphaltPBRMaps() {
  const N = 256;
  const height = new Float32Array(N * N);
  let s = 0x1234abcd;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  // Fine stone grain + 2x2 medium aggregate clumps
  for (let i = 0; i < N * N; i++) height[i] = rnd() * 0.65;
  for (let y = 0; y < N; y += 2) {
    for (let x = 0; x < N; x += 2) {
      const clump = rnd() * 0.35;
      for (let dy = 0; dy < 2; dy++) {
        for (let dx = 0; dx < 2; dx++) {
          height[((y + dy) & (N - 1)) * N + ((x + dx) & (N - 1))] += clump;
        }
      }
    }
  }

  // Albedo canvas
  const cMap = document.createElement("canvas");
  cMap.width = cMap.height = N;
  const gMap = cMap.getContext("2d");
  const imgMap = gMap.createImageData(N, N);

  // Roughness canvas (green channel used by Three.js PBR)
  const cRough = document.createElement("canvas");
  cRough.width = cRough.height = N;
  const gRough = cRough.getContext("2d");
  const imgRough = gRough.createImageData(N, N);

  // Normal canvas (Sobel derivative of heightfield)
  const cNorm = document.createElement("canvas");
  cNorm.width = cNorm.height = N;
  const gNorm = cNorm.getContext("2d");
  const imgNorm = gNorm.createImageData(N, N);

  for (let y = 0; y < N; y++) {
    // Subtle longitudinal tire-polished wear band near wheel-track lanes
    const vNorm = y / N;
    const laneWear =
      Math.exp(-Math.pow((vNorm - 0.34) / 0.11, 2)) +
      Math.exp(-Math.pow((vNorm - 0.66) / 0.11, 2));
    for (let x = 0; x < N; x++) {
      const idx = y * N + x;
      const h = height[idx];
      const o = idx * 4;

      // Charcoal race-grade asphalt with subtle cool slate tint
      const lum = 72 + h * 36 - laneWear * 10;
      imgMap.data[o] = lum;
      imgMap.data[o + 1] = lum + 3;
      imgMap.data[o + 2] = lum + 8;
      imgMap.data[o + 3] = 255;

      // Roughness: aggregate peaks rough, tire-polished tracks slightly smoother
      const rVal = Math.max(125, Math.min(250, Math.round(218 + (h - 0.5) * 44 - laneWear * 34)));
      imgRough.data[o] = imgRough.data[o + 1] = imgRough.data[o + 2] = rVal;
      imgRough.data[o + 3] = 255;

      // Wrap-aware central differences for tileable normal map
      const xm = (x - 1 + N) & (N - 1), xp = (x + 1) & (N - 1);
      const ym = (y - 1 + N) & (N - 1), yp = (y + 1) & (N - 1);
      const dx = (height[y * N + xp] - height[y * N + xm]) * 1.65;
      const dy = (height[yp * N + x] - height[ym * N + x]) * 1.65;
      const invLen = 1 / Math.hypot(dx, dy, 1);
      imgNorm.data[o] = Math.round((-dx * invLen * 0.5 + 0.5) * 255);
      imgNorm.data[o + 1] = Math.round((-dy * invLen * 0.5 + 0.5) * 255);
      imgNorm.data[o + 2] = Math.round((invLen * 0.5 + 0.5) * 255);
      imgNorm.data[o + 3] = 255;
    }
  }
  gMap.putImageData(imgMap, 0, 0);
  gRough.putImageData(imgRough, 0, 0);
  gNorm.putImageData(imgNorm, 0, 0);

  // Sparse quartz aggregate glints + subtle bitumen patches + longitudinal tar seam
  for (let k = 0; k < 340; k++) {
    gMap.fillStyle = `rgba(185,192,202,${0.12 + rnd() * 0.16})`;
    gMap.fillRect((rnd() * N) | 0, (rnd() * N) | 0, 1 + ((rnd() * 2) | 0), 1);
  }
  for (let k = 0; k < 6; k++) {
    gMap.fillStyle = `rgba(24,28,35,${0.14 + rnd() * 0.12})`;
    gMap.beginPath();
    gMap.ellipse(rnd() * N, rnd() * N, 12 + rnd() * 28, 5 + rnd() * 14, rnd() * 3, 0, 7);
    gMap.fill();
  }
  // Faint centerline paving joint
  gMap.fillStyle = "rgba(18,20,25,0.22)";
  gMap.fillRect(0, (N >> 1) - 1, N, 2);

  const setup = (canvas, isSRGB) => {
    const t = new THREE.CanvasTexture(canvas);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.anisotropy = 8;
    if (isSRGB) t.colorSpace = THREE.SRGBColorSpace;
    return t;
  };
  return {
    map: setup(cMap, true),
    normalMap: setup(cNorm, false),
    roughnessMap: setup(cRough, false),
  };
}

/** Procedural turf texture for shoulders, aprons & plain — breaks up smooth
 *  vertex-colored polygons with mowed grass striping and organic micro-grain. */
export function grassTexture() {
  const N = 256;
  const c = document.createElement("canvas");
  c.width = c.height = N;
  const g = c.getContext("2d");
  const img = g.createImageData(N, N);
  let s = 0x51a7beef;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);
  for (let y = 0; y < N; y++) {
    // Subtle mowed circuit-turf bands
    const stripe = Math.sin((y / N) * Math.PI * 4) * 6;
    for (let x = 0; x < N; x++) {
      const o = (y * N + x) * 4;
      const n = (rnd() - 0.5) * 28 + stripe;
      img.data[o] = Math.max(0, Math.min(255, 206 + n * 0.7));
      img.data[o + 1] = Math.max(0, Math.min(255, 218 + n));
      img.data[o + 2] = Math.max(0, Math.min(255, 198 + n * 0.6));
      img.data[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  // Subtle wildflower / dry-blade flecks
  for (let k = 0; k < 420; k++) {
    g.fillStyle = rnd() > 0.35 ? "rgba(180,196,155,0.22)" : "rgba(92,108,74,0.25)";
    g.fillRect((rnd() * N) | 0, (rnd() * N) | 0, 2, 1 + ((rnd() * 3) | 0));
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

/** Crisp procedural checkered start/finish banner texture. */
function checkerTexture() {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 64;
  const g = c.getContext("2d");
  const cols = 16, rows = 3;
  const cw = c.width / cols, ch = c.height / rows;
  for (let r = 0; r < rows; r++) {
    for (let k = 0; k < cols; k++) {
      g.fillStyle = (r + k) % 2 === 0 ? "#f2f5fa" : "#12161d";
      g.fillRect(k * cw, r * ch, cw + 0.5, ch + 0.5);
    }
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

export function buildTrackMeshes(track) {
  const group = new THREE.Group();

  // ---- Terrain (shoulders + v2 aprons + ground) with PBR textured grass
  const terrainGeo = new THREE.BufferGeometry();
  const terrainVerts = track.verts.slice();
  for (let i = 0; i < track.S; i++) {
    terrainVerts[(i * 4 + 1) * 3 + 1] -= 0.04;
    terrainVerts[(i * 4 + 2) * 3 + 1] -= 0.04;
  }
  const vcount = track.verts.length / 3;
  const terrainUv = new Float32Array(vcount * 2);
  for (let v = 0; v < vcount; v++) {
    terrainUv[v * 2] = terrainVerts[v * 3] * 0.08;
    terrainUv[v * 2 + 1] = terrainVerts[v * 3 + 2] * 0.08;
  }
  terrainGeo.setAttribute("position", new THREE.BufferAttribute(terrainVerts, 3));
  terrainGeo.setAttribute("uv", new THREE.BufferAttribute(terrainUv, 2));
  terrainGeo.setIndex(new THREE.BufferAttribute(track.tris.slice(), 1));
  terrainGeo.computeVertexNormals();
  {
    const apronEnd = vcount >= track.S * 12 + 4 ? track.S * 12 : track.S * 4;
    const colors = new Float32Array(vcount * 3);
    let s = (Number(fnv1a64(track.bytes) & 0xffffffffn) >>> 0) || 1;
    const rnd = () => (s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 2 ** 32;
    const base = new THREE.Color(0x314536);
    for (let v = 0; v < vcount; v++) {
      if (v < track.S * 4) {
        const m = (rnd() - 0.5) * 0.18;
        colors[v * 3] = base.r * (1 + m * 0.7);
        colors[v * 3 + 1] = base.g * (1 + m);
        colors[v * 3 + 2] = base.b * (1 + m * 0.5);
      } else if (v < apronEnd) {
        const m = (rnd() - 0.5) * 0.18;
        const shade = 1 - 0.042 * (((v - track.S * 4) % 4) + 1);
        colors[v * 3] = base.r * (1 + m * 0.7) * shade;
        colors[v * 3 + 1] = base.g * (1 + m) * shade;
        colors[v * 3 + 2] = base.b * (1 + m * 0.5) * shade;
      } else {
        colors[v * 3] = base.r * 0.85; colors[v * 3 + 1] = base.g * 0.88; colors[v * 3 + 2] = base.b * 0.85;
      }
    }
    terrainGeo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  }
  const turfTex = grassTexture();
  const grassMat = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    map: turfTex,
    vertexColors: true,
    roughness: 0.94,
    metalness: 0.0,
    envMapIntensity: 0.35,
  });
  group.add(new THREE.Mesh(terrainGeo, grassMat));
  group.userData.grassMat = grassMat;
  group.userData.grassTex = turfTex;

  // ---- Ribbon (drivable PBR asphalt matching physics parabolic crown)
  const n = track.S;
  const ROWS = [-1, -0.5, 0, 0.5, 1];
  const roadH = (f) => 0.005 - 0.025 * f * f;
  const R = ROWS.length;
  const pos = new Float32Array(n * R * 3);
  const uvs = new Float32Array(n * R * 2);
  let arc = 0;
  for (let i = 0; i < n; i++) {
    const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    const half = track.width[i] * 0.5;
    const uAlong = arc / 6;
    for (let r = 0; r < R; r++) {
      const f = ROWS[r];
      const p = c.clone().addScaledVector(side, f * half).addScaledVector(u, roadH(f));
      pos.set([p.x, p.y, p.z], (i * R + r) * 3);
      uvs.set([uAlong, (f + 1) / 2], (i * R + r) * 2);
    }
    arc += 2.5;
  }
  const idx = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    for (let r = 0; r < R - 1; r++) {
      const a = i * R + r, b = i * R + r + 1, c2 = j * R + r, d = j * R + r + 1;
      idx.push(a, b, c2, c2, b, d);
    }
  }
  const ribbonGeo = new THREE.BufferGeometry();
  ribbonGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  ribbonGeo.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
  ribbonGeo.setIndex(idx);
  ribbonGeo.computeVertexNormals();
  const pbrMaps = asphaltPBRMaps();
  const asphaltMat = new THREE.MeshStandardMaterial({
    map: pbrMaps.map,
    normalMap: pbrMaps.normalMap,
    normalScale: new THREE.Vector2(0.24, 0.24),
    roughnessMap: pbrMaps.roughnessMap,
    roughness: 0.86,
    metalness: 0.04,
    envMapIntensity: 0.55,
    side: THREE.DoubleSide,
  });
  group.add(new THREE.Mesh(ribbonGeo, asphaltMat));
  group.userData.asphaltMat = asphaltMat;

  // Two-row strip index reused by edge lines and overlays.
  const stripIdx = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    stripIdx.push(2 * i, 2 * i + 1, 2 * j, 2 * j, 2 * i + 1, 2 * j + 1);
  }

  // ---- Painted 18 cm 3D white edge ribbons (no 1px WebGL line aliasing,
  // crisp under headlights & golden-hour sun alike).
  const edgeMat = new THREE.MeshStandardMaterial({
    color: 0xeaf0f8,
    emissive: 0xcfdbe8,
    emissiveIntensity: 0.22,
    roughness: 0.52,
    metalness: 0.0,
    side: THREE.DoubleSide,
  });
  for (const sign of [-1, 1]) {
    const ep = new Float32Array(n * 2 * 3);
    for (let i = 0; i < n; i++) {
      const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const half = track.width[i] * 0.5;
      const latA = sign * (half - 0.22);
      const latB = sign * (half - 0.02);
      const pA = c.clone().addScaledVector(side, latA).addScaledVector(u, roadH(latA / half) + 0.011);
      const pB = c.clone().addScaledVector(side, latB).addScaledVector(u, roadH(latB / half) + 0.011);
      ep.set([pA.x, pA.y, pA.z], i * 6);
      ep.set([pB.x, pB.y, pB.z], i * 6 + 3);
    }
    const eg = new THREE.BufferGeometry();
    eg.setAttribute("position", new THREE.BufferAttribute(ep, 3));
    eg.setIndex(stripIdx);
    eg.computeVertexNormals();
    const em = new THREE.Mesh(eg, edgeMat);
    em.renderOrder = 3;
    group.add(em);
  }

  // ---- FIA painted 3D beveled kerbs (alternating heritage red/white)
  const KERB_WIDTH = 1.15;
  const kappa = smoothedCurvature(track);
  const kerbPos = [];
  const kerbCol = [];
  const red = [0.84, 0.16, 0.14], white = [0.93, 0.94, 0.95];
  for (const sign of [-1, 1]) {
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      if (Math.abs(kappa[i]) <= KERB_KAPPA || Math.abs(kappa[j]) <= KERB_KAPPA) continue;
      const col = i % 2 ? red : white;
      const quad = [];
      for (const s of [i, j]) {
        const c = v3(track.center[s]), u = v3(track.up[s]), t = v3(track.tangent[s]);
        const sd = new THREE.Vector3().crossVectors(u, t).normalize();
        const rise = (s % 2) * 0.016; // subtle alternating sawtooth bump visual
        const inner = c.clone().addScaledVector(sd, sign * (track.width[s] * 0.5)).addScaledVector(u, 0.025 + rise);
        const outer = c.clone().addScaledVector(sd, sign * (track.width[s] * 0.5 + KERB_WIDTH)).addScaledVector(u, 0.075 + rise);
        quad.push(inner, outer);
      }
      for (const vv of [quad[0], quad[1], quad[2], quad[1], quad[3], quad[2]]) {
        kerbPos.push(vv.x, vv.y, vv.z);
        kerbCol.push(...col);
      }
    }
  }
  if (kerbPos.length) {
    const kg = new THREE.BufferGeometry();
    kg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(kerbPos), 3));
    kg.setAttribute("color", new THREE.BufferAttribute(new Float32Array(kerbCol), 3));
    kg.computeVertexNormals();
    group.add(new THREE.Mesh(kg, new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.65,
      metalness: 0.02,
      emissive: 0x222224,
      emissiveIntensity: 0.35,
      side: THREE.DoubleSide,
    })));
  }

  // ---- Racing-line lateral offsets & braking-zone buildup
  const lineOff = new Float32Array(n);
  {
    const offs = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const t2 = v3(track.tangent[(i + 1) % n]);
      const inside = Math.sign(t2.sub(t).dot(side)) || 1;
      const pull = Math.min(1, Math.abs(kappa[i]) / 0.02);
      offs[i] = inside * pull * (track.width[i] * 0.5 - 2.0);
    }
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let k = -8; k <= 8; k++) sum += offs[(i + k + n) % n];
      lineOff[i] = sum / 17;
    }
  }

  const brake = new Float32Array(n);
  {
    const LOOK = 12;
    for (let i = 0; i < n; i++) {
      for (let d = 0; d <= LOOK; d++) {
        if (Math.abs(kappa[(i + d) % n]) > KERB_KAPPA) { brake[i] = 1 - d / (LOOK + 2); break; }
      }
    }
    const sm = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let k = -3; k <= 3; k++) sum += brake[(i + k + n) % n];
      sm[i] = sum / 7;
    }
    brake.set(sm);
  }

  // ---- Rubbered-in racing groove (darkened laid rubber + heavy braking zone)
  {
    const LINE_W = 2.4;
    const lp = new Float32Array(n * 2 * 3);
    const lc = new Float32Array(n * 2 * 4);
    for (let i = 0; i < n; i++) {
      const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const b = brake[i];
      const w = LINE_W * (1 + 0.55 * b);
      const halfW = track.width[i] * 0.5;
      const latA = lineOff[i] - w / 2, latD = lineOff[i] + w / 2;
      const hA = roadH(latA / halfW) + 0.008, hD = roadH(latD / halfW) + 0.008;
      const a = c.clone().addScaledVector(side, latA).addScaledVector(u, hA);
      const d = c.clone().addScaledVector(side, latD).addScaledVector(u, hD);
      lp.set([a.x, a.y, a.z], i * 6);
      lp.set([d.x, d.y, d.z], i * 6 + 3);
      const shade = 1 - 0.42 * b;
      const rgba = [0.06 * shade, 0.07 * shade, 0.09 * shade, 0.18 + 0.22 * b];
      lc.set(rgba, i * 8);
      lc.set(rgba, i * 8 + 4);
    }
    const lg = new THREE.BufferGeometry();
    lg.setAttribute("position", new THREE.BufferAttribute(lp, 3));
    lg.setAttribute("color", new THREE.BufferAttribute(lc, 4));
    lg.setIndex(stripIdx);
    const line = new THREE.Mesh(lg, new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide,
    }));
    line.renderOrder = 2;
    group.add(line);
  }

  // ---- Off-line marbles/dust band near track margins
  {
    const dp = new Float32Array(2 * n * 2 * 3);
    const dc = new Float32Array(2 * n * 2 * 4);
    const didx = [];
    [-1, 1].forEach((sign, sideNo) => {
      const vbase = sideNo * n * 2;
      for (let i = 0; i < n; i++) {
        const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
        const side = new THREE.Vector3().crossVectors(u, t).normalize();
        const half = track.width[i] * 0.5;
        const mid = sign * (half - 1.3);
        const gap = Math.abs(lineOff[i] - mid);
        const alpha = 0.075 * Math.min(1, Math.max(0, (gap - 1.4) / 1.6));
        [sign * (half - 2.1), sign * (half - 0.55)].forEach((lat, e) => {
          const p = c.clone().addScaledVector(side, lat).addScaledVector(u, roadH(lat / half) + 0.004);
          const v = vbase + i * 2 + e;
          dp.set([p.x, p.y, p.z], v * 3);
          dc.set([0.62, 0.65, 0.69, alpha], v * 4);
        });
      }
      for (let i = 0; i < n; i++) {
        const j = (i + 1) % n;
        didx.push(vbase + 2 * i, vbase + 2 * i + 1, vbase + 2 * j,
          vbase + 2 * j, vbase + 2 * i + 1, vbase + 2 * j + 1);
      }
    });
    const dg = new THREE.BufferGeometry();
    dg.setAttribute("position", new THREE.BufferAttribute(dp, 3));
    dg.setAttribute("color", new THREE.BufferAttribute(dc, 4));
    dg.setIndex(didx);
    const dust = new THREE.Mesh(dg, new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide,
    }));
    dust.renderOrder = 1;
    group.add(dust);
  }

  // ---- Checkered FIA Start/Finish line + staggered starting grid boxes + sector strips
  {
    const c0 = v3(track.center[0]), u0 = v3(track.up[0]).normalize(), t0 = v3(track.tangent[0]).normalize();
    const s0 = new THREE.Vector3().crossVectors(u0, t0).normalize();
    const basis0 = new THREE.Matrix4().makeBasis(s0, u0, t0);
    const q0 = new THREE.Quaternion().setFromRotationMatrix(basis0);
    const startGeo = new THREE.PlaneGeometry(track.width[0] - 0.4, 1.8);
    startGeo.rotateX(-Math.PI / 2);
    const startMesh = new THREE.Mesh(startGeo, new THREE.MeshStandardMaterial({
      map: checkerTexture(),
      emissive: 0x333840,
      emissiveIntensity: 0.25,
      roughness: 0.55,
      metalness: 0.0,
    }));
    startMesh.position.copy(c0).addScaledVector(u0, 0.015);
    startMesh.quaternion.copy(q0);
    startMesh.renderOrder = 4;
    group.add(startMesh);

    // Painted starting-grid brackets behind sample 0 (pole + P2..P4 visual)
    const boxMat = new THREE.MeshBasicMaterial({ color: 0xe6edf5, transparent: true, opacity: 0.72, side: THREE.DoubleSide });
    for (let slot = 0; slot < 4; slot++) {
      const idxBack = (n - 3 - slot * 3) % n;
      const cs = v3(track.center[idxBack]), us = v3(track.up[idxBack]).normalize(), ts = v3(track.tangent[idxBack]).normalize();
      const ss = new THREE.Vector3().crossVectors(us, ts).normalize();
      const qs = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(ss, us, ts));
      const lat = (slot % 2 === 0 ? -1 : 1) * Math.min(2.2, track.width[idxBack] * 0.22);
      const gridG = new THREE.Group();
      gridG.position.copy(cs).addScaledVector(ss, lat).addScaledVector(us, roadH(lat / (track.width[idxBack] * 0.5)) + 0.014);
      gridG.quaternion.copy(qs);
      const barF = new THREE.Mesh(new THREE.PlaneGeometry(2.4, 0.14), boxMat);
      barF.rotation.x = -Math.PI / 2;
      barF.position.z = 1.6;
      gridG.add(barF);
      for (const sx of [-1.13, 1.13]) {
        const barS = new THREE.Mesh(new THREE.PlaneGeometry(0.14, 1.4), boxMat);
        barS.rotation.x = -Math.PI / 2;
        barS.position.set(sx, 0, 0.9);
        gridG.add(barS);
      }
      group.add(gridG);
    }
  }

  for (const cp of track.checkpoints) {
    const c = v3(track.center[cp]), u = v3(track.up[cp]).normalize(), t = v3(track.tangent[cp]).normalize();
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    const g = new THREE.PlaneGeometry(track.width[cp] - 0.5, 0.55);
    g.rotateX(-Math.PI / 2);
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
      color: 0x22e0d0, transparent: true, opacity: 0.34, side: THREE.DoubleSide,
    }));
    m.position.copy(c).addScaledVector(u, 0.015);
    m.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(side, u, t));
    m.renderOrder = 4;
    group.add(m);
  }

  return group;
}

export class Minimap {
  constructor(canvas, track) {
    this.ctx = canvas.getContext("2d");
    this.w = canvas.width; this.h = canvas.height;
    this.track = track;
    this.trail = [];
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (const c of track.center) {
      minX = Math.min(minX, c[0]); maxX = Math.max(maxX, c[0]);
      minZ = Math.min(minZ, c[2]); maxZ = Math.max(maxZ, c[2]);
    }
    const pad = 14;
    const scale = Math.min((this.w - 2 * pad) / (maxX - minX), (this.h - 2 * pad) / (maxZ - minZ));
    this.map = (p) => [pad + (p[0] - minX) * scale, pad + (p[2] - minZ) * scale];
  }

  draw(carPos, checkpointMask, ghostPos = null) {
    const { ctx, track, trail } = this;
    ctx.clearRect(0, 0, this.w, this.h);
    ctx.lineCap = ctx.lineJoin = "round";
    ctx.beginPath();
    track.center.forEach((c, i) => {
      const [x, y] = this.map(c);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.strokeStyle = "rgba(5,9,15,.6)"; ctx.lineWidth = 5; ctx.stroke();
    ctx.strokeStyle = "#b9c4d4"; ctx.lineWidth = 1.6; ctx.stroke();

    const dot = (i, color, r = 2.6) => {
      const [x, y] = this.map(track.center[i]);
      ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill();
    };
    dot(0, "#ffffff", 3.0);
    track.checkpoints.forEach((cp, k) => dot(cp, checkpointMask & (1 << k) ? "#7fe3a1" : "rgba(34,224,208,.7)"));

    // Motion trail (~2 s)
    const [cx, cy] = this.map(carPos);
    const now = performance.now();
    const tail = trail[trail.length - 1];
    if (tail && Math.hypot(cx - tail[0], cy - tail[1]) > 25) trail.length = 0;
    if (!tail || now - tail[2] > 50) trail.push([cx, cy, now]);
    while (trail.length && now - trail[0][2] > 2000) trail.shift();
    ctx.lineWidth = 1.4;
    for (let i = 1; i < trail.length; i++) {
      ctx.strokeStyle = `rgba(34,224,208,${(0.55 * (1 - (now - trail[i][2]) / 2000)).toFixed(3)})`;
      ctx.beginPath();
      ctx.moveTo(trail[i - 1][0], trail[i - 1][1]);
      ctx.lineTo(trail[i][0], trail[i][1]);
      ctx.stroke();
    }

    // Ghost dot (translucent cyan/ice blue) when racing a ghost
    if (ghostPos) {
      const [gx, gy] = this.map(ghostPos);
      ctx.fillStyle = "rgba(191,217,255,0.85)";
      ctx.strokeStyle = "rgba(0,0,0,0.5)";
      ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.arc(gx, gy, 3.0, 0, 7); ctx.fill(); ctx.stroke();
    }

    // Player car dot
    ctx.fillStyle = "#ffffff"; ctx.strokeStyle = "rgba(0,0,0,.65)"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(cx, cy, 3.8, 0, 7); ctx.fill(); ctx.stroke();
  }
}

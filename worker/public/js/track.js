// TRK1 parser (CONTRACTS §8) + three.js geometry + minimap rendering.
import * as THREE from "../vendor/three.module.js";

export function parseTrack(buf) {
  const bytes = new Uint8Array(buf);
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== 0x4b525453) throw new Error("bad TRK1 magic");
  if (dv.getUint16(4, true) !== 1) throw new Error("bad TRK1 version");
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

  return { bytes, center, up, tangent, width, checkpoints, verts, tris, spawnIndex, S };
}

export function fnv1a64(bytes) {
  let h = 0xcbf29ce484222325n;
  for (let i = 0; i < bytes.length; i++) h = ((h ^ BigInt(bytes[i])) * 0x100000001b3n) & 0xffffffffffffffffn;
  return h;
}

const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

export function buildTrackMeshes(track) {
  const group = new THREE.Group();

  // Terrain (shoulders + surface base) — dark gray, flat shaded.
  const terrainGeo = new THREE.BufferGeometry();
  terrainGeo.setAttribute("position", new THREE.BufferAttribute(track.verts.slice(), 3));
  terrainGeo.setIndex(new THREE.BufferAttribute(track.tris.slice(), 1));
  terrainGeo.computeVertexNormals();
  group.add(new THREE.Mesh(terrainGeo, new THREE.MeshLambertMaterial({ color: 0x2d3b31 })));

  // Ribbon (drivable surface) lifted a hair above the terrain to avoid z-fight.
  const n = track.S;
  const pos = new Float32Array(n * 2 * 3);
  for (let i = 0; i < n; i++) {
    const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    const half = track.width[i] * 0.5;
    const l = c.clone().addScaledVector(side, -half).addScaledVector(u, 0.05);
    const r = c.clone().addScaledVector(side, half).addScaledVector(u, 0.05);
    pos.set([l.x, l.y, l.z], i * 6);
    pos.set([r.x, r.y, r.z], i * 6 + 3);
  }
  const idx = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    idx.push(2 * i, 2 * i + 1, 2 * j, 2 * j, 2 * i + 1, 2 * j + 1);
  }
  const ribbonGeo = new THREE.BufferGeometry();
  ribbonGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  ribbonGeo.setIndex(idx);
  ribbonGeo.computeVertexNormals();
  group.add(new THREE.Mesh(ribbonGeo, new THREE.MeshLambertMaterial({ color: 0x5a626e })));

  // White edge lines — the single biggest readability win at speed.
  for (const offset of [0, 3]) {
    const edge = new Float32Array((n + 1) * 3);
    for (let i = 0; i <= n; i++) {
      const src = (i % n) * 6 + offset;
      edge[i * 3] = pos[src];
      edge[i * 3 + 1] = pos[src + 1] + 0.02;
      edge[i * 3 + 2] = pos[src + 2];
    }
    const lineGeo = new THREE.BufferGeometry();
    lineGeo.setAttribute("position", new THREE.BufferAttribute(edge, 3));
    group.add(new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xdfe6f0 })));
  }

  // Kerb visuals: red/white striped strips through tight corners. Mirrors the
  // generator's rule (trackgen/generate_track.py: KERB_KAPPA, smoothing window)
  // — the physical rumble geometry is in the terrain mesh; this is the paint.
  const KERB_KAPPA = 0.022;
  const KERB_SMOOTH = 15;
  const KERB_WIDTH = 1.1;
  const raw = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const t = track.tangent[i];
    const t2 = track.tangent[(i + 1) % n];
    raw[i] = (t[0] * t2[2] - t[2] * t2[0]) / 2.5;
  }
  const kappa = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    let sum = 0;
    for (let k = -((KERB_SMOOTH / 2) | 0); k <= (KERB_SMOOTH / 2) | 0; k++) sum += raw[(i + k + n) % n];
    kappa[i] = sum / KERB_SMOOTH;
  }
  const kerbPos = [];
  const kerbCol = [];
  const red = [0.85, 0.2, 0.16], white = [0.92, 0.92, 0.9];
  for (const sign of [-1, 1]) {
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      if (Math.abs(kappa[i]) <= KERB_KAPPA || Math.abs(kappa[j]) <= KERB_KAPPA) continue;
      const col = i % 2 ? red : white;
      const quad = [];
      for (const s of [i, j]) {
        const c = v3(track.center[s]), u = v3(track.up[s]), t = v3(track.tangent[s]);
        const sd = new THREE.Vector3().crossVectors(u, t).normalize();
        const inner = c.clone().addScaledVector(sd, sign * (track.width[s] * 0.5)).addScaledVector(u, 0.08);
        const outer = c.clone().addScaledVector(sd, sign * (track.width[s] * 0.5 + KERB_WIDTH)).addScaledVector(u, 0.08);
        quad.push(inner, outer);
      }
      // two tris: inner_i, outer_i, inner_j / outer_i, outer_j, inner_j
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
    group.add(new THREE.Mesh(kg, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide })));
  }

  // Start line + checkpoint gates: thin bright quads across the track.
  const gate = (i, color, opacity) => {
    const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    const half = track.width[i] * 0.5;
    const g = new THREE.PlaneGeometry(half * 2, 1.2);
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide }));
    m.position.copy(c).addScaledVector(u, 0.08);
    m.lookAt(c.clone().add(t));
    m.rotateX(Math.PI / 2);
    return m;
  };
  group.add(gate(0, 0xffffff, 0.9));
  for (const cp of track.checkpoints) group.add(gate(cp, 0x8fd3ff, 0.35));

  return group;
}

export class Minimap {
  constructor(canvas, track) {
    this.ctx = canvas.getContext("2d");
    this.w = canvas.width; this.h = canvas.height;
    this.track = track;
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (const c of track.center) {
      minX = Math.min(minX, c[0]); maxX = Math.max(maxX, c[0]);
      minZ = Math.min(minZ, c[2]); maxZ = Math.max(maxZ, c[2]);
    }
    const pad = 14;
    const scale = Math.min((this.w - 2 * pad) / (maxX - minX), (this.h - 2 * pad) / (maxZ - minZ));
    this.map = (p) => [pad + (p[0] - minX) * scale, pad + (p[2] - minZ) * scale];
  }

  draw(carPos, checkpointMask) {
    const { ctx, track } = this;
    ctx.clearRect(0, 0, this.w, this.h);
    ctx.strokeStyle = "#4c566a"; ctx.lineWidth = 4; ctx.lineCap = "round";
    ctx.beginPath();
    track.center.forEach((c, i) => {
      const [x, y] = this.map(c);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.closePath(); ctx.stroke();
    // start + checkpoints
    const dot = (i, color) => {
      const [x, y] = this.map(track.center[i]);
      ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill();
    };
    dot(0, "#ffffff");
    track.checkpoints.forEach((cp, k) => dot(cp, checkpointMask & (1 << k) ? "#7fe3a1" : "#8fd3ff"));
    // car
    const [cx, cy] = this.map(carPos);
    ctx.fillStyle = "#ffd479"; ctx.beginPath(); ctx.arc(cx, cy, 5, 0, 7); ctx.fill();
  }
}

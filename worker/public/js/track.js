// TRK1 parser (CONTRACTS §8) + three.js geometry + minimap rendering.
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
  o += 12; // spawn_index u32, spawn_yaw f32, reserved f32 — end of the v1 payload

  // TRK1 v2 (CONTRACTS §8): a prop table appended after the v1 tail.
  // prop_count u32 (0..4096), then prop_count 36-byte records:
  //   type u16 (0 tire_stack, 1 armco_segment, 2 wall_generic), _pad u16,
  //   pos f32[3] (world-space box center), yaw f32 (about +Y, radians),
  //   half f32[3] (box half-extents, m); 4 reserved bytes pad each to 36.
  // Each prop is one static collision box in the kernel; here we only carry
  // the data so ambience.js can draw visuals that match the collision.
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

// Curvature constants shared by the kerb paint (below) and trackside ambience
// (js/ambience.js). Mirrors the generator's rule (trackgen/generate_track.py:
// KERB_KAPPA, smoothing window).
export const KERB_KAPPA = 0.022;
export const KERB_SMOOTH = 15;

/** Smoothed signed curvature per centerline sample (sign = turn direction).
 *  Same math the kerb painter uses; exported so ambience placement (tire
 *  barriers, braking boards) agrees with where the kerbs are. */
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

/** Procedural asphalt: mid-gray noise + sparse aggregate specks and darker
 *  patches. Tiled along the ribbon's arc length (one repeat / 6 m). */
function asphaltTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 256;
  const g = c.getContext("2d");
  const img = g.createImageData(256, 256);
  let s = 0x1234abcd;
  const rnd = () => ((s = (s * 1664525 + 1013904223) >>> 0), s / 0xffffffff);
  for (let i = 0; i < img.data.length; i += 4) {
    const base = 128 + (rnd() - 0.5) * 24; // fine grain
    img.data[i] = base;
    img.data[i + 1] = base + 4;
    img.data[i + 2] = base + 10;
    img.data[i + 3] = 255;
  }
  g.putImageData(img, 0, 0);
  for (let k = 0; k < 260; k++) { // lighter aggregate specks
    g.fillStyle = `rgba(190,196,205,${0.10 + rnd() * 0.15})`;
    g.fillRect((rnd() * 256) | 0, (rnd() * 256) | 0, 1 + ((rnd() * 2) | 0), 1);
  }
  for (let k = 0; k < 7; k++) { // occasional repair patches
    g.fillStyle = `rgba(30,34,42,${0.12 + rnd() * 0.10})`;
    g.beginPath();
    g.ellipse(rnd() * 256, rnd() * 256, 12 + rnd() * 30, 6 + rnd() * 14, rnd() * 3, 0, 7);
    g.fill();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

export function buildTrackMeshes(track) {
  const group = new THREE.Group();

  // Terrain (shoulders + surface base) — dark green with subtle per-vertex
  // grass mottling (APEX §3 run-off variation), deterministic from the track
  // hash. Apron tracks (TERRAIN.md §1) append 8 apron verts per sample right
  // after the 4-row ribbon — v(i,side,row) = 4S + i*8 + side*4 + row, rows
  // 0..3 outward — mottled like the shoulders but stepped slightly darker
  // toward the plain. Kerb rumble verts (appended after ribbon + apron) keep
  // the flat base tone; their visible paint is the red/white overlay anyway.
  const terrainGeo = new THREE.BufferGeometry();
  terrainGeo.setAttribute("position", new THREE.BufferAttribute(track.verts.slice(), 3));
  terrainGeo.setIndex(new THREE.BufferAttribute(track.tris.slice(), 1));
  terrainGeo.computeVertexNormals();
  {
    const vcount = track.verts.length / 3;
    // Apron detection is structural: ribbon(4S) + apron(8S) + ground quad(4)
    // is only reachable with the apron block (kerbs alone cap at +4S).
    const apronEnd = vcount >= track.S * 12 + 4 ? track.S * 12 : track.S * 4;
    const colors = new Float32Array(vcount * 3);
    let s = (Number(fnv1a64(track.bytes) & 0xffffffffn) >>> 0) || 1;
    const rnd = () => (s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 2 ** 32;
    const base = new THREE.Color(0x2d3b31);
    for (let v = 0; v < vcount; v++) {
      if (v < track.S * 4) {
        const m = (rnd() - 0.5) * 0.16;
        colors[v * 3] = base.r * (1 + m * 0.7);
        colors[v * 3 + 1] = base.g * (1 + m);
        colors[v * 3 + 2] = base.b * (1 + m * 0.5);
      } else if (v < apronEnd) {
        const m = (rnd() - 0.5) * 0.16;
        const shade = 1 - 0.045 * (((v - track.S * 4) % 4) + 1); // darker toward the plain
        colors[v * 3] = base.r * (1 + m * 0.7) * shade;
        colors[v * 3 + 1] = base.g * (1 + m) * shade;
        colors[v * 3 + 2] = base.b * (1 + m * 0.5) * shade;
      } else {
        colors[v * 3] = base.r; colors[v * 3 + 1] = base.g; colors[v * 3 + 2] = base.b;
      }
    }
    terrainGeo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  }
  group.add(new THREE.Mesh(terrainGeo, new THREE.MeshLambertMaterial({ color: 0xffffff, vertexColors: true })));

  // Ribbon (drivable surface) lifted a hair above the terrain to avoid z-fight.
  // Textured: procedural asphalt (noise + aggregate) tiled along arc length.
  const n = track.S;
  const pos = new Float32Array(n * 2 * 3);
  const uvs = new Float32Array(n * 2 * 2);
  let arc = 0;
  for (let i = 0; i < n; i++) {
    const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    const half = track.width[i] * 0.5;
    const l = c.clone().addScaledVector(side, -half).addScaledVector(u, 0.05);
    const r = c.clone().addScaledVector(side, half).addScaledVector(u, 0.05);
    pos.set([l.x, l.y, l.z], i * 6);
    pos.set([r.x, r.y, r.z], i * 6 + 3);
    const uAlong = arc / 6; // one texture repeat per 6 m of road
    uvs.set([uAlong, 0], i * 4);
    uvs.set([uAlong, 1], i * 4 + 2);
    arc += 2.5;
  }
  const idx = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    idx.push(2 * i, 2 * i + 1, 2 * j, 2 * j, 2 * i + 1, 2 * j + 1);
  }
  const ribbonGeo = new THREE.BufferGeometry();
  ribbonGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  ribbonGeo.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
  ribbonGeo.setIndex(idx);
  ribbonGeo.computeVertexNormals();
  group.add(new THREE.Mesh(ribbonGeo, new THREE.MeshLambertMaterial({ map: asphaltTexture(), side: THREE.DoubleSide })));

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

  // Kerb visuals: red/white striped strips through tight corners — the
  // physical rumble geometry is in the terrain mesh; this is the paint.
  // (Curvature math + constants live in smoothedCurvature above, shared with
  // js/ambience.js so barriers/boards land on the same corners.)
  const KERB_WIDTH = 1.1;
  const kappa = smoothedCurvature(track);
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

  // Racing-line lateral offsets: pull toward each corner's inside (apex)
  // proportionally to curvature, smoothed so the line sweeps, not zigzags.
  // Shared by the rubber band and the off-line dust tint below.
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

  // Braking factor per sample: 1 just before a tight (|κ| > KERB_KAPPA)
  // corner, easing off over ~30 m (12 samples) — cars travel toward
  // increasing sample index. Drives the rubber band's width/darkness.
  const brake = new Float32Array(n);
  {
    const LOOK = 12;
    for (let i = 0; i < n; i++) {
      for (let d = 0; d <= LOOK; d++) {
        if (Math.abs(kappa[(i + d) % n]) > KERB_KAPPA) { brake[i] = 1 - d / (LOOK + 2); break; }
      }
    }
    const sm = new Float32Array(n); // soften so the band swells, not steps
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let k = -3; k <= 3; k++) sum += brake[(i + k + n) % n];
      sm[i] = sum / 7;
    }
    brake.set(sm);
  }

  // Rubbered-in racing line: a translucent dark band, the worn groove every
  // used circuit carries. Grip story (APEX §2): into braking zones before
  // tight corners the band widens, darkens and gains opacity (heavy rubber +
  // lock-up dust); per-vertex RGBA (itemSize 4 → vertex alpha). Visual only.
  {
    const LINE_W = 2.3;
    const lp = new Float32Array(n * 2 * 3);
    const lc = new Float32Array(n * 2 * 4);
    for (let i = 0; i < n; i++) {
      const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const b = brake[i];
      const w = LINE_W * (1 + 0.5 * b); // up to ~3.45 m wide under braking
      const a = c.clone().addScaledVector(side, lineOff[i] - w / 2).addScaledVector(u, 0.065);
      const d = c.clone().addScaledVector(side, lineOff[i] + w / 2).addScaledVector(u, 0.065);
      lp.set([a.x, a.y, a.z], i * 6);
      lp.set([d.x, d.y, d.z], i * 6 + 3);
      const shade = 1 - 0.4 * b; // darker rubber where braking is heavy
      const rgba = [0.078 * shade, 0.094 * shade, 0.122 * shade, 0.16 + 0.17 * b];
      lc.set(rgba, i * 8);
      lc.set(rgba, i * 8 + 4);
    }
    const lg = new THREE.BufferGeometry();
    lg.setAttribute("position", new THREE.BufferAttribute(lp, 3));
    lg.setAttribute("color", new THREE.BufferAttribute(lc, 4));
    lg.setIndex(idx);
    const line = new THREE.Mesh(lg, new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide }));
    line.renderOrder = 2; // over the dust tint
    group.add(line);
  }

  // Off-line dust: a very faint lighter tint near each edge — the track is
  // cleanest on the groove, dusty where nobody runs. Fades out wherever the
  // racing line sweeps close to that edge (per-vertex alpha).
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
        const alpha = 0.07 * Math.min(1, Math.max(0, (gap - 1.4) / 1.6));
        [sign * (half - 2.1), sign * (half - 0.55)].forEach((lat, e) => {
          const p = c.clone().addScaledVector(side, lat).addScaledVector(u, 0.058);
          const v = vbase + i * 2 + e;
          dp.set([p.x, p.y, p.z], v * 3);
          dc.set([0.66, 0.68, 0.71, alpha], v * 4);
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
      vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide }));
    dust.renderOrder = 1; // under the racing line
    group.add(dust);
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
    this.trail = []; // recent mapped car positions: [x, y, tMs], ~2 s window
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
    const { ctx, track, trail } = this;
    ctx.clearRect(0, 0, this.w, this.h);
    // Track ribbon floats directly over the scene (no panel), so it carries
    // its own contrast: dark halo under-stroke + thin light line on top —
    // reads against bright day sky and near-black night alike.
    ctx.lineCap = ctx.lineJoin = "round";
    ctx.beginPath();
    track.center.forEach((c, i) => {
      const [x, y] = this.map(c);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.strokeStyle = "rgba(5,9,15,.55)"; ctx.lineWidth = 4.5; ctx.stroke();
    ctx.strokeStyle = "#b9c4d4"; ctx.lineWidth = 1.5; ctx.stroke();
    // start + checkpoints (cleared ones turn green)
    const dot = (i, color, r = 2.6) => {
      const [x, y] = this.map(track.center[i]);
      ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill();
    };
    dot(0, "#ffffff", 2.8);
    track.checkpoints.forEach((cp, k) => dot(cp, checkpointMask & (1 << k) ? "#7fe3a1" : "rgba(143,211,255,.65)"));
    // motion trail: last ~2 s of positions, fading with age
    const [cx, cy] = this.map(carPos);
    const now = performance.now();
    const tail = trail[trail.length - 1];
    if (tail && Math.hypot(cx - tail[0], cy - tail[1]) > 25) trail.length = 0; // respawn teleport — drop stale streak
    if (!tail || now - tail[2] > 50) trail.push([cx, cy, now]);
    while (trail.length && now - trail[0][2] > 2000) trail.shift();
    ctx.lineWidth = 1.2;
    for (let i = 1; i < trail.length; i++) {
      ctx.strokeStyle = `rgba(255,255,255,${(0.45 * (1 - (now - trail[i][2]) / 2000)).toFixed(3)})`;
      ctx.beginPath();
      ctx.moveTo(trail[i - 1][0], trail[i - 1][1]);
      ctx.lineTo(trail[i][0], trail[i][1]);
      ctx.stroke();
    }
    // car
    ctx.fillStyle = "#ffffff"; ctx.strokeStyle = "rgba(0,0,0,.6)"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(cx, cy, 3.5, 0, 7); ctx.fill(); ctx.stroke();
  }
}

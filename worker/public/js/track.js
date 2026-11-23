// TRK1 parser (CONTRACTS §8) + 2026 AAA PBR three.js track surface & geometry + F1 broadcast minimap.
// Features:
//   • 1024×1024 multi-scale procedural PBR asphalt suite (albedo + Sobel tangent normal + roughness)
//     with coarse basalt/granite aggregate, longitudinal paver tracks & sealed tar-crack veins;
//   • 512×512 seamless 4-channel world-space puddle basin & macro-weathering noise texture;
//   • Custom MeshPhysicalMaterial onBeforeCompile shader ('ApexAsphalt2026'): world-space macro
//     breakup (zero straightaway tiling), vertex-attribute rubbered racing line & braking streaks,
//     dynamic standing water puddles (uWetness, uTime) with darkened wet porous albedo, mirror
//     clearcoat reflections & animated raindrop/wind micro-ripples;
//   • 3D sculpted 5-profile sawtooth chamfered FIA kerbs with 512×256 concrete/scuff PBR maps;
//   • Weathered FIA track-limit lines, straightaway verge accents, checkered start/finish strip,
//     staggered F1 starting grid boxes, and multi-tone mowed alpine turf + grass normal map;
//   • Crisp F1 broadcast-style Minimap with racing stroke, checkered start bar, glow beacon & ghost.
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

// ---------------------------------------------------------------------------
// Fast deterministic tileable noise & Sobel normal helpers
// ---------------------------------------------------------------------------

function hash2Periodic(ix, iy, period, seed) {
  const x = ((ix % period) + period) % period;
  const y = ((iy % period) + period) % period;
  let h = (Math.imul(x, 374761393) + Math.imul(y, 668265263) + Math.imul(seed, 1442695041)) | 0;
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

function tileValueNoise(u, v, period, seed) {
  const fx = u * period;
  const fy = v * period;
  const ix = Math.floor(fx);
  const iy = Math.floor(fy);
  const tx = fx - ix;
  const ty = fy - iy;
  const sx = tx * tx * (3 - 2 * tx);
  const sy = ty * ty * (3 - 2 * ty);
  const n00 = hash2Periodic(ix, iy, period, seed);
  const n10 = hash2Periodic(ix + 1, iy, period, seed);
  const n01 = hash2Periodic(ix, iy + 1, period, seed);
  const n11 = hash2Periodic(ix + 1, iy + 1, period, seed);
  const nx0 = n00 + (n10 - n00) * sx;
  const nx1 = n01 + (n11 - n01) * sx;
  return nx0 + (nx1 - nx0) * sy;
}

function tileFbm(u, v, basePeriod, octaves, seed) {
  let sum = 0, amp = 0.5, norm = 0, p = basePeriod;
  for (let o = 0; o < octaves; o++) {
    sum += amp * tileValueNoise(u, v, p, seed + o * 97);
    norm += amp;
    amp *= 0.5;
    p <<= 1;
  }
  return sum / norm;
}

/** Write OpenGL/Three.js tangent-space Sobel normal map from tileable heightfield. */
function buildSobelNormalCanvas(height, W, H, strength) {
  const c = document.createElement("canvas");
  c.width = W;
  c.height = H;
  const g = c.getContext("2d");
  const img = g.createImageData(W, H);
  const d = img.data;
  const maskX = W - 1;
  const maskY = H - 1;
  for (let y = 0; y < H; y++) {
    const ym = ((y - 1) & maskY) * W;
    const y0 = y * W;
    const yp = ((y + 1) & maskY) * W;
    for (let x = 0; x < W; x++) {
      const xm = (x - 1) & maskX;
      const xp = (x + 1) & maskX;
      const tl = height[ym + xm], tc = height[ym + x], tr = height[ym + xp];
      const ml = height[y0 + xm],                      mr = height[y0 + xp];
      const bl = height[yp + xm], bc = height[yp + x], br = height[yp + xp];
      const gx = ((tr + 2 * mr + br) - (tl + 2 * ml + bl)) * strength;
      const gy = ((bl + 2 * bc + br) - (tl + 2 * tc + tr)) * strength;
      // OpenGL/Three.js tangent-space (flipY=true): +X -> -gx, +V (-Y canvas) -> +gy
      const nx = -gx;
      const ny = gy;
      const invLen = 1 / Math.hypot(nx, ny, 1);
      const o = (y0 + x) * 4;
      d[o]     = Math.max(0, Math.min(255, Math.round((nx * invLen * 0.5 + 0.5) * 255)));
      d[o + 1] = Math.max(0, Math.min(255, Math.round((ny * invLen * 0.5 + 0.5) * 255)));
      d[o + 2] = Math.max(0, Math.min(255, Math.round((invLen * 0.5 + 0.5) * 255)));
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return c;
}

// ---------------------------------------------------------------------------
// 1024×1024 Multi-Scale Procedural PBR Asphalt Suite + 512×512 Puddle Map
// ---------------------------------------------------------------------------

/** Seamless 512×512 4-channel world-space noise map:
 *  R = smooth puddle basins, G = macro resurfacing/weathering breakup,
 *  B = stretched streak/ripple modulation, A = fine aggregate porosity. */
function buildPuddleNoiseTexture() {
  const N = 512;
  const data = new Uint8Array(N * N * 4);
  const invN = 1 / N;
  for (let y = 0; y < N; y++) {
    const v = y * invN;
    for (let x = 0; x < N; x++) {
      const u = x * invN;
      // R: multi-octave smooth organic puddle basins (warped slightly for natural pools)
      const wx = tileValueNoise(u, v, 4, 17) * 0.08;
      const wy = tileValueNoise(u, v, 4, 53) * 0.08;
      const basin = tileFbm(u + wx, v + wy, 4, 4, 101);
      // G: medium-low frequency asphalt resurfacing / macro tonal breakup
      const macro = tileFbm(u, v, 4, 4, 307);
      // B: medium-high frequency streak/wind variation
      const streak = tileFbm(u, v, 8, 3, 619);
      // A: fine porosity & damp-rim granulation
      const pores = tileFbm(u, v, 16, 3, 911);
      const o = (y * N + x) * 4;
      data[o]     = Math.max(0, Math.min(255, (basin * 255) | 0));
      data[o + 1] = Math.max(0, Math.min(255, (macro * 255) | 0));
      data[o + 2] = Math.max(0, Math.min(255, (streak * 255) | 0));
      data[o + 3] = Math.max(0, Math.min(255, (pores * 255) | 0));
    }
  }
  const tex = new THREE.DataTexture(data, N, N, THREE.RGBAFormat);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.generateMipmaps = true;
  tex.needsUpdate = true;
  return tex;
}

/** 1024×1024 Multi-Scale Procedural PBR Asphalt:
 *  - Coarse basalt & granite crushed stone aggregate + fine sand/bitumen grit
 *  - Longitudinal paver roller passes & compacted wheel-lane tonal depth
 *  - Organic sealed tar-crack veins (seamless across tile borders, lower roughness)
 *  - True 3×3 Sobel tangent-space normal map + physically calibrated roughness map. */
function asphaltPBRMaps() {
  const N = 1024;
  const mask = N - 1;
  const height = new Float32Array(N * N);
  const chipTint = new Float32Array(N * N);
  const tarMask = new Float32Array(N * N);

  let s = 0x2026a5f7;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  // Layer 1: Fine micro-grit sand/bitumen bed
  for (let i = 0; i < N * N; i++) {
    const r1 = rnd();
    height[i] = 0.28 + r1 * 0.24;
    chipTint[i] = (rnd() - 0.5) * 0.14;
  }

  // Layer 2: Multi-scale crushed basalt & granite aggregate stones (2px to 6px)
  const stoneCount = 92000;
  for (let k = 0; k < stoneCount; k++) {
    const cx = (rnd() * N) | 0;
    const cy = (rnd() * N) | 0;
    const rx = 1 + ((rnd() * 3.2) | 0);
    const ry = 1 + ((rnd() * 2.6) | 0);
    // Classify aggregate rock: dark basalt (52%), mid slate (36%), pale quartz/granite (12%)
    const roll = rnd();
    let hBoost, tint;
    if (roll < 0.52) {
      hBoost = 0.16 + rnd() * 0.20;
      tint = -0.22 - rnd() * 0.14;
    } else if (roll < 0.88) {
      hBoost = 0.20 + rnd() * 0.24;
      tint = 0.04 + rnd() * 0.14;
    } else {
      hBoost = 0.24 + rnd() * 0.22;
      tint = 0.26 + rnd() * 0.24;
    }
    for (let dy = -ry; dy <= ry; dy++) {
      const py = ((cy + dy) & mask) * N;
      const ny = dy / (ry + 0.35);
      for (let dx = -rx; dx <= rx; dx++) {
        const nx = dx / (rx + 0.35);
        const d2 = nx * nx + ny * ny;
        if (d2 > 1.0) continue;
        const fall = 1.0 - d2 * 0.72;
        const idx = py + ((cx + dx) & mask);
        const cand = 0.36 + hBoost * fall;
        if (cand > height[idx]) {
          height[idx] = cand;
          chipTint[idx] = tint * (0.7 + 0.3 * fall);
        }
      }
    }
  }

  // Layer 3: Subtle low-frequency paver roller ripples & macro stone compaction
  // Precompute small 64×64 periodic grid and bilinearly add (blazing fast at 1024²)
  const G = 64;
  const gMask = G - 1;
  const macroGrid = new Float32Array(G * G);
  for (let i = 0; i < G * G; i++) macroGrid[i] = rnd();
  const invStep = G / N;
  for (let y = 0; y < N; y++) {
    const gy = y * invStep;
    const iy = gy | 0;
    const ty = gy - iy;
    const sy = ty * ty * (3 - 2 * ty);
    const iy0 = (iy & gMask) * G;
    const iy1 = ((iy + 1) & gMask) * G;
    // Longitudinal paver roller micro-tracks across road width (V = y/N)
    const vNorm = y / N;
    const rollerWave = Math.sin(vNorm * Math.PI * 12) * 0.025 + Math.sin(vNorm * Math.PI * 28 + 1.1) * 0.015;
    const rowOff = y * N;
    for (let x = 0; x < N; x++) {
      const gx = x * invStep;
      const ix = gx | 0;
      const tx = gx - ix;
      const sx = tx * tx * (3 - 2 * tx);
      const ix0 = ix & gMask;
      const ix1 = (ix + 1) & gMask;
      const m0 = macroGrid[iy0 + ix0] + (macroGrid[iy0 + ix1] - macroGrid[iy0 + ix0]) * sx;
      const m1 = macroGrid[iy1 + ix0] + (macroGrid[iy1 + ix1] - macroGrid[iy1 + ix0]) * sx;
      const mVal = m0 + (m1 - m0) * sy;
      height[rowOff + x] += (mVal - 0.5) * 0.14 + rollerWave;
    }
  }

  // Layer 4: Sealed bitumen tar-crack veins (periodic wrapped random walks)
  const crackCount = 16;
  for (let c = 0; c < crackCount; c++) {
    let px = rnd() * N;
    let py = rnd() * N;
    let ang = rnd() * Math.PI * 2;
    const steps = 110 + ((rnd() * 160) | 0);
    const widthR = 1 + (rnd() > 0.65 ? 1 : 0);
    for (let st = 0; st < steps; st++) {
      ang += (rnd() - 0.5) * 0.42;
      px += Math.cos(ang) * 1.45;
      py += Math.sin(ang) * 1.45;
      const ix = ((px | 0) % N + N) & mask;
      const iy = ((py | 0) % N + N) & mask;
      for (let dy = -widthR; dy <= widthR; dy++) {
        const wy = ((iy + dy) & mask) * N;
        for (let dx = -widthR; dx <= widthR; dx++) {
          const dist = Math.hypot(dx, dy);
          if (dist > widthR + 0.5) continue;
          const w = 1 - dist / (widthR + 0.8);
          const idx = wy + ((ix + dx) & mask);
          if (w > tarMask[idx]) tarMask[idx] = w;
          height[idx] -= w * 0.16; // recessed crack channel with raised sealant rim
        }
      }
    }
  }
  // Subtle longitudinal center paving joint seam at y = N/2
  for (let x = 0; x < N; x++) {
    const wobble = Math.round(Math.sin((x / N) * Math.PI * 6) * 1.2);
    for (let dy = -2; dy <= 2; dy++) {
      const yy = ((N >> 1) + dy + wobble) & mask;
      const w = (1 - Math.abs(dy) / 2.6) * 0.65;
      const idx = yy * N + x;
      if (w > tarMask[idx]) tarMask[idx] = w;
      height[idx] -= w * 0.12;
    }
  }

  // Compose Albedo & Roughness canvases
  const cMap = document.createElement("canvas");
  cMap.width = cMap.height = N;
  const gMap = cMap.getContext("2d");
  const imgMap = gMap.createImageData(N, N);
  const dMap = imgMap.data;

  const cRough = document.createElement("canvas");
  cRough.width = cRough.height = N;
  const gRough = cRough.getContext("2d");
  const imgRough = gRough.createImageData(N, N);
  const dRough = imgRough.data;

  for (let y = 0; y < N; y++) {
    const vNorm = y / N;
    // Dual longitudinal wheel-track compaction bands + subtle paver roller sheen
    const laneWear =
      Math.exp(-Math.pow((vNorm - 0.31) / 0.105, 2)) +
      Math.exp(-Math.pow((vNorm - 0.69) / 0.105, 2));
    const edgeLighten = Math.pow(Math.min(1, Math.abs(vNorm - 0.5) * 2.05), 3.0) * 11;
    const rowOff = y * N;
    for (let x = 0; x < N; x++) {
      const idx = rowOff + x;
      const h = height[idx];
      const tint = chipTint[idx];
      const tar = tarMask[idx];

      // Deep realistic charcoal race-grade basalt tarmac (never flat gray)
      let baseLum = 56 + (h - 0.32) * 52 + tint * 34 - laneWear * 9 + edgeLighten;
      // Sealed tar veins are rich dark glossy bitumen
      baseLum = baseLum * (1 - tar * 0.58) + 20 * (tar * 0.58);
      const rCol = Math.max(16, Math.min(220, baseLum * 0.97 + tint * 5));
      const gCol = Math.max(18, Math.min(225, baseLum * 1.00 + 2));
      const bCol = Math.max(21, Math.min(235, baseLum * 1.06 + 5 - tint * 3));

      const o = idx * 4;
      dMap[o]     = rCol | 0;
      dMap[o + 1] = gCol | 0;
      dMap[o + 2] = bCol | 0;
      dMap[o + 3] = 255;

      // Roughness: stone crowns ~0.72, porous binder ~0.94, wheel tracks smoother, tar veins glossy (~0.36)
      let rough = 228 - (h - 0.45) * 72 - laneWear * 26;
      rough = rough * (1 - tar * 0.62) + 92 * (tar * 0.62);
      const rByte = Math.max(78, Math.min(252, Math.round(rough)));
      dRough[o] = dRough[o + 1] = dRough[o + 2] = rByte;
      dRough[o + 3] = 255;
    }
  }
  gMap.putImageData(imgMap, 0, 0);
  gRough.putImageData(imgRough, 0, 0);

  // Subtle sparse mineral quartz glints & micro bitumen scuff patches
  for (let k = 0; k < 1200; k++) {
    const a = 0.08 + rnd() * 0.14;
    gMap.fillStyle = rnd() > 0.25 ? `rgba(198,205,216,${a})` : `rgba(182,174,158,${a})`;
    gMap.fillRect((rnd() * N) | 0, (rnd() * N) | 0, 1 + ((rnd() * 2) | 0), 1 + ((rnd() * 2) | 0));
  }

  // True 3×3 Sobel normal map with crisp aggregate & tar-crack relief
  const cNorm = buildSobelNormalCanvas(height, N, N, 1.45);

  const setup = (canvas, isSRGB) => {
    const t = new THREE.CanvasTexture(canvas);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.anisotropy = 16;
    if (isSRGB) t.colorSpace = THREE.SRGBColorSpace;
    return t;
  };
  return {
    map: setup(cMap, true),
    normalMap: setup(cNorm, false),
    roughnessMap: setup(cRough, false),
    puddleMap: buildPuddleNoiseTexture(),
  };
}

// ---------------------------------------------------------------------------
// 512×512 Lush Alpine Circuit Grass Albedo + Normal Map
// ---------------------------------------------------------------------------

/** Procedural 512×512 alpine circuit turf texture with mowed striping, organic
 *  FBM mottling, directional grass blades & natural clover/dry-stalk flecks. */
export function grassTexture() {
  const N = 512;
  const c = document.createElement("canvas");
  c.width = c.height = N;
  const g = c.getContext("2d");
  const img = g.createImageData(N, N);
  const d = img.data;
  const invN = 1 / N;
  let s = 0x51a7beef;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  for (let y = 0; y < N; y++) {
    const v = y * invN;
    // Crisp mowed FIA circuit-verge lawn stripes (soft alternating light/dark bands)
    const mowerStripe = Math.sin(v * Math.PI * 8) * 11;
    for (let x = 0; x < N; x++) {
      const u = x * invN;
      const macro = (tileFbm(u, v, 4, 3, 409) - 0.5) * 34;
      const clump = (tileValueNoise(u, v, 24, 733) - 0.5) * 20;
      const grain = (rnd() - 0.5) * 24;
      const delta = mowerStripe + macro + clump + grain;
      const warm = (tileValueNoise(u, v, 8, 881) - 0.5) * 16;
      const o = (y * N + x) * 4;
      d[o]     = Math.max(0, Math.min(255, Math.round(196 + delta * 0.72 + warm)));
      d[o + 1] = Math.max(0, Math.min(255, Math.round(218 + delta * 0.96 + warm * 0.3)));
      d[o + 2] = Math.max(0, Math.min(255, Math.round(182 + delta * 0.58 - warm * 0.4)));
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);

  // Fine organic grass blades & alpine meadow flecks
  for (let k = 0; k < 2600; k++) {
    const bx = rnd() * N;
    const by = rnd() * N;
    const len = 2 + rnd() * 5;
    const ang = -0.35 + rnd() * 0.7;
    const roll = rnd();
    g.strokeStyle = roll > 0.48
      ? "rgba(195,214,158,0.22)"
      : roll > 0.12
        ? "rgba(84,108,68,0.28)"
        : "rgba(168,154,108,0.20)";
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(bx, by);
    g.lineTo(bx + Math.sin(ang) * len, by - Math.cos(ang) * len);
    g.stroke();
  }

  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** Matching 512×512 tangent-space normal map for alpine turf so golden-hour sun
 *  grazes hillside meadows and mowed circuit verges with realistic depth. */
export function grassNormalMap() {
  const N = 512;
  const height = new Float32Array(N * N);
  const invN = 1 / N;
  let s = 0x77b4c9e1;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  for (let y = 0; y < N; y++) {
    const v = y * invN;
    const stripe = Math.sin(v * Math.PI * 8) * 0.08;
    const rowOff = y * N;
    for (let x = 0; x < N; x++) {
      const u = x * invN;
      const clump = tileFbm(u, v, 8, 3, 521) * 0.55;
      const blade = tileValueNoise(u, v, 48, 947) * 0.25;
      height[rowOff + x] = clump + blade + stripe + rnd() * 0.16;
    }
  }
  const cNorm = buildSobelNormalCanvas(height, N, N, 1.35);
  const tex = new THREE.CanvasTexture(cNorm);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.anisotropy = 8;
  return tex;
}

// ---------------------------------------------------------------------------
// 512×256 Weathered FIA Sawtooth Kerb PBR Texture Suite
// ---------------------------------------------------------------------------

function kerbPBRMaps() {
  const W = 512, H = 256;
  const height = new Float32Array(W * H);
  let s = 0x9e3779b9;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  const cMap = document.createElement("canvas");
  cMap.width = W; cMap.height = H;
  const gMap = cMap.getContext("2d");
  const imgMap = gMap.createImageData(W, H);
  const dMap = imgMap.data;

  const cRough = document.createElement("canvas");
  cRough.width = W; cRough.height = H;
  const gRough = cRough.getContext("2d");
  const imgRough = gRough.createImageData(W, H);
  const dRough = imgRough.data;

  // U = along kerb (2 blocks: red [0..0.5), white [0.5..1.0))
  // V = across profile (0 = inner lip, 0.15..0.75 = painted sawtooth deck, 0.75..1.0 = concrete backing)
  for (let y = 0; y < H; y++) {
    const v = 1 - y / (H - 1); // Three.js flipY=true -> bottom of canvas is V=0 (inner lip)
    const isBacking = v > 0.78;
    const backingBlend = Math.max(0, Math.min(1, (v - 0.74) / 0.08));
    // Apex rubber scuff band sits right along inner bevel & lower deck (v = 0.08..0.52)
    const scuffZone = Math.exp(-Math.pow((v - 0.28) / 0.20, 2));
    for (let x = 0; x < W; x++) {
      const u = x / W;
      const isRed = u < 0.5;
      // Molded concrete expansion seam every 0.5 U + sawtooth rib every 0.25 U
      const seamDist = Math.min(Math.abs(u - 0.5), Math.min(u, 1 - u));
      const seamGroove = seamDist < 0.012 ? (1 - seamDist / 0.012) : 0;
      const sawPhase = (u * 4) % 1;
      const sawBump = !isBacking ? Math.sin(sawPhase * Math.PI) * 0.22 * (1 - backingBlend) : 0;

      const pore = tileFbm(u, v, 8, 3, 211);
      const grit = (rnd() - 0.5) * 0.12;
      height[y * W + x] = 0.5 + sawBump + (pore - 0.5) * 0.18 + grit - seamGroove * 0.35;

      // Base FIA heritage crimson red vs warm concrete white
      let r = isRed ? 204 : 234;
      let g = isRed ? 34  : 237;
      let b = isRed ? 30  : 240;

      // Subtle concrete backing color on outer shoulder
      const concR = 136 + (pore - 0.5) * 28;
      const concG = 138 + (pore - 0.5) * 26;
      const concB = 134 + (pore - 0.5) * 24;
      r = r * (1 - backingBlend) + concR * backingBlend;
      g = g * (1 - backingBlend) + concG * backingBlend;
      b = b * (1 - backingBlend) + concB * backingBlend;

      // Subtle concrete pore micro-wear & darkened block seam
      const wear = (pore - 0.5) * 22 + grit * 26;
      r += wear; g += wear; b += wear;
      const seamDark = 1 - seamGroove * 0.48;
      r *= seamDark; g *= seamDark; b *= seamDark;

      // Tire rubber scuff streaks along apex edge
      const scuffNoise = tileValueNoise(u * 2.0, v * 0.6, 8, 643);
      const scuff = scuffZone * Math.max(0, scuffNoise - 0.32) * 0.92;
      r = r * (1 - scuff) + 34 * scuff;
      g = g * (1 - scuff) + 35 * scuff;
      b = b * (1 - scuff) + 39 * scuff;

      const o = (y * W + x) * 4;
      dMap[o]     = Math.max(0, Math.min(255, r | 0));
      dMap[o + 1] = Math.max(0, Math.min(255, g | 0));
      dMap[o + 2] = Math.max(0, Math.min(255, b | 0));
      dMap[o + 3] = 255;

      // Roughness: painted kerb satin-matte (0.62), rubber scuffs smoother (0.48), concrete backing rough (0.88)
      let rough = 158 + backingBlend * 68 - scuff * 36 + (pore - 0.5) * 28 + seamGroove * 40;
      const rb = Math.max(95, Math.min(248, Math.round(rough)));
      dRough[o] = dRough[o + 1] = dRough[o + 2] = rb;
      dRough[o + 3] = 255;
    }
  }
  gMap.putImageData(imgMap, 0, 0);
  gRough.putImageData(imgRough, 0, 0);

  // Subtle crisp rubber skid marks curving diagonally onto the apex edge
  gMap.strokeStyle = "rgba(26,28,33,0.34)";
  for (let k = 0; k < 22; k++) {
    const sx = rnd() * W;
    gMap.lineWidth = 2 + rnd() * 5;
    gMap.beginPath();
    gMap.moveTo(sx, H);
    gMap.quadraticCurveTo(sx + 24, H * 0.72, sx + 48, H * (0.48 + rnd() * 0.28));
    gMap.stroke();
  }

  const cNorm = buildSobelNormalCanvas(height, W, H, 1.35);
  const setup = (canvas, isSRGB) => {
    const t = new THREE.CanvasTexture(canvas);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.anisotropy = 12;
    if (isSRGB) t.colorSpace = THREE.SRGBColorSpace;
    return t;
  };
  return {
    map: setup(cMap, true),
    normalMap: setup(cNorm, false),
    roughnessMap: setup(cRough, false),
  };
}

// ---------------------------------------------------------------------------
// Weathered Road Markings: FIA Edge Line, Checkered Start & Grid Paint
// ---------------------------------------------------------------------------

/** Weathered FIA thermoplastic white track-limit line texture (asphalt bleed + micro-scuffs). */
function roadPaintTexture() {
  const W = 512, H = 64;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  const img = g.createImageData(W, H);
  const d = img.data;
  let s = 0x31415926;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  for (let y = 0; y < H; y++) {
    const edgeV = Math.min(y, H - 1 - y) / 6;
    const edgeAlpha = Math.max(0, Math.min(1, edgeV));
    for (let x = 0; x < W; x++) {
      const wear = tileValueNoise(x / W, y / H, 16, 431);
      const chip = rnd();
      const lum = Math.round(226 + (wear - 0.5) * 26 - (chip > 0.92 ? 42 : 0));
      const a = Math.round(255 * edgeAlpha * (0.84 + 0.16 * wear) * (chip > 0.965 ? 0.55 : 1.0));
      const o = (y * W + x) * 4;
      d[o]     = Math.max(0, Math.min(255, lum));
      d[o + 1] = Math.max(0, Math.min(255, lum + 2));
      d[o + 2] = Math.max(0, Math.min(255, lum + 4));
      d[o + 3] = Math.max(0, Math.min(255, a));
    }
  }
  g.putImageData(img, 0, 0);
  // Subtle rubber tire crossing scuffs
  for (let k = 0; k < 14; k++) {
    g.fillStyle = `rgba(32,35,40,${0.12 + rnd() * 0.14})`;
    g.fillRect(rnd() * W, 0, 6 + rnd() * 14, H);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 12;
  return tex;
}

/** High-res 512×128 weathered FIA checkered start/finish line with launch wheel scuffs. */
function checkerTexture() {
  const W = 512, H = 128;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  const cols = 18, rows = 3;
  const cw = W / cols, ch = H / rows;
  let s = 0x27182818;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  // Thin crisp FIA white timing border bands front & rear
  g.fillStyle = "#11151c";
  g.fillRect(0, 0, W, H);
  const padY = 10;
  const innerH = H - padY * 2;
  const rowH = innerH / rows;
  g.fillStyle = "#ebeff5";
  g.fillRect(0, 0, W, 6);
  g.fillRect(0, H - 6, W, 6);

  for (let r = 0; r < rows; r++) {
    for (let k = 0; k < cols; k++) {
      g.fillStyle = (r + k) % 2 === 0 ? "#eef2f7" : "#12161e";
      g.fillRect(k * cw, padY + r * rowH, cw + 0.6, rowH + 0.6);
    }
  }
  // Subtle asphalt stone pore weathering + dual launch rubber tracks
  const img = g.getImageData(0, 0, W, H);
  const d = img.data;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const u = x / W;
      const trackWear =
        Math.exp(-Math.pow((u - 0.30) / 0.055, 2)) +
        Math.exp(-Math.pow((u - 0.70) / 0.055, 2));
      const grit = (rnd() - 0.5) * 18;
      const o = (y * W + x) * 4;
      const dark = 1 - trackWear * 0.24;
      d[o]     = Math.max(10, Math.min(255, Math.round(d[o] * dark + grit)));
      d[o + 1] = Math.max(10, Math.min(255, Math.round(d[o + 1] * dark + grit)));
      d[o + 2] = Math.max(12, Math.min(255, Math.round(d[o + 2] * dark + grit)));
    }
  }
  g.putImageData(img, 0, 0);

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 12;
  return tex;
}

/** Weathered FIA F1 starting grid bracket texture (U-box + launch wheel burn marks). */
function gridBoxTexture() {
  const W = 256, H = 256;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  let s = 0x10293847;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  // Subtle twin dark rubber launch burnout marks inside the box
  const gradL = g.createLinearGradient(0, 56, 0, H);
  gradL.addColorStop(0, "rgba(18,20,24,0.0)");
  gradL.addColorStop(0.25, "rgba(18,20,24,0.34)");
  gradL.addColorStop(1, "rgba(18,20,24,0.08)");
  g.fillStyle = gradL;
  g.fillRect(46, 56, 32, 195);
  g.fillRect(W - 78, 56, 32, 195);

  // Crisp FIA white front stop bar + side bracket arms
  g.fillStyle = "rgba(238,243,249,0.92)";
  g.fillRect(14, 16, W - 28, 18);       // front line
  g.fillRect(14, 16, 16, 138);          // left wing
  g.fillRect(W - 30, 16, 16, 138);      // right wing
  // Subtle yellow FIA tire-alignment tick in front of box
  g.fillStyle = "rgba(242,198,42,0.88)";
  g.fillRect(W / 2 - 22, 42, 44, 7);

  // Weathering speckles (asphalt bleed-through)
  g.globalCompositeOperation = "destination-out";
  for (let k = 0; k < 650; k++) {
    g.fillStyle = `rgba(0,0,0,${0.08 + rnd() * 0.24})`;
    g.fillRect(rnd() * W, rnd() * H, 1 + rnd() * 2.5, 1 + rnd() * 2.5);
  }
  g.globalCompositeOperation = "source-over";

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

// ---------------------------------------------------------------------------
// Custom 2026 AAA Wet Puddle + Macro Breakup + Rubber Groove PBR Shader
// ---------------------------------------------------------------------------

const ASPHALT_VERT_PARS = /* glsl */ `
attribute vec3 aRoadData;
varying vec3 vWorldPos;
varying vec2 vRoadUv;
varying vec3 vRoadData;
`;

const ASPHALT_VERT_BODY = /* glsl */ `
vWorldPos = ( modelMatrix * vec4( transformed, 1.0 ) ).xyz;
vRoadUv = uv;
vRoadData = aRoadData;
`;

const ASPHALT_FRAG_PARS = /* glsl */ `
uniform float uWetness;
uniform float uTime;
uniform sampler2D uPuddleMap;
varying vec3 vWorldPos;
varying vec2 vRoadUv;
varying vec3 vRoadData;

float apexHash21( vec2 p ) {
  p = fract( p * vec2( 234.34, 435.345 ) );
  p += dot( p, p + 34.23 );
  return fract( p.x * p.y );
}

// Smooth concentric raindrop rings + subtle breeze shimmer in standing puddles
vec2 apexRainRipples( vec2 wp, float t ) {
  vec2 slope = vec2( 0.0 );
  // Two staggered raindrop grid scales for organic non-repeating impacts
  for ( int layer = 0; layer < 2; layer++ ) {
    float sc = layer == 0 ? 1.15 : 1.85;
    float spd = layer == 0 ? 0.72 : 0.94;
    vec2 g = wp * sc + vec2( float( layer ) * 17.3, float( layer ) * 31.7 );
    vec2 cell = floor( g );
    vec2 f = fract( g ) - 0.5;
    float h = apexHash21( cell );
    float phase = fract( t * spd + h );
    vec2 offset = ( vec2( apexHash21( cell + 11.7 ), apexHash21( cell + 43.1 ) ) - 0.5 ) * 0.42;
    vec2 d = f - offset;
    float dist = length( d );
    float rad = phase * 0.46;
    float env = ( 1.0 - phase ) * smoothstep( 0.0, 0.08, phase ) * smoothstep( 0.48, 0.14, dist );
    float ring = sin( ( dist - rad ) * 44.0 ) * exp( -abs( dist - rad ) * 17.0 ) * env;
    slope += ( d / ( dist + 0.002 ) ) * ( ring * 0.16 );
  }
  // Subtle wind micro-capillary wave across mirror water film
  slope.x += sin( wp.x * 3.4 + wp.y * 1.7 + t * 2.6 ) * 0.007;
  slope.y += cos( wp.y * 3.1 - wp.x * 1.4 + t * 2.2 ) * 0.007;
  return slope;
}
`;

const ASPHALT_FRAG_COLOR = /* glsl */ `
vec2 apexWp = vWorldPos.xz;
vec4 apexMacro = texture2D( uPuddleMap, apexWp * 0.0115 );
vec4 apexMed   = texture2D( uPuddleMap, apexWp * 0.043 + vec2( 0.37, 0.61 ) );
vec4 apexFine  = texture2D( uPuddleMap, apexWp * 0.19 - vec2( 0.19, 0.43 ) );

float apexRubber  = clamp( vRoadData.x, 0.0, 1.0 );
float apexBrake   = clamp( vRoadData.y, 0.0, 1.0 );
float apexMarbles = clamp( vRoadData.z, 0.0, 1.0 );

// Multi-frequency world-space asphalt breakup (eliminates straightaway tiling)
float apexPatch = ( apexMacro.g - 0.5 ) * 0.24 + ( apexMed.g - 0.5 ) * 0.12 + ( apexFine.g - 0.5 ) * 0.05;
diffuseColor.rgb *= ( 1.0 + apexPatch );

// Compacted vulcanized rubber racing line + longitudinal braking-zone lockup streaks
float apexStreakTex = texture2D( uPuddleMap, vec2( vRoadUv.x * 0.12, vRoadUv.y * 5.2 ) ).b;
float apexBrakeStreak = apexBrake * smoothstep( 0.30, 0.84, apexStreakTex ) * 0.48;
float apexRubAmt = clamp( apexRubber * 0.38 + apexBrakeStreak * 0.30, 0.0, 0.56 );
diffuseColor.rgb = mix( diffuseColor.rgb, diffuseColor.rgb * vec3( 0.44, 0.46, 0.50 ), apexRubAmt );

// Compatible with both direct updateSurface(wetness, timeSec) and legacy lighting.js roughness drive
float apexEffWet = max( uWetness, clamp( ( 0.82 - roughness ) / 0.56, 0.0, 1.0 ) );

// Dusty off-line marbles near margins (washes darker when wet)
float apexDust = apexMarbles * ( 0.18 + 0.16 * apexFine.g ) * ( 1.0 - apexEffWet * 0.72 );
diffuseColor.rgb = mix( diffuseColor.rgb, vec3( 0.23, 0.24, 0.25 ), apexDust );

// Natural standing-water puddle basins (pool off crown center & in micro-depressions)
float apexLatEdge = abs( vRoadUv.y - 0.5 ) * 2.0;
float apexBasin = ( apexMacro.r * 0.64 + apexMed.r * 0.28 + apexFine.a * 0.08 )
  * mix( 0.86, 1.09, smoothstep( 0.14, 0.85, apexLatEdge ) );
float apexPuddleThresh = mix( 0.92, 0.555, apexEffWet );
float apexPuddle = smoothstep( apexPuddleThresh - 0.045, apexPuddleThresh + 0.065, apexBasin )
  * smoothstep( 0.04, 0.32, apexEffWet );
float apexDamp = clamp( apexEffWet * ( 0.72 + 0.28 * apexMed.a ) + apexPuddle * 0.35, 0.0, 1.0 );

// Physical wet porous basalt darkening (deep rain-soaked charcoal vs mirror puddles)
float apexMatLum = max( max( diffuse.r, diffuse.g ), max( diffuse.b, 0.001 ) );
float apexWetTarget = mix( 1.0, 0.38, apexDamp ) * mix( 1.0, 0.74, apexPuddle );
float apexSafeWet = mix( 1.0, clamp( apexWetTarget / apexMatLum, 0.28, 1.0 ), apexEffWet );
diffuseColor.rgb *= apexSafeWet;
diffuseColor.rgb *= mix( vec3( 1.0 ), vec3( 0.90, 0.95, 1.03 ), apexEffWet * 0.42 );
`;

const ASPHALT_FRAG_ROUGHNESS = /* glsl */ `
#ifdef USE_ROUGHNESSMAP
  float apexDryRough = clamp( texelRoughness.g * 0.96 + ( apexMed.g - 0.5 ) * 0.08, 0.48, 0.98 );
#else
  float apexDryRough = 0.86;
#endif
apexDryRough = mix( apexDryRough, apexDryRough * 0.77, apexRubber * 0.78 );
apexDryRough = mix( apexDryRough, 0.95, apexMarbles * 0.48 );
float apexWetRough = mix( 0.44, 0.28, apexFine.a ) * mix( 1.0, 0.84, apexRubber );
roughnessFactor = mix( mix( apexDryRough, apexWetRough, apexDamp ), 0.032, apexPuddle );
`;

const ASPHALT_FRAG_NORMAL = /* glsl */ `
vec2 apexRippleSlope = vec2( 0.0 );
#ifdef USE_NORMALMAP_TANGENTSPACE
  if ( apexPuddle > 0.01 ) {
    float apexRipFade = smoothstep( 58.0, 7.0, length( vViewPosition ) );
    apexRippleSlope = apexRainRipples( vWorldPos.xz, uTime ) * apexPuddle * apexRipFade;
    vec3 apexWaterN = normalize( nonPerturbedNormal + tbn[ 0 ] * apexRippleSlope.x + tbn[ 1 ] * apexRippleSlope.y );
    normal = normalize( mix( normal, apexWaterN, apexPuddle * 0.94 ) );
  }
#endif
`;

const ASPHALT_FRAG_CLEARCOAT_NORMAL = /* glsl */ `
#if defined( USE_CLEARCOAT ) && defined( USE_NORMALMAP_TANGENTSPACE )
  if ( apexPuddle > 0.01 ) {
    clearcoatNormal = normalize(
      nonPerturbedNormal + tbn[ 0 ] * ( apexRippleSlope.x * 0.85 ) + tbn[ 1 ] * ( apexRippleSlope.y * 0.85 )
    );
  }
#endif
`;

const ASPHALT_FRAG_CLEARCOAT_PHYS = /* glsl */ `
#ifdef USE_CLEARCOAT
  float apexWaterFilm = clamp( apexEffWet * 0.22 + apexPuddle * 0.78, 0.0, 1.0 );
  material.clearcoat = mix( 0.04 + apexRubber * 0.06, 1.0, apexWaterFilm );
  float apexTargetCcR = mix( 0.44, mix( 0.28, 0.035, apexPuddle ), apexEffWet );
  material.clearcoatRoughness = clamp( apexTargetCcR + geometryRoughness * ( 1.0 - apexPuddle * 0.80 ), 0.03, 1.0 );
#endif
`;

// ---------------------------------------------------------------------------
// Track Mesh Builder
// ---------------------------------------------------------------------------

export function buildTrackMeshes(track) {
  const group = new THREE.Group();
  const n = track.S;
  const kappa = smoothedCurvature(track);

  // ---- Terrain (shoulders + v2 aprons + ground) with lush alpine turf PBR + dirt verge transition
  const terrainGeo = new THREE.BufferGeometry();
  const terrainVerts = track.verts.slice();
  for (let i = 0; i < n; i++) {
    terrainVerts[(i * 4 + 1) * 3 + 1] -= 0.04;
    terrainVerts[(i * 4 + 2) * 3 + 1] -= 0.04;
  }
  const vcount = track.verts.length / 3;
  const terrainUv = new Float32Array(vcount * 2);
  for (let v = 0; v < vcount; v++) {
    terrainUv[v * 2] = terrainVerts[v * 3] * 0.065;
    terrainUv[v * 2 + 1] = terrainVerts[v * 3 + 2] * 0.065;
  }
  terrainGeo.setAttribute("position", new THREE.BufferAttribute(terrainVerts, 3));
  terrainGeo.setAttribute("uv", new THREE.BufferAttribute(terrainUv, 2));
  terrainGeo.setIndex(new THREE.BufferAttribute(track.tris.slice(), 1));
  terrainGeo.computeVertexNormals();
  {
    const apronEnd = vcount >= n * 12 + 4 ? n * 12 : n * 4;
    const colors = new Float32Array(vcount * 3);
    let s = (Number(fnv1a64(track.bytes) & 0xffffffffn) >>> 0) || 1;
    const rnd = () => (s = (Math.imul(s, 1664525) + 1013904223) >>> 0) / 2 ** 32;
    const grassRich = new THREE.Color(0x365037);
    const grassDeep = new THREE.Color(0x283e2c);
    const vergeSoil = new THREE.Color(0x524b3d);
    const tmpC = new THREE.Color();

    for (let v = 0; v < vcount; v++) {
      const m = (rnd() - 0.5) * 0.16;
      if (v < n * 4) {
        const lane = v & 3;
        // Inner road-edge shoulder verts (1 & 2) carry warm gravel/compacted soil verge transition
        if (lane === 1 || lane === 2) {
          tmpC.copy(vergeSoil).lerp(grassRich, 0.32);
        } else {
          tmpC.copy(grassRich).lerp(vergeSoil, 0.12);
        }
        colors[v * 3]     = tmpC.r * (1 + m * 0.7);
        colors[v * 3 + 1] = tmpC.g * (1 + m);
        colors[v * 3 + 2] = tmpC.b * (1 + m * 0.55);
      } else if (v < apronEnd) {
        const row = (v - n * 4) & 3;
        const tRow = (row + 1) / 4;
        tmpC.copy(grassRich).lerp(grassDeep, tRow * 0.72);
        const shade = 1 - 0.03 * (row + 1);
        colors[v * 3]     = tmpC.r * (1 + m * 0.7) * shade;
        colors[v * 3 + 1] = tmpC.g * (1 + m) * shade;
        colors[v * 3 + 2] = tmpC.b * (1 + m * 0.55) * shade;
      } else {
        colors[v * 3]     = grassDeep.r * 0.92;
        colors[v * 3 + 1] = grassDeep.g * 0.95;
        colors[v * 3 + 2] = grassDeep.b * 0.92;
      }
    }
    terrainGeo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  }
  const turfTex = grassTexture();
  const turfNorm = grassNormalMap();
  const grassMat = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    map: turfTex,
    normalMap: turfNorm,
    normalScale: new THREE.Vector2(0.55, 0.55),
    vertexColors: true,
    roughness: 0.94,
    metalness: 0.0,
    envMapIntensity: 0.42,
  });
  group.add(new THREE.Mesh(terrainGeo, grassMat));
  group.userData.grassMat = grassMat;
  group.userData.grassTex = turfTex;

  // ---- Compute racing-line lateral offsets & heavy braking-zone intensities
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
    const LOOK = 14;
    for (let i = 0; i < n; i++) {
      for (let d = 0; d <= LOOK; d++) {
        if (Math.abs(kappa[(i + d) % n]) > KERB_KAPPA) {
          brake[i] = 1 - d / (LOOK + 2);
          break;
        }
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

  // ---- High-res 11-row drivable PBR asphalt ribbon (matches physics parabolic crown)
  // Bakes aRoadData = (rubberGroove, brakeZone, edgeMarbles) directly into PBR shader!
  const ROWS = [-1, -0.86, -0.68, -0.48, -0.25, 0, 0.25, 0.48, 0.68, 0.86, 1];
  const roadH = (f) => 0.005 - 0.025 * f * f;
  const R = ROWS.length;
  const pos = new Float32Array(n * R * 3);
  const uvs = new Float32Array(n * R * 2);
  const roadData = new Float32Array(n * R * 3);
  let arc = 0;
  for (let i = 0; i < n; i++) {
    const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
    const side = new THREE.Vector3().crossVectors(u, t).normalize();
    const half = track.width[i] * 0.5;
    const uAlong = arc / 6;
    const bVal = brake[i];
    const grooveW = 1.45 * (1 + 0.42 * bVal);
    for (let r = 0; r < R; r++) {
      const f = ROWS[r];
      const lat = f * half;
      const p = c.clone().addScaledVector(side, lat).addScaledVector(u, roadH(f));
      const vIdx = i * R + r;
      pos.set([p.x, p.y, p.z], vIdx * 3);
      uvs.set([uAlong, (f + 1) * 0.5], vIdx * 2);

      // Dual-track rubbered-in racing groove centered on lineOff[i]
      const dLine = lat - lineOff[i];
      const wheelTrack =
        0.55 * Math.exp(-Math.pow((dLine - 0.78) / (grooveW * 0.58), 2)) +
        0.55 * Math.exp(-Math.pow((dLine + 0.78) / (grooveW * 0.58), 2)) +
        0.42 * Math.exp(-Math.pow(dLine / grooveW, 2));
      const rub = Math.min(1, wheelTrack * (0.62 + 0.52 * bVal));
      const brk = Math.min(1, bVal * Math.exp(-Math.pow(dLine / (grooveW * 1.15), 2)));

      // Off-line marbles accumulate near outer margins away from the racing line
      const nearEdge = Math.max(0, Math.min(1, (Math.abs(f) - 0.56) / 0.38));
      const offLine = Math.max(0, Math.min(1, (Math.abs(dLine) - 1.55) / 1.8));
      const marbles = nearEdge * offLine;
      roadData.set([rub, brk, marbles], vIdx * 3);
    }
    arc += 2.5;
  }
  const idx = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    for (let r = 0; r < R - 1; r++) {
      const a = i * R + r, b = i * R + r + 1, c2 = j * R + r, d = j * R + r + 1;
      // CCW from +up: (c2 - a) x (b - a) = (+t) x (+side) = +up
      idx.push(a, c2, b, c2, d, b);
    }
  }
  const ribbonGeo = new THREE.BufferGeometry();
  ribbonGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  ribbonGeo.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
  ribbonGeo.setAttribute("aRoadData", new THREE.BufferAttribute(roadData, 3));
  ribbonGeo.setIndex(idx);
  ribbonGeo.computeVertexNormals();

  const pbrMaps = asphaltPBRMaps();
  const asphaltUniforms = {
    uWetness: { value: 0.0 },
    uTime: { value: 0.0 },
    uPuddleMap: { value: pbrMaps.puddleMap },
  };
  const asphaltMat = new THREE.MeshPhysicalMaterial({
    color: 0xffffff,
    map: pbrMaps.map,
    normalMap: pbrMaps.normalMap,
    normalScale: new THREE.Vector2(0.38, 0.38),
    roughnessMap: pbrMaps.roughnessMap,
    roughness: 0.86,
    metalness: 0.02,
    ior: 1.48,
    reflectivity: 0.72,
    clearcoat: 0.08,
    clearcoatRoughness: 0.38,
    envMapIntensity: 1.42,
    side: THREE.DoubleSide,
  });
  asphaltMat.customProgramCacheKey = () => "ApexAsphalt2026";
  asphaltMat.onBeforeCompile = (shader) => {
    shader.uniforms.uWetness = asphaltUniforms.uWetness;
    shader.uniforms.uTime = asphaltUniforms.uTime;
    shader.uniforms.uPuddleMap = asphaltUniforms.uPuddleMap;

    shader.vertexShader = shader.vertexShader
      .replace("#include <common>", `#include <common>\n${ASPHALT_VERT_PARS}`)
      .replace("#include <begin_vertex>", `#include <begin_vertex>\n${ASPHALT_VERT_BODY}`);

    shader.fragmentShader = shader.fragmentShader
      .replace("#include <common>", `#include <common>\n${ASPHALT_FRAG_PARS}`)
      .replace("#include <color_fragment>", `#include <color_fragment>\n${ASPHALT_FRAG_COLOR}`)
      .replace("#include <roughnessmap_fragment>", `#include <roughnessmap_fragment>\n${ASPHALT_FRAG_ROUGHNESS}`)
      .replace("#include <normal_fragment_maps>", `#include <normal_fragment_maps>\n${ASPHALT_FRAG_NORMAL}`)
      .replace("#include <clearcoat_normal_fragment_maps>", `#include <clearcoat_normal_fragment_maps>\n${ASPHALT_FRAG_CLEARCOAT_NORMAL}`)
      .replace("#include <lights_physical_fragment>", `#include <lights_physical_fragment>\n${ASPHALT_FRAG_CLEARCOAT_PHYS}`);
  };
  const ribbonMesh = new THREE.Mesh(ribbonGeo, asphaltMat);
  ribbonMesh.receiveShadow = true;
  group.add(ribbonMesh);
  group.userData.asphaltMat = asphaltMat;

  // Two-row strip index pairs (sign-aware CCW so normals always point +up)
  const stripIdxPos = [];
  const stripIdxNeg = [];
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    stripIdxPos.push(2 * i, 2 * j, 2 * i + 1, 2 * j, 2 * j + 1, 2 * i + 1);
    stripIdxNeg.push(2 * i, 2 * i + 1, 2 * j, 2 * j, 2 * i + 1, 2 * j + 1);
  }

  // ---- Weathered 20 cm FIA white track-limit edge lines + straightaway verge accents
  const paintTex = roadPaintTexture();
  const edgeMat = new THREE.MeshStandardMaterial({
    color: 0xf0f4fa,
    map: paintTex,
    transparent: true,
    emissive: 0xd0dce8,
    emissiveIntensity: 0.16,
    roughness: 0.56,
    metalness: 0.0,
    polygonOffset: true,
    polygonOffsetFactor: -1,
    polygonOffsetUnits: -2,
    side: THREE.DoubleSide,
  });
  for (const sign of [-1, 1]) {
    const ep = new Float32Array(n * 2 * 3);
    const euv = new Float32Array(n * 2 * 2);
    for (let i = 0; i < n; i++) {
      const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const half = track.width[i] * 0.5;
      const latA = sign * (half - 0.23);
      const latB = sign * (half - 0.02);
      const pA = c.clone().addScaledVector(side, latA).addScaledVector(u, roadH(latA / half) + 0.011);
      const pB = c.clone().addScaledVector(side, latB).addScaledVector(u, roadH(latB / half) + 0.011);
      ep.set([pA.x, pA.y, pA.z], i * 6);
      ep.set([pB.x, pB.y, pB.z], i * 6 + 3);
      const uLen = (i * 2.5) / 5.0;
      euv.set([uLen, 0], i * 4);
      euv.set([uLen, 1], i * 4 + 2);
    }
    const eg = new THREE.BufferGeometry();
    eg.setAttribute("position", new THREE.BufferAttribute(ep, 3));
    eg.setAttribute("uv", new THREE.BufferAttribute(euv, 2));
    eg.setIndex(sign > 0 ? stripIdxPos : stripIdxNeg);
    eg.computeVertexNormals();
    const em = new THREE.Mesh(eg, edgeMat);
    em.renderOrder = 3;
    em.receiveShadow = true;
    group.add(em);
  }

  // Subtle dashed outer concrete verge rumble blocks on non-kerb straight sections
  {
    const vp = [], vuv = [], vidx = [];
    for (const sign of [-1, 1]) {
      for (let i = 0; i < n; i += 2) {
        const j = (i + 1) % n;
        if (Math.abs(kappa[i]) > KERB_KAPPA * 0.85 || Math.abs(kappa[j]) > KERB_KAPPA * 0.85) continue;
        // Skip right over start/finish grid area for clean pit-straight look
        if (i < 4 || i > n - 18) continue;
        const vb = vp.length / 3;
        for (const s of [i, j]) {
          const c = v3(track.center[s]), u = v3(track.up[s]), t = v3(track.tangent[s]);
          const sd = new THREE.Vector3().crossVectors(u, t).normalize();
          const half = track.width[s] * 0.5;
          const p0 = c.clone().addScaledVector(sd, sign * (half + 0.02)).addScaledVector(u, roadH(1) + 0.008);
          const p1 = c.clone().addScaledVector(sd, sign * (half + 0.42)).addScaledVector(u, roadH(1) + 0.016);
          vp.push(p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);
        }
        vuv.push(0, 0, 0, 1, 1, 0, 1, 1);
        if (sign > 0) vidx.push(vb, vb + 2, vb + 1, vb + 2, vb + 3, vb + 1);
        else vidx.push(vb, vb + 1, vb + 2, vb + 2, vb + 1, vb + 3);
      }
    }
    if (vp.length) {
      const vg = new THREE.BufferGeometry();
      vg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(vp), 3));
      vg.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(vuv), 2));
      vg.setIndex(vidx);
      vg.computeVertexNormals();
      const vergeMat = new THREE.MeshStandardMaterial({
        color: 0x6f767f,
        roughness: 0.86,
        metalness: 0.02,
        side: THREE.DoubleSide,
      });
      const vm = new THREE.Mesh(vg, vergeMat);
      vm.receiveShadow = true;
      group.add(vm);
    }
  }

  // ---- 3D Sculpted Sawtooth Chamfered FIA Kerbs with PBR Concrete/Scuff Maps
  const kerbMaps = kerbPBRMaps();
  const kerbMat = new THREE.MeshPhysicalMaterial({
    color: 0xffffff,
    map: kerbMaps.map,
    normalMap: kerbMaps.normalMap,
    normalScale: new THREE.Vector2(0.85, 0.85),
    roughnessMap: kerbMaps.roughnessMap,
    roughness: 0.64,
    metalness: 0.02,
    clearcoat: 0.05,
    clearcoatRoughness: 0.36,
    envMapIntensity: 0.95,
    side: THREE.DoubleSide,
  });
  group.userData.kerbMat = kerbMat;

  {
    // 5-station 3D cross-section: flush inner lip -> bevel rise -> sawtooth deck -> crest -> buried concrete backing
    const PROF_D = [-0.02, 0.14, 0.72, 1.06, 1.32];
    const PROF_H = [-0.014, 0.032, 0.074, 0.082, -0.032];
    const PROF_V = [0.0, 0.14, 0.56, 0.78, 1.0];
    const P = PROF_D.length;
    const kerbPos = [];
    const kerbUv = [];
    const kerbIdx = [];

    const sampleRing = (sIdx, sign, sawAmp, rampScale) => {
      const c = v3(track.center[sIdx]), u = v3(track.up[sIdx]), t = v3(track.tangent[sIdx]);
      const sd = new THREE.Vector3().crossVectors(u, t).normalize();
      const half = track.width[sIdx] * 0.5;
      const pts = [];
      for (let p = 0; p < P; p++) {
        const dOut = PROF_D[p];
        const saw = (p === 1 || p === 2) ? sawAmp * (p === 2 ? 1.0 : 0.6) : 0;
        const h = PROF_H[p] >= 0
          ? roadH(1) + (PROF_H[p] + saw) * rampScale
          : roadH(1) + PROF_H[p];
        pts.push(c.clone().addScaledVector(sd, sign * (half + dOut)).addScaledVector(u, h));
      }
      return pts;
    };

    const pushStrip = (ringA, ringB, uA, uB, sign) => {
      const base = kerbPos.length / 3;
      for (let p = 0; p < P; p++) {
        kerbPos.push(ringA[p].x, ringA[p].y, ringA[p].z);
        kerbUv.push(uA, PROF_V[p]);
      }
      for (let p = 0; p < P; p++) {
        kerbPos.push(ringB[p].x, ringB[p].y, ringB[p].z);
        kerbUv.push(uB, PROF_V[p]);
      }
      for (let p = 0; p < P - 1; p++) {
        const a0 = base + p, a1 = base + p + 1, b0 = base + P + p, b1 = base + P + p + 1;
        if (sign > 0) kerbIdx.push(a0, b0, a1, b0, b1, a1);
        else kerbIdx.push(a0, a1, b0, b0, a1, b1);
      }
    };

    for (const sign of [-1, 1]) {
      const active = new Uint8Array(n);
      for (let i = 0; i < n; i++) active[i] = Math.abs(kappa[i]) > KERB_KAPPA ? 1 : 0;

      for (let i = 0; i < n; i++) {
        const j = (i + 1) % n;
        if (!active[i] || !active[j]) continue;
        const prevAct = active[(i - 1 + n) % n];
        const nextAct = active[(j + 1) % n];
        const rampA = prevAct ? 1.0 : 0.25;
        const rampB = nextAct ? 1.0 : 0.25;
        const sawA = (i & 1) ? 0.018 : 0.0;
        const sawB = (j & 1) ? 0.018 : 0.0;
        // Each sample step (2.5m) maps 0.5 U => exact 2.5m red / 2.5m white FIA blocks
        const uA = i * 0.5;
        const uB = uA + 0.5;
        pushStrip(sampleRing(i, sign, sawA, rampA), sampleRing(j, sign, sawB, rampB), uA, uB, sign);
      }
    }

    if (kerbPos.length) {
      const kg = new THREE.BufferGeometry();
      kg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(kerbPos), 3));
      kg.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(kerbUv), 2));
      kg.setIndex(kerbIdx);
      kg.computeVertexNormals();
      const km = new THREE.Mesh(kg, kerbMat);
      km.receiveShadow = true;
      group.add(km);
    }
  }

  // ---- Checkered FIA Start/Finish Line + Staggered F1 Grid Boxes + Sector Strips
  {
    const c0 = v3(track.center[0]), u0 = v3(track.up[0]).normalize(), t0 = v3(track.tangent[0]).normalize();
    const s0 = new THREE.Vector3().crossVectors(u0, t0).normalize();
    const basis0 = new THREE.Matrix4().makeBasis(s0, u0, t0);
    const q0 = new THREE.Quaternion().setFromRotationMatrix(basis0);

    const startGeo = new THREE.PlaneGeometry(track.width[0] - 0.44, 2.05);
    startGeo.rotateX(-Math.PI / 2);
    const startMat = new THREE.MeshStandardMaterial({
      map: checkerTexture(),
      emissive: 0x262c34,
      emissiveIntensity: 0.20,
      roughness: 0.58,
      metalness: 0.0,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    });
    const startMesh = new THREE.Mesh(startGeo, startMat);
    startMesh.position.copy(c0).addScaledVector(u0, 0.015);
    startMesh.quaternion.copy(q0);
    startMesh.renderOrder = 4;
    startMesh.receiveShadow = true;
    group.add(startMesh);

    // 4 Staggered F1 Starting Grid Boxes (weathered paint + twin launch burn marks)
    const gridTex = gridBoxTexture();
    const gridMat = new THREE.MeshStandardMaterial({
      map: gridTex,
      transparent: true,
      roughness: 0.62,
      metalness: 0.0,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
      side: THREE.DoubleSide,
    });
    const gridGeo = new THREE.PlaneGeometry(2.55, 3.1);
    gridGeo.rotateX(-Math.PI / 2);
    gridGeo.rotateY(Math.PI); // aligns front stop bar (+top of canvas) to +Z travel direction
    for (let slot = 0; slot < 4; slot++) {
      const idxBack = (n - 3 - slot * 3) % n;
      const cs = v3(track.center[idxBack]), us = v3(track.up[idxBack]).normalize(), ts = v3(track.tangent[idxBack]).normalize();
      const ss = new THREE.Vector3().crossVectors(us, ts).normalize();
      const qs = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(ss, us, ts));
      const lat = (slot % 2 === 0 ? -1 : 1) * Math.min(2.35, track.width[idxBack] * 0.23);
      const gm = new THREE.Mesh(gridGeo, gridMat);
      gm.position.copy(cs)
        .addScaledVector(ss, lat)
        .addScaledVector(us, roadH(lat / (track.width[idxBack] * 0.5)) + 0.014);
      gm.quaternion.copy(qs);
      gm.renderOrder = 4;
      gm.receiveShadow = true;
      group.add(gm);
    }
  }

  // ---- Subtle FIA Sector Timing Loops at Checkpoints
  {
    const secMat = new THREE.MeshStandardMaterial({
      color: 0x26e8d6,
      emissive: 0x16a89c,
      emissiveIntensity: 0.45,
      transparent: true,
      opacity: 0.42,
      roughness: 0.45,
      depthWrite: false,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
      side: THREE.DoubleSide,
    });
    for (const cp of track.checkpoints) {
      const c = v3(track.center[cp]), u = v3(track.up[cp]).normalize(), t = v3(track.tangent[cp]).normalize();
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const g = new THREE.PlaneGeometry(track.width[cp] - 0.5, 0.42);
      g.rotateX(-Math.PI / 2);
      const m = new THREE.Mesh(g, secMat);
      m.position.copy(c).addScaledVector(u, 0.015);
      m.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(side, u, t));
      m.renderOrder = 4;
      group.add(m);
    }
  }

  // ---- Dynamic Surface Wetness & Ripple Hook (zero per-frame allocations)
  group.userData.updateSurface = (wetness, timeSec = 0) => {
    const w = wetness > 1 ? 1 : wetness < 0 ? 0 : (wetness || 0);
    asphaltUniforms.uWetness.value = w;
    asphaltUniforms.uTime.value = timeSec || 0;
    asphaltMat.clearcoat = 0.06 + 0.86 * w;
    asphaltMat.clearcoatRoughness = 0.42 - 0.34 * w;
    asphaltMat.envMapIntensity = 0.92 + 0.26 * w;
    kerbMat.roughness = 0.64 - 0.34 * w;
    kerbMat.clearcoat = 0.05 + 0.58 * w;
    kerbMat.clearcoatRoughness = 0.36 - 0.22 * w;
    kerbMat.envMapIntensity = 0.82 + 0.38 * w;
    grassMat.roughness = 0.94 - 0.18 * w;
  };

  return group;
}

// ---------------------------------------------------------------------------
// 2026 F1 Broadcast-Style Minimap
// ---------------------------------------------------------------------------

export class Minimap {
  constructor(canvas, track) {
    this.ctx = canvas.getContext("2d");
    this.w = canvas.width;
    this.h = canvas.height;
    this.track = track;
    this.trail = [];

    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (const c of track.center) {
      if (c[0] < minX) minX = c[0];
      if (c[0] > maxX) maxX = c[0];
      if (c[2] < minZ) minZ = c[2];
      if (c[2] > maxZ) maxZ = c[2];
    }
    const pad = 16;
    const spanX = Math.max(1, maxX - minX);
    const spanZ = Math.max(1, maxZ - minZ);
    const scale = Math.min((this.w - 2 * pad) / spanX, (this.h - 2 * pad) / spanZ);
    const offX = (this.w - spanX * scale) * 0.5;
    const offY = (this.h - spanZ * scale) * 0.5;
    this.map = (p) => [offX + (p[0] - minX) * scale, offY + (p[2] - minZ) * scale];

    // Precompute projected circuit points + start/finish perpendicular tick
    this._pts = track.center.map((c) => this.map(c));
    this._cpPts = track.checkpoints.map((cp) => this._pts[cp]);
    const p0 = this._pts[0];
    const p1 = this._pts[1 % this._pts.length];
    const dx = p1[0] - p0[0], dy = p1[1] - p0[1];
    const len = Math.hypot(dx, dy) || 1;
    this._sfNx = -dy / len;
    this._sfNy = dx / len;
  }

  draw(carPos, checkpointMask, ghostPos = null) {
    const { ctx, trail, _pts, _cpPts } = this;
    ctx.clearRect(0, 0, this.w, this.h);
    ctx.lineCap = ctx.lineJoin = "round";

    // Build closed circuit path once
    ctx.beginPath();
    for (let i = 0; i < _pts.length; i++) {
      const pt = _pts[i];
      if (i === 0) ctx.moveTo(pt[0], pt[1]);
      else ctx.lineTo(pt[0], pt[1]);
    }
    ctx.closePath();

    // 3-layer crisp F1 broadcast track stroke: outer halo -> dark tarmac body -> clean racing line
    ctx.strokeStyle = "rgba(4, 8, 15, 0.78)";
    ctx.lineWidth = 7.6;
    ctx.stroke();
    ctx.strokeStyle = "rgba(24, 33, 46, 0.94)";
    ctx.lineWidth = 4.6;
    ctx.stroke();
    ctx.strokeStyle = "#d5e0ee";
    ctx.lineWidth = 1.9;
    ctx.stroke();

    // Checkered FIA Start/Finish tick perpendicular to circuit
    const s0 = _pts[0];
    const nx = this._sfNx, ny = this._sfNy;
    ctx.lineCap = "butt";
    ctx.strokeStyle = "#0b1018";
    ctx.lineWidth = 4.2;
    ctx.beginPath();
    ctx.moveTo(s0[0] - nx * 5.8, s0[1] - ny * 5.8);
    ctx.lineTo(s0[0] + nx * 5.8, s0[1] + ny * 5.8);
    ctx.stroke();
    // Alternating white/dark micro-checks across start bar
    for (let k = -2; k <= 1; k++) {
      const a = k * 2.4;
      const b = a + 2.4;
      ctx.strokeStyle = (k & 1) === 0 ? "#ffffff" : "#151c27";
      ctx.lineWidth = 2.4;
      ctx.beginPath();
      ctx.moveTo(s0[0] + nx * a, s0[1] + ny * a);
      ctx.lineTo(s0[0] + nx * b, s0[1] + ny * b);
      ctx.stroke();
    }
    ctx.lineCap = "round";

    // Sector checkpoint markers
    for (let k = 0; k < _cpPts.length; k++) {
      const pt = _cpPts[k];
      const hit = (checkpointMask & (1 << k)) !== 0;
      ctx.fillStyle = hit ? "#6ef2a8" : "rgba(34, 224, 208, 0.85)";
      ctx.strokeStyle = "rgba(5, 10, 18, 0.85)";
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(pt[0], pt[1], hit ? 2.8 : 2.4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }

    // Smooth glowing motion trail (~2 s)
    const [cx, cy] = this.map(carPos);
    const now = performance.now();
    const tail = trail[trail.length - 1];
    if (tail && Math.hypot(cx - tail[0], cy - tail[1]) > 25) trail.length = 0;
    if (!tail || now - tail[2] > 45) trail.push([cx, cy, now]);
    while (trail.length && now - trail[0][2] > 2000) trail.shift();

    for (let i = 1; i < trail.length; i++) {
      const age = 1 - (now - trail[i][2]) / 2000;
      ctx.strokeStyle = `rgba(38, 232, 214, ${(0.62 * age * age).toFixed(3)})`;
      ctx.lineWidth = 1.1 + 1.4 * age;
      ctx.beginPath();
      ctx.moveTo(trail[i - 1][0], trail[i - 1][1]);
      ctx.lineTo(trail[i][0], trail[i][1]);
      ctx.stroke();
    }

    // Ghost dot (translucent ice-blue beacon with subtle halo)
    if (ghostPos) {
      const [gx, gy] = this.map(ghostPos);
      ctx.fillStyle = "rgba(175, 215, 255, 0.25)";
      ctx.beginPath();
      ctx.arc(gx, gy, 5.6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "rgba(198, 224, 255, 0.92)";
      ctx.strokeStyle = "rgba(6, 12, 22, 0.72)";
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(gx, gy, 3.0, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }

    // Player car broadcast beacon with subtle cyan/white radial glow
    const grad = ctx.createRadialGradient(cx, cy, 1.0, cx, cy, 9.5);
    grad.addColorStop(0, "rgba(38, 236, 220, 0.55)");
    grad.addColorStop(0.55, "rgba(38, 236, 220, 0.16)");
    grad.addColorStop(1, "rgba(38, 236, 220, 0.0)");
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(cx, cy, 9.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "rgba(6, 12, 22, 0.85)";
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.arc(cx, cy, 3.9, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
}

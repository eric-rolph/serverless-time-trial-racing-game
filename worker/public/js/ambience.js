// 2026 AAA Procedural Circuit World — seeded deterministically from TRK1 bytes.
// Zero external URLs/assets; 100% Three.js r170 PBR + InstancedMesh architecture:
//   • Exact CONTRACTS §8 ground plain (groundY = minY - 1.35) with alpine turf normal map;
//   • Two-ring layered alpine horizon (wooded foothills + craggy snow-dusted alpine peaks);
//   • Architectural F1/FIA Pit Complex (cantilevered roof, illuminated recessed garage bays,
//     reflective curtain-wall hospitality floor, glass race control box) + sleek FIA Timing Tower;
//   • Modern Stadium Grandstands with angled tension-cable roofs, under-roof LED stadium light bars,
//     glass terrace balustrades, high-res 512×128 tiered crowd & FIA circuit sponsor barrier boards;
//   • Open Space-Frame FIA Start/Finish Timing Gantry with 1024×256 dual-sided LED timing banner,
//     underside grid floodlight bar & 5 hooded high-HDR FIA start-light pods (setStartLights);
//   • 12–14 Trackside Stadium Floodlight Masts angled toward the tarmac with emissive LED banks;
//   • Digital FIA LED Marshal Flag Panels animated in updateAmbience(timeSec, wetness);
//   • Collision-exact v2 barriers (ribbed tire walls, galvanized W-profile Armco with I-posts &
//     reflector chevrons, weathered precast concrete walls with catch-fence uprights) + v1 fallback;
//   • Raked FIA pebble gravel traps (256×256 albedo + Sobel normal) & 4-tier pines + organic oaks.
import * as THREE from "../vendor/three.module.js";
import { fnv1a64, smoothedCurvature, KERB_KAPPA, grassTexture, grassNormalMap } from "./track.js";

const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

/** Merge an array of BufferGeometries (positions + normals, and UVs if present on all). */
function mergeSimpleGeos(geos) {
  const expanded = geos.map((g) => (g.index ? g.toNonIndexed() : g));
  let totalVerts = 0;
  let hasUv = true;
  for (const g of expanded) {
    totalVerts += g.attributes.position.count;
    if (!g.attributes.uv) hasUv = false;
  }
  const pos = new Float32Array(totalVerts * 3);
  const nrm = new Float32Array(totalVerts * 3);
  const uv = hasUv ? new Float32Array(totalVerts * 2) : null;
  let off = 0;
  for (const g of expanded) {
    if (!g.attributes.normal) g.computeVertexNormals();
    const count = g.attributes.position.count;
    pos.set(g.attributes.position.array, off * 3);
    nrm.set(g.attributes.normal.array, off * 3);
    if (uv) uv.set(g.attributes.uv.array, off * 2);
    off += count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  out.setAttribute("normal", new THREE.BufferAttribute(nrm, 3));
  if (uv) out.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  return out;
}

/** Register an emissive material with glowMaterials and record its base daytime intensity. */
function registerGlow(glowMaterials, mat, baseEmissive) {
  mat.userData.baseEmissive = baseEmissive;
  mat.emissiveIntensity = baseEmissive;
  glowMaterials.push(mat);
  return mat;
}

/** Build OpenGL/Three.js tangent-space Sobel normal canvas from tileable heightfield. */
function sobelNormalCanvas(height, W, H, strength) {
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  const img = g.createImageData(W, H);
  const d = img.data;
  const mx = W - 1, my = H - 1;
  for (let y = 0; y < H; y++) {
    const ym = ((y - 1) & my) * W;
    const y0 = y * W;
    const yp = ((y + 1) & my) * W;
    for (let x = 0; x < W; x++) {
      const xm = (x - 1) & mx;
      const xp = (x + 1) & mx;
      const gx = ((height[ym + xp] + 2 * height[y0 + xp] + height[yp + xp]) -
                  (height[ym + xm] + 2 * height[y0 + xm] + height[yp + xm])) * strength;
      const gy = ((height[yp + xm] + 2 * height[yp + x] + height[yp + xp]) -
                  (height[ym + xm] + 2 * height[ym + x] + height[ym + xp])) * strength;
      const nx = -gx, ny = gy;
      const inv = 1 / Math.hypot(nx, ny, 1);
      const o = (y0 + x) * 4;
      d[o]     = Math.max(0, Math.min(255, Math.round((nx * inv * 0.5 + 0.5) * 255)));
      d[o + 1] = Math.max(0, Math.min(255, Math.round((ny * inv * 0.5 + 0.5) * 255)));
      d[o + 2] = Math.max(0, Math.min(255, Math.round((inv * 0.5 + 0.5) * 255)));
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return c;
}

// ---------------------------------------------------------------------------
// Procedural 2026 Circuit Textures (all 100% offline canvas/typed-array)
// ---------------------------------------------------------------------------

/** Crisp 1024×256 dual-sided FIA LED timing gantry banner texture. */
function gantryBannerTexture() {
  const W = 1024, H = 256;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");

  // Deep obsidian LED panel background with subtle carbon gradient
  const bg = g.createLinearGradient(0, 0, 0, H);
  bg.addColorStop(0, "#090d15");
  bg.addColorStop(0.5, "#0e1522");
  bg.addColorStop(1, "#070b12");
  g.fillStyle = bg;
  g.fillRect(0, 0, W, H);

  // Diagonal motorsport chevron accent blocks on far ends
  g.fillStyle = "#d92b24";
  for (let k = 0; k < 3; k++) {
    const xL = 22 + k * 28;
    g.beginPath();
    g.moveTo(xL, 26); g.lineTo(xL + 16, 26); g.lineTo(xL + 44, H - 26); g.lineTo(xL + 28, H - 26);
    g.closePath(); g.fill();
    const xR = W - 22 - k * 28;
    g.beginPath();
    g.moveTo(xR, 26); g.lineTo(xR - 16, 26); g.lineTo(xR - 44, H - 26); g.lineTo(xR - 28, H - 26);
    g.closePath(); g.fill();
  }

  // Luminous top & bottom FIA cyan telemetry light bars
  g.fillStyle = "#22e6d6";
  g.fillRect(0, 0, W, 12);
  g.fillRect(0, H - 12, W, 12);
  g.fillStyle = "rgba(34,230,214,0.22)";
  g.fillRect(0, 12, W, 8);
  g.fillRect(0, H - 20, W, 8);

  // Main broadcast lettering
  g.fillStyle = "#f2f7fd";
  g.font = "900 84px sans-serif";
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText("STTR · APEX CIRCUIT", W / 2, 106);

  // Sub-banner telemetry ticker
  g.fillStyle = "#26ecd8";
  g.font = "700 32px monospace";
  g.fillText("FIA GRADE 1  ·  400 HZ DETERMINISTIC TIME TRIAL", W / 2, 184);

  // Subtle LED matrix scanline micro-grid for photorealistic close-up
  g.fillStyle = "rgba(0,0,0,0.22)";
  for (let y = 0; y < H; y += 4) g.fillRect(0, y, W, 1);

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 12;
  return tex;
}

/** High-contrast 1024×128 FIA circuit sponsor barrier board strip. */
function sponsorBannerTexture() {
  const W = 1024, H = 128;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");

  const panels = [
    { bg: "#101620", accent: "#e02e26", txt: "#f3f6fb", label: "APEX MOTORSPORT", sub: "PRECISION SIM" },
    { bg: "#0c1c24", accent: "#20e4d2", txt: "#eff8fa", label: "400HZ TELEMETRY", sub: "LIVE DATA" },
    { bg: "#1b1418", accent: "#f0a824", txt: "#f6f4ef", label: "BOX3D PHYSICS", sub: "DETERMINISTIC" },
    { bg: "#101826", accent: "#4898ff", txt: "#f0f5fc", label: "ZERO-TRUST EDGE", sub: "VERIFIED LAPS" },
  ];
  const pw = W / panels.length;
  panels.forEach((p, idx) => {
    const x0 = idx * pw;
    g.fillStyle = p.bg;
    g.fillRect(x0, 0, pw, H);
    // Bold angled left accent stripe
    g.fillStyle = p.accent;
    g.beginPath();
    g.moveTo(x0, 0); g.lineTo(x0 + 22, 0); g.lineTo(x0 + 10, H); g.lineTo(x0, H);
    g.closePath(); g.fill();
    g.fillRect(x0, 0, pw, 6);
    g.fillRect(x0, H - 6, pw, 6);
    // Metallic panel seam divider
    g.fillStyle = "#3a434f";
    g.fillRect(x0 + pw - 3, 0, 3, H);

    g.fillStyle = p.txt;
    g.font = "900 24px sans-serif";
    g.textAlign = "center";
    g.textBaseline = "middle";
    g.fillText(p.label, x0 + pw * 0.53, 56);
    g.fillStyle = p.accent;
    g.font = "700 13px monospace";
    g.fillText(p.sub, x0 + pw * 0.53, 92);
  });

  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** High-detail 512×128 tiered stadium grandstand seats + organic crowd texture (smooth filtered). */
function stadiumCrowdTexture(seedVal) {
  const W = 512, H = 128;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  let s = (seedVal ^ 0x41c64e6d) >>> 0 || 1;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  // Concrete stadium bowl backdrop
  g.fillStyle = "#1d222b";
  g.fillRect(0, 0, W, H);

  const rows = 8;
  const rowH = H / rows;
  const jacketCols = [
    "#c9322b", "#2a528a", "#d6dce6", "#232a36", "#d8a828",
    "#20b8a8", "#8f3448", "#3a6e4d", "#e46b28", "#4d596b",
  ];
  const skinTones = ["#d9b296", "#c49474", "#8e6248", "#e5c6ae", "#a8795c"];

  for (let r = 0; r < rows; r++) {
    const yTop = r * rowH;
    // Concrete step riser & shadow
    g.fillStyle = "#2b313c";
    g.fillRect(0, yTop + rowH - 3, W, 3);
    g.fillStyle = "rgba(0,0,0,0.38)";
    g.fillRect(0, yTop, W, 3);

    // Seat backs + spectators across row
    for (let x = 2; x < W - 2; x += 6) {
      // Staircase aisle gaps every 128px
      const aisleDist = Math.min(x % 128, 128 - (x % 128));
      if (aisleDist < 7) {
        g.fillStyle = "#3a404a";
        g.fillRect(x, yTop + 2, 5, rowH - 4);
        continue;
      }
      // Molded stadium seat back (alternating team section blocks)
      const secCol = ((x / 128) | 0) % 2 === 0 ? "#24354e" : "#442228";
      g.fillStyle = secCol;
      g.fillRect(x, yTop + 3, 5, rowH - 6);

      // Occupied seat (~80% crowd density)
      if (rnd() < 0.80) {
        // Shoulders/torso
        g.fillStyle = jacketCols[(rnd() * jacketCols.length) | 0];
        if (typeof g.roundRect === "function") {
          g.beginPath();
          g.roundRect(x - 0.5, yTop + 6, 6, rowH - 7, 2);
          g.fill();
        } else {
          g.fillRect(x - 0.5, yTop + 6, 6, rowH - 7);
        }
        // Head / cap
        g.fillStyle = rnd() < 0.35
          ? jacketCols[(rnd() * jacketCols.length) | 0]
          : skinTones[(rnd() * skinTones.length) | 0];
        g.beginPath();
        g.arc(x + 2.5, yTop + 4.8, 2.0, 0, Math.PI * 2);
        g.fill();
      }
    }
  }

  // Subtle top under-canopy ambient occlusion gradient
  const ao = g.createLinearGradient(0, 0, 0, H);
  ao.addColorStop(0, "rgba(6,10,16,0.45)");
  ao.addColorStop(0.25, "rgba(6,10,16,0.08)");
  ao.addColorStop(1, "rgba(6,10,16,0.0)");
  g.fillStyle = ao;
  g.fillRect(0, 0, W, H);

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = THREE.RepeatWrapping;
  tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.magFilter = THREE.LinearFilter;
  tex.anisotropy = 8;
  return tex;
}

/** 512×256 architectural pit-building upper hospitality glass curtain-wall texture (albedo + emissiveMap). */
function pitGlassTexture() {
  const W = 512, H = 256;
  const cMap = document.createElement("canvas");
  cMap.width = W; cMap.height = H;
  const g = cMap.getContext("2d");

  const cEm = document.createElement("canvas");
  cEm.width = W; cEm.height = H;
  const ge = cEm.getContext("2d");

  // Dark tinted reflective architectural glass base
  const glassGrad = g.createLinearGradient(0, 0, 0, H);
  glassGrad.addColorStop(0, "#0c1624");
  glassGrad.addColorStop(0.55, "#142438");
  glassGrad.addColorStop(1, "#0a111c");
  g.fillStyle = glassGrad;
  g.fillRect(0, 0, W, H);

  ge.fillStyle = "#000000";
  ge.fillRect(0, 0, W, H);

  // Subtle warm hospitality suite interior ceiling light strips + lounges behind curtain mullions
  const bays = 8;
  const bw = W / bays;
  for (let b = 0; b < bays; b++) {
    const x0 = b * bw;
    // Interior recessed ceiling light band (warm architectural white)
    const lit = b !== 2 && b !== 6;
    if (lit) {
      const wg = g.createLinearGradient(0, 26, 0, 150);
      wg.addColorStop(0, "rgba(255,226,184,0.42)");
      wg.addColorStop(0.3, "rgba(255,214,162,0.16)");
      wg.addColorStop(1, "rgba(40,65,92,0.02)");
      g.fillStyle = wg;
      g.fillRect(x0 + 8, 22, bw - 16, 150);

      const eg = ge.createLinearGradient(0, 24, 0, 145);
      eg.addColorStop(0, "rgba(255,232,192,0.92)");
      eg.addColorStop(0.22, "rgba(255,210,152,0.42)");
      eg.addColorStop(1, "rgba(255,195,128,0.06)");
      ge.fillStyle = eg;
      ge.fillRect(x0 + 10, 22, bw - 20, 130);
    }
    // Subtle dark hospitality table silhouettes near floor
    g.fillStyle = "rgba(8,12,18,0.55)";
    g.fillRect(x0 + 14, H - 72, bw - 28, 48);

    // Anodized dark aluminum vertical mullions
    g.fillStyle = "#161c24";
    g.fillRect(x0, 0, 6, H);
    ge.fillStyle = "#000000";
    ge.fillRect(x0, 0, 6, H);
  }
  // Horizontal floor spandrel transom beams
  g.fillStyle = "#181e28";
  g.fillRect(0, 0, W, 16);
  g.fillRect(0, H - 22, W, 22);
  g.fillRect(0, H * 0.62, W, 7);
  ge.fillStyle = "#000000";
  ge.fillRect(0, 0, W, 16);
  ge.fillRect(0, H - 22, W, 22);
  ge.fillRect(0, H * 0.62, W, 7);

  const map = new THREE.CanvasTexture(cMap);
  map.wrapS = THREE.RepeatWrapping;
  map.colorSpace = THREE.SRGBColorSpace;
  map.anisotropy = 8;

  const emissiveMap = new THREE.CanvasTexture(cEm);
  emissiveMap.wrapS = THREE.RepeatWrapping;
  emissiveMap.colorSpace = THREE.SRGBColorSpace;
  emissiveMap.anisotropy = 8;

  return { map, emissiveMap };
}

/** 256×512 sleek vertical FIA Timing Tower LED leaderboard display texture. */
function timingTowerTexture() {
  const W = 256, H = 512;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");

  g.fillStyle = "#070b12";
  g.fillRect(0, 0, W, H);

  // Header badge
  g.fillStyle = "#d9261e";
  g.fillRect(10, 10, W - 20, 48);
  g.fillStyle = "#ffffff";
  g.font = "900 28px sans-serif";
  g.textAlign = "center";
  g.textBaseline = "middle";
  g.fillText("APEX  FIA", W / 2, 35);

  // Sector & lap status bar
  g.fillStyle = "#111b2a";
  g.fillRect(10, 66, W - 20, 34);
  g.fillStyle = "#22eedd";
  g.font = "700 18px monospace";
  g.fillText("400HZ · LIVE", W / 2, 84);

  // P1..P8 luminous timing rows
  const rows = [
    ["1", "VER", "1:18.4", "#c85eff"],
    ["2", "LEC", "+0.112", "#26ecd8"],
    ["3", "NOR", "+0.248", "#26ecd8"],
    ["4", "HAM", "+0.391", "#26ecd8"],
    ["5", "PIA", "+0.504", "#e8eef6"],
    ["6", "RUS", "+0.627", "#e8eef6"],
    ["7", "ALO", "+0.790", "#e8eef6"],
    ["8", "SAI", "+0.915", "#e8eef6"],
  ];
  rows.forEach(([pos, tag, gap, col], idx) => {
    const y = 112 + idx * 46;
    g.fillStyle = idx === 0 ? "#1a132c" : "#0e1522";
    g.fillRect(12, y, W - 24, 38);
    g.fillStyle = idx === 0 ? "#c85eff" : "#22e0d0";
    g.fillRect(12, y, 6, 38);
    g.fillStyle = "#f2f6fc";
    g.font = "900 22px monospace";
    g.textAlign = "left";
    g.fillText(pos, 26, y + 20);
    g.fillText(tag, 54, y + 20);
    g.fillStyle = col;
    g.textAlign = "right";
    g.font = "700 20px monospace";
    g.fillText(gap, W - 20, y + 20);
  });

  // Subtle LED matrix grid
  g.fillStyle = "rgba(0,0,0,0.24)";
  for (let y = 0; y < H; y += 4) g.fillRect(0, y, W, 1);

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** 256×256 procedural stadium floodlight LED diode array texture. */
function floodlightArrayTexture() {
  const W = 256, H = 128;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  g.fillStyle = "#141920";
  g.fillRect(0, 0, W, H);
  const cols = 8, rows = 4;
  const cw = (W - 16) / cols, ch = (H - 16) / rows;
  for (let r = 0; r < rows; r++) {
    for (let k = 0; k < cols; k++) {
      const cx = 8 + k * cw + cw * 0.5;
      const cy = 8 + r * ch + ch * 0.5;
      const rad = Math.min(cw, ch) * 0.42;
      const gr = g.createRadialGradient(cx, cy, 1, cx, cy, rad);
      gr.addColorStop(0, "#ffffff");
      gr.addColorStop(0.55, "#fff4d8");
      gr.addColorStop(1, "#4a4338");
      g.fillStyle = gr;
      g.beginPath();
      g.arc(cx, cy, rad, 0, Math.PI * 2);
      g.fill();
    }
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/** 128×128 FIA Digital LED Marshal Flag Panel dot-matrix display texture. */
function marshalLedTexture() {
  const N = 128;
  const c = document.createElement("canvas");
  c.width = N; c.height = N;
  const g = c.getContext("2d");
  g.fillStyle = "#080c10";
  g.fillRect(0, 0, N, N);

  // High-contrast LED matrix chevron & corner sector indicator dots
  const step = 8;
  for (let y = 8; y < N - 8; y += step) {
    for (let x = 8; x < N - 8; x += step) {
      const nx = (x - N / 2) / (N / 2);
      const ny = (y - N / 2) / (N / 2);
      const border = Math.abs(nx) > 0.72 || Math.abs(ny) > 0.72;
      const chev = Math.abs(Math.abs(ny) * 0.75 - (nx + 0.25)) < 0.22 && Math.abs(ny) < 0.55;
      const on = border || chev;
      g.fillStyle = on ? "#eafff8" : "#121a20";
      g.beginPath();
      g.arc(x + 4, y + 4, on ? 2.8 : 1.8, 0, Math.PI * 2);
      g.fill();
    }
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/** 256×256 procedural raked FIA gravel-trap pebble albedo + Sobel normal map. */
function gravelTrapPBRMaps() {
  const N = 256;
  const mask = N - 1;
  const height = new Float32Array(N * N);
  const cMap = document.createElement("canvas");
  cMap.width = cMap.height = N;
  const gMap = cMap.getContext("2d");
  const img = gMap.createImageData(N, N);
  const d = img.data;
  let s = 0x6c8e9a31;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  // Base longitudinal FIA raked gravel trap grooves (parallel to V across gravel width)
  for (let y = 0; y < N; y++) {
    // 10 crisp parallel rake furrows across each tile
    const rake = Math.sin((y / N) * Math.PI * 20) * 0.34;
    for (let x = 0; x < N; x++) {
      height[y * N + x] = 0.45 + rake + (rnd() - 0.5) * 0.22;
    }
  }
  // Scatter thousands of rounded river-stone gravel pebbles into heightfield
  const stoneTint = new Float32Array(N * N);
  for (let k = 0; k < 4600; k++) {
    const cx = (rnd() * N) | 0;
    const cy = (rnd() * N) | 0;
    const r = 1 + ((rnd() * 2.4) | 0);
    const hVal = 0.26 + rnd() * 0.24;
    const tint = (rnd() - 0.5) * 0.55;
    for (let dy = -r; dy <= r; dy++) {
      const py = ((cy + dy) & mask) * N;
      for (let dx = -r; dx <= r; dx++) {
        const d2 = (dx * dx + dy * dy) / ((r + 0.4) * (r + 0.4));
        if (d2 > 1) continue;
        const idx = py + ((cx + dx) & mask);
        height[idx] += hVal * (1 - d2);
        stoneTint[idx] = tint;
      }
    }
  }
  for (let y = 0; y < N; y++) {
    const rakeShade = 1 + Math.sin((y / N) * Math.PI * 20) * 0.12;
    for (let x = 0; x < N; x++) {
      const idx = y * N + x;
      const h = height[idx];
      const t = stoneTint[idx];
      const lum = (138 + (h - 0.5) * 58 + t * 44) * rakeShade;
      const o = idx * 4;
      d[o]     = Math.max(40, Math.min(230, Math.round(lum * 1.07)));
      d[o + 1] = Math.max(38, Math.min(220, Math.round(lum * 0.99)));
      d[o + 2] = Math.max(32, Math.min(205, Math.round(lum * 0.82)));
      d[o + 3] = 255;
    }
  }
  gMap.putImageData(img, 0, 0);
  const cNorm = sobelNormalCanvas(height, N, N, 1.45);
  const setup = (canvas, srgb) => {
    const t = new THREE.CanvasTexture(canvas);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.anisotropy = 8;
    if (srgb) t.colorSpace = THREE.SRGBColorSpace;
    return t;
  };
  return { map: setup(cMap, true), normalMap: setup(cNorm, false) };
}

/** 512×256 weathered precast FIA concrete pit/straight wall albedo + normal map. */
function concreteWallPBRMaps() {
  const W = 512, H = 256;
  const height = new Float32Array(W * H);
  const cMap = document.createElement("canvas");
  cMap.width = W; cMap.height = H;
  const g = cMap.getContext("2d");
  const img = g.createImageData(W, H);
  const d = img.data;
  let s = 0x89abcdef;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);

  for (let y = 0; y < H; y++) {
    const v = y / H;
    // Lower damp/dirt splash + bumper-height tire rub zone
    const dirtGrad = Math.pow(v, 2.2) * 26;
    const rubBand = Math.exp(-Math.pow((v - 0.58) / 0.14, 2));
    for (let x = 0; x < W; x++) {
      // Precast concrete panel vertical joints every 128px
      const joint = (x & 127) < 3 ? 1 : 0;
      const grain = (rnd() - 0.5) * 18;
      const streak = Math.sin(x * 0.08 + y * 0.015) * 6;
      height[y * W + x] = 0.5 + grain * 0.012 - joint * 0.35;
      let lum = 132 + grain + streak - dirtGrad - joint * 42;
      // Tire rub darkening along bumper height
      const rub = rubBand * Math.max(0, Math.sin(x * 0.045) * 0.5 + 0.35) * 24;
      lum -= rub;
      const o = (y * W + x) * 4;
      d[o]     = Math.max(24, Math.min(220, lum | 0));
      d[o + 1] = Math.max(25, Math.min(222, (lum + 2) | 0));
      d[o + 2] = Math.max(28, Math.min(226, (lum + 5) | 0));
      d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);

  // Crisp subtle FIA top safety stripe (red & white alternating)
  for (let k = 0; k < 8; k++) {
    g.fillStyle = k % 2 === 0 ? "rgba(205,40,34,0.82)" : "rgba(232,236,242,0.82)";
    g.fillRect(k * 64, 8, 64, 16);
  }
  const cNorm = sobelNormalCanvas(height, W, H, 1.2);
  const setup = (canvas, srgb) => {
    const t = new THREE.CanvasTexture(canvas);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.anisotropy = 8;
    if (srgb) t.colorSpace = THREE.SRGBColorSpace;
    return t;
  };
  return { map: setup(cMap, true), normalMap: setup(cNorm, false) };
}

/** 256×128 brushed galvanized steel texture for corrugated Armco guardrails. */
function galvanizedMetalTexture() {
  const W = 256, H = 128;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  const img = g.createImageData(W, H);
  const d = img.data;
  let s = 0x55aa55aa;
  const rnd = () => ((s = (Math.imul(s, 1664525) + 1013904223) >>> 0), s / 4294967296);
  for (let y = 0; y < H; y++) {
    const streak = (rnd() - 0.5) * 14;
    for (let x = 0; x < W; x++) {
      const spangle = ((x >> 2) + (y >> 2)) & 3;
      const n = (rnd() - 0.5) * 16 + streak + (spangle - 1.5) * 5;
      const v = Math.max(125, Math.min(235, Math.round(182 + n)));
      const o = (y * W + x) * 4;
      d[o] = v - 3; d[o + 1] = v; d[o + 2] = v + 6; d[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  // Subtle bolted lap seam every 128px
  g.fillStyle = "rgba(42,48,56,0.45)";
  g.fillRect(126, 0, 3, H);
  const tex = new THREE.CanvasTexture(c);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** Procedural crisp 512×352 FIA braking distance board ("150", "100", "50"). */
function brakeBoardTexture(meters, bars) {
  const W = 512, H = 352;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  g.fillStyle = "#f2f5f9";
  g.fillRect(0, 0, W, H);
  g.strokeStyle = "#161c24";
  g.lineWidth = 16;
  g.strokeRect(8, 8, W - 16, H - 16);
  // FIA red vertical countdown bars on left third
  g.fillStyle = "#d92b24";
  for (let k = 0; k < bars; k++) g.fillRect(36 + k * 48, 42, 28, H - 84);
  // Bold distance numeral
  g.fillStyle = "#0f141c";
  g.font = "900 168px sans-serif";
  g.textAlign = "right";
  g.textBaseline = "middle";
  g.fillText(String(meters), W - 34, H * 0.53);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** High-visibility 512×256 FIA yellow/black apex sightline chevron board. */
function chevronTexture(dirSign) {
  const W = 512, H = 256;
  const c = document.createElement("canvas");
  c.width = W; c.height = H;
  const g = c.getContext("2d");
  g.fillStyle = "#f2ca22";
  g.fillRect(0, 0, W, H);
  g.fillStyle = "#12161d";
  for (let k = -1; k < 4; k++) {
    const x0 = k * 144 + 24;
    g.beginPath();
    if (dirSign >= 0) {
      g.moveTo(x0, 0); g.lineTo(x0 + 72, 0); g.lineTo(x0 + 136, H / 2);
      g.lineTo(x0 + 72, H); g.lineTo(x0, H); g.lineTo(x0 + 64, H / 2);
    } else {
      g.moveTo(x0 + 136, 0); g.lineTo(x0 + 64, 0); g.lineTo(x0, H / 2);
      g.lineTo(x0 + 64, H); g.lineTo(x0 + 136, H); g.lineTo(x0 + 72, H / 2);
    }
    g.closePath();
    g.fill();
  }
  g.strokeStyle = "#12161d";
  g.lineWidth = 10;
  g.strokeRect(5, 5, W - 10, H - 10);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  return tex;
}

/** Sculpted unit-scale ribbed racing tire geometry (exact radius = 1.0, height = 1.0, Y in [-0.5, 0.5]). */
function buildRibbedTireGeo() {
  const pts = [
    new THREE.Vector2(0.0, -0.50),
    new THREE.Vector2(0.56, -0.50),
    new THREE.Vector2(0.56, -0.36),
    new THREE.Vector2(0.86, -0.50),
    new THREE.Vector2(0.97, -0.44),
    new THREE.Vector2(1.00, -0.32),
    new THREE.Vector2(0.96, -0.20),
    new THREE.Vector2(1.00, -0.08),
    new THREE.Vector2(0.96,  0.00),
    new THREE.Vector2(1.00,  0.08),
    new THREE.Vector2(0.96,  0.20),
    new THREE.Vector2(1.00,  0.32),
    new THREE.Vector2(0.97,  0.44),
    new THREE.Vector2(0.86,  0.50),
    new THREE.Vector2(0.56,  0.36),
    new THREE.Vector2(0.56,  0.50),
    new THREE.Vector2(0.0,   0.50),
  ];
  return new THREE.LatheGeometry(pts, 14);
}

/** Build a 5-tier sculpted alpine conifer crown with scalloped branch overhangs. */
function buildSculptedConiferCrownGeo() {
  const tiers = [
    { r: 1.72, h: 1.85, y: 1.78 },
    { r: 1.44, h: 1.75, y: 2.62 },
    { r: 1.15, h: 1.65, y: 3.42 },
    { r: 0.84, h: 1.50, y: 4.18 },
    { r: 0.50, h: 1.32, y: 4.88 },
  ];
  const geos = tiers.map((t, ti) => {
    const g = new THREE.ConeGeometry(t.r, t.h, 12, 3, false);
    const p = g.attributes.position;
    for (let i = 0; i < p.count; i++) {
      const vx = p.getX(i), vy = p.getY(i), vz = p.getZ(i);
      const rad = Math.hypot(vx, vz);
      if (rad > 0.05) {
        const ang = Math.atan2(vz, vx);
        const scallop = 1.0 + 0.16 * Math.sin(ang * 5 + ti * 1.6) + 0.09 * Math.cos(ang * 3 - ti * 1.1);
        const droop = vy < 0 ? -0.18 * (rad / t.r) * (0.5 + 0.5 * Math.sin(ang * 6 + ti)) : 0;
        p.setXYZ(i, vx * scallop, vy + droop, vz * scallop);
      }
    }
    g.translate(0, t.y, 0);
    g.computeVertexNormals();
    return g;
  });
  return mergeSimpleGeos(geos);
}

/** Build a smooth multi-lobe organic broadleaf woodland crown using displaced sub-2 icosahedra. */
function buildOrganicBroadleafCrownGeo() {
  const lobes = [
    { r: 1.34, x: 0.0,   y: 3.18, z: 0.0 },
    { r: 1.02, x: 0.80,  y: 2.66, z: 0.44 },
    { r: 0.96, x: -0.74, y: 2.74, z: -0.38 },
    { r: 0.88, x: -0.36, y: 2.98, z: 0.74 },
    { r: 0.86, x: 0.46,  y: 3.48, z: -0.56 },
    { r: 0.76, x: -0.22, y: 3.64, z: 0.18 },
  ];
  const geos = lobes.map((lb, li) => {
    const g = new THREE.IcosahedronGeometry(lb.r, 2);
    const p = g.attributes.position;
    for (let i = 0; i < p.count; i++) {
      const vx = p.getX(i), vy = p.getY(i), vz = p.getZ(i);
      const bump = 1.0
        + 0.13 * Math.sin(vx * 3.4 + li * 2.1) * Math.cos(vz * 3.1 - li)
        + 0.08 * Math.sin(vy * 4.2 + vx * 2.3)
        + 0.05 * Math.cos((vx + vz) * 5.2 + li);
      p.setXYZ(i, vx * bump + lb.x, vy * bump * 0.90 + lb.y, vz * bump + lb.z);
    }
    g.computeVertexNormals();
    return g;
  });
  return mergeSimpleGeos(geos);
}

// ---------------------------------------------------------------------------
// Main Ambience Builder
// ---------------------------------------------------------------------------

/** @returns THREE.Group (userData.glowMaterials, userData.setStartLights, userData.updateAmbience). */
export function buildAmbience(track, hash = null) {
  const group = new THREE.Group();
  const glowMaterials = [];
  group.userData.glowMaterials = glowMaterials;
  const n = track.S;
  const props = track.props && track.props.length ? track.props : null;

  let seed = (Number((hash ?? fnv1a64(track.bytes)) & 0xffffffffn) >>> 0) || 1;
  const rand = () => (seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 2 ** 32;

  const upY = new THREE.Vector3(0, 1, 0);
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

  /** Right-handed building basis: +X along straight, +Y up, +Z away from track (-Z faces track). */
  const buildingBasis = (th, sh, out) => {
    const fwdOut = sh.clone().multiplyScalar(out).normalize();
    const rightAlong = new THREE.Vector3().crossVectors(upY, fwdOut).normalize();
    const q = new THREE.Quaternion().setFromRotationMatrix(
      new THREE.Matrix4().makeBasis(rightAlong, upY, fwdOut),
    );
    return { rightAlong, fwdOut, q };
  };

  // ------------------------------------------------------------- ground plain + 2-ring alpine horizon
  let minY = Infinity, maxCY = -Infinity, cx = 0, cz = 0, maxR = 0;
  for (const c of track.center) {
    if (c[1] < minY) minY = c[1];
    if (c[1] > maxCY) maxCY = c[1];
    cx += c[0] / n;
    cz += c[2] / n;
  }
  for (const c of track.center) maxR = Math.max(maxR, Math.hypot(c[0] - cx, c[2] - cz));
  // CONTRACTS §8 exact ground rule — MUST stay minY - 1.35!
  const groundY = minY - 1.35;

  const groundTex = grassTexture();
  groundTex.repeat.set(42, 42);
  const groundNorm = grassNormalMap();
  groundNorm.repeat.set(42, 42);
  const groundGeo = new THREE.CircleGeometry(maxR + 460, 64);
  groundGeo.rotateX(-Math.PI / 2);
  const ground = new THREE.Mesh(
    groundGeo,
    new THREE.MeshStandardMaterial({
      color: 0x2b3e2e,
      map: groundTex,
      normalMap: groundNorm,
      normalScale: new THREE.Vector2(0.45, 0.45),
      roughness: 0.95,
      metalness: 0.0,
      envMapIntensity: 0.35,
    }),
  );
  ground.position.set(cx, groundY - 0.02, cz);
  ground.receiveShadow = true;
  group.add(ground);

  // ---- Layered Two-Ring Alpine Horizon (foreground wooded foothills + craggy snow-dusted peaks)
  {
    const buildMountainRing = ({
      segs, rIn, rMid, rPeak, rOut, hBase, hAmp,
      colValley, colSlope, colRock, colSnow, snowThresh, phases, craggy,
    }) => {
      const stations = 4; // in -> mid -> peak -> out
      const mPos = new Float32Array(segs * stations * 3);
      const mCol = new Float32Array(segs * stations * 3);
      const mIdx = [];
      const tmp = new THREE.Color();
      const [p1, p2, p3, p4] = phases;
      for (let k = 0; k < segs; k++) {
        const a = (k / segs) * Math.PI * 2;
        const ca = Math.cos(a), sa = Math.sin(a);
        let ridge = hBase
          + hAmp * 0.46 * Math.sin(3 * a + p1)
          + hAmp * 0.30 * Math.sin(7 * a + p2)
          + hAmp * 0.16 * Math.cos(13 * a + p3)
          + hAmp * 0.10 * Math.sin(23 * a + p4);
        if (craggy) {
          // Sharp ridged alpine silhouette peaks
          ridge += hAmp * 0.24 * Math.pow(Math.abs(Math.sin(5 * a + p1 * 0.7)), 1.6);
          ridge += hAmp * 0.14 * Math.abs(Math.cos(17 * a + p2 * 1.3));
        }
        const hPeak = Math.max(10, ridge);
        const hMid = hPeak * (0.48 + 0.12 * Math.sin(11 * a + p3));
        const rJitter = 1.0 + 0.06 * Math.sin(9 * a + p2);
        const b0 = k * stations * 3;
        mPos.set([cx + ca * rIn, groundY - 2.0, cz + sa * rIn], b0);
        mPos.set([cx + ca * rMid * rJitter, groundY + hMid, cz + sa * rMid * rJitter], b0 + 3);
        mPos.set([cx + ca * rPeak * rJitter, groundY + hPeak, cz + sa * rPeak * rJitter], b0 + 6);
        mPos.set([cx + ca * rOut, groundY - 8.0, cz + sa * rOut], b0 + 9);

        // Valley base color
        mCol.set([colValley.r, colValley.g, colValley.b], b0);
        // Mid slope (timberline -> slate rock)
        const midRock = Math.min(1, Math.max(0, (hMid - 24) / 36));
        tmp.copy(colSlope).lerp(colRock, midRock);
        mCol.set([tmp.r, tmp.g, tmp.b], b0 + 3);
        // Peak crest (rock -> sunlit snow-dusted alpine cap)
        const peakRock = Math.min(1, Math.max(0, (hPeak - 22) / 34));
        const snowMix = snowThresh > 0 ? Math.min(1, Math.max(0, (hPeak - snowThresh) / 28)) : 0;
        tmp.copy(colSlope).lerp(colRock, peakRock).lerp(colSnow, snowMix * 0.85);
        mCol.set([tmp.r, tmp.g, tmp.b], b0 + 6);
        // Far back slope
        mCol.set([colValley.r * 0.72, colValley.g * 0.76, colValley.b * 0.86], b0 + 9);

        const nk = (k + 1) % segs;
        for (let st = 0; st < stations - 1; st++) {
          const i0 = k * stations + st;
          const i1 = nk * stations + st;
          mIdx.push(i0, i1, i0 + 1, i0 + 1, i1, i1 + 1);
        }
      }
      const mg = new THREE.BufferGeometry();
      mg.setAttribute("position", new THREE.BufferAttribute(mPos, 3));
      mg.setAttribute("color", new THREE.BufferAttribute(mCol, 3));
      mg.setIndex(mIdx);
      mg.computeVertexNormals();
      return new THREE.Mesh(mg, new THREE.MeshStandardMaterial({
        vertexColors: true, roughness: 0.96, metalness: 0.0, envMapIntensity: 0.25,
      }));
    };

    // Ring 1: Closer forested foothills
    group.add(buildMountainRing({
      segs: 84,
      rIn: maxR + 100,
      rMid: maxR + 145,
      rPeak: maxR + 190,
      rOut: maxR + 265,
      hBase: 22,
      hAmp: 19,
      colValley: new THREE.Color(0x233629),
      colSlope: new THREE.Color(0x2c4232),
      colRock: new THREE.Color(0x404d53),
      colSnow: new THREE.Color(0x8896a2),
      snowThresh: 0,
      phases: [rand() * 6.28, rand() * 6.28, rand() * 6.28, rand() * 6.28],
      craggy: false,
    }));

    // Ring 2: Tall craggy snow-dusted alpine mountain range behind
    group.add(buildMountainRing({
      segs: 104,
      rIn: maxR + 185,
      rMid: maxR + 255,
      rPeak: maxR + 325,
      rOut: maxR + 445,
      hBase: 54,
      hAmp: 48,
      colValley: new THREE.Color(0x283842),
      colSlope: new THREE.Color(0x3a4a58),
      colRock: new THREE.Color(0x526170),
      colSnow: new THREE.Color(0xc8d4e0),
      snowThresh: 66,
      phases: [rand() * 6.28, rand() * 6.28, rand() * 6.28, rand() * 6.28],
      craggy: true,
    }));
  }

  // Skirts (v1 ONLY — v2 uses terrain aprons per TERRAIN.md)
  if (!props) {
    const skirtMat = new THREE.MeshStandardMaterial({
      color: 0x2d3b31, roughness: 0.94, metalness: 0.0, side: THREE.DoubleSide,
    });
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

  // ---------------------------------------------- fast XZ-binned terrain height sampling (v2)
  const APRON_COS = Math.cos((20 * Math.PI) / 180);
  const groundHit = (() => {
    if (!props) return () => null;
    const verts = track.verts, tris = track.tris;
    const triCount = tris.length / 3;
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (let i = 0; i < verts.length; i += 3) {
      const vx = verts[i], vz = verts[i + 2];
      if (vx < minX) minX = vx;
      if (vx > maxX) maxX = vx;
      if (vz < minZ) minZ = vz;
      if (vz > maxZ) maxZ = vz;
    }
    const GRID = 32;
    const invSpanX = GRID / Math.max(1e-3, maxX - minX);
    const invSpanZ = GRID / Math.max(1e-3, maxZ - minZ);
    const cells = Array.from({ length: GRID * GRID }, () => []);
    for (let t = 0; t < triCount; t++) {
      const i0 = tris[t * 3] * 3, i1 = tris[t * 3 + 1] * 3, i2 = tris[t * 3 + 2] * 3;
      const ax = verts[i0], az = verts[i0 + 2];
      const bx = verts[i1], bz = verts[i1 + 2];
      const cx2 = verts[i2], cz2 = verts[i2 + 2];
      const x0 = Math.max(0, Math.min(GRID - 1, Math.floor((Math.min(ax, bx, cx2) - minX) * invSpanX)));
      const x1 = Math.max(0, Math.min(GRID - 1, Math.floor((Math.max(ax, bx, cx2) - minX) * invSpanX)));
      const z0 = Math.max(0, Math.min(GRID - 1, Math.floor((Math.min(az, bz, cz2) - minZ) * invSpanZ)));
      const z1 = Math.max(0, Math.min(GRID - 1, Math.floor((Math.max(az, bz, cz2) - minZ) * invSpanZ)));
      for (let gz = z0; gz <= z1; gz++) {
        const row = gz * GRID;
        for (let gx = x0; gx <= x1; gx++) cells[row + gx].push(t);
      }
    }
    return (x, z) => {
      const gx = Math.floor((x - minX) * invSpanX);
      const gz = Math.floor((z - minZ) * invSpanZ);
      if (gx < 0 || gx >= GRID || gz < 0 || gz >= GRID) return null;
      const list = cells[gz * GRID + gx];
      let bestY = -Infinity, bestNy = 1;
      for (let k = 0; k < list.length; k++) {
        const t = list[k];
        const i0 = tris[t * 3] * 3, i1 = tris[t * 3 + 1] * 3, i2 = tris[t * 3 + 2] * 3;
        const ax = verts[i0], ay = verts[i0 + 1], az = verts[i0 + 2];
        const bx = verts[i1], by = verts[i1 + 1], bz = verts[i1 + 2];
        const cx2 = verts[i2], cy2 = verts[i2 + 1], cz2 = verts[i2 + 2];
        const e1x = bx - ax, e1y = by - ay, e1z = bz - az;
        const e2x = cx2 - ax, e2y = cy2 - ay, e2z = cz2 - az;
        const det = e1x * e2z - e1z * e2x;
        if (Math.abs(det) < 1e-9) continue;
        const dx = x - ax, dz = z - az;
        const u = (dx * e2z - dz * e2x) / det;
        if (u < -1e-6 || u > 1 + 1e-6) continue;
        const v = (e1x * dz - e1z * dx) / det;
        if (v < -1e-6 || u + v > 1 + 1e-6) continue;
        const y = ay + u * e1y + v * e2y;
        if (y > bestY) {
          bestY = y;
          const nx = e1y * e2z - e1z * e2y;
          const ny = e1z * e2x - e1x * e2z;
          const nz = e1x * e2y - e1y * e2x;
          bestNy = Math.abs(ny) / (Math.hypot(nx, ny, nz) || 1);
        }
      }
      return bestY > -Infinity ? { point: { y: bestY }, face: { normal: { y: bestNy } } } : null;
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

  const clearOfTrack = (p, extraMargin = 7.0) => {
    for (let i = 0; i < n; i += 2) {
      const c = track.center[i];
      const dx = p.x - c[0], dz = p.z - c[2];
      const r = track.width[i] * 0.5 + extraMargin;
      if (dx * dx + dz * dz < r * r) return false;
    }
    return true;
  };

  const clearOfProps = (p, margin = 2.0) => {
    if (!props) return true;
    for (let i = 0; i < props.length; i++) {
      const pr = props[i];
      const dx = p.x - pr.pos[0], dz = p.z - pr.pos[2];
      const r = Math.hypot(pr.half[0], pr.half[2]) + margin;
      if (dx * dx + dz * dz < r * r) return false;
    }
    return true;
  };

  const buildingFootprints = [];
  const clearOfBuildings = (p, margin = 0) =>
    !buildingFootprints.some((fp) => {
      const dx = p.x - fp.x, dz = p.z - fp.z;
      const rr = fp.r + margin;
      return dx * dx + dz * dz < rr * rr;
    });

  // Shared architectural PBR materials
  const archSteelMat = new THREE.MeshStandardMaterial({
    color: 0x1e242e, roughness: 0.36, metalness: 0.78, envMapIntensity: 1.2,
  });
  const archSteelLightMat = new THREE.MeshStandardMaterial({
    color: 0x5c6674, roughness: 0.44, metalness: 0.62, envMapIntensity: 1.0,
  });
  const archConcreteMat = new THREE.MeshStandardMaterial({
    color: 0x565e68, roughness: 0.86, metalness: 0.05,
  });
  const darkRoofMat = new THREE.MeshStandardMaterial({
    color: 0x151a22, roughness: 0.42, metalness: 0.55, envMapIntensity: 1.1,
  });
  const sponsorTex = sponsorBannerTexture();
  const sponsorMat = new THREE.MeshStandardMaterial({
    map: sponsorTex, roughness: 0.48, metalness: 0.08, side: THREE.DoubleSide,
  });

  // ------------------------------------------- Modern Stadium Grandstands + Sponsor Boards
  let mainStandOut = 1;
  let mainStraightLen = 40;
  {
    const STEP_H = 0.58, STEP_D = 1.32;
    const standDefs = [];
    const tryStand = (i0, out, L, tiers, dist, isMain = false) => {
      const f = frame(i0);
      const th = f.t.clone().setY(0).normalize();
      const sh = new THREE.Vector3().crossVectors(upY, th).normalize();
      const base = f.c.clone().addScaledVector(sh, out * dist);
      base.y = groundY;
      const depth = tiers * STEP_D + 3.2;
      for (const [dl, dd] of [
        [0, 0], [-L / 2, 0], [L / 2, 0],
        [-L / 2, depth], [L / 2, depth], [0, depth],
      ]) {
        const p = base.clone().addScaledVector(th, dl).addScaledVector(sh, out * dd);
        if (!clearOfTrack(p, 7.2)) return false;
      }
      const hit = groundHit(base.x, base.z);
      if (hit) {
        if (Math.abs(hit.face.normal.y) < APRON_COS) return false;
        base.y = hit.point.y;
      }
      standDefs.push({ base, th, sh, out, L, tiers, isMain });
      buildingFootprints.push({
        x: base.x + sh.x * out * depth * 0.5,
        z: base.z + sh.z * out * depth * 0.5,
        r: Math.hypot(L * 0.5, depth * 0.5) + 2.8,
      });
      return true;
    };

    let run = 1;
    while (run < 16 && Math.abs(kappa[run % n]) < 0.008 && Math.abs(kappa[(n - run) % n]) < 0.008) run++;
    mainStraightLen = Math.max(32, Math.min(72, run * 2 * 2.5 * 0.8));
    const d0 = track.width[0] * 0.5 + 11.5;
    if (tryStand(0, 1, mainStraightLen, 6, d0, true)) {
      mainStandOut = 1;
    } else if (tryStand(0, -1, mainStraightLen, 6, d0, true)) {
      mainStandOut = -1;
    }
    let small = 0;
    for (const z of zones) {
      if (small >= 3) break;
      if (z.len < 4) continue;
      if (tryStand(z.peakIdx, -insideSign(z.peakIdx), 18, 4, track.width[z.peakIdx] * 0.5 + 15.5, false)) {
        small++;
      }
    }

    if (standDefs.length) {
      const standLedMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        color: 0x161b22,
        emissive: 0xfff1d6,
        emissiveIntensity: 0.72,
        roughness: 0.25,
        metalness: 0.1,
      }), 0.72);
      const glassRailMat = new THREE.MeshPhysicalMaterial({
        color: 0x8bb6d8,
        roughness: 0.12,
        metalness: 0.15,
        transmission: 0.0,
        transparent: true,
        opacity: 0.36,
        clearcoat: 1.0,
        envMapIntensity: 1.6,
        side: THREE.DoubleSide,
      });
      const crowdTex = stadiumCrowdTexture(seed);
      const crowdMat = new THREE.MeshStandardMaterial({
        map: crowdTex, roughness: 0.88, metalness: 0.0, side: THREE.DoubleSide,
      });
      const cableMat = new THREE.MeshStandardMaterial({
        color: 0x9aa6b4, roughness: 0.35, metalness: 0.85,
      });

      const boxGeo = new THREE.BoxGeometry(1, 1, 1);

      for (const sd of standDefs) {
        const { base, th, sh, out, L, tiers, isMain } = sd;
        const { q } = buildingBasis(th, sh, out);
        const sg = new THREE.Group();
        sg.position.copy(base);
        sg.quaternion.copy(q);

        const depth = tiers * STEP_D;
        const topTierY = tiers * STEP_H;

        // Ground-hugging foundation plinth (extends 1.8m below base.y to seal any slight slope)
        const plinth = new THREE.Mesh(boxGeo, archConcreteMat);
        plinth.scale.set(L + 0.6, 1.8, depth + 1.8);
        plinth.position.set(0, -0.88, depth * 0.5 + 0.3);
        sg.add(plinth);

        // Stepped concrete seating tiers (local +Z goes deeper away from track)
        for (let k = 0; k < tiers; k++) {
          const h = (k + 1) * STEP_H;
          const step = new THREE.Mesh(boxGeo, archConcreteMat);
          step.scale.set(L, h, STEP_D + 0.06);
          step.position.set(0, h * 0.5, k * STEP_D + STEP_D * 0.5);
          sg.add(step);
        }

        // Solid back concourse wall + architectural side wing-walls
        const backH = topTierY + 2.3;
        const backWall = new THREE.Mesh(boxGeo, archConcreteMat);
        backWall.scale.set(L + 0.4, backH, 0.55);
        backWall.position.set(0, backH * 0.5, depth + 0.3);
        sg.add(backWall);

        for (const sx of [-L * 0.5 - 0.16, L * 0.5 + 0.16]) {
          const sideW = new THREE.Mesh(boxGeo, archSteelMat);
          sideW.scale.set(0.32, backH * 0.82, depth + 0.8);
          sideW.position.set(sx, backH * 0.41, depth * 0.5 + 0.1);
          sg.add(sideW);
        }

        // Front terrace wall + translucent glass safety balustrade + brushed steel top rail
        const frontWall = new THREE.Mesh(boxGeo, archConcreteMat);
        frontWall.scale.set(L + 0.2, 0.75, 0.28);
        frontWall.position.set(0, 0.375, -0.14);
        sg.add(frontWall);

        const glassRail = new THREE.Mesh(boxGeo, glassRailMat);
        glassRail.scale.set(L, 0.85, 0.05);
        glassRail.position.set(0, 1.18, -0.14);
        sg.add(glassRail);

        const handRail = new THREE.Mesh(boxGeo, archSteelLightMat);
        handRail.scale.set(L + 0.1, 0.07, 0.09);
        handRail.position.set(0, 1.62, -0.14);
        sg.add(handRail);

        // Circuit sponsor barrier board along front fascia (-Z facing track)
        const sGeo = new THREE.PlaneGeometry(L, 0.88);
        const sUv = sGeo.attributes.uv;
        const sRep = Math.max(1, Math.round(L / 16));
        for (let u = 0; u < sUv.count; u++) sUv.setX(u, sUv.getX(u) * sRep);
        const sBoard = new THREE.Mesh(sGeo, sponsorMat);
        sBoard.position.set(0, 0.46, -0.30);
        sBoard.rotation.y = Math.PI; // face -Z toward track
        sg.add(sBoard);

        // Angled cantilevered roof canopy (raked upward ~9° toward track for F1 stadium look)
        const roofTilt = 0.15; // tilts front edge (-Z) upward!
        const roofH = topTierY + 3.55;
        const roofDepth = depth + 3.2;
        const roofGroup = new THREE.Group();
        roofGroup.position.set(0, roofH, depth * 0.48);
        roofGroup.rotation.x = roofTilt;

        const roofDeck = new THREE.Mesh(boxGeo, darkRoofMat);
        roofDeck.scale.set(L + 1.8, 0.24, roofDepth);
        roofGroup.add(roofDeck);

        // Sleek bright fascia lip + recessed under-roof stadium LED light bar
        const fascia = new THREE.Mesh(boxGeo, archSteelLightMat);
        fascia.scale.set(L + 1.9, 0.32, 0.16);
        fascia.position.set(0, 0.04, -roofDepth * 0.5);
        roofGroup.add(fascia);

        const ledBar = new THREE.Mesh(boxGeo, standLedMat);
        ledBar.scale.set(L + 0.4, 0.10, 0.22);
        ledBar.position.set(0, -0.15, -roofDepth * 0.5 + 0.65);
        roofGroup.add(ledBar);
        sg.add(roofGroup);

        // Rear raked steel tension-mast pylons + diagonal tension-stay cables supporting cantilever
        const bays = Math.max(2, Math.round(L / 10));
        const mastTopY = roofH + 3.4;
        const mastZ = depth + 0.95;
        const frontAttachZ = -0.9;
        const frontAttachY = roofH + Math.sin(roofTilt) * (roofDepth * 0.5) + 0.18;

        const fwdZ = new THREE.Vector3(0, 0, 1);
        for (let b = 0; b <= bays; b++) {
          const mx = -L * 0.5 + (b / bays) * L;
          // Rear steel column rising above canopy
          const mast = new THREE.Mesh(boxGeo, archSteelMat);
          mast.scale.set(0.32, mastTopY, 0.36);
          mast.position.set(mx, mastTopY * 0.5, mastZ);
          sg.add(mast);

          // Diagonal tension stay cable from mast top to cantilevered roof nose
          const pTop = new THREE.Vector3(mx, mastTopY - 0.15, mastZ - 0.1);
          const pFront = new THREE.Vector3(mx, frontAttachY, frontAttachZ);
          const span = pFront.clone().sub(pTop);
          const cLen = span.length();
          const cable = new THREE.Mesh(boxGeo, cableMat);
          cable.scale.set(0.055, 0.055, cLen);
          cable.position.copy(pTop).addScaledVector(span, 0.5);
          cable.quaternion.setFromUnitVectors(fwdZ, span.clone().normalize());
          sg.add(cable);

          // Under-deck steel cantilever rib beam
          const rib = new THREE.Mesh(boxGeo, archSteelMat);
          rib.scale.set(0.20, 0.36, roofDepth - 0.5);
          rib.position.set(mx, roofH - 0.24, depth * 0.48);
          rib.rotation.x = roofTilt;
          sg.add(rib);
        }

        // High-res tiered stadium seating & crowd plane draped across steps
        const cPos = new Float32Array([
          -L * 0.5, STEP_H + 0.22, 0.24,
           L * 0.5, STEP_H + 0.22, 0.24,
          -L * 0.5, topTierY + 0.26, (tiers - 1) * STEP_D + 0.95,
           L * 0.5, topTierY + 0.26, (tiers - 1) * STEP_D + 0.95,
        ]);
        const rep = Math.max(1, L / 18);
        const cUv = new Float32Array([0, 0, rep, 0, 0, 1, rep, 1]);
        const cg = new THREE.BufferGeometry();
        cg.setAttribute("position", new THREE.BufferAttribute(cPos, 3));
        cg.setAttribute("uv", new THREE.BufferAttribute(cUv, 2));
        cg.setIndex([0, 2, 1, 1, 2, 3]);
        cg.computeVertexNormals();
        sg.add(new THREE.Mesh(cg, crowdMat));

        // Optional FIA rooftop signage crest on main grandstand
        if (isMain) {
          const crest = new THREE.Mesh(boxGeo, archSteelMat);
          crest.scale.set(Math.min(18, L * 0.45), 1.05, 0.28);
          crest.position.set(0, frontAttachY + 0.62, -1.1);
          sg.add(crest);
        }

        group.add(sg);
      }
    }
  }

  // ------------------------------------------- Architectural Pit Complex + FIA Timing Tower
  const timingPulseMats = [];
  {
    const pitOut = -mainStandOut;
    const f0 = frame(0);
    const th0 = f0.t.clone().setY(0).normalize();
    const sh0 = new THREE.Vector3().crossVectors(upY, th0).normalize();
    const pitL = Math.max(30, Math.min(62, mainStraightLen * 0.92));
    const pitDepth = 12.5;
    const pitDist = track.width[0] * 0.5 + 11.5;

    const tryPit = (sideOut) => {
      const base = f0.c.clone().addScaledVector(sh0, sideOut * pitDist);
      base.y = groundY;
      for (const [dl, dd] of [
        [0, 0], [-pitL / 2, 0], [pitL / 2, 0],
        [-pitL / 2, pitDepth], [pitL / 2, pitDepth], [0, pitDepth],
      ]) {
        const p = base.clone().addScaledVector(th0, dl).addScaledVector(sh0, sideOut * dd);
        if (!clearOfTrack(p, 7.2) || !clearOfBuildings(p, 2.0)) return null;
      }
      const hit = groundHit(base.x, base.z);
      if (hit) {
        if (Math.abs(hit.face.normal.y) < APRON_COS) return null;
        base.y = hit.point.y;
      }
      return { base, sideOut };
    };

    const pitPlacement = tryPit(pitOut) || tryPit(-pitOut);
    if (pitPlacement) {
      const { base, sideOut } = pitPlacement;
      const { q } = buildingBasis(th0, sh0, sideOut);
      buildingFootprints.push({
        x: base.x + sh0.x * sideOut * pitDepth * 0.5,
        z: base.z + sh0.z * sideOut * pitDepth * 0.5,
        r: Math.hypot(pitL * 0.5, pitDepth * 0.5) + 4.5,
      });

      const pg = new THREE.Group();
      pg.position.copy(base);
      pg.quaternion.copy(q);

      const boxGeo = new THREE.BoxGeometry(1, 1, 1);
      const garageH = 4.2;
      const hospH = 3.6;
      const totalH = garageH + hospH;

      // Ground-hugging concrete pit-apron foundation slab (extends below slope)
      const slab = new THREE.Mesh(boxGeo, archConcreteMat);
      slab.scale.set(pitL + 6.0, 2.0, pitDepth + 4.0);
      slab.position.set(0, -0.98, pitDepth * 0.5 - 0.5);
      pg.add(slab);

      // Lower Garage Level: recessed dark interior + glowing warm overhead garage LED bays
      const garageBack = new THREE.Mesh(boxGeo, new THREE.MeshStandardMaterial({
        color: 0x161a20, roughness: 0.72, metalness: 0.15,
      }));
      garageBack.scale.set(pitL - 0.6, garageH, pitDepth - 2.6);
      garageBack.position.set(0, garageH * 0.5, pitDepth * 0.5 + 1.1);
      pg.add(garageBack);

      const garageLedMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        color: 0x1c1e22,
        emissive: 0xfff0d2,
        emissiveIntensity: 0.92,
        roughness: 0.25,
      }), 0.92);

      const bayCount = Math.max(4, Math.min(8, Math.round(pitL / 7.5)));
      const bayW = pitL / bayCount;
      for (let b = 0; b <= bayCount; b++) {
        const bx = -pitL * 0.5 + b * bayW;
        // Heavy architectural charcoal steel bay column
        const col = new THREE.Mesh(boxGeo, archSteelMat);
        col.scale.set(0.62, garageH + 0.2, 1.8);
        col.position.set(bx, garageH * 0.5, 0.9);
        pg.add(col);

        if (b < bayCount) {
          const midX = bx + bayW * 0.5;
          // Recessed overhead garage bay LED light panel illuminating each team box
          const bayLight = new THREE.Mesh(boxGeo, garageLedMat);
          bayLight.scale.set(bayW - 1.4, 0.12, 1.35);
          bayLight.position.set(midX, garageH - 0.24, 1.15);
          pg.add(bayLight);
          // Subtle back-wall team diagnostic accent light bar
          const bayBackGlow = new THREE.Mesh(boxGeo, garageLedMat);
          bayBackGlow.scale.set(bayW - 2.0, 0.55, 0.12);
          bayBackGlow.position.set(midX, garageH * 0.62, 2.28);
          pg.add(bayBackGlow);
        }
      }

      // First-floor mezzanine fascia deck + sponsor strip along pit lane
      const mezDeck = new THREE.Mesh(boxGeo, archSteelMat);
      mezDeck.scale.set(pitL + 1.2, 0.62, pitDepth + 0.8);
      mezDeck.position.set(0, garageH + 0.31, pitDepth * 0.5);
      pg.add(mezDeck);

      // Upper Hospitality / Race Control Floor: dark reflective glass curtain wall
      const pitGlass = pitGlassTexture();
      const glassRep = Math.max(1, Math.round(pitL / 18));
      pitGlass.map.repeat.set(glassRep, 1);
      pitGlass.emissiveMap.repeat.set(glassRep, 1);
      const hospGlassMat = registerGlow(glowMaterials, new THREE.MeshPhysicalMaterial({
        map: pitGlass.map,
        emissiveMap: pitGlass.emissiveMap,
        emissive: 0xffe4bc,
        emissiveIntensity: 0.52,
        roughness: 0.10,
        metalness: 0.24,
        clearcoat: 1.0,
        clearcoatRoughness: 0.08,
        envMapIntensity: 1.85,
      }), 0.52);

      const hospCore = new THREE.Mesh(boxGeo, [
        archSteelMat, archSteelMat, darkRoofMat, archSteelMat, archSteelMat, hospGlassMat,
      ]);
      hospCore.scale.set(pitL - 0.4, hospH, pitDepth - 1.2);
      hospCore.position.set(0, garageH + 0.62 + hospH * 0.5, pitDepth * 0.5 + 0.4);
      pg.add(hospCore);

      // Cantilevered Glass Race-Control Overhang Box aligned near Start/Finish line
      const rcBox = new THREE.Mesh(boxGeo, [
        hospGlassMat, hospGlassMat, darkRoofMat, archSteelMat, archSteelMat, hospGlassMat,
      ]);
      rcBox.scale.set(9.2, 2.95, 3.2);
      rcBox.position.set(0, garageH + 2.35, -1.15);
      pg.add(rcBox);

      // Sweeping cantilevered aerodynamic roof canopy with thin glowing cyan LED architectural edge
      const canopyY = totalH + 0.82;
      const canopy = new THREE.Mesh(boxGeo, darkRoofMat);
      canopy.scale.set(pitL + 4.4, 0.36, pitDepth + 4.8);
      canopy.position.set(0, canopyY, pitDepth * 0.5 - 1.4);
      pg.add(canopy);

      const canopyAccentMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        color: 0x0c2428,
        emissive: 0x22ebd8,
        emissiveIntensity: 0.88,
        roughness: 0.25,
      }), 0.88);
      const canopyLed = new THREE.Mesh(boxGeo, canopyAccentMat);
      canopyLed.scale.set(pitL + 4.44, 0.12, 0.12);
      canopyLed.position.set(0, canopyY - 0.04, -2.62);
      pg.add(canopyLed);

      // Low pit-wall sponsor barrier in front of garage apron (set safely back from road)
      const pwGeo = new THREE.PlaneGeometry(pitL, 0.92);
      const pwUv = pwGeo.attributes.uv;
      const pwRep = Math.max(1, Math.round(pitL / 16));
      for (let u = 0; u < pwUv.count; u++) pwUv.setX(u, pwUv.getX(u) * pwRep);
      const pwBoard = new THREE.Mesh(pwGeo, sponsorMat);
      pwBoard.position.set(0, 0.52, -2.25);
      pwBoard.rotation.y = Math.PI;
      pg.add(pwBoard);

      // ---- Sleek Vertical FIA Timing Tower Pylon next to the pit complex (track-clearance verified)
      const { rightAlong, fwdOut } = buildingBasis(th0, sh0, sideOut);
      let towerX = pitL * 0.5 + 3.0;
      let towerZ = 1.6;
      const towerWorldA = base.clone().addScaledVector(rightAlong, towerX).addScaledVector(fwdOut, towerZ);
      if (!clearOfTrack(towerWorldA, 7.2)) {
        const altX = -pitL * 0.5 - 3.0;
        const towerWorldB = base.clone().addScaledVector(rightAlong, altX).addScaledVector(fwdOut, towerZ);
        if (clearOfTrack(towerWorldB, 7.2)) {
          towerX = altX;
        } else {
          towerX = pitL * 0.32;
          towerZ = pitDepth + 2.1;
        }
      }
      const towerWorldFinal = base.clone().addScaledVector(rightAlong, towerX).addScaledVector(fwdOut, towerZ);
      buildingFootprints.push({ x: towerWorldFinal.x, z: towerWorldFinal.z, r: 3.4 });

      const towerH = 16.4;
      const towerBase = new THREE.Mesh(boxGeo, archConcreteMat);
      towerBase.scale.set(3.2, 2.8, 2.8);
      towerBase.position.set(towerX, 0.2, towerZ);
      pg.add(towerBase);

      const towerBody = new THREE.Mesh(boxGeo, archSteelMat);
      towerBody.scale.set(2.45, towerH, 1.45);
      towerBody.position.set(towerX, towerH * 0.5 + 0.5, towerZ);
      pg.add(towerBody);

      // Tapered architectural rooftop antenna fin + cyan beacon crown
      const towerCap = new THREE.Mesh(boxGeo, archSteelLightMat);
      towerCap.scale.set(2.65, 0.42, 1.65);
      towerCap.position.set(towerX, towerH + 0.65, towerZ);
      pg.add(towerCap);

      const spire = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.10, 3.6, 8), archSteelLightMat);
      spire.position.set(towerX, towerH + 2.55, towerZ);
      pg.add(spire);

      // 3-sided luminous FIA Timing Tower LED leaderboard screens (visible from all approaches)
      const ttTex = timingTowerTexture();
      const ttMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        map: ttTex,
        emissiveMap: ttTex,
        emissive: 0xffffff,
        emissiveIntensity: 0.78,
        roughness: 0.28,
        metalness: 0.1,
      }), 0.78);
      const screenGeo = new THREE.PlaneGeometry(2.15, 11.4);
      const screenFront = new THREE.Mesh(screenGeo, ttMat);
      screenFront.position.set(towerX, 9.6, towerZ - 0.74);
      screenFront.rotation.y = Math.PI; // face track (-Z)
      pg.add(screenFront);

      const screenSideGeo = new THREE.PlaneGeometry(1.24, 11.4);
      const screenLeft = new THREE.Mesh(screenSideGeo, ttMat);
      screenLeft.position.set(towerX - 1.24, 9.6, towerZ);
      screenLeft.rotation.y = -Math.PI / 2;
      pg.add(screenLeft);

      const screenRight = new THREE.Mesh(screenSideGeo, ttMat);
      screenRight.position.set(towerX + 1.24, 9.6, towerZ);
      screenRight.rotation.y = Math.PI / 2;
      pg.add(screenRight);

      // Pulsing telemetry LED strips framing the timing tower edges
      const ttPulseMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        color: 0x0a1c20,
        emissive: 0x22f0dc,
        emissiveIntensity: 0.95,
        roughness: 0.2,
      }), 0.95);
      timingPulseMats.push(ttPulseMat);
      for (const sx of [-1.24, 1.24]) {
        const edgeLed = new THREE.Mesh(boxGeo, ttPulseMat);
        edgeLed.scale.set(0.08, towerH - 1.6, 0.12);
        edgeLed.position.set(towerX + sx, towerH * 0.5 + 0.6, towerZ - 0.73);
        pg.add(edgeLed);
      }

      group.add(pg);
    }
  }

  // ------------------------------------------- Trackside Stadium Floodlight Masts (12–14 towers)
  {
    const mastCount = 14;
    const mastSites = [];
    for (let k = 0; k < mastCount; k++) {
      const idx = Math.floor((n * (k + 0.25)) / mastCount) % n;
      const f = frame(idx);
      const prefSide = k % 2 === 0 ? -insideSign(idx) : insideSign(idx);
      let placed = null;
      for (const sgn of [prefSide, -prefSide]) {
        for (const extra of [12.5, 16.5]) {
          const p = f.c.clone().addScaledVector(f.side, sgn * (track.width[idx] * 0.5 + extra));
          p.y = groundY;
          if (!clearOfTrack(p, 8.2) || !clearOfBuildings(p, 3.0) || !clearOfProps(p, 2.2)) continue;
          const hit = groundHit(p.x, p.z);
          if (hit) {
            if (Math.abs(hit.face.normal.y) < APRON_COS) continue;
            p.y = hit.point.y;
          }
          // Face inward toward the track centerline
          const toTrack = f.c.clone().sub(p).setY(0).normalize();
          placed = { p, toTrack };
          break;
        }
        if (placed) break;
      }
      if (placed) {
        mastSites.push(placed);
        buildingFootprints.push({ x: placed.p.x, z: placed.p.z, r: 2.6 });
      }
    }

    if (mastSites.length) {
      // Composite steel mast geometry (concrete plinth + tapered steel shaft + angled cantilever boom + housing)
      const plinthG = new THREE.CylinderGeometry(0.55, 0.72, 1.8, 8);
      plinthG.translate(0, 0.2, 0); // dips 0.7m below groundY/hitY for clean slope embed
      const poleG = new THREE.CylinderGeometry(0.16, 0.36, 15.6, 8);
      poleG.translate(0, 8.4, 0);
      const collarG = new THREE.CylinderGeometry(0.32, 0.32, 0.35, 8);
      collarG.translate(0, 15.6, 0);
      const boomG = new THREE.BoxGeometry(0.24, 0.28, 3.2);
      boomG.rotateX(-0.22);
      boomG.translate(0, 16.15, 1.35);
      const backHeadG = new THREE.BoxGeometry(3.4, 1.45, 0.38);
      backHeadG.rotateX(0.42);
      backHeadG.translate(0, 16.25, 2.72);
      const mastGeo = mergeSimpleGeos([plinthG, poleG, collarG, boomG, backHeadG]);

      // Emissive LED floodlight panel facing inward/downward (+Z tilted down)
      const faceGeo = new THREE.PlaneGeometry(3.15, 1.24);
      faceGeo.rotateX(0.42);
      faceGeo.translate(0, 16.17, 2.94);

      const floodTex = floodlightArrayTexture();
      const floodLedMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        map: floodTex,
        emissiveMap: floodTex,
        emissive: 0xfff2d8,
        emissiveIntensity: 0.90,
        roughness: 0.22,
        metalness: 0.15,
      }), 0.90);

      const masts = new THREE.InstancedMesh(mastGeo, archSteelMat, mastSites.length);
      const faces = new THREE.InstancedMesh(faceGeo, floodLedMat, mastSites.length);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
      const rightV = new THREE.Vector3();
      mastSites.forEach((st, k) => {
        rightV.crossVectors(upY, st.toTrack).normalize();
        q.setFromRotationMatrix(new THREE.Matrix4().makeBasis(rightV, upY, st.toTrack));
        m4.compose(st.p, q, one);
        masts.setMatrixAt(k, m4);
        faces.setMatrixAt(k, m4);
      });
      group.add(masts, faces);
    }
  }

  // ------------------------------------------- Digital FIA LED Marshal Flag Panels
  const marshalAnim = [];
  {
    const ledTex = marshalLedTexture();
    const yellowFrameMat = new THREE.MeshStandardMaterial({
      color: 0xf2c822, roughness: 0.42, metalness: 0.18,
    });
    const postGeo = new THREE.CylinderGeometry(0.065, 0.085, 3.1, 8);
    postGeo.translate(0, 0.85, 0); // dips 0.7m below ground for solid slope seating
    const frameGeo = new THREE.BoxGeometry(1.16, 1.16, 0.14);
    frameGeo.translate(0, 2.52, 0);
    const screenGeo = new THREE.PlaneGeometry(0.96, 0.96);
    screenGeo.rotateY(Math.PI); // face -Z toward approaching cars
    screenGeo.translate(0, 2.52, -0.078);

    // Choose up to 8 well-spaced corners & straight sectors
    const panelIndices = [];
    for (const z of zones) {
      if (panelIndices.length >= 6) break;
      const idx = (z.start - 5 + n) % n;
      const far = panelIndices.every((pi) => {
        const d = Math.abs(idx - pi);
        return Math.min(d, n - d) > 36;
      });
      if (far) panelIndices.push(idx);
    }
    // Guarantee at least start/finish & halfway marshal posts on smooth ovals
    for (const fallback of [Math.floor(n * 0.12), Math.floor(n * 0.52), Math.floor(n * 0.78)]) {
      if (panelIndices.length >= 8) break;
      if (panelIndices.every((pi) => Math.min(Math.abs(fallback - pi), n - Math.abs(fallback - pi)) > 36)) {
        panelIndices.push(fallback);
      }
    }

    // 3 phase-offset marshal LED materials so corners pulse with organic live circuit rhythm
    const panelMats = [0, 1, 2].map((phaseIdx) => {
      const m = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
        color: 0x0a1412,
        map: ledTex,
        emissiveMap: ledTex,
        emissive: new THREE.Color(0x18e898),
        emissiveIntensity: 0.95,
        roughness: 0.24,
        metalness: 0.12,
      }), 0.95);
      marshalAnim.push({ mat: m, phase: phaseIdx * 2.094 });
      return m;
    });

    panelIndices.forEach((idx, k) => {
      const f = frame(idx);
      const out = -insideSign(idx);
      const p = f.c.clone().addScaledVector(f.side, out * (track.width[idx] * 0.5 + 3.2));
      if (!clearOfProps(p, 1.2)) return;
      const hit = groundHit(p.x, p.z);
      if (hit && Math.abs(hit.face.normal.y) >= APRON_COS) p.y = hit.point.y;
      const pg = new THREE.Group();
      pg.position.copy(p);
      pg.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
      pg.rotateY(-out * 0.18); // subtle toe-in toward approaching drivers
      pg.add(
        new THREE.Mesh(postGeo, archSteelMat),
        new THREE.Mesh(frameGeo, yellowFrameMat),
        new THREE.Mesh(screenGeo, panelMats[k % panelMats.length]),
      );
      group.add(pg);
    });
  }

  // ------------------------------------------- Higher-Detail 4-Tier Conifers + Organic Broadleaves
  const conifers = [], broads = [];
  const tryTree = (p, scaleMin, scaleSpan) => {
    p.y = groundY;
    if (!clearOfTrack(p, 7.5)) return;
    if (!clearOfBuildings(p, 1.5)) return;
    if (!clearOfProps(p, 2.2)) return;
    const hit = groundHit(p.x, p.z);
    if (hit) {
      if (Math.abs(hit.face.normal.y) < APRON_COS) return;
      p.y = hit.point.y;
    }
    const entry = {
      p,
      scale: scaleMin + rand() * scaleSpan,
      stretchY: 0.92 + rand() * 0.28,
      rot: rand() * Math.PI * 2,
      leanX: (rand() - 0.5) * 0.08,
      leanZ: (rand() - 0.5) * 0.08,
    };
    (rand() < 0.66 ? conifers : broads).push(entry);
  };

  // Near-track woodland belt
  for (let i = 0; i < n; i += 3 + ((rand() * 3) | 0)) {
    for (const s of [-1, 1]) {
      if (rand() < 0.24) continue;
      const f = frame(i);
      const lat = track.width[i] * 0.5 + 11.5 + rand() * 19;
      const p = f.c.clone()
        .addScaledVector(f.side, s * lat)
        .addScaledVector(f.t, (rand() - 0.5) * 5.0);
      tryTree(p, 0.85, 0.78);
    }
  }
  // Secondary deeper alpine forest clusters for lush valley depth
  for (let i = 0; i < n; i += 6 + ((rand() * 4) | 0)) {
    for (const s of [-1, 1]) {
      if (rand() < 0.35) continue;
      const f = frame(i);
      const lat = track.width[i] * 0.5 + 32 + rand() * 34;
      const p = f.c.clone()
        .addScaledVector(f.side, s * lat)
        .addScaledVector(f.t, (rand() - 0.5) * 9.0);
      tryTree(p, 1.05, 0.95);
    }
  }

  const barkMat = new THREE.MeshStandardMaterial({ color: 0x3b2c22, roughness: 0.95, metalness: 0.0 });
  const leafMat = new THREE.MeshStandardMaterial({
    color: 0xffffff, roughness: 0.86, metalness: 0.0, envMapIntensity: 0.42,
  });
  const m4t = new THREE.Matrix4(), qt = new THREE.Quaternion(), eut = new THREE.Euler();
  const svt = new THREE.Vector3();

  if (conifers.length) {
    const trunkGeo = new THREE.CylinderGeometry(0.13, 0.27, 1.9, 7);
    trunkGeo.translate(0, 0.85, 0); // dips 0.1m into ground
    const canopyGeo = buildSculptedConiferCrownGeo();
    const trunks = new THREE.InstancedMesh(trunkGeo, barkMat, conifers.length);
    const canopies = new THREE.InstancedMesh(canopyGeo, leafMat, conifers.length);
    const pineCols = [0x173022, 0x1e3c29, 0x24472e, 0x152a1e, 0x2b4e32, 0x203a2c].map((c) => new THREE.Color(c));
    conifers.forEach((tr, k) => {
      eut.set(tr.leanX, tr.rot, tr.leanZ);
      qt.setFromEuler(eut);
      svt.set(tr.scale, tr.scale * tr.stretchY, tr.scale);
      m4t.compose(tr.p, qt, svt);
      trunks.setMatrixAt(k, m4t);
      canopies.setMatrixAt(k, m4t);
      canopies.setColorAt(k, pineCols[(rand() * pineCols.length) | 0]);
    });
    group.add(trunks, canopies);
  }

  if (broads.length) {
    const trunkGeo = new THREE.CylinderGeometry(0.15, 0.30, 2.2, 7);
    trunkGeo.translate(0, 1.0, 0);
    const crownGeo = buildOrganicBroadleafCrownGeo();
    const trunks = new THREE.InstancedMesh(trunkGeo, barkMat, broads.length);
    const crowns = new THREE.InstancedMesh(crownGeo, leafMat, broads.length);
    const broadCols = [0x2a4a28, 0x38592d, 0x475d2b, 0x5c5829, 0x305232, 0x4f6132].map((c) => new THREE.Color(c));
    broads.forEach((tr, k) => {
      eut.set(tr.leanX, tr.rot, tr.leanZ);
      qt.setFromEuler(eut);
      svt.set(tr.scale, tr.scale * tr.stretchY, tr.scale);
      m4t.compose(tr.p, qt, svt);
      trunks.setMatrixAt(k, m4t);
      crowns.setMatrixAt(k, m4t);
      crowns.setColorAt(k, broadCols[(rand() * broadCols.length) | 0]);
    });
    group.add(trunks, crowns);
  }

  // ------------------------------------------- Collision-Exact Barriers (v2) / Heuristics (v1)
  const tireMat = new THREE.MeshStandardMaterial({
    color: 0xffffff, roughness: 0.78, metalness: 0.05, envMapIntensity: 0.55,
  });

  if (props) {
    // Type 0: Ribbed Tire Wall Stacks (exact pr.pos, pr.yaw, pr.half collision bounds!)
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
      const tireGeo = buildRibbedTireGeo();
      const barrier = new THREE.InstancedMesh(tireGeo, tireMat, total);
      const red = new THREE.Color(0xc82e24), white = new THREE.Color(0xe4e8ee);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const dir = new THREE.Vector3(), p = new THREE.Vector3(), s = new THREE.Vector3();
      let k = 0;
      for (const it of items) {
        const { pr } = it;
        q.setFromAxisAngle(upY, pr.yaw);
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
      const lat = track.width[i] * 0.5 + 2.4 + rand() * 0.8;
      const p = f.c.clone().addScaledVector(f.side, -insideSign(i) * lat)
        .addScaledVector(f.t, (rand() - 0.5) * 1.2);
      stacks.push({ p, u: f.u });
    }
    if (stacks.length) {
      const tireGeo = buildRibbedTireGeo();
      const barrier = new THREE.InstancedMesh(tireGeo, tireMat, stacks.length * 2);
      const red = new THREE.Color(0xc82e24), white = new THREE.Color(0xe4e8ee);
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const sc = new THREE.Vector3(0.55, 0.55, 0.55);
      const pos = new THREE.Vector3();
      stacks.forEach((st, k) => {
        for (const level of [0, 1]) {
          pos.copy(st.p).addScaledVector(st.u, 0.28 + level * 0.56);
          m4.compose(pos, q, sc);
          barrier.setMatrixAt(2 * k + level, m4);
          barrier.setColorAt(2 * k + level, (k + level) % 2 ? red : white);
        }
      });
      group.add(barrier);
    }
  }

  // ------------------------------------------- Raked FIA Pebble Gravel-Trap Run-Off Beds
  {
    const gMaps = gravelTrapPBRMaps();
    const pos = [], uvs = [], col = [], gidx = [];
    const tintBase = new THREE.Color(0xd2c8b0);
    for (const z of zones.slice(0, 4)) {
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
        const wSpan = A.distanceTo(B);
        for (const f of [0.05, 0.5, 0.92]) {
          const p = A.clone().lerp(B, f).addScaledVector(u, 0.048);
          pos.push(p.x, p.y, p.z);
          uvs.push((s * 2.5) / 3.0, (f * wSpan) / 3.0);
          const m = 0.92 + (rand() - 0.5) * 0.12;
          col.push(tintBase.r * m, tintBase.g * m, tintBase.b * m);
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
      g.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(uvs), 2));
      g.setAttribute("color", new THREE.BufferAttribute(new Float32Array(col), 3));
      g.setIndex(gidx);
      g.computeVertexNormals();
      const gravelMesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
        map: gMaps.map,
        normalMap: gMaps.normalMap,
        normalScale: new THREE.Vector2(0.85, 0.85),
        vertexColors: true,
        roughness: 0.96,
        metalness: 0.0,
        envMapIntensity: 0.35,
        side: THREE.DoubleSide,
      }));
      gravelMesh.receiveShadow = true;
      group.add(gravelMesh);
    }
  }

  // ------------------------------------------- Corrugated Galvanized Armco + I-Posts + Precast Walls + Catch-Fence
  const galvTex = galvanizedMetalTexture();
  const armcoMat = new THREE.MeshStandardMaterial({
    color: 0xc4cdd8,
    map: galvTex,
    metalness: 0.84,
    roughness: 0.28,
    envMapIntensity: 1.35,
    side: THREE.DoubleSide,
  });
  const postSteelMat = new THREE.MeshStandardMaterial({
    color: 0x4c545e, metalness: 0.62, roughness: 0.48,
  });

  if (props) {
    // Type 1: W-Profile Galvanized Armco (exact pr.pos, pr.yaw, pr.half) + I-posts + reflector chevrons
    const armco = props.filter((p) => p.type === 1);
    if (armco.length) {
      const loop = [
        [1, 0], [0.52, 1], [0, 0.267], [-0.52, 1], [-1, 0],
        [-0.52, -1], [0, -0.267], [0.52, -1],
      ];
      const m = loop.length;
      const rp = new Float32Array(m * 2 * 3);
      const ruv = new Float32Array(m * 2 * 2);
      loop.forEach(([y, z], k) => {
        rp.set([-1, y, z], k * 6);
        rp.set([1, y, z], k * 6 + 3);
        const vCoord = k / m;
        ruv.set([0, vCoord], k * 4);
        ruv.set([2, vCoord], k * 4 + 2);
      });
      const ridx = [];
      for (let k = 0; k < m; k++) {
        const j = (k + 1) % m;
        ridx.push(2 * k, 2 * k + 1, 2 * j, 2 * j, 2 * k + 1, 2 * j + 1);
      }
      const railGeo = new THREE.BufferGeometry();
      railGeo.setAttribute("position", new THREE.BufferAttribute(rp, 3));
      railGeo.setAttribute("uv", new THREE.BufferAttribute(ruv, 2));
      railGeo.setIndex(ridx);
      railGeo.computeVertexNormals();

      const railMesh = new THREE.InstancedMesh(railGeo, armcoMat, armco.length);
      const postXfs = [];
      const chevXfs = [];
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const p = new THREE.Vector3(), s = new THREE.Vector3(), dirX = new THREE.Vector3();

      armco.forEach((pr, k) => {
        q.setFromAxisAngle(upY, pr.yaw);
        p.set(pr.pos[0], pr.pos[1], pr.pos[2]);
        s.set(pr.half[0], pr.half[1], pr.half[2]);
        m4.compose(p, q, s);
        railMesh.setMatrixAt(k, m4);

        // Vertical steel I-posts tucked cleanly inside Armco centerline every ~2.2m
        const alongX = pr.half[0] >= pr.half[2];
        const halfLen = alongX ? pr.half[0] : pr.half[2];
        const pCount = Math.max(1, Math.min(24, Math.floor((halfLen * 2) / 2.2) + 1));
        dirX.set(alongX ? 1 : 0, 0, alongX ? 0 : 1).applyQuaternion(q);
        for (let pi = 0; pi < pCount; pi++) {
          const frac = pCount === 1 ? 0 : -1 + (2 * pi) / (pCount - 1);
          const off = frac * Math.max(0, halfLen - 0.15);
          const postP = new THREE.Vector3(pr.pos[0], pr.pos[1] - pr.half[1] * 0.25, pr.pos[2])
            .addScaledVector(dirX, off);
          postXfs.push({ p: postP, q: q.clone(), h: pr.half[1] * 1.95 });
          // Red/white retro-reflective top delineator chevron on alternating posts
          if ((pi + k) % 2 === 0) {
            const chevP = new THREE.Vector3(pr.pos[0], pr.pos[1] + pr.half[1] + 0.09, pr.pos[2])
              .addScaledVector(dirX, off);
            chevXfs.push({ p: chevP, q: q.clone(), red: (pi % 2) === 0 });
          }
        }
      });
      group.add(railMesh);

      if (postXfs.length) {
        const iPostGeo = new THREE.BoxGeometry(0.11, 1.0, 0.11);
        const posts = new THREE.InstancedMesh(iPostGeo, postSteelMat, postXfs.length);
        const scP = new THREE.Vector3();
        postXfs.forEach((xf, idx) => {
          scP.set(1, xf.h, 1);
          m4.compose(xf.p, xf.q, scP);
          posts.setMatrixAt(idx, m4);
        });
        group.add(posts);
      }
      if (chevXfs.length) {
        const chevGeo = new THREE.BoxGeometry(0.14, 0.16, 0.06);
        const chevMat = new THREE.MeshStandardMaterial({
          color: 0xffffff, emissive: 0x382020, emissiveIntensity: 0.35, roughness: 0.32, metalness: 0.1,
        });
        const chevs = new THREE.InstancedMesh(chevGeo, chevMat, chevXfs.length);
        const cRed = new THREE.Color(0xe62c22), cWht = new THREE.Color(0xf4f7fc);
        const one = new THREE.Vector3(1, 1, 1);
        chevXfs.forEach((xf, idx) => {
          m4.compose(xf.p, xf.q, one);
          chevs.setMatrixAt(idx, m4);
          chevs.setColorAt(idx, xf.red ? cRed : cWht);
        });
        group.add(chevs);
      }
    }

    // Type 2: Weathered Precast FIA Concrete Walls (exact pr.pos, pr.yaw, pr.half) + Catch-Fence Uprights
    const walls = props.filter((p) => p.type === 2);
    if (walls.length) {
      const cMaps = concreteWallPBRMaps();
      const wallGeo = new THREE.BoxGeometry(2, 2, 2);
      const wallMat = new THREE.MeshStandardMaterial({
        map: cMaps.map,
        normalMap: cMaps.normalMap,
        normalScale: new THREE.Vector2(0.75, 0.75),
        roughness: 0.84,
        metalness: 0.06,
      });
      const wallMesh = new THREE.InstancedMesh(wallGeo, wallMat, walls.length);
      const fencePosts = [];
      const fenceRails = [];
      const m4 = new THREE.Matrix4(), q = new THREE.Quaternion();
      const p = new THREE.Vector3(), s = new THREE.Vector3(), dirW = new THREE.Vector3();

      walls.forEach((pr, k) => {
        q.setFromAxisAngle(upY, pr.yaw);
        p.set(pr.pos[0], pr.pos[1], pr.pos[2]);
        s.set(pr.half[0], pr.half[1], pr.half[2]);
        m4.compose(p, q, s);
        wallMesh.setMatrixAt(k, m4);

        // Subtle FIA debris catch-fence steel uprights & tension cables rising above wall top
        const alongX = pr.half[0] >= pr.half[2];
        const halfLen = alongX ? pr.half[0] : pr.half[2];
        const topY = pr.pos[1] + pr.half[1];
        const fCount = Math.max(2, Math.min(20, Math.floor((halfLen * 2) / 3.4) + 1));
        dirW.set(alongX ? 1 : 0, 0, alongX ? 0 : 1).applyQuaternion(q);
        for (let fi = 0; fi < fCount; fi++) {
          const frac = -1 + (2 * fi) / (fCount - 1);
          const fp = new THREE.Vector3(pr.pos[0], topY, pr.pos[2])
            .addScaledVector(dirW, frac * Math.max(0, halfLen - 0.12));
          fencePosts.push({ p: fp, q: q.clone() });
        }
        // Horizontal tension cable rails spanning the concrete wall length above wall top
        for (const cy of [0.9, 1.75, 2.55]) {
          fenceRails.push({
            p: new THREE.Vector3(pr.pos[0], topY + cy, pr.pos[2]),
            q: q.clone(),
            sx: alongX ? halfLen * 2 : 0.03,
            sz: alongX ? 0.03 : halfLen * 2,
          });
        }
      });
      group.add(wallMesh);

      if (fencePosts.length) {
        const upGeo = new THREE.CylinderGeometry(0.035, 0.045, 2.7, 6);
        upGeo.translate(0, 1.35, 0);
        const fMesh = new THREE.InstancedMesh(upGeo, postSteelMat, fencePosts.length);
        const one = new THREE.Vector3(1, 1, 1);
        fencePosts.forEach((xf, idx) => {
          m4.compose(xf.p, xf.q, one);
          fMesh.setMatrixAt(idx, m4);
        });
        group.add(fMesh);
      }
      if (fenceRails.length) {
        const wireGeo = new THREE.BoxGeometry(1, 0.028, 1);
        const wireMat = new THREE.MeshStandardMaterial({
          color: 0x8a95a2, metalness: 0.78, roughness: 0.35,
        });
        const wMesh = new THREE.InstancedMesh(wireGeo, wireMat, fenceRails.length);
        const sw = new THREE.Vector3();
        fenceRails.forEach((xf, idx) => {
          sw.set(xf.sx, 1, xf.sz);
          m4.compose(xf.p, xf.q, sw);
          wMesh.setMatrixAt(idx, m4);
        });
        group.add(wMesh);
      }
    }
  } else {
    // v1 heuristic Armco guardrails
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
        const r = track.width[k] * 0.5 + 5;
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
      const half = track.width[i] * 0.5;
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
      const ruv = new Float32Array(prof.length * 2 * 2);
      prof.forEach(([y, zz], k) => {
        rp.set([-1.275, y, zz], k * 6);
        rp.set([1.275, y, zz], k * 6 + 3);
        const vCoord = k / (prof.length - 1);
        ruv.set([0, vCoord], k * 4);
        ruv.set([1, vCoord], k * 4 + 2);
      });
      const ridx = [];
      for (let k = 0; k < prof.length - 1; k++) {
        ridx.push(2 * k, 2 * k + 1, 2 * k + 2, 2 * k + 2, 2 * k + 1, 2 * k + 3);
      }
      const railGeo = new THREE.BufferGeometry();
      railGeo.setAttribute("position", new THREE.BufferAttribute(rp, 3));
      railGeo.setAttribute("uv", new THREE.BufferAttribute(ruv, 2));
      railGeo.setIndex(ridx);
      railGeo.computeVertexNormals();
      const railMesh = new THREE.InstancedMesh(railGeo, armcoMat, use.length);
      const postGeo = new THREE.BoxGeometry(0.14, 0.95, 0.10);
      postGeo.translate(0, 0.32, 0);
      const postMesh = new THREE.InstancedMesh(postGeo, postSteelMat, use.length);
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

  // ------------------------------------------- Open Space-Frame FIA Start/Finish Timing Gantry
  const startLightMats = [];
  {
    const f = frame(0);
    const gantry = new THREE.Group();
    gantry.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
    gantry.position.copy(f.c);

    const postX = track.width[0] * 0.5 + 1.85;
    const bridgeY = 6.2;
    const trussGeos = [];
    const addBeam = (ax, ay, az, bx, by, bz, radius) => {
      const a = new THREE.Vector3(ax, ay, az);
      const b = new THREE.Vector3(bx, by, bz);
      const dir = b.clone().sub(a);
      const len = dir.length();
      const g = new THREE.BoxGeometry(radius * 2, len, radius * 2);
      const q = new THREE.Quaternion().setFromUnitVectors(upY, dir.clone().normalize());
      g.applyQuaternion(q);
      g.translate((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5);
      trussGeos.push(g);
    };

    // Left & Right Open Space-Frame Steel Tower Columns (4 corner chords + X-bracing)
    for (const s of [-1, 1]) {
      const cxP = s * postX;
      const hw = 0.44, hd = 0.46;
      for (const dx of [-hw, hw]) {
        for (const dz of [-hd, hd]) {
          addBeam(cxP + dx, 0.3, dz, cxP + dx, bridgeY + 0.85, dz, 0.065);
        }
      }
      const tiers = 5;
      for (let t = 0; t < tiers; t++) {
        const y0 = 0.75 + (t / tiers) * (bridgeY - 0.5);
        const y1 = 0.75 + ((t + 1) / tiers) * (bridgeY - 0.5);
        // Horizontal collar & X-diagonals on front/back/side faces
        addBeam(cxP - hw, y0, -hd, cxP + hw, y0, -hd, 0.04);
        addBeam(cxP - hw, y0,  hd, cxP + hw, y0,  hd, 0.04);
        addBeam(cxP - hw, y0, -hd, cxP - hw, y0,  hd, 0.04);
        addBeam(cxP + hw, y0, -hd, cxP + hw, y0,  hd, 0.04);
        addBeam(cxP - hw, y0, -hd, cxP + hw, y1, -hd, 0.036);
        addBeam(cxP + hw, y0,  hd, cxP - hw, y1,  hd, 0.036);
        addBeam(cxP + hw, y0, -hd, cxP + hw, y1,  hd, 0.036);
        addBeam(cxP - hw, y0,  hd, cxP - hw, y1, -hd, 0.036);
      }

      // Beveled Concrete Plinth Foundation
      const plinth = new THREE.Mesh(new THREE.BoxGeometry(1.32, 1.35, 1.36), archConcreteMat);
      plinth.position.set(cxP, 0.25, 0);
      gantry.add(plinth);
      const collarStripe = new THREE.Mesh(
        new THREE.BoxGeometry(1.36, 0.22, 1.40),
        new THREE.MeshStandardMaterial({ color: 0xe6be24, roughness: 0.55 }),
      );
      collarStripe.position.set(cxP, 0.78, 0);
      gantry.add(collarStripe);
    }

    // Overhead Cantilevered Space-Frame Truss Span across track
    const spanHalf = postX + 0.85;
    const bBot = bridgeY - 0.22, bTop = bridgeY + 0.82;
    const bDz = 0.44;
    // 4 main horizontal chords
    for (const yC of [bBot, bTop]) {
      for (const zC of [-bDz, bDz]) {
        addBeam(-spanHalf, yC, zC, spanHalf, yC, zC, 0.07);
      }
    }
    const bays = Math.max(8, Math.round((spanHalf * 2) / 1.8));
    for (let b = 0; b < bays; b++) {
      const x0 = -spanHalf + (b / bays) * (spanHalf * 2);
      const x1 = -spanHalf + ((b + 1) / bays) * (spanHalf * 2);
      const flip = b % 2 === 0;
      addBeam(x0, bBot, -bDz, x0, bTop, -bDz, 0.04);
      addBeam(x0, bBot,  bDz, x0, bTop,  bDz, 0.04);
      addBeam(x0, bTop, -bDz, x0, bTop,  bDz, 0.04);
      addBeam(x0, bBot, -bDz, x0, bBot,  bDz, 0.04);
      if (flip) {
        addBeam(x0, bBot, -bDz, x1, bTop, -bDz, 0.036);
        addBeam(x0, bBot,  bDz, x1, bTop,  bDz, 0.036);
      } else {
        addBeam(x0, bTop, -bDz, x1, bBot, -bDz, 0.036);
        addBeam(x0, bTop,  bDz, x1, bBot,  bDz, 0.036);
      }
    }
    gantry.add(new THREE.Mesh(mergeSimpleGeos(trussGeos), archSteelMat));

    // Dual-Sided 1024×256 Glowing LED Timing Fascia Banner (faces BOTH -Z and +Z!)
    const bannerTex = gantryBannerTexture();
    const bannerMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
      map: bannerTex,
      emissiveMap: bannerTex,
      emissive: 0xffffff,
      emissiveIntensity: 0.65,
      roughness: 0.34,
      metalness: 0.14,
    }), 0.65);
    const bannerW = Math.min(2 * postX - 1.4, 9.6);
    const bannerBox = new THREE.Mesh(
      new THREE.BoxGeometry(bannerW, 1.56, 0.28),
      [archSteelMat, archSteelMat, archSteelMat, archSteelMat, bannerMat, bannerMat],
    );
    bannerBox.position.set(0, bridgeY + 0.34, 0);
    gantry.add(bannerBox);

    // Recessed Underside Track Floodlight Bar illuminating Start/Finish Grid
    const stripMat = registerGlow(glowMaterials, new THREE.MeshStandardMaterial({
      color: 0x10141c,
      emissive: 0xfff2d2,
      emissiveIntensity: 0.78,
      roughness: 0.26,
    }), 0.78);
    const strip = new THREE.Mesh(new THREE.BoxGeometry(2 * postX - 0.8, 0.10, 0.24), stripMat);
    strip.position.set(0, bBot - 0.08, 0);
    gantry.add(strip);

    // 5 High-Contrast Hooded FIA LED Start-Light Pods (dual stacked lenses per pod!)
    const podBar = new THREE.Mesh(new THREE.BoxGeometry(4.1, 0.84, 0.26), archSteelMat);
    podBar.position.set(0, 4.92, -0.52);
    gantry.add(podBar);
    // Twin vertical drop hangers connecting light bar to truss bridge
    for (const hx of [-1.45, 1.45]) {
      const hanger = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.82, 0.12), archSteelMat);
      hanger.position.set(hx, 5.65, -0.50);
      gantry.add(hanger);
    }

    const hoodGeo = new THREE.CylinderGeometry(0.155, 0.165, 0.26, 16, 1, true);
    hoodGeo.rotateX(Math.PI / 2);
    const hoodMat = new THREE.MeshStandardMaterial({
      color: 0x0d1016, roughness: 0.55, metalness: 0.35, side: THREE.DoubleSide,
    });
    const lensGeo = new THREE.CircleGeometry(0.132, 18);

    for (let k = 0; k < 5; k++) {
      const px = (k - 2) * 0.74;
      const lm = new THREE.MeshStandardMaterial({
        color: 0x160606,
        emissive: 0x280505,
        emissiveIntensity: 0.08,
        roughness: 0.16,
        metalness: 0.08,
      });
      // Dual stacked FIA lenses (top & bottom) inside individual sun-hood visors
      for (const py of [5.10, 4.74]) {
        const hood = new THREE.Mesh(hoodGeo, hoodMat);
        hood.position.set(px, py, -0.76);
        gantry.add(hood);

        const lamp = new THREE.Mesh(lensGeo, lm);
        lamp.position.set(px, py, -0.66);
        lamp.rotation.y = Math.PI; // face approaching driver (-Z)
        gantry.add(lamp);
      }
      startLightMats.push(lm);
    }
    group.add(gantry);
  }

  /** Sync 5 hooded FIA start gantry pods with countdown: high HDR bloom (4.2 red / 3.8 green). */
  group.userData.setStartLights = (litCount, isGo = false) => {
    for (let k = 0; k < startLightMats.length; k++) {
      const m = startLightMats[k];
      if (isGo) {
        m.color.setHex(0x0c381a);
        m.emissive.setHex(0x14ff6a);
        m.emissiveIntensity = 3.8;
      } else if (k < litCount) {
        m.color.setHex(0x400a08);
        m.emissive.setHex(0xff1c12);
        m.emissiveIntensity = 4.2;
      } else {
        m.color.setHex(0x160606);
        m.emissive.setHex(0x280505);
        m.emissiveIntensity = 0.08;
      }
    }
  };

  // ------------------------------------------- Braking Boards (150/100/50m) + Apex Chevrons
  {
    const tex150 = brakeBoardTexture(150, 3);
    const tex100 = brakeBoardTexture(100, 2);
    const tex50 = brakeBoardTexture(50, 1);
    const boardMats = [tex150, tex100, tex50].map((tex) => new THREE.MeshStandardMaterial({
      map: tex,
      emissive: 0x2a2e36,
      emissiveMap: tex,
      emissiveIntensity: 0.24,
      roughness: 0.42,
      metalness: 0.05,
    }));
    const backMat = new THREE.MeshStandardMaterial({ color: 0x1b2028, roughness: 0.72, metalness: 0.25 });

    const chosenZones = [];
    for (const z of zones) {
      if (chosenZones.length >= 4) break;
      const farEnough = chosenZones.every((oz) => {
        const d = Math.abs(z.start - oz.start);
        return Math.min(d, n - d) > 55;
      });
      if (farEnough) chosenZones.push(z);
    }

    const postBox = new THREE.BoxGeometry(0.08, 1.15, 0.08);
    const boardBox = new THREE.BoxGeometry(1.24, 0.86, 0.07);
    const chevBox = new THREE.BoxGeometry(2.25, 1.02, 0.09);

    for (const z of chosenZones) {
      const out = -insideSign(z.peakIdx);
      const specs = [[60, boardMats[0]], [40, boardMats[1]], [20, boardMats[2]]];
      for (const [back, faceMat] of specs) {
        const bi = (z.start - back + n) % n;
        if (Math.abs(kappa[bi]) > 0.018) continue;
        const f = frame(bi);
        const pos = f.c.clone().addScaledVector(f.side, out * (track.width[bi] * 0.5 + 2.8));
        const board = new THREE.Group();
        board.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(f.side, f.u, f.t));
        board.position.copy(pos);
        for (const sx of [-0.36, 0.36]) {
          const post = new THREE.Mesh(postBox, postSteelMat);
          post.position.set(sx, 0.55, 0);
          board.add(post);
        }
        // Box faces: +x,-x,+y,-y,+z,-z — local -z faces approaching cars
        const face = new THREE.Mesh(boardBox, [
          backMat, backMat, backMat, backMat, backMat, faceMat,
        ]);
        face.position.y = 1.44;
        board.add(face);
        group.add(board);
      }

      // Apex chevron sightline board on the outside of each signature corner
      const fApex = frame(z.peakIdx);
      const cTex = chevronTexture(insideSign(z.peakIdx));
      const cFaceMat = new THREE.MeshStandardMaterial({
        map: cTex,
        emissive: 0x302b18,
        emissiveMap: cTex,
        emissiveIntensity: 0.22,
        roughness: 0.44,
        metalness: 0.05,
      });
      const chev = new THREE.Group();
      chev.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(fApex.side, fApex.u, fApex.t));
      chev.position.copy(fApex.c).addScaledVector(fApex.side, out * (track.width[z.peakIdx] * 0.5 + 3.6));
      for (const sx of [-0.80, 0.80]) {
        const cPost = new THREE.Mesh(postBox, postSteelMat);
        cPost.position.set(sx, 0.55, 0);
        chev.add(cPost);
      }
      const cFace = new THREE.Mesh(chevBox, [
        backMat, backMat, backMat, backMat, backMat, cFaceMat,
      ]);
      cFace.position.y = 1.48;
      chev.add(cFace);
      group.add(chev);
    }
  }

  // ------------------------------------------- Dynamic Circuit Life Hook (zero per-frame allocations)
  const _colDryA = new THREE.Color(0x16e896);
  const _colDryB = new THREE.Color(0x20dcd6);
  const _colWetA = new THREE.Color(0xffb818);
  const _colWetB = new THREE.Color(0x2298ff);
  const _colMix = new THREE.Color();

  group.userData.updateAmbience = (timeSec = 0, wetness = 0) => {
    const t = timeSec || 0;
    const w = wetness > 1 ? 1 : wetness < 0 ? 0 : (wetness || 0);
    // Digital FIA LED marshal flag panels: emerald/cyan dry telemetry pulse -> amber/blue wet caution
    for (let i = 0; i < marshalAnim.length; i++) {
      const entry = marshalAnim[i];
      const pulse = 0.5 + 0.5 * Math.sin(t * 3.6 + entry.phase);
      const bright = 0.62 + 0.38 * pulse;
      if (w > 0.35) {
        const blink = 0.5 + 0.5 * Math.sin(t * 6.0 + entry.phase);
        _colMix.copy(_colWetA).lerp(_colWetB, blink * 0.45);
      } else {
        _colMix.copy(_colDryA).lerp(_colDryB, pulse * 0.55);
      }
      entry.mat.emissive.copy(_colMix).multiplyScalar(bright);
    }
    // Subtle FIA timing tower accent heartbeat
    for (let i = 0; i < timingPulseMats.length; i++) {
      const hb = 0.74 + 0.26 * Math.sin(t * 2.2 + i);
      timingPulseMats[i].emissive.setRGB(0.13 * hb, 0.94 * hb, 0.86 * hb);
    }
  };

  group.traverse((o) => {
    if (o.isInstancedMesh) {
      o.instanceMatrix.needsUpdate = true;
      o.computeBoundingSphere();
    }
  });

  return group;
}

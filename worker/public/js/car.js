// 2026 AAA Procedural Formula / Hypercar — zero external assets, Three.js r170.
// Smooth Catmull-Rom multi-station spline-lofted monocoque & droop nosecone,
// hourglass "Coke-bottle" undercut sidepods with recessed honeycomb radiators
// & cooling louvers, cambered NACA airfoil multi-element front & rear wings
// with swan-neck pylons and sculpted louvred endplates, multi-strake Venturi
// rear diffuser, curved teardrop carbon Halo + tinted polycarbonate aero screen,
// sculpted racing helmet with iridescent visor & HANS collar, butterfly carbon
// F1 steering wheel with live 20 Hz 256×128 OLED telemetry & 15 RGB shift LEDs,
// articulated double-wishbone + pushrod carbon suspension with non-spinning
// uprights & 6-piston calipers, HDR rear LED blades / FIA rain strobe / nose
// laser DRLs + volumetric dusk/night headlight cones, twin titanium exhausts
// with animated plasma backfire, and beveled racing slicks on forged 10-spoke
// concave Y-rims with glowing cross-drilled carbon-ceramic brake rotors.
//
// Preserves 100% compatibility with Crumple soft-body crash deformation:
// all static bodywork meshes have unique geometries baked into group space in
// `group.userData.bodyMeshes` BEFORE dynamic sub-groups are attached.
import * as THREE from "../vendor/three.module.js";

const CARBON_DARK = 0x13161c;
const CYAN_HEX = 0x19e8d8;
const TITANIUM_HEX = 0xbfc8d4;

// Zero-allocation scratch math for live suspension & telemetry updates
const _Z_AXIS = new THREE.Vector3(0, 0, 1);
const _rodDir = new THREE.Vector3();
const _tmpColor = new THREE.Color();

// ---------------------------------------------------------------------------
// Deterministic pseudo-random helpers (for procedural flake & weave detail)
// ---------------------------------------------------------------------------
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function makeCanvas(w, h) {
  if (typeof document === "undefined") return null;
  const cv = document.createElement("canvas");
  cv.width = w;
  cv.height = h;
  const ctx = cv.getContext("2d");
  return ctx ? { cv, ctx } : null;
}

/** Convert a scalar Float32Array heightfield [0..1] to a wrapping tangent-space normal map. */
function heightFieldToNormalTexture(heights, w, h, strength = 2.2, repeat = true) {
  const res = makeCanvas(w, h);
  if (!res) return null;
  const { cv, ctx } = res;
  const img = ctx.createImageData(w, h);
  const d = img.data;
  const at = (x, y) => {
    const xx = repeat ? ((x % w) + w) % w : Math.max(0, Math.min(w - 1, x));
    const yy = repeat ? ((y % h) + h) % h : Math.max(0, Math.min(h - 1, y));
    return heights[yy * w + xx];
  };
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const dx = (at(x + 1, y) - at(x - 1, y)) * strength;
      const dy = (at(x, y + 1) - at(x, y - 1)) * strength;
      let nx = -dx;
      let ny = -dy;
      let nz = 1.0;
      const inv = 1 / Math.hypot(nx, ny, nz);
      nx *= inv;
      ny *= inv;
      nz *= inv;
      const idx = (y * w + x) * 4;
      d[idx] = Math.round((nx * 0.5 + 0.5) * 255);
      d[idx + 1] = Math.round((ny * 0.5 + 0.5) * 255);
      d[idx + 2] = Math.round((nz * 0.5 + 0.5) * 255);
      d[idx + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.NoColorSpace;
  tex.wrapS = tex.wrapT = repeat ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
  tex.anisotropy = 8;
  return tex;
}

// ---------------------------------------------------------------------------
// Cached Procedural PBR Textures (generated once, shared cleanly)
// ---------------------------------------------------------------------------
let _carbonCache = null;
function getCarbonTextures() {
  if (_carbonCache !== null) return _carbonCache;
  const size = 512;
  const albRes = makeCanvas(size, size);
  const rghRes = makeCanvas(size, size);
  if (!albRes || !rghRes) {
    _carbonCache = {};
    return _carbonCache;
  }
  const albImg = albRes.ctx.createImageData(size, size);
  const rghImg = rghRes.ctx.createImageData(size, size);
  const ad = albImg.data;
  const rd = rghImg.data;
  const heights = new Float32Array(size * size);

  // 2×2 diagonal twill carbon weave: 32 cells across 512px (16px per fiber tow)
  const cell = 16;
  for (let y = 0; y < size; y++) {
    const ty = Math.floor(y / cell);
    const fy = (y % cell) / cell;
    for (let x = 0; x < size; x++) {
      const tx = Math.floor(x / cell);
      const fx = (x % cell) / cell;
      // 2×2 twill pattern: diagonal step shift
      const warp = ((tx + ty) & 3) < 2;
      const across = warp ? fy : fx;
      const along = warp ? fx : fy;
      // Crowned elliptical bundle cross-section + fine micro-filament striations
      const arch = Math.sin(across * Math.PI);
      const filaments = 0.5 + 0.5 * Math.sin(across * Math.PI * 10);
      const weaveDip = 0.85 + 0.15 * Math.sin(along * Math.PI);
      const h = (0.18 + 0.72 * arch * weaveDip + 0.08 * filaments * arch);
      heights[y * size + x] = h;

      // Warp vs weft anisotropic specular brightness shift
      const base = warp ? 20 + h * 34 : 15 + h * 24;
      const sheen = warp ? 1.06 : 0.94;
      const idx = (y * size + x) * 4;
      ad[idx] = Math.min(255, Math.round(base * 0.96 * sheen));
      ad[idx + 1] = Math.min(255, Math.round(base * 1.00 * sheen));
      ad[idx + 2] = Math.min(255, Math.round(base * 1.10 * sheen));
      ad[idx + 3] = 255;

      // Roughness: smoother glossy resin on tow crowns, rougher in weave valleys
      const rVal = Math.round((0.58 - 0.34 * arch + (warp ? -0.03 : 0.04)) * 255);
      rd[idx] = rd[idx + 1] = rd[idx + 2] = Math.max(40, Math.min(230, rVal));
      rd[idx + 3] = 255;
    }
  }
  albRes.ctx.putImageData(albImg, 0, 0);
  rghRes.ctx.putImageData(rghImg, 0, 0);

  const map = new THREE.CanvasTexture(albRes.cv);
  map.colorSpace = THREE.SRGBColorSpace;
  map.wrapS = map.wrapT = THREE.RepeatWrapping;
  map.repeat.set(4, 4);
  map.anisotropy = 8;

  const roughnessMap = new THREE.CanvasTexture(rghRes.cv);
  roughnessMap.colorSpace = THREE.NoColorSpace;
  roughnessMap.wrapS = roughnessMap.wrapT = THREE.RepeatWrapping;
  roughnessMap.repeat.set(4, 4);
  roughnessMap.anisotropy = 8;

  const normalMap = heightFieldToNormalTexture(heights, size, size, 3.0, true);
  if (normalMap) normalMap.repeat.set(4, 4);

  _carbonCache = { map, normalMap, roughnessMap };
  return _carbonCache;
}

const _liveryCache = new Map();
function getLiveryTextures(colorHex = 0xd9483b) {
  const key = typeof colorHex === "number" ? colorHex : new THREE.Color(colorHex).getHex();
  if (_liveryCache.has(key)) return _liveryCache.get(key);

  const W = 1024;
  const H = 1024;
  const albRes = makeCanvas(W, H);
  const rghRes = makeCanvas(512, 512);
  if (!albRes || !rghRes) {
    const empty = {};
    _liveryCache.set(key, empty);
    return empty;
  }

  const { cv: albCv, ctx } = albRes;
  const baseCol = new THREE.Color(key);
  const darkCol = baseCol.clone().multiplyScalar(0.56);
  const hiCol = baseCol.clone().lerp(new THREE.Color(0xffffff), 0.16);
  const css = (c) => `#${c.getHexString()}`;

  // 1. Rich metallic basecoat gradient across circumferential U (bottom dark -> flanks vibrant -> crown highlight)
  const bg = ctx.createLinearGradient(0, 0, W, 0);
  bg.addColorStop(0.00, "#12151c");
  bg.addColorStop(0.11, css(darkCol));
  bg.addColorStop(0.28, css(baseCol));
  bg.addColorStop(0.50, css(hiCol));
  bg.addColorStop(0.72, css(baseCol));
  bg.addColorStop(0.89, css(darkCol));
  bg.addColorStop(1.00, "#12151c");
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, W, H);

  // Subtle longitudinal shading from nose (v=0, top of canvas? Note: flipY=true in Three.js -> y=0 is v=1 tail, y=H is v=0 nose)
  const longGrad = ctx.createLinearGradient(0, 0, 0, H);
  longGrad.addColorStop(0.0, "rgba(8,10,16,0.38)");
  longGrad.addColorStop(0.25, "rgba(0,0,0,0.06)");
  longGrad.addColorStop(0.65, "rgba(255,255,255,0.05)");
  longGrad.addColorStop(1.0, "rgba(0,0,0,0.14)");
  ctx.fillStyle = longGrad;
  ctx.fillRect(0, 0, W, H);

  // 2. Micro-metallic paint flake sparkle (deterministic)
  const rnd = mulberry32(20260101 ^ key);
  for (let i = 0; i < 16000; i++) {
    const fx = rnd() * W;
    const fy = rnd() * H;
    const bright = rnd() > 0.45;
    ctx.fillStyle = bright
      ? `rgba(255,248,242,${(0.04 + rnd() * 0.08).toFixed(3)})`
      : `rgba(12,6,8,${(0.04 + rnd() * 0.07).toFixed(3)})`;
    ctx.fillRect(fx, fy, 1.5, 1.5);
  }

  // 3. Exposed obsidian-carbon lower aero skirt along bottom flanks (u < 0.14 and u > 0.86)
  ctx.fillStyle = "#11141a";
  ctx.fillRect(0, 0, W * 0.135, H);
  ctx.fillRect(W * 0.865, 0, W * 0.135, H);

  // Diagonal carbon weave hatch hint inside obsidian zones
  ctx.strokeStyle = "rgba(255,255,255,0.045)";
  ctx.lineWidth = 2;
  for (let y = -W; y < H + W; y += 10) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(W * 0.14, y + W * 0.14);
    ctx.moveTo(W * 0.86, y);
    ctx.lineTo(W, y + W * 0.14);
    ctx.stroke();
  }

  // 4. Sculpted 2026 geometric motorsport livery flashes (symmetric around U = 0.50!)
  const drawSymStrip = (pts, fill) => {
    ctx.fillStyle = fill;
    // Left half
    ctx.beginPath();
    pts.forEach(([u, v], i) => {
      const px = u * W;
      const py = (1 - v) * H;
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.fill();
    // Mirrored right half
    ctx.beginPath();
    pts.forEach(([u, v], i) => {
      const px = (1 - u) * W;
      const py = (1 - v) * H;
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.fill();
  };

  // Obsidian sweeping aero wedge from mid-nose back over engine cover
  drawSymStrip([
    [0.13, 0.06], [0.22, 0.24], [0.31, 0.56], [0.38, 0.94],
    [0.33, 0.96], [0.23, 0.60], [0.15, 0.30], [0.11, 0.10],
  ], "#12151c");

  // Rear engine-cover dark carbon taper shroud
  drawSymStrip([
    [0.36, 0.68], [0.50, 0.80], [0.50, 1.00], [0.12, 1.00], [0.12, 0.72],
  ], "#131720");

  // Electric cyan razor flash running alongside obsidian wedge
  drawSymStrip([
    [0.215, 0.10], [0.242, 0.10], [0.340, 0.56], [0.408, 0.94],
    [0.386, 0.95], [0.316, 0.57], [0.215, 0.10],
  ], "#19e8d8");

  // Crisp titanium-white secondary pinstripe
  drawSymStrip([
    [0.252, 0.12], [0.262, 0.12], [0.354, 0.56], [0.420, 0.93],
    [0.411, 0.935], [0.344, 0.565],
  ], "#f2f6fb");

  // Center dorsal racing stripe (nose cone spine to airbox)
  ctx.fillStyle = "#11141b";
  ctx.fillRect(W * 0.486, H * 0.18, W * 0.028, H * 0.74);
  ctx.fillStyle = "#19e8d8";
  ctx.fillRect(W * 0.480, H * 0.20, W * 0.005, H * 0.70);
  ctx.fillRect(W * 0.515, H * 0.20, W * 0.005, H * 0.70);

  // 5. Crisp racing number "01" roundel on upper nose bulkhead (v ≈ 0.20 -> py ≈ 0.80*H)
  ctx.save();
  ctx.translate(W * 0.50, H * 0.79);
  ctx.fillStyle = "#10141c";
  ctx.beginPath();
  ctx.roundRect(-52, -32, 104, 64, 10);
  ctx.fill();
  ctx.strokeStyle = "#19e8d8";
  ctx.lineWidth = 3;
  ctx.stroke();
  ctx.fillStyle = "#f6f9fc";
  ctx.font = "900 44px 'Segoe UI', Arial, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("01", 0, 2);
  ctx.restore();

  // Clean sponsor typography along upper nose and side flanks
  const drawText = (txt, x, y, size, color, rot = 0, skew = -0.18) => {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(rot);
    ctx.transform(1, 0, skew, 1, 0, 0);
    ctx.fillStyle = color;
    ctx.font = `900 ${size}px 'Segoe UI', Arial, sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(txt, 0, 0);
    ctx.restore();
  };
  drawText("APEX", W * 0.50, H * 0.89, 26, "#ffffff", 0, -0.14);
  drawText("AERO-26", W * 0.23, H * 0.55, 22, "#eef4fa", -Math.PI / 2, -0.12);
  drawText("AERO-26", W * 0.77, H * 0.55, 22, "#eef4fa", Math.PI / 2, -0.12);
  drawText("STTR", W * 0.30, H * 0.35, 20, "#19e8d8", -Math.PI / 2, -0.12);
  drawText("STTR", W * 0.70, H * 0.35, 20, "#19e8d8", Math.PI / 2, -0.12);

  // 6. Aerospace panel seams & Dzus fastener rivets (also mirrored into normal + roughness!)
  const seamLines = [
    // Longitudinal panel joints
    [0.14, 0.05, 0.14, 0.95],
    [0.86, 0.05, 0.86, 0.95],
    [0.37, 0.16, 0.37, 0.88],
    [0.63, 0.16, 0.63, 0.88],
    // Circumferential bulkhead joints (nose cone seam, cockpit cowl seam, engine cover hatch)
    [0.12, 0.18, 0.88, 0.18],
    [0.12, 0.36, 0.88, 0.36],
    [0.12, 0.62, 0.88, 0.62],
    [0.16, 0.82, 0.84, 0.82],
  ];

  ctx.strokeStyle = "rgba(8,10,14,0.68)";
  ctx.lineWidth = 2.2;
  for (const [u0, v0, u1, v1] of seamLines) {
    ctx.beginPath();
    ctx.moveTo(u0 * W, (1 - v0) * H);
    ctx.lineTo(u1 * W, (1 - v1) * H);
    ctx.stroke();
    // Subtle metallic highlight lip next to panel seam
    ctx.strokeStyle = "rgba(255,255,255,0.14)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(u0 * W + 2, (1 - v0) * H + 1.5);
    ctx.lineTo(u1 * W + 2, (1 - v1) * H + 1.5);
    ctx.stroke();
    ctx.strokeStyle = "rgba(8,10,14,0.68)";
    ctx.lineWidth = 2.2;
  }

  // Tiny titanium Dzus quick-release fasteners along bulkhead seams
  ctx.fillStyle = "rgba(215,224,235,0.55)";
  for (const vSeam of [0.18, 0.36, 0.62]) {
    for (let u = 0.20; u <= 0.80; u += 0.075) {
      ctx.beginPath();
      ctx.arc(u * W, (1 - vSeam) * H - 6, 2.2, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // 7. Roughness & Normal map (512×512) matching paint vs carbon trim & recessed panel seams
  const rctx = rghRes.ctx;
  const R = 512;
  rctx.fillStyle = "rgb(56,56,56)"; // glossy paint ~0.22 roughness
  rctx.fillRect(0, 0, R, R);
  // Satin obsidian skirt zones (~0.42 roughness)
  rctx.fillStyle = "rgb(108,108,108)";
  rctx.fillRect(0, 0, R * 0.14, R);
  rctx.fillRect(R * 0.86, 0, R * 0.14, R);
  rctx.fillRect(R * 0.12, 0, R * 0.76, R * 0.24);
  // Matte panel seams
  rctx.strokeStyle = "rgb(200,200,200)";
  rctx.lineWidth = 2.5;
  for (const [u0, v0, u1, v1] of seamLines) {
    rctx.beginPath();
    rctx.moveTo(u0 * R, (1 - v0) * R);
    rctx.lineTo(u1 * R, (1 - v1) * R);
    rctx.stroke();
  }

  // Heightfield for subtle panel seam normal engraving + faint clearcoat micro-texture
  const heights = new Float32Array(R * R);
  heights.fill(0.8);
  const drawSeamHeight = (x0, y0, x1, y1) => {
    const steps = Math.ceil(Math.hypot(x1 - x0, y1 - y0) * 1.5);
    for (let s = 0; s <= steps; s++) {
      const t = s / Math.max(1, steps);
      const px = Math.round(x0 + (x1 - x0) * t);
      const py = Math.round(y0 + (y1 - y0) * t);
      for (let oy = -1; oy <= 1; oy++) {
        for (let ox = -1; ox <= 1; ox++) {
          const xx = px + ox;
          const yy = py + oy;
          if (xx >= 0 && xx < R && yy >= 0 && yy < R) {
            const d = Math.hypot(ox, oy);
            heights[yy * R + xx] = Math.min(heights[yy * R + xx], d < 0.5 ? 0.15 : 0.45);
          }
        }
      }
    }
  };
  for (const [u0, v0, u1, v1] of seamLines) {
    drawSeamHeight(u0 * R, (1 - v0) * R, u1 * R, (1 - v1) * R);
  }
  const nRnd = mulberry32(99123 ^ key);
  for (let i = 0; i < heights.length; i++) {
    heights[i] += (nRnd() - 0.5) * 0.025;
  }

  const map = new THREE.CanvasTexture(albCv);
  map.colorSpace = THREE.SRGBColorSpace;
  map.wrapS = THREE.RepeatWrapping;
  map.wrapT = THREE.ClampToEdgeWrapping;
  map.anisotropy = 8;

  const roughnessMap = new THREE.CanvasTexture(rghRes.cv);
  roughnessMap.colorSpace = THREE.NoColorSpace;
  roughnessMap.wrapS = THREE.RepeatWrapping;
  roughnessMap.wrapT = THREE.ClampToEdgeWrapping;
  roughnessMap.anisotropy = 4;

  const normalMap = heightFieldToNormalTexture(heights, R, R, 1.8, false);
  if (normalMap) {
    normalMap.wrapS = THREE.RepeatWrapping;
    normalMap.wrapT = THREE.ClampToEdgeWrapping;
  }

  const pack = { map, normalMap, roughnessMap };
  _liveryCache.set(key, pack);
  return pack;
}

let _sponsorCache = null;
function getSponsorTexture() {
  if (_sponsorCache !== null) return _sponsorCache;
  const res = makeCanvas(512, 512);
  if (!res) return null;
  const { cv, ctx } = res;
  const rows = [
    ["APEX", "#f6f9fc"],
    ["STTR", "#19e8d8"],
    ["AERO-26", "#f6f9fc"],
    ["BOX3D", "#19e8d8"],
  ];
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "900 84px 'Segoe UI', Arial, sans-serif";
  for (let i = 0; i < rows.length; i++) {
    const [word, fill] = rows[i];
    ctx.save();
    ctx.translate(256, 66 + i * 128);
    ctx.transform(1, 0, -0.18, 1, 0, 0);
    ctx.shadowColor = "rgba(0,0,0,0.55)";
    ctx.shadowBlur = 8;
    ctx.fillStyle = fill;
    ctx.fillText(word, 0, 0);
    ctx.restore();
  }
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 8;
  _sponsorCache = tex;
  return _sponsorCache;
}

function makeSponsorMaterial(row, opacity) {
  const base = getSponsorTexture();
  if (!base) return null;
  const tex = base.clone();
  tex.needsUpdate = true;
  tex.repeat.set(1, 0.25);
  tex.offset.set(0, 0.75 - row * 0.25);
  return new THREE.MeshBasicMaterial({
    map: tex,
    transparent: true,
    opacity,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: -1,
  });
}

let _tireCache = null;
function getTireTextures() {
  if (_tireCache !== null) return _tireCache;
  const S = 512;
  const trAlb = makeCanvas(S, S);
  const trRgh = makeCanvas(S, S);
  const swAlb = makeCanvas(S, S);
  if (!trAlb || !trRgh || !swAlb) {
    _tireCache = {};
    return _tireCache;
  }

  // --- 1. Racing slick tread: scrubbed heat-cycle contact bands, rubber graining, wear dimples
  const tImg = trAlb.ctx.createImageData(S, S);
  const rImg = trRgh.ctx.createImageData(S, S);
  const tH = new Float32Array(S * S);
  const rnd = mulberry32(424242);
  for (let y = 0; y < S; y++) {
    // y maps across tire width (0..1): 0.5 = tread center, 0/1 = outer shoulder edges
    const v = y / (S - 1);
    const edgeDist = Math.abs(v - 0.5) * 2.0; // 0 center .. 1 shoulder
    const scrubBand = Math.exp(-Math.pow((v - 0.5) * 2.5, 2)); // center contact sheen
    const moldSeam = Math.abs(v - 0.5) < 0.005 ? 1 : 0;
    for (let x = 0; x < S; x++) {
      const grain = (rnd() - 0.5) * (0.14 + 0.18 * edgeDist);
      const streak = Math.sin(x * 0.06 + y * 0.7) * 0.025 + Math.sin(y * 0.22) * 0.02;
      // Subtle F1 wear-indicator dimples along 3 lateral tracks
      let dimple = 0;
      if (Math.abs(v - 0.28) < 0.015 || Math.abs(v - 0.50) < 0.015 || Math.abs(v - 0.72) < 0.015) {
        const cx = ((x % 128) - 64);
        const cy = ((y % 112) - 56);
        if (cx * cx + cy * cy < 18) dimple = 0.65;
      }
      const h = 0.52 + grain * 0.35 + streak - dimple * 0.45 + moldSeam * 0.14;
      tH[y * S + x] = h;

      // Dark charcoal rubber with warmer matte-satin scrubbed contact path
      const lum = Math.max(12, Math.min(54, Math.round(25 + scrubBand * 7 + grain * 18 + streak * 20 - dimple * 14)));
      const idx = (y * S + x) * 4;
      tImg.data[idx] = lum;
      tImg.data[idx + 1] = lum + 1;
      tImg.data[idx + 2] = lum + 3;
      tImg.data[idx + 3] = 255;

      const rough = Math.max(150, Math.min(242, Math.round((0.88 - scrubBand * 0.17 + edgeDist * 0.06 + grain * 0.08) * 255)));
      rImg.data[idx] = rImg.data[idx + 1] = rImg.data[idx + 2] = rough;
      rImg.data[idx + 3] = 255;
    }
  }
  trAlb.ctx.putImageData(tImg, 0, 0);
  trRgh.ctx.putImageData(rImg, 0, 0);

  const treadMap = new THREE.CanvasTexture(trAlb.cv);
  treadMap.colorSpace = THREE.SRGBColorSpace;
  treadMap.wrapS = treadMap.wrapT = THREE.RepeatWrapping;
  treadMap.anisotropy = 8;

  const treadRoughness = new THREE.CanvasTexture(trRgh.cv);
  treadRoughness.colorSpace = THREE.NoColorSpace;
  treadRoughness.wrapS = treadRoughness.wrapT = THREE.RepeatWrapping;
  treadRoughness.anisotropy = 4;

  const treadNormal = heightFieldToNormalTexture(tH, S, S, 1.4, true);

  // --- 2. Polar-mapped racing slick sidewall (crisp Pirelli-style compound stripe + arched lettering!)
  const sctx = swAlb.ctx;
  const cx = S / 2;
  const cy = S / 2;
  sctx.fillStyle = "#17191f";
  sctx.fillRect(0, 0, S, S);

  // Subtle concentric molded rubber ridges & radial carcass texture
  for (let r = 148; r <= 252; r += 2) {
    const shade = 22 + Math.round(Math.sin(r * 0.45) * 3.5);
    sctx.strokeStyle = `rgb(${shade},${shade + 1},${shade + 3})`;
    sctx.lineWidth = 1.5;
    sctx.beginPath();
    sctx.arc(cx, cy, r, 0, Math.PI * 2);
    sctx.stroke();
  }

  // Rim bead outer shadow ring
  sctx.strokeStyle = "#0b0c10";
  sctx.lineWidth = 10;
  sctx.beginPath();
  sctx.arc(cx, cy, 165, 0, Math.PI * 2);
  sctx.stroke();

  // Signature 2026 Motorsport compound color arc (electric cyan + racing yellow brackets!)
  const drawArcBand = (radius, width, startA, endA, color) => {
    sctx.strokeStyle = color;
    sctx.lineWidth = width;
    sctx.lineCap = "butt";
    sctx.beginPath();
    sctx.arc(cx, cy, radius, startA, endA);
    sctx.stroke();
  };
  for (let k = 0; k < 2; k++) {
    const off = k * Math.PI;
    drawArcBand(206, 9, off + 0.34, off + Math.PI - 0.34, "#19e8d8");
    drawArcBand(206, 5, off - 0.24, off + 0.24, "#ffd426");
  }

  // Crisp arched sidewall typography: "APEX P-ZERO" (top & bottom) + "SOFT · 305/670-18"
  const drawCurvedText = (text, radius, centerAngle, spanPerChar, font, color) => {
    sctx.save();
    sctx.font = font;
    sctx.fillStyle = color;
    sctx.textAlign = "center";
    sctx.textBaseline = "middle";
    const total = (text.length - 1) * spanPerChar;
    for (let i = 0; i < text.length; i++) {
      const ang = centerAngle - total * 0.5 + i * spanPerChar;
      sctx.save();
      sctx.translate(cx + Math.cos(ang) * radius, cy + Math.sin(ang) * radius);
      sctx.rotate(ang + Math.PI / 2);
      sctx.fillText(text[i], 0, 0);
      sctx.restore();
    }
    sctx.restore();
  };
  drawCurvedText("APEX  P-ZERO", 227, -Math.PI / 2, 0.088, "900 18px 'Segoe UI', Arial, sans-serif", "#eef4fa");
  drawCurvedText("APEX  P-ZERO", 227, Math.PI / 2, 0.088, "900 18px 'Segoe UI', Arial, sans-serif", "#eef4fa");
  drawCurvedText("SOFT · SLICK · 305/670-18", 187, 0, 0.060, "700 11px 'Segoe UI', Arial, sans-serif", "#a8b6c6");
  drawCurvedText("SOFT · SLICK · 305/670-18", 187, Math.PI, 0.060, "700 11px 'Segoe UI', Arial, sans-serif", "#a8b6c6");

  // Sidewall normal heightfield (subtle concentric molding rings + raised lettering relief)
  const swH = new Float32Array(S * S);
  const swData = sctx.getImageData(0, 0, S, S).data;
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      const r = Math.hypot(x - cx, y - cy);
      const idx = (y * S + x) * 4;
      const lum = (swData[idx] + swData[idx + 1] + swData[idx + 2]) / 765;
      const ridge = Math.sin(r * 0.5) * 0.06;
      const bevel = r > 235 ? (256 - r) / 21 * 0.15 : 0.15;
      swH[y * S + x] = 0.45 + ridge + bevel + lum * 0.22;
    }
  }

  const sidewallMap = new THREE.CanvasTexture(swAlb.cv);
  sidewallMap.colorSpace = THREE.SRGBColorSpace;
  sidewallMap.anisotropy = 8;

  const sidewallNormal = heightFieldToNormalTexture(swH, S, S, 1.5, false);

  _tireCache = { treadMap, treadNormal, treadRoughness, sidewallMap, sidewallNormal };
  return _tireCache;
}

let _rotorCache = null;
function getBrakeRotorTextures() {
  if (_rotorCache !== null) return _rotorCache;
  const S = 512;
  const alb = makeCanvas(S, S);
  const rgh = makeCanvas(S, S);
  const emi = makeCanvas(S, S);
  if (!alb || !rgh || !emi) {
    _rotorCache = {};
    return _rotorCache;
  }
  const cx = S / 2;
  const cy = S / 2;
  const actx = alb.ctx;
  const rctx = rgh.ctx;
  const ectx = emi.ctx;

  // Base metallic carbon-ceramic disc
  actx.fillStyle = "#363b45";
  actx.fillRect(0, 0, S, S);
  rctx.fillStyle = "rgb(95,95,95)";
  rctx.fillRect(0, 0, S, S);
  ectx.fillStyle = "#000000";
  ectx.fillRect(0, 0, S, S);

  // Swept friction track glow mask (outer annular friction band glows hottest in middle!)
  const glowGrad = ectx.createRadialGradient(cx, cy, 118, cx, cy, 248);
  glowGrad.addColorStop(0.00, "#000000");
  glowGrad.addColorStop(0.14, "#4a4a4a");
  glowGrad.addColorStop(0.45, "#ffffff");
  glowGrad.addColorStop(0.86, "#e8e8e8");
  glowGrad.addColorStop(1.00, "#505050");
  ectx.fillStyle = glowGrad;
  ectx.beginPath();
  ectx.arc(cx, cy, 250, 0, Math.PI * 2);
  ectx.fill();

  // Concentric brushed carbon-ceramic lathe scoring rings
  const rnd = mulberry32(887766);
  for (let r = 122; r <= 248; r += 1.5) {
    const v = 54 + Math.round((rnd() - 0.5) * 28 + Math.sin(r * 0.4) * 8);
    actx.strokeStyle = `rgb(${v},${v + 2},${v + 6})`;
    actx.lineWidth = 1.6;
    actx.beginPath();
    actx.arc(cx, cy, r, 0, Math.PI * 2);
    actx.stroke();

    const rv = 72 + Math.round(rnd() * 44);
    rctx.strokeStyle = `rgb(${rv},${rv},${rv})`;
    rctx.lineWidth = 1.4;
    rctx.beginPath();
    rctx.arc(cx, cy, r, 0, Math.PI * 2);
    rctx.stroke();
  }

  // Inner dark oxidized rotor hat region
  actx.fillStyle = "#1b1e24";
  actx.beginPath();
  actx.arc(cx, cy, 128, 0, Math.PI * 2);
  actx.fill();
  ectx.fillStyle = "#000000";
  ectx.beginPath();
  ectx.arc(cx, cy, 126, 0, Math.PI * 2);
  ectx.fill();

  // 12 curved-arc cross-drilled cooling hole groups + curved thermal slots
  const heights = new Float32Array(S * S);
  heights.fill(0.75);
  const punchCircle = (px, py, rad) => {
    actx.fillStyle = "#08090c";
    actx.beginPath();
    actx.arc(px, py, rad, 0, Math.PI * 2);
    actx.fill();
    rctx.fillStyle = "rgb(240,240,240)";
    rctx.beginPath();
    rctx.arc(px, py, rad + 0.5, 0, Math.PI * 2);
    rctx.fill();
    // Keep cross-drilled holes dark silhouettes against blazing orange-white rotor!
    ectx.fillStyle = "#060606";
    ectx.beginPath();
    ectx.arc(px, py, rad * 0.92, 0, Math.PI * 2);
    ectx.fill();

    const rInt = Math.ceil(rad + 2);
    for (let oy = -rInt; oy <= rInt; oy++) {
      for (let ox = -rInt; ox <= rInt; ox++) {
        const xx = Math.round(px + ox);
        const yy = Math.round(py + oy);
        if (xx >= 0 && xx < S && yy >= 0 && yy < S) {
          const dist = Math.hypot(ox, oy);
          if (dist < rad + 1.5) {
            const k = Math.max(0, Math.min(1, (dist - rad * 0.65) / (rad * 0.55)));
            heights[yy * S + xx] = Math.min(heights[yy * S + xx], 0.1 + 0.65 * k);
          }
        }
      }
    }
  };

  const groups = 12;
  for (let g = 0; g < groups; g++) {
    const baseAng = (g / groups) * Math.PI * 2;
    // 4 staggered cross-drilled holes curving outward
    for (let h = 0; h < 4; h++) {
      const rr = 148 + h * 26;
      const ang = baseAng + h * 0.085;
      punchCircle(cx + Math.cos(ang) * rr, cy + Math.sin(ang) * rr, 5.2);
    }
    // Swept cooling slot between drill arcs
    const slotAng = baseAng + Math.PI / groups;
    actx.strokeStyle = "#101218";
    actx.lineWidth = 4.2;
    actx.lineCap = "round";
    actx.beginPath();
    actx.moveTo(cx + Math.cos(slotAng) * 145, cy + Math.sin(slotAng) * 145);
    actx.lineTo(cx + Math.cos(slotAng + 0.14) * 238, cy + Math.sin(slotAng + 0.14) * 238);
    actx.stroke();

    ectx.strokeStyle = "#1a1a1a";
    ectx.lineWidth = 3.5;
    ectx.lineCap = "round";
    ectx.beginPath();
    ectx.moveTo(cx + Math.cos(slotAng) * 145, cy + Math.sin(slotAng) * 145);
    ectx.lineTo(cx + Math.cos(slotAng + 0.14) * 238, cy + Math.sin(slotAng + 0.14) * 238);
    ectx.stroke();
  }

  // Concentric height scoring for normal map
  for (let y = 0; y < S; y++) {
    for (let x = 0; x < S; x++) {
      const r = Math.hypot(x - cx, y - cy);
      if (r > 128 && r < 248) {
        heights[y * S + x] += Math.sin(r * 1.4) * 0.035;
      }
    }
  }

  const map = new THREE.CanvasTexture(alb.cv);
  map.colorSpace = THREE.SRGBColorSpace;
  map.anisotropy = 8;

  const roughnessMap = new THREE.CanvasTexture(rgh.cv);
  roughnessMap.colorSpace = THREE.NoColorSpace;
  roughnessMap.anisotropy = 4;

  const emissiveMap = new THREE.CanvasTexture(emi.cv);
  emissiveMap.colorSpace = THREE.NoColorSpace;
  emissiveMap.anisotropy = 4;

  const normalMap = heightFieldToNormalTexture(heights, S, S, 2.5, false);

  _rotorCache = { map, normalMap, roughnessMap, emissiveMap };
  return _rotorCache;
}

let _grilleCache = null;
function getRadiatorMeshTextures() {
  if (_grilleCache !== null) return _grilleCache;
  const S = 256;
  const res = makeCanvas(S, S);
  if (!res) {
    _grilleCache = {};
    return _grilleCache;
  }
  const { cv, ctx } = res;
  ctx.fillStyle = "#06070a";
  ctx.fillRect(0, 0, S, S);
  // Crisp titanium honeycomb radiator matrix
  const rad = 10;
  const dx = rad * 1.5;
  const dy = rad * Math.sqrt(3);
  ctx.strokeStyle = "#2b323e";
  ctx.lineWidth = 2.4;
  for (let row = -1; row < S / dy + 2; row++) {
    for (let col = -1; col < S / dx + 2; col++) {
      const x = col * dx;
      const y = row * dy + (col & 1 ? dy * 0.5 : 0);
      ctx.beginPath();
      for (let k = 0; k < 6; k++) {
        const a = (k * Math.PI) / 3;
        const px = x + Math.cos(a) * (rad - 1.2);
        const py = y + Math.sin(a) * (rad - 1.2);
        if (k === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.closePath();
      ctx.stroke();
    }
  }
  const map = new THREE.CanvasTexture(cv);
  map.colorSpace = THREE.SRGBColorSpace;
  map.wrapS = map.wrapT = THREE.RepeatWrapping;
  map.repeat.set(3, 2);
  _grilleCache = { map };
  return _grilleCache;
}

let _beamTexCache = null;
function getBeamTexture() {
  if (_beamTexCache !== null) return _beamTexCache;
  const res = makeCanvas(128, 256);
  if (!res) return null;
  const { cv, ctx } = res;
  // Soft longitudinal + transverse volumetric falloff (y=0 tip near lamp, y=256 far fade)
  const g = ctx.createLinearGradient(0, 0, 0, 256);
  g.addColorStop(0.00, "rgba(255,246,226,0.0)");
  g.addColorStop(0.06, "rgba(255,248,232,0.88)");
  g.addColorStop(0.32, "rgba(240,248,255,0.42)");
  g.addColorStop(0.72, "rgba(220,238,255,0.12)");
  g.addColorStop(1.00, "rgba(200,225,255,0.0)");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 128, 256);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  _beamTexCache = tex;
  return _beamTexCache;
}

let _flameTexCache = null;
function getFlameTexture() {
  if (_flameTexCache !== null) return _flameTexCache;
  const res = makeCanvas(128, 256);
  if (!res) return null;
  const { cv, ctx } = res;
  // Incandescent blue-white core transitioning to fiery amber-orange exhaust shock diamonds
  const g = ctx.createLinearGradient(0, 0, 0, 256);
  g.addColorStop(0.00, "rgba(255,255,255,0.0)");
  g.addColorStop(0.08, "rgba(210,248,255,1.0)");
  g.addColorStop(0.28, "rgba(95,195,255,0.92)");
  g.addColorStop(0.54, "rgba(255,142,36,0.82)");
  g.addColorStop(0.82, "rgba(255,64,14,0.34)");
  g.addColorStop(1.00, "rgba(255,32,0,0.0)");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 128, 256);
  // Subtle supersonic Mach shock-diamond bright bands
  ctx.fillStyle = "rgba(255,255,255,0.35)";
  ctx.fillRect(0, 42, 128, 12);
  ctx.fillRect(0, 88, 128, 10);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  _flameTexCache = tex;
  return _flameTexCache;
}

// ---------------------------------------------------------------------------
// Smooth Spline-Lofting & Airfoil Geometry Engine (NO BOXY PRISMS!)
// ---------------------------------------------------------------------------

/** Smoothly subdivide loft stations via Catmull-Rom interpolation along Z. */
function subdivideStations(ctrl, stepsPerSeg = 2) {
  if (ctrl.length < 2 || stepsPerSeg <= 1) return ctrl;
  const keys = ["z", "cx", "cy", "rx", "ryTop", "ryBot", "nTop", "nBot", "pinch", "skewX"];
  const defaults = { cx: 0, cy: 0, nTop: 2.4, nBot: 3.0, pinch: 0, skewX: 0 };
  const val = (st, k) => (st[k] !== undefined ? st[k] : defaults[k] ?? 0);
  const catmull = (p0, p1, p2, p3, t) => {
    const t2 = t * t;
    const t3 = t2 * t;
    return 0.5 * (
      (2 * p1) +
      (-p0 + p2) * t +
      (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
      (-p0 + 3 * p1 - 3 * p2 + p3) * t3
    );
  };
  const out = [];
  const last = ctrl.length - 1;
  for (let i = 0; i < last; i++) {
    const s0 = ctrl[Math.max(0, i - 1)];
    const s1 = ctrl[i];
    const s2 = ctrl[i + 1];
    const s3 = ctrl[Math.min(last, i + 2)];
    for (let s = 0; s < stepsPerSeg; s++) {
      const t = s / stepsPerSeg;
      const st = {};
      for (const k of keys) {
        let v = catmull(val(s0, k), val(s1, k), val(s2, k), val(s3, k), t);
        if (k === "rx" || k === "ryTop" || k === "ryBot") v = Math.max(0.004, v);
        if (k === "nTop" || k === "nBot") v = Math.max(1.15, Math.min(6.0, v));
        st[k] = v;
      }
      out.push(st);
    }
  }
  out.push(ctrl[last]);
  return out;
}

/**
 * Loft a smooth organic multi-station superellipse/teardrop surface along Z.
 * Produces seamless stitched vertex normals, arc-length V UVs, guaranteed
 * outward triangle winding, and optional rounded front/back endcaps.
 */
function buildLoftedSurface(stations, radialSegs = 24, closedEnds = true) {
  const nS = stations.length;
  const cols = radialSegs + 1;
  const zSpan = Math.abs(stations[nS - 1].z - stations[0].z) || 1;
  const descendingZ = stations[nS - 1].z < stations[0].z;

  // Arc-length normalized longitudinal V coordinate
  const vCoords = new Float32Array(nS);
  let accLen = 0;
  for (let i = 1; i < nS; i++) {
    const a = stations[i - 1];
    const b = stations[i];
    accLen += Math.hypot((b.cx ?? 0) - (a.cx ?? 0), (b.cy ?? 0) - (a.cy ?? 0), b.z - a.z);
    vCoords[i] = accLen;
  }
  const totalLen = accLen || zSpan;
  for (let i = 0; i < nS; i++) vCoords[i] /= totalLen;

  const positions = [];
  const uvs = [];
  const indices = [];

  for (let i = 0; i < nS; i++) {
    const st = stations[i];
    const cx = st.cx ?? 0;
    const cy = st.cy ?? 0;
    const rx = Math.max(0.002, st.rx);
    const ryTop = Math.max(0.002, st.ryTop);
    const ryBot = Math.max(0.002, st.ryBot);
    const nTop = st.nTop ?? 2.4;
    const nBot = st.nBot ?? 3.0;
    const pinch = st.pinch ?? 0;
    const skewX = st.skewX ?? 0;
    const v = st.v !== undefined ? st.v : vCoords[i];

    for (let j = 0; j <= radialSegs; j++) {
      // Seam placed cleanly at bottom center (theta = -PI/2)
      const u = j / radialSegs;
      const theta = -Math.PI * 0.5 + u * Math.PI * 2;
      const ca = Math.cos(theta);
      const sa = Math.sin(theta);
      const isTop = sa >= 0;
      const exp = 2 / (isTop ? nTop : nBot);
      const yOff = isTop
        ? ryTop * Math.pow(sa, exp)
        : -ryBot * Math.pow(-sa, exp);
      const pinchScale = isTop ? (1 - pinch * Math.pow(sa, 1.35)) : 1;
      const hNorm = yOff / Math.max(0.01, ryTop + ryBot);
      const xOff = Math.sign(ca) * rx * pinchScale * Math.pow(Math.abs(ca), exp) + skewX * hNorm;
      positions.push(cx + xOff, cy + yOff, st.z);
      uvs.push(u, v);
    }
  }

  for (let i = 0; i < nS - 1; i++) {
    const rowA = i * cols;
    const rowB = (i + 1) * cols;
    for (let j = 0; j < radialSegs; j++) {
      const a = rowA + j;
      const b = rowB + j;
      const c = rowB + j + 1;
      const d = rowA + j + 1;
      if (descendingZ) {
        indices.push(a, b, c, a, c, d);
      } else {
        indices.push(a, c, b, a, d, c);
      }
    }
  }

  const closeFront = typeof closedEnds === "boolean" ? closedEnds : !!closedEnds.front;
  const closeBack = typeof closedEnds === "boolean" ? closedEnds : !!closedEnds.back;

  if (closeFront) {
    const st = stations[0];
    const dz = descendingZ ? 0.012 : -0.012;
    const cIdx = positions.length / 3;
    positions.push(st.cx ?? 0, (st.cy ?? 0) + (st.ryTop - st.ryBot) * 0.15, st.z + dz);
    uvs.push(0.5, 0.0);
    for (let j = 0; j < radialSegs; j++) {
      if (descendingZ) indices.push(cIdx, j, j + 1);
      else indices.push(cIdx, j + 1, j);
    }
  }
  if (closeBack) {
    const st = stations[nS - 1];
    const dz = descendingZ ? -0.012 : 0.012;
    const cIdx = positions.length / 3;
    const base = (nS - 1) * cols;
    positions.push(st.cx ?? 0, (st.cy ?? 0) + (st.ryTop - st.ryBot) * 0.15, st.z + dz);
    uvs.push(0.5, 1.0);
    for (let j = 0; j < radialSegs; j++) {
      if (descendingZ) indices.push(cIdx, base + j + 1, base + j);
      else indices.push(cIdx, base + j, base + j + 1);
    }
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute("uv", new THREE.Float32BufferAttribute(uvs, 2));
  geo.setIndex(indices);
  geo.computeVertexNormals();

  // Weld seam normals at j = 0 and j = radialSegs for mirror-smooth clearcoat sheen
  const norm = geo.attributes.normal;
  for (let i = 0; i < nS; i++) {
    const i0 = i * cols;
    const i1 = i * cols + radialSegs;
    const nx = norm.getX(i0) + norm.getX(i1);
    const ny = norm.getY(i0) + norm.getY(i1);
    const nz = norm.getZ(i0) + norm.getZ(i1);
    const len = Math.hypot(nx, ny, nz) || 1;
    norm.setXYZ(i0, nx / len, ny / len, nz / len);
    norm.setXYZ(i1, nx / len, ny / len, nz / len);
  }
  norm.needsUpdate = true;
  return geo;
}

/**
 * Build a genuine cambered NACA-style racing airfoil wing lofted along X.
 * Supports F1 gull-wing center spoon dip, swept tips, dihedral, and spanwise twist.
 */
function buildAirfoilWing({
  span = 1.6,
  chord = 0.36,
  tipChord = null,
  thickness = 0.036,
  camber = -0.025, // negative = downforce inverted racing airfoil (arched bottom!)
  aoa = 0.12,      // positive pitches nose down (downforce on front/rear)
  sweep = 0.0,     // rearward sweep at tips (-Z)
  dihedral = 0.0,
  centerDrop = 0.0, // F1 front-wing central spoon dip
  twist = 0.0,
  spanSegs = 20,
  chordSegs = 16,
}) {
  const cTip = tipChord ?? chord;
  const cols = chordSegs + 1;
  const positions = [];
  const uvs = [];
  const indices = [];

  // Closed airfoil profile parameter t in [0..1]: 0=TE -> 0.5=LE -> 1=TE
  const profilePoint = (t, localChord) => {
    const phi = t * Math.PI * 2;
    const xc = 0.5 * (1 + Math.cos(phi)); // 1 at TE -> 0 at LE -> 1 at TE
    const upper = t <= 0.5;
    // NACA symmetric thickness distribution
    const halfT = (thickness / Math.max(0.05, chord)) * 5.0 * (
      0.2969 * Math.sqrt(Math.max(0, xc)) -
      0.1260 * xc -
      0.3516 * xc * xc +
      0.2843 * xc * xc * xc -
      0.1015 * xc * xc * xc * xc
    );
    const yc = camber * 4 * xc * (1 - xc);
    const y = (yc + (upper ? halfT : -halfT)) * localChord;
    const z = (0.5 - xc) * localChord; // +Z leading edge, -Z trailing edge
    return [y, z];
  };

  for (let i = 0; i <= spanSegs; i++) {
    const s = i / spanSegs; // 0..1 across span
    const uSpan = s * 2 - 1; // -1..+1
    const absU = Math.abs(uSpan);
    const x = uSpan * (span * 0.5);
    const localChord = chord + (cTip - chord) * (absU * absU);
    const localAoa = aoa + twist * absU;
    const cosA = Math.cos(localAoa);
    const sinA = Math.sin(localAoa);
    const yOffset = dihedral * absU * absU - centerDrop * Math.exp(-Math.pow(absU * 2.8, 2));
    const zOffset = -sweep * absU * absU;

    for (let j = 0; j <= chordSegs; j++) {
      const t = j / chordSegs;
      const [py, pz] = profilePoint(t, localChord);
      // Pitch around X: positive aoa drops leading edge (+Z) downward for downforce
      const ry = py * cosA - pz * sinA + yOffset;
      const rz = py * sinA + pz * cosA + zOffset;
      positions.push(x, ry, rz);
      uvs.push(s, t);
    }
  }

  for (let i = 0; i < spanSegs; i++) {
    const rA = i * cols;
    const rB = (i + 1) * cols;
    for (let j = 0; j < chordSegs; j++) {
      const a = rA + j;
      const b = rB + j;
      const c = rB + j + 1;
      const d = rA + j + 1;
      indices.push(a, c, b, a, d, c);
    }
  }

  // Close left & right airfoil wingtips
  for (const end of [0, spanSegs]) {
    const base = end * cols;
    let cy = 0;
    let cz = 0;
    for (let j = 0; j < chordSegs; j++) {
      cy += positions[(base + j) * 3 + 1];
      cz += positions[(base + j) * 3 + 2];
    }
    cy /= chordSegs;
    cz /= chordSegs;
    const cIdx = positions.length / 3;
    positions.push(positions[base * 3], cy, cz);
    uvs.push(end === 0 ? 0 : 1, 0.5);
    for (let j = 0; j < chordSegs; j++) {
      if (end === 0) indices.push(cIdx, base + j + 1, base + j);
      else indices.push(cIdx, base + j, base + j + 1);
    }
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute("uv", new THREE.Float32BufferAttribute(uvs, 2));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return geo;
}

/** Build a smoothly curved ribbon decal hugging a 3D flank path (zero floating edges). */
function buildCurvedDecal(pts, halfH, outwardSign = 1) {
  const n = pts.length;
  const pos = [];
  const uv = [];
  const idx = [];
  for (let i = 0; i < n; i++) {
    const u = i / (n - 1);
    const [x, y, z] = pts[i];
    pos.push(x, y - halfH, z, x, y + halfH, z);
    uv.push(u, 0, u, 1);
  }
  for (let i = 0; i < n - 1; i++) {
    const a = i * 2;
    const b = a + 1;
    const c = a + 3;
    const d = a + 2;
    if (outwardSign > 0) idx.push(a, d, c, a, c, b);
    else idx.push(a, c, d, a, b, c);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  return geo;
}

/** Smoothly beveled extruded 2D aero plate (centered on Z=0 before caller rotates/places). */
function buildExtrudedAero(pts2D, depth, bevelSize = 0.005, bevelSegs = 2) {
  const shape = new THREE.Shape();
  shape.moveTo(pts2D[0][0], pts2D[0][1]);
  for (let i = 1; i < pts2D.length; i++) {
    shape.lineTo(pts2D[i][0], pts2D[i][1]);
  }
  shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: Math.max(0.004, depth),
    bevelEnabled: bevelSize > 0,
    bevelThickness: bevelSize,
    bevelSize,
    bevelSegments: bevelSegs,
    curveSegments: 8,
  });
  geo.translate(0, 0, -depth * 0.5);
  geo.computeVertexNormals();
  return geo;
}

/** Merge an array of BufferGeometries (matching attributes: pos/normal/uv) into 1 draw call. */
function buildMergedGeometry(geos) {
  let totalVerts = 0;
  let totalIdx = 0;
  const normalized = geos.map((g) => {
    const geom = g.index ? g : g;
    if (!geom.attributes.normal) geom.computeVertexNormals();
    const vc = geom.attributes.position.count;
    const ic = geom.index ? geom.index.count : vc;
    totalVerts += vc;
    totalIdx += ic;
    return geom;
  });

  const pos = new Float32Array(totalVerts * 3);
  const norm = new Float32Array(totalVerts * 3);
  const uv = new Float32Array(totalVerts * 2);
  const idx = new Uint32Array(totalIdx);

  let vOff = 0;
  let iOff = 0;
  for (const g of normalized) {
    const pAttr = g.attributes.position;
    const nAttr = g.attributes.normal;
    const uAttr = g.attributes.uv;
    const vc = pAttr.count;
    pos.set(pAttr.array, vOff * 3);
    norm.set(nAttr.array, vOff * 3);
    if (uAttr) uv.set(uAttr.array, vOff * 2);
    if (g.index) {
      const src = g.index.array;
      for (let k = 0; k < src.length; k++) idx[iOff + k] = src[k] + vOff;
      iOff += src.length;
    } else {
      for (let k = 0; k < vc; k++) idx[iOff + k] = k + vOff;
      iOff += vc;
    }
    vOff += vc;
    g.dispose();
  }

  const out = new THREE.BufferGeometry();
  out.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  out.setAttribute("normal", new THREE.BufferAttribute(norm, 3));
  out.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  out.setIndex(new THREE.BufferAttribute(idx, 1));
  return out;
}

// ---------------------------------------------------------------------------
// High-Fidelity Racing Wheel: Beveled Slick + 10-Spoke Forged Y-Rim + Glowing Rotor
// ---------------------------------------------------------------------------

/** Build a custom X-axis-revolved racing slick tire (r = 0.33 m, w = 0.31 m)
 *  with 2 material groups: 0 = scrubbed slick tread, 1 = polar-UV sidewalls. */
function buildSlickTireGeometry(radialSegs = 36) {
  // Profile: [axialX, radius, isSidewall] from left rim bead -> left sidewall -> tread -> right sidewall -> right bead
  const prof = [
    [-0.144, 0.208, 1],
    [-0.150, 0.236, 1],
    [-0.156, 0.268, 1],
    [-0.154, 0.296, 1],
    [-0.146, 0.316, 1],
    [-0.132, 0.326, 0],
    [-0.105, 0.330, 0],
    [-0.052, 0.331, 0],
    [0.000, 0.3315, 0],
    [0.052, 0.331, 0],
    [0.105, 0.330, 0],
    [0.132, 0.326, 0],
    [0.146, 0.316, 1],
    [0.154, 0.296, 1],
    [0.156, 0.268, 1],
    [0.150, 0.236, 1],
    [0.144, 0.208, 1],
  ];

  // Build separate ring strips per material group so UVs don't bleed across tread/sidewall border
  const positions = [];
  const uvs = [];
  const indicesTread = [];
  const indicesSide = [];
  const cols = radialSegs + 1;

  // Duplicate boundary rings where sidewall transitions to tread for clean UVs
  const rings = [];
  for (let i = 0; i < prof.length; i++) {
    const p = prof[i];
    if (i > 0 && prof[i - 1][2] !== p[2]) {
      // Push boundary copy for previous strip & new strip
      rings.push({ x: p[0], r: p[1], side: prof[i - 1][2], breakAfter: true });
    }
    rings.push({ x: p[0], r: p[1], side: p[2], breakAfter: false });
  }

  for (let i = 0; i < rings.length; i++) {
    const { x, r, side } = rings[i];
    const vTread = (x + 0.146) / 0.292;
    for (let j = 0; j <= radialSegs; j++) {
      const u = j / radialSegs;
      const a = u * Math.PI * 2;
      const ca = Math.cos(a);
      const sa = Math.sin(a);
      positions.push(x, ca * r, sa * r);
      if (side) {
        // Polar planar UV centered at (0.5, 0.5), radius 0.33 -> 0.5
        const rNorm = (r / 0.331) * 0.495;
        uvs.push(0.5 + ca * rNorm, 0.5 + sa * rNorm);
      } else {
        uvs.push(u * 2.0, Math.max(0, Math.min(1, vTread)));
      }
    }
  }

  for (let i = 0; i < rings.length - 1; i++) {
    if (rings[i].breakAfter) continue;
    const isSide = rings[i].side === 1 && rings[i + 1].side === 1;
    const target = isSide ? indicesSide : indicesTread;
    const rA = i * cols;
    const rB = (i + 1) * cols;
    for (let j = 0; j < radialSegs; j++) {
      const a = rA + j;
      const b = rB + j;
      const c = rB + j + 1;
      const d = rA + j + 1;
      target.push(a, c, b, a, d, c);
    }
  }

  const allIdx = new Uint32Array(indicesTread.length + indicesSide.length);
  allIdx.set(indicesTread, 0);
  allIdx.set(indicesSide, indicesTread.length);

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute("uv", new THREE.Float32BufferAttribute(uvs, 2));
  geo.setIndex(new THREE.BufferAttribute(allIdx, 1));
  geo.addGroup(0, indicesTread.length, 0);
  geo.addGroup(indicesTread.length, indicesSide.length, 1);
  geo.computeVertexNormals();

  // Weld seam normals around circumference
  const norm = geo.attributes.normal;
  for (let i = 0; i < rings.length; i++) {
    const i0 = i * cols;
    const i1 = i * cols + radialSegs;
    const nx = norm.getX(i0) + norm.getX(i1);
    const ny = norm.getY(i0) + norm.getY(i1);
    const nz = norm.getZ(i0) + norm.getZ(i1);
    const len = Math.hypot(nx, ny, nz) || 1;
    norm.setXYZ(i0, nx / len, ny / len, nz / len);
    norm.setXYZ(i1, nx / len, ny / len, nz / len);
  }
  norm.needsUpdate = true;
  return geo;
}

/** Build merged symmetric 10-spoke concave forged Y-rim spokes (both wheel faces). */
function buildForgedYSpokesGeometry() {
  const parts = [];
  const m4 = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const pos = new THREE.Vector3();
  const scl = new THREE.Vector3(1, 1, 1);
  const dir = new THREE.Vector3();
  const yAxis = new THREE.Vector3(0, 1, 0);

  const addStrut = (ax, ay, az, bx, by, bz, wX, wTan) => {
    dir.set(bx - ax, by - ay, bz - az);
    const len = dir.length();
    if (len < 1e-4) return;
    dir.divideScalar(len);
    const g = new THREE.BoxGeometry(wX, len, wTan);
    q.setFromUnitVectors(yAxis, dir);
    pos.set((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5);
    m4.compose(pos, q, scl);
    g.applyMatrix4(m4);
    parts.push(g);
  };

  // Symmetric outer concave faces on both +X and -X so buildWheel() works on all 4 corners!
  for (const side of [-1, 1]) {
    const xHub = side * 0.074;
    const xFork = side * 0.104;
    const xRim = side * 0.128;

    // 5 twin-Y spoke pairs = 10 sculpted spokes per face
    for (let k = 0; k < 5; k++) {
      const a0 = (k / 5) * Math.PI * 2;
      const r0 = 0.056;
      const rFork = 0.112;
      const rTip = 0.200;
      const spread = 0.235; // Y-branch angular half-spread

      const y0 = Math.cos(a0) * r0;
      const z0 = Math.sin(a0) * r0;
      const yf = Math.cos(a0) * rFork;
      const zf = Math.sin(a0) * rFork;

      // Stem from hub boss to Y-junction
      addStrut(xHub, y0, z0, xFork, yf, zf, 0.022, 0.022);

      // Left & right concave Y-arms sweeping to stepped outer rim flange
      for (const bSign of [-1, 1]) {
        const aTip = a0 + bSign * spread;
        const yt = Math.cos(aTip) * rTip;
        const zt = Math.sin(aTip) * rTip;
        addStrut(xFork, yf, zf, xRim, yt, zt, 0.018, 0.0155);
      }
    }

    // Concave machined central hub bowl ring
    const hubRing = new THREE.CylinderGeometry(0.068, 0.076, 0.024, 18);
    hubRing.rotateZ(Math.PI / 2);
    hubRing.translate(side * 0.072, 0, 0);
    parts.push(hubRing);
  }

  return buildMergedGeometry(parts);
}

/** Build double-sided cross-drilled carbon-ceramic brake rotor mesh geometry (centered at x = 0). */
function buildBrakeRotorGeometry() {
  const parts = [];
  const R = 0.168;
  const halfT = 0.0155;

  // +X and -X rotor faces with planar radial UV matching 512×512 cross-drilled texture
  for (const s of [-1, 1]) {
    const face = new THREE.CircleGeometry(R, 32);
    face.rotateY(s > 0 ? Math.PI / 2 : -Math.PI / 2);
    face.translate(s * halfT, 0, 0);
    parts.push(face);
  }
  // Ventilated outer edge rim
  const rim = new THREE.CylinderGeometry(R, R, halfT * 2, 32, 1, true);
  rim.rotateZ(Math.PI / 2);
  parts.push(rim);
  return buildMergedGeometry(parts);
}

let _sharedWheelGeos = null;
function getWheelGeometries() {
  if (_sharedWheelGeos) return _sharedWheelGeos;
  const tire = buildSlickTireGeometry(36);

  // Stepped multi-piece forged rim barrel (closed double-walled LatheGeometry revolved around X)
  // Outer wall runs -y -> +y (+R normal); inner visible dish & barrel runs +y -> -y (-R inward normal!)
  const barrelPts = [
    new THREE.Vector2(0.214, -0.146),
    new THREE.Vector2(0.214, 0.146),
    new THREE.Vector2(0.203, 0.146),
    new THREE.Vector2(0.198, 0.132),
    new THREE.Vector2(0.192, 0.114),
    new THREE.Vector2(0.190, 0.058),
    new THREE.Vector2(0.190, -0.058),
    new THREE.Vector2(0.192, -0.114),
    new THREE.Vector2(0.198, -0.132),
    new THREE.Vector2(0.203, -0.146),
    new THREE.Vector2(0.214, -0.146),
  ];
  const barrel = new THREE.LatheGeometry(barrelPts, 32);
  barrel.rotateZ(Math.PI / 2);

  const spokes = buildForgedYSpokesGeometry();

  // Anodized center-lock hex nut + splined inner cap on both wheel faces
  const nutParts = [];
  for (const s of [-1, 1]) {
    const hex = new THREE.CylinderGeometry(0.036, 0.044, 0.034, 6);
    hex.rotateZ(Math.PI / 2);
    hex.translate(s * 0.088, 0, 0);
    nutParts.push(hex);
    const tip = new THREE.CylinderGeometry(0.022, 0.028, 0.018, 12);
    tip.rotateZ(Math.PI / 2);
    tip.translate(s * 0.106, 0, 0);
    nutParts.push(tip);
  }
  const centerNut = buildMergedGeometry(nutParts);

  const rotor = buildBrakeRotorGeometry();

  // Inner aluminum rotor hat + 10 floating-disc bobbins
  const hatParts = [];
  const hatCore = new THREE.CylinderGeometry(0.086, 0.086, 0.056, 20);
  hatCore.rotateZ(Math.PI / 2);
  hatParts.push(hatCore);
  for (let b = 0; b < 10; b++) {
    const a = (b / 10) * Math.PI * 2;
    const bob = new THREE.CylinderGeometry(0.007, 0.007, 0.042, 6);
    bob.rotateZ(Math.PI / 2);
    bob.translate(0, Math.cos(a) * 0.089, Math.sin(a) * 0.089);
    hatParts.push(bob);
  }
  const rotorHat = buildMergedGeometry(hatParts);

  _sharedWheelGeos = { tire, barrel, spokes, centerNut, rotor, rotorHat };
  return _sharedWheelGeos;
}

/**
 * Build a 2026 AAA racing slick wheel with beveled profile, forged 10-spoke
 * concave Y-rim, anodized center-lock nut, and glowing cross-drilled
 * carbon-ceramic brake rotor.
 * Exposes `holder.userData.setBrakeGlow(heat)` (heat 0..1, zero allocations).
 */
export function buildWheel(opacity = 1) {
  const translucent = opacity < 1;
  const geos = getWheelGeometries();
  const holder = new THREE.Group();

  const std = (color, extra = {}) => new THREE.MeshStandardMaterial({
    color,
    transparent: translucent,
    opacity,
    depthWrite: !translucent,
    ...extra,
  });

  if (translucent) {
    // Lightweight translucent ghost wheel
    const tireMesh = new THREE.Mesh(geos.tire, [
      std(0x202632, { roughness: 0.85, metalness: 0.05 }),
      std(0x283242, { roughness: 0.78, metalness: 0.08 }),
    ]);
    const barrelMesh = new THREE.Mesh(geos.barrel, std(0x4b5c74, { metalness: 0.7, roughness: 0.3 }));
    const spokeMesh = new THREE.Mesh(geos.spokes, std(0x9fb8d6, { metalness: 0.82, roughness: 0.25 }));
    const nutMesh = new THREE.Mesh(geos.centerNut, std(CYAN_HEX, { metalness: 0.65, roughness: 0.25 }));
    holder.add(tireMesh, barrelMesh, spokeMesh, nutMesh);
    holder.userData.setBrakeGlow = () => {};
    return holder;
  }

  const tireTex = getTireTextures();
  const rotorTex = getBrakeRotorTextures();

  const treadMat = std(0xffffff, {
    map: tireTex.treadMap ?? null,
    normalMap: tireTex.treadNormal ?? null,
    normalScale: new THREE.Vector2(0.55, 0.55),
    roughnessMap: tireTex.treadRoughness ?? null,
    roughness: 0.88,
    metalness: 0.03,
    envMapIntensity: 0.65,
  });
  const sidewallMat = std(0xffffff, {
    map: tireTex.sidewallMap ?? null,
    normalMap: tireTex.sidewallNormal ?? null,
    normalScale: new THREE.Vector2(0.45, 0.45),
    roughness: 0.78,
    metalness: 0.04,
    envMapIntensity: 0.75,
  });

  const tireMesh = new THREE.Mesh(geos.tire, [treadMat, sidewallMat]);

  const barrelMat = std(0x222630, {
    metalness: 0.88,
    roughness: 0.24,
    envMapIntensity: 1.25,
  });
  const barrelMesh = new THREE.Mesh(geos.barrel, barrelMat);

  const spokeMat = std(0xadb7c4, {
    metalness: 0.92,
    roughness: 0.17,
    envMapIntensity: 1.35,
  });
  const spokeMesh = new THREE.Mesh(geos.spokes, spokeMat);

  const nutMat = std(CYAN_HEX, {
    metalness: 0.78,
    roughness: 0.22,
    emissive: CYAN_HEX,
    emissiveIntensity: 0.24,
    envMapIntensity: 1.3,
  });
  const nutMesh = new THREE.Mesh(geos.centerNut, nutMat);

  // Per-wheel cloned material for independent front/rear incandescent brake glow
  const discMat = std(0xd8dde6, {
    map: rotorTex.map ?? null,
    normalMap: rotorTex.normalMap ?? null,
    normalScale: new THREE.Vector2(0.8, 0.8),
    roughnessMap: rotorTex.roughnessMap ?? null,
    roughness: 0.42,
    metalness: 0.72,
    emissive: new THREE.Color(0xff2804),
    emissiveMap: rotorTex.emissiveMap ?? null,
    emissiveIntensity: 0.0,
    envMapIntensity: 1.0,
  });
  const discMesh = new THREE.Mesh(geos.rotor, discMat);

  const hatMat = std(0x2a2f3a, { metalness: 0.84, roughness: 0.28 });
  const hatMesh = new THREE.Mesh(geos.rotorHat, hatMat);

  holder.add(tireMesh, barrelMesh, spokeMesh, nutMesh, discMesh, hatMesh);

  // Zero-allocation incandescent blackbody glow ramp (cherry red -> molten orange -> white-hot!)
  holder.userData.setBrakeGlow = (heat) => {
    const h = heat <= 0 ? 0 : heat >= 1 ? 1 : heat;
    discMat.emissiveIntensity = h * h * 4.8;
    discMat.emissive.setRGB(
      0.85 + 0.15 * h,
      0.06 + 0.54 * h * h + 0.14 * h,
      0.01 + 0.30 * h * h * h,
    );
  };

  return holder;
}

// ---------------------------------------------------------------------------
// Live 256×128 OLED Steering-Wheel Telemetry Renderer (throttled ~20 Hz)
// ---------------------------------------------------------------------------
function createOledDisplay() {
  const res = makeCanvas(256, 128);
  if (!res) return null;
  const { cv, ctx } = res;
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;

  let lastDrawTime = -10;
  let lastKey = "";

  const draw = (rpm, speedMs, gear, brake, throttle, wetness, timeSec) => {
    if (timeSec - lastDrawTime < 0.048 && lastDrawTime >= 0) return;
    const kmh = Math.round(Math.max(0, speedMs) * 3.6);
    const rpmRound = Math.round(rpm / 25) * 25;
    const gearStr = gear === 0 ? "R" : gear === "N" ? "N" : String(gear ?? 1);
    const flash = rpm >= 7200 && Math.sin(timeSec * 36) > 0;
    const key = `${gearStr}|${kmh}|${rpmRound}|${brake > 0.15 ? 1 : 0}|${throttle > 0.2 ? 1 : 0}|${wetness > 0.35 ? 1 : 0}|${flash ? 1 : 0}`;
    if (key === lastKey) return;
    lastKey = key;
    lastDrawTime = timeSec;

    // Deep OLED black-slate background
    ctx.fillStyle = flash ? "#1c0810" : "#050910";
    ctx.fillRect(0, 0, 256, 128);

    // Subtle telemetry grid framing
    ctx.strokeStyle = "#142436";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(3, 3, 250, 122);
    ctx.beginPath();
    ctx.moveTo(82, 30);
    ctx.lineTo(82, 122);
    ctx.moveTo(174, 30);
    ctx.lineTo(174, 122);
    ctx.moveTo(6, 28);
    ctx.lineTo(250, 28);
    ctx.stroke();

    // Top segmented progressive RGB RPM bar (18 segments)
    const segs = 18;
    const rpmNorm = Math.max(0, Math.min(1, (rpm - 1400) / 6200));
    const litSegs = Math.round(rpmNorm * segs);
    for (let i = 0; i < segs; i++) {
      const on = flash || i < litSegs;
      ctx.fillStyle = on
        ? (i < 6 ? "#1df068" : i < 12 ? "#ff3b28" : "#19e8ff")
        : "#121a24";
      ctx.fillRect(8 + i * 13.4, 8, 10.6, 13);
    }

    // Center hero GEAR numeral
    ctx.fillStyle = flash ? "#19e8ff" : "#f6faff";
    ctx.font = "900 72px 'Segoe UI', Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(gearStr, 128, 78);
    ctx.fillStyle = "#4d6b85";
    ctx.font = "700 11px 'Segoe UI', Arial, sans-serif";
    ctx.fillText("GEAR", 128, 114);

    // Left panel: SPEED (km/h) + THR/BRK live micro-bars
    ctx.fillStyle = "#eef6ff";
    ctx.font = "900 30px 'Segoe UI', Arial, sans-serif";
    ctx.fillText(String(kmh), 44, 62);
    ctx.fillStyle = "#5c7d99";
    ctx.font = "700 11px 'Segoe UI', Arial, sans-serif";
    ctx.fillText("KM/H", 44, 84);

    // Throttle (green) & Brake (red) micro-gauges
    ctx.fillStyle = "#101a24";
    ctx.fillRect(10, 96, 66, 8);
    ctx.fillRect(10, 108, 66, 8);
    ctx.fillStyle = "#1de868";
    ctx.fillRect(10, 96, Math.round(Math.max(0, Math.min(1, throttle)) * 66), 8);
    ctx.fillStyle = "#ff3224";
    ctx.fillRect(10, 108, Math.round(Math.max(0, Math.min(1, brake)) * 66), 8);

    // Right panel: numeric RPM + mode badge
    ctx.fillStyle = rpm >= 7000 ? "#19e8ff" : "#d8e6f4";
    ctx.font = "800 22px 'Segoe UI', Arial, sans-serif";
    ctx.fillText(String(Math.round(rpm)), 214, 60);
    ctx.fillStyle = "#5c7d99";
    ctx.font = "700 11px 'Segoe UI', Arial, sans-serif";
    ctx.fillText("RPM", 214, 80);

    // Status pill (SHIFT / WET / BRK / APEX)
    const badgeCol = flash ? "#19e8ff" : brake > 0.35 ? "#ff3828" : wetness > 0.35 ? "#26a8ff" : "#153242";
    const badgeTxt = flash ? "SHIFT" : brake > 0.35 ? "BRAKE" : wetness > 0.35 ? "WET" : "APEX";
    ctx.fillStyle = badgeCol;
    ctx.fillRect(184, 96, 60, 20);
    ctx.fillStyle = (flash || brake > 0.35 || wetness > 0.35) ? "#050910" : "#8fdce8";
    ctx.font = "900 12px 'Segoe UI', Arial, sans-serif";
    ctx.fillText(badgeTxt, 214, 107);

    tex.needsUpdate = true;
  };

  // Initial standby telemetry frame
  draw(1200, 0, 1, 0, 0, 0, 0);
  return { tex, draw };
}

// ---------------------------------------------------------------------------
// Hero 2026 Formula / Hypercar Builder
// ---------------------------------------------------------------------------

/**
 * Build the sculpted 2026 AAA formula/hypercar race car.
 *
 * Exposes live hooks on `group.userData`:
 *  • `bodyMeshes` — baked static bodywork meshes for Crumple crash deformation
 *  • `updateSuspension(wheelLocalPos)` — 4 Vector3s in car-local space (zero allocs)
 *  • `updateTelemetry({ steerAngle, rpm, brake, throttle, speed, gear, wetness, timeSec, cameraMode, backfire })`
 *  • `setHeadlights(intensity)` — 0..1+ drives laser nose DRLs & volumetric headlight beams
 */
export function buildCar(colorHex = 0xd9483b, opacity = 1, withWheels = false, deformable = false) {
  const group = new THREE.Group();
  const translucent = opacity < 1;

  // --- Procedural PBR Material Palette --------------------------------------
  const carbonTex = translucent ? {} : getCarbonTextures();
  const liveryTex = translucent ? {} : getLiveryTextures(colorHex);
  const grilleTex = translucent ? {} : getRadiatorMeshTextures();

  const stdMat = (c, extra = {}) => new THREE.MeshStandardMaterial({
    color: c,
    transparent: translucent,
    opacity,
    depthWrite: !translucent,
    ...extra,
  });

  // Main clearcoat metallic livery paint (1024×1024 procedural livery + micro-flake + panel seams)
  const paintLivery = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
    ? new THREE.MeshPhysicalMaterial({
        color: 0xffffff,
        map: liveryTex.map ?? null,
        normalMap: liveryTex.normalMap ?? null,
        normalScale: new THREE.Vector2(0.32, 0.32),
        roughnessMap: liveryTex.roughnessMap ?? null,
        metalness: 0.52,
        roughness: 0.24,
        clearcoat: 1.0,
        clearcoatRoughness: 0.055,
        envMapIntensity: 1.45,
      })
    : stdMat(colorHex, { metalness: 0.48, roughness: 0.26 });

  // Solid metallic clearcoat paint (wings, mirrors, aero accents)
  const paintSolid = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
    ? new THREE.MeshPhysicalMaterial({
        color: colorHex,
        metalness: 0.52,
        roughness: 0.22,
        clearcoat: 1.0,
        clearcoatRoughness: 0.055,
        envMapIntensity: 1.45,
      })
    : stdMat(colorHex, { metalness: 0.48, roughness: 0.26 });

  // Gloss clearcoated 2×2 twill woven carbon fiber (Halo, wings, endplates, mirrors, tub)
  const carbonGloss = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
    ? new THREE.MeshPhysicalMaterial({
        color: 0xffffff,
        map: carbonTex.map ?? null,
        normalMap: carbonTex.normalMap ?? null,
        normalScale: new THREE.Vector2(0.55, 0.55),
        roughnessMap: carbonTex.roughnessMap ?? null,
        metalness: 0.26,
        roughness: 0.32,
        clearcoat: 0.85,
        clearcoatRoughness: 0.12,
        envMapIntensity: 1.15,
      })
    : stdMat(CARBON_DARK, { metalness: 0.30, roughness: 0.42 });

  // Satin structural woven carbon (Venturi floor, bargeboards, diffuser, suspension)
  const carbonSatin = translucent
    ? stdMat(CARBON_DARK, { metalness: 0.25, roughness: 0.48 })
    : stdMat(0xd8dde6, {
        map: carbonTex.map ?? null,
        normalMap: carbonTex.normalMap ?? null,
        normalScale: new THREE.Vector2(0.65, 0.65),
        roughnessMap: carbonTex.roughnessMap ?? null,
        metalness: 0.20,
        roughness: 0.48,
        envMapIntensity: 0.9,
      });

  // Electric cyan metallic motorsport accent
  const accentMat = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
    ? new THREE.MeshPhysicalMaterial({
        color: CYAN_HEX,
        metalness: 0.45,
        roughness: 0.22,
        clearcoat: 0.9,
        clearcoatRoughness: 0.08,
        emissive: CYAN_HEX,
        emissiveIntensity: 0.28,
        envMapIntensity: 1.25,
      })
    : stdMat(CYAN_HEX, { metalness: 0.45, roughness: 0.28, emissive: CYAN_HEX, emissiveIntensity: 0.25 });

  // Heat-blued machined titanium (exhaust pipes, fasteners, aero struts)
  const titaniumMat = stdMat(TITANIUM_HEX, {
    metalness: 0.94,
    roughness: 0.18,
    envMapIntensity: 1.4,
  });
  const heatBlueMat = stdMat(0x526e9e, {
    metalness: 0.92,
    roughness: 0.22,
    envMapIntensity: 1.4,
  });
  const exhaustInnerMat = stdMat(0x160a06, {
    metalness: 0.35,
    roughness: 0.72,
    emissive: new THREE.Color(0xff420c),
    emissiveIntensity: 0.05,
  });

  const radiatorMat = stdMat(0xffffff, {
    map: grilleTex.map ?? null,
    color: translucent ? 0x10141c : 0xd0d8e4,
    metalness: 0.55,
    roughness: 0.65,
  });

  // Dynamic HDR emissive lighting materials (owned by this car instance!)
  const brakeLightMat = stdMat(0x280506, {
    emissive: new THREE.Color(0xff1418),
    emissiveIntensity: 0.9,
    roughness: 0.16,
    metalness: 0.1,
  });
  const rainStrobeMat = stdMat(0x280506, {
    emissive: new THREE.Color(0xff1014),
    emissiveIntensity: 0.9,
    roughness: 0.16,
    metalness: 0.1,
  });
  const drlMat = stdMat(0xeef8ff, {
    emissive: new THREE.Color(0xdcf6ff),
    emissiveIntensity: translucent ? 0.5 : 1.15,
    roughness: 0.12,
    metalness: 0.1,
  });
  const projectorMat = stdMat(0xfff6e2, {
    emissive: new THREE.Color(0xfff2d2),
    emissiveIntensity: translucent ? 0.3 : 0.45,
    roughness: 0.1,
    metalness: 0.15,
  });

  const add = (geo, material, x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0) => {
    const m = new THREE.Mesh(geo, material);
    m.position.set(x, y, z);
    m.rotation.set(rx, ry, rz);
    group.add(m);
    return m;
  };

  // =========================================================================
  // 1. SWEPT AERO MONOCOQUE, DROOP NOSECONE & ROLL-HOOP AIRBOX ENGINE COVER
  // =========================================================================
  const radialSegs = translucent ? 16 : 24;

  // 1a. Forward Sculpted Droop Nosecone & Front Bulkhead (z = +2.34 -> +0.82)
  const noseCtrl = [
    { z: 2.34, cy: -0.025, rx: 0.068, ryTop: 0.036, ryBot: 0.026, nTop: 2.2, nBot: 2.6, pinch: 0.08 },
    { z: 2.12, cy: 0.005, rx: 0.128, ryTop: 0.064, ryBot: 0.044, nTop: 2.3, nBot: 2.9, pinch: 0.12 },
    { z: 1.72, cy: 0.055, rx: 0.185, ryTop: 0.108, ryBot: 0.086, nTop: 2.5, nBot: 3.1, pinch: 0.15 },
    { z: 1.28, cy: 0.105, rx: 0.245, ryTop: 0.158, ryBot: 0.152, nTop: 2.7, nBot: 3.4, pinch: 0.16 },
    { z: 0.82, cy: 0.145, rx: 0.325, ryTop: 0.245, ryBot: 0.258, nTop: 2.8, nBot: 3.6, pinch: 0.18 },
  ];
  add(
    buildLoftedSurface(subdivideStations(noseCtrl, translucent ? 1 : 3), radialSegs, { front: true, back: true }),
    paintLivery,
  );

  // Sculpted carbon nose tip aero vane + pitot sensor
  add(
    buildLoftedSurface([
      { z: 2.355, cy: -0.026, rx: 0.044, ryTop: 0.022, ryBot: 0.018, nTop: 2.2, nBot: 2.4 },
      { z: 2.22, cy: -0.010, rx: 0.102, ryTop: 0.046, ryBot: 0.032, nTop: 2.2, nBot: 2.6 },
    ], 14, true),
    accentMat,
  );

  // 1b. Lower Survival-Cell Carbon Monocoque Keel under cockpit (z = +0.86 -> -0.46)
  const keelCtrl = [
    { z: 0.86, cy: 0.02, rx: 0.320, ryTop: 0.18, ryBot: 0.165, nTop: 3.0, nBot: 3.8, pinch: 0.06 },
    { z: 0.30, cy: 0.01, rx: 0.365, ryTop: 0.15, ryBot: 0.160, nTop: 3.2, nBot: 4.0, pinch: 0.04 },
    { z: -0.46, cy: 0.02, rx: 0.355, ryTop: 0.22, ryBot: 0.165, nTop: 3.2, nBot: 3.8, pinch: 0.08 },
  ];
  add(buildLoftedSurface(subdivideStations(keelCtrl, 2), 18, true), carbonGloss);

  // 1c. Sculpted Left & Right Curved Cockpit Side Coaming Bolsters (leaves true open cockpit bay!)
  for (const s of [-1, 1]) {
    const coamCtrl = [
      { z: 0.86, cx: s * 0.24, cy: 0.28, rx: 0.088, ryTop: 0.105, ryBot: 0.14, nTop: 2.4, nBot: 2.8, skewX: -s * 0.03 },
      { z: 0.42, cx: s * 0.33, cy: 0.31, rx: 0.068, ryTop: 0.115, ryBot: 0.15, nTop: 2.3, nBot: 2.8, skewX: -s * 0.02 },
      { z: -0.05, cx: s * 0.34, cy: 0.33, rx: 0.066, ryTop: 0.130, ryBot: 0.15, nTop: 2.3, nBot: 2.8, skewX: -s * 0.02 },
      { z: -0.44, cx: s * 0.28, cy: 0.38, rx: 0.085, ryTop: 0.165, ryBot: 0.18, nTop: 2.4, nBot: 2.8, skewX: -s * 0.04 },
    ];
    add(buildLoftedSurface(subdivideStations(coamCtrl, 2), 14, true), paintSolid);
  }

  // Sculpted forward cockpit cowl eyebrow fairing (cradles steering column & aero screen)
  add(
    buildLoftedSurface([
      { z: 0.92, cy: 0.33, rx: 0.28, ryTop: 0.075, ryBot: 0.06, nTop: 2.4, nBot: 3.0, pinch: 0.22 },
      { z: 0.68, cy: 0.39, rx: 0.31, ryTop: 0.068, ryBot: 0.05, nTop: 2.2, nBot: 3.2, pinch: 0.26 },
    ], 16, true),
    carbonGloss,
  );

  // 1d. Tall Aerodynamic Roll-Hoop Airbox Snorkel & Sweeping Tapered Engine Cover (z = -0.34 -> -2.14)
  const engineCtrl = [
    { z: -0.34, cy: 0.22, rx: 0.335, ryTop: 0.56, ryBot: 0.34, nTop: 2.1, nBot: 3.6, pinch: 0.52 },
    { z: -0.62, cy: 0.21, rx: 0.340, ryTop: 0.57, ryBot: 0.35, nTop: 2.0, nBot: 3.6, pinch: 0.54 },
    { z: -1.06, cy: 0.16, rx: 0.285, ryTop: 0.44, ryBot: 0.30, nTop: 1.9, nBot: 3.4, pinch: 0.52 },
    { z: -1.56, cy: 0.12, rx: 0.185, ryTop: 0.26, ryBot: 0.25, nTop: 2.1, nBot: 3.0, pinch: 0.40 },
    { z: -1.94, cy: 0.12, rx: 0.125, ryTop: 0.15, ryBot: 0.20, nTop: 2.3, nBot: 2.8, pinch: 0.22 },
    { z: -2.14, cy: 0.13, rx: 0.078, ryTop: 0.085, ryBot: 0.11, nTop: 2.4, nBot: 2.6, pinch: 0.10 },
  ];
  add(
    buildLoftedSurface(subdivideStations(engineCtrl, translucent ? 1 : 3), radialSegs, { front: true, back: true }),
    paintLivery,
  );

  // Recessed Overhead Airbox Intake Scoop Lip + Honeycomb Mesh Mouth (z = -0.33)
  add(
    buildLoftedSurface([
      { z: -0.31, cy: 0.635, rx: 0.145, ryTop: 0.135, ryBot: 0.095, nTop: 1.8, nBot: 2.8, pinch: 0.35 },
      { z: -0.44, cy: 0.640, rx: 0.160, ryTop: 0.142, ryBot: 0.100, nTop: 1.8, nBot: 2.8, pinch: 0.36 },
    ], 16, { front: false, back: true }),
    carbonGloss,
  );
  add(
    buildLoftedSurface([
      { z: -0.335, cy: 0.635, rx: 0.122, ryTop: 0.112, ryBot: 0.078, nTop: 1.8, nBot: 2.8, pinch: 0.34 },
      { z: -0.375, cy: 0.635, rx: 0.115, ryTop: 0.106, ryBot: 0.072, nTop: 1.8, nBot: 2.8, pinch: 0.34 },
    ], 14, true),
    radiatorMat,
  );

  // 1e. Curved Dorsal Shark-Fin Spine + Overhead Broadcast T-Cam Pod
  const finShape = [
    [-0.58, 0.77],
    [-1.12, 0.72],
    [-1.62, 0.61],
    [-1.88, 0.45],
    [-1.86, 0.25],
    [-1.20, 0.36],
    [-0.58, 0.48],
  ];
  const finGeo = buildExtrudedAero(finShape, 0.022, 0.004, 2);
  finGeo.rotateY(-Math.PI / 2);
  add(finGeo, paintSolid);

  // Electric cyan dorsal razor edge along shark fin top
  const finTrim = buildExtrudedAero([
    [-0.59, 0.785],
    [-1.12, 0.735],
    [-1.63, 0.625],
    [-1.88, 0.470],
    [-1.86, 0.440],
    [-1.61, 0.595],
    [-1.11, 0.705],
    [-0.59, 0.755],
  ], 0.026, 0.003, 1);
  finTrim.rotateY(-Math.PI / 2);
  add(finTrim, accentMat);

  // Broadcast T-Cam stem & winglet bar atop roll hoop (y = 0.88, z = -0.56)
  const tStem = buildExtrudedAero([
    [-0.50, 0.75], [-0.62, 0.75], [-0.60, 0.88], [-0.52, 0.88],
  ], 0.036, 0.004, 1);
  tStem.rotateY(-Math.PI / 2);
  add(tStem, carbonGloss);
  add(buildAirfoilWing({
    span: 0.30, chord: 0.075, thickness: 0.024, camber: 0, aoa: 0, spanSegs: 8, chordSegs: 8,
  }), accentMat, 0, 0.885, -0.56);

  // =========================================================================
  // 2. SCULPTED HOURGLASS "COKE-BOTTLE" SIDEPODS + RADIATORS + LOUVERS
  // =========================================================================
  for (const s of [-1, 1]) {
    // Organic sculpted undercut sidepod body (high front lip, bold shoulder, tight Coke-bottle waist!)
    const podCtrl = [
      { z: 0.58, cx: s * 0.52, cy: 0.145, rx: 0.215, ryTop: 0.155, ryBot: 0.105, nTop: 2.6, nBot: 2.4, pinch: 0.14, skewX: -s * 0.04 },
      { z: 0.34, cx: s * 0.56, cy: 0.145, rx: 0.232, ryTop: 0.195, ryBot: 0.165, nTop: 2.5, nBot: 2.8, pinch: 0.16, skewX: -s * 0.05 },
      { z: -0.08, cx: s * 0.54, cy: 0.125, rx: 0.228, ryTop: 0.200, ryBot: 0.220, nTop: 2.4, nBot: 3.2, pinch: 0.18, skewX: -s * 0.05 },
      { z: -0.58, cx: s * 0.45, cy: 0.095, rx: 0.180, ryTop: 0.168, ryBot: 0.200, nTop: 2.3, nBot: 3.0, pinch: 0.20, skewX: -s * 0.04 },
      { z: -1.08, cx: s * 0.30, cy: 0.070, rx: 0.125, ryTop: 0.125, ryBot: 0.160, nTop: 2.2, nBot: 2.8, pinch: 0.18, skewX: -s * 0.02 },
      { z: -1.58, cx: s * 0.17, cy: 0.060, rx: 0.072, ryTop: 0.082, ryBot: 0.105, nTop: 2.2, nBot: 2.6, pinch: 0.12, skewX: 0 },
    ];
    add(
      buildLoftedSurface(subdivideStations(podCtrl, translucent ? 1 : 2), translucent ? 14 : 20, { front: false, back: true }),
      paintLivery,
    );

    // Beveled carbon air intake shroud lip + forward upper visor overbite
    add(
      buildLoftedSurface([
        { z: 0.625, cx: s * 0.52, cy: 0.165, rx: 0.212, ryTop: 0.135, ryBot: 0.078, nTop: 2.8, nBot: 2.5, skewX: -s * 0.03 },
        { z: 0.545, cx: s * 0.53, cy: 0.150, rx: 0.222, ryTop: 0.158, ryBot: 0.105, nTop: 2.6, nBot: 2.5, skewX: -s * 0.03 },
      ], 18, { front: false, back: true }),
      carbonGloss,
    );

    // Recessed honeycomb radiator core inside intake mouth
    add(
      buildLoftedSurface([
        { z: 0.575, cx: s * 0.52, cy: 0.160, rx: 0.182, ryTop: 0.106, ryBot: 0.062, nTop: 2.6, nBot: 2.5 },
        { z: 0.535, cx: s * 0.52, cy: 0.155, rx: 0.178, ryTop: 0.102, ryBot: 0.058, nTop: 2.6, nBot: 2.5 },
      ], 14, true),
      radiatorMat,
    );

    // Cyan horizontal aero flow-conditioner blade across sidepod intake
    add(
      buildAirfoilWing({
        span: 0.36, chord: 0.12, thickness: 0.015, camber: -0.01, aoa: 0.06, spanSegs: 6, chordSegs: 8,
      }),
      accentMat,
      s * 0.52, 0.275, 0.61,
    );

    // Recessed carbon cooling louvers along sidepod top downwash deck (4 angled gills)
    for (let g = 0; g < 4; g++) {
      const gz = 0.04 - g * 0.145;
      const gx = s * (0.52 - g * 0.025);
      const gy = 0.312 - g * 0.018;
      add(
        buildAirfoilWing({
          span: 0.17 - g * 0.014, chord: 0.055, thickness: 0.010, camber: 0, aoa: 0.24, spanSegs: 4, chordSegs: 6,
        }),
        carbonSatin,
        gx, gy, gz, 0, s * 0.16, -s * 0.12,
      );
    }

    // Sculpted sidepod shoulder cyan aero accent fillet
    const streakCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(s * 0.68, 0.25, 0.54),
      new THREE.Vector3(s * 0.74, 0.26, 0.12),
      new THREE.Vector3(s * 0.66, 0.22, -0.45),
      new THREE.Vector3(s * 0.44, 0.16, -1.04),
      new THREE.Vector3(s * 0.25, 0.13, -1.50),
    ]);
    add(new THREE.TubeGeometry(streakCurve, 20, 0.014, 6, false), accentMat);
  }

  // =========================================================================
  // 3. GROUND-EFFECT VENTURI FLOOR, BARGEBOARDS & MULTI-STRAKE DIFFUSER
  // =========================================================================
  // Sculpted planar Venturi floor (wide mid-body, cleanly narrowed ahead of rear slicks!)
  const floorPts = [
    [-0.36, 0.86],
    [0.36, 0.86],
    [0.64, 0.62],
    [0.78, 0.28],
    [0.78, -0.58],
    [0.54, -0.90],
    [0.54, -1.78],
    [0.44, -2.02],
    [-0.44, -2.02],
    [-0.54, -1.78],
    [-0.54, -0.90],
    [-0.78, -0.58],
    [-0.78, 0.28],
    [-0.64, 0.62],
  ];
  const floorGeo = buildExtrudedAero(floorPts, 0.036, 0.008, 2);
  floorGeo.rotateX(Math.PI / 2); // shape (x, y) -> (x, 0, y) so +Y shape maps to +Z forward!
  add(floorGeo, carbonSatin, 0, -0.155, 0);

  // Raised Floor Edge Wings + Sculpted Bargeboard Vortex Generators (L & R)
  for (const s of [-1, 1]) {
    // Swept floor-edge aero winglet strip
    const edgeCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(s * 0.68, -0.125, 0.58),
      new THREE.Vector3(s * 0.80, -0.115, 0.18),
      new THREE.Vector3(s * 0.80, -0.115, -0.54),
      new THREE.Vector3(s * 0.58, -0.125, -0.84),
    ]);
    add(new THREE.TubeGeometry(edgeCurve, 18, 0.014, 6, false), accentMat);

    // Sculpted boomerang bargeboard fence (centered locally at z=0.47 so yaw pivots cleanly!)
    const bbShape = [
      [0.33, -0.15],
      [0.17, 0.08],
      [-0.23, 0.05],
      [-0.33, -0.15],
    ];
    const bbGeo = buildExtrudedAero(bbShape, 0.018, 0.004, 2);
    bbGeo.rotateY(-Math.PI / 2);
    add(bbGeo, carbonGloss, s * 0.735, 0, 0.47, 0, s * 0.08, 0);

    // Secondary lower bargeboard turning vane (centered locally at z=0.50)
    const vaneGeo = buildExtrudedAero([
      [0.22, -0.15], [0.06, -0.02], [-0.16, -0.04], [-0.22, -0.15],
    ], 0.014, 0.003, 1);
    vaneGeo.rotateY(-Math.PI / 2);
    add(vaneGeo, carbonSatin, s * 0.56, 0, 0.50, 0, -s * 0.12, 0);
  }

  // Swept Venturi Rear Diffuser Ramp + 4 Sculpted Carbon Diffuser Strakes
  add(
    buildAirfoilWing({
      span: 1.04, chord: 0.46, tipChord: 0.42, thickness: 0.032, camber: 0.025, aoa: 0.34, spanSegs: 12, chordSegs: 10,
    }),
    carbonSatin,
    0, -0.075, -1.94,
  );
  for (const sx of [-0.46, -0.18, 0.18, 0.46]) {
    const strakeGeo = buildExtrudedAero([
      [-1.70, -0.155],
      [-2.13, -0.135],
      [-2.14, 0.025],
      [-1.74, -0.055],
    ], 0.018, 0.004, 2);
    strakeGeo.rotateY(-Math.PI / 2);
    add(strakeGeo, carbonGloss, sx, 0, 0);
  }

  // =========================================================================
  // 4. MULTI-ELEMENT CAMBERED AIRFOIL FRONT WING + SCULPTED ENDPLATES
  // =========================================================================
  // 4a. Gull-wing cambered Front Wing Mainplane (swept tips, central nose spoon dip!)
  add(
    buildAirfoilWing({
      span: 1.78,
      chord: 0.34,
      tipChord: 0.28,
      thickness: 0.032,
      camber: -0.032,
      aoa: 0.06,
      sweep: 0.14,
      centerDrop: 0.042,
      spanSegs: 22,
      chordSegs: 12,
    }),
    carbonGloss,
    0, -0.095, 2.10,
  );

  // 4b. Secondary Cambered Aero Flap (clearcoat painted!)
  add(
    buildAirfoilWing({
      span: 1.72,
      chord: 0.19,
      tipChord: 0.16,
      thickness: 0.022,
      camber: -0.036,
      aoa: 0.24,
      sweep: 0.14,
      centerDrop: 0.025,
      spanSegs: 20,
      chordSegs: 10,
    }),
    paintSolid,
    0, -0.042, 1.94,
  );

  // 4c. Upper Trim Aero Flap with Cyan Trailing Accent
  add(
    buildAirfoilWing({
      span: 1.64,
      chord: 0.125,
      tipChord: 0.10,
      thickness: 0.016,
      camber: -0.030,
      aoa: 0.38,
      sweep: 0.14,
      centerDrop: 0.015,
      spanSegs: 18,
      chordSegs: 10,
    }),
    accentMat,
    0, 0.018, 1.82,
  );

  // 4d. Sculpted Curved Front Wing Endplates + Outboard Diveplanes + Nose Pylons
  for (const s of [-1, 1]) {
    // Centered locally around z=1.95 so slight outward cant pivots flush on wingtip!
    const fEndShape = [
      [0.26, -0.135],
      [0.20, 0.045],
      [-0.03, 0.125],
      [-0.25, 0.115],
      [-0.27, -0.125],
    ];
    const fEndGeo = buildExtrudedAero(fEndShape, 0.024, 0.005, 2);
    fEndGeo.rotateY(-Math.PI / 2);
    add(fEndGeo, paintSolid, s * 0.892, 0, 1.95, 0, s * 0.04, -s * 0.05);

    // Swept outboard carbon diveplane / canard on front endplate
    add(
      buildAirfoilWing({
        span: 0.055, chord: 0.26, thickness: 0.014, camber: -0.02, aoa: 0.28, spanSegs: 4, chordSegs: 8,
      }),
      carbonGloss,
      s * 0.922, 0.014, 1.92, 0, s * 0.08, s * 0.14,
    );

    // Front wing swan-neck nose attachment pylons
    const fPylonGeo = buildExtrudedAero([
      [2.14, -0.11], [2.16, 0.02], [1.96, 0.05], [1.98, -0.10],
    ], 0.018, 0.004, 1);
    fPylonGeo.rotateY(-Math.PI / 2);
    add(fPylonGeo, carbonGloss, s * 0.115, 0, 0);
  }

  // =========================================================================
  // 5. REAR WING: SWAN-NECK PYLONS, CAMBERED DRS AIRFOILS & LOUVRED ENDPLATES
  // =========================================================================
  // 5a. Cambered Spoon Rear Wing Mainplane (positive aoa pitches trailing edge UP for downforce!)
  add(
    buildAirfoilWing({
      span: 1.46,
      chord: 0.38,
      tipChord: 0.34,
      thickness: 0.036,
      camber: -0.042,
      aoa: 0.20,
      centerDrop: 0.036,
      spanSegs: 20,
      chordSegs: 14,
    }),
    carbonGloss,
    0, 0.725, -1.94,
  );

  // 5b. Upper DRS Aero Flap (clearcoat painted!)
  add(
    buildAirfoilWing({
      span: 1.44,
      chord: 0.20,
      tipChord: 0.18,
      thickness: 0.022,
      camber: -0.032,
      aoa: 0.44,
      centerDrop: 0.018,
      spanSegs: 18,
      chordSegs: 12,
    }),
    paintSolid,
    0, 0.822, -2.075,
  );

  // Center DRS hydraulic actuator pod atop rear wing
  add(
    buildLoftedSurface([
      { z: -1.84, cy: 0.77, rx: 0.030, ryTop: 0.032, ryBot: 0.026 },
      { z: -2.08, cy: 0.81, rx: 0.024, ryTop: 0.028, ryBot: 0.022 },
    ], 12, true),
    carbonGloss,
  );

  // 5c. Lower Bi-Plane Beam Wing (diffuser airflow extractor)
  add(
    buildAirfoilWing({
      span: 1.24,
      chord: 0.24,
      tipChord: 0.20,
      thickness: 0.022,
      camber: -0.035,
      aoa: 0.28,
      spanSegs: 14,
      chordSegs: 10,
    }),
    carbonGloss,
    0, 0.455, -1.98,
  );

  // 5d. Twin Curved Swan-Neck Carbon Rear Wing Pylons (arch UP & OVER mainplane!)
  for (const s of [-1, 1]) {
    const swanShape = [
      [-1.76, 0.16],
      [-1.72, 0.54],
      [-1.69, 0.78],
      [-1.78, 0.815],
      [-1.96, 0.775],
      [-1.96, 0.720],
      [-1.81, 0.745],
      [-1.80, 0.52],
      [-1.88, 0.16],
    ];
    const swanGeo = buildExtrudedAero(swanShape, 0.020, 0.004, 2);
    swanGeo.rotateY(-Math.PI / 2);
    add(swanGeo, carbonGloss, s * 0.105, 0, 0);
  }

  // 5e. Sculpted S-Curve Rear Wing Endplates + Recessed Cooling Louvers + Cyan Trim
  for (const s of [-1, 1]) {
    const rEndShape = [
      [-1.72, 0.86],
      [-2.16, 0.88],
      [-2.17, 0.36],
      [-1.96, 0.28],
      [-1.80, 0.36],
      [-1.74, 0.54],
    ];
    const rEndGeo = buildExtrudedAero(rEndShape, 0.024, 0.006, 2);
    rEndGeo.rotateY(-Math.PI / 2);
    add(rEndGeo, paintSolid, s * 0.735, 0, 0);

    // Upper cyan accent strip on rear endplate edge
    const rEdge = buildExtrudedAero([
      [-1.73, 0.875], [-2.16, 0.895], [-2.16, 0.855], [-1.73, 0.835],
    ], 0.028, 0.003, 1);
    rEdge.rotateY(-Math.PI / 2);
    add(rEdge, accentMat, s * 0.735, 0, 0);

    // 4 sculpted aero vortex louver gills embedded along rear endplate flank
    for (let lv = 0; lv < 4; lv++) {
      const lGeo = new THREE.BoxGeometry(0.032, 0.016, 0.105);
      add(lGeo, carbonSatin, s * 0.735, 0.66 - lv * 0.055, -1.90 - lv * 0.025, 0.36, 0, 0);
    }
  }

  // =========================================================================
  // 6. CURVED TEARDROP TITANIUM-CARBON HALO & AERODYNAMIC WING MIRRORS
  // =========================================================================
  const haloCurve = new THREE.CatmullRomCurve3([
    new THREE.Vector3(-0.275, 0.555, -0.34),
    new THREE.Vector3(-0.345, 0.660, 0.16),
    new THREE.Vector3(-0.215, 0.715, 0.62),
    new THREE.Vector3(0.000, 0.742, 0.80),
    new THREE.Vector3(0.215, 0.715, 0.62),
    new THREE.Vector3(0.345, 0.660, 0.16),
    new THREE.Vector3(0.275, 0.555, -0.34),
  ]);
  add(new THREE.TubeGeometry(haloCurve, translucent ? 22 : 36, 0.029, 10, false), carbonGloss);

  // Central front aerodynamic teardrop Halo pillar (only scale X around x=0 so Y/Z stay locked to Halo apex!)
  const pillarCurve = new THREE.CatmullRomCurve3([
    new THREE.Vector3(0, 0.742, 0.80),
    new THREE.Vector3(0, 0.585, 0.885),
    new THREE.Vector3(0, 0.375, 0.94),
  ]);
  const pillarGeo = new THREE.TubeGeometry(pillarCurve, 12, 0.024, 8, false);
  pillarGeo.scale(0.65, 1.0, 1.0);
  add(pillarGeo, carbonGloss);

  // Subtle cyan Halo crest aero trim
  const haloAccent = new THREE.CatmullRomCurve3([
    new THREE.Vector3(-0.18, 0.742, 0.64),
    new THREE.Vector3(0.00, 0.770, 0.805),
    new THREE.Vector3(0.18, 0.742, 0.64),
  ]);
  add(new THREE.TubeGeometry(haloAccent, 14, 0.007, 6, false), accentMat);

  // Aerodynamic Wing Mirrors on Twin Curved Carbon Stalks
  for (const s of [-1, 1]) {
    // Primary lower stalk + outer aero stay
    const stalk1 = new THREE.CatmullRomCurve3([
      new THREE.Vector3(s * 0.36, 0.36, 0.52),
      new THREE.Vector3(s * 0.48, 0.43, 0.53),
      new THREE.Vector3(s * 0.56, 0.45, 0.52),
    ]);
    add(new THREE.TubeGeometry(stalk1, 10, 0.009, 6, false), carbonGloss);
    const stalk2 = new THREE.CatmullRomCurve3([
      new THREE.Vector3(s * 0.54, 0.31, 0.50),
      new THREE.Vector3(s * 0.56, 0.44, 0.51),
    ]);
    add(new THREE.TubeGeometry(stalk2, 8, 0.0075, 6, false), carbonGloss);

    // Sculpted teardrop clearcoat mirror housing
    add(
      buildLoftedSurface([
        { z: 0.58, cx: s * 0.58, cy: 0.46, rx: 0.045, ryTop: 0.028, ryBot: 0.024, nTop: 2.2, nBot: 2.4 },
        { z: 0.53, cx: s * 0.59, cy: 0.46, rx: 0.088, ryTop: 0.042, ryBot: 0.036, nTop: 2.4, nBot: 2.6 },
        { z: 0.47, cx: s * 0.59, cy: 0.46, rx: 0.082, ryTop: 0.038, ryBot: 0.034, nTop: 2.6, nBot: 2.8 },
      ], 14, { front: true, back: false }),
      paintSolid,
    );
    // Reflective chrome mirror glass facing rearward (-Z)
    add(new THREE.CircleGeometry(0.034, 14), titaniumMat, s * 0.59, 0.46, 0.472, 0, Math.PI, 0);
  }

  // =========================================================================
  // 7. TWIN HEAT-BLUED TITANIUM EXHAUST PIPES
  // =========================================================================
  for (const s of [-1, 1]) {
    // Outer heat-blued titanium sleeve
    const exOuter = new THREE.CylinderGeometry(0.046, 0.052, 0.24, 16, 1, true);
    exOuter.rotateX(Math.PI / 2);
    add(exOuter, heatBlueMat, s * 0.095, 0.215, -2.02);
    // Machined metallic beveled lip ring
    const exLip = new THREE.TorusGeometry(0.046, 0.006, 8, 16);
    add(exLip, titaniumMat, s * 0.095, 0.215, -2.14);
    // Glowing recessed exhaust throat
    const exIn = new THREE.CylinderGeometry(0.039, 0.042, 0.20, 14);
    exIn.rotateX(Math.PI / 2);
    add(exIn, exhaustInnerMat, s * 0.095, 0.215, -2.01);
  }

  // =========================================================================
  // 8. HDR EMISSIVE REAR LED LIGHT BLADES, FIA RAIN STROBE & LASER NOSE DRLs
  // =========================================================================
  // Twin vertical razor-sharp rear wing endplate LED light blades
  for (const s of [-1, 1]) {
    const blade = new THREE.BoxGeometry(0.028, 0.38, 0.026);
    add(blade, brakeLightMat, s * 0.738, 0.59, -2.162);
  }
  // High-mount horizontal rear beam LED light bar
  add(new THREE.BoxGeometry(0.56, 0.026, 0.028), brakeLightMat, 0, 0.455, -2.11);
  // Central FIA rear crash-structure rain-light strobe LED array
  add(new THREE.BoxGeometry(0.088, 0.155, 0.042), rainStrobeMat, 0, 0.115, -2.152);
  add(new THREE.BoxGeometry(0.112, 0.180, 0.032), carbonGloss, 0, 0.115, -2.138);

  // Swept angular nose laser DRL eyebrow blades + twin recessed LED projector pods
  for (const s of [-1, 1]) {
    // Sleek carbon headlight intake cheek bezel
    const pod = buildLoftedSurface([
      { z: 1.66, cx: s * 0.195, cy: 0.145, rx: 0.048, ryTop: 0.028, ryBot: 0.022 },
      { z: 1.46, cx: s * 0.228, cy: 0.165, rx: 0.056, ryTop: 0.034, ryBot: 0.026 },
    ], 12, true);
    add(pod, carbonGloss);
    // Razor-sharp laser DRL eyebrow strip
    const drlCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(s * 0.165, 0.162, 1.675),
      new THREE.Vector3(s * 0.210, 0.172, 1.620),
      new THREE.Vector3(s * 0.245, 0.184, 1.500),
    ]);
    add(new THREE.TubeGeometry(drlCurve, 10, 0.0085, 6, false), drlMat);
    // Recessed projector lamp lens
    const proj = new THREE.SphereGeometry(0.024, 10, 8);
    proj.scale(1.1, 0.75, 0.65);
    add(proj, projectorMat, s * 0.205, 0.148, 1.662);
  }

  // =========================================================================
  // 9. CRISP MOTORSPORT SPONSOR DECALS (curved skin-tight & left-to-right!)
  // =========================================================================
  if (!translucent) {
    const addDecal = (row, w, h, x, y, z, rx = 0, ry = 0, rz = 0) => {
      const m = makeSponsorMaterial(row, opacity);
      if (m) add(new THREE.PlaneGeometry(w, h), m, x, y, z, rx, ry, rz);
    };
    // Curved skin-tight sidepod flank decals wrapping Coke-bottle contour cleanly
    const mLeft = makeSponsorMaterial(0, opacity);
    if (mLeft) {
      add(buildCurvedDecal([
        [0.798, 0.145, 0.36],
        [0.792, 0.135, 0.08],
        [0.742, 0.120, -0.24],
        [0.640, 0.100, -0.58],
      ], 0.085, 1), mLeft);
    }
    const mRight = makeSponsorMaterial(1, opacity);
    if (mRight) {
      add(buildCurvedDecal([
        [-0.640, 0.100, -0.58],
        [-0.742, 0.120, -0.24],
        [-0.792, 0.135, 0.08],
        [-0.798, 0.145, 0.36],
      ], 0.085, 1), mRight);
    }
    // Rear wing DRS flap rear face (tilted to face chase cam, reading cleanly left-to-right!)
    addDecal(3, 1.18, 0.18, 0, 0.828, -2.142, 0.42, Math.PI, 0);
    // Nose cone crest (hugs curved droop nose slope, upright from behind/above)
    addDecal(2, 0.34, 0.085, 0, 0.227, 1.46, -Math.PI / 2 + 0.225, 0, Math.PI);
  }

  // =========================================================================
  // CRUMPLE SOFT-BODY BAKE (MUST HAPPEN BEFORE DYNAMIC SUB-GROUPS!)
  // Guarantees every mesh in `bodyMeshes` owns a unique geometry baked in group space.
  // =========================================================================
  if (deformable) {
    const bodyMeshes = [];
    const seenGeos = new Set();
    for (const child of group.children) {
      if (!child.isMesh) continue;
      if (seenGeos.has(child.geometry)) {
        child.geometry = child.geometry.clone();
      }
      seenGeos.add(child.geometry);
      child.updateMatrix();
      child.geometry.applyMatrix4(child.matrix);
      child.position.set(0, 0, 0);
      child.rotation.set(0, 0, 0);
      child.scale.set(1, 1, 1);
      child.updateMatrix();
      bodyMeshes.push(child);
    }
    group.userData.bodyMeshes = bodyMeshes;
  }

  // =========================================================================
  // DYNAMIC SUB-GROUPS (attached AFTER Crumple bake!)
  // =========================================================================

  // --- A. Curved Tinted Polycarbonate Aero Screen (wraps cleanly under Halo)
  {
    const segU = 18;
    const segV = 5;
    const pos = [];
    const uv = [];
    const idx = [];
    for (let iv = 0; iv <= segV; iv++) {
      const v = iv / segV; // 0 = cowl base -> 1 = upper lip under Halo
      for (let iu = 0; iu <= segU; iu++) {
        const u = iu / segU; // 0..1 across -X..+X
        const a = (u - 0.5) * Math.PI * 0.92; // -83°..+83° around nose front
        const rX = 0.315 - v * 0.045;
        const rZ = 0.42 + v * 0.11;
        const x = Math.sin(a) * rX;
        const z = 0.26 + Math.cos(a) * rZ;
        const y = 0.405 + v * (0.215 + 0.04 * Math.cos(a));
        pos.push(x, y, z);
        uv.push(u, v);
      }
    }
    const cols = segU + 1;
    for (let iv = 0; iv < segV; iv++) {
      for (let iu = 0; iu < segU; iu++) {
        const a = iv * cols + iu;
        const b = a + 1;
        const c = a + cols + 1;
        const d = a + cols;
        idx.push(a, b, c, a, c, d);
      }
    }
    const screenGeo = new THREE.BufferGeometry();
    screenGeo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    screenGeo.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
    screenGeo.setIndex(idx);
    screenGeo.computeVertexNormals();

    const screenMat = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
      ? new THREE.MeshPhysicalMaterial({
          color: 0x0b2234,
          metalness: 0.12,
          roughness: 0.06,
          clearcoat: 1.0,
          clearcoatRoughness: 0.04,
          transparent: true,
          opacity: 0.34,
          depthWrite: false,
          side: THREE.DoubleSide,
          envMapIntensity: 1.5,
        })
      : new THREE.MeshStandardMaterial({
          color: 0x163854,
          transparent: true,
          opacity: opacity * 0.3,
          depthWrite: false,
          side: THREE.DoubleSide,
        });
    const screenMesh = new THREE.Mesh(screenGeo, screenMat);
    group.add(screenMesh);
  }

  // --- B. Sculpted Driver, HANS Collar & Racing Helmet with Iridescent Visor
  // Automatically hidden when `cameraMode === "cockpit"` so camera never clips!
  const driverGroup = new THREE.Group();
  {
    // Race suit shoulders + upper chest tucked in carbon bucket seat
    const suitMat = stdMat(0x1b202a, { roughness: 0.84, metalness: 0.05 });
    const torso = new THREE.Mesh(
      buildLoftedSurface([
        { z: 0.05, cy: 0.24, rx: 0.22, ryTop: 0.08, ryBot: 0.08 },
        { z: -0.15, cy: 0.27, rx: 0.26, ryTop: 0.11, ryBot: 0.09 },
        { z: -0.32, cy: 0.32, rx: 0.21, ryTop: 0.12, ryBot: 0.08 },
      ], 14, true),
      suitMat,
    );
    driverGroup.add(torso);

    // Carbon HANS neck-support collar yoke + racing harness straps
    const hansCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(-0.11, 0.33, 0.00),
      new THREE.Vector3(-0.12, 0.39, -0.16),
      new THREE.Vector3(0.00, 0.43, -0.27),
      new THREE.Vector3(0.12, 0.39, -0.16),
      new THREE.Vector3(0.11, 0.33, 0.00),
    ]);
    driverGroup.add(new THREE.Mesh(new THREE.TubeGeometry(hansCurve, 16, 0.022, 8, false), carbonGloss));
    for (const s of [-1, 1]) {
      const strap = new THREE.Mesh(new THREE.BoxGeometry(0.044, 0.012, 0.22), paintSolid);
      strap.position.set(s * 0.095, 0.355, -0.02);
      strap.rotation.x = 0.26;
      driverGroup.add(strap);
    }

    // Sculpted Aero Racing Helmet (smooth Lathe profile + rear aero spoiler!)
    const helmetPts = [
      new THREE.Vector2(0.001, -0.135),
      new THREE.Vector2(0.095, -0.132),
      new THREE.Vector2(0.135, -0.095),
      new THREE.Vector2(0.148, -0.030),
      new THREE.Vector2(0.152, 0.032),
      new THREE.Vector2(0.140, 0.095),
      new THREE.Vector2(0.105, 0.138),
      new THREE.Vector2(0.052, 0.156),
      new THREE.Vector2(0.001, 0.160),
    ];
    const helmetGeo = new THREE.LatheGeometry(helmetPts, 22);
    helmetGeo.scale(0.94, 1.0, 1.16);
    const helmetMat = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
      ? new THREE.MeshPhysicalMaterial({
          color: 0xf4f7fc,
          metalness: 0.32,
          roughness: 0.16,
          clearcoat: 1.0,
          clearcoatRoughness: 0.05,
          envMapIntensity: 1.4,
        })
      : stdMat(0xe8eef6, { metalness: 0.35, roughness: 0.22 });
    const helmet = new THREE.Mesh(helmetGeo, helmetMat);
    helmet.position.set(0, 0.505, -0.145);
    helmet.rotation.x = 0.08;
    driverGroup.add(helmet);

    // Helmet racing stripe & rear aerodynamic ducktail spoiler
    const hStripe = new THREE.Mesh(new THREE.BoxGeometry(0.048, 0.03, 0.25), paintSolid);
    hStripe.position.set(0, 0.648, -0.155);
    driverGroup.add(hStripe);
    const hSpoil = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.022, 0.075), accentMat);
    hSpoil.position.set(0, 0.575, -0.30);
    hSpoil.rotation.x = 0.28;
    driverGroup.add(hSpoil);

    // Metallic Iridescent Curved Racing Visor
    const visorGeo = new THREE.SphereGeometry(
      0.156, 18, 10,
      -Math.PI * 0.34, Math.PI * 0.68,
      Math.PI * 0.34, Math.PI * 0.23,
    );
    visorGeo.scale(0.94, 1.0, 1.16);
    const visorMat = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
      ? new THREE.MeshPhysicalMaterial({
          color: 0x0a1c2e,
          metalness: 0.88,
          roughness: 0.06,
          clearcoat: 1.0,
          clearcoatRoughness: 0.03,
          iridescence: 0.92,
          iridescenceIOR: 1.38,
          emissive: 0x0a3848,
          emissiveIntensity: 0.22,
          envMapIntensity: 1.6,
        })
      : stdMat(0x0c2438, { metalness: 0.9, roughness: 0.1 });
    const visor = new THREE.Mesh(visorGeo, visorMat);
    visor.position.set(0, 0.508, -0.136);
    visor.rotation.y = Math.PI / 2; // center visor facing forward (+Z)
    driverGroup.add(visor);
  }
  group.add(driverGroup);

  // --- C. Sculpted Cockpit F1 Butterfly Steering Wheel + 256×128 OLED + 15 RGB Shift LEDs
  const wheelRig = new THREE.Group();
  // Positioned cleanly inside the lower-center of cockpit view beneath the Halo!
  wheelRig.position.set(0, 0.69, 0.72);
  wheelRig.rotation.x = -0.34;

  // Steering column shaft disappearing cleanly into forward bulkhead
  const colShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, 0.36, 10), carbonSatin);
  colShaft.rotation.x = Math.PI / 2;
  colShaft.position.set(0, -0.01, 0.18);
  wheelRig.add(colShaft);

  const steerWheel = new THREE.Group();
  const shiftLeds = [];
  let oled = null;

  {
    // Sculpted beveled butterfly carbon steering wheel faceplate
    const swPts = [
      [-0.145, 0.062], [-0.075, 0.078], [0.075, 0.078], [0.145, 0.062],
      [0.162, 0.015], [0.155, -0.056], [0.115, -0.078], [0.062, -0.058],
      [-0.062, -0.058], [-0.115, -0.078], [-0.155, -0.056], [-0.162, 0.015],
    ];
    const swPlateGeo = buildExtrudedAero(swPts, 0.020, 0.005, 2);
    steerWheel.add(new THREE.Mesh(swPlateGeo, carbonGloss));

    // Ergonomic sculpted left & right Alcantara/rubber hand grips + rear carbon shift paddles
    const gripMat = stdMat(0x1b1e25, { roughness: 0.94, metalness: 0.04 });
    for (const s of [-1, 1]) {
      const gCurve = new THREE.CatmullRomCurve3([
        new THREE.Vector3(s * 0.132, 0.058, -0.003),
        new THREE.Vector3(s * 0.152, 0.012, -0.006),
        new THREE.Vector3(s * 0.144, -0.042, -0.004),
        new THREE.Vector3(s * 0.116, -0.068, -0.002),
      ]);
      steerWheel.add(new THREE.Mesh(new THREE.TubeGeometry(gCurve, 12, 0.0175, 8, false), gripMat));

      // Rear carbon shift paddle
      const paddle = new THREE.Mesh(new THREE.BoxGeometry(0.058, 0.026, 0.006), carbonSatin);
      paddle.position.set(s * 0.095, 0.015, 0.022);
      paddle.rotation.y = s * 0.18;
      steerWheel.add(paddle);
    }

    // 6 multi-color miniature anodized rotary encoders & pushbuttons (facing driver -Z)
    const knobColors = [0x19e8d8, 0xff3228, 0xffcc22, 0xf2f6fa, 0x1de868, 0xff7a18];
    const knobPos = [
      [0.096, 0.026], [0.096, -0.018], [0.055, -0.042],
      [-0.096, 0.026], [-0.096, -0.018], [-0.055, -0.042],
    ];
    const knobGeo = new THREE.CylinderGeometry(0.0085, 0.0095, 0.012, 10);
    knobGeo.rotateX(Math.PI / 2);
    for (let i = 0; i < knobPos.length; i++) {
      const km = stdMat(knobColors[i], { metalness: 0.65, roughness: 0.25 });
      const knob = new THREE.Mesh(knobGeo, km);
      knob.position.set(knobPos[i][0], knobPos[i][1], -0.015);
      steerWheel.add(knob);
    }

    // Live 256×128 OLED Telemetry Display (player car only; ghost uses lightweight static screen)
    oled = translucent ? null : createOledDisplay();
    const oledMat = oled
      ? new THREE.MeshBasicMaterial({ map: oled.tex })
      : stdMat(0x091a26, { emissive: CYAN_HEX, emissiveIntensity: 0.45 });
    const oledScreen = new THREE.Mesh(new THREE.PlaneGeometry(0.132, 0.066), oledMat);
    // Rotate Y by PI so normal faces -Z (driver eye) & UV u:0->1 runs screen-left -> screen-right!
    oledScreen.rotation.y = Math.PI;
    oledScreen.position.set(0, -0.002, -0.0165);
    steerWheel.add(oledScreen);

    // Subtle recessed carbon bezel around OLED
    const bezel = new THREE.Mesh(new THREE.BoxGeometry(0.140, 0.074, 0.006), carbonSatin);
    bezel.position.set(0, -0.002, -0.013);
    steerWheel.add(bezel);

    // 15 Progressive Glowing RGB Shift LEDs across top wheel brow
    // Positioned screen-left (+X) to screen-right (-X) so RPM climbs naturally left-to-right!
    const ledGeo = new THREE.BoxGeometry(0.0105, 0.0115, 0.008);
    for (let k = 0; k < 15; k++) {
      const col = k < 5 ? 0x1af064 : k < 10 ? 0xff3624 : 0x19e8ff;
      const lm = stdMat(0x12161e, {
        emissive: new THREE.Color(col),
        emissiveIntensity: 0.06,
        roughness: 0.18,
        metalness: 0.1,
      });
      const led = new THREE.Mesh(ledGeo, lm);
      const t = (k / 14) - 0.5; // -0.5 left .. +0.5 right
      const archY = 0.056 - t * t * 0.022;
      led.position.set((7 - k) * 0.0138, archY, -0.0175);
      steerWheel.add(led);
      shiftLeds.push({ mat: lm, threshold: 2800 + k * 310 });
    }
  }
  wheelRig.add(steerWheel);
  group.add(wheelRig);

  // --- D. Articulated Double-Wishbone (True V-Triangles!) + Pushrod Carbon Suspension
  // Plus non-spinning carbon uprights, aero brake ducts & 6-piston Brembo calipers!
  const suspRigs = [];
  if (!withWheels) {
    const suspGroup = new THREE.Group();
    // Aero-oval carbon wishbone blade (unit length along +Z)
    const wishboneGeo = new THREE.CylinderGeometry(0.011, 0.012, 1, 8);
    wishboneGeo.scale(1.25, 0.62, 1.0);
    wishboneGeo.rotateX(Math.PI / 2);

    // Circular carbon pushrod (unit length along +Z)
    const pushrodGeo = new THREE.CylinderGeometry(0.009, 0.009, 1, 8);
    pushrodGeo.rotateX(Math.PI / 2);

    // Sculpted 6-piston racing caliper (curved anodized block hugging 0.168m rotor, clearing 0.190m barrel & ±0.063m spokes)
    const caliperGeo = buildExtrudedAero([
      [-0.140, -0.058], [-0.172, -0.042], [-0.174, 0.042],
      [-0.140, 0.058], [-0.116, 0.032], [-0.116, -0.032],
    ], 0.074, 0.006, 2);
    caliperGeo.rotateY(Math.PI / 2); // width along X ([-0.043..+0.043]), profile in Y-Z plane!

    const caliperMat = stdMat(colorHex, {
      metalness: 0.62,
      roughness: 0.24,
      emissive: colorHex,
      emissiveIntensity: 0.08,
    });

    // Sculpted inboard carbon wheel upright & aero brake cooling duct scoop
    const uprightGeo = new THREE.CylinderGeometry(0.068, 0.074, 0.045, 14);
    uprightGeo.rotateZ(Math.PI / 2);
    const ductGeo = new THREE.BoxGeometry(0.048, 0.14, 0.095);

    // Order matches SimStateV1: 0 FL (-X,+Z), 1 FR (+X,+Z), 2 RL (-X,-Z), 3 RR (+X,-Z)
    const hardpoints = [
      {
        side: -1, front: true,
        uF: [-0.21, 0.155, 1.48], uR: [-0.26, 0.145, 1.12],
        lF: [-0.19, -0.105, 1.46], lR: [-0.25, -0.110, 1.10],
        pr: [-0.24, 0.205, 1.24],
      },
      {
        side: 1, front: true,
        uF: [0.21, 0.155, 1.48], uR: [0.26, 0.145, 1.12],
        lF: [0.19, -0.105, 1.46], lR: [0.25, -0.110, 1.10],
        pr: [0.24, 0.205, 1.24],
      },
      {
        side: -1, front: false,
        uF: [-0.26, 0.125, -1.06], uR: [-0.18, 0.130, -1.52],
        lF: [-0.25, -0.115, -1.08], lR: [-0.17, -0.110, -1.54],
        pr: [-0.20, 0.195, -1.24],
      },
      {
        side: 1, front: false,
        uF: [0.26, 0.125, -1.06], uR: [0.18, 0.130, -1.52],
        lF: [0.25, -0.115, -1.08], lR: [0.17, -0.110, -1.54],
        pr: [0.20, 0.195, -1.24],
      },
    ];

    for (const hp of hardpoints) {
      const upF = new THREE.Mesh(wishboneGeo, carbonSatin);
      const upR = new THREE.Mesh(wishboneGeo, carbonSatin);
      const loF = new THREE.Mesh(wishboneGeo, carbonSatin);
      const loR = new THREE.Mesh(wishboneGeo, carbonSatin);
      const push = new THREE.Mesh(pushrodGeo, carbonGloss);
      suspGroup.add(upF, upR, loF, loR, push);

      // Non-spinning wheel upright + caliper sub-group centered at wheel center
      const hubNode = new THREE.Group();
      const upright = new THREE.Mesh(uprightGeo, carbonSatin);
      upright.position.set(-hp.side * 0.165, 0, 0);
      const duct = new THREE.Mesh(ductGeo, carbonGloss);
      duct.position.set(-hp.side * 0.188, 0.025, 0.045);
      const caliper = new THREE.Mesh(caliperGeo, caliperMat);
      // Mount caliper on inboard-facing side of brake rotor (rear side on fronts, front side on rears)
      if (!hp.front) caliper.rotation.y = Math.PI;
      hubNode.add(upright, duct, caliper);
      suspGroup.add(hubNode);

      suspRigs.push({
        side: hp.side,
        front: hp.front,
        upF, upR, loF, loR, push, hubNode,
        uFx: hp.uF[0], uFy: hp.uF[1], uFz: hp.uF[2],
        uRx: hp.uR[0], uRy: hp.uR[1], uRz: hp.uR[2],
        lFx: hp.lF[0], lFy: hp.lF[1], lFz: hp.lF[2],
        lRx: hp.lR[0], lRy: hp.lR[1], lRz: hp.lR[2],
        prx: hp.pr[0], pry: hp.pr[1], prz: hp.pr[2],
      });
    }
    group.add(suspGroup);
  }

  // --- E. Dynamic Dusk/Night Volumetric Headlight Beam Cones (player car only)
  const headlightBeamGroup = new THREE.Group();
  let beamMat = null;
  if (!translucent) {
    const beamTex = getBeamTexture();
    beamMat = new THREE.MeshBasicMaterial({
      map: beamTex,
      color: 0xf4f8ff,
      transparent: true,
      opacity: 0.0,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    const coneLen = 11.5;
    const coneGeo = new THREE.CylinderGeometry(0.06, 1.38, coneLen, 16, 1, true);
    // Align narrow tip at z=0, flaring along +Z
    coneGeo.translate(0, -coneLen * 0.5, 0);
    coneGeo.rotateX(-Math.PI / 2);
    for (const s of [-1, 1]) {
      const cone = new THREE.Mesh(coneGeo, beamMat);
      cone.position.set(s * 0.21, 0.15, 1.66);
      cone.rotation.x = 0.045; // pitched slightly downward onto the tarmac
      cone.rotation.y = s * 0.05;
      headlightBeamGroup.add(cone);
    }
    headlightBeamGroup.visible = false;
    group.add(headlightBeamGroup);
  }

  // --- F. Animated Exhaust Backfire Plasma Flame Cones (player car only)
  const flameGroup = new THREE.Group();
  const flameCones = [];
  let flameOuterMat = null;
  let flameInnerMat = null;
  if (!translucent) {
    const flameTex = getFlameTexture();
    flameOuterMat = new THREE.MeshBasicMaterial({
      map: flameTex,
      color: 0xffaa44,
      transparent: true,
      opacity: 0.9,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    flameInnerMat = new THREE.MeshBasicMaterial({
      map: flameTex,
      color: 0xc8f4ff,
      transparent: true,
      opacity: 0.95,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      side: THREE.DoubleSide,
    });

    // Wide bright core (r=0.044, v=1) seated inside exhaust mouth (z=0) -> pointed fiery tail (z=-0.72, v=0)
    const outerGeo = new THREE.CylinderGeometry(0.044, 0.004, 0.72, 12, 1, true);
    outerGeo.translate(0, -0.36, 0);
    outerGeo.rotateX(Math.PI / 2);
    const innerGeo = new THREE.CylinderGeometry(0.026, 0.002, 0.44, 10, 1, true);
    innerGeo.translate(0, -0.22, 0);
    innerGeo.rotateX(Math.PI / 2);

    for (const s of [-1, 1]) {
      const pair = new THREE.Group();
      pair.position.set(s * 0.095, 0.215, -2.13);
      const outM = new THREE.Mesh(outerGeo, flameOuterMat);
      const inM = new THREE.Mesh(innerGeo, flameInnerMat);
      pair.add(outM, inM);
      flameGroup.add(pair);
      flameCones.push(pair);
    }
    flameGroup.visible = false;
    group.add(flameGroup);
  }

  // =========================================================================
  // LIVE COSMETIC HOOKS (ZERO PER-FRAME ALLOCATIONS!)
  // =========================================================================

  /** Orient a unit-length +Z rod strictly in car-local space from A -> B (zero allocs). */
  function orientRod(mesh, ax, ay, az, bx, by, bz) {
    const dx = bx - ax;
    const dy = by - ay;
    const dz = bz - az;
    const len = Math.hypot(dx, dy, dz);
    if (len < 1e-5) return;
    mesh.position.set((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5);
    _rodDir.set(dx / len, dy / len, dz / len);
    mesh.quaternion.setFromUnitVectors(_Z_AXIS, _rodDir);
    mesh.scale.set(1, 1, len);
  }

  /** Update articulated double-wishbone V-arms, pushrods, uprights & calipers (zero allocs). */
  group.userData.updateSuspension = (wheelLocalPos) => {
    if (!wheelLocalPos || suspRigs.length === 0) return;
    for (let i = 0; i < suspRigs.length; i++) {
      const r = suspRigs[i];
      const wp = wheelLocalPos[i];
      if (!wp) continue;
      const wx = wp.x;
      const wy = wp.y;
      const wz = wp.z;
      const jointX = wx - r.side * 0.165;
      const upY = wy + 0.112;
      const loY = wy - 0.105;

      // Upper V-wishbone (front & rear legs)
      orientRod(r.upF, r.uFx, r.uFy, r.uFz, jointX, upY, wz);
      orientRod(r.upR, r.uRx, r.uRy, r.uRz, jointX, upY, wz);
      // Lower V-wishbone (front & rear legs)
      orientRod(r.loF, r.lFx, r.lFy, r.lFz, jointX, loY, wz);
      orientRod(r.loR, r.lRx, r.lRy, r.lRz, jointX, loY, wz);
      // Diagonal pushrod (chassis rocker -> lower upright clevis)
      orientRod(r.push, r.prx, r.pry, r.prz, jointX, loY + 0.02, wz);

      // Track non-spinning upright + aero brake duct + 6-piston caliper
      r.hubNode.position.set(wx, wy, wz);
    }
  };

  /** Drive cockpit steering wheel, live OLED dash, 15 shift LEDs, rear blades, rain strobe & backfire. */
  group.userData.updateTelemetry = ({
    steerAngle = 0,
    rpm = 1200,
    brake = 0,
    throttle = 0,
    speed = 0,
    gear = 1,
    wetness = 0,
    timeSec = 0,
    cameraMode = "chase",
    backfire = 0,
  } = {}) => {
    // CRITICAL BUG FIX: negative sign so +steerAngle left turn rotates wheel CCW left in cockpit!
    steerWheel.rotation.z = -steerAngle * 3.2;

    // Steer front non-spinning brake calipers & aero ducts smoothly with front wheel lock
    for (let i = 0; i < suspRigs.length; i++) {
      if (suspRigs[i].front) suspRigs[i].hubNode.rotation.y = steerAngle;
    }

    // Hide driver helmet & shoulders in cockpit view so camera never clips
    driverGroup.visible = cameraMode !== "cockpit";

    if (translucent) return;

    // 15 progressive RGB shift LEDs across upper steering wheel brow
    const flash = rpm >= 7200 && Math.sin(timeSec * 38) > 0;
    for (let k = 0; k < shiftLeds.length; k++) {
      const l = shiftLeds[k];
      l.mat.emissiveIntensity = flash ? 4.2 : rpm >= l.threshold ? 2.6 : 0.05;
    }

    // Throttled ~20 Hz live 256×128 OLED telemetry canvas update
    if (oled) {
      oled.draw(rpm, speed, gear, brake, throttle, wetness, timeSec);
    }

    // HDR Rear LED light blades & FIA wet-weather rain strobe
    const brakeGlow = 0.85 + brake * 5.2;
    const rainPulse = wetness > 0.35 ? (Math.sin(timeSec * 20) > 0 ? 4.6 : 0.55) : 0.0;
    brakeLightMat.emissiveIntensity = brakeGlow;
    rainStrobeMat.emissiveIntensity = Math.max(brakeGlow, rainPulse);

    // Exhaust thermal inner throat glow + animated plasma backfire cones
    const hotLoad = throttle * Math.max(0, (rpm - 4200) / 3400);
    const bf = backfire <= 0 ? 0 : backfire >= 1 ? 1 : backfire;
    exhaustInnerMat.emissiveIntensity = 0.08 + hotLoad * 1.4 + bf * 4.2;

    if (flameCones.length > 0) {
      const showFlame = bf > 0.02;
      flameGroup.visible = showFlame;
      if (showFlame) {
        const flickA = 0.82 + 0.24 * Math.sin(timeSec * 92) + 0.14 * Math.sin(timeSec * 157);
        const flickB = 0.82 + 0.24 * Math.cos(timeSec * 105) + 0.14 * Math.sin(timeSec * 173);
        const radScale = 0.65 + bf * 0.65;
        flameCones[0].scale.set(radScale, radScale, (0.35 + bf * 0.95) * flickA);
        flameCones[1].scale.set(radScale, radScale, (0.35 + bf * 0.95) * flickB);
        flameCones[0].rotation.z = timeSec * 19;
        flameCones[1].rotation.z = -timeSec * 23;
        flameOuterMat.opacity = Math.min(1, bf * 1.15);
        flameInnerMat.opacity = Math.min(1, bf * 1.35);
      }
    }
  };

  /** Adjust nose laser DRL blades, headlamp projectors & volumetric dusk/night beam cones. */
  group.userData.setHeadlights = (intensity = 0) => {
    const v = Math.max(0, Math.min(1.5, intensity));
    if (translucent) return;
    drlMat.emissiveIntensity = 1.05 + v * 3.2;
    projectorMat.emissiveIntensity = 0.35 + v * 4.8;
    _tmpColor.setRGB(0.86 + 0.14 * Math.min(1, v), 0.95 + 0.05 * Math.min(1, v), 1.0);
    drlMat.emissive.copy(_tmpColor);
    if (beamMat) {
      headlightBeamGroup.visible = v > 0.03;
      beamMat.opacity = Math.min(0.32, v * 0.22);
    }
  };

  // --- G. Static Wheels for Ghost Car (`withWheels = true`)
  if (withWheels) {
    const wheelCoords = [
      [-0.84, -0.15, 1.30],
      [0.84, -0.15, 1.30],
      [-0.84, -0.15, -1.30],
      [0.84, -0.15, -1.30],
    ];
    for (const [wx, wy, wz] of wheelCoords) {
      const w = buildWheel(opacity);
      w.position.set(wx, wy, wz);
      group.add(w);
    }
  }

  return group;
}

// 2026 AAA Sim-Racing Physical Atmosphere & Lighting Core —
//   • Physical Rayleigh + Henyey-Greenstein Mie sky dome with crisp HDR solar
//     disc, limb darkening, optical solar corona, moonlit night sky, twinkling
//     multi-magnitude starfield + Milky Way veil, and dual-layer drifting
//     procedural cirrus + sunlit silver-lining cumulus cloud deck;
//   • High-contrast automotive PMREM IBL cubemap with intense solar specular
//     lobes, overhead cloud-break skylight strips, night/wet stadium floodlight
//     hotspots, and dark ground bounce for vivid clearcoat & puddle reflections;
//   • Light-space texel-snapped 2048² PCFSoftShadowMap (±34 m tight frustum,
//     zero edge shimmer or shadow crawl);
//   • Dual wide + long-throw motorsport LED headlights, 16 cantilevered circuit
//     stadium floodlights with 2 pooled nearest-tower dynamic PointLights, and
//     per-condition HDR exposure/bloom parameters for PostProcessPipeline.
//
// Auto mode follows the player's local clock (purely cosmetic — zero physics
// coupling). Press L (wired in app.js) to cycle:
//   auto → golden hour → midday → overcast → wet → night → auto.
import * as THREE from "../vendor/three.module.js";

const smoothstep = (u) => u * u * (3 - 2 * u);
const DEG = Math.PI / 180;
const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

// Per-condition physical lighting, atmosphere, cloud & HDR post-process targets.
const COND = {
  golden: {
    el: 14, az: 18, sunColor: 0xffb168, sunInt: 3.15,
    hemiSky: 0xffd0a6, hemiGround: 0x3b312b, hemiInt: 0.66,
    skyTop: 0x1d428a, skyHorizon: 0xffa65c, skyBottom: 0x262220,
    glowC: 0xffc478, haze: 1.05, disc: 1.0,
    cloudCover: 0.46, cirrusCover: 0.58, cloudTint: 0xffcfa0, cloudShadow: 0x523c4e,
    stars: 0.0, wetness: 0.0, envInt: 1.08,
    fog: 0xdca06e, fogNear: 185, fogFar: 920,
    exposure: 1.06, bloomStrength: 0.42, bloomThreshold: 1.05,
    head: 0.22, glow: 0.65, pole: 0.14,
  },
  midday: {
    el: 60, az: 82, sunColor: 0xfff7ea, sunInt: 3.45,
    hemiSky: 0xbad9ff, hemiGround: 0x3b444d, hemiInt: 0.94,
    skyTop: 0x1652b8, skyHorizon: 0x9ec9ed, skyBottom: 0x30373f,
    glowC: 0xfff8ec, haze: 0.36, disc: 1.0,
    cloudCover: 0.38, cirrusCover: 0.42, cloudTint: 0xffffff, cloudShadow: 0x728aa6,
    stars: 0.0, wetness: 0.0, envInt: 1.12,
    fog: 0xa0c2e0, fogNear: 280, fogFar: 1250,
    exposure: 1.00, bloomStrength: 0.28, bloomThreshold: 1.15,
    head: 0.0, glow: 0.22, pole: 0.0,
  },
  overcast: {
    el: 50, az: 68, sunColor: 0xd8e1eb, sunInt: 0.68,
    hemiSky: 0xb4bec9, hemiGround: 0x353b42, hemiInt: 1.28,
    skyTop: 0x4c5866, skyHorizon: 0x95a0ab, skyBottom: 0x2d3238,
    glowC: 0xc8d0da, haze: 0.14, disc: 0.0,
    cloudCover: 0.86, cirrusCover: 0.25, cloudTint: 0xc9d0d8, cloudShadow: 0x505a66,
    stars: 0.0, wetness: 0.0, envInt: 0.90,
    fog: 0x909ba6, fogNear: 145, fogFar: 680,
    exposure: 1.02, bloomStrength: 0.26, bloomThreshold: 1.08,
    head: 0.30, glow: 0.50, pole: 0.18,
  },
  wet: {
    el: 44, az: 68, sunColor: 0xc6d5e6, sunInt: 0.88,
    hemiSky: 0x9ab0c7, hemiGround: 0x181e26, hemiInt: 1.24,
    skyTop: 0x303c4d, skyHorizon: 0x6c7c8d, skyBottom: 0x181e25,
    glowC: 0xb8c8da, haze: 0.22, disc: 0.0,
    cloudCover: 0.94, cirrusCover: 0.20, cloudTint: 0x95a5b5, cloudShadow: 0x2d3744,
    stars: 0.0, wetness: 1.0, envInt: 1.06,
    fog: 0x647484, fogNear: 120, fogFar: 560,
    exposure: 1.04, bloomStrength: 0.46, bloomThreshold: 0.96,
    head: 0.78, glow: 1.15, pole: 0.64,
  },
  night: {
    el: 18, az: 245, sunColor: 0x8fb4f2, sunInt: 0.56,
    hemiSky: 0x2c4674, hemiGround: 0x101824, hemiInt: 0.86,
    skyTop: 0x040712, skyHorizon: 0x10203a, skyBottom: 0x05080f,
    glowC: 0x4b70aa, haze: 0.18, disc: 0.0,
    cloudCover: 0.26, cirrusCover: 0.30, cloudTint: 0x2a3f62, cloudShadow: 0x09101e,
    stars: 1.0, wetness: 0.0, envInt: 0.96,
    fog: 0x09101d, fogNear: 120, fogFar: 600,
    exposure: 1.12, bloomStrength: 0.66, bloomThreshold: 0.82,
    head: 1.0, glow: 2.35, pole: 1.0,
  },
};

const AUTO = [
  [0.0, "night", -90],
  [4.5, "night", -20],
  [6.5, "golden", 18],
  [9.5, "midday", 55],
  [15.0, "midday", 115],
  [18.0, "golden", 160],
  [20.5, "night", 205],
  [24.0, "night", 270],
];

const ORDER = [null, "golden", "midday", "overcast", "wet", "night"];
const LABELS = {
  golden: "golden hour",
  midday: "midday",
  overcast: "overcast",
  wet: "wet",
  night: "night",
};

const DOME_R = 1400;
const SUN_DIST = 1320;
const SHADOW_HALF = 34;
const SHADOW_RES = 2048;
const SHADOW_TEXEL = (SHADOW_HALF * 2) / SHADOW_RES;

/** Soft radial gradient canvas texture for volumetric lamp / solar halation. */
function radialTex(stops) {
  const c = document.createElement("canvas");
  c.width = c.height = 128;
  const g = c.getContext("2d");
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  for (const [t, col] of stops) grad.addColorStop(t, col);
  g.fillStyle = grad;
  g.fillRect(0, 0, 128, 128);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

const DOME_VERT = /* glsl */ `
varying vec3 vDir;
void main() {
  vDir = position;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;

// 2026 Physical Rayleigh + Henyey-Greenstein Mie Sky Dome:
//   • Air-mass horizon scattering + anti-solar twilight belt;
//   • Crisp HDR solar disc (> 5.0 radiance) + limb darkening + optical corona
//     (and HDR lunar disc + halo at night);
//   • Two-magnitude twinkling spectral starfield + faint Milky Way band;
//   • Dual-layer drifting cirrus + 5-octave lit cumulus deck with 2-step
//     directional optical depth, Beer-Lambert silver-lining, and aerial horizon fade.
const DOME_FRAG = /* glsl */ `
uniform vec3 topColor;
uniform vec3 horizonColor;
uniform vec3 bottomColor;
uniform vec3 fogColor;
uniform vec3 glowColor;
uniform vec3 cloudTint;
uniform vec3 cloudShadow;
uniform float glowAmount;
uniform float discVis;
uniform float cloudCover;
uniform float cirrusCover;
uniform float starVis;
uniform float uTime;
uniform vec3 sunDir;
varying vec3 vDir;

float hash21(vec2 p) {
  p = fract(p * vec2(234.34, 435.345));
  p += dot(p, p + 34.23);
  return fract(p.x * p.y);
}

float noise2(vec2 p) {
  vec2 i = floor(p);
  vec2 f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  float a = hash21(i);
  float b = hash21(i + vec2(1.0, 0.0));
  float c = hash21(i + vec2(0.0, 1.0));
  float d = hash21(i + vec2(1.0, 1.0));
  return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float fbm4(vec2 p) {
  float v = 0.0;
  float a = 0.5;
  const mat2 rot = mat2(0.80, -0.60, 0.60, 0.80);
  for (int i = 0; i < 4; i++) {
    v += a * noise2(p);
    p = rot * p * 2.03 + vec2(13.7, 7.1);
    a *= 0.5;
  }
  return v;
}

float fbm5(vec2 p) {
  float v = 0.0;
  float a = 0.5;
  const mat2 rot = mat2(0.80, -0.60, 0.60, 0.80);
  for (int i = 0; i < 5; i++) {
    v += a * noise2(p);
    p = rot * p * 2.04 + vec2(11.3, 19.7);
    a *= 0.48;
  }
  return v;
}

// Henyey-Greenstein Mie phase function
float hgPhase(float cosTheta, float g) {
  float g2 = g * g;
  float denom = max(1.0 + g2 - 2.0 * g * cosTheta, 0.002);
  return (1.0 - g2) / (4.0 * 3.14159265 * pow(denom, 1.5));
}

void main() {
  vec3 d = normalize(vDir);
  vec3 sDir = normalize(sunDir);
  float elev = clamp(d.y, 0.0, 1.0);
  float cosTheta = dot(d, sDir);
  float cosPos = max(cosTheta, 0.0);

  // --- 1. Physical Rayleigh vertical ramp + horizon scattering belt
  float rayleighT = pow(max(elev, 1e-5), 0.52);
  vec3 skyCol = d.y >= 0.0
    ? mix(horizonColor, topColor, rayleighT)
    : mix(horizonColor, bottomColor, pow(max(-d.y, 1e-5), 0.42));

  // Subtle horizon aerial scattering uplift (captures golden-hour peach/rose belt)
  float horizonBand = exp(-abs(d.y) * 9.5);
  float sunLow = clamp(1.0 - sDir.y * 1.6, 0.0, 1.0);
  skyCol += horizonColor * (horizonBand * 0.22 * (0.4 + 0.6 * sunLow));

  // --- 2. Mie forward-scattering aura around sun/moon
  float mieBroad = hgPhase(cosTheta, 0.64) * 1.15;
  float mieAureole = hgPhase(cosTheta, 0.935) * 0.16;
  float airMass = 1.0 + 0.75 * pow(max(1.0 - elev, 1e-5), 3.0);
  skyCol += glowColor * ((mieBroad + mieAureole) * glowAmount * airMass);

  // --- 3. Crisp HDR Solar Disc + optical corona (blooms organically > 1.5 HDR)
  float sunAngular = 1.0 - cosTheta;
  if (discVis > 0.01 && d.y > -0.04) {
    // Limb-darkened sun disc (~1.35° apparent radius)
    float discMask = smoothstep(0.00034, 0.00018, sunAngular);
    float limb = pow(max(clamp(1.0 - sunAngular / 0.00032, 0.0, 1.0), 1e-5), 0.38);
    float corona = exp(-sunAngular * 520.0) * 1.9 + exp(-sunAngular * 55.0) * 0.52;
    vec3 sunCoreCol = mix(glowColor, vec3(1.0, 0.985, 0.95), 0.65);
    skyCol += sunCoreCol * ((discMask * (4.4 + 1.8 * limb) + corona) * discVis * smoothstep(-0.04, 0.03, d.y));
  }

  // --- 4. Night starfield + faint Milky Way band + crisp HDR Moon
  if (starVis > 0.01 && d.y > 0.01) {
    float skyFade = smoothstep(0.01, 0.22, d.y);

    // Subtle Milky Way galactic band tilted across the sky
    vec3 mwAxis = normalize(vec3(0.42, 0.72, -0.55));
    float mwLat = abs(dot(d, mwAxis));
    float mwBand = exp(-mwLat * mwLat * 18.0);
    vec2 mwUv = d.xz / (d.y + 0.45) * 2.4;
    float mwDust = fbm4(mwUv + vec2(3.1, -1.7));
    float mwRift = smoothstep(0.32, 0.68, fbm4(mwUv * 2.1 + vec2(-2.4, 5.2)));
    skyCol += vec3(0.11, 0.16, 0.28) * (mwBand * mwDust * (0.45 + 0.55 * mwRift) * starVis * skyFade * 0.55);

    // Primary bright stars
    vec2 sp1 = d.xz / (d.y + 0.26) * 88.0;
    vec2 c1 = floor(sp1);
    vec2 f1 = fract(sp1) - 0.5;
    float h1 = hash21(c1);
    if (h1 > 0.978) {
      vec2 jit = vec2(hash21(c1 + 17.3) - 0.5, hash21(c1 + 41.9) - 0.5) * 0.58;
      float dist = length(f1 - jit);
      float mag = pow(max((h1 - 0.978) / 0.022, 1e-5), 1.4);
      float star = smoothstep(0.17, 0.015, dist) * (0.55 + 1.35 * mag);
      float tw = 0.74 + 0.26 * sin(uTime * (2.0 + h1 * 4.5) + h1 * 91.0);
      vec3 sCol = mix(vec3(0.72, 0.85, 1.0), vec3(1.0, 0.90, 0.74), fract(h1 * 137.3));
      skyCol += sCol * (star * tw * starVis * skyFade * 1.45);
    }

    // Secondary fainter pinpoint stars
    vec2 sp2 = d.xz / (d.y + 0.30) * 168.0 + vec2(31.4, -19.2);
    vec2 c2 = floor(sp2);
    vec2 f2 = fract(sp2) - 0.5;
    float h2 = hash21(c2);
    if (h2 > 0.972) {
      vec2 jit = vec2(hash21(c2 + 9.7) - 0.5, hash21(c2 + 53.1) - 0.5) * 0.5;
      float star2 = smoothstep(0.18, 0.02, length(f2 - jit));
      float tw2 = 0.80 + 0.20 * sin(uTime * 1.8 + h2 * 73.0);
      skyCol += vec3(0.78, 0.86, 1.0) * (star2 * tw2 * starVis * skyFade * 0.52);
    }

    // Crisp HDR Moon disc & lunar aureole at night
    float moonMask = smoothstep(0.00024, 0.00014, sunAngular);
    float moonHalo = exp(-sunAngular * 220.0) * 0.65 + exp(-sunAngular * 24.0) * 0.14;
    skyCol += vec3(0.80, 0.89, 1.0) * ((moonMask * 2.6 + moonHalo) * starVis * skyFade);
  }

  // --- 5. High-Altitude Wispy Cirrus Layer
  if (d.y > 0.015 && cirrusCover > 0.01) {
    vec2 cUv = d.xz / (d.y + 0.095) * 0.56;
    vec2 cDrift = vec2(uTime * 0.0042, uTime * 0.0014);
    float warp = fbm4(cUv * 1.2 + cDrift);
    vec2 wUv = vec2(cUv.x * 1.65 + warp * 0.55, cUv.y * 0.52 - warp * 0.35);
    float cN = fbm4(wUv + cDrift * 1.3);
    float cMask = smoothstep(0.56, 0.82, cN) * cirrusCover * smoothstep(0.015, 0.20, d.y);
    vec3 cirCol = mix(cloudTint, glowColor, 0.28 + 0.42 * pow(cosPos, 3.0));
    cirCol *= 1.0 + 0.45 * pow(cosPos, 6.0) * glowAmount;
    skyCol = mix(skyCol, cirCol, cMask * 0.42);
  }

  // --- 6. Mid-Altitude Volumetric-Look Lit Cumulus / Stratocumulus Deck
  if (d.y > 0.012 && cloudCover > 0.01) {
    vec2 uv = d.xz / (d.y + 0.138) * 0.78;
    vec2 drift = vec2(uTime * 0.0092, uTime * 0.0034);
    float base = fbm5(uv + drift);
    float detail = fbm4(uv * 2.35 - drift * 1.45 + vec2(4.7, -3.3));
    float billow = 1.0 - abs(detail * 2.0 - 1.0);
    float density = base * 0.70 + detail * 0.20 + billow * 0.10;

    float thresh = mix(0.65, 0.26, cloudCover);
    float mask = smoothstep(thresh, thresh + 0.23, density);

    // 2-step directional light-march toward sun for self-shadowing + silver lining
    float sunHLen = length(sDir.xz);
    vec2 sunHoriz = sunHLen > 1e-4 ? (sDir.xz / sunHLen) : vec2(1.0, 0.0);
    vec2 sunStep = sunHoriz * (0.065 / max(sDir.y + 0.35, 0.35));
    float s1 = fbm4(uv + drift + sunStep);
    float s2 = fbm4(uv + drift + sunStep * 2.4);
    float opticalDepth = max(0.0, (s1 * 0.62 + s2 * 0.38) - thresh + 0.08);
    float directTrans = exp(-opticalDepth * 3.1);
    float powder = 1.0 - exp(-max(density - thresh, 0.0) * 5.2);
    float lit = clamp(directTrans * (0.58 + 0.42 * powder) + 0.22 + pow(cosPos, 3.0) * 0.32, 0.0, 1.0);

    vec3 cCol = mix(cloudShadow, cloudTint, lit);
    // Silver-lining Henyey-Greenstein rim glow along sunlit cloud edges
    float edgeThin = (1.0 - mask * 0.62) * smoothstep(0.05, 0.45, mask);
    float silver = hgPhase(cosTheta, 0.76) * (0.35 + 0.65 * directTrans) * edgeThin;
    cCol += glowColor * (silver * (0.45 + 0.95 * glowAmount));

    // Aerial perspective fade near the horizon
    float horFade = smoothstep(0.012, 0.22, d.y);
    float opacity = mask * horFade * mix(0.74, 0.96, cloudCover);
    skyCol = mix(skyCol, cCol, opacity);
  }

  // --- 7. Seamless aerial-perspective horizon fog lock
  float fogLock = smoothstep(0.11, -0.02, d.y);
  skyCol = mix(skyCol, fogColor, fogLock * 0.85);

  gl_FragColor = vec4(max(skyCol, vec3(0.0)), 1.0);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

// High-contrast studio/circuit HDRI environment sphere for PMREMGenerator IBL:
// vivid sky/horizon contrast, bright cloud-break sky strips, intense solar
// specular lobe, night/wet overhead stadium floodlight hotspots, and dark
// grounded tarmac reflection below horizon so clearcoat paint & wet puddles gleam.
const ENV_FRAG = /* glsl */ `
uniform vec3 topColor;
uniform vec3 horizonColor;
uniform vec3 bottomColor;
uniform vec3 glowColor;
uniform float glowAmount;
uniform float sunBoost;
uniform float poleVis;
uniform float wetness;
uniform vec3 sunDir;
varying vec3 vDir;

void main() {
  vec3 d = normalize(vDir);
  vec3 sDir = normalize(sunDir);

  // High-contrast sky-vs-dark-ground reflection split (key to metallic clearcoat)
  vec3 col;
  if (d.y >= 0.0) {
    col = mix(horizonColor * 1.18, topColor * 0.92, pow(max(d.y, 1e-5), 0.52));
    // Crisp bright horizon reflection ribbon (classic automotive highlight line)
    float horStrip = exp(-d.y * 15.0);
    col += horizonColor * (horStrip * (0.55 + 0.35 * wetness));
    // Stylized overhead cloud-break softbox strips for sleek hood/sidepod reflections
    float azSky = atan(d.z, d.x + 1e-6);
    float band1 = exp(-pow((d.y - 0.42) * 7.5, 2.0)) * (0.5 + 0.5 * sin(azSky * 2.0 + 0.6));
    float band2 = exp(-pow((d.y - 0.72) * 8.0, 2.0)) * (0.5 + 0.5 * cos(azSky * 3.0 - 0.8));
    col += mix(horizonColor, vec3(1.0), 0.55) * ((band1 * 0.38 + band2 * 0.25) * (0.55 + 0.45 * wetness));
  } else {
    // Dark ground bounce with sharp drop across horizon line
    col = mix(horizonColor * 0.32, bottomColor * 0.28, pow(max(-d.y, 1e-5), 0.35));
  }

  // Intense HDR directional solar specular lobe (razor glints on paint & puddles)
  float cs = max(dot(d, sDir), 0.0);
  col += glowColor * (pow(cs, 8.0) * glowAmount * 1.15 + pow(cs, 128.0) * sunBoost);

  // Night & wet circuit stadium floodlight hotspots + trackside cyan/amber horizon glints
  if (poleVis > 0.01) {
    float az = atan(d.z, d.x + 1e-6);
    float ringElev = exp(-pow((d.y - 0.36) * 11.5, 2.0));
    float lamps = pow(max(0.5 + 0.5 * cos(az * 6.0), 0.0), 18.0);
    vec3 lampCol = mix(vec3(1.0, 0.86, 0.68), vec3(0.84, 0.94, 1.0), 0.5 + 0.5 * sin(az * 3.0));
    col += lampCol * (ringElev * lamps * poleVis * 14.5);

    // Subtle trackside FIA cyan/warm-amber low-horizon reflection accents
    float lowBand = exp(-pow((d.y - 0.05) * 22.0, 2.0));
    float accent = pow(max(0.5 + 0.5 * sin(az * 4.0 + 1.1), 0.0), 10.0);
    col += vec3(0.14, 0.88, 0.82) * (lowBand * accent * poleVis * 1.6);
  }

  gl_FragColor = vec4(max(col, vec3(0.0)), 1.0);
}`;

export class Lighting {
  /**
   * @param {THREE.Scene} scene
   * @param {THREE.Object3D} car player car group — headlights are parented to it.
   * @param {THREE.Material[]} glowMaterials materials whose emissiveIntensity tracks condition.
   * @param {THREE.WebGLRenderer|null} renderer optional — drives exposure, shadowMap & PMREM IBL.
   * @param {object|null} track optional parsed TRK1 — places 16 instanced stadium floodlights.
   * @param {object|null} trackMats optional { asphaltMat, grassMat, kerbMat, updateSurface }.
   */
  constructor(scene, car, glowMaterials = [], renderer = null, track = null, trackMats = {}) {
    this.scene = scene;
    this.car = car;
    this.glow = glowMaterials || [];
    this.renderer = renderer;
    this.trackMats = trackMats || {};
    this.surfaceHooks = this.trackMats; // backwards-compat alias

    this.override = null; // null = auto (local clock)
    this.wetness = 0;
    this.conditionKey = "golden";
    this.currentExposure = COND.golden.exposure;
    this.bloomStrength = COND.golden.bloomStrength;
    this.bloomThreshold = COND.golden.bloomThreshold;

    this._t0 = performance.now();
    this._envCache = new Map();
    this._activeEnvId = -1;
    this._poleLampPos = [];
    this._poleLights = [];
    this._A = COND.golden;
    this._B = COND.golden;
    this._u = 0;

    if (renderer && renderer.shadowMap) {
      renderer.shadowMap.enabled = true;
      renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    }

    // ---- Key light + light-space texel-snapped 2048² shadow rig + hemisphere fill
    this.hemi = new THREE.HemisphereLight(0xffd0a6, 0x3b312b, 0.66);
    this.sun = new THREE.DirectionalLight(0xffb168, 3.15);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(SHADOW_RES, SHADOW_RES);
    const sc = this.sun.shadow.camera;
    sc.left = -SHADOW_HALF;
    sc.right = SHADOW_HALF;
    sc.top = SHADOW_HALF;
    sc.bottom = -SHADOW_HALF;
    sc.near = 10;
    sc.far = 360;
    sc.updateProjectionMatrix();
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.025;
    this.sun.shadow.radius = 2.0;
    scene.add(this.hemi, this.sun, this.sun.target);

    // ---- Upgraded dual wide + focused long-throw motorsport LED headlights
    this.heads = [];
    // Wide low-beam flood pair (illuminates kerbs, apexes & wide shoulders)
    for (const x of [-0.38, 0.38]) {
      const s = new THREE.SpotLight(0xfff5e4, 0, 110, 0.78, 0.65, 1.1);
      s.position.set(x, 0.36, 1.72);
      s.target.position.set(x * 6.8, -1.05, 32);
      car.add(s);
      car.add(s.target);
      this.heads.push(s);
    }
    // Focused long-throw high-beam center spot (cuts straight down 180 m straights)
    const highBeam = new THREE.SpotLight(0xeef7ff, 0, 185, 0.36, 0.42, 1.0);
    highBeam.position.set(0, 0.42, 1.88);
    highBeam.target.position.set(0, -0.75, 58);
    car.add(highBeam);
    car.add(highBeam.target);
    this.heads.push(highBeam);

    // Scratch math objects (zero per-frame allocations)
    this._c1 = new THREE.Color();
    this._c2 = new THREE.Color();
    this._cFog = new THREE.Color();
    this._dir = new THREE.Vector3(0, 1, 0);
    this._wUp = new THREE.Vector3(0, 1, 0);
    this._lRight = new THREE.Vector3();
    this._lUp = new THREE.Vector3();
    this._snapPos = new THREE.Vector3();

    this._buildSky();
    this._initPMREM();
    if (track) this._buildPoles(track);
    this._applyShadowFlags();

    if (!(scene.background instanceof THREE.Color)) scene.background = new THREE.Color();
    this.update();
  }

  _initPMREM() {
    if (!this.renderer || typeof THREE.PMREMGenerator !== "function") return;
    try {
      this._pmrem = new THREE.PMREMGenerator(this.renderer);
      this._pmrem.compileEquirectangularShader?.();
      this._envUniforms = {
        topColor: { value: new THREE.Color(0x1d428a) },
        horizonColor: { value: new THREE.Color(0xffa65c) },
        bottomColor: { value: new THREE.Color(0x262220) },
        glowColor: { value: new THREE.Color(0xffc478) },
        glowAmount: { value: 1.05 },
        sunBoost: { value: 24.0 },
        poleVis: { value: 0.0 },
        wetness: { value: 0.0 },
        sunDir: { value: new THREE.Vector3(0, 0.25, 0.97) },
      };
      this._envScene = new THREE.Scene();
      const envSphere = new THREE.Mesh(
        new THREE.SphereGeometry(20, 32, 18),
        new THREE.ShaderMaterial({
          uniforms: this._envUniforms,
          vertexShader: DOME_VERT,
          fragmentShader: ENV_FRAG,
          side: THREE.BackSide,
          depthWrite: false,
        }),
      );
      this._envScene.add(envSphere);
    } catch {
      this._pmrem = null;
    }
  }

  /** Bake (or fetch cached) high-contrast PMREM IBL cubemap for PBR paint, metal & puddles. */
  _syncEnvMap(envId, isAuto, top, hor, bot, glow, haze, sunBoost, poleVis, wetness, dir) {
    if (!this._pmrem) return;
    if (envId === this._activeEnvId) return;
    this._activeEnvId = envId;
    if (!isAuto && this._envCache.has(envId)) {
      this.scene.environment = this._envCache.get(envId).texture;
      return;
    }
    const U = this._envUniforms;
    U.topColor.value.copy(top);
    U.horizonColor.value.copy(hor);
    U.bottomColor.value.copy(bot);
    U.glowColor.value.copy(glow);
    U.glowAmount.value = haze;
    U.sunBoost.value = sunBoost;
    U.poleVis.value = poleVis;
    U.wetness.value = wetness;
    U.sunDir.value.copy(dir);
    // Low sigma (0.022) preserves crisp specular sky/sun/floodlight highlights on clearcoat & puddles
    const rt = this._pmrem.fromScene(this._envScene, 0.022);
    if (isAuto) {
      this._autoEnvRT?.dispose();
      this._autoEnvRT = rt;
    } else {
      this._envCache.set(envId, rt);
    }
    this.scene.environment = rt.texture;
  }

  _buildSky() {
    this.domeUniforms = {
      topColor: { value: new THREE.Color(0x1d428a) },
      horizonColor: { value: new THREE.Color(0xffa65c) },
      bottomColor: { value: new THREE.Color(0x262220) },
      fogColor: { value: new THREE.Color(0xdca06e) },
      glowColor: { value: new THREE.Color(0xffc478) },
      cloudTint: { value: new THREE.Color(0xffcfa0) },
      cloudShadow: { value: new THREE.Color(0x523c4e) },
      glowAmount: { value: 1.05 },
      discVis: { value: 1.0 },
      cloudCover: { value: 0.46 },
      cirrusCover: { value: 0.58 },
      starVis: { value: 0.0 },
      uTime: { value: 0.0 },
      sunDir: { value: new THREE.Vector3(0, 0.25, 0.97) },
    };
    const mat = new THREE.ShaderMaterial({
      uniforms: this.domeUniforms,
      vertexShader: DOME_VERT,
      fragmentShader: DOME_FRAG,
      side: THREE.BackSide,
      depthWrite: false,
      depthTest: false,
    });
    this.dome = new THREE.Mesh(new THREE.SphereGeometry(DOME_R, 48, 24), mat);
    this.dome.renderOrder = -100;
    this.dome.frustumCulled = false;
    this.scene.add(this.dome);

    // Subtle soft atmospheric lens veil sprite (primary crisp HDR solar disc
    // lives inside DOME_FRAG so clouds naturally occlude it with silver lining).
    this._haloTex = radialTex([
      [0, "rgba(255,255,255,0.52)"],
      [0.24, "rgba(255,250,238,0.16)"],
      [0.62, "rgba(255,244,224,0.03)"],
      [1, "rgba(255,240,210,0)"],
    ]);
    const discTex = radialTex([
      [0, "rgba(255,255,255,0.55)"],
      [0.42, "rgba(255,250,238,0.22)"],
      [1, "rgba(255,245,220,0)"],
    ]);
    const sprite = (map, scale) => {
      const s = new THREE.Sprite(new THREE.SpriteMaterial({
        map, transparent: true, opacity: 1, blending: THREE.AdditiveBlending,
        depthWrite: false, fog: false, toneMapped: false,
      }));
      s.scale.set(scale, scale, 1);
      this.scene.add(s);
      return s;
    };
    this.sunDisc = sprite(discTex, 42);
    this.sunHalo = sprite(this._haloTex, 180);
  }

  // ---- 16 instanced stadium floodlights around the circuit (alternating warm/cool
  // LED luminaires) + 2 pooled nearest-tower dynamic PointLights so sweeping pools
  // of real specular light glide across the car & wet asphalt at night/dusk.
  _buildPoles(track) {
    const n = track.S;
    const count = 16;

    // Tapered galvanized steel mast + angled cantilevered arm
    const mastGeo = new THREE.CylinderGeometry(0.085, 0.165, 9.2, 8);
    mastGeo.translate(0, 4.6, 0);
    const armGeo = new THREE.BoxGeometry(2.1, 0.14, 0.16);
    armGeo.translate(-0.92, 9.15, 0);
    const housingGeo = new THREE.BoxGeometry(1.75, 0.22, 0.52);
    housingGeo.translate(-1.35, 9.02, 0);

    const poleMat = new THREE.MeshStandardMaterial({
      color: 0x262d38, roughness: 0.48, metalness: 0.72,
    });
    // Warm stadium LED emitters (even towers)
    this.poleHeadMat = new THREE.MeshStandardMaterial({
      color: 0x1b202a, roughness: 0.35, metalness: 0.25,
      emissive: 0xffe2b4, emissiveIntensity: 0,
    });
    // Crisp cool stadium LED emitters (odd towers) for realistic Marina Bay contrast
    this.poleHeadCoolMat = new THREE.MeshStandardMaterial({
      color: 0x1b202a, roughness: 0.35, metalness: 0.25,
      emissive: 0xdff0ff, emissiveIntensity: 0,
    });
    this.poleGlowMat = new THREE.SpriteMaterial({
      map: this._haloTex, color: 0xffdea8, transparent: true, opacity: 0,
      blending: THREE.AdditiveBlending, depthWrite: false, fog: false, toneMapped: false,
    });
    this.poleGlowCoolMat = new THREE.SpriteMaterial({
      map: this._haloTex, color: 0xcce8ff, transparent: true, opacity: 0,
      blending: THREE.AdditiveBlending, depthWrite: false, fog: false, toneMapped: false,
    });

    const idxs = [];
    for (let k = 0; k < count; k++) idxs.push(Math.floor((n * (k + 0.5)) / count));

    const masts = new THREE.InstancedMesh(mastGeo, poleMat, count);
    const arms = new THREE.InstancedMesh(armGeo, poleMat, count);
    const warmCount = Math.ceil(count / 2);
    const coolCount = Math.floor(count / 2);
    const warmHeads = new THREE.InstancedMesh(housingGeo, this.poleHeadMat, warmCount);
    const coolHeads = new THREE.InstancedMesh(housingGeo, this.poleHeadCoolMat, coolCount);
    masts.castShadow = masts.receiveShadow = true;
    arms.castShadow = arms.receiveShadow = true;

    const m4 = new THREE.Matrix4();
    const side = new THREE.Vector3();
    let wi = 0, ci = 0;

    idxs.forEach((i, k) => {
      const c = v3(track.center[i]);
      const u = v3(track.up[i]).normalize();
      const t = v3(track.tangent[i]).normalize();
      side.crossVectors(u, t).normalize();
      const pos = c.clone().addScaledVector(side, track.width[i] * 0.5 + 5.2).addScaledVector(u, -1.2);
      m4.makeBasis(side, u, t).setPosition(pos);
      masts.setMatrixAt(k, m4);
      arms.setMatrixAt(k, m4);

      const isWarm = (k % 2) === 0;
      if (isWarm) warmHeads.setMatrixAt(wi++, m4);
      else coolHeads.setMatrixAt(ci++, m4);

      const lampPos = pos.clone().addScaledVector(u, 8.85).addScaledVector(side, -1.35);
      this._poleLampPos.push({ pos: lampPos, warm: isWarm });

      const spr = new THREE.Sprite(isWarm ? this.poleGlowMat : this.poleGlowCoolMat);
      spr.position.copy(lampPos);
      spr.scale.set(8.2, 8.2, 1);
      this.scene.add(spr);
    });

    this.scene.add(masts, arms, warmHeads, coolHeads);

    // 2 pooled dynamic PointLights that track the 2 nearest stadium towers to the car
    for (let k = 0; k < 2; k++) {
      const pl = new THREE.PointLight(0xffe6be, 0, 52, 1.35);
      pl.castShadow = false;
      this.scene.add(pl);
      this._poleLights.push(pl);
    }
  }

  _applyShadowFlags() {
    const lit = (m) => m && (m.isMeshLambertMaterial || m.isMeshStandardMaterial ||
      m.isMeshPhongMaterial || m.isMeshPhysicalMaterial);
    this.scene.traverse((o) => {
      if (!o.isMesh && !o.isInstancedMesh) return;
      const m = Array.isArray(o.material) ? o.material[0] : o.material;
      if (!lit(m) || m.transparent) return;
      o.receiveShadow = true;
      const g = o.geometry;
      if (g) {
        if (!g.boundingSphere) g.computeBoundingSphere();
        if (g.boundingSphere && g.boundingSphere.radius < 55) o.castShadow = true;
      }
    });
  }

  /** L key: auto → golden hour → midday → overcast → wet → night → auto. */
  cycle() {
    this.override = ORDER[(ORDER.indexOf(this.override) + 1) % ORDER.length];
    this.update();
    return this.override ? LABELS[this.override] : "auto (local clock)";
  }

  hour() {
    const d = new Date();
    return d.getHours() + d.getMinutes() / 60 + d.getSeconds() / 3600;
  }

  _f(k) {
    return this._A[k] + (this._B[k] - this._A[k]) * this._u;
  }

  _mix(k) {
    return this._c1.set(this._A[k]).lerp(this._c2.set(this._B[k]), this._u);
  }

  /**
   * Per-frame zero-allocation lighting, atmosphere, PMREM IBL, texel-snapped
   * shadow frustum, headlight, stadium floodlight & surface wetness update.
   */
  update(timeSec, cam = null) {
    const t = (typeof timeSec === "number" && Number.isFinite(timeSec))
      ? timeSec
      : (performance.now() - this._t0) * 0.001;

    let A, B, u, az, envId, isAuto, domKey;
    if (this.override && COND[this.override]) {
      A = B = COND[this.override];
      u = 0;
      az = A.az;
      envId = ORDER.indexOf(this.override);
      isAuto = false;
      domKey = this.override;
    } else {
      const h = this.hour();
      A = B = COND.night;
      u = 0;
      az = AUTO[0][2];
      domKey = "night";
      for (let i = 0; i + 1 < AUTO.length; i++) {
        if (h >= AUTO[i][0] && h < AUTO[i + 1][0]) {
          u = smoothstep((h - AUTO[i][0]) / (AUTO[i + 1][0] - AUTO[i][0]));
          A = COND[AUTO[i][1]];
          B = COND[AUTO[i + 1][1]];
          az = AUTO[i][2] + (AUTO[i + 1][2] - AUTO[i][2]) * u;
          domKey = u < 0.5 ? AUTO[i][1] : AUTO[i + 1][1];
          break;
        }
      }
      // Refresh auto PMREM at most once per ~20 seconds (numeric key = zero string GC)
      envId = 1000 + Math.floor(h * 180);
      isAuto = true;
    }

    this._A = A;
    this._B = B;
    this._u = u;

    this.conditionKey = domKey;
    this.wetness = this._f("wetness");
    this.currentExposure = this._f("exposure");
    this.bloomStrength = this._f("bloomStrength");
    this.bloomThreshold = this._f("bloomThreshold");

    // Atmosphere: fog + background lock
    this._cFog.copy(this._mix("fog"));
    this.scene.background.copy(this._cFog);
    if (this.scene.fog) {
      this.scene.fog.color.copy(this._cFog);
      this.scene.fog.near = this._f("fogNear");
      this.scene.fog.far = this._f("fogFar");
    }

    this.hemi.color.copy(this._mix("hemiSky"));
    this.hemi.groundColor.copy(this._mix("hemiGround"));
    this.hemi.intensity = this._f("hemiInt");
    this.sun.color.copy(this._mix("sunColor"));
    this.sun.intensity = this._f("sunInt");
    if (this.renderer) this.renderer.toneMappingExposure = this.currentExposure;
    if ("environmentIntensity" in this.scene) this.scene.environmentIntensity = this._f("envInt");

    // Sun / moon direction vector
    const el = this._f("el") * DEG;
    const azr = az * DEG;
    this._dir.set(
      Math.cos(azr) * Math.cos(el),
      Math.sin(el),
      Math.sin(azr) * Math.cos(el),
    ).normalize();

    // ---- Light-space texel-snapped shadow camera (eliminates edge crawling/shimmer)
    const p = this.car.position;
    if (Math.abs(this._dir.y) > 0.99) this._wUp.set(0, 0, 1);
    else this._wUp.set(0, 1, 0);
    this._lRight.crossVectors(this._wUp, this._dir).normalize();
    this._lUp.crossVectors(this._dir, this._lRight).normalize();

    const projX = p.dot(this._lRight);
    const projY = p.dot(this._lUp);
    const projZ = p.dot(this._dir);
    const snapX = Math.round(projX / SHADOW_TEXEL) * SHADOW_TEXEL;
    const snapY = Math.round(projY / SHADOW_TEXEL) * SHADOW_TEXEL;

    this._snapPos
      .copy(this._lRight).multiplyScalar(snapX)
      .addScaledVector(this._lUp, snapY)
      .addScaledVector(this._dir, projZ);

    this.sun.target.position.copy(this._snapPos);
    this.sun.target.updateMatrixWorld();
    this.sun.position.copy(this._snapPos).addScaledVector(this._dir, 165);

    // ---- Sky dome + layered cirrus & lit cumulus + starfield + HDR sun/moon
    // Anchor sky dome to active camera (falls back to car) so far TV tripods never clip it
    const skyOrigin = cam ? cam.position : p;
    this.dome.position.copy(skyOrigin);
    const U = this.domeUniforms;
    U.topColor.value.copy(this._mix("skyTop"));
    U.horizonColor.value.copy(this._mix("skyHorizon"));
    U.bottomColor.value.copy(this._mix("skyBottom"));
    U.fogColor.value.copy(this._cFog);
    U.glowColor.value.copy(this._mix("glowC"));
    U.cloudTint.value.copy(this._mix("cloudTint"));
    U.cloudShadow.value.copy(this._mix("cloudShadow"));
    U.glowAmount.value = this._f("haze");
    U.discVis.value = this._f("disc");
    U.cloudCover.value = this._f("cloudCover");
    U.cirrusCover.value = this._f("cirrusCover");
    U.starVis.value = this._f("stars");
    U.uTime.value = t;
    U.sunDir.value.copy(this._dir);

    // ---- High-contrast PMREM studio/sky IBL (solar glints + floodlight lobes)
    const pf = this._f("pole");
    const disc = this._f("disc");
    const sunBoost = disc > 0.05 ? 26.0 : this.wetness > 0.5 ? 5.5 : 1.8;
    this._syncEnvMap(
      envId,
      isAuto,
      U.topColor.value,
      U.horizonColor.value,
      U.bottomColor.value,
      U.glowColor.value,
      U.glowAmount.value,
      sunBoost,
      pf,
      this.wetness,
      this._dir,
    );

    // ---- Track surface dynamic wetness (prefer track.js custom updateSurface hook)
    if (typeof this.trackMats?.updateSurface === "function") {
      this.trackMats.updateSurface(this.wetness, t);
    } else if (this.trackMats?.asphaltMat) {
      const am = this.trackMats.asphaltMat;
      const w = this.wetness;
      am.roughness = 0.86 - 0.64 * w;
      am.metalness = 0.04 + 0.16 * w;
      am.envMapIntensity = 0.62 + 1.65 * w;
      am.color.setRGB(1.0 - 0.30 * w, 1.0 - 0.27 * w, 1.0 - 0.22 * w);
      if (this.trackMats.kerbMat) {
        this.trackMats.kerbMat.roughness = 0.65 - 0.42 * w;
        this.trackMats.kerbMat.envMapIntensity = 0.55 + 1.15 * w;
      }
      if (this.trackMats.grassMat) {
        this.trackMats.grassMat.roughness = 0.94 - 0.26 * w;
      }
    }

    // ---- Subtle additive atmospheric sun halo veil (primary HDR disc is in DOME_FRAG)
    this.sunDisc.visible = this.sunHalo.visible = disc > 0.01;
    if (this.sunDisc.visible) {
      this.sunDisc.position.copy(skyOrigin).addScaledVector(this._dir, SUN_DIST);
      this.sunHalo.position.copy(this.sunDisc.position);
      this.sunDisc.material.opacity = disc * 0.42;
      this.sunHalo.material.opacity = disc * 0.48;
      this.sunDisc.material.color.copy(this._mix("glowC")).lerp(this._c2.set(0xffffff), 0.55);
      this.sunHalo.material.color.copy(this.sunDisc.material.color);
    }

    // ---- Car headlights (dual wide + focused high-beam) & car.js headlight hook
    const headLevel = this._f("head");
    this.car?.userData?.setHeadlights?.(headLevel);
    if (this.heads.length >= 3) {
      this.heads[0].intensity = headLevel * 58; // wide left low-beam
      this.heads[1].intensity = headLevel * 58; // wide right low-beam
      this.heads[2].intensity = headLevel * 86; // focused center high-beam
    } else {
      for (let i = 0; i < this.heads.length; i++) this.heads[i].intensity = headLevel * 58;
    }

    // ---- Emissive gantry / trackside glow materials
    const glowVal = this._f("glow");
    for (let i = 0; i < this.glow.length; i++) this.glow[i].emissiveIntensity = glowVal;

    // ---- 16 stadium floodlights + 2 pooled dynamic PointLights tracking nearest towers
    if (this.poleHeadMat) {
      this.poleHeadMat.emissiveIntensity = 5.4 * pf;
      this.poleHeadCoolMat.emissiveIntensity = 5.2 * pf;
      this.poleGlowMat.opacity = 0.64 * pf;
      this.poleGlowCoolMat.opacity = 0.58 * pf;

      if (this._poleLights.length >= 2 && this._poleLampPos.length >= 2) {
        if (pf > 0.02) {
          let i0 = 0, i1 = 1;
          let d0 = Infinity, d1 = Infinity;
          for (let k = 0; k < this._poleLampPos.length; k++) {
            const d2 = this._poleLampPos[k].pos.distanceToSquared(p);
            if (d2 < d0) {
              d1 = d0; i1 = i0;
              d0 = d2; i0 = k;
            } else if (d2 < d1) {
              d1 = d2; i1 = k;
            }
          }
          const info0 = this._poleLampPos[i0];
          const pl0 = this._poleLights[0];
          pl0.position.copy(info0.pos);
          pl0.color.setHex(info0.warm ? 0xffe4bc : 0xdcf0ff);
          const a0 = Math.max(0, 1 - Math.max(0, Math.sqrt(d0) - 14) / 38);
          pl0.intensity = 42 * pf * a0 * a0;

          const info1 = this._poleLampPos[i1];
          const pl1 = this._poleLights[1];
          pl1.position.copy(info1.pos);
          pl1.color.setHex(info1.warm ? 0xffe4bc : 0xdcf0ff);
          const a1 = Math.max(0, 1 - Math.max(0, Math.sqrt(d1) - 14) / 38);
          pl1.intensity = 42 * pf * a1 * a1;
        } else {
          this._poleLights[0].intensity = 0;
          this._poleLights[1].intensity = 0;
        }
      }
    }
  }
}

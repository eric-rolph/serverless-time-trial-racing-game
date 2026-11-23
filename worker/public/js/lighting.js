// APEX P0 "Foundation" SOTA lighting core (docs/APEX.md §1) — physical sky dome
// with Rayleigh/Mie scattering, drifting volumetric-style clouds & night stars,
// procedural PMREM IBL environment maps (real sky/sun reflections on PBR paint,
// carbon, Armco & wet tarmac), shadow-casting key light, ACES Filmic exposure,
// and five reference conditions:
//
//   condition    sun                       sky/fog              extras
//   golden hour  low warm (~15°, +X-ish)   warm haze + clouds   long shadows, golden rim IBL
//   midday       high neutral (~62°)       crisp blue + clouds  tight shadows, bright IBL
//   overcast     faint diffuse             thick grey clouds    soft wrap IBL
//   wet          faint + boosted hemi      brooding overcast    glossy wet-asphalt reflection IBL
//   night        off                       starfield + moon     headlights, gantry glow, 12 floodlights
//
// Auto mode follows the PLAYER'S LOCAL CLOCK (purely cosmetic — physics and
// replays never see it). Press L (wired in app.js) to cycle
// auto → golden hour → midday → overcast → wet → night → auto.
import * as THREE from "../vendor/three.module.js";

const smoothstep = (u) => u * u * (3 - 2 * u);
const DEG = Math.PI / 180;
const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

// Per-condition parameters. Colors as hex, scalars lerped.
const COND = {
  golden: {
    el: 15, az: 15, sunColor: 0xffb066, sunInt: 2.75,
    hemiSky: 0xffd2ad, hemiGround: 0x4a4038, hemiInt: 0.62,
    skyTop: 0x29549c, skyHorizon: 0xffb36c, skyBottom: 0x33302e,
    glowC: 0xffc680, haze: 0.95, disc: 1.0,
    cloudCover: 0.44, cloudTint: 0xffd2a4, cloudShadow: 0x5c4654, stars: 0.0,
    wetness: 0.0, envInt: 0.85,
    fog: 0xdda476, fogNear: 105, fogFar: 460,
    exposure: 1.08, head: 0.15, glow: 0.55, pole: 0.08,
  },
  midday: {
    el: 62, az: 85, sunColor: 0xfff5e4, sunInt: 3.15,
    hemiSky: 0xc2dcff, hemiGround: 0x424a52, hemiInt: 0.92,
    skyTop: 0x1e59be, skyHorizon: 0xa4cceb, skyBottom: 0x3a4149,
    glowC: 0xffffff, haze: 0.35, disc: 1.0,
    cloudCover: 0.36, cloudTint: 0xffffff, cloudShadow: 0x7c93ad, stars: 0.0,
    wetness: 0.0, envInt: 0.92,
    fog: 0xa5c3df, fogNear: 170, fogFar: 560,
    exposure: 1.0, head: 0.0, glow: 0.18, pole: 0.0,
  },
  overcast: {
    el: 52, az: 70, sunColor: 0xdde4ec, sunInt: 0.48,
    hemiSky: 0xb6bec8, hemiGround: 0x3d4248, hemiInt: 1.18,
    skyTop: 0x55606e, skyHorizon: 0x99a3ad, skyBottom: 0x35393f,
    glowC: 0xc9d0d8, haze: 0.12, disc: 0.0,
    cloudCover: 0.84, cloudTint: 0xc5ccd4, cloudShadow: 0x56606b, stars: 0.0,
    wetness: 0.0, envInt: 0.78,
    fog: 0x949ea8, fogNear: 80, fogFar: 350,
    exposure: 0.96, head: 0.25, glow: 0.45, pole: 0.12,
  },
  wet: {
    el: 48, az: 70, sunColor: 0xcbd8e6, sunInt: 0.55,
    hemiSky: 0xa8bfd4, hemiGround: 0x1e242c, hemiInt: 1.35,
    skyTop: 0x3d4856, skyHorizon: 0x7c8996, skyBottom: 0x1f242b,
    glowC: 0xb9c7d6, haze: 0.14, disc: 0.0,
    cloudCover: 0.92, cloudTint: 0x9aa8b6, cloudShadow: 0x3a444f, stars: 0.0,
    wetness: 1.0, envInt: 1.25,
    fog: 0x727f8d, fogNear: 62, fogFar: 310,
    exposure: 0.94, head: 0.55, glow: 0.75, pole: 0.35,
  },
  night: {
    el: 12, az: 250, sunColor: 0x8fb0e8, sunInt: 0.24,
    hemiSky: 0x263a60, hemiGround: 0x0e131b, hemiInt: 0.58,
    skyTop: 0x04060c, skyHorizon: 0x0e1828, skyBottom: 0x05070a,
    glowC: 0x263c5e, haze: 0.08, disc: 0.0,
    cloudCover: 0.25, cloudTint: 0x22324d, cloudShadow: 0x080d16, stars: 1.0,
    wetness: 0.0, envInt: 0.52,
    fog: 0x060910, fogNear: 60, fogFar: 300,
    exposure: 0.98, head: 1.0, glow: 1.85, pole: 1.0,
  },
};

const AUTO = [
  [0.0, "night", -90],
  [4.5, "night", -20],
  [6.5, "golden", 15],
  [9.5, "midday", 55],
  [15.0, "midday", 115],
  [18.0, "golden", 160],
  [20.5, "night", 205],
  [24.0, "night", 270],
];

const ORDER = [null, "golden", "midday", "overcast", "wet", "night"];
const LABELS = { golden: "golden hour", midday: "midday", overcast: "overcast", wet: "wet", night: "night" };

const DOME_R = 660; // camera far is 800; dome follows the car
const SUN_DIST = 610;

/** Soft radial gradient canvas texture for the sun disc / halos. */
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

// Rayleigh-inspired horizon scattering + Mie solar corona + 2-layer drifting
// procedural clouds with directional sun rim lighting + crisp night starfield.
const DOME_FRAG = /* glsl */ `
uniform vec3 topColor;
uniform vec3 horizonColor;
uniform vec3 bottomColor;
uniform vec3 glowColor;
uniform vec3 cloudTint;
uniform vec3 cloudShadow;
uniform float glowAmount;
uniform float cloudCover;
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

float fbm(vec2 p) {
  float v = 0.0;
  float a = 0.5;
  mat2 rot = mat2(0.80, -0.60, 0.60, 0.80);
  for (int i = 0; i < 4; i++) {
    v += a * noise2(p);
    p = rot * p * 2.04 + vec2(13.7, 7.1);
    a *= 0.5;
  }
  return v;
}

void main() {
  vec3 d = normalize(vDir);

  // Physical Rayleigh-like vertical atmospheric ramp + subtle horizon belt
  float elev = clamp(d.y, 0.0, 1.0);
  vec3 col = d.y >= 0.0
    ? mix(horizonColor, topColor, pow(elev, 0.58))
    : mix(horizonColor, bottomColor, pow(clamp(-d.y, 0.0, 1.0), 0.45));

  // Mie forward-scattering around the sun (broad atmospheric bloom + tight aureole)
  float cosTheta = max(dot(d, normalize(sunDir)), 0.0);
  float mieBroad = pow(cosTheta, 5.0) * 0.55 + pow(cosTheta, 28.0) * 0.65;
  float horizonBoost = 1.0 + 0.6 * pow(1.0 - elev, 3.0);
  col += glowColor * (mieBroad * glowAmount * horizonBoost);

  // Night starfield above horizon (only computed when visible)
  if (starVis > 0.01 && d.y > 0.02) {
    vec2 sp = d.xz / (d.y + 0.25) * 95.0;
    vec2 cell = floor(sp);
    vec2 frac = fract(sp) - 0.5;
    float h = hash21(cell);
    if (h > 0.984) {
      vec2 jitter = vec2(hash21(cell + 17.3) - 0.5, hash21(cell + 41.9) - 0.5) * 0.55;
      float dist = length(frac - jitter);
      float star = smoothstep(0.16, 0.01, dist);
      float twinkle = 0.72 + 0.28 * sin(uTime * (2.2 + h * 4.0) + h * 62.8);
      vec3 starCol = mix(vec3(0.75, 0.86, 1.0), vec3(1.0, 0.92, 0.78), fract(h * 97.3));
      col += starCol * star * twinkle * starVis * smoothstep(0.02, 0.22, d.y) * 1.15;
    }
  }

  // Drifting volumetric-style cloud deck (planar projection for realistic horizon perspective)
  if (d.y > 0.015 && cloudCover > 0.01) {
    vec2 uv = d.xz / (d.y + 0.14) * 0.72;
    vec2 drift = vec2(uTime * 0.0085, uTime * 0.0032);
    float n1 = fbm(uv + drift);
    float n2 = fbm(uv * 2.15 - drift * 1.4 + vec2(4.2, -3.1));
    float density = n1 * 0.68 + n2 * 0.32;
    float thresh = mix(0.64, 0.28, cloudCover);
    float mask = smoothstep(thresh, thresh + 0.22, density);

    // Directional sun-lit silver/golden lining vs shaded underside
    float sunSample = fbm(uv + drift + normalize(sunDir).xz * 0.07);
    float lit = clamp((density - sunSample) * 2.4 + 0.55 + pow(cosTheta, 4.0) * 0.55, 0.0, 1.0);
    vec3 cCol = mix(cloudShadow, cloudTint, lit);
    cCol += glowColor * pow(cosTheta, 8.0) * glowAmount * 0.45 * (1.0 - mask * 0.45);

    // Horizon aerial perspective fade so clouds dissolve cleanly into haze
    float horizonFade = smoothstep(0.015, 0.20, d.y);
    col = mix(col, cCol, mask * horizonFade * mix(0.68, 0.94, cloudCover));
  }

  gl_FragColor = vec4(col, 1.0);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

// Lightweight sky/sun equirectangular-style cube source for PMREMGenerator IBL.
const ENV_FRAG = /* glsl */ `
uniform vec3 topColor;
uniform vec3 horizonColor;
uniform vec3 bottomColor;
uniform vec3 glowColor;
uniform float glowAmount;
uniform float sunBoost;
uniform vec3 sunDir;
varying vec3 vDir;
void main() {
  vec3 d = normalize(vDir);
  vec3 col = d.y >= 0.0
    ? mix(horizonColor, topColor, pow(clamp(d.y, 0.0, 1.0), 0.55))
    : mix(horizonColor * 0.45, bottomColor, pow(clamp(-d.y, 0.0, 1.0), 0.4));
  float cs = max(dot(d, normalize(sunDir)), 0.0);
  col += glowColor * (pow(cs, 6.0) * glowAmount * 0.9 + pow(cs, 96.0) * sunBoost);
  gl_FragColor = vec4(col, 1.0);
}`;

export class Lighting {
  /** @param car the player car group — headlights are parented to it.
   *  @param glowMaterials materials whose emissiveIntensity tracks the condition.
   *  @param renderer optional — drives exposure + procedural PMREM IBL envmap.
   *  @param track optional parsed TRK1 — places instanced trackside floodlights.
   *  @param surfaceHooks optional { asphaltMat, grassMat } for dynamic wetness. */
  constructor(scene, car, glowMaterials = [], renderer = null, track = null, surfaceHooks = null) {
    this.scene = scene;
    this.car = car;
    this.glow = glowMaterials;
    this.renderer = renderer;
    this.surfaceHooks = surfaceHooks;
    this.override = null; // null = auto (local clock)
    this.wetness = 0;
    this._t0 = performance.now();
    this._envCache = new Map();
    this._activeEnvKey = null;

    // ---- key light + shadow rig (the ONE shadow caster) + hemisphere fill
    this.hemi = new THREE.HemisphereLight(0xffd2ad, 0x4a4038, 0.62);
    this.sun = new THREE.DirectionalLight(0xffb066, 2.75);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -48; sc.right = 48; sc.top = 48; sc.bottom = -48;
    sc.near = 10; sc.far = 420;
    sc.updateProjectionMatrix();
    this.sun.shadow.bias = -0.00035;
    this.sun.shadow.normalBias = 0.022;
    scene.add(this.hemi, this.sun, this.sun.target);

    // ---- headlights: high-output motorsport LED spot pair on the nose.
    this.heads = [-0.36, 0.36].map((x) => {
      const s = new THREE.SpotLight(0xfff6e4, 0, 115, 0.66, 0.55, 1);
      s.position.set(x, 0.36, 1.68);
      s.target.position.set(x * 5.0, -1.1, 38);
      car.add(s);
      car.add(s.target);
      return s;
    });

    this._buildSky();
    this._initPMREM();
    if (track) this._buildPoles(track);
    this._applyShadowFlags();

    this._c1 = new THREE.Color();
    this._c2 = new THREE.Color();
    this._dir = new THREE.Vector3(0, 1, 0);
    if (!(scene.background instanceof THREE.Color)) scene.background = new THREE.Color();
    this.update();
  }

  _initPMREM() {
    if (!this.renderer || typeof THREE.PMREMGenerator !== "function") return;
    try {
      this._pmrem = new THREE.PMREMGenerator(this.renderer);
      this._pmrem.compileEquirectangularShader?.();
      this._envUniforms = {
        topColor: { value: new THREE.Color(0x29549c) },
        horizonColor: { value: new THREE.Color(0xffb36c) },
        bottomColor: { value: new THREE.Color(0x33302e) },
        glowColor: { value: new THREE.Color(0xffc680) },
        glowAmount: { value: 0.9 },
        sunBoost: { value: 12.0 },
        sunDir: { value: new THREE.Vector3(0, 0.26, 0.97) },
      };
      this._envScene = new THREE.Scene();
      const envSphere = new THREE.Mesh(
        new THREE.SphereGeometry(20, 24, 14),
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

  /** Bake (or fetch cached) PMREM sky/sun environment map for PBR reflections. */
  _syncEnvMap(key, top, hor, bot, glow, haze, sunBoost, dir) {
    if (!this._pmrem) return;
    if (key === this._activeEnvKey) return;
    this._activeEnvKey = key;
    if (key !== "auto" && this._envCache.has(key)) {
      this.scene.environment = this._envCache.get(key).texture;
      return;
    }
    const U = this._envUniforms;
    U.topColor.value.copy(top);
    U.horizonColor.value.copy(hor);
    U.bottomColor.value.copy(bot);
    U.glowColor.value.copy(glow);
    U.glowAmount.value = haze;
    U.sunBoost.value = sunBoost;
    U.sunDir.value.copy(dir);
    const rt = this._pmrem.fromScene(this._envScene, 0.035);
    if (key === "auto") {
      this._autoEnvRT?.dispose();
      this._autoEnvRT = rt;
    } else {
      this._envCache.set(key, rt);
    }
    this.scene.environment = rt.texture;
  }

  _buildSky() {
    this.domeUniforms = {
      topColor: { value: new THREE.Color(0x29549c) },
      horizonColor: { value: new THREE.Color(0xffb36c) },
      bottomColor: { value: new THREE.Color(0x33302e) },
      glowColor: { value: new THREE.Color(0xffc680) },
      cloudTint: { value: new THREE.Color(0xffd2a4) },
      cloudShadow: { value: new THREE.Color(0x5c4654) },
      glowAmount: { value: 0.95 },
      cloudCover: { value: 0.44 },
      starVis: { value: 0.0 },
      uTime: { value: 0.0 },
      sunDir: { value: new THREE.Vector3(0, 0.26, 0.97) },
    };
    const mat = new THREE.ShaderMaterial({
      uniforms: this.domeUniforms,
      vertexShader: DOME_VERT,
      fragmentShader: DOME_FRAG,
      side: THREE.BackSide,
      depthWrite: false,
      depthTest: false,
    });
    this.dome = new THREE.Mesh(new THREE.SphereGeometry(DOME_R, 40, 20), mat);
    this.dome.renderOrder = -100;
    this.dome.frustumCulled = false;
    this.scene.add(this.dome);

    const discTex = radialTex([
      [0, "rgba(255,255,255,1)"], [0.38, "rgba(255,255,255,1)"],
      [0.52, "rgba(255,248,230,0.42)"], [1, "rgba(255,245,220,0)"],
    ]);
    this._haloTex = radialTex([
      [0, "rgba(255,255,255,0.58)"], [0.28, "rgba(255,255,255,0.18)"], [1, "rgba(255,255,255,0)"],
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
    this.sunDisc = sprite(discTex, 48);
    this.sunHalo = sprite(this._haloTex, 195);
  }

  // ---- 12 instanced trackside cobra-head floodlights around the circuit.
  // Cheap (2 InstancedMeshes) and transforms night mode into a Marina Bay-style
  // lit circuit with dramatic overhead lamp rhythm.
  _buildPoles(track) {
    const n = track.S;
    const poleGeo = new THREE.CylinderGeometry(0.08, 0.14, 8.2, 6);
    poleGeo.translate(0, 4.1, 0);
    const headGeo = new THREE.BoxGeometry(1.7, 0.18, 0.38);
    headGeo.translate(-0.7, 8.05, 0);
    const poleMat = new THREE.MeshStandardMaterial({ color: 0x222831, roughness: 0.55, metalness: 0.65 });
    this.poleHeadMat = new THREE.MeshStandardMaterial({
      color: 0x181e27, roughness: 0.4, metalness: 0.3, emissive: 0xffe4b8, emissiveIntensity: 0,
    });
    this.poleGlowMat = new THREE.SpriteMaterial({
      map: this._haloTex, color: 0xffdba4, transparent: true, opacity: 0,
      blending: THREE.AdditiveBlending, depthWrite: false, fog: false, toneMapped: false,
    });
    const count = 12;
    const idxs = [];
    for (let k = 0; k < count; k++) idxs.push(Math.floor((n * (k + 0.5)) / count));
    const poles = new THREE.InstancedMesh(poleGeo, poleMat, idxs.length);
    const heads = new THREE.InstancedMesh(headGeo, this.poleHeadMat, idxs.length);
    poles.castShadow = poles.receiveShadow = true;
    const m4 = new THREE.Matrix4();
    idxs.forEach((i, k) => {
      const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      const pos = c.clone().addScaledVector(side, track.width[i] / 2 + 4.8).addScaledVector(u, -1.3);
      m4.makeBasis(side, u, t).setPosition(pos);
      poles.setMatrixAt(k, m4);
      heads.setMatrixAt(k, m4);
      const spr = new THREE.Sprite(this.poleGlowMat);
      spr.position.copy(pos).addScaledVector(u, 7.95).addScaledVector(side, -0.8);
      spr.scale.set(6.5, 6.5, 1);
      this.scene.add(spr);
    });
    this.scene.add(poles, heads);
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
      if (!g.boundingSphere) g.computeBoundingSphere();
      if (g.boundingSphere && g.boundingSphere.radius < 50) o.castShadow = true;
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

  /** Cheap per-frame update: blends condition, advances cloud drift, moves shadow
   *  frustum & sky dome with car, updates wet-asphalt PBR sheen, syncs IBL. */
  update() {
    let A, B, u, az, envKey;
    if (this.override) {
      A = B = COND[this.override];
      u = 0;
      az = A.az;
      envKey = this.override;
    } else {
      const h = this.hour();
      A = B = COND.night; u = 0; az = AUTO[0][2];
      for (let i = 0; i + 1 < AUTO.length; i++) {
        if (h >= AUTO[i][0] && h < AUTO[i + 1][0]) {
          u = smoothstep((h - AUTO[i][0]) / (AUTO[i + 1][0] - AUTO[i][0]));
          A = COND[AUTO[i][1]];
          B = COND[AUTO[i + 1][1]];
          az = AUTO[i][2] + (AUTO[i + 1][2] - AUTO[i][2]) * u;
          break;
        }
      }
      // Refresh auto PMREM at most once per ~20 seconds (sun barely moves).
      envKey = `auto-${Math.floor(h * 180)}`;
    }
    const f = (k) => A[k] + (B[k] - A[k]) * u;
    const mix = (k) => this._c1.set(A[k]).lerp(this._c2.set(B[k]), u);

    this.wetness = f("wetness");

    // Atmosphere: fog + background fallback
    this.scene.background.copy(mix("fog"));
    if (this.scene.fog) {
      this.scene.fog.color.copy(mix("fog"));
      this.scene.fog.near = f("fogNear");
      this.scene.fog.far = f("fogFar");
    }
    this.hemi.color.copy(mix("hemiSky"));
    this.hemi.groundColor.copy(mix("hemiGround"));
    this.hemi.intensity = f("hemiInt");
    this.sun.color.copy(mix("sunColor"));
    this.sun.intensity = f("sunInt");
    if (this.renderer) this.renderer.toneMappingExposure = f("exposure");
    if ("environmentIntensity" in this.scene) this.scene.environmentIntensity = f("envInt");

    // Sun direction (drives shadows, disc, dome haze, and PMREM IBL together)
    const el = f("el") * DEG, azr = az * DEG;
    this._dir.set(Math.cos(azr) * Math.cos(el), Math.sin(el), Math.sin(azr) * Math.cos(el));

    // Shadow frustum follows the car, snapped to a 2 m grid to kill shimmer
    const p = this.car.position;
    const sx = Math.round(p.x / 2) * 2, sy = Math.round(p.y / 2) * 2, sz = Math.round(p.z / 2) * 2;
    this.sun.target.position.set(sx, sy, sz);
    this.sun.position.set(sx, sy, sz).addScaledVector(this._dir, 180);

    // Sky dome + drifting clouds + stars
    this.dome.position.copy(p);
    const U = this.domeUniforms;
    U.topColor.value.copy(mix("skyTop"));
    U.horizonColor.value.copy(mix("skyHorizon"));
    U.bottomColor.value.copy(mix("skyBottom"));
    U.glowColor.value.copy(mix("glowC"));
    U.cloudTint.value.copy(mix("cloudTint"));
    U.cloudShadow.value.copy(mix("cloudShadow"));
    U.glowAmount.value = f("haze");
    U.cloudCover.value = f("cloudCover");
    U.starVis.value = f("stars");
    U.uTime.value = (performance.now() - this._t0) * 0.001;
    U.sunDir.value.copy(this._dir);

    // Procedural PMREM environment map (real sky + sun reflections on PBR paint/metal/wet road)
    const sunBoost = f("disc") > 0.05 ? 14.0 : this.wetness > 0.5 ? 4.5 : 1.5;
    this._syncEnvMap(
      envKey,
      U.topColor.value,
      U.horizonColor.value,
      U.bottomColor.value,
      U.glowColor.value,
      U.glowAmount.value,
      sunBoost,
      this._dir,
    );

    // Dynamic wet-tarmac PBR response: darkened saturated wet stone + low roughness + sky reflections
    if (this.surfaceHooks?.asphaltMat) {
      const am = this.surfaceHooks.asphaltMat;
      const w = this.wetness;
      am.roughness = 0.86 - 0.62 * w; // dry ~0.86 → wet mirror-sheen ~0.24
      am.metalness = 0.04 + 0.14 * w;
      am.envMapIntensity = 0.55 + 1.45 * w;
      am.color.setRGB(1.0 - 0.28 * w, 1.0 - 0.25 * w, 1.0 - 0.20 * w);
    }

    const disc = f("disc");
    this.sunDisc.visible = this.sunHalo.visible = disc > 0.01;
    if (this.sunDisc.visible) {
      this.sunDisc.position.copy(p).addScaledVector(this._dir, SUN_DIST);
      this.sunHalo.position.copy(this.sunDisc.position);
      this.sunDisc.material.opacity = disc;
      this.sunHalo.material.opacity = disc * 0.65;
      this.sunDisc.material.color.copy(mix("glowC")).lerp(this._c2.set(0xffffff), 0.5);
      this.sunHalo.material.color.copy(this.sunDisc.material.color);
    }

    // Night rig: headlights, gantry glow, trackside floodlights
    for (const s of this.heads) s.intensity = f("head") * 44;
    for (const m of this.glow) m.emissiveIntensity = f("glow");
    if (this.poleHeadMat) {
      const pf = f("pole");
      this.poleHeadMat.emissiveIntensity = 3.2 * pf;
      this.poleGlowMat.opacity = 0.58 * pf;
    }
  }
}

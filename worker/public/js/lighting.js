// APEX P0 "Foundation" lighting core (docs/APEX.md §1) — sky dome + sun disc,
// one shadow-casting key light, ACES exposure, five reference conditions:
//
//   condition    sun                       sky/fog              extras
//   golden hour  low warm (~15°, +X-ish)   warm haze horizon    long shadows (DEFAULT)
//   midday       high neutral (~62°)       crisp blue           tight shadows
//   overcast     faint diffuse             grey dome, no disc   soft ambient
//   wet          faint + boosted hemi      dark grey, no disc   dark ground, car glistens
//   night        off                       deep sky, no disc    headlights, glow, light poles
//
// Auto mode follows the PLAYER'S LOCAL CLOCK (purely cosmetic — physics and
// replays never see it), blending night → golden → midday → golden → night
// across the day; overcast/wet are manual looks. Press L (wired in app.js) to
// cycle auto → golden hour → midday → overcast → wet → night → auto.
//
// Night stays drivable: two spotlight headlights parented to the car, the
// gantry strip/banner emissives crank up (glowMaterials), three instanced
// trackside light poles wake up (emissive head + soft additive glow sprite —
// the "bloom" look without a postprocessing pass), and the track's edge lines
// + kerb paint use Line/MeshBasicMaterial which ignore lighting entirely.
//
// Shadow rig: single 2048² PCFSoft directional shadow map, ±45 m ortho frustum
// snapped to a 2 m grid around the car. Cast/receive flags are set ONCE at
// construction by traversing the scene: any opaque lit (lambert/standard/
// phong) mesh receives; those whose geometry bounding sphere is < 50 m also
// cast (car, wheels, trees, tire barriers, gantry, boards, poles) — the
// track-sized surfaces (terrain, ribbon, skirts, ground disc) only receive.
// Construct AFTER all scene meshes exist (app.js does).
//
// Works with both Lambert (current) and MeshStandardMaterial (PBR pass):
// hemisphere + directional intensities are tuned for ACESFilmic at the
// per-condition exposures below.
import * as THREE from "../vendor/three.module.js";

const smoothstep = (u) => u * u * (3 - 2 * u);
const DEG = Math.PI / 180;
const v3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);

// Per-condition parameters. Colors as hex, scalars lerped.
//   el/az    sun elevation/azimuth (deg; az used in override mode — auto mode
//            sweeps azimuth east→west via the AUTO anchors below)
//   disc     sun-disc sprite opacity (0 = hidden: overcast/wet/night)
//   haze     forward-scatter glow strength baked into the dome shader
//   exposure renderer.toneMappingExposure (ACESFilmic)
//   head     headlight level 0..1 · glow = gantry emissiveIntensity ·
//   pole     trackside light-pole level 0..1
const COND = {
  golden: {
    el: 15, az: 15, sunColor: 0xffb066, sunInt: 2.6,
    hemiSky: 0xffd2ad, hemiGround: 0x4a4038, hemiInt: 0.55,
    skyTop: 0x3a66a8, skyHorizon: 0xffb877, skyBottom: 0x33302e,
    glowC: 0xffc27d, haze: 0.9, disc: 1.0,
    fog: 0xe0a877, fogNear: 95, fogFar: 430,
    exposure: 1.12, head: 0.15, glow: 0.5, pole: 0.05,
  },
  midday: {
    el: 62, az: 85, sunColor: 0xfff3dd, sunInt: 3.0,
    hemiSky: 0xbfd8ff, hemiGround: 0x40464e, hemiInt: 0.95,
    skyTop: 0x2c66c4, skyHorizon: 0xa9cdec, skyBottom: 0x3a4149,
    glowC: 0xffffff, haze: 0.3, disc: 1.0,
    fog: 0xa7c2de, fogNear: 160, fogFar: 540,
    exposure: 1.02, head: 0.0, glow: 0.15, pole: 0.0,
  },
  overcast: {
    el: 52, az: 70, sunColor: 0xdde4ec, sunInt: 0.4,
    hemiSky: 0xb2bac4, hemiGround: 0x3d4147, hemiInt: 1.15,
    skyTop: 0x5c6774, skyHorizon: 0x9ba4ad, skyBottom: 0x35393f,
    glowC: 0xc9d0d8, haze: 0.1, disc: 0.0,
    fog: 0x98a1aa, fogNear: 75, fogFar: 340,
    exposure: 0.95, head: 0.25, glow: 0.45, pole: 0.1,
  },
  wet: {
    el: 50, az: 70, sunColor: 0xccd6e2, sunInt: 0.35,
    hemiSky: 0xa8bccf, hemiGround: 0x1f242b, hemiInt: 1.5,
    skyTop: 0x46505c, skyHorizon: 0x7f8b97, skyBottom: 0x22262c,
    glowC: 0xb9c4cf, haze: 0.08, disc: 0.0,
    fog: 0x76828e, fogNear: 60, fogFar: 300,
    exposure: 0.9, head: 0.45, glow: 0.65, pole: 0.25,
  },
  night: {
    el: 8, az: 250, sunColor: 0x9db8e8, sunInt: 0.0,
    hemiSky: 0x1d2b47, hemiGround: 0x0a0d12, hemiInt: 0.4,
    skyTop: 0x05060a, skyHorizon: 0x101a28, skyBottom: 0x05070a,
    glowC: 0x203048, haze: 0.05, disc: 0.0,
    fog: 0x05070c, fogNear: 48, fogFar: 270,
    exposure: 0.88, head: 1.0, glow: 1.6, pole: 1.0,
  },
};

// Auto (local clock) anchors: [hour, condition, sun azimuth°]. The condition
// params blend smoothly across each segment; azimuth is anchored separately
// so the sun sweeps east → west through the day (240+30 ≡ 270 wraps to -90).
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

const DOME_R = 650; // camera far is 800; dome follows the car
const SUN_DIST = 600; // sun sprites sit just inside the dome

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

const DOME_FRAG = /* glsl */ `
uniform vec3 topColor;
uniform vec3 horizonColor;
uniform vec3 bottomColor;
uniform vec3 glowColor;
uniform float glowAmount;
uniform vec3 sunDir;
varying vec3 vDir;
void main() {
  vec3 d = normalize(vDir);
  vec3 col = d.y >= 0.0
    ? mix(horizonColor, topColor, pow(clamp(d.y, 0.0, 1.0), 0.65))
    : mix(horizonColor, bottomColor, pow(clamp(-d.y, 0.0, 1.0), 0.5));
  // forward-scatter haze around the sun (sunrise/sunset warmth)
  float s = pow(max(dot(d, sunDir), 0.0), 6.0);
  col += glowColor * (s * glowAmount);
  gl_FragColor = vec4(col, 1.0);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

export class Lighting {
  /** @param car the player car group — headlights are parented to it.
   *  @param glowMaterials materials whose emissiveIntensity tracks the
   *  condition (gantry strip/banner from buildAmbience).
   *  @param renderer optional — drives per-condition toneMappingExposure.
   *  @param track optional parsed TRK1 — places 3 trackside light poles.
   *  Construct AFTER every scene mesh exists: shadow cast/receive flags are
   *  set once here via scene traversal. */
  constructor(scene, car, glowMaterials = [], renderer = null, track = null) {
    this.scene = scene;
    this.car = car;
    this.glow = glowMaterials;
    this.renderer = renderer;
    this.override = null; // null = auto (local clock)

    // ---- key light + shadow rig (the ONE shadow caster) + hemisphere fill
    this.hemi = new THREE.HemisphereLight(0xffd2ad, 0x4a4038, 0.55);
    this.sun = new THREE.DirectionalLight(0xffb066, 2.6);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const sc = this.sun.shadow.camera;
    sc.left = -45; sc.right = 45; sc.top = 45; sc.bottom = -45;
    sc.near = 10; sc.far = 400;
    sc.updateProjectionMatrix();
    // normalBias shrinks shadows toward their caster by nb/tan(sunElev) —
    // at the 15° golden-hour sun, the old 0.8 m pushed shadows ~3 m off the
    // contact patches and the car read as hovering. Keep it under one shadow
    // texel (frustum 90 m / 2048 ≈ 4.4 cm) and lean on depth bias for acne.
    this.sun.shadow.bias = -0.0004;
    this.sun.shadow.normalBias = 0.02;
    scene.add(this.hemi, this.sun, this.sun.target);

    // ---- headlights: spot pair on the nose aiming forward/down. decay=1 so
    // a modest intensity carries ~30 m of tarmac (three r155+ physical lights).
    this.heads = [-0.32, 0.32].map((x) => {
      const s = new THREE.SpotLight(0xf7f2df, 0, 75, 0.5, 0.55, 1);
      s.position.set(x, 0.32, 1.6);
      s.target.position.set(x * 4, -1.2, 30);
      car.add(s);
      car.add(s.target);
      return s;
    });

    this._buildSky();
    if (track) this._buildPoles(track);
    this._applyShadowFlags();

    this._c1 = new THREE.Color();
    this._c2 = new THREE.Color();
    this._dir = new THREE.Vector3(0, 1, 0);
    if (!(scene.background instanceof THREE.Color)) scene.background = new THREE.Color();
    this.update();
  }

  // ---- sky dome (gradient shader, fog-exempt, renders behind everything)
  // + sun disc / halo sprites (additive, depth-tested so terrain occludes).
  _buildSky() {
    this.domeUniforms = {
      topColor: { value: new THREE.Color(0x3a66a8) },
      horizonColor: { value: new THREE.Color(0xffb877) },
      bottomColor: { value: new THREE.Color(0x33302e) },
      glowColor: { value: new THREE.Color(0xffc27d) },
      glowAmount: { value: 0.9 },
      sunDir: { value: new THREE.Vector3(0, 0.26, 0.97) },
    };
    const mat = new THREE.ShaderMaterial({
      uniforms: this.domeUniforms,
      vertexShader: DOME_VERT,
      fragmentShader: DOME_FRAG,
      side: THREE.BackSide,
      depthWrite: false,
      depthTest: false, // drawn first (renderOrder) — everything paints over it
    });
    this.dome = new THREE.Mesh(new THREE.SphereGeometry(DOME_R, 32, 15), mat);
    this.dome.renderOrder = -100;
    this.dome.frustumCulled = false;
    this.scene.add(this.dome);

    const discTex = radialTex([
      [0, "rgba(255,255,255,1)"], [0.4, "rgba(255,255,255,1)"],
      [0.52, "rgba(255,250,235,0.35)"], [1, "rgba(255,250,235,0)"],
    ]);
    this._haloTex = radialTex([
      [0, "rgba(255,255,255,0.55)"], [0.3, "rgba(255,255,255,0.16)"], [1, "rgba(255,255,255,0)"],
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
    this.sunDisc = sprite(discTex, 46);
    this.sunHalo = sprite(this._haloTex, 170); // subtle glow halo
  }

  // ---- 3 instanced trackside light poles (night rig). Emissive cobra head +
  // soft additive glow sprite fakes the bloom look — no postprocessing pass.
  _buildPoles(track) {
    const n = track.S;
    const poleGeo = new THREE.CylinderGeometry(0.08, 0.13, 7.5, 6);
    poleGeo.translate(0, 3.75, 0);
    const headGeo = new THREE.BoxGeometry(1.5, 0.16, 0.34);
    headGeo.translate(-0.6, 7.3, 0); // arm reaches back toward the track (-x = -side)
    const poleMat = new THREE.MeshLambertMaterial({ color: 0x1b2129 });
    this.poleHeadMat = new THREE.MeshLambertMaterial({ color: 0x14181f, emissive: 0xffe2b0, emissiveIntensity: 0 });
    this.poleGlowMat = new THREE.SpriteMaterial({
      map: this._haloTex, color: 0xffd9a0, transparent: true, opacity: 0,
      blending: THREE.AdditiveBlending, depthWrite: false, fog: false, toneMapped: false,
    });
    const idxs = [Math.floor(n * 0.18), Math.floor(n * 0.5), Math.floor(n * 0.82)];
    const poles = new THREE.InstancedMesh(poleGeo, poleMat, idxs.length);
    const heads = new THREE.InstancedMesh(headGeo, this.poleHeadMat, idxs.length);
    poles.castShadow = poles.receiveShadow = true;
    const m4 = new THREE.Matrix4();
    idxs.forEach((i, k) => {
      const c = v3(track.center[i]), u = v3(track.up[i]), t = v3(track.tangent[i]);
      const side = new THREE.Vector3().crossVectors(u, t).normalize();
      // base sits past the shoulder (which drops ~1.2 m below the ribbon)
      const pos = c.clone().addScaledVector(side, track.width[i] / 2 + 4.5).addScaledVector(u, -1.4);
      m4.makeBasis(side, u, t).setPosition(pos);
      poles.setMatrixAt(k, m4);
      heads.setMatrixAt(k, m4);
      const spr = new THREE.Sprite(this.poleGlowMat);
      spr.position.copy(pos).addScaledVector(u, 7.25).addScaledVector(side, -0.7);
      spr.scale.set(5, 5, 1);
      this.scene.add(spr);
    });
    this.scene.add(poles, heads);
  }

  // ---- one-time shadow flags. Opaque lit meshes receive; small ones (< 50 m
  // bounding sphere: car, wheels, props, instanced trees/barriers) also cast.
  // Basic/Line/Shader materials (paint, sky) and transparent ones (ghost car,
  // gates, racing line) are skipped entirely.
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

  /** L key: auto → golden hour → midday → overcast → wet → night → auto.
   *  Returns a label for #statusText. */
  cycle() {
    this.override = ORDER[(ORDER.indexOf(this.override) + 1) % ORDER.length];
    this.update();
    return this.override ? LABELS[this.override] : "auto (local clock)";
  }

  hour() {
    const d = new Date();
    return d.getHours() + d.getMinutes() / 60 + d.getSeconds() / 3600;
  }

  /** Cheap enough to call every frame (a handful of lerps + sprite moves).
   *  Also slaves the shadow frustum + sky dome to the car position. */
  update() {
    let A, B, u, az;
    if (this.override) {
      A = B = COND[this.override];
      u = 0;
      az = A.az;
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
    }
    const f = (k) => A[k] + (B[k] - A[k]) * u;
    const mix = (k) => this._c1.set(A[k]).lerp(this._c2.set(B[k]), u);

    // atmosphere: fog + background fallback (the dome covers the whole view)
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

    // sun direction (drives shadows, disc position and dome haze together)
    const el = f("el") * DEG, azr = az * DEG;
    this._dir.set(Math.cos(azr) * Math.cos(el), Math.sin(el), Math.sin(azr) * Math.cos(el));

    // shadow frustum follows the car, snapped to a 2 m grid to calm shimmer
    const p = this.car.position;
    const sx = Math.round(p.x / 2) * 2, sy = Math.round(p.y / 2) * 2, sz = Math.round(p.z / 2) * 2;
    this.sun.target.position.set(sx, sy, sz);
    this.sun.position.set(sx, sy, sz).addScaledVector(this._dir, 180);

    // sky dome + sun disc/halo
    this.dome.position.copy(p);
    const U = this.domeUniforms;
    U.topColor.value.copy(mix("skyTop"));
    U.horizonColor.value.copy(mix("skyHorizon"));
    U.bottomColor.value.copy(mix("skyBottom"));
    U.glowColor.value.copy(mix("glowC"));
    U.glowAmount.value = f("haze");
    U.sunDir.value.copy(this._dir);

    const disc = f("disc");
    this.sunDisc.visible = this.sunHalo.visible = disc > 0.01;
    if (this.sunDisc.visible) {
      this.sunDisc.position.copy(p).addScaledVector(this._dir, SUN_DIST);
      this.sunHalo.position.copy(this.sunDisc.position);
      this.sunDisc.material.opacity = disc;
      this.sunHalo.material.opacity = disc * 0.6;
      this.sunDisc.material.color.copy(mix("glowC")).lerp(this._c2.set(0xffffff), 0.5);
      this.sunHalo.material.color.copy(this.sunDisc.material.color);
    }

    // night rig: headlights, gantry glow, trackside pole lights
    for (const s of this.heads) s.intensity = f("head") * 14;
    for (const m of this.glow) m.emissiveIntensity = f("glow");
    if (this.poleHeadMat) {
      const pf = f("pole");
      this.poleHeadMat.emissiveIntensity = 2.5 * pf;
      this.poleGlowMat.opacity = 0.5 * pf;
    }
  }
}

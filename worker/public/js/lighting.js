// Time-of-day lighting driven by the PLAYER'S LOCAL CLOCK — purely cosmetic
// (physics/replays never see it). Four keyframed looks anchored at local
// 0h (night), 6h (dawn), 12h (day), 18h (dusk); the current hour blends
// smoothly between the two neighbouring keyframes.
//
//   key    hour  sky/fog             sun                 headlights  glow
//   night   0    near-black blue     off                 on          bright
//   dawn    6    pink-orange horizon low warm sun        fading      medium
//   day    12    bright neutral      high, 1.7×          off         subtle
//   dusk   18    orange-red horizon  low red sun         waking      medium
//
// Night stays drivable: two spotlight headlights parented to the car, the
// gantry strip/banner emissives crank up (glowMaterials), and the track's
// edge lines + kerb paint use Line/MeshBasicMaterial which ignore lighting
// entirely, so the racing line always reads. Press L (wired in app.js) to
// cycle auto → dawn → day → dusk → night → auto for testing.
import * as THREE from "../vendor/three.module.js";

const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const smoothstep = (u) => u * u * (3 - 2 * u);

// Keyframes: colors as hex, scalars lerped. head = headlight level 0..1,
// glow = emissiveIntensity for the gantry strip/banner materials.
const KEYS = {
  night: { sky: 0x04060c, fog: 0x04060c, fogNear: 55, fogFar: 300,
           hemiSky: 0x24344d, hemiGround: 0x090c12, hemiInt: 0.38,
           sunColor: 0x9db8e8, sunInt: 0.0, head: 1.0, glow: 1.5 },
  dawn:  { sky: 0x53324a, fog: 0x8a4a3e, fogNear: 110, fogFar: 420,
           hemiSky: 0xffb289, hemiGround: 0x36323e, hemiInt: 0.75,
           sunColor: 0xffa864, sunInt: 1.15, head: 0.3, glow: 0.7 },
  day:   { sky: 0x87aed6, fog: 0x87aed6, fogNear: 150, fogFar: 480,
           hemiSky: 0xbdd4ff, hemiGround: 0x2a2f38, hemiInt: 1.25,
           sunColor: 0xfff2d8, sunInt: 1.7, head: 0.0, glow: 0.15 },
  dusk:  { sky: 0x3a1f33, fog: 0x9c4a2e, fogNear: 100, fogFar: 400,
           hemiSky: 0xff9a66, hemiGround: 0x2b2531, hemiInt: 0.7,
           sunColor: 0xff7038, sunInt: 1.25, head: 0.4, glow: 0.9 },
};
// Hour anchors around the 24 h wheel (wraps: 18h→24h blends dusk→night).
const SEGMENTS = [
  [0, 6, KEYS.night, KEYS.dawn],
  [6, 12, KEYS.dawn, KEYS.day],
  [12, 18, KEYS.day, KEYS.dusk],
  [18, 24, KEYS.dusk, KEYS.night],
];
const OVERRIDE_HOURS = { dawn: 6, day: 12, dusk: 18, night: 0 };
const OVERRIDE_ORDER = [null, "dawn", "day", "dusk", "night"];

export class Lighting {
  /** @param car the player car group — headlights are parented to it.
   *  @param glowMaterials materials whose emissiveIntensity tracks the
   *  keyframes (gantry strip/banner from buildAmbience). */
  constructor(scene, car, glowMaterials = []) {
    this.scene = scene;
    this.glow = glowMaterials;
    this.override = null; // null = auto (local clock)

    this.hemi = new THREE.HemisphereLight(0xbdd4ff, 0x2a2f38, 1.25);
    this.sun = new THREE.DirectionalLight(0xfff2d8, 1.7);
    this.sun.position.set(120, 180, 60);
    scene.add(this.hemi, this.sun);

    // Headlights: spot pair on the nose aiming forward/down. decay=1 so a
    // modest intensity carries ~30 m of tarmac (three r155+ physical lights).
    this.heads = [-0.32, 0.32].map((x) => {
      const s = new THREE.SpotLight(0xf7f2df, 0, 75, 0.5, 0.55, 1);
      s.position.set(x, 0.32, 1.6);
      s.target.position.set(x * 4, -1.2, 30);
      car.add(s);
      car.add(s.target);
      return s;
    });

    this._c1 = new THREE.Color();
    this._c2 = new THREE.Color();
    if (!(scene.background instanceof THREE.Color)) scene.background = new THREE.Color();
    this.update();
  }

  /** L key: auto → dawn → day → dusk → night → auto. Returns a label for
   *  #statusText. */
  cycle() {
    this.override = OVERRIDE_ORDER[(OVERRIDE_ORDER.indexOf(this.override) + 1) % OVERRIDE_ORDER.length];
    this.update();
    return this.override ?? "auto (local clock)";
  }

  hour() {
    if (this.override) return OVERRIDE_HOURS[this.override];
    const d = new Date();
    return d.getHours() + d.getMinutes() / 60 + d.getSeconds() / 3600;
  }

  /** Cheap enough to call every frame (a handful of color lerps). */
  update() {
    const h = this.hour();
    let A = KEYS.night, B = KEYS.night, u = 0;
    for (const [a, b, ka, kb] of SEGMENTS) {
      if (h >= a && h < b) {
        A = ka; B = kb;
        u = smoothstep((h - a) / (b - a));
        break;
      }
    }
    const f = (k) => A[k] + (B[k] - A[k]) * u;
    const mix = (k) => this._c1.set(A[k]).lerp(this._c2.set(B[k]), u);

    this.scene.background.copy(mix("sky"));
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

    // Sun path: simple sinusoid — solar noon peaks at ~60° elevation, sweeps
    // east → west through the day. Below-horizon hours keep a token low angle
    // (intensity is 0 there anyway, direction just needs to stay sane).
    const dayT = clamp((h - 6) / 12, 0, 1);
    const elev = Math.max(0.06, Math.sin(Math.PI * dayT) * (Math.PI / 3));
    const az = Math.PI * (0.12 + 0.76 * dayT);
    this.sun.position
      .set(Math.cos(az) * Math.cos(elev), Math.sin(elev), Math.sin(az) * Math.cos(elev))
      .multiplyScalar(320);

    for (const s of this.heads) s.intensity = f("head") * 10;
    for (const m of this.glow) m.emissiveIntensity = f("glow");
  }
}

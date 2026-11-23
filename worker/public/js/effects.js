// 2026 AAA Motorsport VFX Engine — 100% read-only on SimStateV1 + lighting,
// zero physics coupling, ZERO per-frame allocations inside update():
//   • Feathered procedural tire-tread skidmark ribbons (MAX_SKIDS = 1100):
//     vulcanized hot rubber on dry tarmac, translucent water-clearing wake on
//     wet asphalt, earthy dark ruts off-track;
//   • 128×128 multi-octave volumetric-look cloud puff billboards (MAX_SMOKE = 340)
//     with true per-instance linear alpha fading, continuous spin & turbulence
//     for tire smoke, golden-lit off-track gravel roost, and multi-tiered F1
//     wet-weather rooster-tail spray + diffuser vortex mist;
//   • HDR incandescent titanium spark showers (MAX_SPARKS = 160) with ground
//     bounce for chassis impacts, high-speed kerb hits, and floor-plank bottoming;
//   • Automatic gear-change / overrun exhaust backfire pops with dual-layer HDR
//     blue-orange plasma jet, glowing ember burst, rear flash PointLight, and
//     `effects.backfireGlow` (0..1) sync hook for car.js;
//   • 3D atmospheric diagonal rain streaks (MAX_RAIN = 360) wrapping seamlessly
//     around the camera when wetness > 0.45.
import * as THREE from "../vendor/three.module.js";

const MAX_SKIDS = 1100;
const MAX_SMOKE = 340;
const MAX_SPARKS = 160;
const MAX_RAIN = 360;

const WHEEL_RADIUS = 0.33;
const SKID_HALF_W = 0.155;

const RAIN_SPAN_X = 30;
const RAIN_SPAN_Y = 16;
const RAIN_SPAN_Z = 30;

/** Deterministic fast LCG for procedural CanvasTextures. */
function makeRng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** 128×128 feathered tire-tread contact-patch skidmark texture (soft lateral
 *  falloff + twin slick shoulder stress ribs + longitudinal rubber striation). */
function featheredSkidTexture() {
  const N = 128;
  const c = document.createElement("canvas");
  c.width = c.height = N;
  const g = c.getContext("2d");
  const img = g.createImageData(N, N);
  const rnd = makeRng(0x5a7f1337);

  // Subtle longitudinal striation profile across tread width
  const ribNoise = new Float32Array(N);
  for (let x = 0; x < N; x++) ribNoise[x] = 0.84 + rnd() * 0.28;

  for (let y = 0; y < N; y++) {
    const vWave = 0.92 + 0.08 * Math.sin((y / N) * Math.PI * 8);
    for (let x = 0; x < N; x++) {
      const u = x / (N - 1);
      // Soft feathered contact-patch edges (zero harsh geometric ribbon edges)
      const edgeL = Math.min(1, Math.max(0, u / 0.17));
      const edgeR = Math.min(1, Math.max(0, (1 - u) / 0.17));
      const feather = edgeL * edgeL * (3 - 2 * edgeL) * edgeR * edgeR * (3 - 2 * edgeR);

      // Twin shoulder stress bands + mild center-rib variation
      const shL = Math.exp(-Math.pow((u - 0.24) / 0.11, 2));
      const shR = Math.exp(-Math.pow((u - 0.76) / 0.11, 2));
      const center = Math.exp(-Math.pow((u - 0.5) / 0.22, 2));
      const striation = ribNoise[x] * (0.72 + 0.28 * (shL + shR) + 0.10 * center);
      const grain = 0.86 + rnd() * 0.22;

      const alpha = Math.min(1, feather * striation * vWave * grain);
      const shade = Math.min(255, Math.round(235 + (grain - 0.95) * 40));
      const o = (y * N + x) * 4;
      img.data[o] = shade;
      img.data[o + 1] = shade;
      img.data[o + 2] = shade;
      img.data[o + 3] = Math.round(alpha * 255);
    }
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = THREE.ClampToEdgeWrapping;
  t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

/** 128×128 multi-octave volumetric-look cloud puff texture with sculpted
 *  billowy value noise + subtle top-lit shading gradient. */
function volumetricPuffTexture() {
  const N = 128;
  const c = document.createElement("canvas");
  c.width = c.height = N;
  const g = c.getContext("2d");
  const img = g.createImageData(N, N);
  const rnd = makeRng(0x9e3779b9);

  // Tileable value noise grid for 4-octave FBM
  const G = 16;
  const grid = new Float32Array(G * G);
  for (let i = 0; i < G * G; i++) grid[i] = rnd();
  const sampleGrid = (x, y) => {
    const xi = Math.floor(x);
    const yi = Math.floor(y);
    const xf = x - xi;
    const yf = y - yi;
    const ux = xf * xf * (3 - 2 * xf);
    const uy = yf * yf * (3 - 2 * yf);
    const x0 = ((xi % G) + G) % G;
    const x1 = (x0 + 1) % G;
    const y0 = ((yi % G) + G) % G;
    const y1 = (y0 + 1) % G;
    const a = grid[y0 * G + x0];
    const b = grid[y0 * G + x1];
    const cc = grid[y1 * G + x0];
    const d = grid[y1 * G + x1];
    return (a + (b - a) * ux) * (1 - uy) + (cc + (d - cc) * ux) * uy;
  };
  const fbm = (x, y) => {
    let v = 0, amp = 0.5, freq = 2.6;
    for (let o = 0; o < 4; o++) {
      v += amp * sampleGrid(x * freq + o * 3.7, y * freq - o * 2.1);
      freq *= 2.04;
      amp *= 0.5;
    }
    return v;
  };

  for (let y = 0; y < N; y++) {
    const ny = (y / (N - 1)) * 2 - 1;
    for (let x = 0; x < N; x++) {
      const nx = (x / (N - 1)) * 2 - 1;
      const r = Math.hypot(nx, ny);
      const radial = Math.max(0, 1 - r);
      const env = radial * radial * (3 - 2 * radial);
      const n = fbm(x / N, y / N);
      const billow = Math.min(1, Math.max(0, (n - 0.22) * 1.45));
      const alpha = Math.min(1, env * (0.48 + 0.68 * billow));

      // Subtle sculpted top-left volumetric shading
      const lit = Math.min(1, Math.max(0.62, 0.80 - nx * 0.14 - ny * 0.24 + (n - 0.5) * 0.26));
      const lum = Math.round(lit * 255);
      const o = (y * N + x) * 4;
      img.data[o] = lum;
      img.data[o + 1] = lum;
      img.data[o + 2] = Math.min(255, Math.round(lum * 1.02));
      img.data[o + 3] = Math.round(alpha * 255);
    }
  }
  g.putImageData(img, 0, 0);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// Instanced volumetric puff shader (true per-instance RGB + Alpha fading,
// smooth camera-proximity near-fade so spray/smoke never blinds chase cam)
const PUFF_VERT = /* glsl */ `
attribute vec4 aInstColor;
varying vec2 vUv;
varying vec4 vColor;
varying float vCamDist;
void main() {
  vUv = uv;
  vColor = aInstColor;
  vec4 mvPosition = vec4(position, 1.0);
  #ifdef USE_INSTANCING
    mvPosition = instanceMatrix * mvPosition;
  #endif
  mvPosition = modelViewMatrix * mvPosition;
  vCamDist = -mvPosition.z;
  gl_Position = projectionMatrix * mvPosition;
}`;

const PUFF_FRAG = /* glsl */ `
uniform sampler2D uMap;
varying vec2 vUv;
varying vec4 vColor;
varying float vCamDist;
void main() {
  float nearFade = smoothstep(1.6, 5.4, vCamDist);
  if (nearFade < 0.004) discard;
  vec4 tex = texture2D(uMap, vUv);
  float a = tex.a * vColor.a * nearFade;
  if (a < 0.004) discard;
  gl_FragColor = vec4(tex.rgb * vColor.rgb, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

// Instanced HDR additive titanium spark shader (supports radiance > 3.0 for bloom trails)
const SPARK_VERT = /* glsl */ `
attribute vec4 aInstColor;
varying float vAlong;
varying vec4 vColor;
void main() {
  vAlong = clamp(position.z / 0.17, -1.0, 1.0);
  vColor = aInstColor;
  vec4 mvPosition = vec4(position, 1.0);
  #ifdef USE_INSTANCING
    mvPosition = instanceMatrix * mvPosition;
  #endif
  mvPosition = modelViewMatrix * mvPosition;
  gl_Position = projectionMatrix * mvPosition;
}`;

const SPARK_FRAG = /* glsl */ `
varying float vAlong;
varying vec4 vColor;
void main() {
  // White-hot leading head tapering smoothly along the streak tail
  float core = smoothstep(0.0, 0.55, 1.0 - abs(vAlong));
  vec3 col = vColor.rgb * (0.45 + 0.90 * core) * vColor.a;
  gl_FragColor = vec4(col, vColor.a * core);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

// Instanced atmospheric 3D rain streak shader (camera near-fade prevents near-plane flashes)
const RAIN_VERT = /* glsl */ `
varying vec2 vUv;
varying float vCamDist;
void main() {
  vUv = uv;
  vec4 mvPosition = vec4(position, 1.0);
  #ifdef USE_INSTANCING
    mvPosition = instanceMatrix * mvPosition;
  #endif
  mvPosition = modelViewMatrix * mvPosition;
  vCamDist = -mvPosition.z;
  gl_Position = projectionMatrix * mvPosition;
}`;

const RAIN_FRAG = /* glsl */ `
uniform float uOpacity;
varying vec2 vUv;
varying float vCamDist;
void main() {
  float nearFade = smoothstep(0.9, 2.8, vCamDist);
  float across = 1.0 - abs(vUv.x * 2.0 - 1.0);
  float along = sin(vUv.y * 3.14159265);
  float alpha = across * across * along * uOpacity * nearFade;
  if (alpha < 0.004) discard;
  gl_FragColor = vec4(0.76, 0.86, 0.98, alpha);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

// Dual-layer HDR exhaust backfire plasma jet shader (blooms explosively in PostProcessPipeline)
const FLAME_VERT = /* glsl */ `
uniform float uLength;
varying float vNozzle;
void main() {
  // Nozzle sits at local z = 0; flame tip extends to z = -uLength
  vNozzle = clamp(1.0 + position.z / max(uLength, 0.01), 0.0, 1.0);
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;

const FLAME_FRAG = /* glsl */ `
uniform vec3 uColor;
uniform float uIntensity;
varying float vNozzle;
void main() {
  float tipFade = smoothstep(0.0, 0.72, vNozzle);
  float baseHot = smoothstep(0.32, 1.0, vNozzle);
  float a = tipFade * uIntensity;
  if (a < 0.01) discard;
  vec3 col = uColor * (0.65 + 0.95 * baseHot) * uIntensity;
  gl_FragColor = vec4(col, a);
  #include <tonemapping_fragment>
  #include <colorspace_fragment>
}`;

export class RaceEffects {
  constructor(scene) {
    this.scene = scene;
    this.backfireGlow = 0;

    // --------------------------------------------------------- 1. Feathered skidmark ribbons
    this._skidPos = new Float32Array(MAX_SKIDS * 6 * 3);
    this._skidUv = new Float32Array(MAX_SKIDS * 6 * 2);
    this._skidCol = new Float32Array(MAX_SKIDS * 6 * 4);
    this._skidHead = 0;

    this._skidGeo = new THREE.BufferGeometry();
    this._skidGeo.setAttribute("position", new THREE.BufferAttribute(this._skidPos, 3));
    this._skidGeo.setAttribute("uv", new THREE.BufferAttribute(this._skidUv, 2));
    this._skidGeo.setAttribute("color", new THREE.BufferAttribute(this._skidCol, 4));
    this._skidGeo.attributes.position.setUsage(THREE.DynamicDrawUsage);
    this._skidGeo.attributes.uv.setUsage(THREE.DynamicDrawUsage);
    this._skidGeo.attributes.color.setUsage(THREE.DynamicDrawUsage);

    this._skidMesh = new THREE.Mesh(
      this._skidGeo,
      new THREE.MeshBasicMaterial({
        map: featheredSkidTexture(),
        vertexColors: true,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        polygonOffset: true,
        polygonOffsetFactor: -2,
        polygonOffsetUnits: -2,
      }),
    );
    this._skidMesh.frustumCulled = false;
    this._skidMesh.renderOrder = 5;
    scene.add(this._skidMesh);

    this._prevPatch = [
      new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(),
    ];
    this._hasPrevPatch = [false, false, false, false];
    this._prevRight = [
      new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(),
    ];
    this._skidV = [0, 0, 0, 0];

    // --------------------------------------------------------- 2. Volumetric puff particle pool
    const puffGeo = new THREE.PlaneGeometry(1, 1);
    this._smokeColArr = new Float32Array(MAX_SMOKE * 4);
    this._smokeColAttr = new THREE.InstancedBufferAttribute(this._smokeColArr, 4);
    this._smokeColAttr.setUsage(THREE.DynamicDrawUsage);
    puffGeo.setAttribute("aInstColor", this._smokeColAttr);

    const puffMat = new THREE.ShaderMaterial({
      uniforms: { uMap: { value: volumetricPuffTexture() } },
      vertexShader: PUFF_VERT,
      fragmentShader: PUFF_FRAG,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this._smokeMesh = new THREE.InstancedMesh(puffGeo, puffMat, MAX_SMOKE);
    this._smokeMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this._smokeMesh.frustumCulled = false;
    this._smokeMesh.renderOrder = 6;

    this._smokePool = Array.from({ length: MAX_SMOKE }, () => ({
      alive: false,
      age: 0,
      life: 1,
      pos: new THREE.Vector3(),
      vel: new THREE.Vector3(),
      size0: 0.3,
      size1: 1.5,
      rot: 0,
      spin: 0,
      drag: 2.0,
      vyAccel: 0.3,
      peakAlpha: 0.55,
      r: 0.85,
      g: 0.88,
      b: 0.92,
    }));
    this._smokeCursor = 0;
    scene.add(this._smokeMesh);

    // --------------------------------------------------------- 3. HDR Titanium spark pool
    const sparkGeo = new THREE.BoxGeometry(0.026, 0.026, 0.34);
    this._sparkColArr = new Float32Array(MAX_SPARKS * 4);
    this._sparkColAttr = new THREE.InstancedBufferAttribute(this._sparkColArr, 4);
    this._sparkColAttr.setUsage(THREE.DynamicDrawUsage);
    sparkGeo.setAttribute("aInstColor", this._sparkColAttr);

    const sparkMat = new THREE.ShaderMaterial({
      vertexShader: SPARK_VERT,
      fragmentShader: SPARK_FRAG,
      transparent: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    this._sparkMesh = new THREE.InstancedMesh(sparkGeo, sparkMat, MAX_SPARKS);
    this._sparkMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this._sparkMesh.frustumCulled = false;
    this._sparkMesh.renderOrder = 8;

    this._sparkPool = Array.from({ length: MAX_SPARKS }, () => ({
      alive: false,
      age: 0,
      life: 0.4,
      pos: new THREE.Vector3(),
      vel: new THREE.Vector3(),
      groundY: -999,
      r0: 4.5, g0: 2.6, b0: 0.8,
      r1: 2.8, g1: 0.5, b1: 0.05,
    }));
    this._sparkCursor = 0;
    scene.add(this._sparkMesh);

    // --------------------------------------------------------- 4. HDR Exhaust backfire flame jet
    this._flameGroup = new THREE.Group();
    this._flameGroup.visible = false;
    this._flameGroup.renderOrder = 9;

    const outerConeGeo = new THREE.ConeGeometry(0.095, 0.68, 10, 1, true);
    outerConeGeo.rotateX(-Math.PI / 2); // base at z=0, tip pointing toward -Z
    outerConeGeo.translate(0, 0, -0.34);
    this._flameOuterMat = new THREE.ShaderMaterial({
      uniforms: {
        uColor: { value: new THREE.Vector3(6.8, 2.35, 0.32) },
        uIntensity: { value: 0.0 },
        uLength: { value: 0.68 },
      },
      vertexShader: FLAME_VERT,
      fragmentShader: FLAME_FRAG,
      transparent: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this._flameOuter = new THREE.Mesh(outerConeGeo, this._flameOuterMat);

    const innerConeGeo = new THREE.ConeGeometry(0.052, 0.42, 10, 1, true);
    innerConeGeo.rotateX(-Math.PI / 2);
    innerConeGeo.translate(0, 0, -0.21);
    this._flameInnerMat = new THREE.ShaderMaterial({
      uniforms: {
        uColor: { value: new THREE.Vector3(2.8, 5.4, 9.8) },
        uIntensity: { value: 0.0 },
        uLength: { value: 0.42 },
      },
      vertexShader: FLAME_VERT,
      fragmentShader: FLAME_FRAG,
      transparent: true,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this._flameInner = new THREE.Mesh(innerConeGeo, this._flameInnerMat);
    this._flameGroup.add(this._flameOuter, this._flameInner);
    scene.add(this._flameGroup);

    // Brief warm backfire illumination flash on rear wing, diffuser & wet tarmac
    this._flameLight = new THREE.PointLight(0xff8428, 0, 9.5, 1.5);
    this._flameLight.castShadow = false;
    scene.add(this._flameLight);

    // --------------------------------------------------------- 5. 3D Atmospheric Rain Streaks
    const rainGeo = new THREE.PlaneGeometry(0.016, 0.72);
    this._rainMat = new THREE.ShaderMaterial({
      uniforms: { uOpacity: { value: 0.0 } },
      vertexShader: RAIN_VERT,
      fragmentShader: RAIN_FRAG,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this._rainMesh = new THREE.InstancedMesh(rainGeo, this._rainMat, MAX_RAIN);
    this._rainMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this._rainMesh.frustumCulled = false;
    this._rainMesh.visible = false;
    this._rainMesh.renderOrder = 9;

    const rainRng = makeRng(0x2026cafe);
    this._rainDrops = Array.from({ length: MAX_RAIN }, () => ({
      x: (rainRng() - 0.5) * RAIN_SPAN_X,
      y: (rainRng() - 0.5) * RAIN_SPAN_Y,
      z: (rainRng() - 0.5) * RAIN_SPAN_Z,
      speed: 19.5 + rainRng() * 8.5,
      len: 0.75 + rainRng() * 0.55,
      windX: 1.8 + (rainRng() - 0.5) * 0.8,
      windZ: 0.9 + (rainRng() - 0.5) * 0.8,
    }));
    scene.add(this._rainMesh);

    // --------------------------------------------------------- Scratch objects (ZERO GC in update)
    this._up = new THREE.Vector3();
    this._fwd = new THREE.Vector3();
    this._right = new THREE.Vector3();
    this._patch = new THREE.Vector3();
    this._segDir = new THREE.Vector3();
    this._segRight = new THREE.Vector3();
    this._spawnPos = new THREE.Vector3();
    this._spawnVel = new THREE.Vector3();
    this._exhaustPos = new THREE.Vector3();
    this._camFwd = new THREE.Vector3();
    this._rainBoxCenter = new THREE.Vector3();
    this._rainDir = new THREE.Vector3();
    this._rainSide = new THREE.Vector3();
    this._rainLook = new THREE.Vector3();
    this._rainPos = new THREE.Vector3();
    this._zAxis = new THREE.Vector3(0, 0, 1);
    this._m4 = new THREE.Matrix4();
    this._q = new THREE.Quaternion();
    this._qSpin = new THREE.Quaternion();
    this._s = new THREE.Vector3();
    this._zeroM = new THREE.Matrix4().makeScale(0, 0, 0);
    this._cScratch = new THREE.Color();

    this._emitAcc = [0, 0, 0, 0];
    this._diffuserAcc = 0;
    this._brakeHeat = [0, 0, 0, 0];
    this._prevComp = [0, 0, 0, 0];
    this._kerbSparkCd = 0;
    this._prevGear = -1;
    this._prevThrottle = 0;
    this._popCooldown = 0;
    this._rainTime = 0;

    this.reset();
  }

  reset() {
    this._skidPos.fill(0);
    this._skidUv.fill(0);
    this._skidCol.fill(0);
    this._skidHead = 0;
    this._skidGeo.attributes.position.needsUpdate = true;
    this._skidGeo.attributes.uv.needsUpdate = true;
    this._skidGeo.attributes.color.needsUpdate = true;

    for (let i = 0; i < 4; i++) {
      this._hasPrevPatch[i] = false;
      this._skidV[i] = 0;
      this._emitAcc[i] = 0;
      this._brakeHeat[i] = 0;
      this._prevComp[i] = 0;
    }
    this._diffuserAcc = 0;
    this._kerbSparkCd = 0;
    this._prevGear = -1;
    this._prevThrottle = 0;
    this._popCooldown = 0;
    this.backfireGlow = 0;
    this._flameGroup.visible = false;
    this._flameLight.intensity = 0;

    this._smokeColArr.fill(0);
    for (let i = 0; i < MAX_SMOKE; i++) {
      this._smokePool[i].alive = false;
      this._smokeMesh.setMatrixAt(i, this._zeroM);
    }
    this._smokeMesh.instanceMatrix.needsUpdate = true;
    this._smokeColAttr.needsUpdate = true;

    this._sparkColArr.fill(0);
    for (let i = 0; i < MAX_SPARKS; i++) {
      this._sparkPool[i].alive = false;
      this._sparkMesh.setMatrixAt(i, this._zeroM);
    }
    this._sparkMesh.instanceMatrix.needsUpdate = true;
    this._sparkColAttr.needsUpdate = true;
  }

  _writeSkidQuad(p0, r0, p1, r1, v0, v1, r, g, b, alpha) {
    const seg = this._skidHead;
    this._skidHead = (seg + 1) % MAX_SKIDS;

    const po = seg * 18;
    const uo = seg * 12;
    const co = seg * 24;

    const ax = p0.x - r0.x, ay = p0.y - r0.y, az = p0.z - r0.z;
    const bx = p0.x + r0.x, by = p0.y + r0.y, bz = p0.z + r0.z;
    const cx = p1.x - r1.x, cy = p1.y - r1.y, cz = p1.z - r1.z;
    const dx = p1.x + r1.x, dy = p1.y + r1.y, dz = p1.z + r1.z;

    const P = this._skidPos;
    P[po] = ax; P[po + 1] = ay; P[po + 2] = az;
    P[po + 3] = bx; P[po + 4] = by; P[po + 5] = bz;
    P[po + 6] = cx; P[po + 7] = cy; P[po + 8] = cz;
    P[po + 9] = bx; P[po + 10] = by; P[po + 11] = bz;
    P[po + 12] = dx; P[po + 13] = dy; P[po + 14] = dz;
    P[po + 15] = cx; P[po + 16] = cy; P[po + 17] = cz;

    const UV = this._skidUv;
    UV[uo] = 0; UV[uo + 1] = v0;
    UV[uo + 2] = 1; UV[uo + 3] = v0;
    UV[uo + 4] = 0; UV[uo + 5] = v1;
    UV[uo + 6] = 1; UV[uo + 7] = v0;
    UV[uo + 8] = 1; UV[uo + 9] = v1;
    UV[uo + 10] = 0; UV[uo + 11] = v1;

    const C = this._skidCol;
    for (let k = 0; k < 6; k++) {
      C[co + k * 4] = r;
      C[co + k * 4 + 1] = g;
      C[co + k * 4 + 2] = b;
      C[co + k * 4 + 3] = alpha;
    }
    this._skidGeo.attributes.position.needsUpdate = true;
    this._skidGeo.attributes.uv.needsUpdate = true;
    this._skidGeo.attributes.color.needsUpdate = true;
  }

  _spawnSmoke(pos, vel, size0, size1, life, hexColor, peakAlpha = 0.52, drag = 2.0, vyAccel = 0.35) {
    const p = this._smokePool[this._smokeCursor];
    this._smokeCursor = (this._smokeCursor + 1) % MAX_SMOKE;
    p.alive = true;
    p.age = 0;
    p.life = life;
    p.pos.copy(pos);
    p.vel.copy(vel);
    p.size0 = size0;
    p.size1 = size1;
    p.rot = Math.random() * Math.PI * 2;
    p.spin = (Math.random() - 0.5) * 3.4;
    p.drag = drag;
    p.vyAccel = vyAccel;
    p.peakAlpha = peakAlpha;
    this._cScratch.setHex(hexColor);
    p.r = this._cScratch.r;
    p.g = this._cScratch.g;
    p.b = this._cScratch.b;
  }

  _spawnSpark(pos, vel, life, groundY, r0, g0, b0, r1, g1, b1) {
    const p = this._sparkPool[this._sparkCursor];
    this._sparkCursor = (this._sparkCursor + 1) % MAX_SPARKS;
    p.alive = true;
    p.age = 0;
    p.life = life;
    p.pos.copy(pos);
    p.vel.copy(vel);
    p.groundY = groundY;
    p.r0 = r0; p.g0 = g0; p.b0 = b0;
    p.r1 = r1; p.g1 = g1; p.b1 = b1;
  }

  /** Burst fiery HDR titanium sparks at a world-space crash site. */
  emitImpactSparks(worldPos, severity = 1.0) {
    const sev = Math.max(0.25, Math.min(2.0, severity));
    const count = Math.min(54, Math.max(18, Math.round(sev * 34)));
    const groundY = worldPos.y - 0.25;
    for (let k = 0; k < count; k++) {
      const ang = Math.random() * Math.PI * 2;
      const spd = 4.5 + Math.random() * (11.0 * sev + 5.5);
      this._spawnPos.set(
        worldPos.x + (Math.random() - 0.5) * 0.4,
        worldPos.y + 0.12 + Math.random() * 0.35,
        worldPos.z + (Math.random() - 0.5) * 0.4,
      );
      this._spawnVel.set(
        Math.cos(ang) * spd,
        2.2 + Math.random() * (6.8 * sev),
        Math.sin(ang) * spd,
      );
      const hot = Math.random() > 0.32;
      this._spawnSpark(
        this._spawnPos,
        this._spawnVel,
        0.24 + Math.random() * 0.44,
        groundY,
        hot ? 5.2 : 4.2, hot ? 3.5 : 1.9, hot ? 1.35 : 0.35,
        3.0, 0.55, 0.06,
      );
    }
  }

  /** Trigger a brief incandescent blue-orange exhaust backfire pop & ember burst. */
  emitExhaustPop(carGroup, intensity = 1.0) {
    if (!carGroup) return;
    const inten = Math.max(0.2, Math.min(1.25, intensity));
    this.backfireGlow = Math.max(this.backfireGlow, Math.min(1.0, inten));

    this._fwd.set(0, 0, 1).applyQuaternion(carGroup.quaternion).normalize();
    this._up.set(0, 1, 0).applyQuaternion(carGroup.quaternion).normalize();
    this._right.set(1, 0, 0).applyQuaternion(carGroup.quaternion).normalize();

    this._exhaustPos.set(0, 0.22, -2.18)
      .applyQuaternion(carGroup.quaternion)
      .add(carGroup.position);

    const groundY = carGroup.position.y - 0.35;
    const count = Math.max(7, Math.min(16, Math.round(11 * inten)));
    for (let k = 0; k < count; k++) {
      const backSpd = 9.0 + Math.random() * (11.0 * inten);
      const spreadX = (Math.random() - 0.5) * 2.6;
      const spreadY = (Math.random() - 0.35) * 2.2;
      this._spawnPos.copy(this._exhaustPos)
        .addScaledVector(this._fwd, -Math.random() * 0.18)
        .addScaledVector(this._right, (Math.random() - 0.5) * 0.07)
        .addScaledVector(this._up, (Math.random() - 0.5) * 0.07);
      this._spawnVel.set(0, 0, 0)
        .addScaledVector(this._fwd, -backSpd)
        .addScaledVector(this._right, spreadX)
        .addScaledVector(this._up, spreadY);

      const blueCore = k < 3;
      this._spawnSpark(
        this._spawnPos,
        this._spawnVel,
        0.09 + Math.random() * 0.16,
        groundY,
        blueCore ? 2.8 : 5.6, blueCore ? 4.8 : 2.5, blueCore ? 8.8 : 0.45,
        2.6, 0.52, 0.05,
      );
    }
  }

  /** Emit fiery titanium floor-plank / kerb bottoming sparks underneath the car. */
  _emitKerbSparks(carGroup, sideSign, speed, intensity) {
    const count = Math.max(2, Math.min(6, Math.round(2 + intensity * 4)));
    const groundY = carGroup.position.y - 0.34;
    for (let k = 0; k < count; k++) {
      this._spawnPos.set(
        sideSign * (0.24 + Math.random() * 0.26),
        -0.14,
        -0.85 - Math.random() * 0.65,
      ).applyQuaternion(carGroup.quaternion).add(carGroup.position);

      const backSpd = 6.0 + speed * 0.22 + Math.random() * 5.0;
      this._spawnVel.set(0, 0, 0)
        .addScaledVector(this._fwd, -backSpd)
        .addScaledVector(this._right, sideSign * (1.2 + Math.random() * 2.4) + (Math.random() - 0.5) * 1.4)
        .addScaledVector(this._up, 1.3 + Math.random() * 3.1);

      this._spawnSpark(
        this._spawnPos,
        this._spawnVel,
        0.18 + Math.random() * 0.28,
        groundY,
        5.5, 3.6, 1.4,
        3.2, 0.65, 0.08,
      );
    }
  }

  /**
   * Per-frame zero-allocation VFX update: skidmarks, volumetric smoke/gravel/spray,
   * brake-disc thermal glow, kerb/bottoming titanium sparks, exhaust backfire pops,
   * and 3D atmospheric rain streaks.
   */
  update({
    state,
    carGroup,
    wheelMeshes,
    camera,
    dtSec,
    brake = 0,
    throttle = 0,
    gear = 1,
    offTrack = false,
    wetness = 0,
    sim = null,
    frozen = false,
  }) {
    const dt = Math.min(Math.max(dtSec || 0, 0), 0.05);
    this._up.set(0, 1, 0).applyQuaternion(carGroup.quaternion).normalize();
    this._fwd.set(0, 0, 1).applyQuaternion(carGroup.quaternion).normalize();
    this._right.set(1, 0, 0).applyQuaternion(carGroup.quaternion).normalize();

    const speed = state.speed;
    const liveGear = (typeof gear === "number" && gear >= 0) ? gear : (sim?.gear?.() ?? 1);
    const liveRpm = sim?.rpm?.() ?? (1100 + speed * 115);

    // ---- Automatic downshift / high-RPM upshift / overrun exhaust backfire detection
    if (this._popCooldown > 0) this._popCooldown -= dt;
    if (!frozen && this._prevGear > 0 && liveGear > 0) {
      if (liveGear < this._prevGear && speed > 10 && this._popCooldown <= 0) {
        // Aggressive downshift rev-match bang
        this.emitExhaustPop(carGroup, 0.92 + Math.random() * 0.28);
        this._popCooldown = 0.14;
      } else if (liveGear > this._prevGear && liveRpm > 5100 && speed > 16 && this._popCooldown <= 0) {
        // High-RPM flat-shift ignition cut pop
        this.emitExhaustPop(carGroup, 0.78 + Math.random() * 0.22);
        this._popCooldown = 0.16;
      } else if (
        this._prevThrottle - throttle > 0.42 &&
        liveRpm > 4900 &&
        speed > 18 &&
        this._popCooldown <= 0
      ) {
        // Sharp throttle lift overrun crackle
        this.emitExhaustPop(carGroup, 0.70 + Math.random() * 0.26);
        this._popCooldown = 0.22;
      }
    }
    this._prevGear = liveGear;
    this._prevThrottle = throttle;

    // ---- Update HDR 3D exhaust backfire flame jet + rear flash PointLight
    this.backfireGlow *= Math.exp(-17.5 * dt);
    if (this.backfireGlow < 0.012) {
      this.backfireGlow = 0;
      this._flameGroup.visible = false;
      this._flameLight.intensity = 0;
    } else {
      this._exhaustPos.set(0, 0.22, -2.18)
        .applyQuaternion(carGroup.quaternion)
        .add(carGroup.position);
      this._flameGroup.visible = true;
      this._flameGroup.position.copy(this._exhaustPos);
      this._flameGroup.quaternion.copy(carGroup.quaternion);

      const flicker = 0.84 + 0.32 * Math.random();
      const gScale = this.backfireGlow * flicker;
      const rad = 0.72 + 0.55 * gScale;
      const len = 0.65 + 1.05 * gScale;
      this._flameGroup.scale.set(rad, rad, len);
      this._flameOuterMat.uniforms.uIntensity.value = Math.min(1.4, gScale * 1.25);
      this._flameInnerMat.uniforms.uIntensity.value = Math.min(1.5, gScale * 1.35);

      this._flameLight.position.copy(this._exhaustPos).addScaledVector(this._fwd, -0.55);
      this._flameLight.intensity = 15.0 * this.backfireGlow * flicker;
    }

    // ---- Per-wheel brake thermal glow, feathered skidmarks, kerb sparks & volumetric plumes
    if (this._kerbSparkCd > 0) this._kerbSparkCd -= dt;
    let maxKerbImpulse = 0;
    let kerbSide = 0;

    for (let i = 0; i < 4; i++) {
      const w = state.wheels[i];

      // Brake disc thermal model (front takes 65% of thermal load)
      const frontBias = i < 2 ? 1.18 : 0.72;
      const heatGain = brake * Math.min(1, speed / 34) * frontBias * 1.7;
      const cool = (0.34 + speed * 0.018) * this._brakeHeat[i];
      this._brakeHeat[i] = Math.max(0, Math.min(1, this._brakeHeat[i] + (heatGain - cool) * dt));
      wheelMeshes[i]?.userData?.setBrakeGlow?.(this._brakeHeat[i]);

      if (frozen) {
        this._hasPrevPatch[i] = false;
        this._prevComp[i] = w.compression;
        continue;
      }

      const loaded = w.compression > 0.005;
      const slipLat = Math.abs(w.slipAngle);
      const slipLon = Math.abs(w.slipRatio);
      const slipMag = Math.hypot(slipLat * 1.15, slipLon);
      const temp = sim?.tireTemp?.(i) ?? 78;

      // Detect sudden suspension compression spikes (kerb strikes / high-speed floor bottoming)
      if (dt > 0.0005 && loaded && !offTrack && speed > 26) {
        const dComp = (w.compression - this._prevComp[i]) / dt;
        const kerbHit = dComp > 0.72 && w.compression > 0.035;
        const bottomOut = speed > 44 && w.compression > 0.125 && dComp > 0.22;
        if (kerbHit || bottomOut) {
          const imp = Math.min(1.2, Math.max(dComp * 0.65, (w.compression - 0.09) * 8.0));
          if (imp > maxKerbImpulse) {
            maxKerbImpulse = imp;
            kerbSide = (i % 2 === 0) ? -1 : 1;
          }
        }
      }
      this._prevComp[i] = w.compression;

      // Ground contact patch just above asphalt
      this._patch.set(w.pos[0], w.pos[1], w.pos[2]).addScaledVector(this._up, -(WHEEL_RADIUS - 0.016));

      // ---- Feathered skidmark / wet hydroplane wake / off-track rut
      const wetWake = wetness > 0.40 && !offTrack && loaded && speed > 14;
      const slipSkid = loaded && speed > 3.2 ? Math.max(0, (slipMag - 0.13) / 0.34) : 0;
      const skidIntensity = wetWake
        ? Math.max(slipSkid * 0.65, Math.min(0.42, (speed - 12) / 65) * wetness)
        : slipSkid;

      if (skidIntensity > 0.025) {
        if (this._hasPrevPatch[i]) {
          this._segDir.copy(this._patch).sub(this._prevPatch[i]);
          const segLen = this._segDir.length();
          if (segLen > 0.22 && segLen < 4.5) {
            this._segDir.divideScalar(segLen);
            this._segRight.crossVectors(this._up, this._segDir).normalize().multiplyScalar(SKID_HALF_W);
            if (this._segRight.dot(this._prevRight[i]) < 0) this._segRight.negate();

            const v0 = this._skidV[i];
            const v1 = v0 + segLen * 1.65;
            this._skidV[i] = v1;

            let r, g, b, alpha;
            if (offTrack) {
              r = 0.095; g = 0.068; b = 0.036;
              alpha = Math.min(0.68, skidIntensity * 0.62);
            } else if (wetness > 0.40 && slipSkid < 0.35) {
              // Subtle bright water-clearing wake on glossy wet tarmac
              r = 0.42; g = 0.49; b = 0.58;
              alpha = Math.min(0.30, skidIntensity * 0.48);
            } else {
              // Hot vulcanized rubber on dry/damp asphalt
              const hotDark = Math.min(1, Math.max(0, (temp - 55) / 55));
              r = 0.012 * (1 - 0.35 * hotDark);
              g = 0.013 * (1 - 0.35 * hotDark);
              b = 0.016 * (1 - 0.35 * hotDark);
              alpha = Math.min(0.84, skidIntensity * (0.76 + 0.14 * hotDark) * (1 - wetness * 0.42));
            }
            this._writeSkidQuad(
              this._prevPatch[i], this._prevRight[i],
              this._patch, this._segRight,
              v0, v1, r, g, b, alpha,
            );
            this._prevPatch[i].copy(this._patch);
            this._prevRight[i].copy(this._segRight);
          } else if (segLen >= 4.5) {
            this._prevPatch[i].copy(this._patch);
            this._prevRight[i].copy(this._right).multiplyScalar(SKID_HALF_W);
          }
        } else {
          this._prevPatch[i].copy(this._patch);
          this._prevRight[i].copy(this._right).multiplyScalar(SKID_HALF_W);
          this._hasPrevPatch[i] = true;
        }
      } else {
        this._hasPrevPatch[i] = false;
      }

      // ---- Per-wheel volumetric particle emission (smoke, gravel roost, wet spray)
      this._emitAcc[i] += dt;
      if (loaded && !offTrack && wetness < 0.42 && slipMag > 0.22 && speed > 4.5 && this._emitAcc[i] > 0.030) {
        // 1. Billowing hot tire smoke
        this._emitAcc[i] = 0;
        const hot = Math.min(1, Math.max(0, (temp - 50) / 55));
        const slipBoost = Math.min(1, (slipMag - 0.22) / 0.45);
        this._spawnVel.set(
          (Math.random() - 0.5) * 1.4 - this._fwd.x * (1.5 + speed * 0.04),
          0.65 + Math.random() * 0.75 + hot * 0.35,
          (Math.random() - 0.5) * 1.4 - this._fwd.z * (1.5 + speed * 0.04),
        );
        this._spawnSmoke(
          this._patch,
          this._spawnVel,
          0.26,
          1.45 + hot * 0.75 + slipBoost * 0.35,
          0.62 + Math.random() * 0.42,
          0xe2e8f0,
          0.34 + 0.24 * slipBoost + 0.10 * hot,
          1.85,
          0.55,
        );
      } else if (loaded && offTrack && speed > 5.0 && this._emitAcc[i] > 0.024) {
        // 2. Golden-lit off-track gravel / dirt roost
        this._emitAcc[i] = 0;
        const spdF = Math.min(1, speed / 35);
        this._spawnVel.set(
          (Math.random() - 0.5) * 2.2 - this._fwd.x * (2.6 + speed * 0.12),
          1.35 + Math.random() * (1.8 + spdF * 1.2),
          (Math.random() - 0.5) * 2.2 - this._fwd.z * (2.6 + speed * 0.12),
        );
        const warmDust = Math.random() > 0.35 ? 0xb59a6e : 0x8a7352;
        this._spawnSmoke(
          this._patch,
          this._spawnVel,
          0.22,
          1.25 + spdF * 0.65,
          0.50 + Math.random() * 0.32,
          warmDust,
          0.42 + 0.18 * spdF,
          1.6,
          -2.4, // ballistic dust fall
        );
      } else if (loaded && wetness > 0.32 && speed > 10.5 && this._emitAcc[i] > (i >= 2 ? 0.036 : 0.068)) {
        // 3. Tier-1 wet-weather slick rooster-tail spray (low-slung, airy, camera-safe)
        this._emitAcc[i] = 0;
        const sprayV = Math.min(1, (speed - 10) / 45);
        const rearMul = i >= 2 ? 1.12 : 0.72;
        const outSign = (i % 2 === 0) ? -1 : 1;
        this._spawnVel.set(
          this._right.x * (outSign * (0.45 + Math.random() * 0.65)) - this._fwd.x * (2.4 + speed * 0.09),
          (0.45 + sprayV * 0.95 + Math.random() * 0.40) * rearMul,
          this._right.z * (outSign * (0.45 + Math.random() * 0.65)) - this._fwd.z * (2.4 + speed * 0.09),
        );
        this._spawnSmoke(
          this._patch,
          this._spawnVel,
          0.16,
          (0.82 + sprayV * 0.56) * rearMul,
          0.32 + Math.random() * 0.20,
          0xcadceb,
          (0.11 + 0.11 * sprayV) * wetness,
          2.6,
          -1.35,
        );
      }
    }

    // Trigger kerb / floor-plank bottoming titanium spark burst
    if (!frozen && maxKerbImpulse > 0.15 && this._kerbSparkCd <= 0) {
      this._emitKerbSparks(carGroup, kerbSide, speed, maxKerbImpulse);
      this._kerbSparkCd = 0.048;
    }

    // ---- Tier-2 wet-weather rear diffuser vortex mist plume (stays low beneath chase sightline)
    if (!frozen && wetness > 0.35 && !offTrack && speed > 18) {
      this._diffuserAcc += dt;
      if (this._diffuserAcc > 0.052) {
        this._diffuserAcc = 0;
        const sprayV = Math.min(1, (speed - 18) / 45);
        this._spawnPos.set((Math.random() - 0.5) * 0.65, 0.24 + Math.random() * 0.18, -2.26)
          .applyQuaternion(carGroup.quaternion)
          .add(carGroup.position);
        this._spawnVel.set(
          (Math.random() - 0.5) * 1.15 - this._fwd.x * (2.8 + speed * 0.11),
          0.55 + sprayV * 1.05 + Math.random() * 0.42,
          (Math.random() - 0.5) * 1.15 - this._fwd.z * (2.8 + speed * 0.11),
        );
        this._spawnSmoke(
          this._spawnPos,
          this._spawnVel,
          0.26,
          1.15 + sprayV * 0.62,
          0.36 + Math.random() * 0.22,
          0xc0d6ea,
          (0.10 + 0.11 * sprayV) * wetness,
          2.5,
          -1.15,
        );
      }
    }

    // ---- Advance volumetric smoke / gravel / spray billboards (zero GC, true alpha fade)
    let smokeDirty = false;
    const SCOL = this._smokeColArr;
    for (let i = 0; i < MAX_SMOKE; i++) {
      const p = this._smokePool[i];
      if (!p.alive) continue;
      smokeDirty = true;
      p.age += dt;
      if (p.age >= p.life) {
        p.alive = false;
        this._smokeMesh.setMatrixAt(i, this._zeroM);
        SCOL[i * 4 + 3] = 0;
        continue;
      }
      const u = p.age / p.life;
      p.vel.y += p.vyAccel * dt;
      const damp = Math.max(0, 1 - p.drag * dt);
      p.vel.multiplyScalar(damp);
      p.pos.addScaledVector(p.vel, dt);
      p.rot += p.spin * dt;

      // Soft quadratic growth + smooth fade-in (0..0.14) and ease-out (0.14..1)
      const grow = Math.sqrt(u);
      const sz = p.size0 + (p.size1 - p.size0) * grow;
      this._qSpin.setFromAxisAngle(this._zAxis, p.rot);
      this._q.multiplyQuaternions(camera.quaternion, this._qSpin);
      this._m4.compose(p.pos, this._q, this._s.set(sz, sz, 1));
      this._smokeMesh.setMatrixAt(i, this._m4);

      const fadeIn = Math.min(1, u / 0.14);
      const fadeOut = (1 - u) * (1 - u);
      const o = i * 4;
      SCOL[o] = p.r;
      SCOL[o + 1] = p.g;
      SCOL[o + 2] = p.b;
      SCOL[o + 3] = p.peakAlpha * fadeIn * fadeOut;
    }
    if (smokeDirty) {
      this._smokeMesh.instanceMatrix.needsUpdate = true;
      this._smokeColAttr.needsUpdate = true;
    }

    // ---- Advance ballistic HDR titanium sparks (with ground bounce & thermal cooling)
    let sparkDirty = false;
    const KCOL = this._sparkColArr;
    for (let i = 0; i < MAX_SPARKS; i++) {
      const p = this._sparkPool[i];
      if (!p.alive) continue;
      sparkDirty = true;
      p.age += dt;
      if (p.age >= p.life) {
        p.alive = false;
        this._sparkMesh.setMatrixAt(i, this._zeroM);
        KCOL[i * 4 + 3] = 0;
        continue;
      }
      p.vel.y -= 15.5 * dt;
      p.pos.addScaledVector(p.vel, dt);
      // Elastic tarmac bounce with spark scatter friction
      if (p.pos.y < p.groundY && p.vel.y < 0) {
        p.pos.y = p.groundY;
        p.vel.y = -p.vel.y * 0.42;
        p.vel.x *= 0.78;
        p.vel.z *= 0.78;
      }
      const u = p.age / p.life;
      const vLen = p.vel.length();
      if (vLen > 0.01) {
        this._segDir.copy(p.vel).divideScalar(vLen);
        this._q.setFromUnitVectors(this._zAxis, this._segDir);
      } else {
        this._q.identity();
      }
      const thick = 1.0 - u * 0.65;
      const stretch = thick * (0.75 + Math.min(3.4, vLen * 0.11));
      this._m4.compose(p.pos, this._q, this._s.set(thick, thick, stretch));
      this._sparkMesh.setMatrixAt(i, this._m4);

      const o = i * 4;
      KCOL[o] = p.r0 + (p.r1 - p.r0) * u;
      KCOL[o + 1] = p.g0 + (p.g1 - p.g0) * u;
      KCOL[o + 2] = p.b0 + (p.b1 - p.b0) * u;
      KCOL[o + 3] = (1 - u) * (1 - 0.35 * u);
    }
    if (sparkDirty) {
      this._sparkMesh.instanceMatrix.needsUpdate = true;
      this._sparkColAttr.needsUpdate = true;
    }

    // ---- 3D Atmospheric Rain Streaks (wrapping seamlessly around camera when wet)
    const rainActive = wetness > 0.45;
    this._rainMesh.visible = rainActive;
    if (rainActive) {
      const rainVis = Math.min(1, Math.max(0, (wetness - 0.45) / 0.50));
      this._rainMat.uniforms.uOpacity.value = rainVis * 0.44;
      this._rainTime += dt;

      // Center rain box slightly ahead of camera so visible density is maximized
      camera.getWorldDirection(this._camFwd);
      this._rainBoxCenter.copy(camera.position).addScaledVector(this._camFwd, 6.5);

      const halfX = RAIN_SPAN_X * 0.5;
      const halfY = RAIN_SPAN_Y * 0.5;
      const halfZ = RAIN_SPAN_Z * 0.5;
      const cx = this._rainBoxCenter.x;
      const cy = this._rainBoxCenter.y;
      const cz = this._rainBoxCenter.z;

      for (let i = 0; i < MAX_RAIN; i++) {
        const d = this._rainDrops[i];
        // Seamless world-space modular wrapping around moving camera volume
        const wx = d.x + d.windX * this._rainTime;
        const wy = d.y - d.speed * this._rainTime;
        const wz = d.z + d.windZ * this._rainTime;

        let rx = ((wx - (cx - halfX)) % RAIN_SPAN_X + RAIN_SPAN_X) % RAIN_SPAN_X + (cx - halfX);
        let ry = ((wy - (cy - halfY)) % RAIN_SPAN_Y + RAIN_SPAN_Y) % RAIN_SPAN_Y + (cy - halfY);
        let rz = ((wz - (cz - halfZ)) % RAIN_SPAN_Z + RAIN_SPAN_Z) % RAIN_SPAN_Z + (cz - halfZ);

        // Streak direction vector (falling diagonally with slight wind shear)
        this._rainDir.set(d.windX, -d.speed, d.windZ).normalize();
        // Cylindrical billboard: face camera horizontally while preserving exact streak tilt
        this._rainLook.set(camera.position.x - rx, camera.position.y - ry, camera.position.z - rz);
        this._rainSide.crossVectors(this._rainDir, this._rainLook);
        if (this._rainSide.lengthSq() < 1e-6) this._rainSide.set(1, 0, 0);
        else this._rainSide.normalize();
        this._rainLook.crossVectors(this._rainSide, this._rainDir).normalize();

        this._rainPos.set(rx, ry, rz);
        this._m4.makeBasis(this._rainSide, this._rainDir, this._rainLook);
        this._m4.scale(this._s.set(1, d.len, 1));
        this._m4.setPosition(this._rainPos);
        this._rainMesh.setMatrixAt(i, this._m4);
      }
      this._rainMesh.instanceMatrix.needsUpdate = true;
    }
  }
}

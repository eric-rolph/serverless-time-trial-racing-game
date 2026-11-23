// 2026 AAA HDR Post-Processing Pipeline — WebGL2 Half-Float MSAA capture,
// 3-level Dual Kawase Bloom with Karis anti-firefly soft-knee prefilter &
// 9-tap tent upsample, speed-reactive peripheral chromatic dispersion & motion
// rush (zero apex blur), AgX Filmic Punchy tone mapping, optical lens vignette,
// and triangular-PDF interleaved gradient noise dithering.
//
// Graceful fallback ladder:
//   WebGL2 RGBA16F MSAA (4x) → WebGL2 RGBA16F (0x) → direct renderer.render().
// Zero per-frame allocations inside render().
import * as THREE from "../vendor/three.module.js";

const FULLSCREEN_VERT = /* glsl */ `
varying vec2 vUv;
void main() {
  vUv = uv;
  gl_Position = vec4(position.xy, 0.0, 1.0);
}`;

// Pass 1: Karis-weighted 5-tap Dual Kawase downsample + quadratic soft-knee
// bright-pass extraction into half-res mip1. Kills single-pixel specular
// fireflies on wet tarmac/carbon while preserving wide HDR energy on brake
// rotors, headlights, FIA gantry LEDs, exhaust flames, and solar corona.
const BRIGHT_EXTRACT_FRAG = /* glsl */ `
uniform sampler2D uTex;
uniform vec2 uTexelSize;
uniform float uThreshold;
uniform float uKnee;
varying vec2 vUv;

vec3 softPrefilter(vec3 c, float threshold, float knee) {
  c = clamp(c, vec3(0.0), vec3(32.0));
  float br = max(c.r, max(c.g, c.b));
  float soft = clamp(br - threshold + knee, 0.0, 2.0 * knee);
  soft = (soft * soft) / (4.0 * knee + 1e-5);
  float contrib = max(soft, br - threshold) / max(br, 1e-5);
  return c * contrib;
}

float karisWeight(vec3 c) {
  float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
  return 1.0 / (1.0 + luma);
}

void main() {
  vec2 o = uTexelSize;
  vec3 c0 = softPrefilter(texture2D(uTex, vUv).rgb, uThreshold, uKnee);
  vec3 c1 = softPrefilter(texture2D(uTex, vUv + vec2(-o.x, -o.y)).rgb, uThreshold, uKnee);
  vec3 c2 = softPrefilter(texture2D(uTex, vUv + vec2( o.x, -o.y)).rgb, uThreshold, uKnee);
  vec3 c3 = softPrefilter(texture2D(uTex, vUv + vec2(-o.x,  o.y)).rgb, uThreshold, uKnee);
  vec3 c4 = softPrefilter(texture2D(uTex, vUv + vec2( o.x,  o.y)).rgb, uThreshold, uKnee);

  float w0 = 0.500 * karisWeight(c0);
  float w1 = 0.125 * karisWeight(c1);
  float w2 = 0.125 * karisWeight(c2);
  float w3 = 0.125 * karisWeight(c3);
  float w4 = 0.125 * karisWeight(c4);

  vec3 sum = c0 * w0 + c1 * w1 + c2 * w2 + c3 * w3 + c4 * w4;
  float wSum = max(w0 + w1 + w2 + w3 + w4, 1e-5);
  gl_FragColor = vec4(sum / wSum, 1.0);
}`;

// Downsample pass: 5-tap Dual Kawase filter (13-pixel effective footprint via
// bilinear filtering across half-pixel boundaries).
const KAWASE_DOWN_FRAG = /* glsl */ `
uniform sampler2D uTex;
uniform vec2 uTexelSize;
varying vec2 vUv;

void main() {
  vec2 o = uTexelSize;
  vec3 sum = texture2D(uTex, vUv).rgb * 0.5;
  sum += texture2D(uTex, vUv + vec2(-o.x, -o.y)).rgb * 0.125;
  sum += texture2D(uTex, vUv + vec2( o.x, -o.y)).rgb * 0.125;
  sum += texture2D(uTex, vUv + vec2(-o.x,  o.y)).rgb * 0.125;
  sum += texture2D(uTex, vUv + vec2( o.x,  o.y)).rgb * 0.125;
  gl_FragColor = vec4(max(sum, vec3(0.0)), 1.0);
}`;

// Upsample pass: 9-tap tent filter (1-2-1 / 2-4-2 / 1-2-1) additively merged
// with the finer mip level for zero boxy artifacts and wide cinematic halation.
const KAWASE_UP_FRAG = /* glsl */ `
uniform sampler2D uLowTex;
uniform sampler2D uHighTex;
uniform vec2 uTexelSize;
uniform float uRadius;
uniform float uLowWeight;
varying vec2 vUv;

void main() {
  vec2 d = uTexelSize * uRadius;
  vec3 s  = texture2D(uLowTex, vUv + vec2(-d.x,  d.y)).rgb;
  s      += texture2D(uLowTex, vUv + vec2( 0.0,  d.y)).rgb * 2.0;
  s      += texture2D(uLowTex, vUv + vec2( d.x,  d.y)).rgb;
  s      += texture2D(uLowTex, vUv + vec2(-d.x,  0.0)).rgb * 2.0;
  s      += texture2D(uLowTex, vUv                   ).rgb * 4.0;
  s      += texture2D(uLowTex, vUv + vec2( d.x,  0.0)).rgb * 2.0;
  s      += texture2D(uLowTex, vUv + vec2(-d.x, -d.y)).rgb;
  s      += texture2D(uLowTex, vUv + vec2( 0.0, -d.y)).rgb * 2.0;
  s      += texture2D(uLowTex, vUv + vec2( d.x, -d.y)).rgb;
  vec3 up = s * (1.0 / 16.0);
  vec3 hi = texture2D(uHighTex, vUv).rgb;
  gl_FragColor = vec4(max(hi + up * uLowWeight, vec3(0.0)), 1.0);
}`;

// Final composite: speed-reactive peripheral chromatic dispersion & radial
// motion rush (center apex sightline kept 100% razor sharp), HDR bloom merge,
// exposure, 2026 AgX Filmic Punchy tone mapping, optical lens vignette,
// exact linear→sRGB transfer, and triangular-PDF dithering.
const COMPOSITE_FRAG = /* glsl */ `
uniform sampler2D uScene;
uniform sampler2D uBloom;
uniform float uExposure;
uniform float uBloomStrength;
uniform float uSpeedNorm;
uniform float uWetness;
uniform float uTime;
varying vec2 vUv;

// AgX linear sRGB → AgX Inset matrix (GLSL column-major)
const mat3 AGX_INSET = mat3(
  0.842479062253094,  0.0423282422610123, 0.0423756549057051,
  0.0784335999999992, 0.878468636469772,  0.0784336,
  0.0792237451477643, 0.0791661274605434, 0.879142973793104
);

// AgX Outset matrix → Display sRGB primaries (GLSL column-major)
const mat3 AGX_OUTSET = mat3(
   1.19687900512017,   -0.0528968517574562, -0.0529716355144438,
  -0.0980208811401368,  1.15190312990417,   -0.0980434501171241,
  -0.0990297440797205, -0.0989611768448433,  1.15107367264116
);

vec3 agxDefaultContrastApprox(vec3 x) {
  vec3 x2 = x * x;
  vec3 x4 = x2 * x2;
  return + 15.5     * x4 * x2
         - 40.14    * x4 * x
         + 31.96    * x4
         - 6.868    * x2 * x
         + 0.4298   * x2
         + 0.1191   * x
         - 0.00232;
}

vec3 agxFilmicPunchy(vec3 hdr) {
  const float minEv = -12.47393;
  const float maxEv =  4.026069;
  const vec3 lw = vec3(0.2126, 0.7152, 0.0722);

  vec3 val = AGX_INSET * max(hdr, vec3(0.0));
  val = clamp(log2(max(val, vec3(1e-10))), minEv, maxEv);
  val = (val - minEv) / (maxEv - minEv);
  val = clamp(agxDefaultContrastApprox(val), 0.0, 1.0);

  // Subtle Filmic Punchy Look: richer shadow floor, clean contrast, vivid motorsport hues
  val = pow(max(val * 1.035, vec3(1e-6)), vec3(1.12));
  float luma = dot(val, lw);
  val = mix(vec3(luma), val, 1.13);

  val = AGX_OUTSET * val;
  val = clamp(val, 0.0, 1.0);
  // AgX curve outputs ~2.2-encoded values; return linear for vignette + exact sRGB transfer
  return pow(max(val, vec3(1e-6)), vec3(2.2));
}

vec3 linearToSRGB(vec3 c) {
  vec3 safe = max(c, vec3(0.0));
  vec3 lo = safe * 12.92;
  vec3 hi = 1.055 * pow(max(safe, vec3(1e-6)), vec3(1.0 / 2.4)) - 0.055;
  return mix(lo, hi, step(vec3(0.0031308), safe));
}

// Interleaved Gradient Noise (Jimenez) for triangular-PDF 8-bit de-banding
float ign(vec2 p) {
  return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
}

void main() {
  vec2 centered = vUv - 0.5;
  // Slightly wider horizontal safe zone for wide monitors / apex focus
  vec2 radVec = vec2(centered.x * 1.05, centered.y * 1.12);
  float r2 = dot(radVec, radVec);

  // Strictly zero below ~110 km/h (speedNorm ~0.40), smooth ramp toward 250+ km/h
  float speedFactor = smoothstep(0.40, 1.0, clamp(uSpeedNorm, 0.0, 1.0));
  // Strict apex-protection mask: center 26% radius stays 100% untouched
  float edgeMask = smoothstep(0.075, 0.44, r2);

  // Subtle optical radial edge chromatic dispersion + speed-reactive peripheral streak
  float caStrength = (0.00045 + 0.0034 * speedFactor) * edgeMask;
  vec2 caOffset = centered * caStrength;

  vec3 sceneCol;
  if (caStrength > 0.0001) {
    sceneCol.r = texture2D(uScene, clamp(vUv + caOffset, 0.001, 0.999)).r;
    sceneCol.g = texture2D(uScene, vUv).g;
    sceneCol.b = texture2D(uScene, clamp(vUv - caOffset, 0.001, 0.999)).b;
  } else {
    sceneCol = texture2D(uScene, vUv).rgb;
  }

  // Gentle 3-tap radial peripheral speed softening at high velocity (zero center blur)
  float rushAmt = speedFactor * edgeMask;
  if (rushAmt > 0.005) {
    vec2 stepUv = centered * (0.0075 * rushAmt);
    vec3 tap1 = texture2D(uScene, clamp(vUv - stepUv, 0.001, 0.999)).rgb;
    vec3 tap2 = texture2D(uScene, clamp(vUv - stepUv * 2.0, 0.001, 0.999)).rgb;
    sceneCol = sceneCol * 0.62 + tap1 * 0.25 + tap2 * 0.13;
  }

  // Multi-scale linear HDR bloom merge (+ mild wet-air atmospheric halation)
  vec3 bloom = texture2D(uBloom, vUv).rgb;
  float bloomAmt = uBloomStrength * 0.36 * (1.0 + 0.14 * clamp(uWetness, 0.0, 1.0));
  vec3 hdr = max(sceneCol + bloom * bloomAmt, vec3(0.0));

  // Exposure + 2026 AgX Filmic Punchy tone mapping
  hdr *= uExposure;
  vec3 col = agxFilmicPunchy(hdr);

  // Optical cinema-lens cos-4 vignette (gentle, deepens slightly at high speed)
  float vig = 1.0 - smoothstep(0.20, 0.98, r2) * (0.19 + 0.08 * speedFactor);
  col *= vig;

  // Exact sRGB encoding + triangular-PDF dither (zero 8-bit sky gradient banding)
  vec3 srgb = linearToSRGB(clamp(col, 0.0, 1.0));
  vec2 ditherSeed = gl_FragCoord.xy + vec2(fract(uTime * 13.7) * 29.0, fract(uTime * 7.3) * 17.0);
  float dither = (ign(ditherSeed) - ign(ditherSeed + vec2(17.1, 31.7))) / 255.0;
  srgb = clamp(srgb + dither, 0.0, 1.0);

  gl_FragColor = vec4(srgb, 1.0);
}`;

/** Build a single oversized clip-space triangle covering NDC [-1, 1]^2 seamlessly. */
function buildFullscreenTriangleGeo() {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute(
    "position",
    new THREE.BufferAttribute(new Float32Array([-1, -1, 0, 3, -1, 0, -1, 3, 0]), 3),
  );
  geo.setAttribute(
    "uv",
    new THREE.BufferAttribute(new Float32Array([0, 0, 2, 0, 0, 2]), 2),
  );
  return geo;
}

/** Probe WebGL2 RGBA16F renderability (and optional MSAA) cleanly without leaking GL state. */
function probeHalfFloatSupport(renderer) {
  if (!renderer?.capabilities?.isWebGL2) return { hdr: false, samples: 0 };
  const gl = renderer.getContext?.();
  if (!gl || !gl.RGBA16F || !gl.HALF_FLOAT) return { hdr: false, samples: 0 };

  // Enable WebGL2 float/half-float color buffer rendering extensions if exposed.
  try {
    renderer.extensions?.get?.("EXT_color_buffer_float");
    renderer.extensions?.get?.("EXT_color_buffer_half_float");
  } catch { /* ignore */ }

  const prevFBO = gl.getParameter(gl.FRAMEBUFFER_BINDING);
  const prevRB = gl.getParameter(gl.RENDERBUFFER_BINDING);
  const prevTex = gl.getParameter(gl.TEXTURE_BINDING_2D);

  let hdrOk = false;
  let msaaSamples = 0;

  try {
    // 1. Test non-MSAA RGBA16F color-attachment texture FBO
    const fbo = gl.createFramebuffer();
    const tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, 4, 4, 0, gl.RGBA, gl.HALF_FLOAT, null);
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    hdrOk = gl.checkFramebufferStatus(gl.FRAMEBUFFER) === gl.FRAMEBUFFER_COMPLETE;
    gl.deleteTexture(tex);
    gl.deleteFramebuffer(fbo);

    // 2. If HDR works, probe 4x MSAA RGBA16F renderbuffer completeness
    if (hdrOk && typeof gl.renderbufferStorageMultisample === "function") {
      const maxS = gl.getParameter(gl.MAX_SAMPLES) || 0;
      const want = Math.min(4, maxS);
      if (want >= 2) {
        const mfbo = gl.createFramebuffer();
        const crb = gl.createRenderbuffer();
        const drb = gl.createRenderbuffer();
        gl.bindFramebuffer(gl.FRAMEBUFFER, mfbo);
        gl.bindRenderbuffer(gl.RENDERBUFFER, crb);
        gl.renderbufferStorageMultisample(gl.RENDERBUFFER, want, gl.RGBA16F, 4, 4);
        gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.RENDERBUFFER, crb);
        gl.bindRenderbuffer(gl.RENDERBUFFER, drb);
        gl.renderbufferStorageMultisample(gl.RENDERBUFFER, want, gl.DEPTH_COMPONENT24, 4, 4);
        gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, drb);
        if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) === gl.FRAMEBUFFER_COMPLETE) {
          msaaSamples = want;
        }
        gl.deleteRenderbuffer(drb);
        gl.deleteRenderbuffer(crb);
        gl.deleteFramebuffer(mfbo);
      }
    }
  } catch {
    hdrOk = false;
    msaaSamples = 0;
  } finally {
    gl.bindFramebuffer(gl.FRAMEBUFFER, prevFBO);
    gl.bindRenderbuffer(gl.RENDERBUFFER, prevRB);
    gl.bindTexture(gl.TEXTURE_2D, prevTex);
    // Drain any probe GL errors so caller state stays spotless
    while (gl.getError() !== gl.NO_ERROR) { /* drain */ }
  }

  return { hdr: hdrOk, samples: msaaSamples };
}

export class PostProcessPipeline {
  constructor(renderer, scene, camera) {
    this.renderer = renderer;
    this.scene = scene;
    this.camera = camera;

    const caps = probeHalfFloatSupport(renderer);
    this._enabled = caps.hdr;
    this._samples = caps.samples;
    this._validatedFirstFrame = false;

    this._pw = 0;
    this._ph = 0;
    this._drawSize = new THREE.Vector2(1, 1);

    this._sceneRT = null;
    this._mip1 = null;
    this._mip2 = null;
    this._mip3 = null;
    this._up2 = null;
    this._up1 = null;

    if (!this._enabled) return;

    // Shared fullscreen triangle pass rig (zero allocations during render)
    this._fsGeo = buildFullscreenTriangleGeo();
    this._fsCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this._fsScene = new THREE.Scene();

    this._brightMat = new THREE.ShaderMaterial({
      uniforms: {
        uTex: { value: null },
        uTexelSize: { value: new THREE.Vector2(1 / 1280, 1 / 720) },
        uThreshold: { value: 1.05 },
        uKnee: { value: 0.55 },
      },
      vertexShader: FULLSCREEN_VERT,
      fragmentShader: BRIGHT_EXTRACT_FRAG,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });

    this._downMat = new THREE.ShaderMaterial({
      uniforms: {
        uTex: { value: null },
        uTexelSize: { value: new THREE.Vector2(1 / 640, 1 / 360) },
      },
      vertexShader: FULLSCREEN_VERT,
      fragmentShader: KAWASE_DOWN_FRAG,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });

    this._upMat = new THREE.ShaderMaterial({
      uniforms: {
        uLowTex: { value: null },
        uHighTex: { value: null },
        uTexelSize: { value: new THREE.Vector2(1 / 320, 1 / 180) },
        uRadius: { value: 1.15 },
        uLowWeight: { value: 0.88 },
      },
      vertexShader: FULLSCREEN_VERT,
      fragmentShader: KAWASE_UP_FRAG,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });

    this._compMat = new THREE.ShaderMaterial({
      uniforms: {
        uScene: { value: null },
        uBloom: { value: null },
        uExposure: { value: 1.0 },
        uBloomStrength: { value: 0.42 },
        uSpeedNorm: { value: 0.0 },
        uWetness: { value: 0.0 },
        uTime: { value: 0.0 },
      },
      vertexShader: FULLSCREEN_VERT,
      fragmentShader: COMPOSITE_FRAG,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    });

    this._fsMesh = new THREE.Mesh(this._fsGeo, this._compMat);
    this._fsMesh.frustumCulled = false;
    this._fsScene.add(this._fsMesh);

    // Initialize targets to current drawing buffer dimensions if already sized
    renderer.getDrawingBufferSize(this._drawSize);
    const pr = renderer.getPixelRatio ? renderer.getPixelRatio() : 1;
    const initW = Math.max(1, Math.round(this._drawSize.x / (pr || 1)));
    const initH = Math.max(1, Math.round(this._drawSize.y / (pr || 1)));
    this.setSize(initW, initH, pr || 1);
  }

  _makeRT(w, h, { depth = false, samples = 0 } = {}) {
    return new THREE.WebGLRenderTarget(w, h, {
      type: THREE.HalfFloatType,
      format: THREE.RGBAFormat,
      colorSpace: THREE.LinearSRGBColorSpace,
      minFilter: THREE.LinearFilter,
      magFilter: THREE.LinearFilter,
      wrapS: THREE.ClampToEdgeWrapping,
      wrapT: THREE.ClampToEdgeWrapping,
      depthBuffer: depth,
      stencilBuffer: false,
      generateMipmaps: false,
      samples,
    });
  }

  _allocTargets(pw, ph) {
    this._disposeTargets();
    const w1 = Math.max(1, pw >> 1);
    const h1 = Math.max(1, ph >> 1);
    const w2 = Math.max(1, w1 >> 1);
    const h2 = Math.max(1, h1 >> 1);
    const w3 = Math.max(1, w2 >> 1);
    const h3 = Math.max(1, h2 >> 1);

    this._sceneRT = this._makeRT(pw, ph, { depth: true, samples: this._samples });
    this._mip1 = this._makeRT(w1, h1);
    this._mip2 = this._makeRT(w2, h2);
    this._mip3 = this._makeRT(w3, h3);
    this._up2 = this._makeRT(w2, h2);
    this._up1 = this._makeRT(w1, h1);
    this._pw = pw;
    this._ph = ph;
  }

  _disposeTargets() {
    this._sceneRT?.dispose();
    this._mip1?.dispose();
    this._mip2?.dispose();
    this._mip3?.dispose();
    this._up2?.dispose();
    this._up1?.dispose();
    this._sceneRT = this._mip1 = this._mip2 = this._mip3 = this._up2 = this._up1 = null;
  }

  setSize(width, height, pixelRatio = 1) {
    if (!this._enabled) return;
    const pw = Math.max(1, Math.floor(width * pixelRatio));
    const ph = Math.max(1, Math.floor(height * pixelRatio));
    if (pw === this._pw && ph === this._ph && this._sceneRT) return;

    if (!this._sceneRT) {
      this._allocTargets(pw, ph);
      return;
    }

    const w1 = Math.max(1, pw >> 1);
    const h1 = Math.max(1, ph >> 1);
    const w2 = Math.max(1, w1 >> 1);
    const h2 = Math.max(1, h1 >> 1);
    const w3 = Math.max(1, w2 >> 1);
    const h3 = Math.max(1, h2 >> 1);

    this._sceneRT.setSize(pw, ph);
    this._mip1.setSize(w1, h1);
    this._mip2.setSize(w2, h2);
    this._mip3.setSize(w3, h3);
    this._up2.setSize(w2, h2);
    this._up1.setSize(w1, h1);
    this._pw = pw;
    this._ph = ph;
  }

  _runFullscreenPass(material, target) {
    this._fsMesh.material = material;
    this.renderer.setRenderTarget(target);
    this.renderer.render(this._fsScene, this._fsCam);
  }

  render({
    exposure = 1.0,
    bloomStrength = 0.42,
    bloomThreshold = 1.05,
    speedNorm = 0,
    wetness = 0,
    timeSec = 0,
  } = {}) {
    const renderer = this.renderer;

    if (!this._enabled) {
      renderer.render(this.scene, this.camera);
      return;
    }

    // Keep FBO dimensions synced if canvas resized externally without calling setSize()
    renderer.getDrawingBufferSize(this._drawSize);
    const dbW = this._drawSize.x | 0;
    const dbH = this._drawSize.y | 0;
    if (dbW < 1 || dbH < 1) return;
    if (dbW !== this._pw || dbH !== this._ph || !this._sceneRT) {
      this.setSize(dbW, dbH, 1);
    }

    const prevToneMapping = renderer.toneMapping;
    const prevAutoClear = renderer.autoClear;
    const prevTarget = renderer.getRenderTarget();

    try {
      // 0. Capture full HDR linear scene into HalfFloat MSAA FBO
      renderer.toneMapping = THREE.NoToneMapping;
      renderer.autoClear = true;
      renderer.setRenderTarget(this._sceneRT);

      // Runtime first-frame MSAA safety verification (guards quirky drivers / SwiftShader)
      if (!this._validatedFirstFrame) {
        const gl = renderer.getContext();
        const status = gl.checkFramebufferStatus(gl.FRAMEBUFFER);
        if (status !== gl.FRAMEBUFFER_COMPLETE) {
          if (this._samples > 0) {
            this._samples = 0;
            this._allocTargets(this._pw, this._ph);
            renderer.setRenderTarget(this._sceneRT);
            if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) {
              throw new Error("HDR FBO incomplete");
            }
          } else {
            throw new Error("HDR FBO incomplete");
          }
        }
        this._validatedFirstFrame = true;
      }

      renderer.render(this.scene, this.camera);

      // Post-processing fullscreen passes do not need buffer clears
      renderer.autoClear = false;

      // 1. Bright-pass extract + Karis anti-firefly downsample → mip1 (1/2 res)
      const bU = this._brightMat.uniforms;
      bU.uTex.value = this._sceneRT.texture;
      bU.uTexelSize.value.set(1 / this._pw, 1 / this._ph);
      bU.uThreshold.value = bloomThreshold;
      bU.uKnee.value = Math.max(0.25, bloomThreshold * 0.52);
      this._runFullscreenPass(this._brightMat, this._mip1);

      // 2. Dual Kawase Downsample: mip1 → mip2 (1/4 res) → mip3 (1/8 res)
      const dU = this._downMat.uniforms;
      dU.uTex.value = this._mip1.texture;
      dU.uTexelSize.value.set(1 / this._mip1.width, 1 / this._mip1.height);
      this._runFullscreenPass(this._downMat, this._mip2);

      dU.uTex.value = this._mip2.texture;
      dU.uTexelSize.value.set(1 / this._mip2.width, 1 / this._mip2.height);
      this._runFullscreenPass(this._downMat, this._mip3);

      // 3. 9-tap Tent Filter Upsample additive pyramid: mip3 → up2 → up1
      const uU = this._upMat.uniforms;
      uU.uLowTex.value = this._mip3.texture;
      uU.uHighTex.value = this._mip2.texture;
      uU.uTexelSize.value.set(1 / this._mip3.width, 1 / this._mip3.height);
      uU.uRadius.value = 1.25;
      uU.uLowWeight.value = 0.92;
      this._runFullscreenPass(this._upMat, this._up2);

      uU.uLowTex.value = this._up2.texture;
      uU.uHighTex.value = this._mip1.texture;
      uU.uTexelSize.value.set(1 / this._up2.width, 1 / this._up2.height);
      uU.uRadius.value = 1.05;
      uU.uLowWeight.value = 0.84;
      this._runFullscreenPass(this._upMat, this._up1);

      // 4. Final Composite Pass to Screen (AgX Filmic Punchy + CA + Vignette + Dither)
      const cU = this._compMat.uniforms;
      cU.uScene.value = this._sceneRT.texture;
      cU.uBloom.value = this._up1.texture;
      cU.uExposure.value = exposure;
      cU.uBloomStrength.value = bloomStrength;
      cU.uSpeedNorm.value = speedNorm;
      cU.uWetness.value = wetness;
      cU.uTime.value = timeSec;
      this._runFullscreenPass(this._compMat, prevTarget);
    } catch {
      // Graceful downgrade: if MSAA threw at runtime, retry once without MSAA next frame;
      // otherwise fall back permanently to direct rendering.
      if (this._samples > 0) {
        this._samples = 0;
        this._validatedFirstFrame = false;
        this._disposeTargets();
      } else {
        this._enabled = false;
        this.dispose();
      }
      renderer.toneMapping = prevToneMapping;
      renderer.autoClear = prevAutoClear;
      renderer.setRenderTarget(prevTarget);
      renderer.render(this.scene, this.camera);
      return;
    }

    renderer.toneMapping = prevToneMapping;
    renderer.autoClear = prevAutoClear;
    renderer.setRenderTarget(prevTarget);
  }

  dispose() {
    this._disposeTargets();
    this._fsGeo?.dispose();
    this._brightMat?.dispose();
    this._downMat?.dispose();
    this._upMat?.dispose();
    this._compMat?.dispose();
    this._fsGeo = this._brightMat = this._downMat = this._upMat = this._compMat = null;
  }
}

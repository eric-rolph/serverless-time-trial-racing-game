// Cosmetic race VFX engine — 100% read-only on SimStateV1 + lighting state,
// zero physics coupling, zero per-frame allocations:
//   • Ring-buffered tire skidmark ribbons laid continuously on the tarmac under
//     heavy braking or lateral slide (dark rubber on asphalt, brown ruts off-track);
//   • Instanced soft-billboard particle pool for tire smoke, off-track gravel/
//     dust plumes, and wet-weather rooster-tail spray behind spinning slicks;
//   • Additive incandescent crash-spark bursts triggered by chassis impacts.
import * as THREE from "../vendor/three.module.js";

const MAX_SKIDS = 720;     // quad segments across all 4 wheels (~1440 tris)
const MAX_SMOKE = 220;     // smoke + gravel + wet spray billboards
const MAX_SPARKS = 96;     // high-velocity impact sparks
const WHEEL_RADIUS = 0.33;
const SKID_HALF_W = 0.145; // matches slick tread half-width

function softPuffTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d");
  const grad = g.createRadialGradient(32, 32, 2, 32, 32, 32);
  grad.addColorStop(0.0, "rgba(255,255,255,0.85)");
  grad.addColorStop(0.45, "rgba(255,255,255,0.38)");
  grad.addColorStop(1.0, "rgba(255,255,255,0.0)");
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export class RaceEffects {
  constructor(scene) {
    this.scene = scene;

    // --------------------------------------------------------- skidmark ribbon
    // Each segment is 2 triangles = 6 vertices (non-indexed for zero-cost ring overwrite).
    this._skidPos = new Float32Array(MAX_SKIDS * 6 * 3);
    this._skidCol = new Float32Array(MAX_SKIDS * 6 * 4);
    this._skidHead = 0;
    this._skidGeo = new THREE.BufferGeometry();
    this._skidGeo.setAttribute("position", new THREE.BufferAttribute(this._skidPos, 3));
    this._skidGeo.setAttribute("color", new THREE.BufferAttribute(this._skidCol, 4));
    this._skidMesh = new THREE.Mesh(
      this._skidGeo,
      new THREE.MeshBasicMaterial({
        vertexColors: true,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
      }),
    );
    this._skidMesh.frustumCulled = false;
    this._skidMesh.renderOrder = 5;
    scene.add(this._skidMesh);

    // Previous contact patch per wheel so adjacent quads stitch smoothly
    this._prevPatch = [null, null, null, null];
    this._prevRight = [
      new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(),
    ];

    // --------------------------------------------------------- smoke/dust/spray pool
    const puffGeo = new THREE.PlaneGeometry(1, 1);
    const puffMat = new THREE.MeshBasicMaterial({
      map: softPuffTexture(),
      transparent: true,
      opacity: 0.52,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this._smokeMesh = new THREE.InstancedMesh(puffGeo, puffMat, MAX_SMOKE);
    this._smokeMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this._smokeMesh.frustumCulled = false;
    this._smokeMesh.renderOrder = 6;
    this._smokePool = Array.from({ length: MAX_SMOKE }, () => ({
      alive: false, age: 0, life: 1,
      pos: new THREE.Vector3(), vel: new THREE.Vector3(),
      size0: 0.3, size1: 1.5, color: new THREE.Color(1, 1, 1),
    }));
    this._smokeCursor = 0;
    scene.add(this._smokeMesh);

    // --------------------------------------------------------- crash spark pool
    const sparkGeo = new THREE.BoxGeometry(0.035, 0.035, 0.24);
    const sparkMat = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.95,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    this._sparkMesh = new THREE.InstancedMesh(sparkGeo, sparkMat, MAX_SPARKS);
    this._sparkMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this._sparkMesh.frustumCulled = false;
    this._sparkMesh.renderOrder = 7;
    this._sparkPool = Array.from({ length: MAX_SPARKS }, () => ({
      alive: false, age: 0, life: 0.4,
      pos: new THREE.Vector3(), vel: new THREE.Vector3(),
      color: new THREE.Color(1, 0.65, 0.2),
    }));
    this._sparkCursor = 0;
    scene.add(this._sparkMesh);

    // Scratch math objects (zero GC pressure in rAF)
    this._up = new THREE.Vector3();
    this._fwd = new THREE.Vector3();
    this._right = new THREE.Vector3();
    this._patch = new THREE.Vector3();
    this._segDir = new THREE.Vector3();
    this._segRight = new THREE.Vector3();
    this._m4 = new THREE.Matrix4();
    this._q = new THREE.Quaternion();
    this._s = new THREE.Vector3();
    this._zeroM = new THREE.Matrix4().makeScale(0, 0, 0);
    this._emitAcc = [0, 0, 0, 0];
    this._brakeHeat = [0, 0, 0, 0];

    this.reset();
  }

  reset() {
    this._skidPos.fill(0);
    this._skidCol.fill(0);
    this._skidHead = 0;
    this._skidGeo.attributes.position.needsUpdate = true;
    this._skidGeo.attributes.color.needsUpdate = true;
    for (let i = 0; i < 4; i++) {
      this._prevPatch[i] = null;
      this._emitAcc[i] = 0;
      this._brakeHeat[i] = 0;
    }
    const white = new THREE.Color(1, 1, 1);
    for (let i = 0; i < MAX_SMOKE; i++) {
      this._smokePool[i].alive = false;
      this._smokeMesh.setMatrixAt(i, this._zeroM);
      this._smokeMesh.setColorAt(i, white);
    }
    this._smokeMesh.instanceMatrix.needsUpdate = true;
    this._smokeMesh.instanceColor.needsUpdate = true;
    for (let i = 0; i < MAX_SPARKS; i++) {
      this._sparkPool[i].alive = false;
      this._sparkMesh.setMatrixAt(i, this._zeroM);
      this._sparkMesh.setColorAt(i, white);
    }
    this._sparkMesh.instanceMatrix.needsUpdate = true;
    this._sparkMesh.instanceColor.needsUpdate = true;
  }

  _writeSkidQuad(p0, r0, p1, r1, r, g, b, alpha) {
    const o = this._skidHead * 18;
    const co = this._skidHead * 24;
    this._skidHead = (this._skidHead + 1) % MAX_SKIDS;
    const ax = p0.x - r0.x, ay = p0.y - r0.y, az = p0.z - r0.z;
    const bx = p0.x + r0.x, by = p0.y + r0.y, bz = p0.z + r0.z;
    const cx = p1.x - r1.x, cy = p1.y - r1.y, cz = p1.z - r1.z;
    const dx = p1.x + r1.x, dy = p1.y + r1.y, dz = p1.z + r1.z;
    const P = this._skidPos;
    P[o] = ax; P[o + 1] = ay; P[o + 2] = az;
    P[o + 3] = bx; P[o + 4] = by; P[o + 5] = bz;
    P[o + 6] = cx; P[o + 7] = cy; P[o + 8] = cz;
    P[o + 9] = bx; P[o + 10] = by; P[o + 11] = bz;
    P[o + 12] = dx; P[o + 13] = dy; P[o + 14] = dz;
    P[o + 15] = cx; P[o + 16] = cy; P[o + 17] = cz;
    const C = this._skidCol;
    for (let k = 0; k < 6; k++) {
      C[co + k * 4] = r;
      C[co + k * 4 + 1] = g;
      C[co + k * 4 + 2] = b;
      C[co + k * 4 + 3] = alpha;
    }
    this._skidGeo.attributes.position.needsUpdate = true;
    this._skidGeo.attributes.color.needsUpdate = true;
  }

  _spawnSmoke(pos, vel, size0, size1, life, hexColor) {
    const p = this._smokePool[this._smokeCursor];
    this._smokeCursor = (this._smokeCursor + 1) % MAX_SMOKE;
    p.alive = true;
    p.age = 0;
    p.life = life;
    p.pos.copy(pos);
    p.vel.copy(vel);
    p.size0 = size0;
    p.size1 = size1;
    p.color.setHex(hexColor);
  }

  /** Burst fiery sparks at a world-space crash site. */
  emitImpactSparks(worldPos, severity = 1.0) {
    const count = Math.min(40, Math.max(12, Math.round(severity * 28)));
    for (let k = 0; k < count; k++) {
      const p = this._sparkPool[this._sparkCursor];
      this._sparkCursor = (this._sparkCursor + 1) % MAX_SPARKS;
      p.alive = true;
      p.age = 0;
      p.life = 0.25 + Math.random() * 0.42;
      p.pos.copy(worldPos).addScaledVector(this._up.set(0, 1, 0), 0.15 + Math.random() * 0.25);
      const ang = Math.random() * Math.PI * 2;
      const spd = 4 + Math.random() * (9 * severity + 5);
      p.vel.set(Math.cos(ang) * spd, 2.2 + Math.random() * 6.5, Math.sin(ang) * spd);
      p.color.setHex(Math.random() > 0.3 ? 0xffb84d : 0xfff0b8);
    }
  }

  /** Update skidmarks, tire smoke/gravel/wet-spray, brake-disc heat, and sparks. */
  update({
    state, carGroup, wheelMeshes, camera, dtSec, brake = 0,
    offTrack = false, wetness = 0, sim = null, frozen = false,
  }) {
    const dt = Math.min(dtSec, 0.05);
    this._up.set(0, 1, 0).applyQuaternion(carGroup.quaternion).normalize();
    this._fwd.set(0, 0, 1).applyQuaternion(carGroup.quaternion).normalize();
    this._right.set(1, 0, 0).applyQuaternion(carGroup.quaternion).normalize();
    const speed = state.speed;

    // ---- per-wheel brake disc heat + skidmarks + smoke/gravel/spray
    for (let i = 0; i < 4; i++) {
      const w = state.wheels[i];
      // Brake disc thermal model (visual-only): front takes 65% of brake energy
      const frontBias = i < 2 ? 1.15 : 0.72;
      const heatGain = brake * Math.min(1, speed / 35) * frontBias * 1.65;
      const cool = (0.35 + speed * 0.018) * this._brakeHeat[i];
      this._brakeHeat[i] = Math.max(0, Math.min(1, this._brakeHeat[i] + (heatGain - cool) * dt));
      wheelMeshes[i]?.userData?.setBrakeGlow?.(this._brakeHeat[i]);

      if (frozen) {
        this._prevPatch[i] = null;
        continue;
      }

      const loaded = w.compression > 0.005;
      const slipLat = Math.abs(w.slipAngle);
      const slipLon = Math.abs(w.slipRatio);
      const slipMag = Math.hypot(slipLat * 1.15, slipLon);
      const temp = sim?.tireTemp?.(i) ?? 75;

      // Ground contact point just above asphalt surface
      this._patch.set(w.pos[0], w.pos[1], w.pos[2]).addScaledVector(this._up, -(WHEEL_RADIUS - 0.018));

      // Skidmarks when sliding or locked up on a loaded wheel
      const skidIntensity = loaded && speed > 3.5 ? Math.max(0, (slipMag - 0.14) / 0.35) : 0;
      if (skidIntensity > 0.02) {
        if (this._prevPatch[i]) {
          this._segDir.copy(this._patch).sub(this._prevPatch[i]);
          const segLen = this._segDir.length();
          if (segLen > 0.25 && segLen < 4.5) {
            this._segDir.divideScalar(segLen);
            this._segRight.crossVectors(this._up, this._segDir).normalize().multiplyScalar(SKID_HALF_W);
            if (this._segRight.dot(this._prevRight[i]) < 0) this._segRight.negate();
            const alpha = Math.min(0.76, skidIntensity * (offTrack ? 0.58 : 0.72) * (1 - wetness * 0.45));
            const r = offTrack ? 0.016 : 0.008;
            const g = offTrack ? 0.012 : 0.009;
            const b = offTrack ? 0.008 : 0.011;
            this._writeSkidQuad(this._prevPatch[i], this._prevRight[i], this._patch, this._segRight, r, g, b, alpha);
            this._prevPatch[i].copy(this._patch);
            this._prevRight[i].copy(this._segRight);
          } else if (segLen >= 4.5) {
            this._prevPatch[i].copy(this._patch);
            this._prevRight[i].copy(this._right).multiplyScalar(SKID_HALF_W);
          }
        } else {
          this._prevPatch[i] = this._patch.clone();
          this._prevRight[i].copy(this._right).multiplyScalar(SKID_HALF_W);
        }
      } else {
        this._prevPatch[i] = null;
      }

      // Particle emission accumulator per wheel
      this._emitAcc[i] += dt;
      // 1. Tire smoke when seriously sliding on dry tarmac
      if (loaded && !offTrack && wetness < 0.4 && slipMag > 0.24 && speed > 5 && this._emitAcc[i] > 0.038) {
        this._emitAcc[i] = 0;
        const hot = Math.min(1, Math.max(0, (temp - 55) / 55));
        const vel = new THREE.Vector3(
          (Math.random() - 0.5) * 1.2 - this._fwd.x * 1.4,
          0.65 + Math.random() * 0.65,
          (Math.random() - 0.5) * 1.2 - this._fwd.z * 1.4,
        );
        this._spawnSmoke(this._patch, vel, 0.26, 1.35 + hot * 0.65, 0.55 + Math.random() * 0.35, 0xdbe2ec);
      }
      // 2. Off-track gravel / dirt plume
      else if (loaded && offTrack && speed > 6 && this._emitAcc[i] > 0.032) {
        this._emitAcc[i] = 0;
        const vel = new THREE.Vector3(
          (Math.random() - 0.5) * 2.0 - this._fwd.x * (2.5 + speed * 0.1),
          1.2 + Math.random() * 1.6,
          (Math.random() - 0.5) * 2.0 - this._fwd.z * (2.5 + speed * 0.1),
        );
        this._spawnSmoke(this._patch, vel, 0.22, 1.15, 0.48 + Math.random() * 0.25, 0x9b8969);
      }
      // 3. Wet-track rooster-tail water mist behind spinning slicks
      else if (loaded && wetness > 0.35 && speed > 12 && this._emitAcc[i] > (i >= 2 ? 0.028 : 0.055)) {
        this._emitAcc[i] = 0;
        const sprayV = Math.min(1, (speed - 10) / 45);
        const vel = new THREE.Vector3(
          (Math.random() - 0.5) * 1.1 - this._fwd.x * (3.5 + speed * 0.14),
          0.9 + sprayV * 1.8 + Math.random() * 0.6,
          (Math.random() - 0.5) * 1.1 - this._fwd.z * (3.5 + speed * 0.14),
        );
        this._spawnSmoke(this._patch, vel, 0.28, 1.45 + sprayV * 0.9, 0.38 + Math.random() * 0.22, 0xbcd0e2);
      }
    }

    // ---- advance smoke/dust/spray billboards facing camera
    let smokeDirty = false;
    for (let i = 0; i < MAX_SMOKE; i++) {
      const p = this._smokePool[i];
      if (!p.alive) continue;
      smokeDirty = true;
      p.age += dt;
      if (p.age >= p.life) {
        p.alive = false;
        this._smokeMesh.setMatrixAt(i, this._zeroM);
        continue;
      }
      const u = p.age / p.life;
      p.pos.addScaledVector(p.vel, dt);
      p.vel.multiplyScalar(Math.max(0, 1 - 2.2 * dt));
      const sz = (p.size0 + (p.size1 - p.size0) * u) * (1 - 0.25 * u * u);
      this._m4.compose(p.pos, camera.quaternion, this._s.set(sz, sz, 1));
      this._smokeMesh.setMatrixAt(i, this._m4);
      const fade = (1 - u) * (1 - u);
      this._smokeMesh.setColorAt(i, this._tmpCol(p.color, fade));
    }
    if (smokeDirty) {
      this._smokeMesh.instanceMatrix.needsUpdate = true;
      if (this._smokeMesh.instanceColor) this._smokeMesh.instanceColor.needsUpdate = true;
    }

    // ---- advance ballistic crash sparks
    let sparkDirty = false;
    for (let i = 0; i < MAX_SPARKS; i++) {
      const p = this._sparkPool[i];
      if (!p.alive) continue;
      sparkDirty = true;
      p.age += dt;
      if (p.age >= p.life) {
        p.alive = false;
        this._sparkMesh.setMatrixAt(i, this._zeroM);
        continue;
      }
      p.vel.y -= 14.0 * dt;
      p.pos.addScaledVector(p.vel, dt);
      const u = p.age / p.life;
      this._segDir.copy(p.vel).normalize();
      this._q.setFromUnitVectors(this._tmpZ, this._segDir);
      const sc = 1 - u * 0.7;
      this._m4.compose(p.pos, this._q, this._s.set(sc, sc, sc * (0.8 + p.vel.length() * 0.08)));
      this._sparkMesh.setMatrixAt(i, this._m4);
      this._sparkMesh.setColorAt(i, this._tmpCol(p.color, 1 - u));
    }
    if (sparkDirty) {
      this._sparkMesh.instanceMatrix.needsUpdate = true;
      if (this._sparkMesh.instanceColor) this._sparkMesh.instanceColor.needsUpdate = true;
    }
  }

  _tmpCol(base, scale) {
    if (!this._cScratch) this._cScratch = new THREE.Color();
    return this._cScratch.copy(base).multiplyScalar(scale);
  }

  get _tmpZ() {
    if (!this._zVec) this._zVec = new THREE.Vector3(0, 0, 1);
    return this._zVec;
  }
}

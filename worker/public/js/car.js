// Procedural high-detail formula/prototype race car — zero external assets.
// Sculpted lofted monocoque, tapered nose cone, sculpted undercut sidepods,
// shark-fin engine cover, rear diffuser strakes, halo + driver helmet/visor,
// animated butterfly carbon steering wheel with live RPM shift-LED strip,
// articulated double-wishbone suspension arms tracking live wheel travel,
// machined 5-spoke wheels with glowing carbon-ceramic brake rotors & calipers,
// and dynamic brake / FIA rain light.
//
// Preserves 100% compatibility with Crumple soft-body crash denting: static
// bodywork meshes are baked into group space in `userData.bodyMeshes` BEFORE
// dynamic sub-groups (steering wheel, wishbones) are attached.
import * as THREE from "../vendor/three.module.js";

const CARBON = 0x15171c;
const CYAN = 0x22e0d0;

/** Loft a box between two axis-aligned rectangles (front at fz, back at bz, fz > bz). */
function loft(fz, bz, fw, bw, fBottom, bBottom, fh, bh) {
  const f = [
    [-fw / 2, fBottom, fz], [fw / 2, fBottom, fz],
    [fw / 2, fBottom + fh, fz], [-fw / 2, fBottom + fh, fz],
  ];
  const b = [
    [-bw / 2, bBottom, bz], [bw / 2, bBottom, bz],
    [bw / 2, bBottom + bh, bz], [-bw / 2, bBottom + bh, bz],
  ];
  const quads = [
    [f[0], f[1], f[2], f[3]],
    [b[1], b[0], b[3], b[2]],
    [f[3], f[2], b[2], b[3]],
    [f[1], f[0], b[0], b[1]],
    [f[1], b[1], b[2], f[2]],
    [f[0], f[3], b[3], b[0]],
  ];
  const pos = [];
  for (const [a, bq, c, d] of quads) pos.push(...a, ...bq, ...c, ...a, ...c, ...d);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pos), 3));
  geo.computeVertexNormals();
  return geo;
}

/** Offset loft for asymmetric left/right sidepods (x center shifts between front & back). */
function sideLoft(fz, bz, fx, bx, fw, bw, fBot, bBot, fh, bh) {
  const f = [
    [fx - fw / 2, fBot, fz], [fx + fw / 2, fBot, fz],
    [fx + fw / 2, fBot + fh, fz], [fx - fw / 2, fBot + fh, fz],
  ];
  const b = [
    [bx - bw / 2, bBot, bz], [bx + bw / 2, bBot, bz],
    [bx + bw / 2, bBot + bh, bz], [bx - bw / 2, bBot + bh, bz],
  ];
  const quads = [
    [f[0], f[1], f[2], f[3]],
    [b[1], b[0], b[3], b[2]],
    [f[3], f[2], b[2], b[3]],
    [f[1], f[0], b[0], b[1]],
    [f[1], b[1], b[2], f[2]],
    [f[0], f[3], b[3], b[0]],
  ];
  const pos = [];
  for (const [a, bq, c, d] of quads) pos.push(...a, ...bq, ...c, ...a, ...c, ...d);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pos), 3));
  geo.computeVertexNormals();
  return geo;
}

// --- sponsor wordmarks -------------------------------------------------
const SPONSOR_ROWS = [
  ["APEX", "#ffffff"],
  ["STTR", "#22e0d0"],
  ["EDGE", "#ffffff"],
  ["BOX3D", "#22e0d0"],
];
let sponsorCanvasTex = null;
function getSponsorTexture() {
  if (sponsorCanvasTex || typeof document === "undefined") return sponsorCanvasTex;
  const cv = document.createElement("canvas");
  cv.width = 256;
  cv.height = 256;
  const ctx = cv.getContext("2d");
  if (!ctx) return null;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "bold 42px 'Titillium Web', 'Segoe UI', Arial, sans-serif";
  for (let i = 0; i < SPONSOR_ROWS.length; i++) {
    const [word, fill] = SPONSOR_ROWS[i];
    ctx.save();
    ctx.translate(128, 34 + i * 64);
    ctx.transform(1, 0, -0.16, 1, 0, 0);
    ctx.fillStyle = fill;
    ctx.fillText(word, 0, 0);
    ctx.restore();
  }
  sponsorCanvasTex = new THREE.CanvasTexture(cv);
  sponsorCanvasTex.colorSpace = THREE.SRGBColorSpace;
  sponsorCanvasTex.anisotropy = 4;
  return sponsorCanvasTex;
}

function sponsorMat(row, opacity) {
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
    side: THREE.DoubleSide,
  });
}

/** Racing slick with sculpted sidewall bevel, 5-spoke machined alloy wheel,
 *  cyan sidewall compound ring, center-lock nut, and carbon-ceramic brake disc
 *  whose emissive intensity glows cherry-orange under heavy braking.
 *  Exposed hook: holder.userData.setBrakeGlow(heat01). */
export function buildWheel(opacity = 1) {
  const translucent = opacity < 1;
  const m = (c, extra = {}) =>
    new THREE.MeshStandardMaterial({
      color: c, transparent: translucent, opacity, depthWrite: !translucent, ...extra,
    });
  const holder = new THREE.Group();

  // Main racing slick tread + chamfered shoulders
  const tireMat = m(0x16181e, { metalness: 0.04, roughness: 0.86 });
  const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.33, 0.33, 0.26, 22), tireMat);
  tire.rotation.z = Math.PI / 2;
  const shoulder = new THREE.Mesh(new THREE.CylinderGeometry(0.305, 0.305, 0.305, 22), tireMat);
  shoulder.rotation.z = Math.PI / 2;

  // Inner dark barrel + machined 5-spoke alloy rim
  const barrel = new THREE.Mesh(
    new THREE.CylinderGeometry(0.205, 0.205, 0.295, 18),
    m(0x232832, { metalness: 0.78, roughness: 0.32 }),
  );
  barrel.rotation.z = Math.PI / 2;

  const rimMat = m(0xaeb7c2, { metalness: 0.88, roughness: 0.22 });
  const spokeGeo = new THREE.BoxGeometry(0.312, 0.042, 0.36);
  for (let k = 0; k < 5; k++) {
    const spoke = new THREE.Mesh(spokeGeo, rimMat);
    spoke.rotation.x = (k * Math.PI) / 5;
    holder.add(spoke);
  }
  const hub = new THREE.Mesh(
    new THREE.CylinderGeometry(0.062, 0.062, 0.328, 12),
    m(0x22e0d0, { metalness: 0.72, roughness: 0.26, emissive: CYAN, emissiveIntensity: 0.18 }),
  );
  hub.rotation.z = Math.PI / 2;

  // Cyan slick-compound sidewall stripe
  const ring = new THREE.Mesh(
    new THREE.CylinderGeometry(0.232, 0.232, 0.308, 22),
    m(CYAN, { metalness: 0.45, roughness: 0.32, emissive: CYAN, emissiveIntensity: 0.28 }),
  );
  ring.rotation.z = Math.PI / 2;

  // Carbon-ceramic brake rotor visible through the spokes — heats under braking
  const discMat = m(0x3a3e46, {
    metalness: 0.65, roughness: 0.42, emissive: 0xff3b10, emissiveIntensity: 0.0,
  });
  const disc = new THREE.Mesh(new THREE.CylinderGeometry(0.165, 0.165, 0.04, 16), discMat);
  disc.rotation.z = Math.PI / 2;

  holder.add(tire, shoulder, barrel, ring, hub, disc);
  holder.userData.setBrakeGlow = (heat) => {
    const h = Math.max(0, Math.min(1, heat));
    discMat.emissiveIntensity = h * h * 4.5;
    discMat.emissive.setRGB(1.0, 0.18 + 0.48 * h, 0.03 + 0.12 * h);
  };
  return holder;
}

/** Build the sculpted open-wheeler race car.
 *  Player car exposes live cosmetic hooks on `group.userData`:
 *    • `bodyMeshes` — baked static body meshes for Crumple crash deformation
 *    • `updateTelemetry({ steerAngle, rpm, brake, wetness, speed, timeSec, cameraMode })`
 *    • `updateSuspension(wheelLocalPositions)` — 4 Vector3s in car-local space */
export function buildCar(color, opacity = 1, withWheels = false, deformable = false) {
  const group = new THREE.Group();
  const translucent = opacity < 1;

  // Glossy clearcoat paint catches the procedural PMREM sky & sun reflections!
  const paint = (!translucent && typeof THREE.MeshPhysicalMaterial === "function")
    ? new THREE.MeshPhysicalMaterial({
        color,
        metalness: 0.36,
        roughness: 0.22,
        clearcoat: 0.9,
        clearcoatRoughness: 0.12,
        envMapIntensity: 1.25,
      })
    : new THREE.MeshStandardMaterial({
        color, transparent: translucent, opacity, depthWrite: !translucent,
        metalness: 0.45, roughness: 0.26,
      });

  const mat = (c, extra = {}) =>
    new THREE.MeshStandardMaterial({
      color: c, transparent: translucent, opacity, depthWrite: !translucent, ...extra,
    });
  const carbon = mat(CARBON, { metalness: 0.28, roughness: 0.44, envMapIntensity: 0.9 });
  const accent = mat(CYAN, {
    metalness: 0.42, roughness: 0.28, emissive: CYAN, emissiveIntensity: 0.32,
  });
  const chrome = mat(0xcfd6de, { metalness: 0.92, roughness: 0.16 });
  const brakeLightMat = mat(0x2a0605, {
    emissive: 0xff1e14, emissiveIntensity: 1.0, roughness: 0.25, metalness: 0.1,
  });

  const add = (geo, material, x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0) => {
    const m = new THREE.Mesh(geo, material);
    m.position.set(x, y, z);
    m.rotation.set(rx, ry, rz);
    group.add(m);
    return m;
  };
  const box = (w, h, d) => new THREE.BoxGeometry(w, h, d);
  const wingFaces = [paint, paint, paint, carbon, paint, paint];

  // --- Central carbon floor plank + sculpted bargeboards + rear diffuser strakes
  add(box(1.36, 0.055, 3.6), carbon, 0, -0.155, -0.1);
  for (const s of [-1, 1]) {
    add(box(0.04, 0.14, 0.72), carbon, s * 0.67, -0.09, 0.22); // bargeboard fence
    add(box(0.03, 0.12, 0.42), carbon, s * 0.36, -0.08, -1.96, 0.26, 0, 0); // diffuser strake
  }

  // --- Multi-stage sculpted nose, monocoque tub & tapered engine cover + shark fin
  group.add(new THREE.Mesh(loft(2.22, 1.35, 0.22, 0.44, 0.01, -0.06, 0.12, 0.26), paint)); // slender front nose
  group.add(new THREE.Mesh(loft(1.35, 0.68, 0.44, 0.64, -0.06, -0.12, 0.26, 0.38), paint)); // upper nose bulkhead
  group.add(new THREE.Mesh(loft(0.68, -0.75, 0.64, 0.80, -0.12, -0.15, 0.38, 0.50), carbon)); // carbon monocoque tub
  group.add(new THREE.Mesh(loft(-0.55, -1.98, 0.64, 0.28, -0.15, -0.12, 0.46, 0.18), paint)); // engine cover
  // Dorsal shark-fin spine + roll-hoop airbox intake
  add(box(0.032, 0.34, 1.15), paint, 0, 0.46, -1.22, -0.09, 0, 0);
  add(box(0.036, 0.055, 1.15), accent, 0, 0.61, -1.22, -0.09, 0, 0);
  add(box(0.28, 0.13, 0.14), accent, 0, 0.075, 2.18); // cyan nose tip cap

  // --- Cockpit surround, headrest, airbox & roll-hoop camera T-cam
  add(box(0.52, 0.09, 0.92), carbon, 0, 0.32, 0.05);
  add(box(0.46, 0.21, 0.42), carbon, 0, 0.42, -0.54);
  add(box(0.32, 0.25, 0.52), carbon, 0, 0.44, -0.76);
  add(box(0.24, 0.15, 0.08), mat(0x090a0d, { roughness: 0.9 }), 0, 0.48, -0.49); // airbox inlet mouth
  add(box(0.06, 0.16, 0.06), carbon, 0, 0.65, -0.72); // T-cam stem
  add(box(0.28, 0.045, 0.07), accent, 0, 0.74, -0.72); // onboard T-cam bar

  // --- Titanium Halo hoop + central front pillar
  const haloGeo = new THREE.TorusGeometry(0.335, 0.034, 8, 20);
  haloGeo.rotateX(Math.PI / 2);
  haloGeo.scale(1, 1, 1.26);
  add(haloGeo, carbon, 0, 0.585, 0.02);
  add(new THREE.CylinderGeometry(0.024, 0.042, 0.35, 8), carbon, 0, 0.455, 0.33, 0.48, 0, 0);

  // --- Sculpted tapered sidepods with undercut cooling inlets + aero mirrors
  for (const s of [-1, 1]) {
    group.add(new THREE.Mesh(
      sideLoft(0.44, -1.25, s * 0.46, s * 0.36, 0.36, 0.24, -0.10, -0.12, 0.32, 0.26),
      paint,
    ));
    // Cyan livery flash running cleanly down sidepod shoulder
    group.add(new THREE.Mesh(
      sideLoft(0.45, -1.24, s * 0.46, s * 0.36, 0.37, 0.25, 0.16, 0.10, 0.055, 0.045),
      accent,
    ));
    // Dark recessed radiator mouth
    add(box(0.30, 0.20, 0.07), mat(0x0a0b0e, { roughness: 0.9 }), s * 0.46, 0.04, 0.45);
    // Wing mirror + stay
    add(box(0.16, 0.025, 0.03), carbon, s * 0.44, 0.34, 0.46, 0, 0, s * 0.2);
    add(box(0.15, 0.075, 0.07), paint, s * 0.54, 0.375, 0.48);
  }

  // --- Multi-element front wing + cyan endplates + nose wing pillars
  add(box(1.84, 0.045, 0.48), wingFaces, 0, -0.105, 2.02);
  add(box(1.66, 0.036, 0.20), paint, 0, -0.028, 1.88, 0.26, 0, 0);
  add(box(1.56, 0.032, 0.14), accent, 0, 0.022, 1.79, 0.34, 0, 0);
  for (const s of [-1, 1]) {
    add(box(0.05, 0.21, 0.56), accent, s * 0.90, -0.035, 2.0);
    add(box(0.03, 0.14, 0.16), carbon, s * 0.12, -0.02, 2.0); // front wing swan-neck stays
  }

  // --- Rear wing (dual swan-neck pylons, DRS upper flap, cyan endplates, central exhaust)
  for (const s of [-1, 1]) {
    add(box(0.035, 0.58, 0.16), carbon, s * 0.10, 0.38, -1.88);
    add(box(0.05, 0.46, 0.62), accent, s * 0.72, 0.54, -1.95);
  }
  add(box(1.42, 0.042, 0.42), wingFaces, 0, 0.70, -1.94, -0.16, 0, 0);
  add(box(1.38, 0.034, 0.20), paint, 0, 0.785, -2.06, -0.32, 0, 0); // upper DRS flap
  add(box(1.32, 0.034, 0.24), carbon, 0, 0.46, -1.98, -0.22, 0, 0); // lower beam wing
  // Exhaust tailpipe (metallic chrome ring)
  add(new THREE.CylinderGeometry(0.065, 0.072, 0.24, 12), chrome, 0, 0.18, -2.02, Math.PI / 2, 0, 0);

  // --- Diffuser + dynamic rear brake / FIA rain light + LED nose DRL pods
  add(box(1.04, 0.075, 0.42), carbon, 0, -0.085, -2.02, 0.28, 0, 0);
  add(box(0.11, 0.18, 0.06), brakeLightMat, 0, 0.08, -2.14); // central FIA rain/brake LED array
  add(box(0.46, 0.045, 0.05), brakeLightMat, 0, 0.48, -2.13); // high-mount rear wing LED bar
  for (const s of [-1, 1]) {
    add(box(0.12, 0.055, 0.06), mat(0xfff0d4, { emissive: 0xffe8c0, emissiveIntensity: 0.9 }), s * 0.23, 0.055, 1.56);
  }

  // --- Sponsor decals
  {
    const decal = (row, w, h, x, y, z, rx = 0, ry = 0, rz = 0) => {
      const m = sponsorMat(row, opacity);
      if (m) add(new THREE.PlaneGeometry(w, h), m, x, y, z, rx, ry, rz);
    };
    decal(0, 1.0, 0.24, 0.642, 0.02, -0.35, 0, Math.PI / 2, 0);
    decal(1, 1.0, 0.24, -0.642, 0.02, -0.35, 0, -Math.PI / 2, 0);
    decal(2, 0.44, 0.11, 0, 0.215, 1.32, -Math.PI / 2 + 0.065, 0, 0);
    decal(3, 1.2, 0.28, 0, 0.728, -1.945, -Math.PI / 2 - 0.16, 0, 0);
  }

  // Bake static bodywork BEFORE adding dynamic cockpit/suspension/wheel groups!
  if (deformable) {
    const bodyMeshes = [];
    for (const child of group.children) {
      if (!child.isMesh) continue;
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

  // --- Dynamic driver helmet (hidden automatically in cockpit seat so it never clips camera!)
  const driverGroup = new THREE.Group();
  const helmet = new THREE.Mesh(
    new THREE.SphereGeometry(0.155, 16, 12),
    mat(0xf2f4f8, { metalness: 0.35, roughness: 0.2 }),
  );
  helmet.scale.set(0.95, 1.0, 1.12);
  helmet.position.set(0, 0.44, -0.16);
  const visor = new THREE.Mesh(
    new THREE.SphereGeometry(0.158, 14, 8, -Math.PI * 0.32, Math.PI * 0.64, Math.PI * 0.32, Math.PI * 0.24),
    mat(0x0b1622, { metalness: 0.95, roughness: 0.08, emissive: CYAN, emissiveIntensity: 0.14 }),
  );
  visor.position.set(0, 0.445, -0.15);
  visor.rotation.y = Math.PI / 2;
  driverGroup.add(helmet, visor);
  group.add(driverGroup);

  // --- Dynamic Cockpit Butterfly F1 Steering Wheel + Live Shift-LED Bar
  // Positioned cleanly in the lower center of cockpit view beneath the halo!
  const wheelRig = new THREE.Group();
  wheelRig.position.set(0, 0.62, 0.68);
  wheelRig.rotation.x = -0.32; // tilted comfortably toward driver eye
  const steerWheel = new THREE.Group();
  const swPlate = new THREE.Mesh(box(0.28, 0.14, 0.025), carbon);
  steerWheel.add(swPlate);
  // Left/right ergonomic rubber grips
  const gripMat = mat(0x1f232b, { roughness: 0.92, metalness: 0.05 });
  for (const s of [-1, 1]) {
    const grip = new THREE.Mesh(box(0.042, 0.16, 0.036), gripMat);
    grip.position.set(s * 0.135, -0.005, 0.005);
    steerWheel.add(grip);
  }
  // Tiny luminous cyan telemetry screen in center of steering wheel
  const swScreen = new THREE.Mesh(
    box(0.12, 0.055, 0.008),
    mat(0x091620, { emissive: 0x16808a, emissiveIntensity: 0.75, roughness: 0.2 }),
  );
  swScreen.position.set(0, -0.005, -0.015);
  steerWheel.add(swScreen);
  // 9 progressive RPM shift-LEDs across top rim of steering wheel
  const shiftLeds = [];
  const ledGeo = new THREE.BoxGeometry(0.014, 0.014, 0.01);
  for (let k = 0; k < 9; k++) {
    const col = k < 3 ? 0x22e060 : k < 6 ? 0xff3b2e : CYAN;
    const lm = mat(0x181c22, { emissive: col, emissiveIntensity: 0.05, roughness: 0.2 });
    const led = new THREE.Mesh(ledGeo, lm);
    led.position.set((k - 4) * 0.022, 0.052, -0.015);
    steerWheel.add(led);
    shiftLeds.push({ mat: lm, threshold: 2600 + k * 560 });
  }
  wheelRig.add(steerWheel);
  group.add(wheelRig);

  // --- Articulated Double-Wishbone Suspension Arms (player car only)
  // Connects chassis inner hardpoints to each live physics wheel upright so open
  // wheels are physically tied to the monocoque as suspension compresses!
  const wishbones = [];
  if (!withWheels) {
    const armGeo = new THREE.CylinderGeometry(0.016, 0.018, 1, 6);
    armGeo.rotateX(Math.PI / 2); // align along +Z so lookAt aims straight from A → B
    const armMat = mat(0x1c2028, { metalness: 0.55, roughness: 0.35 });
    // Wheel order matches SimStateV1: 0 FL (-X,+Z), 1 FR (+X,+Z), 2 RL (-X,-Z), 3 RR (+X,-Z)
    const anchors = [
      // Each wheel has upper & lower inner tub anchor points (in car-local space)
      { side: -1, z: 1.30, upper: [-0.26, 0.14, 1.36], lower: [-0.24, -0.10, 1.18] },
      { side: 1, z: 1.30, upper: [0.26, 0.14, 1.36], lower: [0.24, -0.10, 1.18] },
      { side: -1, z: -1.30, upper: [-0.28, 0.12, -1.24], lower: [-0.26, -0.11, -1.42] },
      { side: 1, z: -1.30, upper: [0.28, 0.12, -1.24], lower: [0.26, -0.11, -1.42] },
    ];
    const suspGroup = new THREE.Group();
    for (const a of anchors) {
      const upArm = new THREE.Mesh(armGeo, armMat);
      const loArm = new THREE.Mesh(armGeo, armMat);
      suspGroup.add(upArm, loArm);
      wishbones.push({
        upperMesh: upArm,
        lowerMesh: loArm,
        side: a.side,
        upperIn: new THREE.Vector3(...a.upper),
        lowerIn: new THREE.Vector3(...a.lower),
      });
    }
    group.add(suspGroup);
  }

  const _mid = new THREE.Vector3();
  const _target = new THREE.Vector3();

  /** Update articulated wishbone struts from the 4 wheel centers in car-local space. */
  group.userData.updateSuspension = (wheelLocalPos) => {
    for (let i = 0; i < wishbones.length; i++) {
      const wb = wishbones[i];
      const wp = wheelLocalPos[i];
      // Upper arm: inner tub → upright top (just inboard of rim)
      _target.set(wp.x - wb.side * 0.14, wp.y + 0.11, wp.z);
      _mid.copy(wb.upperIn).add(_target).multiplyScalar(0.5);
      wb.upperMesh.position.copy(_mid);
      wb.upperMesh.lookAt(group.localToWorld(_target.clone()));
      wb.upperMesh.scale.set(1, 1, wb.upperIn.distanceTo(_target));

      // Lower arm: inner tub → upright bottom
      _target.set(wp.x - wb.side * 0.14, wp.y - 0.10, wp.z);
      _mid.copy(wb.lowerIn).add(_target).multiplyScalar(0.5);
      wb.lowerMesh.position.copy(_mid);
      wb.lowerMesh.lookAt(group.localToWorld(_target.clone()));
      wb.lowerMesh.scale.set(1, 1, wb.lowerIn.distanceTo(_target));
    }
  };

  /** Drive cosmetic cockpit & light animation from read-only telemetry. */
  group.userData.updateTelemetry = ({
    steerAngle = 0, rpm = 1200, brake = 0, wetness = 0, timeSec = 0, cameraMode = "chase",
  }) => {
    // Rotate butterfly wheel naturally (~3.2× rack ratio visual)
    steerWheel.rotation.z = steerAngle * 3.2;
    // Hide driver helmet in cockpit seat so camera looks cleanly through the halo
    driverGroup.visible = cameraMode !== "cockpit";
    // Shift LED strip
    const flash = rpm >= 7200 && Math.sin(timeSec * 36) > 0;
    for (let k = 0; k < shiftLeds.length; k++) {
      const l = shiftLeds[k];
      l.mat.emissiveIntensity = flash ? 3.5 : rpm >= l.threshold ? 2.4 : 0.05;
    }
    // Dynamic brake light + FIA wet-weather strobe rain light
    const rainPulse = wetness > 0.35 ? (Math.sin(timeSec * 16) > 0 ? 3.2 : 0.6) : 0.0;
    brakeLightMat.emissiveIntensity = Math.max(0.85, brake * 5.2, rainPulse);
  };

  if (withWheels) {
    for (const [x, z] of [[-0.82, 1.30], [0.82, 1.30], [-0.82, -1.30], [0.82, -1.30]]) {
      const w = buildWheel(opacity);
      w.position.set(x, -0.15, z);
      group.add(w);
    }
  }

  return group;
}

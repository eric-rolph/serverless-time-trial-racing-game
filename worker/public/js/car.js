// Procedural low-poly formula/prototype race car built from three.js
// primitives + lofted BufferGeometry. No assets; ~450 tris for the body,
// ~100 tris per wheel. Footprint ≈ 1.9 m wide × 4.4 m long, matching the
// physics chassis box the old placeholder used (group origin = chassis body
// center; the old box bottom sat at y ≈ -0.2, so the floor plank here sits at
// y ≈ -0.18 and everything stacks up from there). +Z is forward, Y is up.
import * as THREE from "../vendor/three.module.js";

/** Loft a box between two axis-aligned rectangles (front at fz, back at bz,
 *  fz > bz). Non-indexed so computeVertexNormals() yields flat facets — the
 *  whole car reads as chunky folded sheet metal. 12 tris. */
function loft(fz, bz, fw, bw, fBottom, bBottom, fh, bh) {
  const f = [
    [-fw / 2, fBottom, fz], [fw / 2, fBottom, fz],
    [fw / 2, fBottom + fh, fz], [-fw / 2, fBottom + fh, fz],
  ];
  const b = [
    [-bw / 2, bBottom, bz], [bw / 2, bBottom, bz],
    [bw / 2, bBottom + bh, bz], [-bw / 2, bBottom + bh, bz],
  ];
  // Quads wound counter-clockwise seen from outside.
  const quads = [
    [f[0], f[1], f[2], f[3]], // nose face (+z)
    [b[1], b[0], b[3], b[2]], // tail face (−z)
    [f[3], f[2], b[2], b[3]], // top
    [f[1], f[0], b[0], b[1]], // bottom
    [f[1], b[1], b[2], f[2]], // right (+x)
    [f[0], f[3], b[3], b[0]], // left (−x)
  ];
  const pos = [];
  for (const [a, bq, c, d] of quads) pos.push(...a, ...bq, ...c, ...a, ...c, ...d);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pos), 3));
  geo.computeVertexNormals();
  return geo;
}

/** Racing slick: dark tire cylinder + lighter rim disc poking through both
 *  sidewalls. Returned group is the pose holder app.js positions from physics
 *  (rotateY steer, rotateX spin — cylinder axis lies along X). */
export function buildWheel(opacity = 1) {
  const translucent = opacity < 1;
  const m = (c) =>
    new THREE.MeshLambertMaterial({ color: c, transparent: translucent, opacity, depthWrite: !translucent });
  const holder = new THREE.Group();
  const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.33, 0.33, 0.3, 16), m(0x15171c));
  tire.rotation.z = Math.PI / 2;
  const rim = new THREE.Mesh(new THREE.CylinderGeometry(0.19, 0.19, 0.31, 10), m(0x9aa3ad));
  rim.rotation.z = Math.PI / 2;
  holder.add(tire, rim);
  return holder;
}

/** Formula-style car. color = main paint; opacity < 1 → translucent ghost
 *  (transparent + depthWrite:false on EVERY material). withWheels adds four
 *  static slicks for the ghost (the player's wheels are separate physics-posed
 *  meshes — see wheelMeshes in app.js). */
export function buildCar(color, opacity = 1, withWheels = false) {
  const group = new THREE.Group();
  const translucent = opacity < 1;
  const mat = (c, extra = {}) =>
    new THREE.MeshLambertMaterial({
      color: c, transparent: translucent, opacity, depthWrite: !translucent, ...extra,
    });
  const paint = mat(color);          // tone 1: livery
  const dark = mat(0x14181f);        // tone 2: carbon/dark trim
  const accent = mat(0xd8dde6);      // tone 3: wing elements
  const add = (geo, material, x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0) => {
    const m = new THREE.Mesh(geo, material);
    m.position.set(x, y, z);
    m.rotation.set(rx, ry, rz);
    group.add(m);
    return m;
  };
  const box = (w, h, d) => new THREE.BoxGeometry(w, h, d);

  // --- central hull: floor + three lofted sections (nose, tub, engine cover)
  add(box(1.3, 0.06, 3.5), dark, 0, -0.15, -0.1);
  group.add(new THREE.Mesh(loft(2.15, 0.7, 0.26, 0.64, 0.02, -0.12, 0.14, 0.36), paint)); // wedge nose
  group.add(new THREE.Mesh(loft(0.7, -0.75, 0.64, 0.8, -0.12, -0.15, 0.36, 0.5), paint)); // monocoque/tub
  group.add(new THREE.Mesh(loft(-0.55, -2.0, 0.62, 0.3, -0.15, -0.13, 0.44, 0.18), paint)); // engine cover

  // --- open cockpit: dark opening pad, headrest, airbox behind the driver
  add(box(0.5, 0.1, 0.9), dark, 0, 0.33, 0.05);
  add(box(0.46, 0.2, 0.4), dark, 0, 0.42, -0.55);
  add(box(0.3, 0.22, 0.55), dark, 0, 0.4, -0.8);

  // --- halo: horizontal elliptical hoop over the cockpit + forward pylon
  const haloGeo = new THREE.TorusGeometry(0.33, 0.035, 5, 12);
  haloGeo.rotateX(Math.PI / 2);
  haloGeo.scale(1, 1, 1.25);
  add(haloGeo, dark, 0, 0.58, 0.02);
  add(new THREE.CylinderGeometry(0.025, 0.045, 0.34, 6), dark, 0, 0.45, 0.32, 0.5, 0, 0);

  // --- sidepods with dark radiator inlets (car body stays inside x ±0.62 so
  // the open wheels at x ±0.78 stand proud like a proper formula car)
  for (const s of [-1, 1]) {
    add(box(0.36, 0.3, 1.5), paint, s * 0.44, 0.0, -0.35);
    add(box(0.32, 0.22, 0.08), dark, s * 0.44, 0.02, 0.42);
    add(box(0.14, 0.08, 0.06), dark, s * 0.52, 0.38, 0.5); // mirror
  }

  // --- front wing: main plane + tilted flap + endplates (span 1.8 m)
  add(box(1.8, 0.05, 0.5), accent, 0, -0.1, 2.0);
  add(box(1.5, 0.04, 0.22), paint, 0, -0.02, 1.86, 0.25, 0, 0);
  for (const s of [-1, 1]) add(box(0.06, 0.18, 0.55), dark, s * 0.88, -0.04, 2.0);

  // --- rear wing on a central pylon: main plane, lower beam, endplates
  add(box(0.08, 0.6, 0.2), dark, 0, 0.38, -1.88);
  add(box(1.4, 0.04, 0.45), accent, 0, 0.72, -1.95, -0.18, 0, 0);
  add(box(1.3, 0.035, 0.25), paint, 0, 0.5, -2.0, -0.25, 0, 0);
  for (const s of [-1, 1]) add(box(0.05, 0.42, 0.6), dark, s * 0.71, 0.55, -1.95);

  // --- diffuser hint + rain light (emissive — visible at night) + nose pods
  add(box(1.0, 0.08, 0.4), dark, 0, -0.08, -2.0, 0.3, 0, 0);
  add(box(0.4, 0.05, 0.05), mat(0x2a0705, { emissive: 0xff2a1e, emissiveIntensity: 1.2 }), 0, 0.5, -2.13);
  for (const s of [-1, 1])
    add(box(0.1, 0.06, 0.06), mat(0x555049, { emissive: 0xffe9c4, emissiveIntensity: 0.6 }), s * 0.22, 0.06, 1.55);

  // Ghost gets static wheels so the open-wheel silhouette reads; the player
  // car's wheels are separate meshes driven by per-wheel physics state.
  if (withWheels)
    for (const [x, z] of [[-0.78, 1.35], [0.78, 1.35], [-0.78, -1.35], [0.78, -1.35]]) {
      const w = buildWheel(opacity);
      w.position.set(x, -0.15, z);
      group.add(w);
    }

  return group;
}

// Procedural low-poly formula/prototype race car built from three.js
// primitives + lofted BufferGeometry. No assets; ~500 tris for the body,
// ~145 tris per wheel. Footprint ≈ 1.9 m wide × 4.4 m long, matching the
// physics chassis box the old placeholder used (group origin = chassis body
// center; the old box bottom sat at y ≈ -0.2, so the floor plank here sits at
// y ≈ -0.18 and everything stacks up from there). +Z is forward, Y is up.
//
// APEX "kill the box" livery (docs/APEX.md §4): PBR metallic-roughness
// materials — heritage-red paint that catches the sun, near-black carbon
// structure (floor / tub / halo / wing undersides), electric-cyan accent
// stripe (sidepods, nose tip, wing endplates — slightly emissive so it pops
// at night) and procedural canvas sponsor decals. Same geometry budget:
// flat-shaded low-poly aesthetic, materials do the work.
import * as THREE from "../vendor/three.module.js";

// APEX palette (docs/APEX.md design tokens).
const CARBON = 0x151519; // near-black carbon structure
const CYAN = 0x22e0d0; // electric cyan accent

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

// --- sponsor wordmarks -------------------------------------------------
// One shared 256×256 canvas texture, four stacked 256×64 rows of fictional
// wordmarks (Titillium-ish bold sans, white/cyan on transparent). Each decal
// picks a row via texture repeat/offset. Built lazily and cached so player +
// ghost share the same canvas upload; guarded so car.js still parses/loads in
// DOM-less contexts (decals are simply skipped there).
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
    ctx.transform(1, 0, -0.16, 1, 0, 0); // motorsport italic skew
    ctx.fillStyle = fill;
    ctx.fillText(word, 0, 0);
    ctx.restore();
  }
  sponsorCanvasTex = new THREE.CanvasTexture(cv);
  sponsorCanvasTex.colorSpace = THREE.SRGBColorSpace;
  sponsorCanvasTex.anisotropy = 4;
  return sponsorCanvasTex;
}

/** Decal material showing one wordmark row. Always transparent (the canvas
 *  is transparent around the glyphs) and always depthWrite:false — decals
 *  float 2–4 mm off the bodywork so they never fight the paint. Respects the
 *  ghost opacity param like every other car material. Unlit (Basic) so the
 *  wordmarks stay readable day and night. Returns null without a DOM. */
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

/** Racing slick: dark rubber cylinder (rough) + metallic rim disc poking
 *  through both sidewalls + electric-cyan ring accent sandwiched between
 *  sidewall and rim face. Returned group is the pose holder app.js positions
 *  from physics (rotateY steer, rotateX spin — cylinder axis lies along X). */
export function buildWheel(opacity = 1) {
  const translucent = opacity < 1;
  const m = (c, extra = {}) =>
    new THREE.MeshStandardMaterial({
      color: c, transparent: translucent, opacity, depthWrite: !translucent,
      flatShading: true, ...extra,
    });
  const holder = new THREE.Group();
  const tire = new THREE.Mesh(
    new THREE.CylinderGeometry(0.33, 0.33, 0.3, 16),
    m(0x15171c, { metalness: 0.0, roughness: 0.9 }),
  );
  tire.rotation.z = Math.PI / 2;
  const rim = new THREE.Mesh(
    new THREE.CylinderGeometry(0.19, 0.19, 0.31, 10),
    m(0x9aa3ad, { metalness: 0.8, roughness: 0.3 }),
  );
  rim.rotation.z = Math.PI / 2;
  // Cyan rim ring: slightly wider than the rim, slightly shorter than it, so
  // each sidewall shows a cyan annulus between rubber and metal (no coplanar
  // faces — 2.5 mm steps, so no z-fighting).
  const ring = new THREE.Mesh(
    new THREE.CylinderGeometry(0.225, 0.225, 0.305, 10),
    m(CYAN, { metalness: 0.5, roughness: 0.35, emissive: CYAN, emissiveIntensity: 0.25 }),
  );
  ring.rotation.z = Math.PI / 2;
  holder.add(tire, ring, rim);
  return holder;
}

/** Formula-style car. color = main paint; opacity < 1 → translucent ghost
 *  (transparent + depthWrite:false on EVERY material, sponsor decals
 *  included). withWheels adds four static slicks for the ghost (the player's
 *  wheels are separate physics-posed meshes — see wheelMeshes in app.js).
 *
 *  deformable = true bakes every body primitive's local transform into its
 *  geometry (mesh transform reset to identity) so all body vertices live in a
 *  single group-local frame, then publishes the flat list of body meshes on
 *  group.userData.bodyMeshes. crumple.js binds those vertices to a control
 *  lattice for cosmetic crash denting. Purely a mesh/geometry change — visually
 *  identical, no simulation coupling. Default false keeps the old behaviour. */
export function buildCar(color, opacity = 1, withWheels = false, deformable = false) {
  const group = new THREE.Group();
  const translucent = opacity < 1;
  const mat = (c, extra = {}) =>
    new THREE.MeshStandardMaterial({
      color: c, transparent: translucent, opacity, depthWrite: !translucent,
      flatShading: true, ...extra,
    });
  // APEX livery tones (PBR metallic-roughness):
  const paint = mat(color, { metalness: 0.4, roughness: 0.35 }); // heritage-red paint — catches the sun
  const carbon = mat(CARBON, { metalness: 0.2, roughness: 0.55 }); // carbon-fibre structure
  const accent = mat(CYAN, {
    metalness: 0.35, roughness: 0.4, emissive: CYAN, emissiveIntensity: 0.25,
  }); // electric-cyan livery accent, pops at night
  const add = (geo, material, x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0) => {
    const m = new THREE.Mesh(geo, material);
    m.position.set(x, y, z);
    m.rotation.set(rx, ry, rz);
    group.add(m);
    return m;
  };
  const box = (w, h, d) => new THREE.BoxGeometry(w, h, d);
  // BoxGeometry group order is [+x, −x, +y, −y, +z, −z]: paint on top/sides,
  // carbon underside — used for the wing planes.
  const wingFaces = [paint, paint, paint, carbon, paint, paint];

  // --- central hull: floor + three lofted sections (nose, tub, engine cover)
  add(box(1.3, 0.06, 3.5), carbon, 0, -0.15, -0.1); // carbon floor plank
  group.add(new THREE.Mesh(loft(2.15, 0.7, 0.26, 0.64, 0.02, -0.12, 0.14, 0.36), paint)); // wedge nose
  group.add(new THREE.Mesh(loft(0.7, -0.75, 0.64, 0.8, -0.12, -0.15, 0.36, 0.5), carbon)); // exposed-carbon monocoque/tub
  group.add(new THREE.Mesh(loft(-0.55, -2.0, 0.62, 0.3, -0.15, -0.13, 0.44, 0.18), paint)); // engine cover
  add(box(0.28, 0.16, 0.14), accent, 0, 0.09, 2.1); // cyan nose tip cap

  // --- open cockpit: dark opening pad, headrest, airbox behind the driver
  add(box(0.5, 0.1, 0.9), carbon, 0, 0.33, 0.05);
  add(box(0.46, 0.2, 0.4), carbon, 0, 0.42, -0.55);
  add(box(0.3, 0.22, 0.55), carbon, 0, 0.4, -0.8);

  // --- halo: horizontal elliptical hoop over the cockpit + forward pylon
  const haloGeo = new THREE.TorusGeometry(0.33, 0.035, 5, 12);
  haloGeo.rotateX(Math.PI / 2);
  haloGeo.scale(1, 1, 1.25);
  add(haloGeo, carbon, 0, 0.58, 0.02);
  add(new THREE.CylinderGeometry(0.025, 0.045, 0.34, 6), carbon, 0, 0.45, 0.32, 0.5, 0, 0);

  // --- sidepods with dark radiator inlets (car body stays inside x ±0.62 so
  // the open wheels at x ±0.78 stand proud like a proper formula car).
  // Cyan livery stripe runs the length of each sidepod's top edge.
  for (const s of [-1, 1]) {
    add(box(0.36, 0.3, 1.5), paint, s * 0.44, 0.0, -0.35);
    add(box(0.37, 0.06, 1.51), accent, s * 0.44, 0.115, -0.35); // cyan stripe
    add(box(0.32, 0.22, 0.08), carbon, s * 0.44, 0.02, 0.42);
    add(box(0.14, 0.08, 0.06), carbon, s * 0.52, 0.38, 0.5); // mirror
  }

  // --- front wing: main plane (carbon underside) + tilted flap + cyan endplates
  add(box(1.8, 0.05, 0.5), wingFaces, 0, -0.1, 2.0);
  add(box(1.5, 0.04, 0.22), paint, 0, -0.02, 1.86, 0.25, 0, 0);
  for (const s of [-1, 1]) add(box(0.06, 0.18, 0.55), accent, s * 0.88, -0.04, 2.0);

  // --- rear wing on a central pylon: main plane (carbon underside), lower
  // beam, cyan endplates
  add(box(0.08, 0.6, 0.2), carbon, 0, 0.38, -1.88);
  add(box(1.4, 0.04, 0.45), wingFaces, 0, 0.72, -1.95, -0.18, 0, 0);
  add(box(1.3, 0.035, 0.25), paint, 0, 0.5, -2.0, -0.25, 0, 0);
  for (const s of [-1, 1]) add(box(0.05, 0.42, 0.6), accent, s * 0.71, 0.55, -1.95);

  // --- diffuser hint + rain light (emissive — visible at night) + nose pods
  add(box(1.0, 0.08, 0.4), carbon, 0, -0.08, -2.0, 0.3, 0, 0);
  add(box(0.4, 0.05, 0.05), mat(0x2a0705, { emissive: 0xff2a1e, emissiveIntensity: 1.2 }), 0, 0.5, -2.13);
  for (const s of [-1, 1])
    add(box(0.1, 0.06, 0.06), mat(0x555049, { emissive: 0xffe9c4, emissiveIntensity: 0.6 }), s * 0.22, 0.06, 1.55);

  // --- sponsor decals: UV'd planes floating 2–4 mm off the bodywork.
  // Wordmark rows: 0 APEX · 1 STTR · 2 EDGE · 3 BOX3D. Skipped without a DOM.
  {
    const decal = (row, w, h, x, y, z, rx = 0, ry = 0, rz = 0) => {
      const m = sponsorMat(row, opacity);
      if (m) add(new THREE.PlaneGeometry(w, h), m, x, y, z, rx, ry, rz);
    };
    // Sidepod flanks (outer face at x ±0.62): APEX right, STTR left.
    decal(0, 1.0, 0.25, 0.622, -0.02, -0.35, 0, Math.PI / 2, 0);
    decal(1, 1.0, 0.25, -0.622, -0.02, -0.35, 0, -Math.PI / 2, 0);
    // Nose top (follows the wedge slope): EDGE.
    decal(2, 0.44, 0.11, 0, 0.211, 1.3, -Math.PI / 2 + 0.055, 0, 0);
    // Rear wing main-plane top face (tilted with the wing): BOX3D.
    decal(3, 1.2, 0.3, 0, 0.742, -1.954, -Math.PI / 2 - 0.18, 0, 0);
  }

  // Ghost gets static wheels so the open-wheel silhouette reads; the player
  // car's wheels are separate meshes driven by per-wheel physics state.
  if (withWheels)
    for (const [x, z] of [[-0.78, 1.35], [0.78, 1.35], [-0.78, -1.35], [0.78, -1.35]]) {
      const w = buildWheel(opacity);
      w.position.set(x, -0.15, z);
      group.add(w);
    }

  // Deformable player car: bake each body primitive into group space so a
  // single lattice can re-skin every vertex. Sponsor decal planes are baked
  // too, so wordmarks dent with the panel they sit on. Wheel holders are
  // Groups, not Meshes, so they are naturally skipped (and only added when
  // withWheels).
  if (deformable) {
    const bodyMeshes = [];
    for (const child of group.children) {
      if (!child.isMesh) continue;
      child.updateMatrix();
      child.geometry.applyMatrix4(child.matrix); // fold transform into vertices
      child.position.set(0, 0, 0);
      child.rotation.set(0, 0, 0);
      child.scale.set(1, 1, 1);
      child.updateMatrix();
      bodyMeshes.push(child);
    }
    group.userData.bodyMeshes = bodyMeshes;
  }

  return group;
}

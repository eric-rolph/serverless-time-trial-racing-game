//! APEX wgpu + winit Cook-Torrance PBR renderer and interactive race app.
//!
//! Physics cadence lives in `game_loop`; this module draws whatever
//! interpolated pose it is handed at uncapped refresh (`AutoNoVsync`, live
//! aspect ratio — 21:9 / 32:9 native). Features:
//!   • Full-screen physical atmospheric sky dome (Rayleigh/Mie + clouds);
//!   • Cook-Torrance GGX PBR + hemisphere/sky specular IBL + ACES Filmic;
//!   • Procedural world-space asphalt aggregate grain & turf mottling;
//!   • Sculpted 3-material open-wheeler (clearcoat red, woven carbon, cyan
//!     accent) + cylindrical racing slicks + 3D edge lines + striped kerbs;
//!   • Multi-camera seats (`C`: chase / cockpit / bumper) + speed-reactive FOV.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Quat, Vec3};
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

use crate::ffb::{Ffb, FfbConfig};
use crate::game_loop::{interpolate, GameLoop};
use crate::input::{Control, InputSource, KeyboardInput, SelectedInput};
use crate::laplog::LapLog;
use crate::sim::Sim;
use crate::signing::Identity;
use crate::track::Track;

pub const FOV_Y_RADIANS: f32 = 60.0_f32.to_radians();
pub const Z_NEAR: f32 = 0.1;
pub const Z_FAR: f32 = 4000.0;

// Kept public for backward compatibility with unit tests and layout helpers.
#[allow(dead_code)]
pub const CHASSIS_HALF: [f32; 3] = [0.95, 0.6, 2.2];
#[allow(dead_code)]
pub const WHEEL_HALF: [f32; 3] = [0.15, 0.3, 0.3];

// APEX PBR palette: linear RGB + material [roughness, metalness, surface_kind, emissive]
const COLOR_RIBBON: [f32; 4] = [0.14, 0.15, 0.17, 1.0];
const MAT_RIBBON: [f32; 4] = [0.84, 0.04, 1.0, 0.0];

const COLOR_TERRAIN: [f32; 4] = [0.11, 0.18, 0.12, 1.0];
const MAT_TERRAIN: [f32; 4] = [0.95, 0.0, 2.0, 0.0];

const COLOR_EDGE: [f32; 4] = [0.88, 0.91, 0.95, 1.0];
const MAT_EDGE: [f32; 4] = [0.55, 0.0, 0.0, 0.22];

const COLOR_KERB_RED: [f32; 4] = [0.78, 0.11, 0.10, 1.0];
const COLOR_KERB_WHITE: [f32; 4] = [0.88, 0.89, 0.90, 1.0];
const MAT_KERB: [f32; 4] = [0.65, 0.02, 0.0, 0.12];

const COLOR_CHASSIS: [f32; 4] = [0.76, 0.09, 0.07, 1.0];
const MAT_CHASSIS: [f32; 4] = [0.22, 0.42, 0.0, 0.0];

const COLOR_CARBON: [f32; 4] = [0.06, 0.065, 0.08, 1.0];
const MAT_CARBON: [f32; 4] = [0.42, 0.30, 0.0, 0.0];

const COLOR_ACCENT: [f32; 4] = [0.08, 0.82, 0.76, 1.0];
const MAT_ACCENT: [f32; 4] = [0.28, 0.45, 0.0, 0.28];

const COLOR_WHEEL: [f32; 4] = [0.05, 0.055, 0.065, 1.0];
const MAT_WHEEL: [f32; 4] = [0.86, 0.04, 0.0, 0.0];

const COLOR_RIM: [f32; 4] = [0.68, 0.72, 0.78, 1.0];
const MAT_RIM: [f32; 4] = [0.22, 0.88, 0.0, 0.0];

// ---------------------------------------------------------------------------
// CPU mesh generation (pure — unit-testable without a GPU)
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
}

#[derive(Debug, Default, Clone)]
pub struct MeshData {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl MeshData {
    fn append_box(&mut self, center: Vec3, half: Vec3) {
        let faces: [(Vec3, Vec3, Vec3); 6] = [
            (Vec3::X, Vec3::Y, Vec3::Z),
            (-Vec3::X, Vec3::Z, Vec3::Y),
            (Vec3::Y, Vec3::Z, Vec3::X),
            (-Vec3::Y, Vec3::X, Vec3::Z),
            (Vec3::Z, Vec3::X, Vec3::Y),
            (-Vec3::Z, Vec3::Y, Vec3::X),
        ];
        for (n, u, v) in faces {
            let base = self.vertices.len() as u32;
            for (su, sv) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let p = center + (n + u * su + v * sv) * half;
                self.vertices.push(Vertex { pos: p.into(), normal: n.into() });
            }
            self.indices
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }

    fn append_loft(
        &mut self,
        fz: f32,
        bz: f32,
        fw: f32,
        bw: f32,
        f_bot: f32,
        b_bot: f32,
        fh: f32,
        bh: f32,
    ) {
        let f = [
            Vec3::new(-fw * 0.5, f_bot, fz),
            Vec3::new(fw * 0.5, f_bot, fz),
            Vec3::new(fw * 0.5, f_bot + fh, fz),
            Vec3::new(-fw * 0.5, f_bot + fh, fz),
        ];
        let b = [
            Vec3::new(-bw * 0.5, b_bot, bz),
            Vec3::new(bw * 0.5, b_bot, bz),
            Vec3::new(bw * 0.5, b_bot + bh, bz),
            Vec3::new(-bw * 0.5, b_bot + bh, bz),
        ];
        let quads = [
            [f[0], f[1], f[2], f[3]],
            [b[1], b[0], b[3], b[2]],
            [f[3], f[2], b[2], b[3]],
            [f[1], f[0], b[0], b[1]],
            [f[1], b[1], b[2], f[2]],
            [f[0], f[3], b[3], b[0]],
        ];
        for [a, bq, c, d] in quads {
            let n = (bq - a).cross(c - a).normalize_or_zero();
            let base = self.vertices.len() as u32;
            for p in [a, bq, c, d] {
                self.vertices.push(Vertex { pos: p.into(), normal: n.into() });
            }
            self.indices
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }
}

/// Track ribbon: two vertices per centerline sample (left/right edge),
/// normals blended toward the sample's `up`. Closes the loop when the ends
/// meet. Lifted slightly along `up` to avoid z-fighting with the terrain.
pub fn build_ribbon(track: &Track) -> MeshData {
    const LIFT: f32 = 0.03;
    let n = track.centerline.len();
    let mut mesh = MeshData::default();
    if n < 2 {
        return mesh;
    }
    for s in &track.centerline {
        let pos = Vec3::from(s.pos);
        let up = Vec3::from(s.up).normalize_or_zero();
        let tangent = Vec3::from(s.tangent).normalize_or_zero();
        let side = tangent.cross(up).normalize_or_zero();
        let half = s.width * 0.5;
        let l = pos - side * half + up * LIFT;
        let r = pos + side * half + up * LIFT;
        mesh.vertices.push(Vertex { pos: l.into(), normal: up.into() });
        mesh.vertices.push(Vertex { pos: r.into(), normal: up.into() });
    }
    let mut quad = |a: u32, b: u32| {
        mesh.indices
            .extend_from_slice(&[2 * a, 2 * a + 1, 2 * b, 2 * b, 2 * a + 1, 2 * b + 1]);
    };
    for i in 0..(n as u32 - 1) {
        quad(i, i + 1);
    }
    let first = Vec3::from(track.centerline[0].pos);
    let last = Vec3::from(track.centerline[n - 1].pos);
    let avg_spacing = {
        let mut d = 0.0;
        for w in track.centerline.windows(2) {
            d += Vec3::from(w[0].pos).distance(Vec3::from(w[1].pos));
        }
        d / (n - 1) as f32
    };
    if last.distance(first) < avg_spacing * 1.5 {
        quad(n as u32 - 1, 0);
    }
    mesh
}

/// Crisp 22 cm white edge lines + start/finish strip floating just above the ribbon.
pub fn build_edge_lines(track: &Track) -> MeshData {
    const LIFT: f32 = 0.048;
    const LINE_W: f32 = 0.22;
    let n = track.centerline.len();
    let mut mesh = MeshData::default();
    if n < 2 {
        return mesh;
    }
    let is_loop = {
        let first = Vec3::from(track.centerline[0].pos);
        let last = Vec3::from(track.centerline[n - 1].pos);
        let spacing = first.distance(Vec3::from(track.centerline[1].pos)).max(0.1);
        last.distance(first) < spacing * 2.0
    };
    for sign in [-1.0_f32, 1.0_f32] {
        let vbase = mesh.vertices.len() as u32;
        for s in &track.centerline {
            let pos = Vec3::from(s.pos);
            let up = Vec3::from(s.up).normalize_or_zero();
            let tangent = Vec3::from(s.tangent).normalize_or_zero();
            let side = tangent.cross(up).normalize_or_zero();
            let half = s.width * 0.5;
            let inner = pos + side * (sign * (half - LINE_W)) + up * LIFT;
            let outer = pos + side * (sign * half) + up * LIFT;
            mesh.vertices.push(Vertex { pos: inner.into(), normal: up.into() });
            mesh.vertices.push(Vertex { pos: outer.into(), normal: up.into() });
        }
        let segs = if is_loop { n } else { n - 1 };
        for i in 0..segs {
            let j = (i + 1) % n;
            let a = vbase + (2 * i) as u32;
            let b = vbase + (2 * j) as u32;
            mesh.indices.extend_from_slice(&[a, a + 1, b, b, a + 1, b + 1]);
        }
    }
    // Start/finish bar at sample 0
    let s0 = &track.centerline[0];
    let p0 = Vec3::from(s0.pos);
    let u0 = Vec3::from(s0.up).normalize_or_zero();
    let t0 = Vec3::from(s0.tangent).normalize_or_zero();
    let sd0 = t0.cross(u0).normalize_or_zero();
    let hw = s0.width * 0.5 - 0.1;
    let base = mesh.vertices.len() as u32;
    for (sx, sz) in [(-hw, -0.7), (hw, -0.7), (hw, 0.7), (-hw, 0.7)] {
        let p = p0 + sd0 * sx + t0 * sz + u0 * (LIFT + 0.006);
        mesh.vertices.push(Vertex { pos: p.into(), normal: u0.into() });
    }
    mesh.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    mesh
}

/// Striped FIA apex kerbs along tight corners (|smoothed kappa| > 0.022),
/// split into (red_mesh, white_mesh) for clean two-draw coloring.
pub fn build_kerbs(track: &Track) -> (MeshData, MeshData) {
    let n = track.centerline.len();
    let mut red = MeshData::default();
    let mut white = MeshData::default();
    if n < 8 {
        return (red, white);
    }
    let mut raw = vec![0.0_f32; n];
    for i in 0..n {
        let t1 = track.centerline[i].tangent;
        let t2 = track.centerline[(i + 1) % n].tangent;
        raw[i] = (t1[0] * t2[2] - t1[2] * t2[0]) / 2.5;
    }
    let mut kappa = vec![0.0_f32; n];
    for i in 0..n {
        let mut sum = 0.0;
        for k in -7_isize..=7 {
            let idx = (i as isize + k).rem_euclid(n as isize) as usize;
            sum += raw[idx];
        }
        kappa[i] = sum / 15.0;
    }
    const KERB_KAPPA: f32 = 0.022;
    const KERB_W: f32 = 1.1;
    for sign in [-1.0_f32, 1.0_f32] {
        for i in 0..n {
            let j = (i + 1) % n;
            if kappa[i].abs() <= KERB_KAPPA || kappa[j].abs() <= KERB_KAPPA {
                continue;
            }
            let target = if i % 2 == 1 { &mut red } else { &mut white };
            let base = target.vertices.len() as u32;
            for idx in [i, j] {
                let s = &track.centerline[idx];
                let pos = Vec3::from(s.pos);
                let up = Vec3::from(s.up).normalize_or_zero();
                let tan = Vec3::from(s.tangent).normalize_or_zero();
                let sd = tan.cross(up).normalize_or_zero();
                let hw = s.width * 0.5;
                let inner = pos + sd * (sign * hw) + up * 0.045;
                let outer = pos + sd * (sign * (hw + KERB_W)) + up * 0.085;
                target.vertices.push(Vertex { pos: inner.into(), normal: up.into() });
                target.vertices.push(Vertex { pos: outer.into(), normal: up.into() });
            }
            target.indices.extend_from_slice(&[
                base, base + 1, base + 2, base + 1, base + 3, base + 2,
            ]);
        }
    }
    (red, white)
}

/// Terrain: expand the indexed triangle soup to flat-shaded (face-normal)
/// triangles.
pub fn build_terrain(track: &Track) -> MeshData {
    let mut mesh = MeshData::default();
    for tri in &track.terrain_triangles {
        let a = Vec3::from(track.terrain_vertices[tri[0] as usize]);
        let b = Vec3::from(track.terrain_vertices[tri[1] as usize]);
        let c = Vec3::from(track.terrain_vertices[tri[2] as usize]);
        let normal = (b - a).cross(c - a).normalize_or_zero();
        let base = mesh.vertices.len() as u32;
        for p in [a, b, c] {
            mesh.vertices.push(Vertex { pos: p.into(), normal: normal.into() });
        }
        mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }
    mesh
}

/// Axis-aligned box centered at the origin, 24 verts / 36 indices.
pub fn build_box(half: [f32; 3]) -> MeshData {
    let mut mesh = MeshData::default();
    mesh.append_box(Vec3::ZERO, Vec3::from(half));
    mesh
}

/// Sculpted open-wheeler painted bodywork (nose cone, engine cover, sidepods,
/// dorsal shark fin, front & rear wings).
pub fn build_car_body() -> MeshData {
    let mut m = MeshData::default();
    m.append_loft(2.20, 1.32, 0.24, 0.46, 0.01, -0.06, 0.13, 0.26); // front nose
    m.append_loft(1.32, 0.68, 0.46, 0.64, -0.06, -0.12, 0.26, 0.38); // nose bulkhead
    m.append_loft(-0.55, -1.98, 0.64, 0.28, -0.15, -0.12, 0.46, 0.18); // engine cover
    m.append_box(Vec3::new(0.0, 0.46, -1.22), Vec3::new(0.02, 0.17, 0.56)); // shark fin
    // Sidepods
    for s in [-1.0_f32, 1.0_f32] {
        m.append_box(Vec3::new(s * 0.45, 0.02, -0.35), Vec3::new(0.18, 0.15, 0.76));
    }
    // Front & rear wing main planes
    m.append_box(Vec3::new(0.0, -0.10, 2.00), Vec3::new(0.92, 0.025, 0.24));
    m.append_box(Vec3::new(0.0, -0.02, 1.86), Vec3::new(0.80, 0.02, 0.10));
    m.append_box(Vec3::new(0.0, 0.71, -1.95), Vec3::new(0.71, 0.022, 0.21));
    m
}

/// Exposed carbon-fiber structure (floor plank, monocoque tub, halo, diffuser, pylons).
pub fn build_car_carbon() -> MeshData {
    let mut m = MeshData::default();
    m.append_box(Vec3::new(0.0, -0.155, -0.10), Vec3::new(0.68, 0.03, 1.80)); // floor plank
    m.append_loft(0.68, -0.75, 0.64, 0.80, -0.12, -0.15, 0.38, 0.50); // monocoque tub
    m.append_box(Vec3::new(0.0, 0.42, -0.62), Vec3::new(0.22, 0.12, 0.28)); // airbox
    m.append_box(Vec3::new(0.0, 0.58, 0.02), Vec3::new(0.32, 0.03, 0.38)); // halo crown
    m.append_box(Vec3::new(0.0, 0.46, 0.34), Vec3::new(0.03, 0.16, 0.04)); // halo pillar
    m.append_box(Vec3::new(0.0, 0.38, -1.88), Vec3::new(0.05, 0.30, 0.10)); // rear wing pylon
    m.append_box(Vec3::new(0.0, 0.48, -1.98), Vec3::new(0.66, 0.02, 0.12)); // beam wing
    m.append_box(Vec3::new(0.0, -0.08, -2.02), Vec3::new(0.52, 0.04, 0.21)); // diffuser
    m
}

/// Electric-cyan APEX livery accents (nose tip, sidepod flash, wing endplates).
pub fn build_car_accent() -> MeshData {
    let mut m = MeshData::default();
    m.append_box(Vec3::new(0.0, 0.08, 2.18), Vec3::new(0.14, 0.065, 0.07)); // nose tip
    for s in [-1.0_f32, 1.0_f32] {
        m.append_box(Vec3::new(s * 0.45, 0.18, -0.35), Vec3::new(0.185, 0.028, 0.77));
        m.append_box(Vec3::new(s * 0.91, -0.035, 2.00), Vec3::new(0.028, 0.105, 0.28));
        m.append_box(Vec3::new(s * 0.72, 0.54, -1.95), Vec3::new(0.028, 0.23, 0.31));
    }
    m
}

/// Cylindrical racing slick aligned on the X axle (radius 0.33 m, width 0.30 m).
pub fn build_wheel_mesh(radius: f32, half_width: f32, segs: u32) -> MeshData {
    let mut m = MeshData::default();
    let tau = std::f32::consts::TAU;
    // Tread strip
    for i in 0..segs {
        let a0 = (i as f32 / segs as f32) * tau;
        let a1 = ((i + 1) as f32 / segs as f32) * tau;
        let (s0, c0) = a0.sin_cos();
        let (s1, c1) = a1.sin_cos();
        let n0 = Vec3::new(0.0, s0, c0);
        let n1 = Vec3::new(0.0, s1, c1);
        let base = m.vertices.len() as u32;
        m.vertices.push(Vertex { pos: [-half_width, s0 * radius, c0 * radius], normal: n0.into() });
        m.vertices.push(Vertex { pos: [half_width, s0 * radius, c0 * radius], normal: n0.into() });
        m.vertices.push(Vertex { pos: [half_width, s1 * radius, c1 * radius], normal: n1.into() });
        m.vertices.push(Vertex { pos: [-half_width, s1 * radius, c1 * radius], normal: n1.into() });
        m.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    // Left/right sidewalls
    for side in [-1.0_f32, 1.0_f32] {
        let x = side * half_width;
        let n = [side, 0.0, 0.0];
        let center = m.vertices.len() as u32;
        m.vertices.push(Vertex { pos: [x, 0.0, 0.0], normal: n });
        for i in 0..=segs {
            let a = (i as f32 / segs as f32) * tau;
            let (s, c) = a.sin_cos();
            m.vertices.push(Vertex { pos: [x, s * radius, c * radius], normal: n });
        }
        for i in 0..segs {
            let v1 = center + 1 + i;
            let v2 = center + 2 + i;
            if side > 0.0 {
                m.indices.extend_from_slice(&[center, v1, v2]);
            } else {
                m.indices.extend_from_slice(&[center, v2, v1]);
            }
        }
    }
    m
}

// ---------------------------------------------------------------------------
// Multi-seat camera (Chase / Cockpit / Bumper) + speed-reactive FOV
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraSeat {
    Chase,
    Cockpit,
    Bumper,
}

impl CameraSeat {
    pub fn next(self) -> CameraSeat {
        match self {
            CameraSeat::Chase => CameraSeat::Cockpit,
            CameraSeat::Cockpit => CameraSeat::Bumper,
            CameraSeat::Bumper => CameraSeat::Chase,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            CameraSeat::Chase => "chase",
            CameraSeat::Cockpit => "cockpit",
            CameraSeat::Bumper => "bumper",
        }
    }
}

pub struct FollowCam {
    pos: Option<Vec3>,
    pub seat: CameraSeat,
    fov_rad: f32,
}

impl FollowCam {
    pub fn new() -> FollowCam {
        FollowCam { pos: None, seat: CameraSeat::Chase, fov_rad: FOV_Y_RADIANS }
    }

    pub fn cycle_seat(&mut self) {
        self.seat = self.seat.next();
        self.pos = None;
    }

    pub fn view_and_eye(
        &mut self,
        car_pos: Vec3,
        car_quat: Quat,
        speed_mps: f32,
        dt: f32,
    ) -> (Mat4, Vec3, f32) {
        let dt_c = dt.clamp(0.0, 0.1);
        let speed_frac = (speed_mps / 60.0).clamp(0.0, 1.0);
        let fov_target = match self.seat {
            CameraSeat::Cockpit => (68.0 + 4.0 * speed_frac).to_radians(),
            CameraSeat::Chase | CameraSeat::Bumper => (60.0 + 8.0 * speed_frac).to_radians(),
        };
        let fk = 1.0 - (-dt_c * 3.0).exp();
        self.fov_rad += (fov_target - self.fov_rad) * fk;

        match self.seat {
            CameraSeat::Chase => {
                let forward = car_quat * Vec3::Z;
                let flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or(Vec3::Z);
                let desired = car_pos - flat * 8.6 + Vec3::Y * 3.15;
                let eye = match self.pos {
                    None => desired,
                    Some(p) => {
                        let k = 1.0 - (-dt_c * 5.0).exp();
                        p + (desired - p) * k
                    }
                };
                self.pos = Some(eye);
                (Mat4::look_at_rh(eye, car_pos + Vec3::Y * 1.05, Vec3::Y), eye, self.fov_rad)
            }
            CameraSeat::Cockpit => {
                let eye = car_pos + car_quat * Vec3::new(0.0, 0.94, 0.24);
                let ahead = car_pos + car_quat * Vec3::new(0.0, 0.96, 30.0);
                let up = (car_quat * Vec3::Y).normalize_or(Vec3::Y);
                self.pos = Some(eye);
                (Mat4::look_at_rh(eye, ahead, up), eye, self.fov_rad)
            }
            CameraSeat::Bumper => {
                let eye = car_pos + car_quat * Vec3::new(0.0, 0.36, 2.30);
                let ahead = car_pos + car_quat * Vec3::new(0.0, 0.45, 32.0);
                let up = (car_quat * Vec3::Y).normalize_or(Vec3::Y);
                self.pos = Some(eye);
                (Mat4::look_at_rh(eye, ahead, up), eye, self.fov_rad)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// GPU state
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GlobalsU {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    cam_pos: [f32; 4],
    light_dir: [f32; 4],
    sky_top: [f32; 4],
    sky_horizon: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ObjectU {
    model: [[f32; 4]; 4],
    color: [f32; 4],
    material: [f32; 4],
}

const OBJECT_STRIDE: u64 = 256;
const MAX_OBJECTS: usize = 32;

struct GpuMesh {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
}

struct DrawItem {
    mesh: usize,
    model: Mat4,
    color: [f32; 4],
    material: [f32; 4],
}

pub struct Gfx {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth_view: wgpu::TextureView,
    sky_pipeline: wgpu::RenderPipeline,
    pipeline: wgpu::RenderPipeline,
    globals_buf: wgpu::Buffer,
    globals_bg: wgpu::BindGroup,
    object_buf: wgpu::Buffer,
    object_bg: wgpu::BindGroup,
    meshes: Vec<GpuMesh>,
    started: Instant,
}

impl Gfx {
    pub fn new(window: Arc<Window>, cpu_meshes: &[MeshData]) -> Result<Gfx> {
        pollster::block_on(Self::new_async(window, cpu_meshes))
    }

    async fn new_async(window: Arc<Window>, cpu_meshes: &[MeshData]) -> Result<Gfx> {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
        let surface = instance
            .create_surface(window.clone())
            .context("creating wgpu surface")?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .context("no compatible GPU adapter found")?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("sttr-device"),
                ..Default::default()
            })
            .await
            .context("requesting GPU device")?;

        let mut config = surface
            .get_default_config(&adapter, size.width.max(1), size.height.max(1))
            .ok_or_else(|| anyhow!("surface not supported by adapter"))?;
        config.present_mode = wgpu::PresentMode::AutoNoVsync;
        surface.configure(&device, &config);

        let depth_view = create_depth(&device, config.width, config.height);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("sttr-pbr-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let globals_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals-layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<GlobalsU>() as u64),
                },
                count: None,
            }],
        });
        let object_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("object-layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<ObjectU>() as u64),
                },
                count: None,
            }],
        });

        let globals_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<GlobalsU>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let object_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("objects"),
            size: OBJECT_STRIDE * MAX_OBJECTS as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let globals_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("globals-bg"),
            layout: &globals_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: globals_buf.as_entire_binding(),
            }],
        });
        let object_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("object-bg"),
            layout: &object_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &object_buf,
                    offset: 0,
                    size: wgpu::BufferSize::new(std::mem::size_of::<ObjectU>() as u64),
                }),
            }],
        });

        let sky_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("sky-pipeline-layout"),
            bind_group_layouts: &[&globals_layout],
            push_constant_ranges: &[],
        });
        let sky_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("sky-pipeline"),
            layout: Some(&sky_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_sky"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_sky"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("sttr-pipeline-layout"),
            bind_group_layouts: &[&globals_layout, &object_layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("sttr-pbr-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        let meshes = cpu_meshes.iter().map(|m| upload_mesh(&device, m)).collect();

        Ok(Gfx {
            surface,
            device,
            queue,
            config,
            depth_view,
            sky_pipeline,
            pipeline,
            globals_buf,
            globals_bg,
            object_buf,
            object_bg,
            meshes,
            started: Instant::now(),
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);
        self.depth_view = create_depth(&self.device, width, height);
    }

    pub fn aspect(&self) -> f32 {
        self.config.width as f32 / self.config.height.max(1) as f32
    }

    fn render(&mut self, view: Mat4, eye: Vec3, fov_rad: f32, items: &[DrawItem]) -> Result<()> {
        assert!(items.len() <= MAX_OBJECTS);
        let proj = Mat4::perspective_rh(fov_rad, self.aspect(), Z_NEAR, Z_FAR);
        let vp = proj * view;
        let inv_vp = vp.inverse();
        let t = self.started.elapsed().as_secs_f32();

        // Warm golden-hour APEX sun rig (matches web default)
        let sun_dir = Vec3::new(-0.82, -0.32, -0.48).normalize();
        let globals = GlobalsU {
            view_proj: vp.to_cols_array_2d(),
            inv_view_proj: inv_vp.to_cols_array_2d(),
            cam_pos: [eye.x, eye.y, eye.z, t],
            light_dir: [sun_dir.x, sun_dir.y, sun_dir.z, 3.2],
            sky_top: [0.10, 0.24, 0.56, 120.0],     // zenith blue + fog_near
            sky_horizon: [0.85, 0.56, 0.34, 520.0], // warm horizon + fog_far
        };
        self.queue
            .write_buffer(&self.globals_buf, 0, bytemuck::bytes_of(&globals));

        let mut obj_bytes = vec![0u8; OBJECT_STRIDE as usize * items.len()];
        for (i, item) in items.iter().enumerate() {
            let u = ObjectU {
                model: item.model.to_cols_array_2d(),
                color: item.color,
                material: item.material,
            };
            let off = i * OBJECT_STRIDE as usize;
            obj_bytes[off..off + std::mem::size_of::<ObjectU>()]
                .copy_from_slice(bytemuck::bytes_of(&u));
        }
        self.queue.write_buffer(&self.object_buf, 0, &obj_bytes);

        let frame = match self.surface.get_current_texture() {
            Ok(f) => f,
            Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                self.surface.configure(&self.device, &self.config);
                self.surface
                    .get_current_texture()
                    .context("surface unrecoverable after reconfigure")?
            }
            Err(e) => return Err(anyhow!("surface error: {e}")),
        };
        let view_tex = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("sttr-enc") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view_tex,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.14,
                            g: 0.18,
                            b: 0.28,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            // 1. Physical sky dome pass
            pass.set_pipeline(&self.sky_pipeline);
            pass.set_bind_group(0, &self.globals_bg, &[]);
            pass.draw(0..3, 0..1);

            // 2. Cook-Torrance PBR world pass
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.globals_bg, &[]);
            for (i, item) in items.iter().enumerate() {
                let mesh = &self.meshes[item.mesh];
                if mesh.index_count == 0 {
                    continue;
                }
                pass.set_bind_group(1, &self.object_bg, &[(i as u32) * OBJECT_STRIDE as u32]);
                pass.set_vertex_buffer(0, mesh.vbuf.slice(..));
                pass.set_index_buffer(mesh.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..mesh.index_count, 0, 0..1);
            }
        }
        self.queue.submit([encoder.finish()]);
        frame.present();
        Ok(())
    }
}

fn create_depth(device: &wgpu::Device, width: u32, height: u32) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&wgpu::TextureViewDescriptor::default())
}

fn upload_mesh(device: &wgpu::Device, mesh: &MeshData) -> GpuMesh {
    use wgpu::util::DeviceExt;
    let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("vbuf"),
        contents: bytemuck::cast_slice(&mesh.vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("ibuf"),
        contents: bytemuck::cast_slice(&mesh.indices),
        usage: wgpu::BufferUsages::INDEX,
    });
    GpuMesh { vbuf, ibuf, index_count: mesh.indices.len() as u32 }
}

// ---------------------------------------------------------------------------
// Race application (winit event loop driver)
// ---------------------------------------------------------------------------

pub struct RaceContext {
    pub sim: Sim,
    pub track: Track,
    pub input: SelectedInput,
    pub name: String,
    pub submit_to: Option<String>,
    pub out_path: PathBuf,
    pub ffb: FfbConfig,
}

const MESH_RIBBON: usize = 0;
const MESH_TERRAIN: usize = 1;
const MESH_EDGES: usize = 2;
const MESH_KERB_RED: usize = 3;
const MESH_KERB_WHITE: usize = 4;
const MESH_CAR_BODY: usize = 5;
const MESH_CAR_CARBON: usize = 6;
const MESH_CAR_ACCENT: usize = 7;
const MESH_WHEEL_TIRE: usize = 8;
const MESH_WHEEL_RIM: usize = 9;

struct RaceApp {
    ctx: RaceContext,
    game: GameLoop,
    cam: FollowCam,
    window: Option<Arc<Window>>,
    gfx: Option<Gfx>,
    ffb: Option<Ffb>,
    fatal: Option<anyhow::Error>,
    last_render: Option<Instant>,
    last_title: Instant,
    lap_result: Option<u32>,
}

impl RaceApp {
    fn keyboard(&self) -> Option<&Arc<KeyboardInput>> {
        match &self.ctx.input {
            SelectedInput::Keyboard(kb) => Some(kb),
            SelectedInput::Wheel(_) => None,
        }
    }

    fn fail(&mut self, event_loop: &ActiveEventLoop, err: anyhow::Error) {
        eprintln!("fatal: {err:#}");
        self.fatal = Some(err);
        event_loop.exit();
    }

    fn on_key(&mut self, event_loop: &ActiveEventLoop, event: &KeyEvent) {
        let PhysicalKey::Code(code) = event.physical_key else { return };
        let pressed = event.state == ElementState::Pressed;
        if code == KeyCode::Escape && pressed {
            event_loop.exit();
            return;
        }
        if code == KeyCode::KeyC && pressed && !event.repeat {
            self.cam.cycle_seat();
            return;
        }
        let control = match code {
            KeyCode::KeyA | KeyCode::ArrowLeft => Some(Control::SteerLeft),
            KeyCode::KeyD | KeyCode::ArrowRight => Some(Control::SteerRight),
            KeyCode::KeyW | KeyCode::ArrowUp => Some(Control::Throttle),
            KeyCode::KeyS | KeyCode::ArrowDown => Some(Control::Brake),
            KeyCode::Space => Some(Control::Handbrake),
            _ => None,
        };
        if let (Some(c), Some(kb)) = (control, self.keyboard()) {
            kb.set_control(c, pressed);
        }
    }

    fn frame(&mut self) -> Result<()> {
        let input: &dyn InputSource = self.ctx.input.source();
        let fr = self.game.frame(&mut self.ctx.sim, input)?;

        if fr.lap_just_completed {
            self.on_lap_complete()?;
        }

        let pose = interpolate(&self.game.prev, &self.game.curr, fr.alpha);
        let now = Instant::now();
        let rdt = self
            .last_render
            .map(|l| now.duration_since(l).as_secs_f32())
            .unwrap_or(1.0 / 60.0);
        self.last_render = Some(now);

        if let Some(ffb) = &mut self.ffb {
            let torque = self.ctx.sim.ffb_torque();
            ffb.frame_with_speed(torque, pose.speed, rdt);
        }

        let car_pos = Vec3::from(pose.pos);
        let car_quat =
            Quat::from_xyzw(pose.quat[0], pose.quat[1], pose.quat[2], pose.quat[3]).normalize();
        let (view, eye, fov_rad) = self.cam.view_and_eye(car_pos, car_quat, pose.speed, rdt);

        let car_model = Mat4::from_rotation_translation(car_quat, car_pos);
        let mut items = vec![
            DrawItem { mesh: MESH_RIBBON, model: Mat4::IDENTITY, color: COLOR_RIBBON, material: MAT_RIBBON },
            DrawItem { mesh: MESH_TERRAIN, model: Mat4::IDENTITY, color: COLOR_TERRAIN, material: MAT_TERRAIN },
            DrawItem { mesh: MESH_EDGES, model: Mat4::IDENTITY, color: COLOR_EDGE, material: MAT_EDGE },
            DrawItem { mesh: MESH_KERB_RED, model: Mat4::IDENTITY, color: COLOR_KERB_RED, material: MAT_KERB },
            DrawItem { mesh: MESH_KERB_WHITE, model: Mat4::IDENTITY, color: COLOR_KERB_WHITE, material: MAT_KERB },
            DrawItem { mesh: MESH_CAR_BODY, model: car_model, color: COLOR_CHASSIS, material: MAT_CHASSIS },
            DrawItem { mesh: MESH_CAR_CARBON, model: car_model, color: COLOR_CARBON, material: MAT_CARBON },
            DrawItem { mesh: MESH_CAR_ACCENT, model: car_model, color: COLOR_ACCENT, material: MAT_ACCENT },
        ];
        for w in &pose.wheels {
            let rot = car_quat
                * Quat::from_rotation_y(w.steer_angle)
                * Quat::from_rotation_x(w.spin_angle);
            let wm = Mat4::from_rotation_translation(rot, Vec3::from(w.pos));
            items.push(DrawItem { mesh: MESH_WHEEL_TIRE, model: wm, color: COLOR_WHEEL, material: MAT_WHEEL });
            items.push(DrawItem { mesh: MESH_WHEEL_RIM, model: wm, color: COLOR_RIM, material: MAT_RIM });
        }

        if let Some(gfx) = &mut self.gfx {
            gfx.render(view, eye, fov_rad, &items)?;
        }

        if now.duration_since(self.last_title) > Duration::from_millis(200) {
            self.last_title = now;
            if let Some(win) = &self.window {
                let kmh = pose.speed * 3.6;
                let seat = self.cam.seat.label();
                let title = match self.lap_result {
                    Some(ticks) => format!(
                        "sttr-client [APEX] — LAP {:.3}s — {:.0} km/h — cam:{seat} (C)",
                        ticks as f64 * crate::game_loop::DT,
                        kmh
                    ),
                    None => format!(
                        "sttr-client [APEX] — {:.0} km/h — lap {:.0}% — cam:{seat} (C)",
                        kmh,
                        pose.lap_progress * 100.0
                    ),
                };
                win.set_title(&title);
            }
        }
        Ok(())
    }

    fn on_lap_complete(&mut self) -> Result<()> {
        use crate::sim::SimBackend;

        let final_state_hash = self.ctx.sim.state_hash()?;
        let claimed_lap_ticks = self.ctx.sim.lap_time_ticks()?;
        self.lap_result = Some(claimed_lap_ticks);

        let lap_secs = claimed_lap_ticks as f64 * crate::game_loop::DT;
        println!("LAP COMPLETE: {lap_secs:.3} s ({claimed_lap_ticks} ticks)");
        println!("final state hash: {final_state_hash:016x}");

        let log = LapLog {
            track_hash: self.ctx.track.hash,
            ticks: self.game.log.clone(),
            final_state_hash,
            claimed_lap_ticks,
        };
        let bytes = log.serialize();
        std::fs::write(&self.ctx.out_path, &bytes)
            .with_context(|| format!("writing '{}'", self.ctx.out_path.display()))?;
        println!("wrote {} ({} bytes)", self.ctx.out_path.display(), bytes.len());

        let identity = Identity::load_or_generate()?;
        let sig = identity.sign_laplog(&bytes);
        let pubkey = identity.public_key_bytes();

        if let Some(url) = self.ctx.submit_to.clone() {
            let name = self.ctx.name.clone();
            std::thread::spawn(move || {
                println!("submitting to {url} …");
                match crate::submit::submit_blocking(&url, &pubkey, &sig, &name, &bytes) {
                    Ok(outcome) => println!("server verdict: {outcome}"),
                    Err(e) => eprintln!("submit failed: {e:#}"),
                }
            });
        } else {
            println!("(no --submit-to given; lap saved locally only)");
        }
        Ok(())
    }
}

impl ApplicationHandler for RaceApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attrs = Window::default_attributes()
            .with_title("sttr-client [APEX]")
            .with_inner_size(winit::dpi::LogicalSize::new(1600.0, 900.0));
        let window = match event_loop.create_window(attrs) {
            Ok(w) => Arc::new(w),
            Err(e) => return self.fail(event_loop, anyhow!("window creation failed: {e}")),
        };

        let (kerb_red, kerb_white) = build_kerbs(&self.ctx.track);
        let cpu_meshes = [
            build_ribbon(&self.ctx.track),
            build_terrain(&self.ctx.track),
            build_edge_lines(&self.ctx.track),
            kerb_red,
            kerb_white,
            build_car_body(),
            build_car_carbon(),
            build_car_accent(),
            build_wheel_mesh(0.33, 0.15, 18),
            build_wheel_mesh(0.20, 0.158, 12),
        ];
        match Gfx::new(window.clone(), &cpu_meshes) {
            Ok(g) => self.gfx = Some(g),
            Err(e) => return self.fail(event_loop, e),
        }
        self.ffb = Some(Ffb::new(self.ctx.ffb.clone(), &window));
        self.window = Some(window);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(gfx) = &mut self.gfx {
                    gfx.resize(size.width, size.height);
                }
            }
            WindowEvent::KeyboardInput { event, .. } => self.on_key(event_loop, &event),
            WindowEvent::RedrawRequested => {
                if let Err(e) = self.frame() {
                    return self.fail(event_loop, e);
                }
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }
}

pub fn run_race(mut ctx: RaceContext) -> Result<()> {
    use crate::sim::SimBackend;
    let initial = ctx.sim.state()?;

    let event_loop = EventLoop::new().context("creating event loop")?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = RaceApp {
        ctx,
        game: GameLoop::new(initial),
        cam: FollowCam::new(),
        window: None,
        gfx: None,
        ffb: None,
        fatal: None,
        last_render: None,
        last_title: Instant::now(),
        lap_result: None,
    };
    event_loop.run_app(&mut app).context("event loop error")?;
    if let Some(e) = app.fatal.take() {
        return Err(e);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::Track;

    fn track() -> Track {
        Track::parse(&crate::track::synthetic_track_bytes()).unwrap()
    }

    #[test]
    fn ribbon_has_two_verts_per_sample() {
        let t = track();
        let m = build_ribbon(&t);
        assert_eq!(m.vertices.len(), t.centerline.len() * 2);
        assert_eq!(m.indices.len(), 2 * 6);
        let l = Vec3::from(m.vertices[0].pos);
        let r = Vec3::from(m.vertices[1].pos);
        assert!((l.distance(r) - 8.0).abs() < 1e-4);
        assert!(m.indices.iter().all(|&i| (i as usize) < m.vertices.len()));
    }

    #[test]
    fn terrain_is_flat_shaded_triangles() {
        let t = track();
        let m = build_terrain(&t);
        assert_eq!(m.vertices.len(), t.terrain_triangles.len() * 3);
        assert_eq!(m.indices.len(), t.terrain_triangles.len() * 3);
        let n = Vec3::from(m.vertices[0].normal);
        assert!((n.abs() - Vec3::Y).length() < 1e-5, "normal = {n:?}");
    }

    #[test]
    fn box_mesh_shape() {
        let m = build_box([1.0, 2.0, 3.0]);
        assert_eq!(m.vertices.len(), 24);
        assert_eq!(m.indices.len(), 36);
        for v in &m.vertices {
            assert!(v.pos[0].abs() <= 1.0 + 1e-6);
            assert!(v.pos[1].abs() <= 2.0 + 1e-6);
            assert!(v.pos[2].abs() <= 3.0 + 1e-6);
            let n = Vec3::from(v.normal);
            assert!((n.length() - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn sculpted_car_and_wheel_meshes_have_valid_indices_and_normals() {
        for mesh in [
            build_car_body(),
            build_car_carbon(),
            build_car_accent(),
            build_wheel_mesh(0.33, 0.15, 18),
            build_edge_lines(&track()),
        ] {
            assert!(!mesh.vertices.is_empty());
            assert!(!mesh.indices.is_empty());
            assert!(mesh.indices.iter().all(|&i| (i as usize) < mesh.vertices.len()));
            for v in &mesh.vertices {
                let n = Vec3::from(v.normal);
                assert!((n.length() - 1.0).abs() < 1e-4, "unnormalized normal: {n:?}");
            }
        }
    }

    #[test]
    fn camera_seats_cycle_and_widen_fov_with_speed() {
        let mut cam = FollowCam::new();
        assert_eq!(cam.seat, CameraSeat::Chase);
        let (_, _, fov_slow) = cam.view_and_eye(Vec3::ZERO, Quat::IDENTITY, 0.0, 0.5);
        let (_, _, fov_fast) = cam.view_and_eye(Vec3::ZERO, Quat::IDENTITY, 60.0, 0.5);
        assert!(fov_fast > fov_slow);
        cam.cycle_seat();
        assert_eq!(cam.seat, CameraSeat::Cockpit);
        cam.cycle_seat();
        assert_eq!(cam.seat, CameraSeat::Bumper);
        cam.cycle_seat();
        assert_eq!(cam.seat, CameraSeat::Chase);
    }
}

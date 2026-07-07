//! wgpu + winit flat-shaded renderer and the interactive race application.
//!
//! Physics cadence lives in `game_loop`; this module only draws whatever
//! interpolated pose it is handed, as fast as the surface lets it
//! (PresentMode::AutoNoVsync, live aspect ratio — ultrawide is native).

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

use crate::game_loop::{interpolate, GameLoop};
use crate::input::{Control, InputSource, KeyboardInput, SelectedInput};
use crate::laplog::LapLog;
use crate::sim::Sim;
use crate::signing::Identity;
use crate::track::Track;

pub const FOV_Y_RADIANS: f32 = 60.0_f32.to_radians();
pub const Z_NEAR: f32 = 0.1;
pub const Z_FAR: f32 = 4000.0;

// Car visual dimensions (m): 1.9 wide x 1.2 high x 4.4 long, length on Z.
pub const CHASSIS_HALF: [f32; 3] = [0.95, 0.6, 2.2];
pub const WHEEL_HALF: [f32; 3] = [0.15, 0.3, 0.3];

const COLOR_RIBBON: [f32; 4] = [0.28, 0.28, 0.30, 1.0];
const COLOR_TERRAIN: [f32; 4] = [0.42, 0.42, 0.42, 1.0];
const COLOR_CHASSIS: [f32; 4] = [0.85, 0.18, 0.12, 1.0];
const COLOR_WHEEL: [f32; 4] = [0.10, 0.10, 0.12, 1.0];

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
    // segment quads
    let mut quad = |a: u32, b: u32| {
        // a = sample i, b = sample i+1 (vertex pairs 2a,2a+1 and 2b,2b+1)
        mesh.indices.extend_from_slice(&[2 * a, 2 * a + 1, 2 * b, 2 * b, 2 * a + 1, 2 * b + 1]);
    };
    for i in 0..(n as u32 - 1) {
        quad(i, i + 1);
    }
    // Close the loop if the ends are near each other. Closed circuits end one
    // sample-spacing from their start; an open polyline's end gap is much
    // larger — 1.5x average spacing separates the two cleanly.
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
    let [hx, hy, hz] = half;
    let faces: [(Vec3, Vec3, Vec3); 6] = [
        (Vec3::X, Vec3::Y, Vec3::Z),
        (-Vec3::X, Vec3::Z, Vec3::Y),
        (Vec3::Y, Vec3::Z, Vec3::X),
        (-Vec3::Y, Vec3::X, Vec3::Z),
        (Vec3::Z, Vec3::X, Vec3::Y),
        (-Vec3::Z, Vec3::Y, Vec3::X),
    ];
    let he = Vec3::new(hx, hy, hz);
    let mut mesh = MeshData::default();
    for (n, u, v) in faces {
        let base = mesh.vertices.len() as u32;
        for (su, sv) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            let p = (n + u * su + v * sv) * he;
            mesh.vertices.push(Vertex { pos: p.into(), normal: n.into() });
        }
        mesh.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    mesh
}

// ---------------------------------------------------------------------------
// Follow camera
// ---------------------------------------------------------------------------

/// Exponentially damped spring toward a point behind and above the car.
pub struct FollowCam {
    pos: Option<Vec3>,
}

impl FollowCam {
    pub fn new() -> FollowCam {
        FollowCam { pos: None }
    }

    pub fn view(&mut self, car_pos: Vec3, car_quat: Quat, dt: f32) -> Mat4 {
        // NOTE: assumes car forward = +Z in chassis space (see NOTES.md).
        let forward = car_quat * Vec3::Z;
        let flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or(Vec3::Z);
        let desired = car_pos - flat * 10.0 + Vec3::Y * 3.8;
        let pos = match self.pos {
            None => desired,
            Some(p) => {
                let k = 1.0 - (-dt.clamp(0.0, 0.1) * 5.0).exp();
                p + (desired - p) * k
            }
        };
        self.pos = Some(pos);
        Mat4::look_at_rh(pos, car_pos + Vec3::Y * 0.8, Vec3::Y)
    }
}

// ---------------------------------------------------------------------------
// GPU state
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GlobalsU {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ObjectU {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

const OBJECT_STRIDE: u64 = 256; // >= device min_uniform_buffer_offset_alignment
const MAX_OBJECTS: usize = 16;

struct GpuMesh {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    index_count: u32,
}

struct DrawItem {
    mesh: usize,
    model: Mat4,
    color: [f32; 4],
}

pub struct Gfx {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth_view: wgpu::TextureView,
    pipeline: wgpu::RenderPipeline,
    globals_buf: wgpu::Buffer,
    globals_bg: wgpu::BindGroup,
    object_buf: wgpu::Buffer,
    object_bg: wgpu::BindGroup,
    meshes: Vec<GpuMesh>,
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
        // Uncapped fps for high-refresh displays.
        config.present_mode = wgpu::PresentMode::AutoNoVsync;
        surface.configure(&device, &config);

        let depth_view = create_depth(&device, config.width, config.height);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("sttr-shader"),
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

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("sttr-pipeline-layout"),
            bind_group_layouts: &[&globals_layout, &object_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("sttr-pipeline"),
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
                cull_mode: None, // ribbon/terrain winding is data-dependent
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

        let meshes = cpu_meshes
            .iter()
            .map(|m| upload_mesh(&device, m))
            .collect();

        Ok(Gfx {
            surface,
            device,
            queue,
            config,
            depth_view,
            pipeline,
            globals_buf,
            globals_bg,
            object_buf,
            object_bg,
            meshes,
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

    /// Live surface aspect ratio — 21:9 / 32:9 render natively, no letterbox.
    pub fn aspect(&self) -> f32 {
        self.config.width as f32 / self.config.height.max(1) as f32
    }

    fn render(&mut self, view: Mat4, items: &[DrawItem]) -> Result<()> {
        assert!(items.len() <= MAX_OBJECTS);
        let proj = Mat4::perspective_rh(FOV_Y_RADIANS, self.aspect(), Z_NEAR, Z_FAR);
        let globals = GlobalsU {
            view_proj: (proj * view).to_cols_array_2d(),
            light_dir: [0.35, -0.85, 0.4, 0.0],
        };
        self.queue
            .write_buffer(&self.globals_buf, 0, bytemuck::bytes_of(&globals));

        let mut obj_bytes = vec![0u8; OBJECT_STRIDE as usize * items.len()];
        for (i, item) in items.iter().enumerate() {
            let u = ObjectU {
                model: item.model.to_cols_array_2d(),
                color: item.color,
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
                            r: 0.35,
                            g: 0.45,
                            b: 0.60,
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

/// Everything `sttr-client race` needs to run.
pub struct RaceContext {
    pub sim: Sim,
    pub track: Track,
    pub input: SelectedInput,
    pub name: String,
    pub submit_to: Option<String>,
    pub out_path: PathBuf,
}

// Mesh slots
const MESH_RIBBON: usize = 0;
const MESH_TERRAIN: usize = 1;
const MESH_CHASSIS: usize = 2;
const MESH_WHEEL: usize = 3;

struct RaceApp {
    ctx: RaceContext,
    game: GameLoop,
    cam: FollowCam,
    window: Option<Arc<Window>>,
    gfx: Option<Gfx>,
    fatal: Option<anyhow::Error>,
    last_render: Option<Instant>,
    last_title: Instant,
    lap_result: Option<u32>, // lap ticks, once complete
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
        // 1. advance physics on the fixed-DT accumulator
        let input: &dyn InputSource = self.ctx.input.source();
        let fr = self.game.frame(&mut self.ctx.sim, input)?;

        if fr.lap_just_completed {
            self.on_lap_complete()?;
        }

        // 2. interpolate + camera
        let pose = interpolate(&self.game.prev, &self.game.curr, fr.alpha);
        let now = Instant::now();
        let rdt = self
            .last_render
            .map(|l| now.duration_since(l).as_secs_f32())
            .unwrap_or(1.0 / 60.0);
        self.last_render = Some(now);

        let car_pos = Vec3::from(pose.pos);
        let car_quat =
            Quat::from_xyzw(pose.quat[0], pose.quat[1], pose.quat[2], pose.quat[3]).normalize();
        let view = self.cam.view(car_pos, car_quat, rdt);

        // 3. draw list
        let mut items = vec![
            DrawItem { mesh: MESH_RIBBON, model: Mat4::IDENTITY, color: COLOR_RIBBON },
            DrawItem { mesh: MESH_TERRAIN, model: Mat4::IDENTITY, color: COLOR_TERRAIN },
            DrawItem {
                mesh: MESH_CHASSIS,
                model: Mat4::from_rotation_translation(car_quat, car_pos),
                color: COLOR_CHASSIS,
            },
        ];
        for w in &pose.wheels {
            // wheel pos is world-space (see NOTES.md); orientation = chassis
            // yaw'd by steer, spun about the axle
            let rot = car_quat
                * Quat::from_rotation_y(w.steer_angle)
                * Quat::from_rotation_x(w.spin_angle);
            items.push(DrawItem {
                mesh: MESH_WHEEL,
                model: Mat4::from_rotation_translation(rot, Vec3::from(w.pos)),
                color: COLOR_WHEEL,
            });
        }

        if let Some(gfx) = &mut self.gfx {
            gfx.render(view, &items)?;
        }

        // 4. occasional window-title telemetry
        if now.duration_since(self.last_title) > Duration::from_millis(250) {
            self.last_title = now;
            if let Some(win) = &self.window {
                let kmh = pose.speed * 3.6;
                let title = match self.lap_result {
                    Some(ticks) => format!(
                        "sttr-client — LAP {:.3}s — {:.0} km/h",
                        ticks as f64 * crate::game_loop::DT,
                        kmh
                    ),
                    None => format!(
                        "sttr-client — {:.0} km/h — lap {:.0}%",
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
            // Submit off-thread so rendering never stalls on the network.
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
            .with_title("sttr-client")
            .with_inner_size(winit::dpi::LogicalSize::new(1600.0, 900.0));
        let window = match event_loop.create_window(attrs) {
            Ok(w) => Arc::new(w),
            Err(e) => return self.fail(event_loop, anyhow!("window creation failed: {e}")),
        };

        let cpu_meshes = [
            build_ribbon(&self.ctx.track),
            build_terrain(&self.ctx.track),
            build_box(CHASSIS_HALF),
            build_box(WHEEL_HALF),
        ];
        match Gfx::new(window.clone(), &cpu_meshes) {
            Ok(g) => self.gfx = Some(g),
            Err(e) => return self.fail(event_loop, e),
        }
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

/// Run the interactive race window; returns after the window closes.
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
        // 3 samples in a straight line, ends 20 m apart with 10 m spacing:
        // no loop closure, so 2 segments * 2 tris * 3 idx
        assert_eq!(m.indices.len(), 2 * 6);
        // width 8 => edges 4 m either side of center
        let l = Vec3::from(m.vertices[0].pos);
        let r = Vec3::from(m.vertices[1].pos);
        assert!((l.distance(r) - 8.0).abs() < 1e-4);
        // all indices in range
        assert!(m.indices.iter().all(|&i| (i as usize) < m.vertices.len()));
    }

    #[test]
    fn terrain_is_flat_shaded_triangles() {
        let t = track();
        let m = build_terrain(&t);
        assert_eq!(m.vertices.len(), t.terrain_triangles.len() * 3);
        assert_eq!(m.indices.len(), t.terrain_triangles.len() * 3);
        // synthetic triangle lies in the XZ plane -> normal is ±Y
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
}

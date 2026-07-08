//! wasmtime host for sim.wasm — CONTRACTS.md §1. Physics runs ONLY inside the
//! wasm kernel; this module marshals bytes across the boundary and nothing more.

use std::path::Path;

use anyhow::{anyhow, bail, Context, Result};
use wasmtime::{Engine, Instance, Linker, Memory, Module, Store, TypedFunc};

use crate::laplog::TickRecord;

// §1.3 status bits (full contract set; not all are consumed by the client yet)
#[allow(dead_code)]
pub const STATUS_RUNNING: u32 = 0x0000_0001;
pub const STATUS_LAP_COMPLETE: u32 = 0x0000_0002;
#[allow(dead_code)]
pub const STATUS_OFF_TRACK: u32 = 0x0000_0004;
pub const STATUS_LAP_INVALID: u32 = 0x0000_0008;
pub const STATUS_ERROR: u32 = 0x8000_0000;

pub const SIM_STATE_SIZE: usize = 200;
pub const EXPECTED_ABI_VERSION: u32 = 1;

/// Per-wheel dynamic state — §1.2 offset 56, 8 f32 per wheel × 4 wheels.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelState {
    pub pos: [f32; 3],
    pub spin_angle: f32,
    pub steer_angle: f32,
    pub susp_compression: f32,
    pub slip_ratio: f32,
    pub slip_angle: f32,
}

/// `SimStateV1` — packed 200-byte struct, §1.2. Copied out of wasm memory
/// every tick; prev+curr kept for render interpolation.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SimState {
    pub tick: u32,
    pub pos: [f32; 3],
    pub quat: [f32; 4], // (x, y, z, w)
    pub lin_vel: [f32; 3],
    pub ang_vel: [f32; 3],
    pub wheels: [WheelState; 4],
    pub speed: f32,
    pub lap_progress: f32,
    pub checkpoint_mask: u32,
    pub lap_time_ticks: u32,
}

fn f32_at(b: &[u8], off: usize) -> f32 {
    f32::from_le_bytes(b[off..off + 4].try_into().unwrap())
}
fn u32_at(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(b[off..off + 4].try_into().unwrap())
}
fn f32x3_at(b: &[u8], off: usize) -> [f32; 3] {
    [f32_at(b, off), f32_at(b, off + 4), f32_at(b, off + 8)]
}

impl SimState {
    /// Decode the packed little-endian struct at the exact §1.2 offsets.
    pub fn from_bytes(b: &[u8; SIM_STATE_SIZE]) -> SimState {
        let mut wheels = [WheelState::default(); 4];
        for (w, wheel) in wheels.iter_mut().enumerate() {
            let base = 56 + w * 32; // 8 f32 per wheel
            *wheel = WheelState {
                pos: f32x3_at(b, base),
                spin_angle: f32_at(b, base + 12),
                steer_angle: f32_at(b, base + 16),
                susp_compression: f32_at(b, base + 20),
                slip_ratio: f32_at(b, base + 24),
                slip_angle: f32_at(b, base + 28),
            };
        }
        SimState {
            tick: u32_at(b, 0),
            pos: f32x3_at(b, 4),
            quat: [f32_at(b, 16), f32_at(b, 20), f32_at(b, 24), f32_at(b, 28)],
            lin_vel: f32x3_at(b, 32),
            ang_vel: f32x3_at(b, 44),
            wheels,
            speed: f32_at(b, 184),
            lap_progress: f32_at(b, 188),
            checkpoint_mask: u32_at(b, 192),
            lap_time_ticks: u32_at(b, 196),
        }
    }
}

/// Abstraction over the stepped sim so the game loop is unit-testable without
/// sim.wasm. The only real implementation is [`Sim`].
pub trait SimBackend {
    fn step(&mut self, rec: TickRecord) -> Result<u32>;
    fn state(&mut self) -> Result<SimState>;
    fn state_hash(&mut self) -> Result<u64>;
    fn lap_time_ticks(&mut self) -> Result<u32>;
}

/// wasmtime 46 uses its own error type that does not interoperate with
/// anyhow's trait bounds; funnel every wasm-boundary error through this.
fn werr(e: wasmtime::Error) -> anyhow::Error {
    anyhow!("wasm error: {e}")
}

/// Typed handles to every §1.1 export.
pub struct Sim {
    store: Store<()>,
    memory: Memory,
    f_abi_version: TypedFunc<(), u32>,
    f_alloc: TypedFunc<u32, u32>,
    f_load_track: TypedFunc<(u32, u32), i32>,
    f_reset: TypedFunc<(), ()>,
    f_step: TypedFunc<(i32, u32, u32, u32), u32>,
    f_replay: TypedFunc<(u32, u32), u32>,
    f_state_ptr: TypedFunc<(), u32>,
    f_state_size: TypedFunc<(), u32>,
    f_state_hash_lo: TypedFunc<(), u32>,
    f_state_hash_hi: TypedFunc<(), u32>,
    f_lap_time_ticks: TypedFunc<(), u32>,
    /// OPTIONAL export (docs/FFB.md): steering rack torque in Nm. Absent on
    /// kernels built before the FFB work — then `ffb_torque()` returns 0.0.
    f_ffb_torque: Option<TypedFunc<(), f32>>,
}

impl Sim {
    /// Load and instantiate sim.wasm. Any WASI (or other) imports the
    /// Emscripten standalone build declares are stubbed with no-op functions
    /// returning default values — the kernel contract forbids it from relying
    /// on them (§1: no time, no rand, no I/O).
    pub fn load(path: &Path) -> Result<Sim> {
        if !path.exists() {
            bail!(
                "sim.wasm not found at '{}'.\n\
                 The physics kernel is built by the physics workstream \
                 (physics/build_wasm.sh -> physics/out/sim.wasm).\n\
                 Build it first, or point --sim-wasm at an existing binary.",
                path.display()
            );
        }
        let engine = Engine::default();
        let module = Module::from_file(&engine, path).map_err(|e| {
            anyhow!("failed to compile wasm module '{}': {e}", path.display())
        })?;

        let mut store = Store::new(&engine, ());
        let mut linker: Linker<()> = Linker::new(&engine);
        // No-op stubs for wasi_snapshot_preview1 (and anything else the
        // standalone-wasm build imports). Funcs return zeroed results, which
        // for WASI means errno SUCCESS.
        linker
            .define_unknown_imports_as_default_values(&mut store, &module)
            .map_err(|e| anyhow!("failed to stub wasm imports: {e}"))?;

        let instance: Instance = linker
            .instantiate(&mut store, &module)
            .map_err(|e| anyhow!("failed to instantiate sim.wasm: {e}"))?;

        let memory = instance
            .get_memory(&mut store, "memory")
            .ok_or_else(|| anyhow!("sim.wasm does not export 'memory'"))?;

        macro_rules! func {
            ($name:literal) => {
                instance.get_typed_func(&mut store, $name).map_err(|e| {
                    anyhow!("sim.wasm missing or mistyped export '{}': {e}", $name)
                })?
            };
        }

        // Optional FFB export — old binaries simply don't have it.
        let f_ffb_torque = instance
            .get_typed_func::<(), f32>(&mut store, "sim_ffb_torque")
            .ok();

        let sim = Sim {
            f_abi_version: func!("sim_abi_version"),
            f_alloc: func!("sim_alloc"),
            f_load_track: func!("sim_load_track"),
            f_reset: func!("sim_reset"),
            f_step: func!("sim_step"),
            f_replay: func!("sim_replay"),
            f_state_ptr: func!("sim_state_ptr"),
            f_state_size: func!("sim_state_size"),
            f_state_hash_lo: func!("sim_state_hash_lo"),
            f_state_hash_hi: func!("sim_state_hash_hi"),
            f_lap_time_ticks: func!("sim_lap_time_ticks"),
            f_ffb_torque,
            memory,
            store,
        };
        let mut sim = sim;

        let abi = sim.f_abi_version.call(&mut sim.store, ()).map_err(werr)?;
        if abi != EXPECTED_ABI_VERSION {
            bail!("sim.wasm ABI version {abi}, client expects {EXPECTED_ABI_VERSION}");
        }
        let state_size = sim.f_state_size.call(&mut sim.store, ()).map_err(werr)?;
        if state_size as usize != SIM_STATE_SIZE {
            bail!("sim_state_size() = {state_size}, contract says {SIM_STATE_SIZE}");
        }
        Ok(sim)
    }

    /// Copy a host buffer into wasm memory via sim_alloc; returns the wasm ptr.
    fn write_blob(&mut self, bytes: &[u8]) -> Result<u32> {
        let ptr = self
            .f_alloc
            .call(&mut self.store, bytes.len() as u32)
            .map_err(werr)?;
        if ptr == 0 {
            bail!("sim_alloc({}) returned null", bytes.len());
        }
        self.memory
            .write(&mut self.store, ptr as usize, bytes)
            .context("writing blob into wasm memory")?;
        Ok(ptr)
    }

    /// Load the exact track.bin bytes (TRK1) into the kernel.
    pub fn load_track(&mut self, track_bytes: &[u8]) -> Result<()> {
        let ptr = self.write_blob(track_bytes)?;
        let rc = self
            .f_load_track
            .call(&mut self.store, (ptr, track_bytes.len() as u32))
            .map_err(werr)?;
        if rc != 0 {
            bail!("sim_load_track rejected track blob (rc = {rc})");
        }
        Ok(())
    }

    /// Car to spawn pose, tick = 0.
    pub fn reset(&mut self) -> Result<()> {
        self.f_reset.call(&mut self.store, ()).map_err(werr)
    }

    /// One 400 Hz tick with quantized inputs (§2); returns status bits (§1.3).
    pub fn step_quantized(&mut self, steer: i16, throttle: u16, brake: u16, flags: u16) -> Result<u32> {
        self.f_step
            .call(
                &mut self.store,
                (steer as i32, throttle as u32, brake as u32, flags as u32),
            )
            .map_err(werr)
    }

    /// Replay a full tick log inside the kernel at max speed (§1.1 sim_replay).
    /// Records are the same 8-byte layout as LAPLOG §3.
    pub fn replay(&mut self, ticks: &[TickRecord]) -> Result<u32> {
        let mut blob = Vec::with_capacity(ticks.len() * 8);
        for t in ticks {
            blob.extend_from_slice(&t.to_bytes());
        }
        let ptr = self.write_blob(&blob)?;
        self.f_replay
            .call(&mut self.store, (ptr, ticks.len() as u32))
            .map_err(werr)
    }

    fn read_state(&mut self) -> Result<SimState> {
        let ptr = self.f_state_ptr.call(&mut self.store, ()).map_err(werr)? as usize;
        let mut buf = [0u8; SIM_STATE_SIZE];
        self.memory
            .read(&self.store, ptr, &mut buf)
            .context("reading SimStateV1 from wasm memory")?;
        Ok(SimState::from_bytes(&buf))
    }

    /// Steering rack torque in Nm for force feedback (docs/FFB.md). Returns
    /// 0.0 when the loaded kernel predates the `sim_ffb_torque` export or the
    /// call traps — FFB is best-effort and must never take the sim down.
    pub fn ffb_torque(&mut self) -> f32 {
        match &self.f_ffb_torque {
            Some(f) => f.call(&mut self.store, ()).unwrap_or(0.0),
            None => 0.0,
        }
    }

    fn read_state_hash(&mut self) -> Result<u64> {
        let lo = self.f_state_hash_lo.call(&mut self.store, ()).map_err(werr)? as u64;
        let hi = self.f_state_hash_hi.call(&mut self.store, ()).map_err(werr)? as u64;
        Ok((hi << 32) | lo)
    }
}

impl SimBackend for Sim {
    fn step(&mut self, rec: TickRecord) -> Result<u32> {
        self.step_quantized(rec.steer, rec.throttle, rec.brake, rec.flags)
    }
    fn state(&mut self) -> Result<SimState> {
        self.read_state()
    }
    fn state_hash(&mut self) -> Result<u64> {
        self.read_state_hash()
    }
    fn lap_time_ticks(&mut self) -> Result<u32> {
        self.f_lap_time_ticks.call(&mut self.store, ()).map_err(werr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Craft a 200-byte SimStateV1 buffer and check every field decodes from
    /// its exact contract offset.
    #[test]
    fn sim_state_decodes_at_exact_offsets() {
        let mut b = [0u8; SIM_STATE_SIZE];
        b[0..4].copy_from_slice(&7u32.to_le_bytes()); // tick
        b[4..8].copy_from_slice(&1.5f32.to_le_bytes()); // pos.x
        b[8..12].copy_from_slice(&2.5f32.to_le_bytes()); // pos.y
        b[12..16].copy_from_slice(&(-3.0f32).to_le_bytes()); // pos.z
        // quat (x,y,z,w) at 16
        for (i, v) in [0.0f32, 0.0, 0.0, 1.0].iter().enumerate() {
            b[16 + i * 4..20 + i * 4].copy_from_slice(&v.to_le_bytes());
        }
        b[32..36].copy_from_slice(&10.0f32.to_le_bytes()); // lin_vel.x
        b[44..48].copy_from_slice(&0.5f32.to_le_bytes()); // ang_vel.x

        // wheel 0 starts at 56; wheel 3 at 56 + 3*32 = 152
        b[56..60].copy_from_slice(&(-0.9f32).to_le_bytes()); // wheel0 pos.x
        b[68..72].copy_from_slice(&6.28f32.to_le_bytes()); // wheel0 spin_angle (56+12)
        b[72..76].copy_from_slice(&0.3f32.to_le_bytes()); // wheel0 steer_angle (56+16)
        b[152..156].copy_from_slice(&9.0f32.to_le_bytes()); // wheel3 pos.x
        b[176..180].copy_from_slice(&0.11f32.to_le_bytes()); // wheel3 slip_ratio (152+24)
        b[180..184].copy_from_slice(&0.22f32.to_le_bytes()); // wheel3 slip_angle (152+28)

        b[184..188].copy_from_slice(&42.0f32.to_le_bytes()); // speed
        b[188..192].copy_from_slice(&0.75f32.to_le_bytes()); // lap progress
        b[192..196].copy_from_slice(&0b1011u32.to_le_bytes()); // checkpoint mask
        b[196..200].copy_from_slice(&33400u32.to_le_bytes()); // lap_time_ticks

        let s = SimState::from_bytes(&b);
        assert_eq!(s.tick, 7);
        assert_eq!(s.pos, [1.5, 2.5, -3.0]);
        assert_eq!(s.quat, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(s.lin_vel, [10.0, 0.0, 0.0]);
        assert_eq!(s.ang_vel, [0.5, 0.0, 0.0]);
        assert_eq!(s.wheels[0].pos[0], -0.9);
        assert_eq!(s.wheels[0].spin_angle, 6.28);
        assert_eq!(s.wheels[0].steer_angle, 0.3);
        assert_eq!(s.wheels[3].pos[0], 9.0);
        assert_eq!(s.wheels[3].slip_ratio, 0.11);
        assert_eq!(s.wheels[3].slip_angle, 0.22);
        assert_eq!(s.speed, 42.0);
        assert_eq!(s.lap_progress, 0.75);
        assert_eq!(s.checkpoint_mask, 0b1011);
        assert_eq!(s.lap_time_ticks, 33400);
    }

    #[test]
    fn missing_wasm_gives_clear_error() {
        let msg = match Sim::load(Path::new("definitely/not/a/real/sim.wasm")) {
            Ok(_) => panic!("load unexpectedly succeeded"),
            Err(err) => format!("{err}"),
        };
        assert!(msg.contains("sim.wasm not found"), "unhelpful error: {msg}");
        assert!(msg.contains("--sim-wasm"), "should mention the CLI flag: {msg}");
    }
}

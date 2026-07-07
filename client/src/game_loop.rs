//! Decoupled fixed-timestep game loop (CONTRACTS.md §2, ARCHITECTURE.md).
//! Physics always advances in exact DT increments regardless of render rate;
//! rendering interpolates between the two most recent sim states.

use std::time::Instant;

use anyhow::Result;

use crate::input::{InputSource, RawInput};
use crate::laplog::TickRecord;
use crate::sim::{SimBackend, SimState, WheelState, STATUS_LAP_COMPLETE};

#[allow(dead_code)] // contract constant (§2); laplog re-declares its own u16 copy
pub const TICK_RATE: u32 = 400;
/// Matches the single DT literal in physics/include/sim/sim.h (0.0025f).
/// Display/reporting only — the accumulator itself is integer nanoseconds so
/// tick counting never suffers binary-fraction drift (0.0025 is inexact in f64).
pub const DT: f64 = 0.0025;
pub const DT_NANOS: u64 = 2_500_000;
/// Anti-spiral-of-death clamp on frame time.
pub const MAX_FRAME_NANOS: u64 = 250_000_000;

/// Quantize a raw analog sample per CONTRACTS.md §2. The quantized integers
/// are the canonical truth consumed by both the live sim and the validator.
pub fn quantize(raw: &RawInput) -> TickRecord {
    TickRecord {
        steer: (raw.steer.clamp(-1.0, 1.0) * 32767.0).round() as i16,
        throttle: (raw.throttle.clamp(0.0, 1.0) * 65535.0).round() as u16,
        brake: (raw.brake.clamp(0.0, 1.0) * 65535.0).round() as u16,
        flags: raw.handbrake as u16, // bit 0 = handbrake; bits 1–15 reserved = 0
    }
}

/// Interpolated pose handed to the renderer.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RenderPose {
    pub pos: [f32; 3],
    pub quat: [f32; 4],
    pub wheels: [WheelState; 4],
    pub speed: f32,
    pub lap_progress: f32,
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn lerp3(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)]
}

/// Normalized quaternion lerp with hemisphere correction (nlerp). Adequate
/// for the tiny per-tick rotations at 400 Hz.
pub fn nlerp(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    let mut q = [
        lerp(a[0], b[0] * sign, t),
        lerp(a[1], b[1] * sign, t),
        lerp(a[2], b[2] * sign, t),
        lerp(a[3], b[3] * sign, t),
    ];
    let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if len > 1e-12 {
        for c in &mut q {
            *c /= len;
        }
    } else {
        q = [0.0, 0.0, 0.0, 1.0];
    }
    q
}

/// Blend previous and current sim states at `alpha` in [0,1].
pub fn interpolate(prev: &SimState, curr: &SimState, alpha: f32) -> RenderPose {
    let t = alpha.clamp(0.0, 1.0);
    let mut wheels = [WheelState::default(); 4];
    for i in 0..4 {
        let (a, b) = (&prev.wheels[i], &curr.wheels[i]);
        wheels[i] = WheelState {
            pos: lerp3(a.pos, b.pos, t),
            spin_angle: lerp(a.spin_angle, b.spin_angle, t),
            steer_angle: lerp(a.steer_angle, b.steer_angle, t),
            susp_compression: lerp(a.susp_compression, b.susp_compression, t),
            slip_ratio: lerp(a.slip_ratio, b.slip_ratio, t),
            slip_angle: lerp(a.slip_angle, b.slip_angle, t),
        };
    }
    RenderPose {
        pos: lerp3(prev.pos, curr.pos, t),
        quat: nlerp(prev.quat, curr.quat, t),
        wheels,
        speed: lerp(prev.speed, curr.speed, t),
        lap_progress: lerp(prev.lap_progress, curr.lap_progress, t),
    }
}

/// Result of one render-frame's worth of sim stepping.
#[derive(Debug, Clone, Copy)]
pub struct FrameResult {
    /// acc/DT in [0,1) — interpolation factor for rendering.
    pub alpha: f32,
    #[allow(dead_code)] // exercised by unit tests; useful for debugging overlays
    pub ticks_run: u32,
    /// True exactly once, on the frame the first LAP_COMPLETE tick landed.
    pub lap_just_completed: bool,
    /// Latest §1.3 status bits.
    #[allow(dead_code)]
    pub status: u32,
}

pub struct GameLoop {
    acc_ns: u64,
    last_frame: Option<Instant>,
    pub prev: SimState,
    pub curr: SimState,
    /// Quantized tick log — becomes the LAPLOG body. Frozen on lap complete.
    pub log: Vec<TickRecord>,
    frozen: bool,
    last_status: u32,
}

impl GameLoop {
    pub fn new(initial: SimState) -> GameLoop {
        GameLoop {
            acc_ns: 0,
            last_frame: None,
            prev: initial,
            curr: initial,
            log: Vec::with_capacity(crate::laplog::MAX_TICKS as usize),
            frozen: false,
            last_status: 0,
        }
    }

    #[allow(dead_code)] // test/debug helper
    pub fn is_frozen(&self) -> bool {
        self.frozen
    }

    /// Run one render frame: accumulate real time, step the sim zero or more
    /// whole DTs, return alpha for interpolation.
    pub fn frame(&mut self, sim: &mut dyn SimBackend, input: &dyn InputSource) -> Result<FrameResult> {
        let now = Instant::now();
        let frame_ns = self
            .last_frame
            .map(|l| now.duration_since(l).as_nanos().min(u128::from(MAX_FRAME_NANOS)) as u64)
            .unwrap_or(0);
        self.last_frame = Some(now);
        self.advance_ns(frame_ns, sim, input)
    }

    /// Time-explicit seconds wrapper (unit-testable without real clocks).
    #[allow(dead_code)]
    pub fn advance(
        &mut self,
        frame_dt: f64,
        sim: &mut dyn SimBackend,
        input: &dyn InputSource,
    ) -> Result<FrameResult> {
        let ns = (frame_dt.max(0.0) * 1e9).round() as u64;
        self.advance_ns(ns, sim, input)
    }

    /// Integer-nanosecond core: exact tick counting, no float accumulation.
    pub fn advance_ns(
        &mut self,
        frame_ns: u64,
        sim: &mut dyn SimBackend,
        input: &dyn InputSource,
    ) -> Result<FrameResult> {
        self.acc_ns += frame_ns.min(MAX_FRAME_NANOS);

        let mut ticks_run = 0u32;
        let mut lap_just_completed = false;

        while self.acc_ns >= DT_NANOS {
            self.acc_ns -= DT_NANOS;
            let rec = quantize(&input.sample());
            let status = sim.step(rec)?;
            self.last_status = status;
            if !self.frozen {
                self.log.push(rec);
            }
            self.prev = std::mem::replace(&mut self.curr, sim.state()?);
            ticks_run += 1;

            if status & STATUS_LAP_COMPLETE != 0 && !self.frozen {
                // Freeze the log: the record that produced LAP_COMPLETE is the
                // last one included.
                self.frozen = true;
                lap_just_completed = true;
                // Stop consuming leftover accumulator this frame so the lap
                // state is exactly the state at completion when we snapshot.
                break;
            }
        }

        Ok(FrameResult {
            alpha: (self.acc_ns as f64 / DT_NANOS as f64) as f32,
            ticks_run,
            lap_just_completed,
            status: self.last_status,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    // -- test doubles (no wasm, no hardware) --------------------------------

    struct FakeSim {
        tick: u32,
        complete_at: Option<u32>,
        last_rec: Option<TickRecord>,
    }

    impl SimBackend for FakeSim {
        fn step(&mut self, rec: TickRecord) -> Result<u32> {
            self.tick += 1;
            self.last_rec = Some(rec);
            let mut status = crate::sim::STATUS_RUNNING;
            if Some(self.tick) == self.complete_at {
                status |= STATUS_LAP_COMPLETE;
            }
            Ok(status)
        }
        fn state(&mut self) -> Result<SimState> {
            let mut s = SimState::default();
            s.tick = self.tick;
            s.pos = [self.tick as f32, 0.0, 0.0];
            s.quat = [0.0, 0.0, 0.0, 1.0];
            Ok(s)
        }
        fn state_hash(&mut self) -> Result<u64> {
            Ok(0x1234)
        }
        fn lap_time_ticks(&mut self) -> Result<u32> {
            Ok(self.tick)
        }
    }

    struct ConstInput(Mutex<RawInput>);
    impl InputSource for ConstInput {
        fn sample(&self) -> RawInput {
            *self.0.lock().unwrap()
        }
    }
    fn input(steer: f32, throttle: f32, brake: f32, hb: bool) -> ConstInput {
        ConstInput(Mutex::new(RawInput { steer, throttle, brake, handbrake: hb }))
    }

    // -- quantization -------------------------------------------------------

    #[test]
    fn quantize_endpoints_and_center() {
        let r = quantize(&RawInput { steer: -1.0, throttle: 0.0, brake: 1.0, handbrake: true });
        assert_eq!(r.steer, -32767);
        assert_eq!(r.throttle, 0);
        assert_eq!(r.brake, 65535);
        assert_eq!(r.flags, 1);

        let r = quantize(&RawInput { steer: 1.0, throttle: 1.0, brake: 0.0, handbrake: false });
        assert_eq!(r.steer, 32767);
        assert_eq!(r.throttle, 65535);
        assert_eq!(r.flags, 0);

        let r = quantize(&RawInput::default());
        assert_eq!((r.steer, r.throttle, r.brake, r.flags), (0, 0, 0, 0));
    }

    #[test]
    fn quantize_clamps_out_of_range() {
        let r = quantize(&RawInput { steer: -7.0, throttle: 2.0, brake: -1.0, handbrake: false });
        assert_eq!(r.steer, -32767);
        assert_eq!(r.throttle, 65535);
        assert_eq!(r.brake, 0);
    }

    // -- accumulator --------------------------------------------------------

    #[test]
    fn accumulator_runs_exact_tick_counts() {
        let mut sim = FakeSim { tick: 0, complete_at: None, last_rec: None };
        let inp = input(0.5, 1.0, 0.0, false);
        let mut gl = GameLoop::new(SimState::default());

        // 10 ms = exactly 4 ticks at 400 Hz
        let fr = gl.advance(0.010, &mut sim, &inp).unwrap();
        assert_eq!(fr.ticks_run, 4);
        assert!(fr.alpha >= 0.0 && fr.alpha < 1.0);
        assert_eq!(gl.log.len(), 4);

        // 1 ms: accumulates but no tick yet (acc < DT after 4 exact ticks)
        let fr = gl.advance(0.001, &mut sim, &inp).unwrap();
        assert_eq!(fr.ticks_run, 0);
        // another 1.6ms pushes it over one DT
        let fr = gl.advance(0.0016, &mut sim, &inp).unwrap();
        assert_eq!(fr.ticks_run, 1);
        assert_eq!(gl.log.len(), 5);

        // inputs got quantized into the log
        assert_eq!(gl.log[0].steer, (0.5f32 * 32767.0).round() as i16);
        assert_eq!(gl.log[0].throttle, 65535);
    }

    #[test]
    fn frame_dt_is_clamped_against_spiral() {
        let mut sim = FakeSim { tick: 0, complete_at: None, last_rec: None };
        let inp = input(0.0, 0.0, 0.0, false);
        let mut gl = GameLoop::new(SimState::default());
        // A 10-second hitch must be clamped to MAX_FRAME_DT (0.25 s = 100 ticks).
        let fr = gl.advance(10.0, &mut sim, &inp).unwrap();
        assert_eq!(fr.ticks_run, 100);
    }

    #[test]
    fn lap_complete_freezes_log() {
        let mut sim = FakeSim { tick: 0, complete_at: Some(3), last_rec: None };
        let inp = input(0.0, 1.0, 0.0, false);
        let mut gl = GameLoop::new(SimState::default());

        let fr = gl.advance(0.010, &mut sim, &inp).unwrap(); // would be 4 ticks
        assert!(fr.lap_just_completed);
        assert_eq!(gl.log.len(), 3, "log must include the completing tick and stop");
        assert!(gl.is_frozen());

        // further frames keep stepping the sim but never append to the log
        let fr = gl.advance(0.010, &mut sim, &inp).unwrap();
        assert!(!fr.lap_just_completed, "completion fires exactly once");
        assert_eq!(gl.log.len(), 3);
        assert!(fr.ticks_run > 0);
    }

    #[test]
    fn prev_curr_track_last_two_states() {
        let mut sim = FakeSim { tick: 0, complete_at: None, last_rec: None };
        let inp = input(0.0, 0.0, 0.0, false);
        let mut gl = GameLoop::new(SimState::default());
        gl.advance(DT * 2.0 + 1e-9, &mut sim, &inp).unwrap();
        assert_eq!(gl.prev.tick, 1);
        assert_eq!(gl.curr.tick, 2);
    }

    // -- interpolation ------------------------------------------------------

    #[test]
    fn interpolation_lerps_position_and_wheels() {
        let mut a = SimState::default();
        a.pos = [0.0, 0.0, 0.0];
        a.quat = [0.0, 0.0, 0.0, 1.0];
        a.wheels[2].spin_angle = 1.0;
        a.speed = 10.0;
        let mut b = a;
        b.pos = [2.0, 4.0, -6.0];
        b.wheels[2].spin_angle = 2.0;
        b.speed = 20.0;

        let p = interpolate(&a, &b, 0.5);
        assert_eq!(p.pos, [1.0, 2.0, -3.0]);
        assert_eq!(p.wheels[2].spin_angle, 1.5);
        assert_eq!(p.speed, 15.0);
        // endpoints
        assert_eq!(interpolate(&a, &b, 0.0).pos, a.pos);
        assert_eq!(interpolate(&a, &b, 1.0).pos, b.pos);
    }

    #[test]
    fn nlerp_is_normalized_and_hemisphere_corrected() {
        // 90° about Y vs identity
        let id = [0.0, 0.0, 0.0, 1.0];
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let y90 = [0.0, s, 0.0, s];
        let q = nlerp(id, y90, 0.5);
        let len: f32 = q.iter().map(|c| c * c).sum::<f32>().sqrt();
        assert!((len - 1.0).abs() < 1e-6);

        // hemisphere: nlerp(q, -q, t) must stay at ±q, not swing through zero
        let neg = [-y90[0], -y90[1], -y90[2], -y90[3]];
        let q = nlerp(y90, neg, 0.5);
        let dot = q[0] * y90[0] + q[1] * y90[1] + q[2] * y90[2] + q[3] * y90[3];
        assert!(dot.abs() > 0.999, "hemisphere correction failed: dot = {dot}");
    }
}

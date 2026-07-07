//! Input sources. Hardware is polled at ~1 kHz on a dedicated thread; each
//! 400 Hz sim tick consumes the *latest* sample (CONTRACTS.md §2).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use gilrs::{Axis, Button, Gilrs};

/// Raw, un-quantized analog input state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawInput {
    /// −1.0 (full left) .. 1.0 (full right)
    pub steer: f32,
    /// 0.0 .. 1.0
    pub throttle: f32,
    /// 0.0 .. 1.0
    pub brake: f32,
    pub handbrake: bool,
}

impl Default for RawInput {
    fn default() -> Self {
        RawInput { steer: 0.0, throttle: 0.0, brake: 0.0, handbrake: false }
    }
}

pub trait InputSource: Send + Sync {
    /// Return the most recent input sample. Must be cheap (called at 400 Hz).
    fn sample(&self) -> RawInput;
}

// ---------------------------------------------------------------------------
// Axis/trigger binding configuration
// ---------------------------------------------------------------------------

/// A gilrs input a channel can be bound to: an axis or an analog button.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Binding {
    Axis(Axis),
    Button(Button),
}

/// Parse a CLI binding name, e.g. "leftstickx", "rightz", "righttrigger2".
pub fn parse_binding(name: &str) -> Option<Binding> {
    let n = name.to_ascii_lowercase();
    let axis = match n.as_str() {
        "leftstickx" => Some(Axis::LeftStickX),
        "leftsticky" => Some(Axis::LeftStickY),
        "rightstickx" => Some(Axis::RightStickX),
        "rightsticky" => Some(Axis::RightStickY),
        "leftz" => Some(Axis::LeftZ),
        "rightz" => Some(Axis::RightZ),
        "dpadx" => Some(Axis::DPadX),
        "dpady" => Some(Axis::DPadY),
        _ => None,
    };
    if let Some(a) = axis {
        return Some(Binding::Axis(a));
    }
    match n.as_str() {
        "lefttrigger" => Some(Binding::Button(Button::LeftTrigger)),
        "lefttrigger2" => Some(Binding::Button(Button::LeftTrigger2)),
        "righttrigger" => Some(Binding::Button(Button::RightTrigger)),
        "righttrigger2" => Some(Binding::Button(Button::RightTrigger2)),
        "south" => Some(Binding::Button(Button::South)),
        "east" => Some(Binding::Button(Button::East)),
        _ => None,
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WheelConfig {
    pub steer: Binding,
    pub throttle: Binding,
    pub brake: Binding,
    pub handbrake: Button,
}

impl Default for WheelConfig {
    fn default() -> Self {
        WheelConfig {
            steer: Binding::Axis(Axis::LeftStickX),
            throttle: Binding::Button(Button::RightTrigger2),
            brake: Binding::Button(Button::LeftTrigger2),
            handbrake: Button::East,
        }
    }
}

// ---------------------------------------------------------------------------
// Wheel / gamepad input (gilrs, dedicated ~1 kHz poll thread)
// ---------------------------------------------------------------------------

pub struct WheelInput {
    latest: Arc<Mutex<RawInput>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl WheelInput {
    /// Spawn the 1 kHz polling thread. Fails if gilrs cannot initialize or no
    /// gamepad is connected at startup.
    pub fn new(config: WheelConfig) -> Result<WheelInput, String> {
        // Probe on the calling thread so failure is synchronous.
        let probe = Gilrs::new().map_err(|e| format!("gilrs init failed: {e}"))?;
        if probe.gamepads().next().is_none() {
            return Err("no gamepad/wheel detected by gilrs".into());
        }
        drop(probe);

        let latest = Arc::new(Mutex::new(RawInput::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let latest_t = Arc::clone(&latest);
        let stop_t = Arc::clone(&stop);

        let handle = std::thread::Builder::new()
            .name("input-poll-1khz".into())
            .spawn(move || poll_loop(config, latest_t, stop_t))
            .map_err(|e| format!("failed to spawn input thread: {e}"))?;

        Ok(WheelInput { latest, stop, handle: Some(handle) })
    }
}

fn read_binding(gamepad: &gilrs::Gamepad<'_>, b: Binding) -> f32 {
    match b {
        Binding::Axis(a) => gamepad.axis_data(a).map(|d| d.value()).unwrap_or(0.0),
        Binding::Button(btn) => gamepad.button_data(btn).map(|d| d.value()).unwrap_or(0.0),
    }
}

fn poll_loop(config: WheelConfig, latest: Arc<Mutex<RawInput>>, stop: Arc<AtomicBool>) {
    // gilrs context lives on this thread (it is not Send on every platform).
    let mut gilrs = match Gilrs::new() {
        Ok(g) => g,
        Err(e) => {
            eprintln!("input thread: gilrs init failed: {e}");
            return;
        }
    };
    const PERIOD: Duration = Duration::from_micros(1000);
    // sleep() overshoot margin before switching to the busy tail
    const SPIN_MARGIN: Duration = Duration::from_micros(300);

    while !stop.load(Ordering::Relaxed) {
        let t0 = Instant::now();

        // Drain events so cached gamepad state is current.
        while gilrs.next_event().is_some() {}

        if let Some((_, gamepad)) = gilrs.gamepads().next() {
            let raw = RawInput {
                steer: read_binding(&gamepad, config.steer).clamp(-1.0, 1.0),
                throttle: read_binding(&gamepad, config.throttle).clamp(0.0, 1.0),
                brake: read_binding(&gamepad, config.brake).clamp(0.0, 1.0),
                handbrake: gamepad.is_pressed(config.handbrake),
            };
            *latest.lock().unwrap() = raw;
        }

        // ~1 kHz pacing: coarse sleep, then busy tail for precision.
        let elapsed = t0.elapsed();
        if elapsed + SPIN_MARGIN < PERIOD {
            std::thread::sleep(PERIOD - elapsed - SPIN_MARGIN);
        }
        while t0.elapsed() < PERIOD {
            std::hint::spin_loop();
        }
    }
}

impl InputSource for WheelInput {
    fn sample(&self) -> RawInput {
        *self.latest.lock().unwrap()
    }
}

impl Drop for WheelInput {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The input backend chosen for a race session. The keyboard variant keeps
/// the Arc so the window event loop can feed key transitions in.
pub enum SelectedInput {
    Wheel(WheelInput),
    Keyboard(Arc<KeyboardInput>),
}

impl SelectedInput {
    pub fn source(&self) -> &dyn InputSource {
        match self {
            SelectedInput::Wheel(w) => w,
            SelectedInput::Keyboard(k) => k.as_ref(),
        }
    }
}

/// Names of gamepads gilrs can currently see (for auto-select + `devices`).
pub fn detect_gamepads() -> Vec<String> {
    match Gilrs::new() {
        Ok(g) => g
            .gamepads()
            .map(|(_, gp)| format!("{} ({:?})", gp.name(), gp.power_info()))
            .collect(),
        Err(_) => Vec::new(),
    }
}

// ---------------------------------------------------------------------------
// Keyboard input (fed from winit events, smoothed so it is drivable)
// ---------------------------------------------------------------------------

/// Digital controls the window event loop maps keys onto.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    SteerLeft,
    SteerRight,
    Throttle,
    Brake,
    Handbrake,
}

#[derive(Debug)]
struct KbState {
    left: bool,
    right: bool,
    throttle: bool,
    brake: bool,
    handbrake: bool,
    // smoothed analog values
    steer: f32,
    throttle_v: f32,
    brake_v: f32,
    last: Option<Instant>,
}

/// Digital keys with attack/release smoothing. Steer slews at
/// [`STEER_SLEW`]/s toward the held direction and recenters a bit faster.
pub struct KeyboardInput {
    state: Mutex<KbState>,
}

pub const STEER_SLEW: f32 = 3.0; // full-scale units per second, per spec
pub const STEER_RECENTER: f32 = 5.0;
pub const THROTTLE_ATTACK: f32 = 4.0;
pub const THROTTLE_RELEASE: f32 = 8.0;
pub const BRAKE_ATTACK: f32 = 6.0;
pub const BRAKE_RELEASE: f32 = 10.0;

impl KeyboardInput {
    pub fn new() -> Arc<KeyboardInput> {
        Arc::new(KeyboardInput {
            state: Mutex::new(KbState {
                left: false,
                right: false,
                throttle: false,
                brake: false,
                handbrake: false,
                steer: 0.0,
                throttle_v: 0.0,
                brake_v: 0.0,
                last: None,
            }),
        })
    }

    /// Feed a key transition from the window event loop.
    pub fn set_control(&self, control: Control, pressed: bool) {
        let mut s = self.state.lock().unwrap();
        match control {
            Control::SteerLeft => s.left = pressed,
            Control::SteerRight => s.right = pressed,
            Control::Throttle => s.throttle = pressed,
            Control::Brake => s.brake = pressed,
            Control::Handbrake => s.handbrake = pressed,
        }
    }

    /// Advance smoothing by `dt` seconds and return the state. Split out from
    /// `sample()` so tests can drive time deterministically.
    fn advance(&self, dt: f32) -> RawInput {
        let mut s = self.state.lock().unwrap();
        let target_steer = (s.right as i32 - s.left as i32) as f32;

        let rate = if target_steer == 0.0 { STEER_RECENTER } else { STEER_SLEW };
        let max_delta = rate * dt;
        let delta = (target_steer - s.steer).clamp(-max_delta, max_delta);
        s.steer = (s.steer + delta).clamp(-1.0, 1.0);

        s.throttle_v = slew01(s.throttle_v, s.throttle, THROTTLE_ATTACK, THROTTLE_RELEASE, dt);
        s.brake_v = slew01(s.brake_v, s.brake, BRAKE_ATTACK, BRAKE_RELEASE, dt);

        RawInput {
            steer: s.steer,
            throttle: s.throttle_v,
            brake: s.brake_v,
            handbrake: s.handbrake,
        }
    }
}

fn slew01(current: f32, held: bool, attack: f32, release: f32, dt: f32) -> f32 {
    let target = if held { 1.0 } else { 0.0 };
    let rate = if held { attack } else { release };
    let max_delta = rate * dt;
    (current + (target - current).clamp(-max_delta, max_delta)).clamp(0.0, 1.0)
}

impl InputSource for KeyboardInput {
    fn sample(&self) -> RawInput {
        let now = Instant::now();
        let dt = {
            let mut s = self.state.lock().unwrap();
            let dt = s
                .last
                .map(|l| now.duration_since(l).as_secs_f32())
                .unwrap_or(0.0)
                .min(0.1);
            s.last = Some(now);
            dt
        };
        self.advance(dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keyboard_steer_slews_at_3_per_second() {
        let kb = KeyboardInput::new();
        kb.set_control(Control::SteerRight, true);
        // 0.1 s at 3/s -> 0.3
        let r = kb.advance(0.1);
        assert!((r.steer - 0.3).abs() < 1e-5, "steer = {}", r.steer);
        // saturates at 1.0
        for _ in 0..10 {
            kb.advance(0.1);
        }
        assert!((kb.advance(0.0).steer - 1.0).abs() < 1e-5);
        // release: recenters (faster than slew), never overshoots
        kb.set_control(Control::SteerRight, false);
        let r = kb.advance(0.1);
        assert!(r.steer < 1.0 && r.steer > 0.0);
        for _ in 0..10 {
            kb.advance(0.1);
        }
        assert_eq!(kb.advance(0.0).steer, 0.0);
    }

    #[test]
    fn keyboard_throttle_attack_release() {
        let kb = KeyboardInput::new();
        kb.set_control(Control::Throttle, true);
        let r = kb.advance(0.1); // 4/s * 0.1 = 0.4
        assert!((r.throttle - 0.4).abs() < 1e-5);
        kb.set_control(Control::Throttle, false);
        let r = kb.advance(0.025); // 8/s * 0.025 = 0.2 down
        assert!((r.throttle - 0.2).abs() < 1e-5);
    }

    #[test]
    fn keyboard_handbrake_is_instant() {
        let kb = KeyboardInput::new();
        kb.set_control(Control::Handbrake, true);
        assert!(kb.advance(0.001).handbrake);
        kb.set_control(Control::Handbrake, false);
        assert!(!kb.advance(0.001).handbrake);
    }

    #[test]
    fn binding_parser() {
        assert_eq!(parse_binding("LeftStickX"), Some(Binding::Axis(Axis::LeftStickX)));
        assert_eq!(parse_binding("rightz"), Some(Binding::Axis(Axis::RightZ)));
        assert_eq!(
            parse_binding("righttrigger2"),
            Some(Binding::Button(Button::RightTrigger2))
        );
        assert_eq!(parse_binding("warpdrive"), None);
    }
}

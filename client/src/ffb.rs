//! Native force feedback — DirectInput8 constant-force output (docs/FFB.md).
//!
//! Signal path, once per render frame:
//!   kernel `sim_ffb_torque()` (steering rack torque, Nm, 400 Hz native)
//!     -> 1-pole low-pass at `smoothing_hz`
//!     -> * gain / max_torque_nm, min_force floor, invert   (pure, unit-tested)
//!     -> IDirectInputEffect::SetParameters(DICONSTANTFORCE) at `update_hz`
//!
//! The DirectInput plumbing is Windows-only (`device` module below); every
//! other platform gets a stub whose `open` fails, so the client runs normally
//! with one warning. FFB is strictly read-only on sim state — it can never
//! feed back into the input log except through the driver's hands.

use std::time::{Duration, Instant};

use serde::Deserialize;

/// DirectInput nominal full-scale force (DI_FFNOMINALMAX).
pub const DI_MAX_MAGNITUDE: f32 = 10_000.0;

/// Normalized outputs below this are treated as zero instead of being lifted
/// to the min_force floor — keeps numeric noise from buzzing the wheel.
const MIN_FORCE_NOISE_EPS: f32 = 1e-4;

// ---------------------------------------------------------------------------
// Config (ffb.toml — docs/FFB.md §3)
// ---------------------------------------------------------------------------

/// `[ffb]` table of ffb.toml. Unknown keys are ignored (forward compat);
/// missing keys take the defaults below, which match docs/FFB.md §3.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default)]
pub struct FfbConfig {
    pub enabled: bool,
    /// Peak torque of the wheel base in Nm (CSL DD = 8.0); torque clips here.
    pub max_torque_nm: f32,
    /// Overall strength multiplier applied before clipping.
    pub gain: f32,
    /// Deadzone-crossing floor as a fraction of full force.
    pub min_force: f32,
    /// Reserved: parsed but not yet applied by the client.
    pub damping: f32,
    /// Reserved: parsed but not yet applied by the client.
    pub friction: f32,
    /// 1-pole low-pass cutoff (Hz) on rack torque.
    pub smoothing_hz: f32,
    /// Reserved: parsed but not yet applied by the client.
    pub standstill_reduction: bool,
    /// Reserved: parsed but not yet applied by the client.
    pub kerb_vibration_gain: f32,
    /// Reserved: the pneumatic-trail collapse lives in the kernel.
    pub understeer_drop: f32,
    /// Flip force direction if the wheel pulls the wrong way.
    pub invert: bool,
    /// Max DirectInput effect updates per second.
    pub update_hz: f32,
}

impl Default for FfbConfig {
    fn default() -> FfbConfig {
        FfbConfig {
            enabled: true,
            max_torque_nm: 18.0, // rack Nm at full output; 8.0 clipped past ~1 deg slip
            gain: 0.85,
            min_force: 0.02,
            damping: 0.10,
            friction: 0.03,
            smoothing_hz: 120.0,
            standstill_reduction: true,
            kerb_vibration_gain: 0.5,
            understeer_drop: 1.0,
            invert: false,
            update_hz: 400.0,
        }
    }
}

#[derive(Debug, Default, Deserialize)]
struct FfbFile {
    #[serde(default)]
    ffb: FfbConfig,
}

/// Commented default config written on first run. Kept in sync with
/// `FfbConfig::default()` by a unit test.
pub const DEFAULT_FFB_TOML: &str = r#"# sttr-client force feedback (docs/FFB.md §3).
# Signal path each render frame: kernel rack torque (Nm)
#   -> 1-pole low-pass at smoothing_hz
#   -> * gain / max_torque_nm  (clipped to +/- full force)
#   -> min_force floor, invert
#   -> DirectInput constant force in [-10000, 10000], sent at update_hz.
# Delete this file to regenerate the defaults.

[ffb]
enabled = true

# Peak torque your wheel base can produce, in Nm (Fanatec CSL DD = 8.0).
# Rack torque above this clips at full force.
max_torque_nm = 8.0

# Overall strength multiplier applied before clipping.
gain = 0.85

# Deadzone-crossing floor as a fraction of full force: any nonzero output is
# lifted to at least this. Direct drives want ~0; belt/gear rigs need more.
min_force = 0.02

# Reserved (parsed, not applied yet): damper / friction layers.
damping = 0.10
friction = 0.03

# 1-pole low-pass cutoff (Hz) on rack torque before it reaches the wheel.
smoothing_hz = 120

# Reserved (parsed, not applied yet).
standstill_reduction = true
kerb_vibration_gain = 0.5
understeer_drop = 1.0

# Flip force direction if the wheel pulls the wrong way.
invert = false

# Max DirectInput effect updates per second (the driver coalesces).
update_hz = 400
"#;

fn clamped(name: &str, v: f32, lo: f32, hi: f32) -> f32 {
    if !v.is_finite() {
        eprintln!("ffb.toml: {name} is not a finite number; using {lo}");
        return lo;
    }
    let c = v.clamp(lo, hi);
    if c != v {
        eprintln!("ffb.toml: {name} = {v} outside [{lo}, {hi}]; clamped to {c}");
    }
    c
}

impl FfbConfig {
    /// Parse the `[ffb]` table out of a toml string and clamp every field
    /// into a sane range (a hostile config must never panic or divide by 0).
    pub fn from_toml_str(s: &str) -> anyhow::Result<FfbConfig> {
        use anyhow::Context;
        let file: FfbFile = toml::from_str(s).context("parsing ffb.toml")?;
        Ok(file.ffb.sanitized())
    }

    fn sanitized(mut self) -> FfbConfig {
        self.max_torque_nm = clamped("max_torque_nm", self.max_torque_nm, 0.5, 50.0);
        self.gain = clamped("gain", self.gain, 0.0, 3.0);
        self.min_force = clamped("min_force", self.min_force, 0.0, 0.5);
        self.damping = clamped("damping", self.damping, 0.0, 1.0);
        self.friction = clamped("friction", self.friction, 0.0, 1.0);
        self.smoothing_hz = clamped("smoothing_hz", self.smoothing_hz, 1.0, 1000.0);
        self.kerb_vibration_gain = clamped("kerb_vibration_gain", self.kerb_vibration_gain, 0.0, 2.0);
        self.understeer_drop = clamped("understeer_drop", self.understeer_drop, 0.0, 2.0);
        self.update_hz = clamped("update_hz", self.update_hz, 10.0, 1000.0);
        self
    }

    /// ffb.toml lives next to the signing key in the OS config dir.
    pub fn config_path() -> anyhow::Result<std::path::PathBuf> {
        Ok(crate::signing::config_dir()?.join("ffb.toml"))
    }

    /// Load ffb.toml; on first run write the commented default file. Any
    /// failure (unreadable, malformed) logs a warning and returns defaults —
    /// a broken config must never take the race down.
    pub fn load_or_create_default() -> FfbConfig {
        let path = match Self::config_path() {
            Ok(p) => p,
            Err(e) => {
                eprintln!("ffb: {e:#}; using default config");
                return FfbConfig::default();
            }
        };
        if path.exists() {
            return match std::fs::read_to_string(&path) {
                Ok(s) => match FfbConfig::from_toml_str(&s) {
                    Ok(cfg) => cfg,
                    Err(e) => {
                        eprintln!("ffb: {e:#} in '{}'; using defaults", path.display());
                        FfbConfig::default()
                    }
                },
                Err(e) => {
                    eprintln!("ffb: cannot read '{}': {e}; using defaults", path.display());
                    FfbConfig::default()
                }
            };
        }
        // First run: persist the commented defaults so there is a file to edit.
        let write = path
            .parent()
            .map(std::fs::create_dir_all)
            .unwrap_or(Ok(()))
            .and_then(|()| std::fs::write(&path, DEFAULT_FFB_TOML));
        match write {
            Ok(()) => println!("ffb: wrote default config to {}", path.display()),
            Err(e) => eprintln!("ffb: could not write default '{}': {e}", path.display()),
        }
        FfbConfig::default()
    }
}

// ---------------------------------------------------------------------------
// Pure signal shaping (unit-testable without hardware)
// ---------------------------------------------------------------------------

/// 1-pole low-pass (discretized RC filter).
#[derive(Debug, Default)]
pub struct LowPass {
    y: f32,
}

impl LowPass {
    /// Advance one step: `cutoff_hz` in Hz, `dt` in seconds. Non-positive
    /// cutoff or dt degrades to pass-through; non-finite input is ignored.
    pub fn step(&mut self, x: f32, cutoff_hz: f32, dt: f32) -> f32 {
        if !x.is_finite() {
            return self.y;
        }
        if cutoff_hz <= 0.0 || dt <= 0.0 {
            self.y = x;
            return x;
        }
        let alpha = 1.0 - (-std::f32::consts::TAU * cutoff_hz * dt).exp();
        self.y += (x - self.y) * alpha;
        self.y
    }
}

/// Caps effect updates at `update_hz` (DirectInput/USB writes are not free).
#[derive(Debug)]
pub struct RateLimiter {
    interval: Duration,
    next: Option<Instant>,
}

impl RateLimiter {
    pub fn new(interval: Duration) -> RateLimiter {
        RateLimiter { interval, next: None }
    }

    /// True when a send is due at `now`; schedules the following slot.
    pub fn ready(&mut self, now: Instant) -> bool {
        match self.next {
            Some(t) if now < t => false,
            _ => {
                self.next = Some(now + self.interval);
                true
            }
        }
    }
}

/// Map filtered rack torque (Nm) to a DirectInput constant-force magnitude in
/// [-10000, 10000]: normalize by gain / max_torque_nm, apply the min_force
/// floor (deadzone crossing), then invert. Pure function — see unit tests.
pub fn torque_to_magnitude(torque_nm: f32, cfg: &FfbConfig) -> i32 {
    if !torque_nm.is_finite() || cfg.max_torque_nm <= 0.0 {
        return 0;
    }
    let mut n = torque_nm * cfg.gain / cfg.max_torque_nm;
    if cfg.invert {
        n = -n;
    }
    let a = n.abs();
    if a < MIN_FORCE_NOISE_EPS {
        return 0;
    }
    let shaped = (cfg.min_force + a * (1.0 - cfg.min_force)).clamp(0.0, 1.0);
    (n.signum() * shaped * DI_MAX_MAGNITUDE).round() as i32
}

// ---------------------------------------------------------------------------
// Controller (owned by the race app; one per window)
// ---------------------------------------------------------------------------

/// Per-frame FFB driver. Owns the (optional) DirectInput device; when no
/// device is present every call is a cheap no-op, so the race loop never has
/// to care whether FFB is live.
pub struct Ffb {
    cfg: FfbConfig,
    filter: LowPass,
    limiter: RateLimiter,
    device: Option<device::Device>,
    last_magnitude: i32,
    /// False while updates are failing (device lost and unrecoverable this
    /// frame) — used to log state *transitions* instead of spamming per frame.
    link_ok: bool,
}

impl Ffb {
    /// Open the first force-feedback game controller and start a zero-strength
    /// constant-force effect on it. Never fails: any problem (no device, no
    /// FFB support, non-Windows platform) logs one warning and yields an
    /// inert controller.
    pub fn new(cfg: FfbConfig, window: &winit::window::Window) -> Ffb {
        let device = if cfg.enabled {
            match device::Device::open(window) {
                Ok(d) => {
                    println!(
                        "ffb: constant force on '{}' (full force = {} Nm rack torque, gain {}, {} Hz updates)",
                        d.name(),
                        cfg.max_torque_nm,
                        cfg.gain,
                        cfg.update_hz
                    );
                    Some(d)
                }
                Err(e) => {
                    eprintln!("ffb: unavailable — {e:#}; racing without force feedback");
                    None
                }
            }
        } else {
            println!("ffb: disabled (ffb.toml / --no-ffb)");
            None
        };
        Ffb {
            filter: LowPass::default(),
            limiter: RateLimiter::new(Duration::from_secs_f32(1.0 / cfg.update_hz.max(1.0))),
            device,
            last_magnitude: 0,
            link_ok: true,
            cfg,
        }
    }

    /// Feed one render frame's rack torque (Nm) through the filter chain and
    /// out to the wheel. `dt` = seconds since the previous render frame.
    pub fn frame(&mut self, torque_nm: f32, dt: f32) {
        let filtered = self.filter.step(torque_nm, self.cfg.smoothing_hz, dt);
        let Some(device) = self.device.as_mut() else { return };
        if !self.limiter.ready(Instant::now()) {
            return;
        }
        let magnitude = torque_to_magnitude(filtered, &self.cfg);
        if magnitude == self.last_magnitude && self.link_ok {
            return; // nothing new for the wheel; skip the USB round-trip
        }
        match device.set_magnitude(magnitude) {
            Ok(()) => {
                self.last_magnitude = magnitude;
                if !self.link_ok {
                    println!("ffb: device reacquired, forces restored");
                    self.link_ok = true;
                }
            }
            Err(e) => {
                if self.link_ok {
                    eprintln!("ffb: update failed ({e:#}); retrying in the background");
                    self.link_ok = false;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// DirectInput8 device plumbing — Windows only
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod device {
    //! DirectInput8 constant-force effect host.
    //!
    //! Uses the `windows` crate (Win32::Devices::HumanInterfaceDevice), which
    //! has everything except `c_dfDIJoystick2` — that is a *data* export of
    //! dinput8.lib which win32metadata cannot describe, so instead of
    //! hardcoding its 164-entry object table we register a minimal custom
    //! DIDATAFORMAT with a single absolute X axis. That is all FFB needs:
    //! input is read by gilrs, not DirectInput; the data format only has to
    //! name the steering axis so the device can be acquired and the effect
    //! can address axis offset 0.

    use std::ffi::c_void;
    use std::mem::size_of;

    use anyhow::{anyhow, Context, Result};
    use windows::core::{Interface, BOOL, GUID, HRESULT, PCWSTR};
    use windows::Win32::Devices::HumanInterfaceDevice::{
        DirectInput8Create, IDirectInput8W, IDirectInputDevice8W, IDirectInputEffect,
        DICONSTANTFORCE, DIDATAFORMAT, DIDEVICEINSTANCEW, DIEB_NOTRIGGER, DIEDFL_ATTACHEDONLY,
        DIEDFL_FORCEFEEDBACK, DIEFFECT, DIEFF_CARTESIAN, DIEFF_OBJECTOFFSETS, DIENUM_CONTINUE,
        DIEP_START, DIEP_TYPESPECIFICPARAMS, DIERR_INPUTLOST, DIERR_NOTACQUIRED,
        DIERR_NOTEXCLUSIVEACQUIRED, DIDFT_ABSAXIS, DIDFT_ANYINSTANCE, DIDF_ABSAXIS,
        DIOBJECTDATAFORMAT, DIPH_DEVICE, DIPROPAUTOCENTER_OFF, DIPROPDWORD, DIPROPHEADER,
        DIRECTINPUT_VERSION, DISCL_BACKGROUND, DISCL_EXCLUSIVE, DI8DEVCLASS_GAMECTRL,
        DI_FFNOMINALMAX, GUID_ConstantForce, GUID_XAxis,
    };
    use windows::Win32::Foundation::{HINSTANCE, HWND};
    use windows::Win32::System::LibraryLoader::GetModuleHandleW;

    /// DIPROP_AUTOCENTER the way DirectInput actually consumes predefined
    /// properties: MAKEDIPROP(9) is the integer 9 cast to REFGUID — a *fake
    /// pointer*. The `windows` crate exposes it as a real GUID constant whose
    /// address must NOT be passed (dinput checks the pointer value itself).
    const DIPROP_AUTOCENTER_FAKEPTR: *const GUID = 9 as *const GUID;

    pub struct Device {
        // Field order is drop order: effect, then device, then the API object.
        effect: IDirectInputEffect,
        dev: IDirectInputDevice8W,
        _di: IDirectInput8W,
        name: String,
    }

    struct EnumHit {
        guid: GUID,
        name: String,
    }

    unsafe extern "system" fn enum_ffb_devices(
        inst: *mut DIDEVICEINSTANCEW,
        ctx: *mut c_void,
    ) -> BOOL {
        let hits = unsafe { &mut *(ctx as *mut Vec<EnumHit>) };
        let inst = unsafe { &*inst };
        let end = inst
            .tszProductName
            .iter()
            .position(|&c| c == 0)
            .unwrap_or(inst.tszProductName.len());
        hits.push(EnumHit {
            guid: inst.guidInstance,
            name: String::from_utf16_lossy(&inst.tszProductName[..end]),
        });
        BOOL(DIENUM_CONTINUE as i32)
    }

    fn win32_hwnd(window: &winit::window::Window) -> Option<HWND> {
        use winit::raw_window_handle::{HasWindowHandle, RawWindowHandle};
        match window.window_handle().ok()?.as_raw() {
            RawWindowHandle::Win32(h) => Some(HWND(h.hwnd.get() as *mut c_void)),
            _ => None,
        }
    }

    impl Device {
        pub fn name(&self) -> &str {
            &self.name
        }

        /// Full acquisition sequence per docs/FFB.md: DirectInput8Create ->
        /// enumerate FF game controllers -> CreateDevice -> SetDataFormat ->
        /// SetCooperativeLevel(EXCLUSIVE|BACKGROUND) -> autocenter off ->
        /// Acquire -> CreateEffect(constant force, one X axis, infinite).
        pub fn open(window: &winit::window::Window) -> Result<Device> {
            let hwnd =
                win32_hwnd(window).context("window has no Win32 handle (FFB needs one)")?;
            unsafe {
                let hmodule = GetModuleHandleW(PCWSTR::null()).context("GetModuleHandleW")?;
                let mut raw: *mut c_void = std::ptr::null_mut();
                DirectInput8Create(
                    HINSTANCE(hmodule.0),
                    DIRECTINPUT_VERSION,
                    &IDirectInput8W::IID,
                    &mut raw,
                    None,
                )
                .context("DirectInput8Create")?;
                let di = IDirectInput8W::from_raw(raw);

                let mut hits: Vec<EnumHit> = Vec::new();
                di.EnumDevices(
                    DI8DEVCLASS_GAMECTRL,
                    Some(enum_ffb_devices),
                    &mut hits as *mut Vec<EnumHit> as *mut c_void,
                    DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK,
                )
                .context("EnumDevices(game controllers with force feedback)")?;
                if hits.len() > 1 {
                    let names: Vec<&str> = hits.iter().map(|h| h.name.as_str()).collect();
                    println!(
                        "ffb: {} force-feedback devices found ({}); using the first",
                        hits.len(),
                        names.join(", ")
                    );
                }
                let hit = hits.into_iter().next().ok_or_else(|| {
                    anyhow!(
                        "no force-feedback device found \
                         (wheel plugged in, powered, and driver installed?)"
                    )
                })?;

                let mut dev_opt: Option<IDirectInputDevice8W> = None;
                di.CreateDevice(&hit.guid, &mut dev_opt, None)
                    .with_context(|| format!("CreateDevice('{}')", hit.name))?;
                let dev = dev_opt.ok_or_else(|| anyhow!("CreateDevice returned no device"))?;

                // Minimal custom data format: one absolute X axis at offset 0
                // (see module docs for why we do not use c_dfDIJoystick2).
                let mut obj = DIOBJECTDATAFORMAT {
                    pguid: &GUID_XAxis,
                    dwOfs: 0,
                    dwType: DIDFT_ABSAXIS | DIDFT_ANYINSTANCE,
                    dwFlags: 0,
                };
                let mut fmt = DIDATAFORMAT {
                    dwSize: size_of::<DIDATAFORMAT>() as u32,
                    dwObjSize: size_of::<DIOBJECTDATAFORMAT>() as u32,
                    dwFlags: DIDF_ABSAXIS,
                    dwDataSize: 4, // one DWORD-sized axis slot
                    dwNumObjs: 1,
                    rgodf: &mut obj,
                };
                dev.SetDataFormat(&mut fmt).context("SetDataFormat(X axis)")?;

                // Exclusive is REQUIRED for FFB output; background keeps the
                // forces alive across alt-tab instead of killing the effect.
                dev.SetCooperativeLevel(hwnd, DISCL_EXCLUSIVE | DISCL_BACKGROUND)
                    .context("SetCooperativeLevel(exclusive|background)")?;

                // Best-effort: switch off the driver's autocenter spring so it
                // does not fight the physics torque (many drivers ignore this).
                let mut prop = DIPROPDWORD {
                    diph: DIPROPHEADER {
                        dwSize: size_of::<DIPROPDWORD>() as u32,
                        dwHeaderSize: size_of::<DIPROPHEADER>() as u32,
                        dwObj: 0,
                        dwHow: DIPH_DEVICE,
                    },
                    dwData: DIPROPAUTOCENTER_OFF,
                };
                let _ = dev.SetProperty(DIPROP_AUTOCENTER_FAKEPTR, &mut prop.diph);

                dev.Acquire().context(
                    "Acquire(exclusive) — another program may hold the wheel \
                     (Fanatec app, another sim); close it and retry",
                )?;

                // Infinite-duration constant force on the X axis, starting at
                // zero magnitude. Per-frame updates only touch lMagnitude.
                let mut axes = [0u32]; // DIEFF_OBJECTOFFSETS: X axis = data offset 0
                let mut dirs = [0i32]; // single axis: sign lives in lMagnitude
                let mut cf = DICONSTANTFORCE { lMagnitude: 0 };
                let mut eff = DIEFFECT {
                    dwSize: size_of::<DIEFFECT>() as u32,
                    dwFlags: DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS,
                    dwDuration: u32::MAX, // INFINITE
                    dwSamplePeriod: 0,
                    dwGain: DI_FFNOMINALMAX,
                    dwTriggerButton: DIEB_NOTRIGGER,
                    dwTriggerRepeatInterval: 0,
                    cAxes: 1,
                    rgdwAxes: axes.as_mut_ptr(),
                    rglDirection: dirs.as_mut_ptr(),
                    lpEnvelope: std::ptr::null_mut(),
                    cbTypeSpecificParams: size_of::<DICONSTANTFORCE>() as u32,
                    lpvTypeSpecificParams: &mut cf as *mut DICONSTANTFORCE as *mut c_void,
                    dwStartDelay: 0,
                };
                let mut effect_opt: Option<IDirectInputEffect> = None;
                dev.CreateEffect(&GUID_ConstantForce, &mut eff, &mut effect_opt, None)
                    .context("CreateEffect(GUID_ConstantForce)")?;
                let effect =
                    effect_opt.ok_or_else(|| anyhow!("CreateEffect returned no effect"))?;
                if let Err(e) = effect.Start(1, 0) {
                    // Non-fatal: DIEP_START on the first update starts it too.
                    eprintln!("ffb: effect start deferred ({e})");
                }

                Ok(Device { effect, dev, _di: di, name: hit.name })
            }
        }

        /// Push a new constant-force magnitude ([-10000, 10000]). On
        /// DIERR_INPUTLOST / DIERR_NOTACQUIRED / DIERR_NOTEXCLUSIVEACQUIRED
        /// (the classic focus-loss failure) the device is reacquired and the
        /// update retried once.
        pub fn set_magnitude(&mut self, magnitude: i32) -> Result<()> {
            const LOST: [HRESULT; 3] = [
                DIERR_INPUTLOST,
                DIERR_NOTACQUIRED,
                HRESULT(DIERR_NOTEXCLUSIVEACQUIRED),
            ];
            match self.try_set(magnitude) {
                Err(code) if LOST.contains(&code) => {
                    unsafe { self.dev.Acquire() }
                        .map_err(|e| anyhow!("device lost; reacquire failed: {e}"))?;
                    self.try_set(magnitude)
                        .map_err(|c| anyhow!("update failed after reacquire: {c:?}"))
                }
                Err(code) => Err(anyhow!("SetParameters failed: {code:?}")),
                Ok(()) => Ok(()),
            }
        }

        fn try_set(&mut self, magnitude: i32) -> std::result::Result<(), HRESULT> {
            let mut cf = DICONSTANTFORCE { lMagnitude: magnitude };
            // With DIEP_TYPESPECIFICPARAMS only dwSize + the two type-specific
            // fields are read; everything else stays zeroed.
            let mut eff = DIEFFECT {
                dwSize: size_of::<DIEFFECT>() as u32,
                cbTypeSpecificParams: size_of::<DICONSTANTFORCE>() as u32,
                lpvTypeSpecificParams: &mut cf as *mut DICONSTANTFORCE as *mut c_void,
                ..Default::default()
            };
            unsafe {
                self.effect
                    .SetParameters(&mut eff, DIEP_TYPESPECIFICPARAMS | DIEP_START)
            }
            .map_err(|e| e.code())
        }
    }

    impl Drop for Device {
        fn drop(&mut self) {
            // Leave the wheel limp, not pulling: zero the force, stop, release.
            let _ = self.try_set(0);
            unsafe {
                let _ = self.effect.Stop();
                let _ = self.dev.Unacquire();
            }
        }
    }
}

#[cfg(not(windows))]
mod device {
    //! No-op stub: DirectInput FFB is Windows-only. (SDL3 haptics is the
    //! designated portable fallback per docs/FFB.md §2 — not implemented yet.)

    use anyhow::{bail, Result};

    pub struct Device {}

    impl Device {
        pub fn name(&self) -> &str {
            ""
        }
        pub fn open(_window: &winit::window::Window) -> Result<Device> {
            bail!("force feedback is only implemented on Windows (DirectInput8)")
        }
        pub fn set_magnitude(&mut self, _magnitude: i32) -> Result<()> {
            Ok(())
        }
    }
}

// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- config -------------------------------------------------------------

    #[test]
    fn default_file_parses_to_defaults() {
        let cfg = FfbConfig::from_toml_str(DEFAULT_FFB_TOML).unwrap();
        assert_eq!(cfg, FfbConfig::default());
    }

    #[test]
    fn docs_ffb_md_example_parses() {
        // The exact block from docs/FFB.md §3.
        let cfg = FfbConfig::from_toml_str(
            r#"
[ffb]
enabled      = true
max_torque_nm = 8.0
gain          = 0.85
min_force     = 0.02
damping       = 0.10
friction      = 0.03
smoothing_hz  = 120
standstill_reduction = true
kerb_vibration_gain  = 0.5
understeer_drop      = 1.0
invert               = false
update_hz            = 400
"#,
        )
        .unwrap();
        assert_eq!(cfg, FfbConfig::default());
    }

    #[test]
    fn partial_and_empty_files_use_defaults() {
        let cfg = FfbConfig::from_toml_str("[ffb]\nenabled = false\ngain = 1.0\n").unwrap();
        assert!(!cfg.enabled);
        assert_eq!(cfg.gain, 1.0);
        assert_eq!(cfg.max_torque_nm, 8.0); // untouched fields keep defaults

        let cfg = FfbConfig::from_toml_str("").unwrap();
        assert_eq!(cfg, FfbConfig::default());
    }

    #[test]
    fn malformed_toml_is_an_error() {
        assert!(FfbConfig::from_toml_str("[ffb\nenabled = ").is_err());
        assert!(FfbConfig::from_toml_str("[ffb]\ngain = \"loud\"").is_err());
    }

    #[test]
    fn hostile_values_are_clamped() {
        let cfg = FfbConfig::from_toml_str(
            "[ffb]\ngain = 99.0\nmin_force = 0.9\nmax_torque_nm = 0.0\nupdate_hz = 1\nsmoothing_hz = -5\n",
        )
        .unwrap();
        assert_eq!(cfg.gain, 3.0);
        assert_eq!(cfg.min_force, 0.5);
        assert_eq!(cfg.max_torque_nm, 0.5); // never zero => no divide-by-zero
        assert_eq!(cfg.update_hz, 10.0);
        assert_eq!(cfg.smoothing_hz, 1.0);
        let cfg = FfbConfig::from_toml_str("[ffb]\ngain = nan\n").unwrap();
        assert_eq!(cfg.gain, 0.0);
    }

    // -- torque -> magnitude mapping -----------------------------------------

    fn cfg() -> FfbConfig {
        FfbConfig::default() // max 8 Nm, gain 0.85, min_force 0.02, no invert
    }

    #[test]
    fn zero_torque_is_zero_force() {
        assert_eq!(torque_to_magnitude(0.0, &cfg()), 0);
    }

    #[test]
    fn mapping_scales_by_gain_over_max_torque() {
        // 4 Nm: n = 4 * 0.85 / 8 = 0.425 -> floor: 0.02 + 0.425*0.98 = 0.4365
        assert_eq!(torque_to_magnitude(4.0, &cfg()), 4365);
        assert_eq!(torque_to_magnitude(-4.0, &cfg()), -4365);
    }

    #[test]
    fn mapping_clamps_at_full_scale() {
        assert_eq!(torque_to_magnitude(100.0, &cfg()), 10_000);
        assert_eq!(torque_to_magnitude(-100.0, &cfg()), -10_000);
    }

    #[test]
    fn invert_flips_sign() {
        let mut c = cfg();
        c.invert = true;
        assert_eq!(torque_to_magnitude(4.0, &c), -4365);
        assert_eq!(torque_to_magnitude(-4.0, &c), 4365);
    }

    #[test]
    fn min_force_floor_lifts_small_signals() {
        // 0.01 Nm -> n ~= 0.0010625: above noise eps, so floored near 2%.
        let m = torque_to_magnitude(0.01, &cfg());
        assert!(m >= 200, "floor not applied: {m}");
        // but numeric dust below the noise eps stays exactly zero
        assert_eq!(torque_to_magnitude(1e-6, &cfg()), 0);
        // min_force = 0 keeps pure proportional mapping
        let mut c = cfg();
        c.min_force = 0.0;
        assert_eq!(torque_to_magnitude(4.0, &c), 4250); // 0.425 * 10000
    }

    #[test]
    fn non_finite_torque_is_zero_force() {
        // is_finite() gate: NAN/INF must not reach the wheel as garbage.
        assert_eq!(torque_to_magnitude(f32::NAN, &cfg()), 0);
        assert_eq!(torque_to_magnitude(f32::INFINITY, &cfg()), 0);
        assert_eq!(torque_to_magnitude(f32::NEG_INFINITY, &cfg()), 0);
    }

    // -- low-pass -------------------------------------------------------------

    #[test]
    fn lowpass_time_constant() {
        // cutoff = 1/tau Hz with tau = 2*pi s => alpha = 1 - e^-1 after 1 s.
        let mut lp = LowPass::default();
        let fc = 1.0 / std::f32::consts::TAU;
        let y = lp.step(1.0, fc, 1.0);
        assert!((y - 0.632).abs() < 0.001, "one time constant ~63.2%, got {y}");
    }

    #[test]
    fn lowpass_converges_and_high_cutoff_passes_through() {
        let mut lp = LowPass::default();
        for _ in 0..10_000 {
            lp.step(2.5, 120.0, 1.0 / 300.0);
        }
        assert!((lp.step(2.5, 120.0, 1.0 / 300.0) - 2.5).abs() < 1e-4);

        let mut lp = LowPass::default();
        let y = lp.step(1.0, 100_000.0, 0.01); // cutoff >> frame rate
        assert!((y - 1.0).abs() < 1e-4);
    }

    #[test]
    fn lowpass_zero_cutoff_or_dt_passes_through_and_nan_is_ignored() {
        let mut lp = LowPass::default();
        assert_eq!(lp.step(3.0, 0.0, 0.016), 3.0);
        let before = lp.step(1.0, 120.0, 0.016);
        assert_eq!(lp.step(f32::NAN, 120.0, 0.016), before);
    }

    // -- rate limiter ---------------------------------------------------------

    #[test]
    fn rate_limiter_spaces_sends() {
        let mut rl = RateLimiter::new(Duration::from_millis(10));
        let t0 = Instant::now();
        assert!(rl.ready(t0), "first call is always due");
        assert!(!rl.ready(t0 + Duration::from_millis(5)));
        assert!(rl.ready(t0 + Duration::from_millis(10)));
        assert!(!rl.ready(t0 + Duration::from_millis(19)));
        assert!(rl.ready(t0 + Duration::from_millis(25)));
    }
}

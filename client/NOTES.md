# Client workstream notes

Decision log for the native Rust client (`client/`). Newest sections at the
bottom.

## DirectInput FFB

Implemented 2026-07-08 per docs/FFB.md: constant-force output for direct-drive
wheels (target rig: Fanatec CSL DD, 8 Nm), driven by the kernel's
`sim_ffb_torque()` export (steering rack torque in Nm at 400 Hz).

**Files**: `src/ffb.rs` (new), plus small hooks in `sim.rs` (optional typed
func), `main.rs` (`--no-ffb`, config load), `render.rs` (per-frame update),
`signing.rs` (`config_dir()` made `pub(crate)` so ffb.toml lives next to the
signing key).

### windows-rs vs manual FFI

Chose the `windows` crate (0.62, already in the dependency tree transitively
via wgpu/winit — zero new source downloads), target-gated to
`cfg(windows)` with features `Win32_Devices_HumanInterfaceDevice`,
`Win32_Foundation`, `Win32_System_LibraryLoader`. Verified against the crate
source that it provides everything needed: `DirectInput8Create`,
`IDirectInput8W`/`IDirectInputDevice8W`/`IDirectInputEffect`, `DIEFFECT`/
`DICONSTANTFORCE`/`DIDATAFORMAT` structs, and all `DI*` flag/error constants.
Two gaps, both worked around locally rather than dropping to raw FFI:

1. **`c_dfDIJoystick2` does not exist in windows-rs.** It is a *data* export
   of dinput8.lib, which win32metadata cannot describe. Rather than hardcode
   its 164-entry object table (or fight MSVC import-lib semantics for extern
   data statics), we register a minimal custom `DIDATAFORMAT` containing a
   single absolute X axis at offset 0 (`GUID_XAxis`,
   `DIDFT_ABSAXIS|DIDFT_ANYINSTANCE`, `DIDF_ABSAXIS`, dwDataSize 4). This is a
   documented use of `SetDataFormat` and is sufficient because we never *read*
   input through DirectInput (gilrs does that) — the format only has to name
   the steering axis so the device can be acquired and `CreateEffect` can
   address the axis by data-format offset (`DIEFF_OBJECTOFFSETS`, offset 0).
2. **`DIPROP_AUTOCENTER` is exposed as a real GUID constant, which is a
   trap.** In dinput.h the predefined properties are `MAKEDIPROP(n)` — an
   integer cast to `REFGUID`, i.e. a fake pointer whose *value* is n.
   Passing the address of windows-rs's GUID constant would be wrong; we pass
   `9 as *const GUID` directly (autocenter off is best-effort, result
   ignored — many wheel drivers don't implement it).

### Acquisition sequence

`DirectInput8Create` → `EnumDevices(DI8DEVCLASS_GAMECTRL,
DIEDFL_ATTACHEDONLY|DIEDFL_FORCEFEEDBACK)` (first hit wins; all hits logged) →
`CreateDevice` → `SetDataFormat`(custom X-axis format) →
`SetCooperativeLevel(hwnd, DISCL_EXCLUSIVE | DISCL_BACKGROUND)` — exclusive is
*required* for FFB, background keeps forces alive across alt-tab → autocenter
off (best-effort) → `Acquire` → `CreateEffect(GUID_ConstantForce)` with one X
axis, infinite duration, gain `DI_FFNOMINALMAX`, magnitude 0, then `Start(1,0)`.

The HWND comes from winit via the `raw_window_handle` 0.6 re-export
(`window.window_handle().as_raw()` → `RawWindowHandle::Win32`), so the FFB
controller is created in `resumed()` right after window creation.

### Per-frame signal path (render.rs `frame()` step 2.5)

```
torque_nm = sim.ffb_torque()                       (0.0 on old kernels)
  -> 1-pole low-pass, cutoff smoothing_hz           (alpha = 1 - e^(-2π·fc·dt))
  -> n = torque · gain / max_torque_nm; invert ? -n
  -> |n| < 1e-4  ->  0                              (noise gate)
     else magnitude = sign(n) · min(1, min_force + |n|·(1 - min_force)) · 10000
  -> SetParameters(DIEP_TYPESPECIFICPARAMS | DIEP_START), rate-limited to
     update_hz, identical magnitudes skipped
```

The mapping (`torque_to_magnitude`), filter (`LowPass`) and limiter
(`RateLimiter`) are pure and unit-tested; only the ~150 lines of DirectInput
plumbing are untestable without hardware.

**Focus-loss handling** (the classic bug): on `DIERR_INPUTLOST`,
`DIERR_NOTACQUIRED`, or `DIERR_NOTEXCLUSIVEACQUIRED` the update path calls
`Acquire()` and retries once. Failures log on *state transitions* only (lost
once, recovered once) — no per-frame spam. On drop the device zeroes the
force, stops the effect, and unacquires, so the wheel is left limp, not
pulling.

### Config

`ffb.toml` in the OS config dir (same directory as `key.bin`;
`%APPDATA%\sttr-client\config\` on Windows). Written with full comments on
first run; delete to regenerate. All 12 keys from docs/FFB.md §3 parse;
`damping`, `friction`, `standstill_reduction`, `kerb_vibration_gain`, and
`understeer_drop` are parsed-but-reserved (the understeer cue is kernel-side
pneumatic-trail collapse; the damper/friction/kerb layers need effect types we
haven't built yet). Every value is clamped to a sane range on load
(`max_torque_nm` can never be 0 → no divide-by-zero) and a malformed file
falls back to defaults with a warning — config can never take the race down.
`--no-ffb` on the `race` subcommand overrides `enabled`.

**Determinism**: FFB is read-only on sim state. `sim_ffb_torque()` is queried
once per render frame, after stepping; nothing feeds back into the input log
except through the driver's hands. No ABI, laplog, or replay impact
(`sim_ffb_torque` is an *optional* export — `Sim::ffb_torque()` returns 0.0
on kernels that predate it).

**Deferred**: hot-reload of ffb.toml (design doc mentions it; needs a watcher
or a poll — trivial to add later), damper/friction/kerb effect layers, SDL3
haptics as the portable fallback for non-Windows.

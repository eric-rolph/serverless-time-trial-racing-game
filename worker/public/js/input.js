// Input: keyboard fallback, zero-config standard controllers, and multi-device
// wheel rigs via per-channel calibration. Sim rigs are several USB devices
// (wheel base, pedals, handbrake...) — each channel binds to ITS OWN device:
// detection scans the axes of EVERY connected gamepad and picks whichever
// (device, axis) moved the most. Bindings persist by device id string, so a
// steering wheel, Heusinkveld pedals, and a handbrake all coexist.
//
// Browser quirk: a device only appears in navigator.getGamepads() after you
// interact with it once (press/turn it) — the setup panel says so.

import { cancelButtonCapture, readReportBit, startButtonCapture } from "./hid-input.js";

const CAL_KEY = "sttr-input-cal-v2"; // v1 was single-device; ignored on load
const STEER_KEY = "sttr-steer-settings"; // {invert, sensitivity}
const GEARBOX_KEY = "sttr-shift-mode"; // "auto" | "manual" (absent = auto, the default; key shared with the HUD's fallback read)

// Axis channels (per-device analog detect). Shifter buttons ("shiftUp" /
// "shiftDown") live in the same calibration blob but bind (reportId, byte,
// bit) via hid-input's capture flow, not an axis.
export const CHANNELS = ["steer", "throttle", "brake", "handbrake"];

// Input flags bitfield per CONTRACTS §2. The kernel EDGE-DETECTS bits 1/2
// (DRIVETRAIN.md §1) — the client reports held button state as-is; a rising
// edge in the log is one shift request.
const FLAG_HANDBRAKE = 1;
const FLAG_SHIFT_UP = 2;
const FLAG_SHIFT_DOWN = 4;

// Client auto-shifter (DRIVETRAIN.md §6): emits the same logged shift bits
// from sim_rpm()/sim_gear() thresholds — still just input, replay-honest.
// GEAR_RATIOS mirrors the kernel's table (DRIVETRAIN.md §6.5 item 2; index
// 1..6, [0] = |reverse|) so a doomed downshift (matched rpm past the kernel's
// 7800 refuse line) is skipped instead of spamming refused requests.
const GEAR_RATIOS = [9.8, 9.8, 7.9, 6.06, 4.77, 3.75, 2.95];
const AUTO_UP_RPM = 7200;
const AUTO_DOWN_RPM = 3800;
const AUTO_MATCH_LIMIT_RPM = 7600; // mirror of the kernel's 7800, with margin
const AUTO_PULSE_S = 0.02; // bit held ~8 ticks — one clean rising edge
const AUTO_COOLDOWN_S = 0.25; // > the 120 ms downshift engagement; hysteresis

export class Input {
  constructor() {
    this.keys = new Set();
    this.kSteer = 0; // slewed keyboard steering
    this.kBrake = 0; // slewed keyboard brake (rise ~200ms, fast release)
    this.cal = null; // { steer: {id, axis, rest, ext}, throttle: {...}, ... }
    try {
      const stored = JSON.parse(localStorage.getItem(CAL_KEY) ?? "null");
      if (stored && Object.values(stored).every((c) => typeof c?.id === "string")) this.cal = stored;
    } catch {
      /* corrupted -> recalibrate */
    }
    this.calibrating = null;
    this.onCalDone = null;
    // Steering feel: invert (rigs/axes vary) and sensitivity — the ratio of
    // wheel-axis travel to full car lock. sensitivity 2 = half the physical
    // rotation reaches full lock (for high-rotation-range DD bases).
    this.steerSettings = { invert: false, sensitivity: 1.0, ...JSON.parse(localStorage.getItem(STEER_KEY) ?? "{}") };
    // One-time migration: rig axes now default inverted at the source (see
    // sample()); a pre-flip stored invert=true would double-flip, so land
    // existing installs on the corrected default once and mark the blob.
    if (!this.steerSettings.sign2) {
      this.steerSettings.invert = false;
      this.steerSettings.sign2 = true;
      localStorage.setItem(STEER_KEY, JSON.stringify(this.steerSettings));
    }
    // Gearbox mode: AUTO by default; mapping a wheel shifter flips the
    // profile to MANUAL (DRIVETRAIN §6 — wheel users shift for themselves).
    this.gearAuto = (localStorage.getItem(GEARBOX_KEY) ?? "auto") !== "manual";
    this.simHandle = null; // attachSim(), or discovered via globalThis.__sttr.sim
    this.onGearMode = null; // optional hook(msg); default announce = #statusText
    this.autoPulseBits = 0;
    this.autoPulseT = 0;
    this.autoCooldown = 0;
    addEventListener("keydown", (e) => {
      if (e.target.closest?.("input,textarea")) return; // typing in a field, not driving
      if (e.repeat) return;
      if (e.code === "KeyG") this.setGearMode(!this.gearAuto);
      this.keys.add(e.code);
    });
    addEventListener("keyup", (e) => this.keys.delete(e.code));
    this.wireGearPanel();
  }

  /** Gearbox mode as the HUD consumes it (app.js shiftModeLabel). */
  get shiftMode() {
    return this.gearAuto ? "auto" : "manual";
  }

  /** Give the auto-shifter its rpm/gear source (the live player sim). Falls
   *  back to the window.__sttr debug handle when never called. */
  attachSim(sim) {
    this.simHandle = sim;
  }

  sim() {
    return this.simHandle ?? globalThis.__sttr?.sim ?? null;
  }

  /** Engine rpm — null when unattached or on a pre-1.4 wasm (auto-shifter
   *  then stays silent; the 1st-gear fallback still drives). */
  simRpm() {
    const s = this.sim();
    if (!s) return null;
    if (typeof s.rpm === "function") return s.rpm();
    return s.e?.sim_rpm ? s.e.sim_rpm() : null;
  }

  /** Current gear (0 = R, 1..6) — null when unavailable. */
  simGear() {
    const s = this.sim();
    if (!s) return null;
    if (typeof s.gear === "function") return s.gear();
    return s.e?.sim_gear ? s.e.sim_gear() : null;
  }

  setGearMode(auto, note) {
    this.gearAuto = !!auto;
    localStorage.setItem(GEARBOX_KEY, this.gearAuto ? "auto" : "manual");
    const box = typeof document !== "undefined" ? document.getElementById("autoGear") : null;
    if (box) box.checked = this.gearAuto;
    const msg = note ?? (this.gearAuto ? "gearbox: AUTO — G for manual" : "gearbox: MANUAL — Q down / E up, G for auto");
    if (this.onGearMode) this.onGearMode(msg);
    else {
      const st = typeof document !== "undefined" ? document.getElementById("statusText") : null;
      if (st) {
        st.textContent = msg;
        st.className = "info";
      }
    }
  }

  /** Held shift-button state across all sources: keyboard Q/E, standard-pad
   *  LB(4)/RB(5), and mapped wheel-shifter HID bits. Momentary state only —
   *  edge detection is the kernel's job (DRIVETRAIN §1). */
  readShiftBits(pads) {
    let up = this.keys.has("KeyE");
    let down = this.keys.has("KeyQ");
    const std = pads.find((g) => g.mapping === "standard");
    if (std) {
      down ||= std.buttons[4]?.pressed ?? false;
      up ||= std.buttons[5]?.pressed ?? false;
    }
    if (!up && this.cal?.shiftUp) up = readReportBit(this.cal.shiftUp);
    if (!down && this.cal?.shiftDown) down = readReportBit(this.cal.shiftDown);
    return (up ? FLAG_SHIFT_UP : 0) | (down ? FLAG_SHIFT_DOWN : 0);
  }

  /** Auto-gearbox: pulse a shift bit when sim_rpm crosses the thresholds.
   *  Pure input-side sugar — the pulses land in the same quantized log as a
   *  human press, so replays are identical by construction. Hysteresis =
   *  wide up/down gap + a cooldown longer than any shift in progress. */
  autoShiftBits(dtSec) {
    this.autoCooldown = Math.max(0, this.autoCooldown - dtSec);
    if (this.autoPulseT > 0) {
      this.autoPulseT -= dtSec;
      return this.autoPulseBits;
    }
    if (!this.gearAuto || this.autoCooldown > 0) return 0;
    const rpm = this.simRpm();
    const gear = this.simGear();
    if (rpm === null || gear === null || gear < 1) return 0; // no sim / old wasm / reverse is the driver's call
    let bits = 0;
    if (gear < 6 && rpm >= AUTO_UP_RPM) bits = FLAG_SHIFT_UP;
    else if (
      gear > 1 &&
      rpm <= AUTO_DOWN_RPM &&
      (rpm * GEAR_RATIOS[gear - 1]) / GEAR_RATIOS[gear] <= AUTO_MATCH_LIMIT_RPM
    ) {
      bits = FLAG_SHIFT_DOWN;
    }
    if (bits) {
      this.autoPulseBits = bits;
      this.autoPulseT = AUTO_PULSE_S;
      this.autoCooldown = AUTO_COOLDOWN_S;
    }
    return bits;
  }

  /** Wire the config-panel gearbox controls this module owns (checkbox +
   *  shifter capture-to-map buttons). No-ops when the panel isn't in the DOM
   *  (tests, tools). */
  wireGearPanel() {
    if (typeof document === "undefined") return;
    const $ = (id) => document.getElementById(id);
    const msg = (t) => {
      const el = $("calMsg");
      if (el) el.textContent = t;
    };
    const box = $("autoGear");
    if (box) {
      box.checked = this.gearAuto;
      box.addEventListener("change", () => this.setGearMode(box.checked));
    }
    const refresh = () => {
      for (const ch of ["shiftUp", "shiftDown"]) {
        const label = $(ch === "shiftUp" ? "bindShiftUp" : "bindShiftDown");
        const btn = $(ch === "shiftUp" ? "mapShiftUp" : "mapShiftDown");
        const c = this.cal?.[ch];
        if (label) {
          label.textContent = c
            ? `${c.id.replace("WebHID ", "").slice(0, 22)} — report ${c.reportId}, byte ${c.byte}, bit ${c.bit}`
            : "not bound";
        }
        if (btn) btn.textContent = c ? "re-map" : "map";
      }
    };
    this.refreshShiftUI = refresh; // clearCalibration() re-renders through this
    refresh();
    const wire = (btnId, ch, name) => {
      $(btnId)?.addEventListener("click", () => {
        cancelButtonCapture();
        clearTimeout(this.mapPromptTimer);
        clearTimeout(this.mapTimeoutTimer);
        const ok = startButtonCapture((binding) => {
          clearTimeout(this.mapPromptTimer);
          clearTimeout(this.mapTimeoutTimer);
          this.cal ??= {};
          this.cal[ch] = binding;
          localStorage.setItem(CAL_KEY, JSON.stringify(this.cal));
          refresh();
          msg(`${name} → "${binding.id.replace("WebHID ", "").slice(0, 22)}" report ${binding.reportId}, byte ${binding.byte}, bit ${binding.bit}`);
          // A mapped wheel shifter defaults the profile to MANUAL (§6).
          if (this.gearAuto) this.setGearMode(false, `${name.toLowerCase()} mapped — gearbox now MANUAL (G toggles auto)`);
        });
        if (!ok) {
          msg("no raw-HID devices — click 'add device via raw HID' first, then map");
          return;
        }
        msg("HOLD EVERYTHING STILL…");
        this.mapPromptTimer = setTimeout(() => msg(`${name}: press it now!`), 750);
        this.mapTimeoutTimer = setTimeout(() => {
          if (cancelButtonCapture()) {
            msg(`${name}: no button press seen — check the device's report count climbs in the line above, then retry`);
          }
        }, 6000);
      });
    };
    wire("mapShiftUp", "shiftUp", "MAP UPSHIFT");
    wire("mapShiftDown", "shiftDown", "MAP DOWNSHIFT");
  }

  /** Register a function returning gamepad-shaped objects — WebHID-backed
   *  devices that Chrome's Gamepad API hides (once WebHID-opened) or simply
   *  never delivers data for (known with some Fanatec bases/pedals). */
  setVirtualPads(fn) {
    this.virtualPadsFn = fn;
  }

  /** All connected input devices: native gamepads + WebHID virtual pads. */
  gamepads() {
    const pads = [...(navigator.getGamepads?.() ?? [])].filter((g) => g && g.connected);
    for (const v of this.virtualPadsFn?.() ?? []) if (v?.connected) pads.push(v);
    return pads;
  }

  padById(id, pads = this.gamepads()) {
    return pads.find((g) => g.id === id) ?? null;
  }

  /** Begin per-channel detection across ALL devices. Two phases:
   *  0-700 ms  "hold still" — axes that move on their own get blacklisted
   *            (some devices report rolling counters/noise as axes, which
   *            would otherwise win every detection race);
   *  0.7-3.2 s capture — largest deviation among quiet axes wins. */
  startCalibration(channel, onDone) {
    const pads = this.gamepads();
    if (!pads.length) return false;
    this.calibrating = {
      channel,
      baselines: new Map(pads.map((g) => [g.index, [...g.axes]])),
      noisy: new Set(),
      best: null,
      bestDev: 0,
      ticks: 0,
      padsAtStart: pads.map((g) => `${g.id.slice(0, 30)}(${g.axes.length}ax)`),
      t0: performance.now(),
    };
    this.onCalDone = onDone;
    return true;
  }

  /** Call every frame while a wizard is active. */
  tickCalibration() {
    const c = this.calibrating;
    if (!c) return;
    c.ticks++;
    const elapsed = performance.now() - c.t0;
    for (const gp of this.gamepads()) {
      let base = c.baselines.get(gp.index);
      if (!base) {
        c.baselines.set(gp.index, [...gp.axes]); // hot-plugged mid-calibration
        continue;
      }
      gp.axes.forEach((v, i) => {
        const dev = Math.abs(v - base[i]);
        const key = `${gp.index}:${i}`;
        if (elapsed < 700) {
          if (dev > 0.15) c.noisy.add(key); // self-moving axis: counter/noise
          return;
        }
        if (c.noisy.has(key)) return;
        if (dev > c.bestDev) {
          c.bestDev = dev;
          c.best = { id: gp.id, axis: i, rest: base[i], ext: v };
        }
      });
    }
    if (elapsed > 3200) {
      // Snapshot for remote diagnostics (sent via the panel's diag button).
      this.lastCalDebug = {
        channel: c.channel,
        ticks: c.ticks,
        elapsedMs: Math.round(elapsed),
        bestDev: +c.bestDev.toFixed(3),
        best: c.best ? { id: c.best.id, axis: c.best.axis } : null,
        noisy: [...c.noisy],
        padsAtStart: c.padsAtStart,
      };
      if (c.best && c.bestDev > 0.15) {
        this.cal ??= {};
        this.cal[c.channel] = c.best;
        localStorage.setItem(CAL_KEY, JSON.stringify(this.cal));
        const short = c.best.id.split("(")[0].trim().slice(0, 28);
        this.onCalDone?.(
          `${c.channel} → "${short}" axis ${c.best.axis}` +
            (c.noisy.size ? ` (ignored ${c.noisy.size} self-moving axes)` : ""),
        );
      } else {
        // Name the best candidate even when it missed the bar — this makes
        // "nothing happened" failures diagnosable at a glance.
        const seen = c.best
          ? `largest movement: "${c.best.id.replace("WebHID ", "").split("(")[0].trim().slice(0, 22)}" axis ${c.best.axis} moved ${c.bestDev.toFixed(2)} (needs > 0.15)`
          : "zero movement on every axis of every device";
        this.onCalDone?.(`${c.channel}: not bound — ${seen}; ${c.noisy.size} noisy axes ignored`);
      }
      this.calibrating = null;
    }
  }

  /** Human-readable current bindings for the setup panel. */
  bindingSummary() {
    if (!this.cal) return "no bindings yet";
    return Object.entries(this.cal)
      .map(([ch, c]) => {
        const dev = c.id.replace("WebHID ", "").split("(")[0].trim().slice(0, 18);
        // Axis channel vs shifter-button binding (reportId/byte/bit).
        return `${ch}: ${dev} ${c.axis !== undefined ? `ax${c.axis}` : `r${c.reportId} b${c.byte}.${c.bit}`}`;
      })
      .join(" · ");
  }

  /** Slew the keyboard brake toward 0/1 — same clamp pattern as kSteer, but
   *  asymmetric: full pedal takes ~200ms (rise 5.0/s), release is fast (12/s).
   *  Shaping happens BEFORE quantize(); the logged integers stay canonical. */
  slewBrake(target, dtSec) {
    const slew = (target > this.kBrake ? 5.0 : 12) * dtSec;
    this.kBrake += Math.max(-slew, Math.min(slew, target - this.kBrake));
    return this.kBrake;
  }

  /** Read one calibrated channel from its own device. null = unbound/missing. */
  readChannel(ch, signed, pads) {
    const c = this.cal?.[ch];
    if (!c) return null;
    const gp = this.padById(c.id, pads);
    if (!gp) return null;
    const v = gp.axes[c.axis] ?? c.rest;
    const span = c.ext - c.rest;
    if (Math.abs(span) < 1e-3) return null;
    const n = (v - c.rest) / span; // 1 = the direction moved during detect
    // Signed channel (steering): the wizard says "turn LEFT", and left is -1
    // in the sim convention — so the detect direction maps to -1. This makes
    // polarity deterministic regardless of the device's native axis sign.
    return signed ? Math.max(-1, Math.min(1, -n)) : Math.max(0, Math.min(1, n));
  }

  /** Latest sample as floats: steer -1..1, throttle/brake 0..1, plus the
   *  momentary shift buttons (booleans — quantize() packs them into flags). */
  sample(dtSec = 1 / 60) {
    // One navigator.getGamepads() snapshot per tick — threaded through helpers.
    // A single NaN dtSec would poison the slew accumulators permanently.
    if (!Number.isFinite(dtSec) || dtSec < 0) dtSec = 1 / 60;
    if (!Number.isFinite(this.kSteer)) this.kSteer = 0;
    if (!Number.isFinite(this.kBrake)) this.kBrake = 0;
    const pads = this.gamepads();
    const s = this.steerSettings;
    const shape = (v) => Math.max(-1, Math.min(1, v * s.sensitivity * (s.invert ? -1 : 1)));
    const kbBrakeTarget = this.keys.has("ArrowDown") || this.keys.has("KeyS") ? 1 : 0;
    // Shift bits are path-independent: manual buttons OR the auto-shifter's
    // pulses (manual presses still work in auto — the kernel arbitrates).
    const shiftBits = this.readShiftBits(pads) | this.autoShiftBits(dtSec);
    const shiftUp = (shiftBits & FLAG_SHIFT_UP) !== 0;
    const shiftDown = (shiftBits & FLAG_SHIFT_DOWN) !== 0;
    // Calibrated multi-device rig takes precedence — any bound AXIS channel
    // activates it (unbound channels read 0 / fall back to keys; a lone
    // shifter binding must not hijack a standard pad's axes).
    if (this.cal && CHANNELS.some((ch) => this.cal[ch])) {
      const boundIds = new Set(CHANNELS.map((ch) => this.cal?.[ch]?.id).filter(Boolean));
      const present = [...boundIds].filter((id) => this.padById(id, pads)).length;
      // Keyboard stays usable for unbound channels (e.g. steer on keys while
      // only pedals are bound).
      const kbSteer = (this.keys.has("ArrowLeft") || this.keys.has("KeyA") ? -1 : 0) +
                      (this.keys.has("ArrowRight") || this.keys.has("KeyD") ? 1 : 0);
      const rawSteer = this.readChannel("steer", true, pads);
      const rawBrake = this.readChannel("brake", false, pads);
      return {
        // Rig axes default INVERTED (−rawSteer): Fanatec bases report the
        // axis opposite to our world convention (user-verified on a CSL DD).
        // The invert setting stays a relative flip for rigs that differ.
        steer: rawSteer !== null ? shape(-rawSteer) : kbSteer,
        throttle: this.readChannel("throttle", false, pads) ?? (this.keys.has("ArrowUp") || this.keys.has("KeyW") ? 1 : 0),
        // Real pedal passes through untouched; only the keyboard fallback slews.
        brake: rawBrake !== null ? rawBrake : this.slewBrake(kbBrakeTarget, dtSec),
        handbrake: (this.readChannel("handbrake", false, pads) ?? 0) > 0.5 || this.keys.has("Space"),
        shiftUp,
        shiftDown,
        device: `rig: ${present}/${boundIds.size} bound device(s) present`,
      };
    }
    // Zero-config controllers: prefer a standard-mapped pad among ALL devices.
    const std = pads.find((g) => g.mapping === "standard");
    if (std) {
      const dz = (v) => (Math.abs(v) < 0.08 ? 0 : v);
      return {
        steer: shape(dz(std.axes[0] ?? 0)),
        throttle: std.buttons[7]?.value ?? 0,
        brake: std.buttons[6]?.value ?? 0,
        handbrake: std.buttons[0]?.pressed ?? false,
        shiftUp,
        shiftDown,
        device: `${std.id.slice(0, 40)} (standard mapping) — A = handbrake, LB/RB = shift`,
      };
    }
    // Keyboard: slewed steering and brake so it's actually drivable.
    const target = (this.keys.has("ArrowLeft") || this.keys.has("KeyA") ? -1 : 0) +
                   (this.keys.has("ArrowRight") || this.keys.has("KeyD") ? 1 : 0);
    const slew = 3.0 * dtSec;
    this.kSteer += Math.max(-slew, Math.min(slew, target - this.kSteer));
    if (target === 0) this.kSteer *= Math.max(0, 1 - 6 * dtSec);
    return {
      steer: this.kSteer,
      throttle: this.keys.has("ArrowUp") || this.keys.has("KeyW") ? 1 : 0,
      brake: this.slewBrake(kbBrakeTarget, dtSec),
      handbrake: this.keys.has("Space"),
      shiftUp,
      shiftDown,
      device: pads.length ? `keyboard (${pads.length} device(s) seen — press I to bind)` : "keyboard",
    };
  }

  setSteerSettings(patch) {
    this.steerSettings = { ...this.steerSettings, ...patch };
    localStorage.setItem(STEER_KEY, JSON.stringify(this.steerSettings));
  }

  clearCalibration() {
    this.cal = null; // includes shiftUp/shiftDown button bindings
    localStorage.removeItem(CAL_KEY);
    this.refreshShiftUI?.();
  }
}

/** Quantize per CONTRACTS §2 — these integers are the canonical truth.
 *  flags: bit 0 = handbrake, bit 1 = shift up, bit 2 = shift down (held
 *  state; the kernel edge-detects, DRIVETRAIN §1). */
export function quantize(raw) {
  return {
    steer: Math.max(-32767, Math.min(32767, Math.round(raw.steer * 32767))),
    throttle: Math.max(0, Math.min(65535, Math.round(raw.throttle * 65535))),
    brake: Math.max(0, Math.min(65535, Math.round(raw.brake * 65535))),
    flags: (raw.handbrake ? FLAG_HANDBRAKE : 0) | (raw.shiftUp ? FLAG_SHIFT_UP : 0) | (raw.shiftDown ? FLAG_SHIFT_DOWN : 0),
  };
}

// Input: keyboard fallback, zero-config standard controllers, and multi-device
// wheel rigs via per-channel calibration. Sim rigs are several USB devices
// (wheel base, pedals, handbrake...) — each channel binds to ITS OWN device:
// detection scans the axes of EVERY connected gamepad and picks whichever
// (device, axis) moved the most. Bindings persist by device id string, so a
// steering wheel, Heusinkveld pedals, and a handbrake all coexist.
//
// Browser quirk: a device only appears in navigator.getGamepads() after you
// interact with it once (press/turn it) — the setup panel says so.

const CAL_KEY = "sttr-input-cal-v2"; // v1 was single-device; ignored on load

export const CHANNELS = ["steer", "throttle", "brake", "handbrake"];

export class Input {
  constructor() {
    this.keys = new Set();
    this.kSteer = 0; // slewed keyboard steering
    this.cal = null; // { steer: {id, axis, rest, ext}, throttle: {...}, ... }
    try {
      const stored = JSON.parse(localStorage.getItem(CAL_KEY) ?? "null");
      if (stored && Object.values(stored).every((c) => typeof c?.id === "string")) this.cal = stored;
    } catch {
      /* corrupted -> recalibrate */
    }
    this.calibrating = null;
    this.onCalDone = null;
    addEventListener("keydown", (e) => {
      if (!e.repeat) this.keys.add(e.code);
    });
    addEventListener("keyup", (e) => this.keys.delete(e.code));
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

  padById(id) {
    return this.gamepads().find((g) => g.id === id) ?? null;
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
      t0: performance.now(),
    };
    this.onCalDone = onDone;
    return true;
  }

  /** Call every frame while a wizard is active. */
  tickCalibration() {
    const c = this.calibrating;
    if (!c) return;
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
      .map(([ch, c]) => `${ch}: ${c.id.replace("WebHID ", "").split("(")[0].trim().slice(0, 18)} ax${c.axis}`)
      .join(" · ");
  }

  /** Read one calibrated channel from its own device. null = unbound/missing. */
  readChannel(ch, signed) {
    const c = this.cal?.[ch];
    if (!c) return null;
    const gp = this.padById(c.id);
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

  /** Latest sample as floats: steer -1..1, throttle/brake 0..1. */
  sample(dtSec) {
    // Calibrated multi-device rig takes precedence.
    if (this.cal?.steer) {
      const boundIds = new Set(CHANNELS.map((ch) => this.cal?.[ch]?.id).filter(Boolean));
      const present = [...boundIds].filter((id) => this.padById(id)).length;
      return {
        steer: this.readChannel("steer", true) ?? 0,
        throttle: this.readChannel("throttle", false) ?? 0,
        brake: this.readChannel("brake", false) ?? 0,
        handbrake: (this.readChannel("handbrake", false) ?? 0) > 0.5 || this.keys.has("Space"),
        device: `rig: ${present}/${boundIds.size} bound device(s) present`,
      };
    }
    // Zero-config controllers: prefer a standard-mapped pad among ALL devices.
    const std = this.gamepads().find((g) => g.mapping === "standard");
    if (std) {
      const dz = (v) => (Math.abs(v) < 0.08 ? 0 : v);
      return {
        steer: dz(std.axes[0] ?? 0),
        throttle: std.buttons[7]?.value ?? 0,
        brake: std.buttons[6]?.value ?? 0,
        handbrake: std.buttons[0]?.pressed ?? false,
        device: `${std.id.slice(0, 40)} (standard mapping)`,
      };
    }
    // Keyboard: slewed steering so it's actually drivable.
    const target = (this.keys.has("ArrowLeft") || this.keys.has("KeyA") ? -1 : 0) +
                   (this.keys.has("ArrowRight") || this.keys.has("KeyD") ? 1 : 0);
    const slew = 3.0 * dtSec;
    this.kSteer += Math.max(-slew, Math.min(slew, target - this.kSteer));
    if (target === 0) this.kSteer *= Math.max(0, 1 - 6 * dtSec);
    const pads = this.gamepads();
    return {
      steer: this.kSteer,
      throttle: this.keys.has("ArrowUp") || this.keys.has("KeyW") ? 1 : 0,
      brake: this.keys.has("ArrowDown") || this.keys.has("KeyS") ? 1 : 0,
      handbrake: this.keys.has("Space"),
      device: pads.length ? `keyboard (${pads.length} device(s) seen — press I to bind)` : "keyboard",
    };
  }

  clearCalibration() {
    this.cal = null;
    localStorage.removeItem(CAL_KEY);
  }
}

/** Quantize per CONTRACTS §2 — these integers are the canonical truth. */
export function quantize(raw) {
  return {
    steer: Math.max(-32767, Math.min(32767, Math.round(raw.steer * 32767))),
    throttle: Math.max(0, Math.min(65535, Math.round(raw.throttle * 65535))),
    brake: Math.max(0, Math.min(65535, Math.round(raw.brake * 65535))),
    flags: raw.handbrake ? 1 : 0,
  };
}

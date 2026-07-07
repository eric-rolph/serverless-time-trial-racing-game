// Input: keyboard fallback + Gamepad API wheel/pedals with a per-channel
// calibration wizard (rest value + extreme captured while the user moves only
// that control — handles inverted pedals and combined axes automatically).

const CAL_KEY = "sttr-input-cal-v1";

export class Input {
  constructor() {
    this.keys = new Set();
    this.kSteer = 0; // slewed keyboard steering
    this.cal = JSON.parse(localStorage.getItem(CAL_KEY) ?? "null"); // {steer:{axis,rest,ext},throttle:{...},brake:{...}}
    this.calibrating = null; // {channel, baseline:[...], t0}
    this.onCalDone = null;
    addEventListener("keydown", (e) => {
      if (!e.repeat) this.keys.add(e.code);
    });
    addEventListener("keyup", (e) => this.keys.delete(e.code));
  }

  gamepad() {
    return [...(navigator.getGamepads?.() ?? [])].find((g) => g && g.connected && g.axes.length >= 2) ?? null;
  }

  startCalibration(channel, onDone) {
    const gp = this.gamepad();
    if (!gp) return false;
    this.calibrating = { channel, baseline: [...gp.axes], best: -1, bestDev: 0, t0: performance.now() };
    this.onCalDone = onDone;
    return true;
  }

  /** Call every frame while a wizard is active. */
  tickCalibration() {
    const c = this.calibrating;
    if (!c) return;
    const gp = this.gamepad();
    if (!gp) return;
    gp.axes.forEach((v, i) => {
      const dev = Math.abs(v - c.baseline[i]);
      if (dev > c.bestDev) { c.bestDev = dev; c.best = i; c.ext = v; }
    });
    if (performance.now() - c.t0 > 2500) {
      if (c.best >= 0 && c.bestDev > 0.25) {
        this.cal ??= {};
        this.cal[c.channel] = { axis: c.best, rest: c.baseline[c.best], ext: c.ext };
        localStorage.setItem(CAL_KEY, JSON.stringify(this.cal));
        this.onCalDone?.(`${c.channel} → axis ${c.best}`);
      } else {
        this.onCalDone?.(`${c.channel}: no movement detected, try again`);
      }
      this.calibrating = null;
    }
  }

  /** Latest sample as floats: steer -1..1, throttle/brake 0..1. */
  sample(dtSec) {
    const gp = this.gamepad();
    if (gp && this.cal?.steer) {
      const read = (ch, signed) => {
        const c = this.cal[ch];
        if (!c) return 0;
        const v = gp.axes[c.axis] ?? c.rest;
        const span = c.ext - c.rest;
        if (Math.abs(span) < 1e-3) return 0;
        const n = (v - c.rest) / span;
        return signed ? Math.max(-1, Math.min(1, n)) : Math.max(0, Math.min(1, n));
      };
      // Steering: calibration captured one extreme; assume symmetric range
      // around rest (true for wheels reporting -1..1 with 0 center).
      const s = this.cal.steer;
      const steerRaw = ((gp.axes[s.axis] ?? s.rest) - s.rest) / Math.abs(s.ext - s.rest);
      return {
        steer: Math.max(-1, Math.min(1, steerRaw * Math.sign(s.ext - s.rest))),
        throttle: read("throttle", false),
        brake: read("brake", false),
        handbrake: gp.buttons?.[0]?.pressed ?? false,
        device: gp.id,
      };
    }
    // Keyboard: slewed steering so it's actually drivable.
    const target = (this.keys.has("ArrowLeft") || this.keys.has("KeyA") ? -1 : 0) +
                   (this.keys.has("ArrowRight") || this.keys.has("KeyD") ? 1 : 0);
    const slew = 3.0 * dtSec;
    this.kSteer += Math.max(-slew, Math.min(slew, target - this.kSteer));
    if (target === 0) this.kSteer *= Math.max(0, 1 - 6 * dtSec);
    return {
      steer: this.kSteer,
      throttle: this.keys.has("ArrowUp") || this.keys.has("KeyW") ? 1 : 0,
      brake: this.keys.has("ArrowDown") || this.keys.has("KeyS") ? 1 : 0,
      handbrake: this.keys.has("Space"),
      device: gp ? `${gp.id} (uncalibrated — press I)` : "keyboard",
    };
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

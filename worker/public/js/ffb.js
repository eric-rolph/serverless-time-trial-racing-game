// EXPERIMENTAL browser force feedback via WebHID (Chrome/Edge only).
//
// Speaks the Fanatec wheel-base protocol documented by the hid-fanatecff Linux
// driver (github.com/gotzl/hid-fanatecff): 7-byte output reports; constant
// force = [slot<<4|0x1, 0x08, lo, hi, 0, 0, 0x01] with the magnitude as
// unsigned-16 centered at 0x8000; init unlocks forces with three 0xf8 0x09
// commands. Torque source is the physics kernel's sim_ffb_torque() — the same
// rack-torque model desktop sims feed to DirectInput (docs/FFB.md).
//
// Honest caveats: untested protocol variance across firmware versions; Windows
// may deny opening a HID interface another driver holds exclusively; and if
// nothing happens the wheel simply stays passive — there is no unsafe failure
// mode beyond "no force".

const FANATEC_VID = 0x0eb7;
const SLOT = 1;

export class FanatecFFB {
  static supported() {
    return "hid" in navigator;
  }

  /** Must be called from a user gesture (button click). */
  static async connect() {
    const devices = await navigator.hid.requestDevice({ filters: [{ vendorId: FANATEC_VID }] });
    if (!devices.length) throw new Error("no device selected");
    // Prefer an interface that exposes output reports.
    const device =
      devices.find((d) => d.collections.some((c) => c.outputReports?.length)) ?? devices[0];
    if (!device.opened) await device.open();
    const ffb = new FanatecFFB(device);
    await ffb.init();
    return ffb;
  }

  constructor(device) {
    this.device = device;
    this.reportId = device.collections.flatMap((c) => c.outputReports ?? [])[0]?.reportId ?? 0;
    this.lastSent = 0;
    this.lastValue = null;
    this.gain = 0.8;
    this.maxNm = 18.0; // rack torque (Nm) mapping to full wheel output — the
    // brush model peaks near ~16-18 Nm of rack torque; mapping 8 Nm clipped
    // everything past 1 deg of slip into a wall (soft center, then max force).
    // Absolute strength is the wheel base's own setting; this is SHAPE.
    this.invert = false;
    this.smoothed = 0;
    // Input-axis decoding for WebHID devices lives in hid-input.js — app.js
    // registers this device there on connect, so torque and steering input
    // share one open HID connection.
  }

  async send(bytes) {
    await this.device.sendReport(this.reportId, new Uint8Array(bytes));
  }

  async init() {
    // Unlock sequence per hid-fanatecff (three variants of the 0xf8 0x09 cmd).
    for (const x of [0x01, 0x00, 0x04]) {
      await this.send([0xf8, 0x09, 0x01, 0x06, 0xff, x, 0x00]);
    }
  }

  /** torqueNm: physics rack torque. Rate-limited; call every render frame. */
  update(torqueNm, dtSec) {
    // 1-pole low-pass (~120 Hz knee) keeps kerb detail, kills per-tick hash.
    const k = 1 - Math.exp(-2 * Math.PI * 120 * Math.min(dtSec, 0.05));
    this.smoothed += (torqueNm - this.smoothed) * k;

    const now = performance.now();
    if (now - this.lastSent < 8) return; // ≤125 Hz on the wire

    let norm = (this.smoothed / this.maxNm) * this.gain * (this.invert ? -1 : 1);
    norm = Math.max(-1, Math.min(1, norm));
    const s16 = Math.round(norm * 32767);
    const u16 = (s16 + 0x8000) & 0xffff;
    if (this.lastValue !== null && Math.abs(u16 - this.lastValue) < 64 && now - this.lastSent < 50) {
      return; // unchanged within deadband — 50 ms keepalive still applies
    }
    this.lastValue = u16;
    this.lastSent = now;
    this.send([(SLOT << 4) | 0x1, 0x08, u16 & 0xff, (u16 >> 8) & 0xff, 0x00, 0x00, 0x01]).catch(() => {});
  }

  async stop() {
    try {
      await this.send([(SLOT << 4) | 0x3, 0x00, 0x80, 0x80, 0x00, 0x00, 0x01]); // disable slot
      await this.device.close();
    } catch {
      /* device already gone */
    }
  }
}

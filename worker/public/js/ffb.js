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

/** Walk the parsed HID report descriptor and locate every axis-like input
 *  field (Generic Desktop page, usages X..Dial, >= 8 bits). Returns
 *  Map<reportId, [{bit, size, min, max}]> with absolute bit offsets. */
function buildAxisMap(device) {
  const maps = new Map();
  for (const col of device.collections ?? []) {
    for (const rep of col.inputReports ?? []) {
      let bit = 0;
      const fields = [];
      for (const item of rep.items ?? []) {
        const size = item.reportSize ?? 0;
        const count = item.reportCount ?? 0;
        const usages = item.usages ?? [];
        for (let k = 0; k < count; k++) {
          const usage = usages[k] ?? (item.isRange ? (item.usageMinimum ?? 0) + k : undefined);
          if (usage !== undefined && size >= 8) {
            const page = usage >>> 16;
            const id = usage & 0xffff;
            if (page === 0x01 && id >= 0x30 && id <= 0x39) {
              fields.push({
                bit,
                size,
                min: item.logicalMinimum ?? 0,
                max: item.logicalMaximum ?? 2 ** size - 1,
              });
            }
          }
          bit += size;
        }
      }
      if (fields.length) maps.set(rep.reportId ?? 0, fields);
    }
  }
  return maps;
}

/** Little-endian bit-field extraction from a HID report payload. */
function readBits(dv, bitOffset, size, signed) {
  const start = bitOffset >> 3;
  const shift = bitOffset & 7;
  const nBytes = Math.min(Math.ceil((size + shift) / 8), Math.max(0, dv.byteLength - start));
  let acc = 0;
  for (let b = 0; b < nBytes; b++) acc += dv.getUint8(start + b) * 2 ** (8 * b);
  let value = Math.floor(acc / 2 ** shift) % 2 ** size;
  if (signed && value >= 2 ** (size - 1)) value -= 2 ** size;
  return value;
}

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
    this.maxNm = 8.0; // CSL DD peak
    this.invert = false;
    this.smoothed = 0;

    // Chrome removes a device from the Gamepad API once a page opens it via
    // WebHID — so we decode steering/pedal axes from the HID input reports
    // ourselves and expose them as a virtual gamepad. The decoder is generic:
    // built from the report descriptor metadata (Generic Desktop usages
    // 0x30-0x39), no device-specific layout assumptions.
    this.axes = [];
    this.reportsSeen = 0;
    this._axisMap = buildAxisMap(device);
    device.addEventListener("inputreport", (e) => this._decodeInput(e));
  }

  _decodeInput(e) {
    const fields = this._axisMap.get(e.reportId);
    if (!fields) return;
    this.reportsSeen++;
    this.axes = fields.map((f) => {
      const raw = readBits(e.data, f.bit, f.size, f.min < 0);
      const span = f.max - f.min;
      return span > 0 ? ((raw - f.min) / span) * 2 - 1 : 0;
    });
  }

  /** Gamepad-shaped view of this device's HID axes (for the input binder). */
  virtualPad() {
    return {
      id: `WebHID ${this.device.productName || "Fanatec"}`,
      index: 100,
      axes: this.axes,
      buttons: [],
      mapping: "",
      connected: this.device.opened,
    };
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

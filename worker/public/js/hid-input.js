// Raw-HID input sources (WebHID) — the escape hatch for devices Chrome's
// Gamepad API enumerates but never delivers data for (Fanatec bases are a
// known case: axes frozen at 0), or doesn't enumerate at all (some pedals).
// We read HID input reports directly — the browser equivalent of the RawInput
// path desktop sims use — and expose each device as a virtual gamepad for the
// per-channel binder.
//
// Permissions persist per origin, so previously-added devices auto-reconnect
// on page load (input reading only — FFB stays behind its own button).

const sources = new Map(); // HIDDevice -> {axes: number[], reportsSeen, fields: Map}

/** Walk the parsed HID report descriptor and locate every axis-like input
 *  field (Generic Desktop page, usages X..Dial, >= 8 bits). Returns
 *  Map<reportId, [{bit, size, min, max}]> with absolute bit offsets. */
export function buildAxisMap(device) {
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
export function readBits(dv, bitOffset, size, signed) {
  const start = bitOffset >> 3;
  const shift = bitOffset & 7;
  const nBytes = Math.min(Math.ceil((size + shift) / 8), Math.max(0, dv.byteLength - start));
  let acc = 0;
  for (let b = 0; b < nBytes; b++) acc += dv.getUint8(start + b) * 2 ** (8 * b);
  let value = Math.floor(acc / 2 ** shift) % 2 ** size;
  if (signed && value >= 2 ** (size - 1)) value -= 2 ** size;
  return value;
}

/** Open (if needed) and start decoding a device. Idempotent per device. */
export async function registerHidDevice(device) {
  if (sources.has(device)) return sources.get(device);
  if (!device.opened) await device.open();
  const src = { axes: [], reportsSeen: 0, fields: buildAxisMap(device) };
  sources.set(device, src);
  device.addEventListener("inputreport", (e) => {
    const fields = src.fields.get(e.reportId);
    if (!fields) return;
    src.reportsSeen++;
    src.axes = fields.map((f) => {
      const raw = readBits(e.data, f.bit, f.size, f.min < 0);
      const span = f.max - f.min;
      return span > 0 ? ((raw - f.min) / span) * 2 - 1 : 0;
    });
  });
  return src;
}

/** Show the browser device picker (must be called from a user gesture). */
export async function addHidDevices() {
  const devices = await navigator.hid.requestDevice({ filters: [] });
  for (const d of devices) await registerHidDevice(d);
  return devices.length;
}

/** Reopen every previously-granted device (call once at startup). */
export async function autoReconnectHid() {
  try {
    const devices = await navigator.hid.getDevices();
    for (const d of devices) await registerHidDevice(d).catch(() => {});
    return devices.length;
  } catch {
    return 0;
  }
}

/** Gamepad-shaped views for the input binder. */
export function hidVirtualPads() {
  let i = 0;
  return [...sources.entries()].map(([device, src]) => ({
    id: `WebHID ${device.productName || `device ${device.vendorId?.toString(16)}`}`,
    index: 100 + i++,
    axes: src.axes,
    buttons: [],
    mapping: "",
    connected: device.opened,
  }));
}

/** One-line health summary for the setup panel. */
export function hidDiagnostics() {
  if (!sources.size) return null;
  return [...sources.entries()]
    .map(([d, s]) => `${(d.productName || "device").slice(0, 22)}: ${s.axes.length} axes/${s.reportsSeen} reports`)
    .join(" · ");
}

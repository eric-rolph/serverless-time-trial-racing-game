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
          if (usage !== undefined && size >= 8 && fields.length < 24) {
            const page = usage >>> 16;
            const id = usage & 0xffff;
            const max = item.logicalMaximum ?? 2 ** size - 1;
            const min = item.logicalMinimum ?? 0;
            // Axis heuristic, deliberately generous — sim hardware scatters its
            // axes across pages: Generic Desktop X..Slider, Simulation Controls
            // (Steering 0xC8, Accel 0xC4, Brake 0xC5, Clutch 0xC6, Throttle
            // 0xBB...), and Fanatec loves vendor-defined pages. Anything >= 8
            // bits with a real analog span qualifies, except Button-page
            // fields. Dead extra axes are harmless — the binder picks whatever
            // MOVES.
            const genericAxis = page === 0x01 && id >= 0x30 && id <= 0x38;
            const simAxis = page === 0x02 && id >= 0xb0 && id <= 0xd0;
            const analogSpan = page !== 0x09 && max - min >= 255;
            if (genericAxis || simAxis || analogSpan) {
              fields.push({ bit, size, min, max });
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
  const src = { axes: [], reportsSeen: 0, fields: buildAxisMap(device), lastRaw: "" };
  sources.set(device, src);
  device.addEventListener("inputreport", (e) => {
    src.reportsSeen++;
    const fields = src.fields.get(e.reportId);
    if (!fields) {
      // Unknown report id — keep a hex peek so the layout can be diagnosed.
      const bytes = new Uint8Array(e.data.buffer, e.data.byteOffset, Math.min(12, e.data.byteLength));
      src.lastRaw = `r${e.reportId}:` + [...bytes].map((b) => b.toString(16).padStart(2, "0")).join("");
      return;
    }
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

/** Health summary for the setup panel — live axis values make problems
 *  self-evident: 0 reports = no data flow; 0 axes = descriptor not understood
 *  (lastRaw hex shown for diagnosis); values frozen = wrong fields. */
export function hidDiagnostics() {
  if (!sources.size) return null;
  return [...sources.entries()]
    .map(([d, s]) => {
      const vals = s.axes.length
        ? ` [${s.axes.slice(0, 6).map((a) => a.toFixed(2)).join(",")}]`
        : s.lastRaw
          ? ` raw ${s.lastRaw}`
          : "";
      return `${(d.productName || "device").slice(0, 22)}: ${s.fields.size ? [...s.fields.values()][0].length : 0} axes, ${s.reportsSeen} reports${vals}`;
    })
    .join(" — ");
}

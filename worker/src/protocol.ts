// Binding contracts from docs/CONTRACTS.md — LAPLOG v1 (§3), quantization (§2).

export const TICK_RATE = 400;
export const MAX_TICKS = 72_000; // 3 minutes
export const LAPLOG_MAGIC = 0x474c5453; // "STLG" read as u32 LE
export const SIGNING_DOMAIN_TAG = "sttr-lap-v1";
export const MAX_LOG_BYTES = 1024 * 1024;

export interface LapLog {
  version: number;
  tickRate: number;
  trackHash: bigint;
  tickCount: number;
  /** i16 steer values, one per tick (heuristics operate on this channel) */
  steer: Int16Array;
  /** byte offset of the 8·N tick-record block inside the raw buffer */
  ticksByteOffset: number;
  finalStateHash: bigint;
  claimedLapTicks: number;
}

export class LogFormatError extends Error {}

export function parseLapLog(data: ArrayBuffer | ArrayBufferView): LapLog {
  // Single choke point for binary-frame normalization: runtimes disagree on
  // whether WebSocket binary messages arrive as ArrayBuffer or a view.
  const buf: ArrayBuffer = ArrayBuffer.isView(data)
    ? (data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength) as ArrayBuffer)
    : data;
  if (buf.byteLength > MAX_LOG_BYTES) throw new LogFormatError("too_large");
  if (buf.byteLength < 32) throw new LogFormatError("truncated header");
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== LAPLOG_MAGIC) throw new LogFormatError("bad magic");
  const version = dv.getUint16(4, true);
  if (version !== 1) throw new LogFormatError(`unsupported version ${version}`);
  const tickRate = dv.getUint16(6, true);
  if (tickRate !== TICK_RATE) throw new LogFormatError(`tick rate ${tickRate} != ${TICK_RATE}`);
  const trackHash = dv.getBigUint64(8, true);
  const tickCount = dv.getUint32(16, true);
  if (tickCount < 1 || tickCount > MAX_TICKS) throw new LogFormatError("tick count out of range");
  const expected = 20 + 8 * tickCount + 12;
  if (buf.byteLength !== expected)
    throw new LogFormatError(`length ${buf.byteLength} != expected ${expected}`);

  const steer = new Int16Array(tickCount);
  for (let i = 0; i < tickCount; i++) {
    steer[i] = dv.getInt16(20 + 8 * i, true);
    const flags = dv.getUint16(20 + 8 * i + 6, true);
    if ((flags & ~0x0001) !== 0) throw new LogFormatError("reserved flag bits set");
  }
  return {
    version,
    tickRate,
    trackHash,
    tickCount,
    steer,
    ticksByteOffset: 20,
    finalStateHash: dv.getBigUint64(20 + 8 * tickCount, true),
    claimedLapTicks: dv.getUint32(28 + 8 * tickCount, true),
  };
}

export function fnv1a64(bytes: Uint8Array): bigint {
  let h = 0xcbf29ce484222325n;
  for (let i = 0; i < bytes.length; i++) {
    h ^= BigInt(bytes[i]);
    h = (h * 0x100000001b3n) & 0xffffffffffffffffn;
  }
  return h;
}

export const hex64 = (v: bigint): string => v.toString(16).padStart(16, "0");

export function ticksToMs(ticks: number): number {
  return Math.round((ticks * 1000) / TICK_RATE);
}

export interface SubmitHeader {
  type: "submit";
  pubkey: string; // base64, 32 bytes
  sig: string; // base64, 64 bytes
  name: string;
  logBytes: number;
}

export function parseSubmitHeader(text: string): SubmitHeader {
  let obj: unknown;
  try {
    obj = JSON.parse(text);
  } catch {
    throw new LogFormatError("header is not JSON");
  }
  const h = obj as Record<string, unknown>;
  if (h.type !== "submit") throw new LogFormatError("expected type=submit");
  if (typeof h.pubkey !== "string" || typeof h.sig !== "string")
    throw new LogFormatError("missing pubkey/sig");
  const name = typeof h.name === "string" ? h.name.slice(0, 24) : "anon";
  const logBytes = typeof h.logBytes === "number" ? h.logBytes : 0;
  return { type: "submit", pubkey: h.pubkey, sig: h.sig, name, logBytes };
}

export function b64decode(s: string, expectedLen: number): Uint8Array {
  const bin = atob(s);
  if (bin.length !== expectedLen) throw new LogFormatError(`expected ${expectedLen} bytes`);
  const out = new Uint8Array(expectedLen);
  for (let i = 0; i < expectedLen; i++) out[i] = bin.charCodeAt(i);
  return out;
}

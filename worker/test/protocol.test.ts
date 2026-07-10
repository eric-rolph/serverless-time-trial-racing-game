import { describe, expect, it } from "vitest";
import { fnv1a64, hex64, parseLapLog, ticksToMs } from "../src/protocol";

function buildLog(tickCount: number): ArrayBuffer {
  const buf = new ArrayBuffer(20 + 8 * tickCount + 12);
  const dv = new DataView(buf);
  dv.setUint8(0, 0x53); // S
  dv.setUint8(1, 0x54); // T
  dv.setUint8(2, 0x4c); // L
  dv.setUint8(3, 0x47); // G
  dv.setUint16(4, 1, true); // version
  dv.setUint16(6, 400, true); // tick rate
  dv.setBigUint64(8, 0x3fbe91d5a2ca5851n, true); // track hash
  dv.setUint32(16, tickCount, true);
  for (let i = 0; i < tickCount; i++) {
    dv.setInt16(20 + 8 * i, i * 100 - 500, true); // steer
    dv.setUint16(20 + 8 * i + 2, 65535, true); // throttle
    dv.setUint16(20 + 8 * i + 4, 0, true); // brake
    dv.setUint16(20 + 8 * i + 6, 0, true); // flags
  }
  dv.setBigUint64(20 + 8 * tickCount, 0xdeadbeefcafef00dn, true);
  dv.setUint32(28 + 8 * tickCount, 33600, true); // 84 s lap
  return buf;
}

describe("parseLapLog", () => {
  it("round-trips a valid log", () => {
    const log = parseLapLog(buildLog(3));
    expect(log.version).toBe(1);
    expect(log.tickRate).toBe(400);
    expect(log.trackHash).toBe(0x3fbe91d5a2ca5851n);
    expect(log.tickCount).toBe(3);
    expect(Array.from(log.steer)).toEqual([-500, -400, -300]);
    expect(log.ticksByteOffset).toBe(20);
    expect(log.finalStateHash).toBe(0xdeadbeefcafef00dn);
    expect(log.claimedLapTicks).toBe(33600);
  });

  it("rejects bad magic", () => {
    const buf = buildLog(3);
    new DataView(buf).setUint8(0, 0x58);
    expect(() => parseLapLog(buf)).toThrow(/magic/);
  });

  it("rejects length mismatch", () => {
    const buf = buildLog(3);
    new DataView(buf).setUint32(16, 4, true);
    expect(() => parseLapLog(buf)).toThrow(/length/);
  });

  it("accepts handbrake + shift flag bits (0-2)", () => {
    const buf = buildLog(3);
    new DataView(buf).setUint16(26, 0x0007, true);
    expect(() => parseLapLog(buf)).not.toThrow();
  });

  it("rejects reserved flag bits", () => {
    const buf = buildLog(3);
    new DataView(buf).setUint16(26, 0x0008, true);
    expect(() => parseLapLog(buf)).toThrow(/reserved/);
  });
});

describe("fnv1a64", () => {
  it("matches the reference vector", () => {
    expect(hex64(fnv1a64(new TextEncoder().encode("hello")))).toBe("a430d84680aabd0b");
  });
});

describe("ticksToMs", () => {
  it("converts 400 Hz ticks", () => {
    expect(ticksToMs(400)).toBe(1000);
    expect(ticksToMs(33613)).toBe(84033); // rounds
  });
});

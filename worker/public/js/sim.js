// Browser host for sim.wasm — the same canonical binary the referee replays
// (fetched from /api/sim/current so client and validator can never diverge).

export const STATUS = { RUNNING: 0x1, LAP_COMPLETE: 0x2, OFF_TRACK: 0x4, LAP_INVALID: 0x8, ERROR: 0x80000000 };
export const TICK_RATE = 400;
export const DT_MS = 2.5; // dyadic (2^1 + 2^-1) — accumulator subtraction is exact

export class Sim {
  static async load() {
    const resp = await fetch("/api/sim/current");
    if (!resp.ok) throw new Error(`sim fetch failed: ${resp.status}`);
    const bytes = await resp.arrayBuffer();
    const module = await WebAssembly.compile(bytes);
    const imports = {};
    for (const im of WebAssembly.Module.imports(module)) {
      if (im.kind !== "function") continue;
      (imports[im.module] ??= {})[im.name] = () => 0;
    }
    const instance = await WebAssembly.instantiate(module, imports);
    const sim = new Sim(instance.exports);
    if (sim.e.sim_abi_version() !== 1) throw new Error("sim ABI mismatch");
    return sim;
  }

  constructor(exports) {
    this.e = exports;
  }

  loadTrack(trackBytes) {
    const ptr = this.e.sim_alloc(trackBytes.length);
    new Uint8Array(this.e.memory.buffer, ptr, trackBytes.length).set(trackBytes);
    if (this.e.sim_load_track(ptr, trackBytes.length) !== 0) throw new Error("track load failed");
    this.e.sim_reset();
  }

  reset() { this.e.sim_reset(); }

  step(steer, throttle, brake, flags) {
    return this.e.sim_step(steer, throttle, brake, flags) >>> 0;
  }

  /** Copy SimStateV1 (200 bytes, CONTRACTS §1.2) out of wasm memory. */
  state() {
    const dv = new DataView(this.e.memory.buffer, this.e.sim_state_ptr(), 200);
    const wheels = [];
    for (let w = 0; w < 4; w++) {
      const o = 56 + w * 32;
      wheels.push({
        pos: [dv.getFloat32(o, true), dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true)],
        spin: dv.getFloat32(o + 12, true),
        steer: dv.getFloat32(o + 16, true),
        compression: dv.getFloat32(o + 20, true),
      });
    }
    return {
      tick: dv.getUint32(0, true),
      pos: [dv.getFloat32(4, true), dv.getFloat32(8, true), dv.getFloat32(12, true)],
      quat: [dv.getFloat32(16, true), dv.getFloat32(20, true), dv.getFloat32(24, true), dv.getFloat32(28, true)],
      wheels,
      speed: dv.getFloat32(184, true),
      lapProgress: dv.getFloat32(188, true),
      checkpoints: dv.getUint32(192, true),
      lapTicks: dv.getUint32(196, true),
    };
  }

  stateHash() {
    const lo = BigInt(this.e.sim_state_hash_lo() >>> 0);
    const hi = BigInt(this.e.sim_state_hash_hi() >>> 0);
    return (hi << 32n) | lo;
  }

  lapTimeTicks() { return this.e.sim_lap_time_ticks(); }
}

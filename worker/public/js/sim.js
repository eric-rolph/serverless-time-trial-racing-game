// Browser host for sim.wasm — the same canonical binary the referee replays
// (fetched from /api/sim/current so client and validator can never diverge).

export const STATUS = { RUNNING: 0x1, LAP_COMPLETE: 0x2, OFF_TRACK: 0x4, LAP_INVALID: 0x8, ERROR: 0x80000000 };
export const TICK_RATE = 400;
export const DT_MS = 2.5; // dyadic (2^1 + 2^-1) — accumulator subtraction is exact

let cachedModule = null;

export class Sim {
  static async load() {
    if (!cachedModule) {
      const resp = await fetch("/api/sim/current");
      if (!resp.ok) throw new Error(`sim fetch failed: ${resp.status}`);
      cachedModule = await WebAssembly.compile(await resp.arrayBuffer());
    }
    return Sim.instantiate();
  }

  /** Fresh instance from the cached module (used for the ghost car). */
  static async instantiate() {
    const imports = {};
    for (const im of WebAssembly.Module.imports(cachedModule)) {
      if (im.kind !== "function") continue;
      (imports[im.module] ??= {})[im.name] = () => 0;
    }
    const instance = await WebAssembly.instantiate(cachedModule, imports);
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
        slipRatio: dv.getFloat32(o + 24, true),
        slipAngle: dv.getFloat32(o + 28, true),
      });
    }
    return {
      tick: dv.getUint32(0, true),
      pos: [dv.getFloat32(4, true), dv.getFloat32(8, true), dv.getFloat32(12, true)],
      quat: [dv.getFloat32(16, true), dv.getFloat32(20, true), dv.getFloat32(24, true), dv.getFloat32(28, true)],
      // Chassis linear velocity (world frame, m/s). Read for the cosmetic
      // crash-deformation trigger only — never fed back into the sim, so
      // determinism/replay are untouched (CONTRACTS §1.2, offset 32).
      linVel: [dv.getFloat32(32, true), dv.getFloat32(36, true), dv.getFloat32(40, true)],
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

  /** Steering rack torque (Nm) — ABI 1.1 export; 0 on older binaries. */
  ffbTorque() { return this.e.sim_ffb_torque ? this.e.sim_ffb_torque() : 0; }

  /** Tire surface temperature °C (ABI 1.2 export; null on older binaries). */
  tireTemp(i) { return this.e.sim_tire_temp ? this.e.sim_tire_temp(i) : null; }

  /** Crash-damage scalar (ABI 1.3 export; null on older binaries). Component
   *  0 = overall [0,1], 1 = steer loss [0,1], 2 = |front toe| rad,
   *  3 = |rear toe| rad. Output-only — this is the SAME deterministic value the
   *  referee recomputes from the input log, so it can drive cosmetics that must
   *  agree with the physics. Never fed back into the sim (determinism intact). */
  damage(component) { return this.e.sim_damage ? this.e.sim_damage(component) : null; }
}

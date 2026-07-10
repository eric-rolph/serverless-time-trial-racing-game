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
      // Streaming compile overlaps download and compilation, but the engine
      // requires content-type application/wasm. Branch BEFORE touching the
      // body — a Response body is single-use.
      const mime = (resp.headers.get("content-type") ?? "").split(";")[0].trim().toLowerCase();
      cachedModule = mime === "application/wasm" && typeof WebAssembly.compileStreaming === "function"
        ? await WebAssembly.compileStreaming(resp)
        : await WebAssembly.compile(await resp.arrayBuffer());
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
    // Two persistent snapshot buffers for state() — the app holds exactly a
    // prev/curr pair, so alternating between two reusable objects avoids
    // rebuilding the whole object graph 400×/s without ever aliasing the
    // caller's pair. Read-side copy only: wasm memory, stepping and logs are
    // untouched, so determinism/replay are unaffected.
    this._stateBufs = [Sim._blankState(), Sim._blankState()];
    this._stateFlip = 0;
  }

  /** One reusable SimStateV1 snapshot object (field names/shapes are the
   *  public contract — keep in lockstep with state() below). */
  static _blankState() {
    const wheels = [];
    for (let w = 0; w < 4; w++) {
      wheels.push({ pos: [0, 0, 0], spin: 0, steer: 0, compression: 0, slipRatio: 0, slipAngle: 0 });
    }
    return {
      tick: 0,
      pos: [0, 0, 0],
      quat: [0, 0, 0, 0],
      linVel: [0, 0, 0],
      wheels,
      speed: 0,
      lapProgress: 0,
      checkpoints: 0,
      lapTicks: 0,
    };
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

  /** Copy SimStateV1 (200 bytes, CONTRACTS §1.2) out of wasm memory.
   *  Fills one of two persistent buffers in place, alternating per call —
   *  each returned object stays valid until the call after next. */
  state() {
    const dv = new DataView(this.e.memory.buffer, this.e.sim_state_ptr(), 200);
    this._stateFlip ^= 1;
    const s = this._stateBufs[this._stateFlip];
    s.tick = dv.getUint32(0, true);
    const p = s.pos;
    p[0] = dv.getFloat32(4, true); p[1] = dv.getFloat32(8, true); p[2] = dv.getFloat32(12, true);
    const q = s.quat;
    q[0] = dv.getFloat32(16, true); q[1] = dv.getFloat32(20, true); q[2] = dv.getFloat32(24, true); q[3] = dv.getFloat32(28, true);
    // Chassis linear velocity (world frame, m/s). Read for the cosmetic
    // crash-deformation trigger only — never fed back into the sim, so
    // determinism/replay are untouched (CONTRACTS §1.2, offset 32).
    const v = s.linVel;
    v[0] = dv.getFloat32(32, true); v[1] = dv.getFloat32(36, true); v[2] = dv.getFloat32(40, true);
    for (let w = 0; w < 4; w++) {
      const o = 56 + w * 32;
      const wh = s.wheels[w];
      const wp = wh.pos;
      wp[0] = dv.getFloat32(o, true); wp[1] = dv.getFloat32(o + 4, true); wp[2] = dv.getFloat32(o + 8, true);
      wh.spin = dv.getFloat32(o + 12, true);
      wh.steer = dv.getFloat32(o + 16, true);
      wh.compression = dv.getFloat32(o + 20, true);
      wh.slipRatio = dv.getFloat32(o + 24, true);
      wh.slipAngle = dv.getFloat32(o + 28, true);
    }
    s.speed = dv.getFloat32(184, true);
    s.lapProgress = dv.getFloat32(188, true);
    s.checkpoints = dv.getUint32(192, true);
    s.lapTicks = dv.getUint32(196, true);
    return s;
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

  /** Engine speed in rpm (ABI 1.4 export; null on older binaries). Output-only
   *  and unhashed (DRIVETRAIN.md §4) — any divergence would surface in the
   *  chassis hash within ticks because gearing scales forces. */
  rpm() { return this.e.sim_rpm ? this.e.sim_rpm() : null; }

  /** Engaged gear: 0 = reverse, 1..6 (ABI 1.4 export; null on older binaries).
   *  Reports the OUTGOING gear during a shift in progress (DRIVETRAIN.md
   *  §6.5 item 10), so the HUD numeral flips at engagement, not request. */
  gear() { return this.e.sim_gear ? this.e.sim_gear() : null; }
}

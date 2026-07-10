// Synthesized engine + physics-driven tire audio (WebAudio, no assets).
// Starts on first user gesture (browser autoplay policy); M toggles mute.
//
// The tire voices are FEEDBACK, not immersion: every parameter is read from
// the physics state each frame so your ears learn the grip model.
//
//   parameter        source (per wheel)                    → audio mapping
//   ---------------- ------------------------------------- -----------------
//   slipNorm         max(|slipAngle|/0.116, |slipRatio|/0.10)
//                    (peak slip angle ≈ 6.65° = 0.116 rad;  volume onset at
//                    peak slip ratio ≈ 0.10)                0.75 (≈75% of the
//                                                           grip peak), full
//                                                           at 1.0 (the peak)
//   load             compression × 60000 N/m ÷ 3300 N       multiplies volume
//                    (spring rate / static corner weight)   so an unloaded
//                                                           inner wheel stays
//                                                           quiet even at big
//                                                           slip angles
//   slip magnitude   loudest wheel of the axle              pitch UP: freq =
//                                                           base × (0.9+0.3·s)
//   tire temp °C     sim.tireTemp(i), null-guarded          pitch factor =
//                    (old binaries → factor 1)              1−(T−80)·0.004,
//                                                           clamped 0.82–1.14:
//                                                           hot rubber ≈ lower
//                                                           throatier scrub,
//                                                           cold ≈ higher
//                                                           squeal
//   kerb rumble      Σ|Δcompression| frame-to-frame across  low bandpass thump
//                    all four wheels (high-freq suspension  gain from envelope
//                    oscillation; smooth steady-state       above dead zone
//                    cornering compression moves far less
//                    per frame than kerb strikes)
//
// Front and rear axles get separate voices (front bandpass 1050 Hz, rear
// 880 Hz) so understeer and oversteer sound different. All parameter changes
// go through setTargetAtTime — no zipper noise.

const PEAK_SLIP_ANGLE = 0.116; // rad ≈ 6.65° — grip peak of the tire model
const PEAK_SLIP_RATIO = 0.10;
const SPRING_RATE = 60000; // N/m — suspension spring in the physics
const STATIC_LOAD = 3300; // N — approx static per-corner load
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));

export class EngineAudio {
  constructor() {
    this.ctx = null;
    this.muted = false;
    this.prevComp = [0, 0, 0, 0];
    this.kerbEnv = 0;
    this.prevGear = null; // last sim_gear() seen while driving; null re-arms after idle/reset
    this.duckUntil = 0;   // ctx time until which a shift-cut envelope owns engineDuck
  }

  /** Call from any user-gesture handler; idempotent. */
  start() {
    if (this.ctx) {
      // A context created outside real user activation (the armed-idle gamepad
      // start path polls from rAF) comes up suspended; the first genuine
      // gesture routes here — resume it so audio isn't dead until reload.
      if (this.ctx.state === "suspended") this.ctx.resume();
      return;
    }
    const ctx = new AudioContext();
    this.ctx = ctx;

    this.master = ctx.createGain();
    this.master.gain.value = 0.0;
    this.master.connect(ctx.destination);

    // Engine: two detuned saws through a low-pass — reads as a flat-plane V.
    this.osc1 = ctx.createOscillator();
    this.osc2 = ctx.createOscillator();
    this.osc1.type = "sawtooth";
    this.osc2.type = "sawtooth";
    this.engineFilter = ctx.createBiquadFilter();
    this.engineFilter.type = "lowpass";
    this.engineFilter.frequency.value = 900;
    this.engineGain = ctx.createGain();
    this.engineGain.gain.value = 0.05;
    // Duck stage in series AFTER engineGain: update() rewrites engineGain
    // every frame, so shift-cut dips and the limiter stutter get a gain node
    // of their own that those per-frame writes can never stomp on.
    this.engineDuck = ctx.createGain();
    this.engineDuck.gain.value = 1;
    this.osc1.connect(this.engineFilter);
    this.osc2.connect(this.engineFilter);
    this.engineFilter.connect(this.engineGain);
    this.engineGain.connect(this.engineDuck);
    this.engineDuck.connect(this.master);
    this.osc1.start();
    this.osc2.start();

    // One shared looping noise buffer fans out into all tire voices, and is
    // reused by one-shot impact() BufferSources.
    const noiseBuf = ctx.createBuffer(1, ctx.sampleRate, ctx.sampleRate);
    const data = noiseBuf.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    this.noiseBuf = noiseBuf;
    this.noise = ctx.createBufferSource();
    this.noise.buffer = noiseBuf;
    this.noise.loop = true;

    const voice = (freq, Q) => {
      const filter = ctx.createBiquadFilter();
      filter.type = "bandpass";
      filter.frequency.value = freq;
      filter.Q.value = Q;
      const gain = ctx.createGain();
      gain.gain.value = 0;
      this.noise.connect(filter);
      filter.connect(gain);
      gain.connect(this.master);
      return { filter, gain };
    };
    this.front = voice(1050, 2.5); // front-axle scrub (understeer)
    this.rear = voice(880, 2.5); // rear-axle scrub (oversteer)
    this.kerb = voice(110, 1.2); // low thump for kerb strikes
    this.noise.start();

    this.master.gain.linearRampToValueAtTime(this.muted ? 0 : 0.5, ctx.currentTime + 0.5);
  }

  toggleMute() {
    this.muted = !this.muted;
    if (this.ctx) {
      // Smooth 50 ms ramp instead of a hard jump — no clicks.
      const now = this.ctx.currentTime;
      this.master.gain.cancelScheduledValues(now);
      this.master.gain.setTargetAtTime(this.muted ? 0 : 0.5, now, 0.05);
    }
    return this.muted;
  }

  /** Fire-and-forget UI/event blip: OscillatorNode with an exponential-decay
   *  gain envelope. No-op before start() or while muted. */
  beep(freq, ms, type = "square", gain = 0.15) {
    if (!this.ctx || this.muted) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;
    const dur = Math.max(ms, 1) / 1000;
    const osc = ctx.createOscillator();
    osc.type = type;
    osc.frequency.value = freq;
    const g = ctx.createGain();
    g.gain.setValueAtTime(gain, t);
    g.gain.exponentialRampToValueAtTime(0.001, t + dur);
    osc.connect(g);
    g.connect(this.master);
    osc.start(t);
    osc.stop(t + dur + 0.02);
    osc.onended = () => {
      osc.disconnect();
      g.disconnect();
    };
  }

  /** One-shot collision hit: shared noise buffer → lowpass ~2.5 kHz →
   *  ~120 ms exponential decay, layered with a ~200 ms 60 Hz sine "body
   *  thump". severity ≥ 0, clamped to 1. No-op before start() or muted. */
  impact(severity) {
    if (!this.ctx || this.muted) return;
    const amp = Math.min(1, severity) * 0.5;
    if (!(amp > 0)) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;

    // Crunch layer: one-shot BufferSource over the shared noise buffer.
    const src = ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const lp = ctx.createBiquadFilter();
    lp.type = "lowpass";
    lp.frequency.value = 2500;
    const g = ctx.createGain();
    g.gain.setValueAtTime(amp, t);
    g.gain.exponentialRampToValueAtTime(0.001, t + 0.12);
    src.connect(lp);
    lp.connect(g);
    g.connect(this.master);
    src.start(t);
    src.stop(t + 0.15);
    src.onended = () => {
      src.disconnect();
      lp.disconnect();
      g.disconnect();
    };

    // Body thump: low sine under the crunch.
    const osc = ctx.createOscillator();
    osc.type = "sine";
    osc.frequency.value = 60;
    const g2 = ctx.createGain();
    g2.gain.setValueAtTime(amp * 0.8, t);
    g2.gain.exponentialRampToValueAtTime(0.001, t + 0.2);
    osc.connect(g2);
    g2.connect(this.master);
    osc.start(t);
    osc.stop(t + 0.25);
    osc.onended = () => {
      osc.disconnect();
      g2.disconnect();
    };
  }

  /** Gear-change punctuation (DRIVETRAIN.md §6): ~70 ms engine-gain dip (the
   *  audible ignition cut) + a mechanical clunk one-shot (short low-passed
   *  noise burst). Fired from update() when sim_gear() flips — sim_gear
   *  reports the outgoing gear until engagement, so this lands exactly when
   *  drive torque returns, right where the rpm step is. No-op before start()
   *  or while muted. */
  shiftCut() {
    if (!this.ctx || this.muted) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;
    // Dip rides the dedicated duck stage (update()'s per-frame engineGain
    // writes can't overwrite a scheduled envelope there).
    const dg = this.engineDuck.gain;
    dg.cancelScheduledValues(t);
    dg.setValueAtTime(dg.value, t);
    dg.linearRampToValueAtTime(0.15, t + 0.012);
    dg.setValueAtTime(0.15, t + 0.058);
    dg.linearRampToValueAtTime(1, t + 0.085);
    this.duckUntil = t + 0.09; // limiter stutter keeps its hands off until then

    // Clunk: shared noise buffer → 500 Hz lowpass → fast exponential decay.
    const src = ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const lp = ctx.createBiquadFilter();
    lp.type = "lowpass";
    lp.frequency.value = 500;
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.22, t);
    g.gain.exponentialRampToValueAtTime(0.001, t + 0.06);
    src.connect(lp);
    lp.connect(g);
    g.connect(this.master);
    src.start(t);
    src.stop(t + 0.09);
    src.onended = () => {
      src.disconnect();
      lp.disconnect();
      g.disconnect();
    };
  }

  /** state: SimStateV1 view {speed, wheels[]}; throttle 0..1; sim: the Sim
   *  wrapper for guarded tireTemp() reads (may be null); dtMs: real frame
   *  time in ms (kerb transient detection is normalized to a 60 Hz frame);
   *  idle: post-lap / countdown — engine settles to tickover, tire and kerb
   *  voices go silent instead of holding their last pitch. Call every frame. */
  update(state, throttle, sim = null, dtMs = 16.7, idle = false) {
    if (!this.ctx) return;
    const dt = Math.max(0.1, dtMs);

    // --- kerb transient bookkeeping runs even when muted or idle, so that
    // unmuting (or the first driven frame after idle) doesn't see a huge
    // stale Δcompression and fire a spurious thump.
    let delta = 0;
    for (let i = 0; i < 4; i++) {
      const c = state.wheels[i].compression ?? 0;
      delta += Math.abs(c - this.prevComp[i]);
      this.prevComp[i] = c;
    }
    // Normalize to a nominal 60 Hz frame so the 8 mm dead zone means the
    // same thing at any frame rate, then peak-hold with time-based decay.
    const deltaNorm = delta * (16.7 / dt);
    this.kerbEnv = Math.max(this.kerbEnv * Math.exp(-dt / 55), idle ? 0 : deltaNorm);

    if (this.muted) return;
    const t = this.ctx.currentTime;

    if (idle) {
      const f = 900 / 30; // tickover ≈ 900 rpm
      this.osc1.frequency.setTargetAtTime(f, t, 0.05);
      this.osc2.frequency.setTargetAtTime(f * 1.5 + 3, t, 0.05);
      this.engineFilter.frequency.setTargetAtTime(400, t, 0.1);
      this.engineGain.gain.setTargetAtTime(0.03, t, 0.1);
      this.engineDuck.gain.setTargetAtTime(1, t, 0.05); // release any stutter/dip
      this.prevGear = null; // reset re-spawns in 1st — don't clunk on GO
      this.front.gain.gain.setTargetAtTime(0, t, 0.04);
      this.rear.gain.gain.setTargetAtTime(0, t, 0.04);
      this.kerb.gain.gain.setTargetAtTime(0, t, 0.03);
      return;
    }

    // Engine pitch: REAL sim_rpm() when the binary exports it (ABI 1.4 —
    // shift cuts, auto-blips and limiter bounce all arrive through this
    // number for free); null-guarded fallback to the legacy speed-fake on
    // old wasm. Real rpm also retires the fake single-gear illusion.
    const realRpm = sim?.rpm ? sim.rpm() : null;
    const rpm = realRpm ?? (900 + state.speed * 260 + throttle * 600);
    if (realRpm !== null) {
      // Gear change → 70 ms dip + clunk (see shiftCut). prevGear re-arms via
      // idle so a reset back to 1st stays silent.
      const gear = sim.gear ? sim.gear() : null;
      if (gear !== null && this.prevGear !== null && gear !== this.prevGear) this.shiftCut();
      if (gear !== null) this.prevGear = gear;
      // Limiter: hard 15 Hz gain stutter while pinned at the redline. The
      // fuel cut bounces the needle just under 7500 (kernel holds ~7460 max),
      // so "pinned" = above 7350 with meaningful throttle.
      if (t >= this.duckUntil) {
        const pinned = rpm >= 7350 && throttle > 0.5;
        const gate = Math.floor(t * 30) % 2 === 0; // 30 half-cycles/s = 15 Hz
        this.engineDuck.gain.setTargetAtTime(pinned && !gate ? 0.08 : 1, t, 0.004);
      }
    }
    const f = rpm / 30; // Hz
    this.osc1.frequency.setTargetAtTime(f, t, 0.02);
    this.osc2.frequency.setTargetAtTime(f * 1.5 + 3, t, 0.02);
    this.engineFilter.frequency.setTargetAtTime(400 + throttle * 1400, t, 0.05);
    this.engineGain.gain.setTargetAtTime(0.04 + throttle * 0.05, t, 0.05);

    // Tires only speak above walking pace (parking-lot scrub is noise).
    const speedGate = clamp((state.speed - 3) / 6, 0, 1);

    // --- per-axle scrub: wheels 0,1 = front; 2,3 = rear (FL FR RL RR)
    const axles = [
      { v: this.front, wheels: [0, 1], base: 1050 },
      { v: this.rear, wheels: [2, 3], base: 880 },
    ];
    for (const a of axles) {
      let vol = 0, slipMax = 0, tempSum = 0, tempN = 0;
      for (const wi of a.wheels) {
        const w = state.wheels[wi];
        const slipNorm = Math.max(
          Math.abs(w.slipAngle ?? 0) / PEAK_SLIP_ANGLE,
          Math.abs(w.slipRatio ?? 0) / PEAK_SLIP_RATIO,
        );
        // Silent until 75% of the grip peak, full volume past the peak —
        // the onset ramp IS the "you're approaching the limit" warning.
        const onset = clamp((slipNorm - 0.75) / 0.25, 0, 1);
        // Load proxy: spring force / static weight. Unloaded inner wheel
        // (compression → 0) contributes nothing however big its slip angle.
        const load = clamp(((w.compression ?? 0) * SPRING_RATE) / STATIC_LOAD, 0, 1.4);
        vol += onset * load;
        slipMax = Math.max(slipMax, slipNorm);
        const temp = sim?.tireTemp ? sim.tireTemp(wi) : null; // null on old binaries
        if (temp !== null && temp !== undefined) {
          tempSum += temp;
          tempN++;
        }
      }
      const gain = speedGate * clamp(vol / 2, 0, 1) * 0.13;
      // Pitch: UP with slip magnitude (past the peak the scrub rises ~30%),
      // DOWN with surface temp around an 80 °C reference (hot = throaty,
      // cold = squeal). Factor clamped 0.82–1.14 ≈ 60–140 °C range.
      const tempFactor = tempN ? clamp(1 - (tempSum / tempN - 80) * 0.004, 0.82, 1.14) : 1;
      const freq = a.base * (0.9 + 0.3 * clamp(slipMax, 0, 2)) * tempFactor;
      a.v.gain.gain.setTargetAtTime(gain, t, 0.04);
      a.v.filter.frequency.setTargetAtTime(freq, t, 0.06);
    }

    // --- kerb rumble: kerb strikes make the suspension oscillate fast, so
    // per-60Hz-frame |Δcompression| spikes an order of magnitude above smooth
    // cornering/braking weight transfer. Envelope was peak-held (with
    // exp(-dt/55) decay) in the bookkeeping block above; dead zone 8 mm per
    // nominal frame (sum over 4 wheels), full volume ~45 mm.
    const rumble = clamp((this.kerbEnv - 0.008) * 26, 0, 1) * speedGate;
    this.kerb.gain.gain.setTargetAtTime(rumble * 0.3, t, 0.03);
    // Thump pitch rises slightly with speed (kerb ridges arrive faster).
    this.kerb.filter.frequency.setTargetAtTime(85 + state.speed * 1.2, t, 0.1);
  }
}

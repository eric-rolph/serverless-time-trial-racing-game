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
  }

  /** Call from any user-gesture handler; idempotent. */
  start() {
    if (this.ctx) return;
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
    this.osc1.connect(this.engineFilter);
    this.osc2.connect(this.engineFilter);
    this.engineFilter.connect(this.engineGain);
    this.engineGain.connect(this.master);
    this.osc1.start();
    this.osc2.start();

    // One shared looping noise buffer fans out into all tire voices.
    const noiseBuf = ctx.createBuffer(1, ctx.sampleRate, ctx.sampleRate);
    const data = noiseBuf.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
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

    this.master.gain.linearRampToValueAtTime(0.5, ctx.currentTime + 0.5);
  }

  toggleMute() {
    this.muted = !this.muted;
    if (this.ctx) this.master.gain.value = this.muted ? 0 : 0.5;
    return this.muted;
  }

  /** state: SimStateV1 view {speed, wheels[]}; throttle 0..1; sim: the Sim
   *  wrapper for guarded tireTemp() reads (may be null). Call every frame. */
  update(state, throttle, sim = null) {
    if (!this.ctx || this.muted) return;
    const t = this.ctx.currentTime;

    // Fake RPM from speed with a throttle bump; single-gear kernel.
    const rpm = 900 + state.speed * 260 + throttle * 600;
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
    // frame-to-frame |Δcompression| spikes an order of magnitude above smooth
    // cornering/braking weight transfer. Peak-hold envelope with decay, dead
    // zone 8 mm/frame (sum over 4 wheels), full volume ~45 mm/frame.
    let delta = 0;
    for (let i = 0; i < 4; i++) {
      const c = state.wheels[i].compression ?? 0;
      delta += Math.abs(c - this.prevComp[i]);
      this.prevComp[i] = c;
    }
    this.kerbEnv = Math.max(this.kerbEnv * 0.82, delta);
    const rumble = clamp((this.kerbEnv - 0.008) * 26, 0, 1) * speedGate;
    this.kerb.gain.gain.setTargetAtTime(rumble * 0.3, t, 0.03);
    // Thump pitch rises slightly with speed (kerb ridges arrive faster).
    this.kerb.filter.frequency.setTargetAtTime(85 + state.speed * 1.2, t, 0.1);
  }
}

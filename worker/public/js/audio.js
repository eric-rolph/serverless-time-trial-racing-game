// SOTA Procedural Motorsport Audio Engine (WebAudio, zero external assets).
// Starts on first user gesture (browser autoplay policy); M toggles mute.
//
// Synthesis architecture:
//   • Multi-harmonic flat-plane racing engine: 0.5× crank sub, 1× fundamental,
//     1.5×/2×/3× firing-order harmonics → load-controlled asymmetric tanh
//     WaveShaper saturation → intake lowpass + exhaust chest formant resonance
//     → shift-cut/limiter duck stage → master dynamics compressor.
//   • Straight-cut sequential gearbox whine tracking speed & engaged gear.
//   • Off-throttle exhaust overrun pops/crackles on high-RPM lift & downshift.
//   • Velocity-squared aerodynamic wind rush (sells 250+ km/h straight speed).
//   • Physics-calibrated stereo-panned front/rear tire scrub + over-limit
//     high-Q squeal + kerb suspension transient thump + off-track gravel bed.
//
// Strictly read-only on SimStateV1 — zero physics coupling.

const PEAK_SLIP_ANGLE = 0.116; // rad ≈ 6.65° — grip peak of the brush tire model
const PEAK_SLIP_RATIO = 0.10;
const SPRING_RATE = 60000;     // N/m
const STATIC_LOAD = 3300;      // N
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));

/** Asymmetric soft-clipping tanh curve for rich combustion/exhaust harmonic grit. */
function makeDriveCurve(amount = 2.6) {
  const n = 512;
  const curve = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    const x = (i * 2) / (n - 1) - 1;
    // Slight even-order asymmetry (+0.08*x^2) gives authentic exhaust warmth
    curve[i] = Math.tanh((x + 0.08 * x * x) * amount) / Math.tanh(amount);
  }
  return curve;
}

export class EngineAudio {
  constructor() {
    this.ctx = null;
    this.muted = false;
    this.prevComp = [0, 0, 0, 0];
    this.kerbEnv = 0;
    this.prevGear = null;
    this.prevThrottle = 0;
    this.duckUntil = 0;
    this.popCooldown = 0;
  }

  /** Call from any user-gesture handler; idempotent. */
  start() {
    if (this.ctx) {
      if (this.ctx.state === "suspended") this.ctx.resume();
      return;
    }
    const ctx = new AudioContext();
    this.ctx = ctx;

    // Master bus + gentle dynamics compressor so loud kerbs/impacts never clip
    this.compressor = ctx.createDynamicsCompressor();
    this.compressor.threshold.value = -10;
    this.compressor.knee.value = 12;
    this.compressor.ratio.value = 4;
    this.compressor.attack.value = 0.004;
    this.compressor.release.value = 0.12;
    this.master = ctx.createGain();
    this.master.gain.value = 0.0;
    this.compressor.connect(this.master);
    this.master.connect(ctx.destination);
    this.bus = this.compressor;

    // ---- Multi-harmonic racing engine bank ---------------------------------
    const mkOsc = (type, gainVal) => {
      const osc = ctx.createOscillator();
      osc.type = type;
      const g = ctx.createGain();
      g.gain.value = gainVal;
      osc.connect(g);
      osc.start();
      return { osc, g };
    };
    this.engSub = mkOsc("triangle", 0.36);   // 0.5× crank rumble
    this.engFund = mkOsc("sawtooth", 0.42);  // 1.0× firing fundamental
    this.engMid = mkOsc("sawtooth", 0.24);   // 1.5× harmonic warmth
    this.engH2 = mkOsc("sawtooth", 0.20);    // 2.0× upper scream
    this.engH3 = mkOsc("triangle", 0.14);    // 3.0× flat-plane bite

    this.engPreDrive = ctx.createGain();
    this.engPreDrive.gain.value = 0.65;
    for (const o of [this.engSub, this.engFund, this.engMid, this.engH2, this.engH3]) {
      o.g.connect(this.engPreDrive);
    }

    // Soft-clipping exhaust saturation
    this.engShaper = ctx.createWaveShaper();
    this.engShaper.curve = makeDriveCurve(2.8);
    this.engShaper.oversample = "2x";

    // Intake lowpass + tailpipe resonant chest formant
    this.engineFilter = ctx.createBiquadFilter();
    this.engineFilter.type = "lowpass";
    this.engineFilter.frequency.value = 900;
    this.engineFilter.Q.value = 1.4;

    this.exhaustBody = ctx.createBiquadFilter();
    this.exhaustBody.type = "peaking";
    this.exhaustBody.frequency.value = 260;
    this.exhaustBody.Q.value = 1.8;
    this.exhaustBody.gain.value = 5.0;

    this.engineGain = ctx.createGain();
    this.engineGain.gain.value = 0.06;

    // Dedicated duck stage for shift cuts & redline fuel-cut stutter
    this.engineDuck = ctx.createGain();
    this.engineDuck.gain.value = 1.0;

    this.engPreDrive.connect(this.engShaper);
    this.engShaper.connect(this.engineFilter);
    this.engineFilter.connect(this.exhaustBody);
    this.exhaustBody.connect(this.engineGain);
    this.engineGain.connect(this.engineDuck);
    this.engineDuck.connect(this.bus);

    // ---- Straight-cut sequential gearbox whine -----------------------------
    this.gearOsc = ctx.createOscillator();
    this.gearOsc.type = "triangle";
    this.gearOsc.frequency.value = 320;
    this.gearFilter = ctx.createBiquadFilter();
    this.gearFilter.type = "bandpass";
    this.gearFilter.frequency.value = 1200;
    this.gearFilter.Q.value = 3.2;
    this.gearGain = ctx.createGain();
    this.gearGain.gain.value = 0;
    this.gearOsc.connect(this.gearFilter);
    this.gearFilter.connect(this.gearGain);
    this.gearGain.connect(this.bus);
    this.gearOsc.start();

    // ---- Shared looping white-noise source for tires, kerbs, wind & gravel -
    const noiseBuf = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate);
    const data = noiseBuf.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    this.noiseBuf = noiseBuf;
    this.noise = ctx.createBufferSource();
    this.noise.buffer = noiseBuf;
    this.noise.loop = true;

    const stereoVoice = (freq, Q) => {
      const filter = ctx.createBiquadFilter();
      filter.type = "bandpass";
      filter.frequency.value = freq;
      filter.Q.value = Q;
      const gain = ctx.createGain();
      gain.gain.value = 0;
      const pan = typeof ctx.createStereoPanner === "function" ? ctx.createStereoPanner() : null;
      this.noise.connect(filter);
      filter.connect(gain);
      if (pan) {
        gain.connect(pan);
        pan.connect(this.bus);
      } else {
        gain.connect(this.bus);
      }
      return { filter, gain, pan };
    };

    this.front = stereoVoice(1050, 2.4);    // front-axle scrub (understeer)
    this.rear = stereoVoice(880, 2.4);      // rear-axle scrub (oversteer)
    this.squeal = stereoVoice(1850, 6.2);   // high-Q over-limit rubber scream
    this.kerb = stereoVoice(115, 1.3);      // suspension kerb thump
    this.wind = stereoVoice(460, 0.7);      // aerodynamic airflow rush
    this.gravel = stereoVoice(240, 0.9);    // off-track grass/gravel rumble
    this.noise.start();

    this.master.gain.linearRampToValueAtTime(this.muted ? 0 : 0.56, ctx.currentTime + 0.45);
  }

  toggleMute() {
    this.muted = !this.muted;
    if (this.ctx) {
      const now = this.ctx.currentTime;
      this.master.gain.cancelScheduledValues(now);
      this.master.gain.setTargetAtTime(this.muted ? 0 : 0.56, now, 0.05);
    }
    return this.muted;
  }

  /** Fire-and-forget UI/event blip. */
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
    g.connect(this.bus);
    osc.start(t);
    osc.stop(t + dur + 0.02);
    osc.onended = () => {
      osc.disconnect();
      g.disconnect();
    };
  }

  /** One-shot exhaust overrun crackle/pop on downshift or high-RPM throttle lift. */
  _exhaustPop(strength = 1.0) {
    if (!this.ctx || this.muted) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;
    const src = ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const bp = ctx.createBiquadFilter();
    bp.type = "bandpass";
    bp.frequency.value = 180 + Math.random() * 260;
    bp.Q.value = 2.0;
    const g = ctx.createGain();
    const amp = 0.16 * clamp(strength, 0.3, 1.3);
    g.gain.setValueAtTime(amp, t);
    g.gain.exponentialRampToValueAtTime(0.001, t + 0.045 + Math.random() * 0.03);
    src.connect(bp);
    bp.connect(g);
    g.connect(this.bus);
    src.start(t, Math.random() * 0.5);
    src.stop(t + 0.09);
    src.onended = () => {
      src.disconnect();
      bp.disconnect();
      g.disconnect();
    };
  }

  /** One-shot collision crunch + low-frequency chassis thump. */
  impact(severity) {
    if (!this.ctx || this.muted) return;
    const amp = Math.min(1, severity) * 0.52;
    if (!(amp > 0)) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;

    const src = ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const lp = ctx.createBiquadFilter();
    lp.type = "lowpass";
    lp.frequency.value = 2600;
    const g = ctx.createGain();
    g.gain.setValueAtTime(amp, t);
    g.gain.exponentialRampToValueAtTime(0.001, t + 0.14);
    src.connect(lp);
    lp.connect(g);
    g.connect(this.bus);
    src.start(t);
    src.stop(t + 0.17);
    src.onended = () => {
      src.disconnect();
      lp.disconnect();
      g.disconnect();
    };

    const osc = ctx.createOscillator();
    osc.type = "sine";
    osc.frequency.setValueAtTime(75, t);
    osc.frequency.exponentialRampToValueAtTime(38, t + 0.2);
    const g2 = ctx.createGain();
    g2.gain.setValueAtTime(amp * 0.85, t);
    g2.gain.exponentialRampToValueAtTime(0.001, t + 0.22);
    osc.connect(g2);
    g2.connect(this.bus);
    osc.start(t);
    osc.stop(t + 0.26);
    osc.onended = () => {
      osc.disconnect();
      g2.disconnect();
    };
  }

  /** Gear-change punctuation: ~70 ms ignition cut + mechanical actuator clunk +
   *  exhaust bark on downshift. */
  shiftCut(isDownshift = false) {
    if (!this.ctx || this.muted) return;
    const ctx = this.ctx;
    const t = ctx.currentTime;
    const dg = this.engineDuck.gain;
    dg.cancelScheduledValues(t);
    dg.setValueAtTime(dg.value, t);
    dg.linearRampToValueAtTime(0.14, t + 0.012);
    dg.setValueAtTime(0.14, t + 0.056);
    dg.linearRampToValueAtTime(1.0, t + 0.085);
    this.duckUntil = t + 0.09;

    const src = ctx.createBufferSource();
    src.buffer = this.noiseBuf;
    const lp = ctx.createBiquadFilter();
    lp.type = "lowpass";
    lp.frequency.value = 540;
    const g = ctx.createGain();
    g.gain.setValueAtTime(0.24, t);
    g.gain.exponentialRampToValueAtTime(0.001, t + 0.055);
    src.connect(lp);
    lp.connect(g);
    g.connect(this.bus);
    src.start(t);
    src.stop(t + 0.08);
    src.onended = () => {
      src.disconnect();
      lp.disconnect();
      g.disconnect();
    };
    if (isDownshift) this._exhaustPop(1.15);
  }

  /** Per-frame update driven purely by read-only SimStateV1 + inputs. */
  update(state, throttle, sim = null, dtMs = 16.7, idle = false, offTrack = false) {
    if (!this.ctx) return;
    const dt = Math.max(0.1, dtMs);

    // Kerb transient envelope bookkeeping runs even while muted or idle
    let delta = 0;
    for (let i = 0; i < 4; i++) {
      const c = state.wheels[i].compression ?? 0;
      delta += Math.abs(c - this.prevComp[i]);
      this.prevComp[i] = c;
    }
    const deltaNorm = delta * (16.7 / dt);
    this.kerbEnv = Math.max(this.kerbEnv * Math.exp(-dt / 55), idle ? 0 : deltaNorm);
    this.popCooldown = Math.max(0, this.popCooldown - dt * 0.001);

    if (this.muted) return;
    const t = this.ctx.currentTime;

    if (idle) {
      // Gentle cammy motorsport idle (~900 rpm)
      const f = 900 / 30;
      const lope = 1 + 0.025 * Math.sin(t * 14);
      this.engSub.osc.frequency.setTargetAtTime(f * 0.5 * lope, t, 0.05);
      this.engFund.osc.frequency.setTargetAtTime(f * lope, t, 0.05);
      this.engMid.osc.frequency.setTargetAtTime(f * 1.5 + 1.8, t, 0.05);
      this.engH2.osc.frequency.setTargetAtTime(f * 2.0 - 1.5, t, 0.05);
      this.engH3.osc.frequency.setTargetAtTime(f * 3.0, t, 0.05);
      this.engPreDrive.gain.setTargetAtTime(0.35, t, 0.08);
      this.engineFilter.frequency.setTargetAtTime(340, t, 0.1);
      this.engineGain.gain.setTargetAtTime(0.055, t, 0.1);
      this.engineDuck.gain.setTargetAtTime(1, t, 0.05);
      this.gearGain.gain.setTargetAtTime(0, t, 0.05);
      this.wind.gain.gain.setTargetAtTime(0, t, 0.06);
      this.gravel.gain.gain.setTargetAtTime(0, t, 0.05);
      this.squeal.gain.gain.setTargetAtTime(0, t, 0.04);
      this.front.gain.gain.setTargetAtTime(0, t, 0.04);
      this.rear.gain.gain.setTargetAtTime(0, t, 0.04);
      this.kerb.gain.gain.setTargetAtTime(0, t, 0.03);
      this.prevGear = null;
      this.prevThrottle = 0;
      return;
    }

    // ---- Engine harmonics + load saturation + shift cuts + overrun pops ----
    const realRpm = sim?.rpm ? sim.rpm() : null;
    const rpm = realRpm ?? (900 + state.speed * 260 + throttle * 600);
    const gear = sim?.gear ? sim.gear() : 1;

    if (realRpm !== null) {
      if (gear !== null && this.prevGear !== null && gear !== this.prevGear) {
        this.shiftCut(gear < this.prevGear && gear > 0);
      }
      if (gear !== null) this.prevGear = gear;

      // Hard 15 Hz fuel-cut limiter bounce near 7500 rpm
      if (t >= this.duckUntil) {
        const pinned = rpm >= 7350 && throttle > 0.5;
        const gate = Math.floor(t * 30) % 2 === 0;
        this.engineDuck.gain.setTargetAtTime(pinned && !gate ? 0.08 : 1.0, t, 0.004);
      }
    }

    // Overrun exhaust pops when lifting abruptly from throttle at high RPM
    const lifted = this.prevThrottle > 0.45 && throttle < 0.12 && rpm > 4200;
    if ((lifted || (throttle < 0.05 && rpm > 4600 && Math.random() < dt * 0.0035)) && this.popCooldown <= 0) {
      this._exhaustPop(clamp((rpm - 3500) / 3500, 0.4, 1.2));
      this.popCooldown = lifted ? 0.08 : 0.16 + Math.random() * 0.22;
    }
    this.prevThrottle = throttle;

    const rpmNorm = clamp((rpm - 900) / 6600, 0, 1);
    const f = rpm / 30;
    this.engSub.osc.frequency.setTargetAtTime(f * 0.5, t, 0.018);
    this.engFund.osc.frequency.setTargetAtTime(f, t, 0.018);
    this.engMid.osc.frequency.setTargetAtTime(f * 1.5 + 2.2, t, 0.018);
    this.engH2.osc.frequency.setTargetAtTime(f * 2.0 - 2.0, t, 0.018);
    this.engH3.osc.frequency.setTargetAtTime(f * 3.0 + 3.0, t, 0.018);

    // Upper harmonics bite harder as throttle + RPM climb
    this.engH2.g.gain.setTargetAtTime(0.14 + throttle * 0.18 + rpmNorm * 0.12, t, 0.04);
    this.engH3.g.gain.setTargetAtTime(0.06 + throttle * 0.14 + rpmNorm * 0.10, t, 0.04);
    this.engPreDrive.gain.setTargetAtTime(0.45 + throttle * 0.85 + rpmNorm * 0.25, t, 0.04);

    // Intake & exhaust resonances open dramatically under load
    this.engineFilter.frequency.setTargetAtTime(420 + throttle * 1850 + rpmNorm * 1450, t, 0.04);
    this.exhaustBody.frequency.setTargetAtTime(210 + rpmNorm * 240, t, 0.06);
    this.engineGain.gain.setTargetAtTime(0.065 + throttle * 0.065 + rpmNorm * 0.035, t, 0.04);

    // ---- Straight-cut sequential transmission whine ------------------------
    const speedNorm = clamp(state.speed / 72, 0, 1.2);
    const gearMult = gear && gear > 0 ? 1.0 + (7 - gear) * 0.12 : 1.3;
    const whineHz = 280 + state.speed * 26 * gearMult;
    const whineVol = clamp((state.speed - 6) / 40, 0, 1) * (0.014 + throttle * 0.018);
    this.gearOsc.frequency.setTargetAtTime(whineHz, t, 0.03);
    this.gearFilter.frequency.setTargetAtTime(Math.min(4800, whineHz * 1.6), t, 0.04);
    this.gearGain.gain.setTargetAtTime(whineVol, t, 0.05);

    // ---- Aerodynamic wind rush (scales with v^2) ---------------------------
    const windGain = speedNorm * speedNorm * 0.14;
    this.wind.gain.gain.setTargetAtTime(windGain, t, 0.08);
    this.wind.filter.frequency.setTargetAtTime(280 + speedNorm * 680, t, 0.08);

    // ---- Physics-calibrated stereo tire scrub + over-limit rubber squeal ---
    const speedGate = clamp((state.speed - 3) / 6, 0, 1);
    const axles = [
      { v: this.front, wheels: [0, 1], base: 1050 },
      { v: this.rear, wheels: [2, 3], base: 880 },
    ];
    let worstOver = 0;
    let worstPan = 0;
    for (const a of axles) {
      let vol = 0, slipMax = 0, tempSum = 0, tempN = 0;
      const sideLoad = [0, 0];
      a.wheels.forEach((wi, sideIdx) => {
        const w = state.wheels[wi];
        const slipNorm = Math.max(
          Math.abs(w.slipAngle ?? 0) / PEAK_SLIP_ANGLE,
          Math.abs(w.slipRatio ?? 0) / PEAK_SLIP_RATIO,
        );
        const onset = clamp((slipNorm - 0.72) / 0.28, 0, 1);
        const load = clamp(((w.compression ?? 0) * SPRING_RATE) / STATIC_LOAD, 0, 1.4);
        const contrib = onset * load;
        vol += contrib;
        sideLoad[sideIdx] = contrib;
        slipMax = Math.max(slipMax, slipNorm);
        if (slipNorm > 1.0 && contrib > worstOver) {
          worstOver = clamp((slipNorm - 1.0) * load, 0, 1.4);
          worstPan = sideIdx === 0 ? -0.55 : 0.55;
        }
        const temp = sim?.tireTemp ? sim.tireTemp(wi) : null;
        if (temp !== null && temp !== undefined) {
          tempSum += temp;
          tempN++;
        }
      });
      const gain = speedGate * clamp(vol / 2, 0, 1) * 0.14;
      const tempFactor = tempN ? clamp(1 - (tempSum / tempN - 80) * 0.004, 0.82, 1.14) : 1;
      const freq = a.base * (0.9 + 0.3 * clamp(slipMax, 0, 2)) * tempFactor;
      a.v.gain.gain.setTargetAtTime(gain, t, 0.04);
      a.v.filter.frequency.setTargetAtTime(freq, t, 0.06);
      if (a.v.pan && vol > 0.01) {
        const panVal = clamp(((sideLoad[1] - sideLoad[0]) / vol) * 0.65, -0.65, 0.65);
        a.v.pan.pan.setTargetAtTime(panVal, t, 0.05);
      }
    }

    // High-Q over-limit tire scream (kicks in past peak slip = lockup / spin alert)
    const sqGain = speedGate * clamp(worstOver * 0.7, 0, 1) * 0.055;
    this.squeal.gain.gain.setTargetAtTime(sqGain, t, 0.04);
    this.squeal.filter.frequency.setTargetAtTime(1650 + clamp(worstOver, 0, 1.4) * 520, t, 0.05);
    if (this.squeal.pan) this.squeal.pan.pan.setTargetAtTime(worstPan, t, 0.06);

    // ---- Kerb suspension transient thump + off-track gravel/grass bed ------
    const rumble = clamp((this.kerbEnv - 0.008) * 26, 0, 1) * speedGate;
    this.kerb.gain.gain.setTargetAtTime(rumble * 0.32, t, 0.03);
    this.kerb.filter.frequency.setTargetAtTime(85 + state.speed * 1.2, t, 0.1);

    const gravelVol = offTrack ? speedGate * clamp(state.speed / 25, 0.15, 1) * 0.22 : 0;
    this.gravel.gain.gain.setTargetAtTime(gravelVol, t, 0.05);
    this.gravel.filter.frequency.setTargetAtTime(190 + state.speed * 6.5, t, 0.08);
  }
}

// Synthesized engine + tire audio (WebAudio, no assets). Starts on first user
// gesture (browser autoplay policy); M toggles mute.

export class EngineAudio {
  constructor() {
    this.ctx = null;
    this.muted = false;
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

    // Tire screech: filtered noise, gated by slip.
    const noiseBuf = ctx.createBuffer(1, ctx.sampleRate, ctx.sampleRate);
    const data = noiseBuf.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    this.noise = ctx.createBufferSource();
    this.noise.buffer = noiseBuf;
    this.noise.loop = true;
    this.screechFilter = ctx.createBiquadFilter();
    this.screechFilter.type = "bandpass";
    this.screechFilter.frequency.value = 900;
    this.screechFilter.Q.value = 2.5;
    this.screechGain = ctx.createGain();
    this.screechGain.gain.value = 0;
    this.noise.connect(this.screechFilter);
    this.screechFilter.connect(this.screechGain);
    this.screechGain.connect(this.master);
    this.noise.start();

    this.master.gain.linearRampToValueAtTime(0.5, ctx.currentTime + 0.5);
  }

  toggleMute() {
    this.muted = !this.muted;
    if (this.ctx) this.master.gain.value = this.muted ? 0 : 0.5;
    return this.muted;
  }

  /** state: {speed, wheels[]}; throttle 0..1. Call every render frame. */
  update(state, throttle) {
    if (!this.ctx || this.muted) return;
    const t = this.ctx.currentTime;
    // Fake RPM from speed with a throttle bump; single-gear kernel.
    const rpm = 900 + state.speed * 260 + throttle * 600;
    const f = rpm / 30; // Hz
    this.osc1.frequency.setTargetAtTime(f, t, 0.02);
    this.osc2.frequency.setTargetAtTime(f * 1.5 + 3, t, 0.02);
    this.engineFilter.frequency.setTargetAtTime(400 + throttle * 1400, t, 0.05);
    this.engineGain.gain.setTargetAtTime(0.04 + throttle * 0.05, t, 0.05);

    let slip = 0;
    for (const w of state.wheels) {
      slip = Math.max(slip, Math.abs(w.slipAngle ?? 0), Math.abs(w.slipRatio ?? 0));
    }
    const screech = state.speed > 4 ? Math.max(0, Math.min(1, (slip - 0.18) * 3)) : 0;
    this.screechGain.gain.setTargetAtTime(screech * 0.12, t, 0.05);
  }
}

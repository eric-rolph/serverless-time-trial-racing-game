// Unboxed APEX Pro Telemetry Overlay + Lap CSV Exporter (read-only on SimStateV1).
// Draws directly onto a transparent canvas with dark halo strokes (zero boxes):
//   • Rolling 4-second pedal & steering sparkline (throttle green, brake red,
//     steering cyan — Garage 61 / MoTeC style);
//   • Live vertical pedal + FFB rack-torque clipping meter;
//   • Live 2D G-force friction circle with fading trail & peak G readout;
//   • Compact 20 Hz lap telemetry ring/log for 1-click CSV export (KeyX).
const TRACE_LEN = 180; // ~3–4 s at 50–60 fps
const TRAIL_LEN = 26;
const GRAVITY = 9.81;

export class TelemetryHUD {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas ? canvas.getContext("2d") : null;
    this.w = canvas?.width ?? 310;
    this.h = canvas?.height ?? 118;
    this.visible = true;

    this.traceThrottle = new Float32Array(TRACE_LEN);
    this.traceBrake = new Float32Array(TRACE_LEN);
    this.traceSteer = new Float32Array(TRACE_LEN);
    this.head = 0;
    this.count = 0;

    this.gTrail = [];
    this.latG = 0;
    this.lonG = 0;
    this.peakG = 0;
    this.prevVel = null;

    this.ffbNm = 0;
    this.throttle = 0;
    this.brake = 0;
    this.steer = 0;

    // Lap CSV recorder (sampled at ~20 Hz so a 90 s lap is ~1800 lightweight rows)
    this.lapRows = [];
    this.lastCompletedRows = [];
    this._csvAcc = 0;
  }

  toggle() {
    this.visible = !this.visible;
    if (this.canvas) this.canvas.style.display = this.visible ? "block" : "none";
    return this.visible;
  }

  resetLap() {
    this.head = 0;
    this.count = 0;
    this.gTrail.length = 0;
    this.latG = 0;
    this.lonG = 0;
    this.peakG = 0;
    this.prevVel = null;
    this.lapRows = [];
    this._csvAcc = 0;
  }

  finishLap() {
    if (this.lapRows.length) this.lastCompletedRows = this.lapRows.slice();
  }

  /** Feed one render frame of read-only state + inputs. Returns { latG, lonG }
   *  so camera G-force head inertia can reuse the exact smoothed accelerations. */
  sample({ state, throttle, brake, steer, ffbNm = 0, rpm = 0, gear = 1, sim = null, dtSec = 0.016, idle = false, elapsedSec = 0 }) {
    const dt = Math.max(0.002, Math.min(dtSec, 0.1));
    this.throttle = throttle;
    this.brake = brake;
    this.steer = steer;
    this.ffbNm = ffbNm;

    // World velocity delta rotated into car-local axes via inverse chassis quat
    const v = state.linVel;
    if (this.prevVel && !idle) {
      const ax = (v[0] - this.prevVel[0]) / dt;
      const ay = (v[1] - this.prevVel[1]) / dt;
      const az = (v[2] - this.prevVel[2]) / dt;
      // Rotate world accel by conjugate of quat (qx, qy, qz, qw)
      const [qx, qy, qz, qw] = state.quat;
      const ix = -qx, iy = -qy, iz = -qz, iw = qw;
      // t = 2 * cross(q_xyz, a)
      const tx = 2 * (iy * az - iz * ay);
      const ty = 2 * (iz * ax - ix * az);
      const tz = 2 * (ix * ay - iy * ax);
      const lx = ax + iw * tx + (iy * tz - iz * ty);
      const lz = az + iw * tz + (ix * ty - iy * tx);
      const rawLat = Math.max(-3.5, Math.min(3.5, lx / GRAVITY));
      const rawLon = Math.max(-3.5, Math.min(3.5, lz / GRAVITY));
      const alpha = 1 - Math.exp(-12 * dt);
      this.latG += (rawLat - this.latG) * alpha;
      this.lonG += (rawLon - this.lonG) * alpha;
      const totalG = Math.hypot(this.latG, this.lonG);
      if (totalG > this.peakG) this.peakG = totalG;
    } else if (idle) {
      this.latG *= 0.85;
      this.lonG *= 0.85;
    }
    this.prevVel = [v[0], v[1], v[2]];

    if (!idle) {
      this.traceThrottle[this.head] = throttle;
      this.traceBrake[this.head] = brake;
      this.traceSteer[this.head] = steer;
      this.head = (this.head + 1) % TRACE_LEN;
      if (this.count < TRACE_LEN) this.count++;

      this.gTrail.push([this.latG, this.lonG]);
      if (this.gTrail.length > TRAIL_LEN) this.gTrail.shift();

      this._csvAcc += dt;
      if (this._csvAcc >= 0.05) {
        this._csvAcc = 0;
        const w = state.wheels;
        this.lapRows.push([
          elapsedSec.toFixed(3),
          state.lapProgress.toFixed(4),
          (state.speed * 3.6).toFixed(2),
          Math.round(rpm ?? 0),
          gear ?? 1,
          throttle.toFixed(3),
          brake.toFixed(3),
          steer.toFixed(3),
          this.latG.toFixed(3),
          this.lonG.toFixed(3),
          ffbNm.toFixed(2),
          (w[0]?.slipAngle ?? 0).toFixed(3),
          (w[1]?.slipAngle ?? 0).toFixed(3),
          (w[2]?.slipAngle ?? 0).toFixed(3),
          (w[3]?.slipAngle ?? 0).toFixed(3),
          Math.round(sim?.tireTemp?.(0) ?? 0),
          Math.round(sim?.tireTemp?.(1) ?? 0),
          Math.round(sim?.tireTemp?.(2) ?? 0),
          Math.round(sim?.tireTemp?.(3) ?? 0),
        ]);
      }
    }

    return { latG: this.latG, lonG: this.lonG };
  }

  /** Export latest completed lap (or live lap so far) as a MoTeC-ready CSV file. */
  exportCSV(trackHashHex = "track") {
    const rows = this.lastCompletedRows.length ? this.lastCompletedRows : this.lapRows;
    if (!rows.length) return false;
    const header = [
      "time_s", "lap_progress", "speed_kmh", "rpm", "gear",
      "throttle", "brake", "steer", "lat_g", "lon_g", "ffb_nm",
      "slip_fl_rad", "slip_fr_rad", "slip_rl_rad", "slip_rr_rad",
      "temp_fl_c", "temp_fr_c", "temp_rl_c", "temp_rr_c",
    ].join(",");
    const csv = header + "\n" + rows.map((r) => r.join(",")).join("\n") + "\n";
    const blob = new Blob([csv], { type: "text/csv;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `sttr-telemetry-${trackHashHex.slice(0, 8)}.csv`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
    return true;
  }

  draw() {
    if (!this.visible || !this.ctx) return;
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.w, this.h);

    // ---- 1. Rolling 4s Pedal & Steering Trace (left 168 px) ----------------
    const gx = 6, gy = 14, gw = 158, gh = 78;
    // Subtle baseline & midline hairlines (unboxed)
    ctx.strokeStyle = "rgba(233,237,244,0.14)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(gx, gy + gh); ctx.lineTo(gx + gw, gy + gh);
    ctx.moveTo(gx, gy + gh * 0.5); ctx.lineTo(gx + gw, gy + gh * 0.5);
    ctx.moveTo(gx, gy); ctx.lineTo(gx + gw, gy);
    ctx.stroke();

    const drawSeries = (arr, color, mapY, lw = 1.6) => {
      if (this.count < 2) return;
      ctx.beginPath();
      for (let i = 0; i < this.count; i++) {
        const idx = (this.head - this.count + i + TRACE_LEN) % TRACE_LEN;
        const x = gx + (i / (TRACE_LEN - 1)) * gw;
        const y = mapY(arr[idx]);
        i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      }
      ctx.strokeStyle = "rgba(5,9,15,0.65)";
      ctx.lineWidth = lw + 2;
      ctx.stroke();
      ctx.strokeStyle = color;
      ctx.lineWidth = lw;
      ctx.stroke();
    };

    // Steering centered on midline, throttle & brake 0..1 bottom→top
    drawSeries(this.traceSteer, "rgba(34,224,208,0.85)", (v) => gy + gh * (0.5 - v * 0.45), 1.3);
    drawSeries(this.traceBrake, "#ff4b42", (v) => gy + gh * (1 - v * 0.96), 1.8);
    drawSeries(this.traceThrottle, "#37d67a", (v) => gy + gh * (1 - v * 0.96), 1.8);

    // Compact legend caption beneath trace
    ctx.font = '600 9px "JetBrains Mono", monospace';
    ctx.fillStyle = "rgba(152,162,174,0.85)";
    ctx.fillText("THR", gx, gy + gh + 14);
    ctx.fillStyle = "#37d67a"; ctx.fillRect(gx + 22, gy + gh + 8, 8, 3);
    ctx.fillStyle = "rgba(152,162,174,0.85)";
    ctx.fillText("BRK", gx + 38, gy + gh + 14);
    ctx.fillStyle = "#ff4b42"; ctx.fillRect(gx + 60, gy + gh + 8, 8, 3);
    ctx.fillStyle = "rgba(152,162,174,0.85)";
    ctx.fillText("STR", gx + 76, gy + gh + 14);
    ctx.fillStyle = "#22e0d0"; ctx.fillRect(gx + 98, gy + gh + 8, 8, 3);

    // ---- 2. Live Vertical Pedal + FFB Rack-Torque Clipping Bars ------------
    const bx = 176;
    const barW = 7;
    const drawBar = (x, val, color, clip = false) => {
      ctx.fillStyle = "rgba(233,237,244,0.14)";
      ctx.fillRect(x, gy, barW, gh);
      const h = Math.max(0, Math.min(1, val)) * gh;
      ctx.fillStyle = clip ? "#ff3b2e" : color;
      ctx.fillRect(x, gy + gh - h, barW, h);
    };
    drawBar(bx, this.brake, "#ff4b42");
    drawBar(bx + 11, this.throttle, "#37d67a");
    const ffbNorm = Math.min(1, Math.abs(this.ffbNm) / 18.0);
    drawBar(bx + 22, ffbNorm, "#22e0d0", Math.abs(this.ffbNm) > 16.5);
    ctx.fillStyle = "rgba(152,162,174,0.8)";
    ctx.font = '600 8px "JetBrains Mono", monospace';
    ctx.fillText("B", bx + 1, gy + gh + 13);
    ctx.fillText("T", bx + 12, gy + gh + 13);
    ctx.fillText("F", bx + 23, gy + gh + 13);

    // ---- 3. 2D G-Force Friction Circle (right 92 px) -----------------------
    const cx = 258, cy = 53, rad = 38;
    ctx.strokeStyle = "rgba(5,9,15,0.6)";
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(cx, cy, rad, 0, Math.PI * 2); ctx.stroke();

    ctx.strokeStyle = "rgba(233,237,244,0.22)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, rad, 0, Math.PI * 2); ctx.stroke(); // 1.5 G outer ring
    ctx.strokeStyle = "rgba(233,237,244,0.12)";
    ctx.beginPath(); ctx.arc(cx, cy, rad * (1 / 1.5), 0, Math.PI * 2); ctx.stroke(); // 1.0 G ring
    ctx.beginPath();
    ctx.moveTo(cx - rad, cy); ctx.lineTo(cx + rad, cy);
    ctx.moveTo(cx, cy - rad); ctx.lineTo(cx, cy + rad);
    ctx.stroke();

    const gToPx = (lat, lon) => [
      cx + Math.max(-1.6, Math.min(1.6, lat)) * (rad / 1.5),
      cy - Math.max(-1.6, Math.min(1.6, lon)) * (rad / 1.5),
    ];

    // Breadcrumb trail
    for (let i = 1; i < this.gTrail.length; i++) {
      const [x0, y0] = gToPx(this.gTrail[i - 1][0], this.gTrail[i - 1][1]);
      const [x1, y1] = gToPx(this.gTrail[i][0], this.gTrail[i][1]);
      ctx.strokeStyle = `rgba(34,224,208,${(0.55 * (i / this.gTrail.length)).toFixed(3)})`;
      ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    }

    // Live G dot
    const [gxDot, gyDot] = gToPx(this.latG, this.lonG);
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#22e0d0";
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(gxDot, gyDot, 3.6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

    // Numeric G readout
    ctx.font = '600 9.5px "JetBrains Mono", monospace';
    ctx.fillStyle = "#eef2f6";
    ctx.textAlign = "center";
    ctx.fillText(`${Math.hypot(this.latG, this.lonG).toFixed(2)}G  PK ${this.peakG.toFixed(1)}`, cx, gy + gh + 14);
    ctx.textAlign = "left";
  }
}

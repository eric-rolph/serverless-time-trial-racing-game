#!/usr/bin/env node
// End-to-end pipeline prover: drive a lap through the REAL sim.wasm with a
// pure-pursuit controller, emit a signed LAPLOG, optionally submit it to the
// referee Worker. Injected sensor-noise jitter is required to pass the
// telemetry heuristics — which is exactly the point of the demo.
//
// Usage:
//   node tools/autopilot_lap.mjs --wasm physics/out/sim.wasm \
//     --track assets/tracks/dev/track.bin [--out lap.laplog] \
//     [--submit wss://sttr-referee.<acct>.workers.dev/api/submit] [--name bot]
//     [--max-seconds 180] [--target-speed 22]

import { readFileSync, writeFileSync } from "node:fs";

// ---------------------------------------------------------------- args
const args = Object.fromEntries(
  process.argv.slice(2).reduce((acc, a, i, arr) => {
    if (a.startsWith("--")) acc.push([a.slice(2), arr[i + 1] ?? ""]);
    return acc;
  }, []),
);
const WASM = args.wasm ?? "physics/out/sim.wasm";
const TRACK = args.track ?? "assets/tracks/dev/track.bin";
const OUT = args.out ?? "lap.laplog";
const MAX_TICKS = Math.min(72_000, (Number(args["max-seconds"]) || 180) * 400);
const TARGET_SPEED = Number(args["target-speed"]) || 22; // m/s

// ---------------------------------------------------------------- TRK1 parse
const trackBytes = new Uint8Array(readFileSync(TRACK));
const tdv = new DataView(trackBytes.buffer, trackBytes.byteOffset, trackBytes.byteLength);
if (tdv.getUint32(0, true) !== 0x4b525453) throw new Error("bad TRK1 magic"); // "STRK"
const S = tdv.getUint32(16, true);
const center = [];
const tangents = [];
for (let i = 0; i < S; i++) {
  const o = 28 + 40 * i;
  center.push([tdv.getFloat32(o, true), tdv.getFloat32(o + 4, true), tdv.getFloat32(o + 8, true)]);
  tangents.push([tdv.getFloat32(o + 24, true), tdv.getFloat32(o + 28, true), tdv.getFloat32(o + 32, true)]);
}

function fnv1a64(bytes) {
  let h = 0xcbf29ce484222325n;
  for (const b of bytes) h = ((h ^ BigInt(b)) * 0x100000001b3n) & 0xffffffffffffffffn;
  return h;
}
const trackHash = fnv1a64(trackBytes);

// ---------------------------------------------------------------- sim host
const module = new WebAssembly.Module(readFileSync(WASM));
const imports = {};
for (const im of WebAssembly.Module.imports(module)) {
  if (im.kind !== "function") continue;
  (imports[im.module] ??= {})[im.name] = () => 0;
}
const sim = new WebAssembly.Instance(module, imports).exports;
if (sim.sim_abi_version() !== 1) throw new Error("ABI version mismatch");

const trackPtr = sim.sim_alloc(trackBytes.length);
new Uint8Array(sim.memory.buffer, trackPtr, trackBytes.length).set(trackBytes);
if (sim.sim_load_track(trackPtr, trackBytes.length) !== 0) throw new Error("track load failed");
sim.sim_reset();

const state = () => {
  const dv = new DataView(sim.memory.buffer, sim.sim_state_ptr(), 200);
  return {
    tick: dv.getUint32(0, true),
    pos: [dv.getFloat32(4, true), dv.getFloat32(8, true), dv.getFloat32(12, true)],
    quat: [dv.getFloat32(16, true), dv.getFloat32(20, true), dv.getFloat32(24, true), dv.getFloat32(28, true)],
    speed: dv.getFloat32(184, true),
    lapProgress: dv.getFloat32(188, true),
    checkpoints: dv.getUint32(192, true),
    lapTicks: dv.getUint32(196, true),
  };
};

// ---------------------------------------------------------------- controller
// Rotate world vector into car frame via inverse quaternion (q conjugate).
function worldToCar(q, v) {
  const [x, y, z, w] = [q[0], q[1], q[2], q[3]];
  // conj(q) * v * q
  const cx = -x, cy = -y, cz = -z;
  const uvx = cy * v[2] - cz * v[1], uvy = cz * v[0] - cx * v[2], uvz = cx * v[1] - cy * v[0];
  const uuvx = cy * uvz - cz * uvy, uuvy = cz * uvx - cx * uvz, uuvz = cx * uvy - cy * uvx;
  return [v[0] + 2 * (w * uvx + uuvx), v[1] + 2 * (w * uvy + uuvy), v[2] + 2 * (w * uvz + uuvz)];
}

// Deterministic-ish jitter (seeded LCG): emulates wheel ADC noise.
let lcg = 0xC0FFEE;
const rnd = () => ((lcg = (lcg * 1664525 + 1013904223) >>> 0), lcg / 0xffffffff - 0.5);

let nearest = 0;
function control(st) {
  // Advance nearest-sample index within a window (track is a loop).
  let best = Infinity;
  for (let k = -5; k <= 30; k++) {
    const i = (nearest + k + S) % S;
    const d = (center[i][0] - st.pos[0]) ** 2 + (center[i][2] - st.pos[2]) ** 2;
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  const lookaheadM = Math.min(35, Math.max(10, 8 + 0.6 * st.speed));
  const target = center[(nearest + Math.round(lookaheadM / 2.5)) % S];
  const local = worldToCar(st.quat, [target[0] - st.pos[0], 0, target[2] - st.pos[2]]);
  const angle = Math.atan2(local[0], Math.max(0.01, local[2])); // fwd = +Z in car frame
  const steer = Math.max(-1, Math.min(1, angle / 0.5236)); // normalize by 30° lock

  const cornering = Math.min(1, Math.abs(angle) * 2.5);
  const want = TARGET_SPEED * (1 - 0.55 * cornering);
  const throttle = st.speed < want ? 0.8 : 0.0;
  const brake = st.speed > want + 3 ? 0.6 : 0.0;
  return { steer, throttle, brake };
}

const q16 = (v, lo, hi, scale) => Math.max(lo, Math.min(hi, Math.round(v * scale)));

// ---------------------------------------------------------------- drive
const ticks = [];
let status = 0;
console.log(`driving: track=${trackHash.toString(16)} samples=${S} maxTicks=${MAX_TICKS}`);
for (let t = 0; t < MAX_TICKS; t++) {
  const st = state();
  const c = control(st);
  // Sensor-noise injection: ±~120 counts steering, small pedal noise.
  const steer = q16(c.steer + rnd() * 0.0075, -32767, 32767, 32767);
  const throttle = q16(Math.max(0, c.throttle + rnd() * 0.01), 0, 65535, 65535);
  const brake = q16(Math.max(0, c.brake + rnd() * 0.004), 0, 65535, 65535);
  ticks.push([steer, throttle, brake, 0]);
  status = sim.sim_step(steer, throttle, brake, 0) >>> 0;
  if (status & 0x80000000) throw new Error(`sim error at tick ${t}`);
  if (status & 0x8) throw new Error(`lap invalid at tick ${t} (corner cut)`);
  if (status & 0x2) break;
  if (t % 4000 === 0 && t > 0) {
    console.log(
      `  t=${(t / 400).toFixed(0)}s speed=${st.speed.toFixed(1)}m/s progress=${(st.lapProgress * 100).toFixed(0)}% cp=${st.checkpoints.toString(2)}`,
    );
  }
}
if (!(status & 0x2)) throw new Error("lap did not complete within budget");

const final = state();
const hashLo = BigInt(sim.sim_state_hash_lo() >>> 0);
const hashHi = BigInt(sim.sim_state_hash_hi() >>> 0);
const finalHash = (hashHi << 32n) | hashLo;
const lapTicks = sim.sim_lap_time_ticks();
console.log(`LAP COMPLETE: ${(lapTicks / 400).toFixed(3)}s over ${ticks.length} ticks, hash=${finalHash.toString(16)}`);

// ---------------------------------------------------------------- LAPLOG
const N = ticks.length;
const log = new ArrayBuffer(20 + 8 * N + 12);
const ldv = new DataView(log);
[0x53, 0x54, 0x4c, 0x47].forEach((b, i) => ldv.setUint8(i, b));
ldv.setUint16(4, 1, true);
ldv.setUint16(6, 400, true);
ldv.setBigUint64(8, trackHash, true);
ldv.setUint32(16, N, true);
ticks.forEach(([s, th, br, fl], i) => {
  ldv.setInt16(20 + 8 * i, s, true);
  ldv.setUint16(22 + 8 * i, th, true);
  ldv.setUint16(24 + 8 * i, br, true);
  ldv.setUint16(26 + 8 * i, fl, true);
});
ldv.setBigUint64(20 + 8 * N, finalHash, true);
ldv.setUint32(28 + 8 * N, lapTicks, true);
writeFileSync(OUT, Buffer.from(log));
console.log(`wrote ${log.byteLength} bytes -> ${OUT}`);

// ---------------------------------------------------------------- sign+submit
if (args.submit) {
  const kp = await crypto.subtle.generateKey({ name: "Ed25519" }, true, ["sign", "verify"]);
  const pubkey = new Uint8Array(await crypto.subtle.exportKey("raw", kp.publicKey));
  const tag = new TextEncoder().encode("sttr-lap-v1");
  const msg = new Uint8Array(tag.length + log.byteLength);
  msg.set(tag, 0);
  msg.set(new Uint8Array(log), tag.length);
  const sig = new Uint8Array(await crypto.subtle.sign({ name: "Ed25519" }, kp.privateKey, msg));

  console.log(`submitting to ${args.submit} ...`);
  const ws = new WebSocket(args.submit);
  ws.binaryType = "arraybuffer";
  await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error("timeout")), 60_000);
    ws.onopen = () => {
      ws.send(
        JSON.stringify({
          type: "submit",
          pubkey: Buffer.from(pubkey).toString("base64"),
          sig: Buffer.from(sig).toString("base64"),
          name: args.name ?? "autopilot",
          logBytes: log.byteLength,
        }),
      );
      ws.send(log);
    };
    ws.onmessage = (ev) => {
      console.log("  server:", ev.data);
      const msg = JSON.parse(ev.data);
      if (msg.type === "result") {
        clearTimeout(timeout);
        ws.close();
        msg.status === "accepted" ? resolve() : reject(new Error(`rejected: ${msg.reason} ${msg.detail ?? ""}`));
      }
    };
    ws.onerror = (e) => {
      clearTimeout(timeout);
      reject(new Error(`ws error: ${e.message ?? e}`));
    };
  });
  console.log("SUBMISSION ACCEPTED ✔");
}

// wasm_smoke.mjs — proves a JS host (the Cloudflare Worker / V8) can run
// physics/out/sim.wasm: instantiate with minimal WASI stubs, load a flat-oval
// TRK1 blob built in JS, run 4000 scripted ticks twice — once via sim_step,
// once via one sim_replay call on a packed LAPLOG tick array — and assert both
// paths produce the identical state hash.
//
// Run: node physics/tests/wasm_smoke.mjs

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = join(here, "..", "out", "sim.wasm");

// ---------------------------------------------------------------------------
// Flat oval TRK1 blob (mirrors tests/make_test_track.c)
// ---------------------------------------------------------------------------

function buildTrack() {
  const S = 256, C = 3, RX = 60, RZ = 40, WIDTH = 10, HALF = 7;
  const V = 2 * S, T = 2 * S;
  const len = 28 + 40 * S + 4 * C + 12 * V + 12 * T + 12;
  const buf = new ArrayBuffer(len);
  const dv = new DataView(buf);
  let off = 0;
  const u16 = (v) => { dv.setUint16(off, v, true); off += 2; };
  const u32 = (v) => { dv.setUint32(off, v, true); off += 4; };
  const u64 = (v) => { dv.setBigUint64(off, BigInt(v), true); off += 8; };
  const f32 = (v) => { dv.setFloat32(off, v, true); off += 4; };

  u32(0x4b525453); // "STRK"
  u16(1);
  u16(C);
  u64(0x7e57da7a);
  u32(S); u32(V); u32(T);

  const px = new Array(S), pz = new Array(S), sx = new Array(S), sz = new Array(S);
  let tx0 = 0, tz0 = 0;
  for (let i = 0; i < S; i++) {
    const th = (2 * Math.PI * i) / S;
    const posx = Math.fround(RX * Math.sin(th));
    const posz = Math.fround(RZ * Math.cos(th));
    const dx = RX * Math.cos(th), dz = -RZ * Math.sin(th);
    const tl = Math.sqrt(dx * dx + dz * dz);
    const tanx = Math.fround(dx / tl), tanz = Math.fround(dz / tl);
    const sidex = tanz, sidez = -tanx;
    px[i] = posx; pz[i] = posz; sx[i] = sidex; sz[i] = sidez;
    if (i === 0) { tx0 = tanx; tz0 = tanz; }
    f32(posx); f32(0); f32(posz);
    f32(0); f32(1); f32(0);      // up
    f32(tanx); f32(0); f32(tanz); // tangent
    f32(WIDTH);
  }
  u32(S / 4); u32(S / 2); u32((3 * S) / 4); // checkpoints

  for (let i = 0; i < S; i++) {
    f32(px[i] - HALF * sx[i]); f32(0); f32(pz[i] - HALF * sz[i]);
    f32(px[i] + HALF * sx[i]); f32(0); f32(pz[i] + HALF * sz[i]);
  }
  for (let i = 0; i < S; i++) {
    const j = (i + 1) % S;
    const l0 = 2 * i, r0 = 2 * i + 1, l1 = 2 * j, r1 = 2 * j + 1;
    const ax = px[i] - HALF * sx[i], az = pz[i] - HALF * sz[i];
    const bx = px[j] - HALF * sx[j], bz = pz[j] - HALF * sz[j];
    const cx = px[i] + HALF * sx[i], cz = pz[i] + HALF * sz[i];
    const ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
    if (ny > 0) { u32(l0); u32(l1); u32(r0); u32(r0); u32(l1); u32(r1); }
    else { u32(l0); u32(r0); u32(l1); u32(l1); u32(r0); u32(r1); }
  }
  u32(0);                       // spawn index
  f32(Math.atan2(tx0, tz0));    // yaw (forward = +Z at yaw 0)
  f32(0);                       // reserved
  if (off !== len) throw new Error(`track writer mismatch ${off} != ${len}`);
  return new Uint8Array(buf);
}

// Scripted inputs, quantized — the integers are the canonical truth.
function scriptTick(t) {
  if (t < 1200) return { steer: 0, throttle: 65535, brake: 0, flags: 0 };
  if (t < 3000) {
    const phase = ((t - 1200) / 1800) * 2 * Math.PI;
    return { steer: Math.round(18000 * Math.sin(phase)) | 0, throttle: 39321, brake: 0, flags: 0 };
  }
  if (t < 3600) return { steer: 0, throttle: 0, brake: 65535, flags: 0 };
  return { steer: -6000, throttle: 52428, brake: 0, flags: t > 3900 ? 1 : 0 };
}

const TICKS = 4000;

// ---------------------------------------------------------------------------
// Instantiate with minimal stubs (only what the module actually imports).
// ---------------------------------------------------------------------------

const bytes = readFileSync(wasmPath);
const module = new WebAssembly.Module(bytes);

const imports = {};
for (const imp of WebAssembly.Module.imports(module)) {
  imports[imp.module] ??= {};
  if (imp.kind === "function") {
    // no-op stubs: fd_write/proc_exit/clock etc. are never on the hot path
    imports[imp.module][imp.name] = () => 0;
  }
}

const instance = new WebAssembly.Instance(module, imports);
const ex = instance.exports;
ex._initialize?.(); // run wasm ctors (STANDALONE_WASM reactor)

if (ex.sim_abi_version() !== 1) throw new Error("sim_abi_version != 1");
if (ex.sim_state_size() !== 200) throw new Error("sim_state_size != 200");

// --- load track ---
const track = buildTrack();
const trackPtr = ex.sim_alloc(track.length);
if (trackPtr === 0) throw new Error("sim_alloc failed");
new Uint8Array(ex.memory.buffer, trackPtr, track.length).set(track);
const rc = ex.sim_load_track(trackPtr, track.length);
if (rc !== 0) throw new Error(`sim_load_track failed: ${rc}`);

const hash = () =>
  (BigInt(ex.sim_state_hash_hi() >>> 0) << 32n) | BigInt(ex.sim_state_hash_lo() >>> 0);
const hex = (h) => h.toString(16).padStart(16, "0");

// --- path 1: per-tick sim_step ---
ex.sim_reset();
let status = 0;
for (let t = 0; t < TICKS; t++) {
  const { steer, throttle, brake, flags } = scriptTick(t);
  status = ex.sim_step(steer, throttle, brake, flags);
  if (status & 0x80000000) throw new Error(`sim_step ERROR at tick ${t}`);
}
const hashStep = hash();
console.log(`sim_step path:   ${TICKS} ticks, status=0x${status.toString(16)}, hash=${hex(hashStep)}`);

// --- path 2: one sim_replay call on a packed 8-byte-record log ---
const logPtr = ex.sim_alloc(TICKS * 8);
if (logPtr === 0) throw new Error("sim_alloc(log) failed");
{
  const dv = new DataView(ex.memory.buffer, logPtr, TICKS * 8);
  for (let t = 0; t < TICKS; t++) {
    const { steer, throttle, brake, flags } = scriptTick(t);
    dv.setInt16(8 * t + 0, steer, true);
    dv.setUint16(8 * t + 2, throttle, true);
    dv.setUint16(8 * t + 4, brake, true);
    dv.setUint16(8 * t + 6, flags, true);
  }
}
ex.sim_reset();
const replayStatus = ex.sim_replay(logPtr, TICKS);
if (replayStatus & 0x80000000) throw new Error("sim_replay ERROR");
const hashReplay = hash();
console.log(`sim_replay path: ${TICKS} ticks, status=0x${replayStatus.toString(16)}, hash=${hex(hashReplay)}`);

if (hashStep !== hashReplay) {
  console.error("FAIL: sim_step and sim_replay hashes differ");
  process.exit(1);
}

// Sanity: the car actually moved and the state block is readable.
const state = new DataView(ex.memory.buffer, ex.sim_state_ptr(), 200);
const tick = state.getUint32(0, true);
const speed = state.getFloat32(184, true);
const progress = state.getFloat32(188, true);
console.log(`state: tick=${tick} speed=${speed.toFixed(2)} m/s lap_progress=${progress.toFixed(4)}`);
if (tick !== TICKS) throw new Error("tick counter mismatch");

console.log(`wasm_smoke PASS — hash=${hex(hashStep)}`);

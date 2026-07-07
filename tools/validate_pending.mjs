#!/usr/bin/env node
// Free-tier validation sweeper: replays pending:* submissions (parked by the
// Worker in VALIDATE_MODE=queue) against the CANONICAL sim.wasm + track served
// by the deployed Worker, then writes accepted laps to the leaderboard via the
// Cloudflare KV REST API. Runs on a GitHub Actions cron — costs nothing.
//
// Env: CLOUDFLARE_API_TOKEN (required), CF_ACCOUNT_ID, KV_NAMESPACE_ID, WORKER_URL

const TOKEN = process.env.CLOUDFLARE_API_TOKEN;
if (!TOKEN) throw new Error("CLOUDFLARE_API_TOKEN not set");
const ACCOUNT = process.env.CF_ACCOUNT_ID ?? "0d47baeaa7616f08763e6ab8b13aec34";
const NS = process.env.KV_NAMESPACE_ID ?? "427a0d7959b741f3bda621d63196711b";
const WORKER = process.env.WORKER_URL ?? "https://sttr-referee.ericrolph.workers.dev";
const KV = `https://api.cloudflare.com/client/v4/accounts/${ACCOUNT}/storage/kv/namespaces/${NS}`;
const AUTH = { Authorization: `Bearer ${TOKEN}` };

const kvList = async (prefix) =>
  (await (await fetch(`${KV}/keys?prefix=${encodeURIComponent(prefix)}&limit=1000`, { headers: AUTH })).json()).result ?? [];
const kvGet = (key) => fetch(`${KV}/values/${encodeURIComponent(key)}`, { headers: AUTH });
const kvPut = (key, value) =>
  fetch(`${KV}/values/${encodeURIComponent(key)}`, { method: "PUT", headers: AUTH, body: value });
const kvDelete = (key) => fetch(`${KV}/values/${encodeURIComponent(key)}`, { method: "DELETE", headers: AUTH });

// ---- canonical binaries from the deployment itself --------------------------
const simBytes = new Uint8Array(await (await fetch(`${WORKER}/api/sim/current`)).arrayBuffer());
const trackResp = await fetch(`${WORKER}/api/track/current`);
const trackBytes = new Uint8Array(await trackResp.arrayBuffer());
const trackHashHex = trackResp.headers.get("x-track-hash");
const module = new WebAssembly.Module(simBytes);
console.log(`canonical: sim=${simBytes.length}B track=${trackBytes.length}B hash=${trackHashHex}`);

function instantiate() {
  const imports = {};
  for (const im of WebAssembly.Module.imports(module)) {
    if (im.kind !== "function") continue;
    (imports[im.module] ??= {})[im.name] = () => 0;
  }
  return new WebAssembly.Instance(module, imports).exports;
}

const b64decode = (s) => Uint8Array.from(Buffer.from(s, "base64"));
const pad8 = (t) => String(t).padStart(8, "0");

async function verifySig(pubkey, sig, log) {
  const key = await crypto.subtle.importKey("raw", pubkey, { name: "Ed25519" }, false, ["verify"]);
  const tag = new TextEncoder().encode("sttr-lap-v1");
  const msg = new Uint8Array(tag.length + log.length);
  msg.set(tag, 0);
  msg.set(log, tag.length);
  return crypto.subtle.verify({ name: "Ed25519" }, key, sig, msg);
}

const pending = await kvList("pending:");
console.log(`${pending.length} pending submission(s)`);

for (const { name: key } of pending) {
  try {
    const entry = await (await kvGet(key)).json();
    const log = b64decode(entry.logB64);
    const dv = new DataView(log.buffer, log.byteOffset, log.byteLength);
    const tickCount = dv.getUint32(16, true);
    const logTrackHash = dv.getBigUint64(8, true).toString(16).padStart(16, "0");
    const finalHash = dv.getBigUint64(20 + 8 * tickCount, true);
    const claimed = dv.getUint32(28 + 8 * tickCount, true);

    if (logTrackHash !== trackHashHex) throw new Error(`track rotated (log=${logTrackHash})`);
    const pubkey = b64decode(entry.pubkey);
    if (entry.sig && !(await verifySig(pubkey, b64decode(entry.sig), log))) throw new Error("bad signature");

    const sim = instantiate();
    const tp = sim.sim_alloc(trackBytes.length);
    new Uint8Array(sim.memory.buffer, tp, trackBytes.length).set(trackBytes);
    if (sim.sim_load_track(tp, trackBytes.length) !== 0) throw new Error("track load failed");
    sim.sim_reset();
    const lp = sim.sim_alloc(8 * tickCount);
    new Uint8Array(sim.memory.buffer, lp, 8 * tickCount).set(log.subarray(20, 20 + 8 * tickCount));
    const status = sim.sim_replay(lp, tickCount) >>> 0;
    const hash = (BigInt(sim.sim_state_hash_hi() >>> 0) << 32n) | BigInt(sim.sim_state_hash_lo() >>> 0);
    const lapTicks = sim.sim_lap_time_ticks();

    if (!(status & 0x2) || status & 0x8) throw new Error(`lap not valid (status=0x${status.toString(16)})`);
    if (hash !== finalHash || lapTicks !== claimed) throw new Error("replay mismatch");

    // Personal-best dedupe + leaderboard write (mirrors worker/src/leaderboard.ts).
    const pubkeyHex = Buffer.from(pubkey).toString("hex");
    const bestKey = `best:${trackHashHex}:${pubkeyHex}`;
    const bestResp = await kvGet(bestKey);
    const prevTicks = bestResp.ok ? parseInt(await bestResp.text(), 10) : Infinity;
    if (lapTicks < prevTicks) {
      await kvPut(
        `lb:${trackHashHex}:${pad8(lapTicks)}:${pubkeyHex.slice(0, 16)}`,
        JSON.stringify({ pubkey: pubkeyHex, name: entry.name, ticks: lapTicks, ms: Math.round((lapTicks * 1000) / 400), submittedAt: entry.submittedAt, flags: entry.flags ?? [] }),
      );
      await kvPut(bestKey, String(lapTicks));
      if (Number.isFinite(prevTicks)) await kvDelete(`lb:${trackHashHex}:${pad8(prevTicks)}:${pubkeyHex.slice(0, 16)}`);
      console.log(`ACCEPTED ${key}: ${entry.name} ${(lapTicks / 400).toFixed(3)}s (PB)`);
    } else {
      console.log(`accepted ${key}: ${entry.name} — slower than PB, not listed`);
    }
    await kvDelete(key);
  } catch (err) {
    console.log(`REJECTED ${key}: ${err.message}`);
    await kvDelete(key); // rejected submissions don't get retried
  }
}
console.log("sweep complete");

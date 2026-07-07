// LAPLOG v1 serialization, Ed25519 identity (noble, key persisted locally),
// and WebSocket submission per CONTRACTS §§3-5.
import * as ed from "../vendor/ed25519.js";
import { fnv1a64 } from "./track.js";

const KEY_STORAGE = "sttr-ed25519-seed";

async function identity() {
  let hex = localStorage.getItem(KEY_STORAGE);
  if (!hex) {
    const seed = crypto.getRandomValues(new Uint8Array(32));
    hex = [...seed].map((b) => b.toString(16).padStart(2, "0")).join("");
    localStorage.setItem(KEY_STORAGE, hex);
  }
  const priv = new Uint8Array(hex.match(/../g).map((h) => parseInt(h, 16)));
  const pub = await ed.getPublicKeyAsync(priv);
  return { priv, pub };
}

export function buildLapLog(trackBytes, ticks, finalHash, lapTicks) {
  const n = ticks.length;
  const buf = new ArrayBuffer(20 + 8 * n + 12);
  const dv = new DataView(buf);
  [0x53, 0x54, 0x4c, 0x47].forEach((b, i) => dv.setUint8(i, b)); // "STLG"
  dv.setUint16(4, 1, true);
  dv.setUint16(6, 400, true);
  dv.setBigUint64(8, fnv1a64(trackBytes), true);
  dv.setUint32(16, n, true);
  ticks.forEach((t, i) => {
    dv.setInt16(20 + 8 * i, t.steer, true);
    dv.setUint16(22 + 8 * i, t.throttle, true);
    dv.setUint16(24 + 8 * i, t.brake, true);
    dv.setUint16(26 + 8 * i, t.flags, true);
  });
  dv.setBigUint64(20 + 8 * n, finalHash, true);
  dv.setUint32(28 + 8 * n, lapTicks, true);
  return new Uint8Array(buf);
}

const b64 = (bytes) => btoa(String.fromCharCode(...bytes));

export async function submitLap(log, name, onStage) {
  const { priv, pub } = await identity();
  const tag = new TextEncoder().encode("sttr-lap-v1");
  const msg = new Uint8Array(tag.length + log.length);
  msg.set(tag, 0);
  msg.set(log, tag.length);
  const sig = await ed.signAsync(msg, priv);

  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/api/submit`);
  ws.binaryType = "arraybuffer";
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => { ws.close(); reject(new Error("submit timeout")); }, 60_000);
    ws.onopen = () => {
      ws.send(JSON.stringify({ type: "submit", pubkey: b64(pub), sig: b64(sig), name, logBytes: log.length }));
      ws.send(log.buffer);
    };
    ws.onmessage = (ev) => {
      const m = JSON.parse(ev.data);
      if (m.type === "ack") onStage?.(m.stage);
      if (m.type === "result") { clearTimeout(timeout); ws.close(); resolve(m); }
    };
    ws.onerror = () => { clearTimeout(timeout); reject(new Error("websocket error")); };
  });
}

export async function fetchLeaderboard() {
  const r = await fetch("/api/leaderboard");
  if (!r.ok) throw new Error(`leaderboard: ${r.status}`);
  return (await r.json()).entries;
}

/** Download a leaderboard lap's validated input log (LAPLOG bytes). */
export async function fetchReplay(player16) {
  const r = await fetch(`/api/replay?player=${player16}`);
  if (r.status === 404) throw new Error("no replay stored for this lap");
  if (!r.ok) throw new Error(`replay: ${r.status}`);
  return r.arrayBuffer();
}

export const fmtMs = (ms) => {
  const m = Math.floor(ms / 60000), s = Math.floor((ms % 60000) / 1000), f = Math.round(ms % 1000);
  return `${m}:${String(s).padStart(2, "0")}.${String(f).padStart(3, "0")}`;
};

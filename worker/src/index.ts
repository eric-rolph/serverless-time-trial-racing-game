// The asynchronous referee (docs/ARCHITECTURE.md). Pipeline per submission:
//   verify signature → telemetry heuristics → wasm replay → KV leaderboard.

// Bundled artifacts — wrangler rules: CompiledWasm for .wasm, Data for .bin.
// CI copies the freshly built sim.wasm + weekly track here before deploy.
// sim_wasm.bin is the same bytes as sim.wasm, imported as Data so we can SERVE
// the canonical binary to clients — a self-built sim.wasm from a different
// compiler version would hash-mismatch on replay.
import SIM_WASM from "../assets/sim.wasm";
import SIM_WASM_BYTES from "../assets/sim_wasm.bin";
import TRACK_BIN from "../assets/track.bin";

import { analyzeSteer } from "./heuristics";
import { recordLap, topEntries } from "./leaderboard";
import {
  b64decode,
  fnv1a64,
  hex64,
  LogFormatError,
  parseLapLog,
  parseSubmitHeader,
  ticksToMs,
  type SubmitHeader,
} from "./protocol";
import { replayLap } from "./replay";
import { verifySignature } from "./verify";

export interface Env {
  LEADERBOARD: KVNamespace;
  /** "edge" (default): replay at the edge, needs Workers Paid CPU budget.
   *  "queue": free-plan mode — verify signature + heuristics here, park the
   *  log in KV as pending:*, and let the GitHub Actions sweeper
   *  (.github/workflows/validate-pending.yml) replay it within ~30 min. */
  VALIDATE_MODE?: string;
}

const trackBytes = new Uint8Array(TRACK_BIN as ArrayBuffer);
// Computed once per isolate; identity of the currently deployed track.
const trackHash = fnv1a64(trackBytes);
const trackHashHex = hex64(trackHash);

const json = (obj: unknown, status = 200, headers: Record<string, string> = {}) =>
  new Response(JSON.stringify(obj), {
    status,
    headers: { "content-type": "application/json", "access-control-allow-origin": "*", ...headers },
  });

export default {
  async fetch(request: Request, env: Env, ctx: ExecutionContext): Promise<Response> {
    const url = new URL(request.url);

    switch (url.pathname) {
      case "/":
        return json({
          service: "sttr-referee",
          rev: 5,
          track: trackHashHex,
          endpoints: ["/api/submit (ws)", "/api/leaderboard", "/api/track/current", "/api/sim/current", "/api/replay?track&player"],
        });

      case "/api/sim/current":
        return new Response(SIM_WASM_BYTES, {
          headers: {
            "content-type": "application/wasm",
            "x-sim-hash": hex64(fnv1a64(new Uint8Array(SIM_WASM_BYTES as ArrayBuffer))),
            "access-control-allow-origin": "*",
          },
        });

      case "/api/replay": {
        const t = url.searchParams.get("track") ?? trackHashHex;
        const player = url.searchParams.get("player") ?? "";
        if (!/^[0-9a-f]{16}$/.test(t) || !/^[0-9a-f]{16}$/.test(player)) {
          return json({ error: "bad track/player id" }, 400);
        }
        const bytes = await env.LEADERBOARD.get(`log:${t}:${player}`, "arrayBuffer");
        if (!bytes) return json({ error: "no replay stored" }, 404);
        return new Response(bytes, {
          headers: { "content-type": "application/octet-stream", "access-control-allow-origin": "*" },
        });
      }

      case "/api/track/current":
        return new Response(trackBytes, {
          headers: {
            "content-type": "application/octet-stream",
            "x-track-hash": trackHashHex,
            "access-control-allow-origin": "*",
          },
        });

      case "/api/leaderboard": {
        const track = url.searchParams.get("track") ?? trackHashHex;
        if (!/^[0-9a-f]{16}$/.test(track)) return json({ error: "bad track hash" }, 400);
        return json({ track, entries: await topEntries(env.LEADERBOARD, track) });
      }

      case "/api/submit": {
        if (request.headers.get("Upgrade") !== "websocket") {
          return json({ error: "expected websocket" }, 426);
        }
        const pair = new WebSocketPair();
        handleSubmission(pair[1], env, ctx);
        return new Response(null, { status: 101, webSocket: pair[0] });
      }

      default:
        return json({ error: "not found" }, 404);
    }
  },
} satisfies ExportedHandler<Env>;

function handleSubmission(ws: WebSocket, env: Env, ctx: ExecutionContext): void {
  ws.accept();
  let header: SubmitHeader | null = null;
  let settled = false;

  const reject = (reason: string, detail?: string) => {
    settled = true;
    ws.send(JSON.stringify({ type: "result", status: "rejected", reason, detail }));
    ws.close(1000);
  };

  ws.addEventListener("message", (event) => {
    if (settled) return;
    try {
      if (typeof event.data === "string") {
        if (header !== null) return reject("log_malformed", "duplicate header");
        header = parseSubmitHeader(event.data);
        ws.send(JSON.stringify({ type: "ack", stage: "received" }));
        return;
      }
      if (header === null) return reject("log_malformed", "binary before header");
      const h = header;
      settled = true;
      ctx.waitUntil(
        toBuffer(event.data)
          .then((raw) => processLog(ws, env, h, raw))
          .catch((err) => {
            ws.send(
              JSON.stringify({ type: "result", status: "rejected", reason: "log_malformed", detail: String(err) }),
            );
            ws.close(1011);
          }),
      );
    } catch (err) {
      reject(err instanceof LogFormatError ? "log_malformed" : "internal", String(err));
    }
  });
}

/** Runtimes deliver WebSocket binary frames as ArrayBuffer, a view, or a Blob
 *  depending on spec vintage — normalize all three, and name the type if it's
 *  something else so the rejection detail is diagnosable. */
async function toBuffer(data: unknown): Promise<ArrayBuffer> {
  if (data instanceof ArrayBuffer) return data;
  if (ArrayBuffer.isView(data)) {
    return data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength) as ArrayBuffer;
  }
  if (typeof Blob !== "undefined" && data instanceof Blob) return data.arrayBuffer();
  throw new Error(`unsupported binary frame type: ${Object.prototype.toString.call(data)}`);
}

async function processLog(
  ws: WebSocket,
  env: Env,
  header: SubmitHeader,
  raw: ArrayBuffer,
): Promise<void> {
  const rawBytes = new Uint8Array(raw);
  const done = (msg: Record<string, unknown>) => {
    ws.send(JSON.stringify(msg));
    ws.close(1000);
  };

  // Parse + track match --------------------------------------------------------
  let log;
  try {
    log = parseLapLog(raw);
  } catch (err) {
    return done({ type: "result", status: "rejected", reason: "log_malformed", detail: String(err) });
  }
  if (log.trackHash !== trackHash) {
    return done({
      type: "result",
      status: "rejected",
      reason: "track_mismatch",
      detail: `log=${hex64(log.trackHash)} active=${trackHashHex}`,
    });
  }

  // 1. Signature ---------------------------------------------------------------
  const pubkey = b64decode(header.pubkey, 32);
  const sig = b64decode(header.sig, 64);
  if (!(await verifySignature(pubkey, sig, rawBytes))) {
    return done({ type: "result", status: "rejected", reason: "bad_signature" });
  }
  ws.send(JSON.stringify({ type: "ack", stage: "verified" }));

  // 2. Telemetry heuristics ----------------------------------------------------
  const verdict = analyzeSteer(log.steer);
  if (verdict.rejected) {
    return done({
      type: "result",
      status: "rejected",
      reason: "heuristics_failed",
      detail: verdict.flags.join(","),
    });
  }
  ws.send(JSON.stringify({ type: "ack", stage: "heuristics" }));

  // Free-tier mode: defer the CPU-heavy replay to the Actions sweeper.
  if (env.VALIDATE_MODE === "queue") {
    let b64 = "";
    for (let i = 0; i < rawBytes.length; i += 0x8000) {
      b64 += btoa(String.fromCharCode(...rawBytes.subarray(i, i + 0x8000)));
    }
    const key = `pending:${Date.now()}:${crypto.randomUUID().slice(0, 8)}`;
    await env.LEADERBOARD.put(
      key,
      JSON.stringify({ name: header.name, pubkey: header.pubkey, sig: header.sig, logB64: b64, flags: verdict.flags, submittedAt: new Date().toISOString() }),
      { expirationTtl: 7 * 24 * 3600 },
    );
    return done({ type: "result", status: "pending", lapTimeMs: ticksToMs(log.claimedLapTicks) });
  }

  // 3+4. Headless replay through the same physics binary ------------------------
  const replay = replayLap(SIM_WASM, trackBytes, log, rawBytes);
  if (!replay.ok) {
    return done({ type: "result", status: "rejected", reason: "replay_mismatch", detail: replay.reason });
  }
  ws.send(JSON.stringify({ type: "ack", stage: "replayed" }));

  // 5. Leaderboard ---------------------------------------------------------------
  const pubkeyHex = [...pubkey].map((b) => b.toString(16).padStart(2, "0")).join("");
  const { improved, rank } = await recordLap(env.LEADERBOARD, trackHashHex, {
    pubkey: pubkeyHex,
    name: header.name,
    ticks: replay.lapTicks!,
    ms: replay.lapTimeMs!,
    submittedAt: new Date().toISOString(),
    flags: verdict.flags,
  });

  // Replay viewer: persist the validated log so anyone can race this lap as a
  // ghost. Input logs are public by design — they ARE the leaderboard entry.
  if (improved) {
    await env.LEADERBOARD.put(`log:${trackHashHex}:${pubkeyHex.slice(0, 16)}`, rawBytes.buffer as ArrayBuffer, {
      expirationTtl: 30 * 24 * 3600,
    });
  }

  done({ type: "result", status: "accepted", lapTimeMs: ticksToMs(replay.lapTicks!), rank });
}

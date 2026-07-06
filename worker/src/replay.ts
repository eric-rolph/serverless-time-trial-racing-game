// Stages 3+4 of the referee: instantiate the SAME sim.wasm the client ran, feed
// the input log through it headlessly at max compute speed, and compare the
// outcome against the claim (docs/CONTRACTS.md §1, ADR-001).

import { type LapLog, ticksToMs } from "./protocol";

// Status bits — CONTRACTS §1.3
const LAP_COMPLETE = 0x2;
const LAP_INVALID = 0x8;
const SIM_ERROR = 0x80000000;

export interface ReplayResult {
  ok: boolean;
  reason?: "sim_error" | "track_load_failed" | "lap_not_complete" | "lap_invalid" | "hash_mismatch" | "time_mismatch";
  lapTicks?: number;
  lapTimeMs?: number;
  stateHash?: bigint;
  replayCpuMs?: number;
}

interface SimExports {
  memory: WebAssembly.Memory;
  sim_abi_version(): number;
  sim_alloc(len: number): number;
  sim_load_track(ptr: number, len: number): number;
  sim_reset(): void;
  sim_replay(logPtr: number, tickCount: number): number;
  sim_state_hash_lo(): number;
  sim_state_hash_hi(): number;
  sim_lap_time_ticks(): number;
}

/** Build no-op stubs for every function import the module declares (e.g. WASI
 *  leftovers from a STANDALONE_WASM build). None are on the hot path. */
function stubImports(module: WebAssembly.Module): WebAssembly.Imports {
  const imports: WebAssembly.Imports = {};
  for (const im of WebAssembly.Module.imports(module)) {
    if (im.kind !== "function") continue;
    const mod = (imports[im.module] ??= {} as Record<string, WebAssembly.ImportValue>);
    (mod as Record<string, WebAssembly.ImportValue>)[im.name] =
      im.name === "proc_exit"
        ? (code: number) => {
            throw new Error(`sim called proc_exit(${code})`);
          }
        : () => 0;
  }
  return imports;
}

export function replayLap(
  simModule: WebAssembly.Module,
  trackBin: Uint8Array,
  log: LapLog,
  rawLog: Uint8Array,
): ReplayResult {
  const t0 = Date.now();
  const instance = new WebAssembly.Instance(simModule, stubImports(simModule));
  const sim = instance.exports as unknown as SimExports;

  if (sim.sim_abi_version() !== 1) return { ok: false, reason: "sim_error" };

  const trackPtr = sim.sim_alloc(trackBin.length);
  new Uint8Array(sim.memory.buffer, trackPtr, trackBin.length).set(trackBin);
  if (sim.sim_load_track(trackPtr, trackBin.length) !== 0) {
    return { ok: false, reason: "track_load_failed" };
  }
  sim.sim_reset();

  // Tick records live at rawLog[ticksByteOffset .. +8*tickCount] — copy them
  // verbatim into wasm memory; sim_replay consumes the exact quantized integers.
  const tickBytes = rawLog.subarray(log.ticksByteOffset, log.ticksByteOffset + 8 * log.tickCount);
  const logPtr = sim.sim_alloc(tickBytes.length);
  new Uint8Array(sim.memory.buffer, logPtr, tickBytes.length).set(tickBytes);

  const status = sim.sim_replay(logPtr, log.tickCount) >>> 0;
  const replayCpuMs = Date.now() - t0;

  if (status & SIM_ERROR) return { ok: false, reason: "sim_error", replayCpuMs };
  if (status & LAP_INVALID) return { ok: false, reason: "lap_invalid", replayCpuMs };
  if (!(status & LAP_COMPLETE)) return { ok: false, reason: "lap_not_complete", replayCpuMs };

  const stateHash =
    (BigInt(sim.sim_state_hash_hi() >>> 0) << 32n) | BigInt(sim.sim_state_hash_lo() >>> 0);
  const lapTicks = sim.sim_lap_time_ticks();

  if (stateHash !== log.finalStateHash) {
    return { ok: false, reason: "hash_mismatch", lapTicks, stateHash, replayCpuMs };
  }
  if (lapTicks !== log.claimedLapTicks) {
    return { ok: false, reason: "time_mismatch", lapTicks, stateHash, replayCpuMs };
  }
  return { ok: true, lapTicks, lapTimeMs: ticksToMs(lapTicks), stateHash, replayCpuMs };
}

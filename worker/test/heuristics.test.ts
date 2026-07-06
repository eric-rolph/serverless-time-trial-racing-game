import { describe, expect, it } from "vitest";
import { analyzeSteer } from "../src/heuristics";

/** Deterministic LCG so tests never flake. */
function lcg(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 0xffffffff;
  };
}

const N = 8000; // 20 s at 400 Hz

function humanLikeSteer(): Int16Array {
  // Slow steering sweeps + ADC-like noise: exactly what a real wheel produces.
  const rnd = lcg(42);
  const out = new Int16Array(N);
  for (let i = 0; i < N; i++) {
    const base = 18000 * Math.sin((2 * Math.PI * i) / 3000) + 4000 * Math.sin((2 * Math.PI * i) / 700);
    const noise = (rnd() - 0.5) * 240; // ±120 counts sensor noise
    out[i] = Math.max(-32767, Math.min(32767, Math.round(base + noise)));
  }
  return out;
}

function stepFunctionTAS(): Int16Array {
  // Classic scripted input: perfectly held values with instant transitions.
  const out = new Int16Array(N);
  const levels = [0, 15000, -22000, 8000, 0, -15000, 30000, 0];
  for (let i = 0; i < N; i++) out[i] = levels[Math.floor(i / (N / levels.length))];
  return out;
}

function perfectSineTAS(): Int16Array {
  const out = new Int16Array(N);
  for (let i = 0; i < N; i++) out[i] = Math.round(20000 * Math.sin((2 * Math.PI * i) / 1600));
  return out;
}

describe("analyzeSteer", () => {
  it("passes human-like noisy input with zero flags", () => {
    const v = analyzeSteer(humanLikeSteer());
    expect(v.flags).toEqual([]);
    expect(v.rejected).toBe(false);
    expect(v.skipped).toBe(false);
  });

  it("rejects step-function TAS input (>= 2 flags)", () => {
    const v = analyzeSteer(stepFunctionTAS());
    expect(v.flags.length).toBeGreaterThanOrEqual(2);
    expect(v.flags).toContain("flat_runs");
    expect(v.flags).toContain("no_sensor_noise");
    expect(v.rejected).toBe(true);
  });

  it("flags perfectly smoothed synthetic input without hard-rejecting on one signal", () => {
    const v = analyzeSteer(perfectSineTAS());
    expect(v.flags).toContain("spline_smooth");
    // ADR-006: single-flag entries survive but carry the flag for audit.
  });

  it("skips judgment on very short logs", () => {
    const v = analyzeSteer(new Int16Array(500));
    expect(v.skipped).toBe(true);
    expect(v.rejected).toBe(false);
  });
});

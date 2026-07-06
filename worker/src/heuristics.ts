// Stage 2 of the referee: telemetry heuristics (docs/CONTRACTS.md §6, ADR-006).
//
// Real hardware (wheel potentiometer/hall sensor through an ADC) leaves
// fingerprints: perpetual low-amplitude noise, irregular jerk, thousands of
// distinct quantization values. Synthetic TAS inputs are pathologically clean.
// Verdict is advisory-strict: reject only on >= 2 independent flags; replay
// validation remains the hard physics gate.

export interface HeuristicVerdict {
  flags: string[];
  score: number;
  metrics: {
    identicalRunRatio: number;
    jerkVariance: number;
    distinctValueRatio: number;
    microJitterEnergy: number | null;
    quietCoverage: number;
  };
  rejected: boolean;
  skipped: boolean;
}

const DEAD_ZONE = 1000; // i16 counts treated as "wheel centered"
const MIN_SAMPLES = 2000; // < 5 s of input: too short to judge fairly

const THRESH = {
  identicalRunRatio: 0.7, // >70% consecutive-identical outside dead zone
  jerkVariance: 1.0, // counts² — spline-smooth inputs sit near 0
  distinctValueRatio: 0.02, // <2% unique values ⇒ scripted step functions
  microJitterEnergy: 0.5, // mean |Δ| inside quiet windows — ADC never reads flat
  quietCoverageForJitter: 0.1,
};

export function analyzeSteer(steer: Int16Array): HeuristicVerdict {
  const n = steer.length;
  const empty = {
    identicalRunRatio: 0,
    jerkVariance: 0,
    distinctValueRatio: 0,
    microJitterEnergy: null,
    quietCoverage: 0,
  };
  if (n < MIN_SAMPLES) {
    return { flags: [], score: 0, metrics: empty, rejected: false, skipped: true };
  }

  // -- identical_run_ratio: repeats while off-center --------------------------
  let active = 0;
  let identical = 0;
  for (let i = 1; i < n; i++) {
    if (Math.abs(steer[i]) > DEAD_ZONE) {
      active++;
      if (steer[i] === steer[i - 1]) identical++;
    }
  }
  const identicalRunRatio = active > 100 ? identical / active : 0;

  // -- jerk_variance: variance of the second difference -----------------------
  let sum = 0;
  let sumSq = 0;
  for (let i = 2; i < n; i++) {
    const jerk = steer[i] - 2 * steer[i - 1] + steer[i - 2];
    sum += jerk;
    sumSq += jerk * jerk;
  }
  const m = n - 2;
  const jerkVariance = sumSq / m - (sum / m) * (sum / m);

  // -- distinct_value_ratio ----------------------------------------------------
  const seen = new Set<number>();
  for (let i = 0; i < n; i++) seen.add(steer[i]);
  const distinctValueRatio = seen.size / n;

  // -- micro_jitter_energy inside quiet windows --------------------------------
  const WINDOW = 200;
  let quietWindows = 0;
  let totalWindows = 0;
  let jitterSum = 0;
  let jitterCount = 0;
  for (let w = 0; w + WINDOW <= n; w += WINDOW) {
    totalWindows++;
    let lo = 32767;
    let hi = -32768;
    for (let i = w; i < w + WINDOW; i++) {
      if (steer[i] < lo) lo = steer[i];
      if (steer[i] > hi) hi = steer[i];
    }
    if (hi - lo < 500) {
      quietWindows++;
      for (let i = w + 1; i < w + WINDOW; i++) {
        jitterSum += Math.abs(steer[i] - steer[i - 1]);
        jitterCount++;
      }
    }
  }
  const quietCoverage = totalWindows > 0 ? quietWindows / totalWindows : 0;
  const microJitterEnergy = jitterCount > 0 ? jitterSum / jitterCount : null;

  const flags: string[] = [];
  if (identicalRunRatio > THRESH.identicalRunRatio) flags.push("flat_runs");
  if (jerkVariance < THRESH.jerkVariance) flags.push("spline_smooth");
  if (distinctValueRatio < THRESH.distinctValueRatio) flags.push("value_grid");
  if (
    quietCoverage > THRESH.quietCoverageForJitter &&
    microJitterEnergy !== null &&
    microJitterEnergy < THRESH.microJitterEnergy
  ) {
    flags.push("no_sensor_noise");
  }

  return {
    flags,
    score: flags.length,
    metrics: { identicalRunRatio, jerkVariance, distinctValueRatio, microJitterEnergy, quietCoverage },
    rejected: flags.length >= 2,
    skipped: false,
  };
}

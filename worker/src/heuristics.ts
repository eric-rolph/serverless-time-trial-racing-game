// Stage 2 of the referee: multi-channel telemetry heuristics
// (docs/CONTRACTS.md §6, ADR-006).
//
// Real hardware (wheel hall/optical encoder + load-cell/potentiometer pedals)
// leaves physical fingerprints: perpetual ADC micro-noise, bounded human limb
// slew rates, non-synchronized hand/foot transitions, and dense quantization.
// Synthetic TAS macros leave pathologically clean plateaus, impossible single-
// tick full-lock snaps, and exact 2.5 ms cross-channel step alignments.
//
// Verdict stays advisory-strict: reject only on >= 2 independent flags, and
// explicitly exempt binary 0 / 65535 keyboard pedal presses from analog pedal
// plateau checks so honest keyboard drivers are never false-flagged.

export interface HeuristicVerdict {
  flags: string[];
  score: number;
  metrics: {
    identicalRunRatio: number;
    jerkVariance: number;
    distinctValueRatio: number;
    microJitterEnergy: number | null;
    quietCoverage: number;
    slewViolations: number;
    pedalFlatRatio: number | null;
    syncSnapCount: number;
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
  slewStepCounts: 16_000, // ~50% lock jump in 2.5 ms (>7000 deg/s physical wheel)
  slewViolations: 8, // repeated superhuman snaps across a lap
  pedalFlatRatio: 0.92, // >92% flat intermediate analog pedal ticks
  pedalDistinctRatio: 0.015, // <1.5% unique intermediate pedal levels
  pedalMinIntermediate: 1000, // ignores binary 0/65535 keyboard presses
  syncSteerJump: 8_000,
  syncPedalJump: 30_000,
  syncSnapCount: 10,
};

function pedalStaircaseRatio(ch: Uint16Array): { flatRatio: number; distinctRatio: number; active: number } {
  let active = 0;
  let identical = 0;
  const seen = new Set<number>();
  for (let i = 1; i < ch.length; i++) {
    const v = ch[i];
    // Intermediate analog band only — binary 0 / 65535 keyboard holds ignored
    if (v > 500 && v < 65000) {
      active++;
      seen.add(v);
      if (v === ch[i - 1]) identical++;
    }
  }
  if (active < THRESH.pedalMinIntermediate) {
    return { flatRatio: 0, distinctRatio: 1, active };
  }
  return { flatRatio: identical / active, distinctRatio: seen.size / active, active };
}

/** Multi-channel Anti-TAS analysis across steer, throttle, and brake. */
export function analyzeTelemetry(
  steer: Int16Array,
  throttle?: Uint16Array,
  brake?: Uint16Array,
): HeuristicVerdict {
  const n = steer.length;
  const empty = {
    identicalRunRatio: 0,
    jerkVariance: 0,
    distinctValueRatio: 0,
    microJitterEnergy: null,
    quietCoverage: 0,
    slewViolations: 0,
    pedalFlatRatio: null,
    syncSnapCount: 0,
  };
  if (n < MIN_SAMPLES) {
    return { flags: [], score: 0, metrics: empty, rejected: false, skipped: true };
  }

  // -- identical_run_ratio & physiological slew ceiling -----------------------
  let active = 0;
  let identical = 0;
  let slewViolations = 0;
  for (let i = 1; i < n; i++) {
    const dv = Math.abs(steer[i] - steer[i - 1]);
    if (dv > THRESH.slewStepCounts) slewViolations++;
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

  // -- distinct_value_ratio ---------------------------------------------------
  const seen = new Set<number>();
  for (let i = 0; i < n; i++) seen.add(steer[i]);
  const distinctValueRatio = seen.size / n;

  // -- micro_jitter_energy inside quiet windows -------------------------------
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

  // -- multi-channel pedal staircase & frame-exact cross-channel snaps --------
  let pedalFlatRatio: number | null = null;
  let pedalStaircase = false;
  let syncSnapCount = 0;
  const hasPedals =
    throttle && brake && throttle.length === n && brake.length === n;

  if (hasPedals) {
    const tStats = pedalStaircaseRatio(throttle);
    const bStats = pedalStaircaseRatio(brake);
    if (tStats.active >= THRESH.pedalMinIntermediate || bStats.active >= THRESH.pedalMinIntermediate) {
      pedalFlatRatio = Math.max(tStats.flatRatio, bStats.flatRatio);
    }
    if (
      (tStats.active >= THRESH.pedalMinIntermediate &&
        tStats.flatRatio > THRESH.pedalFlatRatio &&
        tStats.distinctRatio < THRESH.pedalDistinctRatio) ||
      (bStats.active >= THRESH.pedalMinIntermediate &&
        bStats.flatRatio > THRESH.pedalFlatRatio &&
        bStats.distinctRatio < THRESH.pedalDistinctRatio)
    ) {
      pedalStaircase = true;
    }

    for (let i = 1; i < n; i++) {
      const ds = Math.abs(steer[i] - steer[i - 1]);
      if (ds <= THRESH.syncSteerJump) continue;
      const dt = Math.abs(throttle[i] - throttle[i - 1]);
      const db = Math.abs(brake[i] - brake[i - 1]);
      if (dt > THRESH.syncPedalJump || db > THRESH.syncPedalJump) {
        syncSnapCount++;
      }
    }
  }

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
  if (slewViolations >= THRESH.slewViolations) flags.push("inhuman_slew");
  if (pedalStaircase) flags.push("pedal_staircase");
  if (syncSnapCount >= THRESH.syncSnapCount) flags.push("sync_macro");

  return {
    flags,
    score: flags.length,
    metrics: {
      identicalRunRatio,
      jerkVariance,
      distinctValueRatio,
      microJitterEnergy,
      quietCoverage,
      slewViolations,
      pedalFlatRatio,
      syncSnapCount,
    },
    rejected: flags.length >= 2,
    skipped: false,
  };
}

/** Backward-compatible single-channel entry point. */
export function analyzeSteer(steer: Int16Array): HeuristicVerdict {
  return analyzeTelemetry(steer);
}

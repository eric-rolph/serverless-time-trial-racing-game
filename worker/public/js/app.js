// Serverless Time Trial — browser client. Same wasm physics, same referee,
// no install. Fixed 400 Hz accumulator + interpolated rendering (uncapped rAF).
import * as THREE from "../vendor/three.module.js";
import { Sim, STATUS, DT_MS } from "./sim.js";
import { parseTrack, buildTrackMeshes, Minimap, fnv1a64 } from "./track.js";
import { Input, quantize } from "./input.js";
import { buildLapLog, submitLap, fetchLeaderboard, fetchReplay, fmtMs } from "./lap.js";
import * as lapApi from "./lap.js";
import { EngineAudio } from "./audio.js";
import { buildCar, buildWheel } from "./car.js";
import { Crumple } from "./crumple.js";
import { buildAmbience } from "./ambience.js";
import { Lighting } from "./lighting.js";
import { FanatecFFB } from "./ffb.js";
import { addHidDevices, autoReconnectHid, hidDiagnostics, hidDump, hidVirtualPads, registerHidDevice } from "./hid-input.js";

const $ = (id) => document.getElementById(id);
const setStatus = (text, cls = "info") => { $("statusText").textContent = text; $("statusText").className = cls; };

// ---------------------------------------------------------------- bootstrap
setStatus("loading physics + track…");
// Backstop: any unhandled async failure surfaces on the status line instead of
// dying silently in the console.
addEventListener("unhandledrejection", (e) => {
  setStatus(`failed to load: ${e.reason?.message ?? e.reason} — reload to retry`, "bad");
});
let sim, trackBuf, track;
try {
  const [simLoaded, trackResp] = await Promise.all([Sim.load(), fetch("/api/track/current")]);
  // sim.wasm response .ok is checked inside Sim.load(); mirror it for the track.
  if (!trackResp.ok) throw new Error(`track fetch failed: ${trackResp.status}`);
  sim = simLoaded;
  trackBuf = await trackResp.arrayBuffer();
  track = parseTrack(trackBuf);
  sim.loadTrack(new Uint8Array(trackBuf));
} catch (err) {
  setStatus(`failed to load: ${err.message} — reload to retry`, "bad");
  throw err; // halt the module — nothing below can run without physics + track
}
const trackHash = fnv1a64(track.bytes);
const trackHashHex = trackHash.toString(16).padStart(16, "0");

$("name").value = localStorage.getItem("sttr-name") ?? "";
$("name").addEventListener("change", () => localStorage.setItem("sttr-name", $("name").value));
$("name").addEventListener("keydown", (e) => { if (e.key === "Enter") e.target.blur(); });

// ---------------------------------------------------------------- three.js
const renderer = new THREE.WebGLRenderer({ canvas: $("gl"), antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1; // per-condition, driven by Lighting.update
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap; // single 2048² map (Lighting)
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1420); // Lighting owns bg/fog from frame 0
scene.fog = new THREE.Fog(0x0e1420, 150, 480);
const camera = new THREE.PerspectiveCamera(62, 1, 0.1, 800);

scene.add(buildTrackMeshes(track));
// Trackside dressing — trees, tire barriers, gantry, brake boards — all
// seeded from the track hash so placement is deterministic per track.
const ambience = buildAmbience(track, trackHash);
scene.add(ambience);

const car = buildCar(0xd9483b, 1, false, true); // deformable body for crash denting
scene.add(car);
// Cosmetic crash deformation (SOFTBODY.md Phase 1). Only the player car owns a
// Crumple; the ghost keeps its own pristine geometries and gets none.
const crumple = new Crumple(car);
const ghostCar = buildCar(0xbfd9ff, 0.35, true); // translucent, with static wheels
ghostCar.visible = false;
scene.add(ghostCar);

const wheelMeshes = [];
for (let i = 0; i < 4; i++) {
  const holder = buildWheel(); // racing slick + rim, posed from physics below
  scene.add(holder);
  wheelMeshes.push(holder);
}

// APEX lighting core (cosmetic; auto = local clock; L cycles the five
// conditions). Owns sky dome + sun disc, the shadow-casting key light,
// hemisphere fill, per-condition exposure, and the car's night headlights.
// Constructed AFTER every scene mesh (car, wheels, track, ambience) so its
// one-time shadow cast/receive traversal sees them all.
const lighting = new Lighting(scene, car, ambience.userData.glowMaterials, renderer, track);

function resize() {
  renderer.setSize(innerWidth, innerHeight, false);
  camera.aspect = innerWidth / innerHeight; // aspect-native: 32:9 just works
  camera.updateProjectionMatrix();
}
addEventListener("resize", resize);
resize();

// ---------------------------------------------------------------- helpers
const b64encode = (bytes) => {
  let s = "";
  for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(s);
};
const b64decode = (s) => Uint8Array.from(atob(s), (c) => c.charCodeAt(0));

// ---------------------------------------------------------------- ghost (PB)
const BEST_KEY = `sttr-best-${trackHashHex}`;
const SECT_KEY = `sttr-sectors-${trackHashHex}`;
let ghostSim = null;
let ghost = null; // {lapTicks, ticks: DataView-able bytes, progress: Float32Array}
let ghostIdx = 0;
let ghostPrev = null;
let ghostCurr = null;
let deltaPtr = 0;

function loadGhost() {
  const stored = localStorage.getItem(BEST_KEY);
  if (!stored) return null;
  try {
    const { lapTicks, ticksB64, progressB64 } = JSON.parse(stored);
    const t = b64decode(ticksB64);
    return { lapTicks, ticks: new DataView(t.buffer, t.byteOffset, t.byteLength), progress: new Float32Array(b64decode(progressB64).buffer) };
  } catch {
    return null;
  }
}

/** Replay ticks through a scratch instance to get the per-tick lap-progress
 *  curve (the delta-to-best lookup table). ~100 ms for a full lap. */
async function buildProgress(dv, lapTicks) {
  const scratch = await Sim.instantiate();
  scratch.loadTrack(new Uint8Array(trackBuf));
  scratch.reset();
  const progress = new Float32Array(lapTicks);
  for (let i = 0; i < lapTicks; i++) {
    scratch.step(dv.getInt16(8 * i, true), dv.getUint16(8 * i + 2, true), dv.getUint16(8 * i + 4, true), dv.getUint16(8 * i + 6, true));
    progress[i] = scratch.state().lapProgress;
  }
  return progress;
}

async function saveGhost(tickRecords, lapTicks) {
  const bytes = new Uint8Array(8 * lapTicks);
  const dv = new DataView(bytes.buffer);
  for (let i = 0; i < lapTicks; i++) {
    dv.setInt16(8 * i, tickRecords[i].steer, true);
    dv.setUint16(8 * i + 2, tickRecords[i].throttle, true);
    dv.setUint16(8 * i + 4, tickRecords[i].brake, true);
    dv.setUint16(8 * i + 6, tickRecords[i].flags, true);
  }
  const progress = await buildProgress(dv, lapTicks);
  localStorage.setItem(
    BEST_KEY,
    JSON.stringify({ lapTicks, ticksB64: b64encode(bytes), progressB64: b64encode(new Uint8Array(progress.buffer)) }),
  );
}

/** Replay viewer: download a leaderboard lap and race it as the ghost. */
async function raceReplay(entry, rank = null) {
  try {
    setStatus(`downloading ${entry.name}'s lap…`);
    const buf = await fetchReplay(entry.pubkey.slice(0, 16));
    const dv = new DataView(buf);
    const tickCount = dv.getUint32(16, true);
    const lapTicks = dv.getUint32(28 + 8 * tickCount, true);
    const ticksDv = new DataView(buf, 20, 8 * tickCount);
    const progress = await buildProgress(ticksDv, lapTicks);
    sessionGhost = { lapTicks, ticks: ticksDv, progress };
    sessionGhostName = entry.name;
    sessionGhostRank = rank;
    await resetRun();
    setStatus(`racing ${entry.name}'s ${fmtMs(entry.ms)} lap — beat the ghost! (R restarts, ghost stays)`, "ok");
  } catch (err) {
    setStatus(`replay: ${err.message}`, "bad");
  }
}

// ---------------------------------------------------------------- game state
const input = new Input();
input.setVirtualPads(hidVirtualPads);
input.attachSim(sim); // auto-gearbox reads sim.rpm()/sim.gear() thresholds (DRIVETRAIN §6)
// Reopen previously-granted WebHID input devices (permission persists).
autoReconnectHid().then((n) => {
  if (n) $("calMsg").textContent = `${n} WebHID device(s) reconnected — bindings ready`;
});
const minimap = new Minimap($("minimap"), track);
const audio = new EngineAudio();

// Tire glyph: the four wheel rects of the car schematic, order FL FR RL RR.
const TIRE_LABELS = ["FL", "FR", "RL", "RR"];
const tireEls = ["fl", "fr", "rl", "rr"].map((p) => document.querySelector(`#tireTemps .wheel.${p}`));
/** Continuous temp → color scale: cold blue → operating-window green →
 *  overheated red (hue interpolated between anchor stops). */
function tireColor(t) {
  const stops = [[35, 212], [60, 148], [105, 96], [125, 6]]; // [°C, hue]
  if (t <= stops[0][0]) return `hsl(${stops[0][1]} 62% 56%)`;
  for (let i = 1; i < stops.length; i++) {
    if (t <= stops[i][0]) {
      const [a, ha] = stops[i - 1], [b, hb] = stops[i];
      return `hsl(${Math.round(ha + ((hb - ha) * (t - a)) / (b - a))} 62% 56%)`;
    }
  }
  return `hsl(${stops[stops.length - 1][1]} 62% 56%)`;
}
// Drivetrain HUD (ABI 1.4): gear numeral, AUTO/MANUAL tag, thin RPM bar.
// All hidden until the loaded binary exports sim_rpm — old wasm keeps the
// pre-gearbox HUD exactly as it was.
const gearEl = $("gear"), shiftModeEl = $("shiftMode"), rpmBarEl = $("rpmBar"), rpmFillEl = $("rpmFill");
const SHIFT_LIGHT_RPM = 7200; // cyan pulse threshold (DRIVETRAIN.md §6); redline (fuel cut) is 7500
const RPM_BAR_MAX = 8000;     // bar full scale — the redline tick sits at 7500/8000 = 93.75% (CSS)
let drivetrainHudOn = false;
/** AUTO/MANUAL per the shift-mode contract (DRIVETRAIN.md §6.6): prefer the
 *  input module's live `.shiftMode` ("auto"|"manual") when it exposes one,
 *  else the persisted localStorage key "sttr-shift-mode" that the input side
 *  keeps in sync. A missing key means "auto" — the spec's default-ON. */
const shiftModeLabel = () =>
  (input.shiftMode ?? localStorage.getItem("sttr-shift-mode") ?? "auto") === "manual" ? "MANUAL" : "AUTO";

const fellOffY = Math.min(...track.center.map((c) => c[1])) - 25;
let prev = sim.state();
let curr = prev;
let ticks = [];
let frozen = false;
let invalid = false;
let acc = 0;
let last = performance.now();
let bestMs = null;
let lastMs = null;
let inputOverride = null; // test hook: (state) => RawInput
let countdownMs = 0; // > 0 → standing start in progress
let started = false; // armed idle until the first keydown / pointerdown / pad input
let countdownHideTimer = 0; // pending auto-hide for the #countdown overlay
let countdownBeepN = 0; // last countdown integer beeped (0 = none yet this run)
// Camera seats (APEX "pick your seat"): C cycles chase → cockpit → bumper → TV.
const CAMERA_SEATS = ["chase", "cockpit", "bumper", "tv"];
const CAMERA_NAMES = { chase: "chase", cockpit: "cockpit", bumper: "bumper", tv: "TV" };
let cameraMode = "chase";
let ffb = null;
let lastThrottle = 0;
let sectorTicks = []; // ticks at S1/S2/S3 boundaries this run
let prevCpMask = 0;
let rolloverTicks = 0;
let sessionGhost = null; // downloaded leaderboard lap (replay viewer)
let sessionGhostName = null;
let sessionGhostRank = null; // leaderboard rank of the downloaded lap (1 = P1)

/** Text overlay in the reused #countdown element — client theater only (never
 *  touches the sim, the input quantiser, or the lap log). Big countdown type
 *  by default; inline color/fontSize override per message, cleared on the next
 *  resetRun. hideAfterMs > 0 auto-hides. Unboxed, per APEX. */
function showOverlay(text, { color = "", fontSize = "", hideAfterMs = 0 } = {}) {
  const el = $("countdown");
  clearTimeout(countdownHideTimer);
  el.textContent = text;
  el.style.color = color;
  el.style.fontSize = fontSize;
  el.style.display = "block";
  if (hideAfterMs) countdownHideTimer = setTimeout(() => (el.style.display = "none"), hideAfterMs);
}

async function resetRun({ countdownMs: startCountdownMs = 400 } = {}) {
  started = true;
  sim.reset();
  prev = curr = sim.state();
  crumple.reset(); // restore the player car to pristine (undented)
  impactCooldown = 0;
  prevDamage = 0;  // sim_reset() zeroes kernel damage; rebaseline the delta trigger
  ticks = [];
  frozen = false;
  invalid = false;
  acc = 0;
  sectorTicks = [];
  prevCpMask = 0;
  deltaPtr = 0;
  $("delta").textContent = "";
  renderSectors([]);
  // Countdown length is client theater: the lap starts at tick 0 from rest
  // either way (no ticks are stepped or logged until it hits zero). Restarts
  // get one beat (400 ms → "1", GO); only the first run gets the full 3-2-1.
  countdownMs = startCountdownMs;
  countdownBeepN = 0; // re-arm the per-integer countdown beeps
  clearTimeout(countdownHideTimer);
  $("countdown").style.color = "";
  $("countdown").style.fontSize = "";

  rolloverTicks = 0;
  ghost = sessionGhost ?? loadGhost();
  ghostIdx = 0;
  if (ghost) {
    ghostSim ??= await Sim.instantiate().then((g) => (g.loadTrack(new Uint8Array(trackBuf)), g));
    ghostSim.reset();
    ghostPrev = ghostCurr = ghostSim.state();
    ghostCar.visible = true;
  } else {
    ghostCar.visible = false;
  }
  // Annotate what the hero delta compares against (ghost identity):
  // leaderboard ghost → "vs P1 name" (session-best reference, purple when
  // ahead); own PB ghost → "vs best lap" (green when ahead).
  $("deltaLabel").textContent = ghost
    ? (sessionGhostName ? `vs P${sessionGhostRank ?? "?"} ${sessionGhostName}` : "vs best lap")
    : "";
}

/** First-run start: leaves the armed idle, runs the full 3-2-1. The triggering
 *  gesture (keydown/pointerdown) also unlocks the AudioContext naturally. */
function startFirstRun() {
  if (started) return;
  audio.start();
  resetRun({ countdownMs: 3200 });
}

addEventListener("keydown", (e) => {
  if (e.target.closest?.("input,textarea")) return; // typing, not driving
  audio.start();
  // Wheel setup must be reachable BEFORE the first run (calibration happens
  // pre-drive), so I toggles the panel instead of arming the start.
  if (e.code === "KeyI") {
    $("config").style.display = $("config").style.display === "block" ? "none" : "block";
    return;
  }
  if (!started) { startFirstRun(); return; }
  if (e.code === "KeyR") resetRun();
  if (e.code === "KeyC") {
    cameraMode = CAMERA_SEATS[(CAMERA_SEATS.indexOf(cameraMode) + 1) % CAMERA_SEATS.length];
    if (cameraMode === "tv") tvIdx = -1; // re-pick the nearest tripod on entry
    setStatus(`camera: ${CAMERA_NAMES[cameraMode]}`);
  }
  if (e.code === "KeyM") setStatus(audio.toggleMute() ? "muted" : "sound on");
  if (e.code === "KeyL") setStatus(`lighting: ${lighting.cycle()}`);
});
addEventListener("pointerdown", (e) => {
  audio.start();
  // Taps on controls (name field, config panel, the whole leaderboard — its
  // rows are ghost-race affordances now) don't launch a run.
  if (!e.target.closest?.("input,textarea,button,select,label,.panel,#board")) startFirstRun();
});

// Calibration UI
for (const btn of document.querySelectorAll("[data-cal]")) {
  btn.addEventListener("click", () => {
    const ch = btn.dataset.cal;
    const action = ch === "steer" ? "turn the wheel fully LEFT" : `press/pull the ${ch} fully`;
    if (!input.startCalibration(ch, (msg) => ($("calMsg").textContent = msg))) {
      $("calMsg").textContent = "no devices visible — wiggle each one once (browser hides idle devices), then retry";
      return;
    }
    $("calMsg").textContent = `HOLD EVERYTHING STILL…`;
    setTimeout(() => {
      if (input.calibrating?.channel === ch) $("calMsg").textContent = `NOW ${action}!`;
    }, 750);
  });
}
$("closeConfig").addEventListener("click", () => ($("config").style.display = "none"));
$("clearCal").addEventListener("click", () => {
  input.clearCalibration();
  $("calMsg").textContent = "bindings cleared — detect each channel again";
});
// Steering feel: invert + sensitivity (higher = less physical rotation for
// full lock, for high-rotation-range DD bases).
$("steerInvert").checked = input.steerSettings.invert;
$("steerSens").value = input.steerSettings.sensitivity;
$("steerSensVal").textContent = `${input.steerSettings.sensitivity.toFixed(1)}×`;
$("steerInvert").addEventListener("change", () => input.setSteerSettings({ invert: $("steerInvert").checked }));
$("steerSens").addEventListener("input", () => {
  const v = Number($("steerSens").value);
  input.setSteerSettings({ sensitivity: v });
  $("steerSensVal").textContent = `${v.toFixed(1)}×`;
});

$("sendDiag").addEventListener("click", async () => {
  try {
    const payload = {
      ts: new Date().toISOString(),
      ua: navigator.userAgent,
      nativePads: [...(navigator.getGamepads?.() ?? [])]
        .filter((g) => g && g.connected)
        .map((g) => ({ id: g.id, mapping: g.mapping, axes: g.axes.map((a) => +a.toFixed(3)), buttons: g.buttons.length })),
      hid: hidDump(),
      bindings: input.cal,
      lastCalibration: input.lastCalDebug ?? null,
    };
    const r = await fetch("/api/diag", {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify(payload),
    });
    const { key } = await r.json();
    $("calMsg").textContent = `diagnostics sent (${key}) — Claude can read it now`;
  } catch (err) {
    $("calMsg").textContent = `diag send failed: ${err.message}`;
  }
});
$("addHid").addEventListener("click", async () => {
  try {
    const n = await addHidDevices();
    $("calMsg").textContent = n
      ? `${n} device(s) added via raw HID — now detect each channel`
      : "no device selected";
  } catch (err) {
    $("calMsg").textContent = `WebHID: ${err.message}`;
  }
});

// FFB UI (WebHID, experimental)
if (!FanatecFFB.supported()) $("ffbConnect").disabled = true;
$("ffbConnect").addEventListener("click", async () => {
  try {
    if (ffb) {
      await ffb.stop();
      ffb = null;
      $("ffbConnect").textContent = "connect wheel FFB";
      $("ffbMsg").textContent = "disconnected";
      return;
    }
    ffb = await FanatecFFB.connect();
    // Same open connection also feeds the input binder (raw HID axes).
    await registerHidDevice(ffb.device);
    $("ffbConnect").textContent = "disconnect FFB";
    $("ffbMsg").textContent = `connected: ${ffb.device.productName || "Fanatec base"} — now re-detect steering above (move the wheel)`;
  } catch (err) {
    $("ffbMsg").textContent = `FFB: ${err.message}`;
  }
});
$("ffbGain").addEventListener("input", () => { if (ffb) ffb.gain = Number($("ffbGain").value); });
$("ffbInvert").addEventListener("change", () => { if (ffb) ffb.invert = $("ffbInvert").checked; });

// ---------------------------------------------------------------- leaderboard
// Full fetched board kept in memory; rendered as top-5 plus a 3-row window
// around the player's own entry (real ranks). Quiet ranked list — the cyan
// accent stays reserved for the record holder's time; P2+ read as
// +gap-to-leader (exact time on hover).
let boardEntries = [];
let medalTargets = null; // { gold, silver, bronze } ms — client-derived from P1
let myKeyHex = null;     // this client's pubkey, lowercase hex (matches entry.pubkey)

// myPubkeyHex ships with lap.js; if an older copy lacks the export, derive the
// same identity locally (same localStorage seed, same vendored ed25519).
const myPubkeyHex = lapApi.myPubkeyHex ?? (async () => {
  const seedHex = localStorage.getItem("sttr-ed25519-seed");
  if (!seedHex) return null;
  const ed = await import("../vendor/ed25519.js");
  const pub = await ed.getPublicKeyAsync(new Uint8Array(seedHex.match(/../g).map((h) => parseInt(h, 16))));
  return [...pub].map((b) => b.toString(16).padStart(2, "0")).join("");
});

const sanitizeName = (s) => String(s).replace(/[<>&]/g, "");

/** Player's best lap in ms: this session's, else the stored PB ghost's. */
function playerBestMs() {
  if (bestMs !== null) return bestMs;
  try {
    const stored = localStorage.getItem(BEST_KEY);
    if (stored) return Math.round((JSON.parse(stored).lapTicks * 1000) / 400);
  } catch { /* corrupt PB blob — treat as no best */ }
  return null;
}

/** One board row (i = index into boardEntries; rank = i + 1). */
function boardRow(e, i, isYou) {
  const time = i === 0
    ? `<span class="t accent">${fmtMs(e.ms)}</span>`
    : `<span class="t" title="${fmtMs(e.ms)}">+${((e.ms - boardEntries[0].ms) / 1000).toFixed(3)}</span>`;
  return `<li data-i="${i}"${isYou ? ' class="you"' : ""}>` +
    `<span class="rank">${i + 1}</span>${time} ${sanitizeName(e.name)}` +
    `<button class="race" title="race this lap as a ghost">▶</button></li>`;
}

async function refreshBoard() {
  try {
    boardEntries = await fetchLeaderboard(); // keep everything the API returns
    if (myKeyHex === null) myKeyHex = await myPubkeyHex().catch(() => null);
    const myIdx = myKeyHex ? boardEntries.findIndex((e) => e.pubkey === myKeyHex) : -1;
    let rows;
    if (!boardEntries.length) {
      rows = [`<li class="sub">no laps yet — be first</li>`];
    } else {
      rows = boardEntries.slice(0, 5).map((e, i) => boardRow(e, i, i === myIdx));
      if (myIdx >= 5) {
        // 3-row window around YOU with real ranks; '···' marks a rank gap.
        const from = Math.max(5, myIdx - 1);
        const to = Math.min(boardEntries.length - 1, myIdx + 1);
        if (from > 5) rows.push(`<li class="sub">···</li>`);
        for (let i = from; i <= to; i++) rows.push(boardRow(boardEntries[i], i, i === myIdx));
      } else if (myIdx === -1) {
        // Not in the fetched set: pin an unranked YOU row with the local best.
        const mine = playerBestMs();
        rows.push(`<li class="you"><span class="rank">YOU · —</span><span class="t">${mine !== null ? fmtMs(mine) : "—"}</span></li>`);
      }
    }
    $("entries").innerHTML = rows.join("");
    const p1 = boardEntries[0]?.ms;
    medalTargets = p1
      ? { gold: Math.round(p1 * 1.02), silver: Math.round(p1 * 1.08), bronze: Math.round(p1 * 1.20) }
      : null;
    renderTarget();
  } catch {
    $("entries").innerHTML = `<li class="sub">leaderboard unavailable</li>`;
  }
}
// Whole row races that ghost (the ▶ button's click bubbles to the row too).
$("entries").addEventListener("click", (e) => {
  const li = e.target.closest("li[data-i]");
  if (li) raceReplay(boardEntries[Number(li.dataset.i)], Number(li.dataset.i) + 1);
});

// Sector small multiples: one thin bar per sector, width proportional to the
// sector's duration (best-known, so proportions are stable lap to lap), fill
// appearing as sectors complete, color = faster/slower than the stored best.
// Exact times live in each bar's title.
const sectorCount = track.checkpoints.length + 1;
const sectorEls = [];
{
  const wrap = $("sectors");
  for (let i = 0; i < sectorCount; i++) {
    const el = document.createElement("span");
    el.className = "sector";
    wrap.appendChild(el);
    sectorEls.push(el);
  }
}

const signed = (s) => (Math.round(s * 100) === 0 ? "0.00" : `${s > 0 ? "+" : "−"}${Math.abs(s).toFixed(2)}`);

function renderSectors(durations) {
  const best = JSON.parse(localStorage.getItem(SECT_KEY) ?? "[]");
  const known = sectorEls.map((_, i) => best[i] ?? durations[i] ?? null);
  const filled = known.filter((k) => k !== null);
  const avg = filled.length ? filled.reduce((a, b) => a + b, 0) / filled.length : 1;
  const widths = known.map((k) => k ?? avg);
  const total = widths.reduce((a, b) => a + b, 0);
  let note = "", noteCls = "";
  for (let i = 0; i < sectorCount; i++) {
    const el = sectorEls[i];
    el.style.flexGrow = (widths[i] / total).toFixed(4);
    if (i < durations.length) {
      const diff = best[i] ? (durations[i] - best[i]) / 400 : null;
      el.className = `sector ${diff === null ? "done" : diff <= 0 ? "faster" : "slower"}`;
      el.title = `S${i + 1} ${fmtMs(Math.round((durations[i] * 1000) / 400))}${diff !== null ? ` (${signed(diff)})` : ""}`;
      if (i === durations.length - 1 && diff !== null) {
        note = `S${i + 1} ${signed(diff)}`;
        noteCls = diff <= 0 ? "faster" : "slower";
      }
    } else {
      el.className = "sector";
      el.title = `S${i + 1}`;
    }
  }
  $("sectorNote").textContent = note;
  $("sectorNote").className = noteCls;
}

// ---------------------------------------------------- goals & board context
// Extra HUD lines are created here (not in index.html markup): all text-only
// and unboxed per APEX, inheriting the #lapInfo / #board type system.

// OPT — theoretical best: the sum of the stored sector bests, shown alongside
// BEST/LAST once a best exists for every sector.
const optWrap = document.createElement("span");
optWrap.style.display = "none";
optWrap.innerHTML = `<span class="lbl" style="margin-left:10px">OPT</span> <span class="val">—</span>`;
$("lapInfo").appendChild(optWrap);
const optVal = optWrap.querySelector(".val");
function updateOpt() {
  let best = [];
  try { best = JSON.parse(localStorage.getItem(SECT_KEY) ?? "[]"); } catch { /* corrupt — treat as none */ }
  const full = Array.isArray(best) && best.length === sectorCount && best.every((b) => Number.isFinite(b));
  if (full) optVal.textContent = fmtMs(Math.round((best.reduce((a, b) => a + b, 0) * 1000) / 400));
  optWrap.style.display = full ? "" : "none";
}

// Medal target line under BEST/LAST: the next unachieved tier vs the player's
// best. Targets are client-derived from P1 (gold 102% · silver 108% ·
// bronze 120%, ms-rounded in refreshBoard).
const targetEl = document.createElement("div");
$("lapInfo").appendChild(targetEl);
const fmtTenths = (ms) => {
  const t = Math.round(ms / 100); // tenths, rounded before splitting
  const m = Math.floor(t / 600), s = (t % 600) / 10;
  return `${m}:${s < 10 ? "0" : ""}${s.toFixed(1)}`;
};
/** Tier a lap of `ms` earns against the current targets, or null. */
function earnedTier(ms) {
  if (!medalTargets || ms === null) return null;
  if (ms <= medalTargets.gold) return "GOLD";
  if (ms <= medalTargets.silver) return "SILVER";
  if (ms <= medalTargets.bronze) return "BRONZE";
  return null;
}
function renderTarget() {
  if (!medalTargets) {
    targetEl.innerHTML = `<span class="lbl">TARGET</span> set the first time`;
    return;
  }
  const best = playerBestMs();
  const next = [["BRONZE", medalTargets.bronze], ["SILVER", medalTargets.silver], ["GOLD", medalTargets.gold]]
    .find(([, ms]) => best === null || best > ms);
  if (!next) { // gold already achieved — nothing left to chase but P1
    targetEl.innerHTML = `<span class="lbl">TARGET · GOLD</span> <span class="val">${fmtTenths(medalTargets.gold)}</span> <span class="lbl">✓</span>`;
    return;
  }
  const gap = best === null ? "" : ` <span class="lbl">(−${((best - next[1]) / 1000).toFixed(1)}s)</span>`;
  targetEl.innerHTML = `<span class="lbl">TARGET · ${next[0]}</span> <span class="val">${fmtTenths(next[1])}</span>${gap}`;
}

// Weekly reset countdown under the board caption. One 60 s interval (created
// once at boot, page-lifetime — no leaks) refreshes the label AND re-polls
// the leaderboard.
const resetEl = document.createElement("div");
resetEl.className = "reset";
document.querySelector("#board .caps").insertAdjacentElement("afterend", resetEl);
function nextMondayUtc(now = new Date()) {
  const d = new Date(Date.UTC(now.getUTCFullYear(), now.getUTCMonth(), now.getUTCDate()));
  d.setUTCDate(d.getUTCDate() + ((8 - d.getUTCDay()) % 7 || 7));
  return d;
}
function updateResetLabel() {
  const left = Math.max(0, nextMondayUtc() - Date.now());
  const dd = Math.floor(left / 86400000);
  const hh = Math.floor((left % 86400000) / 3600000);
  const mm = Math.floor((left % 3600000) / 60000);
  resetEl.textContent = `resets in ${dd > 0 ? `${dd}d ${hh}h` : hh > 0 ? `${hh}h ${mm}m` : `${Math.max(mm, 1)}m`}`;
}
updateResetLabel();
updateOpt();
refreshBoard();
setInterval(() => { updateResetLabel(); refreshBoard(); }, 60_000);

async function onLapComplete() {
  frozen = true;
  if (ffb) ffb.update(0, 0.1);
  const lapTicks = sim.lapTimeTicks();
  lastMs = Math.round((lapTicks * 1000) / 400);
  const isPB = bestMs === null || lastMs < bestMs;
  bestMs = isPB ? lastMs : bestMs;
  // PB fanfare: 3-note ascending arpeggio. Medal tiers get no extra sound —
  // the status line announcement below covers them.
  if (isPB) {
    audio.beep(660, 90);
    setTimeout(() => audio.beep(880, 90), 90);
    setTimeout(() => audio.beep(1100, 90), 180);
  }
  // Client-derived medal for this lap (vs targets from P1), announced below.
  const tier = earnedTier(lastMs);
  const tierTxt = tier ? ` — ${tier} medal` : "";

  // Sector bests + ghost are local-first: saved regardless of server verdict.
  const boundaries = [...sectorTicks, lapTicks];
  const sectors = boundaries.map((t, i) => t - (boundaries[i - 1] ?? 0));
  renderSectors(sectors); // final sector bar, colored against the old bests
  const prevBest = JSON.parse(localStorage.getItem(SECT_KEY) ?? "[]");
  localStorage.setItem(SECT_KEY, JSON.stringify(sectors.map((s, i) => Math.min(s, prevBest[i] ?? Infinity))));
  updateOpt(); // sector bests may have changed → refresh the theoretical best
  if (isPB) await saveGhost(ticks, lapTicks);

  const log = buildLapLog(track.bytes, ticks.slice(0, lapTicks), sim.stateHash(), lapTicks);
  setStatus(`lap ${fmtMs(lastMs)}${tierTxt} — submitting for edge validation…`);
  try {
    const result = await submitLap(log, $("name").value || "anon", (stage) => setStatus(`lap ${fmtMs(lastMs)} — referee: ${stage}…`));
    if (result.status === "accepted") {
      setStatus(`ACCEPTED ✔ ${fmtMs(result.lapTimeMs)}${tierTxt} — world rank #${result.rank}. Press R to go again.`, "ok");
    } else if (result.status === "pending") {
      setStatus(`lap ${fmtMs(lastMs)}${tierTxt} queued — validated within ~30 min (free-tier mode). Press R.`, "info");
    } else {
      setStatus(`rejected: ${result.reason}${result.detail ? ` (${result.detail})` : ""} — press R`, "bad");
    }
  } catch (err) {
    setStatus(`submit failed: ${err.message} — press R`, "bad");
  }
  refreshBoard();
}

// ---------------------------------------------------------------- main loop
const tmpQa = new THREE.Quaternion(), tmpQb = new THREE.Quaternion();
const camTarget = new THREE.Vector3(), camPos = new THREE.Vector3(0, 6, -12);
const _camA = new THREE.Vector3(), _camB = new THREE.Vector3(); // per-frame camera scratch
let fov = 62; // per-seat FOV, smoothed toward its target each frame

// TV seat: ~8 fixed trackside tripods — one every S/8 centerline samples,
// 14 m outside the loop (away from the track centroid), 4 m up. The camera
// stands still on the nearest tripod, tracks the car with lookAt, and hands
// off to the next tripod once the car passes its cross-section — classic
// broadcast coverage. Built once from track data already in memory.
const tvPoints = (() => {
  const S = track.S, n = 8;
  let cx = 0, cz = 0;
  for (const c of track.center) { cx += c[0]; cz += c[2]; }
  cx /= S; cz /= S;
  const pts = [];
  for (let k = 0; k < n; k++) {
    const s = Math.floor((k * S) / n);
    const c = track.center[s], t = track.tangent[s];
    let lx = t[2], lz = -t[0]; // horizontal perpendicular to the travel direction
    const ll = Math.hypot(lx, lz) || 1;
    lx /= ll; lz /= ll;
    const side = (c[0] - cx) * lx + (c[2] - cz) * lz >= 0 ? 1 : -1; // outside = away from centroid
    pts.push({
      pos: new THREE.Vector3(c[0] + 14 * side * lx, c[1] + 4, c[2] + 14 * side * lz),
      gate: new THREE.Vector3(c[0], c[1], c[2]), // the tripod's track cross-section
      dir: new THREE.Vector3(t[0], t[1], t[2]).normalize(), // travel direction there
    });
  }
  return pts;
})();
let tvIdx = -1; // active tripod; -1 → pick the nearest on the next TV frame

// Crash-deformation trigger (render-only). Two sources, preferring the
// authoritative one:
//
//   • ABI 1.3+ binaries export sim_damage(0) — the kernel's DETERMINISTIC
//     overall crash-damage scalar, the exact value the edge referee recomputes
//     from the input log. We drive the visible crumple SEVERITY from its
//     per-tick increase, so the dent the player sees matches the physics damage
//     that will be scored. Direction still comes from the chassis Δvelocity
//     (which face was struck — purely cosmetic).
//   • Older binaries lack the export → fall back to the self-contained Phase-1
//     Δvelocity trigger below.
//
// Either way this is render-only: it never feeds back into the sim, the input
// quantiser, or the lap log, so determinism/replay are untouched.
const IMPACT_DV = 0.7;         // m/s of horizontal Δv in one tick to count as a hit (fallback)
const IMPACT_SEV_SCALE = 2.4;  // Δv (above threshold) that maps to severity 1 (fallback)
const IMPACT_COOLDOWN = 48;    // ticks to wait before another auto-impact (fallback)
const DAMAGE_SEV_SCALE = 4.0;  // kernel overall-damage Δ (per tick) → crumple severity
const DAMAGE_DELTA_MIN = 1e-4; // ignore float noise; a clean lap reads exactly 0 damage
let impactCooldown = 0;
let prevDamage = 0;            // previous tick's sim_damage(0), for the delta trigger
const _dv = new THREE.Vector3();
const _iq = new THREE.Quaternion();

/** World Δv (prev→curr chassis velocity, horizontal) → normalised car-local
 *  crush direction. Returns the magnitude of the world Δv. */
function localCrushDir(prevS, currS) {
  _dv.set(currS.linVel[0] - prevS.linVel[0], 0, currS.linVel[2] - prevS.linVel[2]);
  const mag = _dv.length();
  if (mag > 1e-4) {
    _iq.set(currS.quat[0], currS.quat[1], currS.quat[2], currS.quat[3]).invert();
    _dv.applyQuaternion(_iq).normalize();
  }
  return mag;
}

/** Dent the player car on impact. Cosmetic only — never feeds back into the sim. */
function detectImpact(prevS, currS) {
  const dmg = sim.damage(0); // authoritative overall damage, or null on old binaries

  if (dmg !== null) {
    // --- authoritative path: severity tracks the deterministic damage delta ---
    const delta = dmg - prevDamage;
    prevDamage = dmg;
    if (delta <= DAMAGE_DELTA_MIN) return; // no new damage this tick
    const severity = Math.min(1.5, delta * DAMAGE_SEV_SCALE);
    if (localCrushDir(prevS, currS) <= 1e-4) {
      // Damage rose without a velocity swing (rare): infer the struck axle from
      // whichever toe component took the hit — nose (−Z) vs tail (+Z).
      const front = sim.damage(2) ?? 0, rear = sim.damage(3) ?? 0;
      _dv.set(0, 0, front >= rear ? -1 : 1);
    }
    crumple.applyImpact(_dv, severity);
    audio.impact(severity); // crunch + body thump scaled to the same severity
    return;
  }

  // --- fallback path (pre-1.3 binaries): Phase-1 Δvelocity trigger ---
  if (impactCooldown > 0) { impactCooldown--; return; }
  const dv = localCrushDir(prevS, currS);
  if (dv < IMPACT_DV) return;
  const severity = Math.min(1.5, (dv - IMPACT_DV) / IMPACT_SEV_SCALE + 0.25);
  crumple.applyImpact(_dv, severity);
  audio.impact(severity); // crunch + body thump scaled to the same severity
  impactCooldown = IMPACT_COOLDOWN;
}

/** Step physics by dtMs of wall time. Extracted from the rAF handler so tests
 *  (and the debug hook below) can pump the sim when rAF is throttled. */
function advance(dtMs) {
  if (!started) return; // armed idle — waiting for the first gesture
  if (countdownMs > 0) {
    // Skip-on-input: an eager driver cuts the theater short. Pure client-side
    // presentation — the lap still starts at tick 0 from rest.
    const raw = inputOverride ? inputOverride(curr) : input.sample(dtMs / 1000);
    if (raw.throttle > 0.5 || Math.abs(raw.steer) > 0.5) countdownMs = Math.min(countdownMs, 1);
    countdownMs -= dtMs;
    const n = Math.ceil(countdownMs / 1000);
    $("countdown").textContent = countdownMs <= 0 ? "GO" : String(n);
    $("countdown").style.display = "block";
    // Countdown beeps track the DISPLAYED number: one short blip per integer
    // (fast 400 ms restarts get exactly one "1" blip), long high blip on GO.
    if (countdownMs > 0 && n !== countdownBeepN) {
      countdownBeepN = n;
      audio.beep(660, 80);
    }
    if (countdownMs <= 0) {
      audio.beep(880, 300);
      setStatus("GO — set a time; it will be validated at the edge", "ok");
      clearTimeout(countdownHideTimer);
      countdownHideTimer = setTimeout(() => ($("countdown").style.display = "none"), 700);
    }
    return;
  }
  if (frozen) return;
  acc += dtMs;
  let substeps = 0;
  while (acc >= DT_MS) {
    // Cap physics work per frame (background-tab / long-stall spiral guard):
    // sim time falls behind wall clock smoothly instead of freezing the frame.
    // Every tick that DOES execute below is stepped and logged exactly as
    // always — no protocol or determinism impact.
    if (++substeps > 32) { acc = 0; break; }
    acc -= DT_MS;
    const raw = inputOverride ? inputOverride(curr) : input.sample(DT_MS / 1000);
    lastThrottle = raw.throttle;
    const q = quantize(raw);
    const status = sim.step(q.steer, q.throttle, q.brake, q.flags);
    ticks.push(q);
    prev = curr;
    curr = sim.state();
    detectImpact(prev, curr); // cosmetic crash denting; no sim coupling

    // Ghost runs one recorded tick per live tick — true side-by-side racing.
    if (ghost && ghostIdx < ghost.lapTicks) {
      const o = 8 * ghostIdx;
      ghostSim.step(ghost.ticks.getInt16(o, true), ghost.ticks.getUint16(o + 2, true), ghost.ticks.getUint16(o + 4, true), ghost.ticks.getUint16(o + 6, true));
      ghostPrev = ghostCurr;
      ghostCurr = ghostSim.state();
      ghostIdx++;
    }

    // Sector boundaries: new checkpoint bits appearing.
    if (curr.checkpoints !== prevCpMask) {
      sectorTicks.push(ticks.length);
      prevCpMask = curr.checkpoints;
      const durations = sectorTicks.map((t, i) => t - (sectorTicks[i - 1] ?? 0));
      renderSectors(durations);
      // Two-tone sector chime, matching the bar colors: higher pair when at or
      // under the stored best (or no best yet — first completion reads as a
      // win), lower pair when over it. Same localStorage bests renderSectors
      // reads; no new sim state touched.
      const si = durations.length - 1;
      let bestSect = [];
      try { bestSect = JSON.parse(localStorage.getItem(SECT_KEY) ?? "[]"); } catch { /* corrupt — treat as none */ }
      const faster = !(bestSect[si] > 0) || durations[si] - bestSect[si] <= 0;
      const [c1, c2] = faster ? [880, 1320] : [440, 330];
      audio.beep(c1, 90, "sine", 0.12);
      setTimeout(() => audio.beep(c2, 90, "sine", 0.12), 90);
    }

    if (status & STATUS.LAP_INVALID && !invalid) {
      invalid = true;
      audio.beep(220, 250, "sawtooth"); // low buzz — the "no" sound
      setStatus("lap invalidated (corner cut) — press R to restart", "bad");
      showOverlay("LAP INVALID — R to restart", { color: "var(--bad)", fontSize: "42px", hideAfterMs: 1500 });
    }
    if (status & STATUS.LAP_COMPLETE) {
      if (!invalid) onLapComplete();
      else {
        frozen = true;
        // Crossed the line on an invalidated lap: say so, persistently.
        setStatus("lap invalid (corner cut) — not submitted; press R to restart", "bad");
        showOverlay("LAP INVALID — R to restart", { color: "var(--bad)", fontSize: "42px" });
      }
      break;
    }
    if (ticks.length >= 72000) {
      frozen = true;
      setStatus("3-minute limit reached — press R", "bad");
      break;
    }
    if (curr.pos[1] < fellOffY) {
      resetRun();
      setStatus("off into the void — respawned at the start line", "bad");
      break;
    }
    // Rollover recovery: on its side/roof and stationary → respawn.
    const [qx, , qz] = [curr.quat[0], curr.quat[1], curr.quat[2]];
    const upY = 1 - 2 * (qx * qx + qz * qz);
    rolloverTicks = upY < 0.35 && curr.speed < 1.0 ? rolloverTicks + 1 : 0;
    if (rolloverTicks > 800) { // 2 s
      resetRun();
      setStatus("rolled over — respawned at the start line", "bad");
      break;
    }
  }
}

function poseGroup(group, prevS, currS, alpha) {
  const lerp = (a, b) => a + (b - a) * alpha;
  group.position.set(lerp(prevS.pos[0], currS.pos[0]), lerp(prevS.pos[1], currS.pos[1]), lerp(prevS.pos[2], currS.pos[2]));
  tmpQa.set(...prevS.quat);
  tmpQb.set(...currS.quat);
  group.quaternion.slerpQuaternions(tmpQa, tmpQb, alpha);
}

function frame(now) {
  requestAnimationFrame(frame);
  const dtMs = Math.min(now - last, 250); // anti-spiral clamp
  last = now;
  input.tickCalibration();
  if (!started) {
    // Armed idle: gamepad/wheel activity also starts the first run (keydown
    // and pointerdown are handled by their listeners above).
    const raw = input.sample(dtMs / 1000);
    if (raw.throttle > 0.5 || raw.brake > 0.5 || Math.abs(raw.steer) > 0.5 || raw.handbrake) startFirstRun();
  }
  advance(dtMs);

  const alpha = Math.min(acc / DT_MS, 1);
  const lerp = (a, b) => a + (b - a) * alpha;
  poseGroup(car, prev, curr, alpha);
  if (ghostCar.visible && ghostCurr) poseGroup(ghostCar, ghostPrev, ghostCurr, alpha);

  for (let i = 0; i < 4; i++) {
    const wp = prev.wheels[i], wc = curr.wheels[i];
    wheelMeshes[i].position.set(lerp(wp.pos[0], wc.pos[0]), lerp(wp.pos[1], wc.pos[1]), lerp(wp.pos[2], wc.pos[2]));
    wheelMeshes[i].quaternion.copy(car.quaternion);
    wheelMeshes[i].rotateY(lerp(wp.steer, wc.steer));
    wheelMeshes[i].rotateX(lerp(wp.spin, wc.spin));
  }

  // Camera seats (C cycles) — all posed from the interpolated render state
  // (car group position/quaternion), no extra physics reads.
  if (cameraMode === "chase") {
    // Spring-damped chase (the default).
    const behind = _camA.set(0, 3.2, -8.5).applyQuaternion(car.quaternion).add(car.position);
    camPos.lerp(behind, 1 - Math.exp(-4 * (dtMs / 1000)));
    camera.position.copy(camPos);
    camTarget.lerp(car.position, 0.6);
    camera.lookAt(camTarget.x, camTarget.y + 1.2, camTarget.z);
  } else if (cameraMode === "cockpit") {
    // Driver eye: looks forward through the halo, nose visible at frame bottom.
    const eye = _camA.set(0, 0.95, 0.25).applyQuaternion(car.quaternion).add(car.position);
    const ahead = _camB.set(0, 1.0, 30).applyQuaternion(car.quaternion).add(car.position);
    camera.position.copy(eye);
    camera.lookAt(ahead);
    camPos.copy(eye);
  } else if (cameraMode === "bumper") {
    // Nose level, no car in frame — the road rush sells speed.
    const nose = _camA.set(0, 0.35, 2.3).applyQuaternion(car.quaternion).add(car.position);
    const ahead = _camB.set(0, 0.45, 32).applyQuaternion(car.quaternion).add(car.position);
    camera.position.copy(nose);
    camera.lookAt(ahead);
    camPos.copy(nose);
  } else {
    // TV: stand still on a trackside tripod and pan with the car; hand off to
    // the next tripod once the car passes this one's track cross-section.
    if (tvIdx < 0) {
      let best = 0, bestD = Infinity;
      for (let i = 0; i < tvPoints.length; i++) {
        const d2 = tvPoints[i].pos.distanceToSquared(car.position);
        if (d2 < bestD) { bestD = d2; best = i; }
      }
      tvIdx = best;
    }
    const pt = tvPoints[tvIdx];
    if (_camA.copy(car.position).sub(pt.gate).dot(pt.dir) > 6) tvIdx = (tvIdx + 1) % tvPoints.length;
    camera.position.copy(tvPoints[tvIdx].pos);
    camera.lookAt(car.position);
    camPos.copy(camera.position);
  }

  // Per-seat lens: chase & bumper keep the speed-breathing FOV (62° at rest
  // widening to 68° flat-out, ~200 km/h) so it reads as pace, never as a zoom
  // cut; cockpit is a fixed wide 70°; TV is a 45° broadcast lens that squeezes
  // toward 30° as the car runs away (slight tripod zoom). All exponentially
  // smoothed; one uniform + matrix update, render cost ~zero.
  const fovTarget = cameraMode === "cockpit"
    ? 70
    : cameraMode === "tv"
      ? 45 - 15 * Math.min(1, Math.max(0, (camera.position.distanceTo(car.position) - 30) / 90))
      : 62 + 6 * Math.min(1, curr.speed / 55);
  fov += (fovTarget - fov) * (1 - Math.exp(-2.5 * (dtMs / 1000)));
  if (Math.abs(camera.fov - fov) > 0.005) {
    camera.fov = fov;
    camera.updateProjectionMatrix();
  }

  crumple.update();  // settle crash dents (no-op when the car is undamaged)
  lighting.update(); // condition blend + sky dome / shadow frustum follow the car
  renderer.render(scene, camera);

  // ---- side channels: audio + FFB (read-only on sim state)
  // idle = post-lap freeze, countdown, or armed idle before the first gesture:
  // engine settles to tickover instead of droning at its last pitch.
  audio.update(curr, lastThrottle, sim, dtMs, frozen || countdownMs > 0 || !started);
  if (ffb) {
    const ffbFade = Math.min(1, Math.max(0, (curr.speed - 1.5) / 3)); // standstill fade
    ffb.update(frozen || countdownMs > 0 ? 0 : sim.ffbTorque() * ffbFade, dtMs / 1000);
    $("mFfb").value = ffb.smoothed / ffb.maxNm;
  }

  // ---- HUD
  const runningTicks = frozen ? sim.lapTimeTicks() || ticks.length : ticks.length;
  $("lapTime").textContent = fmtMs(Math.round((runningTicks * 1000) / 400));
  $("speed").textContent = Math.round(curr.speed * 3.6);
  $("bestVal").textContent = bestMs ? fmtMs(bestMs) : "—";
  $("lastVal").textContent = lastMs ? fmtMs(lastMs) : "—";
  minimap.draw(car.position.toArray(), curr.checkpoints);

  // Drivetrain readouts (ABI 1.4, DRIVETRAIN.md §6): gear numeral ('R' for
  // reverse) beside the speed, thin RPM bar with its redline tick, cyan
  // shift-light pulse ≥ 7200 rpm, and the quiet AUTO/MANUAL tag. sim.rpm()
  // is null on pre-1.4 binaries — everything stays hidden.
  const rpmNow = sim.rpm();
  if (rpmNow !== null) {
    if (!drivetrainHudOn) {
      drivetrainHudOn = true;
      gearEl.style.display = "inline";
      shiftModeEl.style.display = "inline";
      rpmBarEl.style.display = "block";
    }
    const g = sim.gear();
    const gearTxt = g === 0 ? "R" : String(g ?? 1);
    if (gearEl.textContent !== gearTxt) gearEl.textContent = gearTxt;
    rpmFillEl.style.width = `${((Math.min(rpmNow, RPM_BAR_MAX) / RPM_BAR_MAX) * 100).toFixed(1)}%`;
    rpmBarEl.classList.toggle("shift", rpmNow >= SHIFT_LIGHT_RPM);
    const mode = shiftModeLabel();
    if (shiftModeEl.textContent !== mode) shiftModeEl.textContent = mode;
  }

  // Tire surface temps (brush-model thermal layer) drive the wheel fills of
  // the car-schematic glyph: cold blue → in-window green → overheated red.
  // Order FL FR RL RR; exact °C on hover.
  if (sim.tireTemp(0) !== null) {
    if ($("tireTemps").style.display !== "block") $("tireTemps").style.display = "block";
    let tip = "";
    for (let i = 0; i < 4; i++) {
      const t = sim.tireTemp(i);
      tireEls[i].style.background = tireColor(t);
      tip += `${TIRE_LABELS[i]} ${Math.round(t)}°C  `;
    }
    $("tireTemps").title = tip.trimEnd();
  }

  // Crash damage (SOFTBODY.md Phase 2, ABI 1.3). Deterministic overall-damage
  // scalar from the kernel — the same value the referee scores. Quiet while
  // pristine; only appears past 1% and tints toward --bad as the car limps.
  const dmg = sim.damage(0);
  if (dmg !== null) {
    const el = $("damage");
    if (dmg > 0.01) {
      el.style.display = "block";
      el.textContent = `DMG ${Math.round(dmg * 100)}%`;
      // muted → --bad, crossing over as damage passes ~half (car handling hurt).
      el.style.color = dmg > 0.5 ? "var(--bad)" : "var(--muted)";
      el.title = `steer −${Math.round((sim.damage(1) ?? 0) * 100)}%`;
    } else if (el.style.display !== "none") {
      el.style.display = "none";
    }
  }

  // Live delta vs ghost: compare tick counts at equal lap progress. This is
  // the hero readout — big, centered, timing-tower convention: PURPLE when
  // ahead of the session-best reference (a downloaded leaderboard ghost),
  // GREEN when ahead of your own PB ghost, YELLOW when behind either.
  if (ghost && !frozen && countdownMs <= 0 && curr.lapProgress > 0.01) {
    while (deltaPtr < ghost.lapTicks - 1 && ghost.progress[deltaPtr] < curr.lapProgress) deltaPtr++;
    const d = (ticks.length - deltaPtr) / 400;
    $("delta").textContent = `${d >= 0 ? "+" : "−"}${Math.abs(d).toFixed(2)}`;
    $("delta").className = d >= 0 ? "behind" : ghost === sessionGhost ? "ahead-session" : "ahead-pb";
  }

  if ($("config").style.display === "block") {
    const raw = input.sample(dtMs / 1000);
    $("mSteer").value = raw.steer;
    $("mThrottle").value = raw.throttle;
    $("mBrake").value = raw.brake;
    $("mHandbrake").value = raw.handbrake ? 1 : 0;
    const pads = input.gamepads();
    $("gpName").textContent = pads.length
      ? pads.map((p) => p.id.split("(")[0].trim().slice(0, 34)).join("  ·  ") + `  —  ${raw.device}`
      : "none detected — wiggle each device once";
    const diag = hidDiagnostics();
    if (diag) $("hidDiag").textContent = diag + " — move a control; its report count should climb";
    $("bindings").textContent = "bound → " + input.bindingSummary();
  }
}

// Armed idle: no auto-countdown at boot. The first gesture (keydown /
// pointerdown / gamepad input) starts the full 3-2-1 — and doubles as the
// AudioContext unlock. Text-only prompt in the reused #countdown element.
showOverlay(
  "press any key to start · W/↑ throttle · A/D steer · S/↓ brake · Q/E gears · G auto · full keys below",
  { fontSize: "20px" },
);
setStatus(
  matchMedia("(pointer: coarse)").matches
    ? "built for keyboard, gamepad or wheel — best on a computer"
    : "ready",
);
requestAnimationFrame(frame);

// Debug/test hook: lets automated checks pump physics when the tab is
// occluded (browsers throttle rAF to ~1 fps for background tabs, which for
// players just pauses the run — the accumulator clamp prevents catch-up).
window.__sttr = {
  advance,
  renderOnce: () => frame(performance.now()),
  skipCountdown: () => (countdownMs = 1),
  info: () => ({ ticks: ticks.length, frozen, invalid, started, countdownMs, speed: curr.speed, pos: curr.pos, quat: curr.quat, checkpoints: curr.checkpoints, lapProgress: curr.lapProgress, ghost: ghostCar.visible, camera: cameraMode, ffbNm: sim.ffbTorque(), rpm: sim.rpm(), gear: sim.gear(), shiftMode: shiftModeLabel() }),
  track: { center: track.center, tangent: track.tangent },
  reset: resetRun,
  setInputOverride: (fn) => (inputOverride = fn),
  // Crash deformation (SOFTBODY.md Phase 1). Phase B can drive dents straight
  // from the kernel's deterministic damage signal via applyImpact(localDir,
  // severity): localDir = inward crush direction in car-local space (nose hit →
  // {x:0,y:0,z:-1}); severity ≈ 0..1.5. resetCrumple restores pristine.
  applyImpact: (localDir, severity) => crumple.applyImpact(localDir, severity),
  resetCrumple: () => crumple.reset(),
  input,
  scene,
  FanatecFFB,
  registerHidDevice,
};

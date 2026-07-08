// Serverless Time Trial — browser client. Same wasm physics, same referee,
// no install. Fixed 400 Hz accumulator + interpolated rendering (uncapped rAF).
import * as THREE from "../vendor/three.module.js";
import { Sim, STATUS, DT_MS } from "./sim.js";
import { parseTrack, buildTrackMeshes, Minimap, fnv1a64 } from "./track.js";
import { Input, quantize } from "./input.js";
import { buildLapLog, submitLap, fetchLeaderboard, fetchReplay, fmtMs } from "./lap.js";
import { EngineAudio } from "./audio.js";
import { buildCar, buildWheel } from "./car.js";
import { buildAmbience } from "./ambience.js";
import { Lighting } from "./lighting.js";
import { FanatecFFB } from "./ffb.js";
import { addHidDevices, autoReconnectHid, hidDiagnostics, hidDump, hidVirtualPads, registerHidDevice } from "./hid-input.js";

const $ = (id) => document.getElementById(id);
const setStatus = (text, cls = "info") => { $("statusText").textContent = text; $("statusText").className = cls; };

// ---------------------------------------------------------------- bootstrap
setStatus("loading physics + track…");
const [sim, trackResp] = await Promise.all([Sim.load(), fetch("/api/track/current")]);
const trackBuf = await trackResp.arrayBuffer();
const track = parseTrack(trackBuf);
sim.loadTrack(new Uint8Array(trackBuf));
const trackHash = fnv1a64(track.bytes);
const trackHashHex = trackHash.toString(16).padStart(16, "0");

$("name").value = localStorage.getItem("sttr-name") ?? "";
$("name").addEventListener("change", () => localStorage.setItem("sttr-name", $("name").value));

// ---------------------------------------------------------------- three.js
const renderer = new THREE.WebGLRenderer({ canvas: $("gl"), antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1420);
scene.fog = new THREE.Fog(0x0e1420, 150, 480);
const camera = new THREE.PerspectiveCamera(62, 1, 0.1, 800);

scene.add(buildTrackMeshes(track));
// Trackside dressing — trees, tire barriers, gantry, brake boards — all
// seeded from the track hash so placement is deterministic per track.
const ambience = buildAmbience(track, trackHash);
scene.add(ambience);

const car = buildCar(0xd9483b);
scene.add(car);
const ghostCar = buildCar(0xbfd9ff, 0.35, true); // translucent, with static wheels
ghostCar.visible = false;
scene.add(ghostCar);

// Time-of-day lighting (cosmetic; local clock; L cycles overrides). Owns the
// hemisphere + sun lights and the car's night headlights.
const lighting = new Lighting(scene, car, ambience.userData.glowMaterials);

const wheelMeshes = [];
for (let i = 0; i < 4; i++) {
  const holder = buildWheel(); // racing slick + rim, posed from physics below
  scene.add(holder);
  wheelMeshes.push(holder);
}

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
async function raceReplay(entry) {
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
    await resetRun();
    setStatus(`racing ${entry.name}'s ${fmtMs(entry.ms)} lap — beat the ghost! (R restarts, ghost stays)`, "ok");
  } catch (err) {
    setStatus(`replay: ${err.message}`, "bad");
  }
}

// ---------------------------------------------------------------- game state
const input = new Input();
input.setVirtualPads(hidVirtualPads);
// Reopen previously-granted WebHID input devices (permission persists).
autoReconnectHid().then((n) => {
  if (n) $("calMsg").textContent = `${n} WebHID device(s) reconnected — bindings ready`;
});
const minimap = new Minimap($("minimap"), track);
const audio = new EngineAudio();
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
let cameraMode = "chase";
let ffb = null;
let lastThrottle = 0;
let sectorTicks = []; // ticks at S1/S2/S3 boundaries this run
let prevCpMask = 0;
let rolloverTicks = 0;
let sessionGhost = null; // downloaded leaderboard lap (replay viewer)
let sessionGhostName = null;

async function resetRun() {
  sim.reset();
  prev = curr = sim.state();
  ticks = [];
  frozen = false;
  invalid = false;
  acc = 0;
  sectorTicks = [];
  prevCpMask = 0;
  deltaPtr = 0;
  $("delta").textContent = "";
  $("sectors").textContent = "";
  countdownMs = 3200;

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
}

addEventListener("keydown", (e) => {
  audio.start();
  if (e.code === "KeyR") resetRun();
  if (e.code === "KeyC") cameraMode = cameraMode === "chase" ? "hood" : "chase";
  if (e.code === "KeyM") setStatus(audio.toggleMute() ? "muted" : "sound on");
  if (e.code === "KeyL") setStatus(`lighting: ${lighting.cycle()}`);
  if (e.code === "KeyI") $("config").style.display = $("config").style.display === "block" ? "none" : "block";
});
addEventListener("pointerdown", () => audio.start());

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

let boardEntries = [];
async function refreshBoard() {
  try {
    boardEntries = (await fetchLeaderboard()).slice(0, 10);
    $("entries").innerHTML = boardEntries.length
      ? boardEntries
          .map(
            (e, i) =>
              `<li><b>${fmtMs(e.ms)}</b> ${e.name.replace(/[<>&]/g, "")} ` +
              `<button class="race" data-i="${i}" title="race this lap as a ghost">▶</button></li>`,
          )
          .join("")
      : `<li class="sub">no laps yet — be first</li>`;
  } catch {
    $("entries").innerHTML = `<li class="sub">leaderboard unavailable</li>`;
  }
}
$("entries").addEventListener("click", (e) => {
  const btn = e.target.closest("button.race");
  if (btn) raceReplay(boardEntries[Number(btn.dataset.i)]);
});
refreshBoard();

function fmtSectors(t) {
  const best = JSON.parse(localStorage.getItem(SECT_KEY) ?? "[]");
  return t
    .map((ticks, i) => {
      const ms = Math.round((ticks * 1000) / 400);
      const d = best[i] ? ((ticks - best[i]) / 400).toFixed(2) : null;
      return `S${i + 1} ${fmtMs(ms)}${d !== null ? ` (${d > 0 ? "+" : ""}${d})` : ""}`;
    })
    .join("  ");
}

async function onLapComplete() {
  frozen = true;
  if (ffb) ffb.update(0, 0.1);
  const lapTicks = sim.lapTimeTicks();
  lastMs = Math.round((lapTicks * 1000) / 400);
  const isPB = bestMs === null || lastMs < bestMs;
  bestMs = isPB ? lastMs : bestMs;

  // Sector bests + ghost are local-first: saved regardless of server verdict.
  const boundaries = [...sectorTicks, lapTicks];
  const sectors = boundaries.map((t, i) => t - (boundaries[i - 1] ?? 0));
  const prevBest = JSON.parse(localStorage.getItem(SECT_KEY) ?? "[]");
  localStorage.setItem(SECT_KEY, JSON.stringify(sectors.map((s, i) => Math.min(s, prevBest[i] ?? Infinity))));
  if (isPB) await saveGhost(ticks, lapTicks);

  const log = buildLapLog(track.bytes, ticks.slice(0, lapTicks), sim.stateHash(), lapTicks);
  setStatus(`lap ${fmtMs(lastMs)} — submitting for edge validation…`);
  try {
    const result = await submitLap(log, $("name").value || "anon", (stage) => setStatus(`lap ${fmtMs(lastMs)} — referee: ${stage}…`));
    if (result.status === "accepted") {
      setStatus(`ACCEPTED ✔ ${fmtMs(result.lapTimeMs)} — world rank #${result.rank}. Press R to go again.`, "ok");
    } else if (result.status === "pending") {
      setStatus(`lap ${fmtMs(lastMs)} queued — validated within ~30 min (free-tier mode). Press R.`, "info");
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

/** Step physics by dtMs of wall time. Extracted from the rAF handler so tests
 *  (and the debug hook below) can pump the sim when rAF is throttled. */
function advance(dtMs) {
  if (countdownMs > 0) {
    countdownMs -= dtMs;
    const n = Math.ceil(countdownMs / 1000);
    $("countdown").textContent = countdownMs <= 0 ? "GO" : String(n);
    $("countdown").style.display = "block";
    if (countdownMs <= 0) {
      setStatus("GO — set a time; it will be validated at the edge", "ok");
      setTimeout(() => ($("countdown").style.display = "none"), 700);
    }
    return;
  }
  if (frozen) return;
  acc += dtMs;
  while (acc >= DT_MS) {
    acc -= DT_MS;
    const raw = inputOverride ? inputOverride(curr) : input.sample(DT_MS / 1000);
    lastThrottle = raw.throttle;
    const q = quantize(raw);
    const status = sim.step(q.steer, q.throttle, q.brake, q.flags);
    ticks.push(q);
    prev = curr;
    curr = sim.state();

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
      $("sectors").textContent = fmtSectors(
        sectorTicks.map((t, i) => t - (sectorTicks[i - 1] ?? 0)),
      );
    }

    if (status & STATUS.LAP_INVALID && !invalid) {
      invalid = true;
      setStatus("lap invalidated (corner cut) — press R to restart", "bad");
    }
    if (status & STATUS.LAP_COMPLETE) {
      if (!invalid) onLapComplete();
      else frozen = true;
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

  // Camera: chase (spring-damped) or hood.
  if (cameraMode === "chase") {
    const behind = new THREE.Vector3(0, 3.2, -8.5).applyQuaternion(car.quaternion).add(car.position);
    camPos.lerp(behind, 1 - Math.exp(-4 * (dtMs / 1000)));
    camera.position.copy(camPos);
    camTarget.lerp(car.position, 0.6);
    camera.lookAt(camTarget.x, camTarget.y + 1.2, camTarget.z);
  } else {
    const hood = new THREE.Vector3(0, 1.15, 0.8).applyQuaternion(car.quaternion).add(car.position);
    const ahead = new THREE.Vector3(0, 0.9, 30).applyQuaternion(car.quaternion).add(car.position);
    camera.position.copy(hood);
    camera.lookAt(ahead);
    camPos.copy(hood);
  }

  lighting.update(); // time-of-day blend follows the local clock
  renderer.render(scene, camera);

  // ---- side channels: audio + FFB (read-only on sim state)
  audio.update(curr, lastThrottle, sim);
  if (ffb) {
    ffb.update(frozen || countdownMs > 0 ? 0 : sim.ffbTorque(), dtMs / 1000);
    $("mFfb").value = ffb.smoothed / ffb.maxNm;
  }

  // ---- HUD
  const runningTicks = frozen ? sim.lapTimeTicks() || ticks.length : ticks.length;
  $("lapTime").textContent = fmtMs(Math.round((runningTicks * 1000) / 400));
  $("speed").textContent = Math.round(curr.speed * 3.6);
  $("lapInfo").textContent = `best: ${bestMs ? fmtMs(bestMs) : "—"}   last: ${lastMs ? fmtMs(lastMs) : "—"}`;
  minimap.draw(car.position.toArray(), curr.checkpoints);

  // Tire surface temps (brush-model thermal layer): blue cold → green in the
  // window → red overheated. Order FL FR RL RR.
  if (sim.tireTemp(0) !== null) {
    const label = ["FL", "FR", "RL", "RR"];
    $("tireTemps").innerHTML = label
      .map((n, i) => {
        const t = sim.tireTemp(i);
        const color = t < 60 ? "#7fb3ff" : t <= 105 ? "#7fe3a1" : "#ff8f8f";
        return `${n} <span style="color:${color}">${Math.round(t)}°</span>`;
      })
      .join("  ");
  }

  // Live delta vs ghost: compare tick counts at equal lap progress.
  if (ghost && !frozen && countdownMs <= 0 && curr.lapProgress > 0.01) {
    while (deltaPtr < ghost.lapTicks - 1 && ghost.progress[deltaPtr] < curr.lapProgress) deltaPtr++;
    const d = (ticks.length - deltaPtr) / 400;
    $("delta").textContent = `${d >= 0 ? "+" : ""}${d.toFixed(2)}`;
    $("delta").style.color = d >= 0 ? "#ff8f8f" : "#7fe3a1";
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

await resetRun();
requestAnimationFrame(frame);

// Debug/test hook: lets automated checks pump physics when the tab is
// occluded (browsers throttle rAF to ~1 fps for background tabs, which for
// players just pauses the run — the accumulator clamp prevents catch-up).
window.__sttr = {
  advance,
  renderOnce: () => frame(performance.now()),
  skipCountdown: () => (countdownMs = 1),
  info: () => ({ ticks: ticks.length, frozen, invalid, countdownMs, speed: curr.speed, pos: curr.pos, quat: curr.quat, checkpoints: curr.checkpoints, lapProgress: curr.lapProgress, ghost: ghostCar.visible, ffbNm: sim.ffbTorque() }),
  track: { center: track.center, tangent: track.tangent },
  reset: resetRun,
  setInputOverride: (fn) => (inputOverride = fn),
  input,
  scene,
  FanatecFFB,
  registerHidDevice,
};

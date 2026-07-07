// Serverless Time Trial — browser client. Same wasm physics, same referee,
// no install. Fixed 400 Hz accumulator + interpolated rendering (uncapped rAF).
import * as THREE from "../vendor/three.module.js";
import { Sim, STATUS, DT_MS } from "./sim.js";
import { parseTrack, buildTrackMeshes, Minimap } from "./track.js";
import { Input, quantize } from "./input.js";
import { buildLapLog, submitLap, fetchLeaderboard, fmtMs } from "./lap.js";

const $ = (id) => document.getElementById(id);
const setStatus = (text, cls = "info") => { $("statusText").textContent = text; $("statusText").className = cls; };

// ---------------------------------------------------------------- bootstrap
setStatus("loading physics + track…");
const [sim, trackResp] = await Promise.all([Sim.load(), fetch("/api/track/current")]);
const trackBuf = await trackResp.arrayBuffer();
const track = parseTrack(trackBuf);
sim.loadTrack(new Uint8Array(trackBuf));

$("name").value = localStorage.getItem("sttr-name") ?? "";
$("name").addEventListener("change", () => localStorage.setItem("sttr-name", $("name").value));

// ---------------------------------------------------------------- three.js
const renderer = new THREE.WebGLRenderer({ canvas: $("gl"), antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1420);
scene.fog = new THREE.Fog(0x0e1420, 150, 480);
const camera = new THREE.PerspectiveCamera(62, 1, 0.1, 800);

scene.add(new THREE.HemisphereLight(0xbdd4ff, 0x2a2f38, 1.25));
const sun = new THREE.DirectionalLight(0xfff2d8, 1.7);
sun.position.set(120, 180, 60);
scene.add(sun);
scene.add(buildTrackMeshes(track));

const car = new THREE.Group();
const chassis = new THREE.Mesh(
  new THREE.BoxGeometry(1.9, 0.9, 4.4),
  new THREE.MeshLambertMaterial({ color: 0xd9483b }),
);
chassis.position.y = 0.25;
car.add(chassis);
const cabin = new THREE.Mesh(
  new THREE.BoxGeometry(1.4, 0.5, 1.8),
  new THREE.MeshLambertMaterial({ color: 0x1a2333 }),
);
cabin.position.set(0, 0.85, -0.3);
car.add(cabin);
scene.add(car);

const wheelMeshes = [];
for (let i = 0; i < 4; i++) {
  const w = new THREE.Mesh(
    new THREE.CylinderGeometry(0.33, 0.33, 0.28, 18),
    new THREE.MeshLambertMaterial({ color: 0x22262e }),
  );
  w.rotation.z = Math.PI / 2;
  const holder = new THREE.Group();
  holder.add(w);
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

// ---------------------------------------------------------------- game state
const input = new Input();
const minimap = new Minimap($("minimap"), track);
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

function resetRun() {
  sim.reset();
  prev = curr = sim.state();
  ticks = [];
  frozen = false;
  invalid = false;
  acc = 0;
  setStatus("GO — set a time and it will be validated at the edge", "ok");
}

addEventListener("keydown", (e) => {
  if (e.code === "KeyR") resetRun();
  if (e.code === "KeyI") $("config").style.display = $("config").style.display === "block" ? "none" : "block";
});

// Calibration UI
for (const btn of document.querySelectorAll("[data-cal]")) {
  btn.addEventListener("click", () => {
    const ch = btn.dataset.cal;
    $("calMsg").textContent = `move ONLY the ${ch} control now…`;
    if (!input.startCalibration(ch, (msg) => ($("calMsg").textContent = msg))) {
      $("calMsg").textContent = "no gamepad — turn the wheel / press a pedal first, then retry";
    }
  });
}
$("closeConfig").addEventListener("click", () => ($("config").style.display = "none"));

async function refreshBoard() {
  try {
    const entries = await fetchLeaderboard();
    $("entries").innerHTML = entries.length
      ? entries.slice(0, 10).map((e) => `<li><b>${fmtMs(e.ms)}</b> ${e.name.replace(/[<>&]/g, "")}</li>`).join("")
      : `<li class="sub">no laps yet — be first</li>`;
  } catch {
    $("entries").innerHTML = `<li class="sub">leaderboard unavailable</li>`;
  }
}
refreshBoard();

async function onLapComplete() {
  frozen = true;
  const lapTicks = sim.lapTimeTicks();
  lastMs = Math.round((lapTicks * 1000) / 400);
  bestMs = bestMs === null ? lastMs : Math.min(bestMs, lastMs);
  const log = buildLapLog(track.bytes, ticks.slice(0, lapTicks), sim.stateHash(), lapTicks);
  setStatus(`lap ${fmtMs(lastMs)} — submitting for edge validation…`);
  try {
    const result = await submitLap(log, $("name").value || "anon", (stage) => setStatus(`lap ${fmtMs(lastMs)} — referee: ${stage}…`));
    if (result.status === "accepted") {
      setStatus(`ACCEPTED ✔ ${fmtMs(result.lapTimeMs)} — world rank #${result.rank}. Press R to go again.`, "ok");
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
  if (!frozen) {
    acc += dtMs;
    while (acc >= DT_MS) {
      acc -= DT_MS;
      const raw = inputOverride ? inputOverride(curr) : input.sample(DT_MS / 1000);
      const q = quantize(raw);
      const status = sim.step(q.steer, q.throttle, q.brake, q.flags);
      ticks.push(q);
      prev = curr;
      curr = sim.state();
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
      // Fell off the world (no barriers in v1): auto-respawn at the line.
      if (curr.pos[1] < fellOffY) {
        resetRun();
        setStatus("off into the void — respawned at the start line", "bad");
        break;
      }
    }
  }
}

function frame(now) {
  requestAnimationFrame(frame);
  const dtMs = Math.min(now - last, 250); // anti-spiral clamp
  last = now;
  input.tickCalibration();
  advance(dtMs);

  // ---- interpolated render (alpha blend between the two newest sim states)
  const alpha = Math.min(acc / DT_MS, 1);
  const lerp = (a, b) => a + (b - a) * alpha;
  car.position.set(lerp(prev.pos[0], curr.pos[0]), lerp(prev.pos[1], curr.pos[1]), lerp(prev.pos[2], curr.pos[2]));
  tmpQa.set(...prev.quat);
  tmpQb.set(...curr.quat);
  car.quaternion.slerpQuaternions(tmpQa, tmpQb, alpha);

  for (let i = 0; i < 4; i++) {
    const wp = prev.wheels[i], wc = curr.wheels[i];
    wheelMeshes[i].position.set(lerp(wp.pos[0], wc.pos[0]), lerp(wp.pos[1], wc.pos[1]), lerp(wp.pos[2], wc.pos[2]));
    wheelMeshes[i].quaternion.copy(car.quaternion);
    wheelMeshes[i].rotateY(lerp(wp.steer, wc.steer));
    wheelMeshes[i].rotateX(lerp(wp.spin, wc.spin));
  }

  // Chase camera: spring toward a point behind and above the car.
  const behind = new THREE.Vector3(0, 3.2, -8.5).applyQuaternion(car.quaternion).add(car.position);
  camPos.lerp(behind, 1 - Math.exp(-4 * (dtMs / 1000)));
  camera.position.copy(camPos);
  camTarget.lerp(car.position, 0.6);
  camera.lookAt(camTarget.x, camTarget.y + 1.2, camTarget.z);

  renderer.render(scene, camera);

  // ---- HUD
  const runningTicks = frozen ? sim.lapTimeTicks() || ticks.length : ticks.length;
  $("lapTime").textContent = fmtMs(Math.round((runningTicks * 1000) / 400));
  $("speed").textContent = Math.round(curr.speed * 3.6);
  $("lapInfo").textContent = `best: ${bestMs ? fmtMs(bestMs) : "—"}   last: ${lastMs ? fmtMs(lastMs) : "—"}`;
  minimap.draw(car.position.toArray(), curr.checkpoints);

  // live meters in config panel
  if ($("config").style.display === "block") {
    const raw = input.sample(dtMs / 1000);
    $("mSteer").value = raw.steer;
    $("mThrottle").value = raw.throttle;
    $("mBrake").value = raw.brake;
    $("gpName").textContent = raw.device;
  }
}

resetRun();
requestAnimationFrame(frame);

// Debug/test hook: lets automated checks pump physics when the tab is
// occluded (browsers throttle rAF to ~1 fps for background tabs, which for
// players just pauses the run — the accumulator clamp prevents catch-up).
window.__sttr = {
  advance,
  renderOnce: () => frame(performance.now()),
  info: () => ({ ticks: ticks.length, frozen, invalid, speed: curr.speed, pos: curr.pos, quat: curr.quat, checkpoints: curr.checkpoints, lapProgress: curr.lapProgress }),
  track: { center: track.center, tangent: track.tangent },
  reset: resetRun,
  setInputOverride: (fn) => (inputOverride = fn),
};

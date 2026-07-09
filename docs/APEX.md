# Project Apex — Visual Direction (binding spec)

Source: "Project Apex - Visual Direction" design doc (user-provided, 2026-07).
Theme: **Grand Prix** — move the game from "reads as a prototype" (one
directional light, two-sided lambert, flat 0.28 ambient over three solid
colors) to a broadcast-grade F1 look. Primary target is the **web three.js
client** (`worker/public/`) since that is what people play; the native wgpu
client (`client/`) mirrors it where feasible. **None of this changes the TRK1
track format or the physics kernel** — it all hangs off geometry already built.

This supersedes the ad-hoc palette of the earlier Tufte HUD pass: it KEEPS
"kill the box" (unboxed layered HUD) but re-skins to the Apex system below.
Gold `#ffd479` is retired as the accent — the accent is now electric cyan.

## Design tokens

### Type
- **Titillium Web** — display / UI / labels (the motorsport face). Weights 400,
  600, 700. Vendor as woff2 in `worker/public/vendor/fonts/` + `@font-face`
  (no external CDN — self-contained).
- **JetBrains Mono** — all timing & numeric telemetry (monospace so digits
  never jitter; replaces the generic `tabular-nums`).

### Color
| token | hex | use |
|---|---|---|
| ink | `#eef2f6` | primary text |
| ink-dim | `#c8cfd6` | secondary text |
| muted | `#98a2ae` | labels |
| faint | `#5c6570` | chrome, rank numerals |
| **accent (electric cyan)** | `#22e0d0` | THE Apex accent — car livery accent, active UI, sun-catch highlights |
| heritage red | `#ff3b2e` | car body, warnings (also `#e4231c`, `#d63a2f`) |
| **delta — fastest (session best)** | `#b57bff` purple | timing-tower convention |
| **delta — personal best** | `#37d67a` green | |
| **delta — off-pace** | `#ffcb2b` yellow | |
| sky-blue | `#7fd3ff` | cool light |
| bg deep | `#0e1116` / `#08090b` / `#05060a` | panels, deep sky |

Timing-tower discipline: **purple = overall/session fastest, green = your PB,
yellow = off-pace.** This replaces the old plain green/red delta.

## Sections (each a workstream)

### 1. Sky, sun & weather
Gradient **dome sky + a real sun disc** replaces the flat clear color. A single
directional key light rotates with the sun and drives shadow direction, color
temperature, and fog together. **Five reference conditions** (a slider, default
Golden Hour): **Golden Hour · Midday · Overcast · Wet · Night.** Night rig =
point lights, emissive trackside ads, car headlights, heavy bloom. (Replaces
the current dawn/day/dusk/night keyframes.)

### 2. Read the road
The single grey ribbon becomes a **story of grip** — decals + thin edge-strips
hung off the existing centerline, **no new track format**: rubbered-in racing
line (have a basic version), grip/dust variation off-line, painted edge strips,
kerb decals, start-line and marshal markings.

### 3. Build the world
Empty grey terrain → **layered run-off**, read from the white line outward:
white line → kerb → run-off (grass / gravel materials) → **Armco / tire
barriers** → **grandstand + crowd**. All **instanced geometry** placed along the
track edge (extends the existing ambience system). "Uniform grey mesh →
grass/gravel/run-off materials + instanced props."

### 4. Kill the box (the car)
The red cuboid becomes a **proper open-wheeler with a real livery**: keep the
heritage red, add an **electric-cyan accent**, **carbon-fibre structure**,
**sponsor blocks**, and **PBR materials** (metallic-roughness) that catch the
sun. (The mesh is already an open-wheeler; this is materials + livery + PBR.)

### 5. Pick your seat (cameras)
Each view reframes the same target look. **Chase (default) · Cockpit · Bumper ·
TV cam.** Cockpit and bumper sell speed and scale; the TV cam is for replays and
the leaderboard. (Currently: chase + hood. Add cockpit, bumper, TV; cycle on C.)

### 6. Read the lap (HUD)
"A time-trial lives on the delta." Broadcast-grade Titillium + JetBrains Mono,
timing-tower colors (purple/green/yellow), just enough telemetry to feel the
car. Keeps the unboxed "kill the box" layout; re-skins type/color and makes the
delta a proper timing-tower readout.

## Roadmap (sequenced by visual return — from the doc)
- **P0 · Foundation (BIGGEST WIN)** — rewrite the lighting core; the whole
  scene lifts at once:
  - PBR shading (metallic-roughness, normal maps) — replace the 0.28 lambert
    (web: `MeshStandardMaterial` + an env/hemisphere for reflections;
    native: `shader.wgsl`).
  - **Sky pass** — gradient dome + sun disc replaces the clear color.
  - **Shadows** — cascaded/directional shadow maps off the existing dir light.
- **P1** — SSAO (contact shading on cockpit, wheels, barriers).
- **P2** — world instancing: grass/gravel/run-off materials + instanced props
  (barriers, stands, gantry, run-off).
- **P3** — car livery + PBR car materials; camera seats; HUD reskin; night rig
  with bloom.

## Implementation notes / constraints
- **Web-first** (`worker/public/`, three.js r0.170, vendored — no CDN at
  runtime). Self-contained: vendor fonts as woff2; sky/PBR/shadows/bloom via
  three.js core + a minimal postprocessing pass (vendor `EffectComposer`/
  `UnrealBloomPass`/`SSAOPass` from three examples, inlined) — keep it cheap
  (uncapped-fps target; guard on perf).
- Performance budget: this runs every frame alongside the 400 Hz sim; shadows
  and bloom must stay within frame. Prefer a single shadow-casting dir light,
  low-res cascade, half-res bloom. SSAO is P1 / optional-quality.
- Determinism/replay/physics: **untouched**. Pure presentation.
- Keep intact: `window.__sttr` hook, ghost car, FFB, calibration/HID panel,
  replay ▶ flow, minimap, damage HUD (from the SOFTBODY wave), tire glyph.

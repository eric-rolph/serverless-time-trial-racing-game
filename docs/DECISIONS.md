# Architectural Decision Records

Append-only. Each ADR: context → decision → consequences.

## ADR-001 — Same wasm binary is the physics authority everywhere

**Context.** The spec demands cross-platform floating-point determinism at 400 Hz.
Native C builds cannot guarantee this across x86/ARM/compilers (FMA contraction,
x87 vs SSE, libm differences) even without `-ffast-math`.

**Decision.** Compile the physics kernel to wasm32 once. The Rust client embeds it
via `wasmtime`; the Worker instantiates the same bytes in V8. WebAssembly semantics
mandate bit-exact IEEE-754 arithmetic, so determinism is a property of the
*artifact*, not of every host platform's compiler.

**Consequences.** Client pays wasm-call overhead per tick (~48k calls per 2-minute
lap — negligible). Native builds are test-only. The wasm binary is a release
artifact whose hash must be tracked (client and validator must run the same build).

## ADR-002 — Box3D pinned as submodule @ `540ea38`, scalar, single-threaded

**Context.** Erin Catto's Box3D (C17) is pre-1.0 and moving. Determinism requires
a frozen snapshot and no data-race-dependent solver ordering.

**Decision.** Git submodule pinned to `540ea387b0c02bf714fbfdcc8fb88c039c35fe6f`.
`BOX3D_DISABLE_SIMD=ON` for native test builds; wasm build is scalar (no SIMD128).
Worker/task-system parallelism disabled (single-threaded solve). Single precision.

**Consequences.** Slower than SIMD/threaded — irrelevant at one car per world.
Upgrades are deliberate events that bump the ABI version and reset leaderboards
(weekly rotation makes this cheap).

## ADR-003 — 400 Hz fixed step, integer-quantized inputs as canonical truth

**Context.** Replay validation requires the validator to see *exactly* what the
sim consumed. Raw analog floats sampled at 1 kHz don't align to the 400 Hz grid.

**Decision.** `DT = 0.0025f`, defined once. Inputs quantized to i16/u16 at the
tick boundary; the quantized integers are simultaneously fed to the sim and
appended to the log. LAPLOG records are the ground truth — not the analog signal.

**Consequences.** No client/validator drift by construction. Quantization
resolution (1/32767 steering) is far below hardware ADC noise, so no fidelity loss.

## ADR-004 — Ed25519 signatures; keypair = identity

**Context.** Need tamper-evidence on submitted logs and a lightweight identity
without accounts.

**Decision.** Client-generated Ed25519 keypair, stored locally. Sign
`"sttr-lap-v1" ‖ LAPLOG`. Worker verifies via WebCrypto `Ed25519` (supported on
Workers). Leaderboard identity is the public key.

**Consequences.** No password/account infrastructure. Lost key = lost identity
(acceptable for a time-trial toy). Signature does not prove human origin — that's
the heuristic gate's job (ADR-006).

## ADR-005 — Cloudflare KV for leaderboard; R2 deferred

**Context.** Spec mandates KV. KV list() is lexicographic.

**Decision.** Zero-padded lap-tick counts in keys make prefix scans return
time-sorted results without an index. R2 credentials exist but replay archival to
R2 is out of MVP scope (noted for future ghost-car feature).

**Consequences.** Eventual consistency (~60 s) on leaderboard reads — fine for
async time trials. 100-entry top list per read via one list() call.

## ADR-006 — Heuristic gate is advisory-strict, replay is authoritative

**Context.** TAS detection from telemetry alone has irreducible false-positive
risk; replay validation already bounds cheating to "inputs a robot could produce".

**Decision.** Heuristics reject only on ≥ 2 independent flags; all flags are
persisted with the entry. The replay + state-hash check is the hard gate.

**Consequences.** A sophisticated TAS with injected fake jitter can pass — this is
an arms race by nature. The design goal is raising cost, not perfection.

## ADR-007 — Raycast-suspension vehicle, Pacejka MF on top of Box3D rigid body

**Context.** Spec: Pacejka Magic Formula tires + independent suspension. Full
constraint-based wheels are overkill and harder to keep deterministic.

**Decision.** Chassis = one Box3D rigid body. Each wheel: fixed-point raycast
suspension (spring + damper along chassis-local axis), slip ratio/angle computed
from contact-patch velocity, Pacejka MF-lite (B,C,D,E per axis) forces applied via
`b3Body_ApplyForce` at the patch. Independent geometry per corner (track width,
wheelbase, camber-free v1).

**Consequences.** Industry-standard arcade-sim architecture (same family as
Unity's WheelCollider / BeamNG's simplified modes). No wheel rigid bodies ⇒ fewer
solver islands ⇒ faster, more deterministic. Kerb/jump fidelity limited by
raycast (acceptable for v1).

## ADR-008 — Emscripten STANDALONE_WASM reactor build

**Context.** The module must load in a Worker (no Emscripten JS glue allowed in
bundle budget) *and* in wasmtime (WASI-flavored host).

**Decision.** `emcc -sSTANDALONE_WASM=1 --no-entry` with a fixed export list; no
imports beyond optional WASI stubs that are never called on the hot path.

**Consequences.** One artifact, two hosts. Must avoid libc calls that pull in
WASI imports the Worker can't satisfy (no printf/fopen in kernel code paths).

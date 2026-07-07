# Force Feedback — research & integration design

Surveyed 2026-07-07. Goal: physics-derived FFB for direct-drive wheels
(Fanatec CSL DD) in the native Rust client.

## Reference projects

| project | what its FFB model does | config parameters worth copying |
|---|---|---|
| [irFFB](https://github.com/nlp80/irFFB) (C++, iRacing) | Feeds 360 Hz *steering-column torque* telemetry straight to the wheel as DirectInput constant-force updates; upsampling + low-latency digital filters over the sim's 60 Hz output | max force (Nm), min force floor, damping (applied near lock stops), reduce-at-standstill (anti-oscillation), understeer/SoP effects |
| [LMUFFB](https://github.com/coasting-nc/LMUFFB) (Le Mans Ultimate) | Steering **rack torque** as the base signal; adds processing on top | gain, rear-align-torque weight, dynamic slope detection (tire saturation), torque anticipation from **pneumatic-trail drop**, damping, smoothing, vibrotactile layers |
| [OpenFFBoard](https://github.com/Ultrawipf/OpenFFBoard) (firmware) | The device side: universal FFB interface implementing the USB PID device class | effect gains per class (constant/spring/damper/friction/inertia), rotation range, endstop behavior — a good checklist of what effects a wheel consumes |
| [unity-ffb](https://github.com/skaughtx0r/unity-ffb) / [SimSteeringTest](https://github.com/amcgee7/SimSteeringTest) | Minimal DirectInput FFB host examples (effect acquisition, constant force + spring updates, focus-loss reacquisition) | the plumbing patterns, incl. the classic re-acquire-on-focus-loss bug |

Key architectural takeaway (consistent across irFFB and LMUFFB): **the base FFB
signal is steering rack torque from the tire model — everything else is
seasoning.** We are unusually well positioned: unlike telemetry apps that
reverse-engineer torque at 60–360 Hz, our kernel computes tire forces at 400 Hz
natively.

## Integration design (native client)

1. **Kernel** (non-breaking ABI addition — new export, SimStateV1 unchanged):
   `sim_ffb_torque() -> f32` — steering rack torque in Nm:
   `Σ_front [ Fy · (pneumatic_trail(slip_angle) + caster_trail) + Fx · scrub_radius ] / steering_ratio`
   with pneumatic trail collapsing toward 0 as slip angle approaches the Pacejka
   peak (this collapse is *the* understeer cue LMUFFB reconstructs artificially).
2. **Client output path**: no mature Rust FFB crate exists → DirectInput8 via
   the `windows` crate: create `ConstantForce` effect on the wheel device,
   update magnitude each render frame (wheel-side interpolation smooths 60–400 Hz
   gaps; irFFB ships ~360 Hz updates through the same API). SDL3 haptics is the
   portable fallback.
3. **Config** (`ffb.toml`, hot-reloadable):

```toml
[ffb]
enabled      = true
max_torque_nm = 8.0    # clip vs CSL DD's 8 Nm peak
gain          = 0.85
min_force     = 0.02   # deadzone-crossing floor (belt-drive rigs need more)
damping       = 0.10   # velocity-proportional, also near lock stops
friction      = 0.03
smoothing_hz  = 120    # 1-pole low-pass on rack torque
standstill_reduction = true   # irFFB-style anti-oscillation below 2 m/s
kerb_vibration_gain  = 0.5    # from susp_compression velocity spikes
understeer_drop      = 1.0    # scale of pneumatic-trail collapse
invert               = false
update_hz            = 400    # effect update rate (DirectInput will coalesce)
```

**Determinism note**: FFB is read-only on sim state — it must never feed back
into the input log except through the human's hands. No ABI or replay impact.

**Browser limitation**: the Gamepad API has no torque FFB (only rumble), and
WebHID cannot express PID effects usefully — so the web client (worker/public)
is play-without-FFB; the native client is where the wheel comes alive.

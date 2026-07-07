//! sttr-client — native client for the serverless time-trial racing game.
//!
//! Subcommands:
//!   race    interactive session (wgpu window, wheel/keyboard input)
//!   submit  submit an existing lap.laplog over WebSocket
//!   replay  headless local verification of a lap.laplog through sim.wasm
//!   devices list input devices gilrs can see

mod game_loop;
mod input;
mod laplog;
mod render;
mod signing;
mod sim;
mod submit;
mod track;

use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::{Args, Parser, Subcommand, ValueEnum};

use crate::input::{
    detect_gamepads, parse_binding, KeyboardInput, SelectedInput, WheelConfig, WheelInput,
};
use crate::laplog::LapLog;
use crate::sim::{Sim, SimBackend, STATUS_ERROR, STATUS_LAP_COMPLETE, STATUS_LAP_INVALID};
use crate::signing::Identity;
use crate::track::Track;

#[derive(Parser)]
#[command(name = "sttr-client", version, about = "serverless time-trial racing client")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Drive: open a window, run the sim at 400 Hz, record and sign your lap.
    Race(RaceArgs),
    /// Submit a previously recorded lap.laplog to the leaderboard server.
    Submit(SubmitArgs),
    /// Headless replay of a lap.laplog through sim.wasm (local verification).
    Replay(ReplayArgs),
    /// List input devices visible to gilrs.
    Devices,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum InputKind {
    Wheel,
    Keyboard,
}

#[derive(Args)]
struct RaceArgs {
    /// Local track.bin (TRK1) path.
    #[arg(long, conflicts_with = "server")]
    track: Option<PathBuf>,
    /// Server base URL; fetches {server}/api/track/current.
    #[arg(long)]
    server: Option<String>,
    /// Path to the deterministic physics kernel.
    #[arg(long, default_value = "../physics/out/sim.wasm")]
    sim_wasm: PathBuf,
    /// Force an input backend (default: wheel if a gamepad is detected).
    #[arg(long, value_enum)]
    input: Option<InputKind>,
    /// gilrs binding for steering (e.g. leftstickx, rightstickx, leftz).
    #[arg(long, default_value = "leftstickx")]
    steer_axis: String,
    /// gilrs binding for throttle (axis or trigger, e.g. righttrigger2, rightz).
    #[arg(long, default_value = "righttrigger2")]
    throttle_axis: String,
    /// gilrs binding for brake (axis or trigger, e.g. lefttrigger2, leftz).
    #[arg(long, default_value = "lefttrigger2")]
    brake_axis: String,
    /// Display name on the leaderboard (max 24 chars).
    #[arg(long, default_value = "anonymous")]
    name: String,
    /// wss:// submit endpoint; when set, completed laps are submitted.
    #[arg(long)]
    submit_to: Option<String>,
    /// Where to write the recorded lap.
    #[arg(long, default_value = "lap.laplog")]
    out: PathBuf,
}

#[derive(Args)]
struct SubmitArgs {
    /// lap.laplog file to submit.
    laplog: PathBuf,
    /// wss:// submit endpoint (CONTRACTS.md §5).
    #[arg(long)]
    to: String,
    /// Display name on the leaderboard (max 24 chars).
    #[arg(long, default_value = "anonymous")]
    name: String,
}

#[derive(Args)]
struct ReplayArgs {
    /// lap.laplog file to replay.
    laplog: PathBuf,
    /// The exact track.bin the lap was driven on.
    #[arg(long)]
    track: PathBuf,
    /// Path to the deterministic physics kernel.
    #[arg(long, default_value = "../physics/out/sim.wasm")]
    sim_wasm: PathBuf,
}

fn main() {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Race(args) => cmd_race(args),
        Command::Submit(args) => cmd_submit(args),
        Command::Replay(args) => cmd_replay(args),
        Command::Devices => cmd_devices(),
    };
    if let Err(e) = result {
        eprintln!("error: {e:#}");
        std::process::exit(1);
    }
}

// ---------------------------------------------------------------------------

fn load_track_bytes(track: &Option<PathBuf>, server: &Option<String>) -> Result<Vec<u8>> {
    match (track, server) {
        (Some(path), _) => {
            std::fs::read(path).with_context(|| format!("reading track '{}'", path.display()))
        }
        (None, Some(server)) => {
            let url = format!("{}/api/track/current", server.trim_end_matches('/'));
            println!("fetching track from {url} …");
            let resp = reqwest::blocking::get(&url)
                .with_context(|| format!("GET {url}"))?
                .error_for_status()
                .context("server returned an error for the track")?;
            let advertised = resp
                .headers()
                .get("x-track-hash")
                .and_then(|v| v.to_str().ok())
                .map(str::to_owned);
            let bytes = resp.bytes().context("reading track body")?.to_vec();
            if let Some(adv) = advertised {
                let actual = format!("{:016x}", laplog::fnv1a64(&bytes));
                if !adv.eq_ignore_ascii_case(&actual) {
                    eprintln!(
                        "warning: x-track-hash header '{adv}' != computed fnv1a64 '{actual}'"
                    );
                }
            }
            Ok(bytes)
        }
        (None, None) => bail!("provide a track: --track <track.bin> or --server <url>"),
    }
}

fn select_input(args: &RaceArgs) -> Result<SelectedInput> {
    let config = WheelConfig {
        steer: parse_binding(&args.steer_axis)
            .with_context(|| format!("unknown --steer-axis '{}'", args.steer_axis))?,
        throttle: parse_binding(&args.throttle_axis)
            .with_context(|| format!("unknown --throttle-axis '{}'", args.throttle_axis))?,
        brake: parse_binding(&args.brake_axis)
            .with_context(|| format!("unknown --brake-axis '{}'", args.brake_axis))?,
        ..WheelConfig::default()
    };

    match args.input {
        Some(InputKind::Wheel) => {
            let w = WheelInput::new(config).map_err(|e| anyhow::anyhow!(e))?;
            println!("input: wheel/gamepad (forced)");
            Ok(SelectedInput::Wheel(w))
        }
        Some(InputKind::Keyboard) => {
            println!("input: keyboard (forced) — WASD/arrows drive, space = handbrake");
            Ok(SelectedInput::Keyboard(KeyboardInput::new()))
        }
        None => {
            let pads = detect_gamepads();
            if pads.is_empty() {
                println!("input: keyboard (no gamepad detected) — WASD/arrows, space = handbrake");
                Ok(SelectedInput::Keyboard(KeyboardInput::new()))
            } else {
                println!("input: wheel/gamepad (detected: {})", pads.join(", "));
                match WheelInput::new(config) {
                    Ok(w) => Ok(SelectedInput::Wheel(w)),
                    Err(e) => {
                        eprintln!("wheel init failed ({e}); falling back to keyboard");
                        Ok(SelectedInput::Keyboard(KeyboardInput::new()))
                    }
                }
            }
        }
    }
}

fn cmd_race(args: RaceArgs) -> Result<()> {
    if args.name.chars().count() > 24 {
        bail!("--name must be at most 24 characters (CONTRACTS.md §5)");
    }
    let track_bytes = load_track_bytes(&args.track, &args.server)?;
    let track = Track::parse(&track_bytes).map_err(|e| anyhow::anyhow!("bad track: {e}"))?;
    println!(
        "track: hash {:016x}, {} centerline samples, {} checkpoints, {} terrain tris",
        track.hash,
        track.centerline.len(),
        track.checkpoints.len(),
        track.terrain_triangles.len()
    );

    let mut sim = Sim::load(&args.sim_wasm)?;
    sim.load_track(&track_bytes)?;
    sim.reset()?;
    println!("sim.wasm loaded from '{}'", args.sim_wasm.display());

    let input = select_input(&args)?;

    render::run_race(render::RaceContext {
        sim,
        track,
        input,
        name: args.name,
        submit_to: args.submit_to,
        out_path: args.out,
    })
}

fn cmd_submit(args: SubmitArgs) -> Result<()> {
    if args.name.chars().count() > 24 {
        bail!("--name must be at most 24 characters (CONTRACTS.md §5)");
    }
    let bytes = std::fs::read(&args.laplog)
        .with_context(|| format!("reading '{}'", args.laplog.display()))?;
    // Validate before sending; the exact on-disk bytes are what gets signed.
    let log = LapLog::deserialize(&bytes).map_err(|e| anyhow::anyhow!("invalid laplog: {e}"))?;
    println!(
        "lap: {} ticks ({:.3} s), track {:016x}, final hash {:016x}",
        log.ticks.len(),
        log.claimed_lap_ticks as f64 * game_loop::DT,
        log.track_hash,
        log.final_state_hash
    );

    let identity = Identity::load_or_generate()?;
    let sig = identity.sign_laplog(&bytes);
    let outcome =
        submit::submit_blocking(&args.to, &identity.public_key_bytes(), &sig, &args.name, &bytes)?;
    println!("server verdict: {outcome}");
    if !outcome.accepted {
        std::process::exit(2);
    }
    Ok(())
}

fn cmd_replay(args: ReplayArgs) -> Result<()> {
    let bytes = std::fs::read(&args.laplog)
        .with_context(|| format!("reading '{}'", args.laplog.display()))?;
    let log = LapLog::deserialize(&bytes).map_err(|e| anyhow::anyhow!("invalid laplog: {e}"))?;

    let track_bytes = std::fs::read(&args.track)
        .with_context(|| format!("reading track '{}'", args.track.display()))?;
    let track_hash = laplog::fnv1a64(&track_bytes);
    if track_hash != log.track_hash {
        bail!(
            "track mismatch: laplog was recorded on {:016x}, '{}' hashes to {track_hash:016x}",
            log.track_hash,
            args.track.display()
        );
    }

    let mut sim = Sim::load(&args.sim_wasm)?;
    sim.load_track(&track_bytes)?;
    sim.reset()?;

    println!("replaying {} ticks …", log.ticks.len());
    let started = std::time::Instant::now();
    let status = sim.replay(&log.ticks)?;
    let elapsed = started.elapsed();

    let final_hash = sim.state_hash()?;
    let lap_ticks = sim.lap_time_ticks()?;

    println!("replay finished in {:.1} ms", elapsed.as_secs_f64() * 1000.0);
    println!("status bits:      {status:#010x}");
    println!("final state hash: {final_hash:016x} (logged {:016x})", log.final_state_hash);
    println!(
        "lap time:         {lap_ticks} ticks = {:.3} s (claimed {} ticks)",
        lap_ticks as f64 * game_loop::DT,
        log.claimed_lap_ticks
    );

    let hash_ok = final_hash == log.final_state_hash;
    let ticks_ok = lap_ticks == log.claimed_lap_ticks;
    let complete = status & STATUS_LAP_COMPLETE != 0;
    let invalid = status & (STATUS_LAP_INVALID | STATUS_ERROR) != 0;

    if hash_ok && ticks_ok && complete && !invalid {
        println!("VERDICT: MATCH — this lap replays deterministically");
        Ok(())
    } else {
        if !hash_ok {
            eprintln!("MISMATCH: final state hash differs");
        }
        if !ticks_ok {
            eprintln!("MISMATCH: lap tick count differs");
        }
        if !complete {
            eprintln!("MISMATCH: replay did not reach LAP_COMPLETE");
        }
        if invalid {
            eprintln!("MISMATCH: replay flagged LAP_INVALID/ERROR");
        }
        std::process::exit(3);
    }
}

fn cmd_devices() -> Result<()> {
    let pads = detect_gamepads();
    if pads.is_empty() {
        println!("no gamepads/wheels detected by gilrs");
        println!("(on Windows, gilrs sees XInput devices; DirectInput-only wheels may not appear)");
    } else {
        println!("detected {} device(s):", pads.len());
        for p in &pads {
            println!("  - {p}");
        }
    }
    Ok(())
}

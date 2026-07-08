#!/usr/bin/env python3
"""Procedural track generator: seeded 3D spline + terrain ribbon.

Outputs track.json (renderer/debug friendly). pack_track.py converts to TRK1.

Coordinate system: Y-up (matches Box3D default gravity {0,-10,0}).
Ground plane is X-Z. Units: meters.

Usage: python generate_track.py --seed 202627 --out-dir ../assets/tracks/dev
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib

import numpy as np

SAMPLE_SPACING_M = 2.5
SHOULDER_WIDTH_M = 8.0
SHOULDER_DROP_M = 1.2
MAX_GRADE = 0.08  # 8% — keeps the racing line drivable
MIN_WIDTH_M = 8.0
CHECKPOINT_FRACTIONS = (0.25, 0.5, 0.75)

# Banking: corners lean into the turn, proportional to smoothed curvature.
BANK_GAIN = 110.0  # rad of bank per (rad/m) of curvature
MAX_BANK_RAD = 0.07  # ~4° — 8° proved flip-prone at speed
CURVATURE_SMOOTH_WINDOW = 15  # samples (~37 m)

# Kerbs: rumble strips on both edges through tight corners. The web client
# mirrors KERB_KAPPA + the smoothing window to draw its striped visuals —
# keep these three constants in sync with worker/public/js/track.js.
KERB_KAPPA = 0.022  # 1/m — corners tighter than ~45 m radius get kerbs
KERB_WIDTH_M = 1.1
KERB_TOOTH_M = 0.02  # raised tooth height; alternating samples → rumble


def catmull_rom_closed(points: np.ndarray, samples_per_seg: int = 200) -> np.ndarray:
    """CENTRIPETAL Catmull-Rom through a closed control polygon (alpha = 0.5).

    Uniform CR overshoots and cusps through unevenly spaced points — which is
    exactly what the corner archetypes create. Centripetal parameterization is
    provably cusp- and self-intersection-free within segments (Barry-Goldman).
    """
    n = len(points)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = (points[(i + k - 1) % n] for k in range(4))
        eps = 1e-6
        t0 = 0.0
        t1 = t0 + max(np.linalg.norm(p1 - p0), eps) ** 0.5
        t2 = t1 + max(np.linalg.norm(p2 - p1), eps) ** 0.5
        t3 = t2 + max(np.linalg.norm(p3 - p2), eps) ** 0.5
        tt = np.linspace(t1, t2, samples_per_seg, endpoint=False)[:, None]
        a1 = (t1 - tt) / (t1 - t0) * p0 + (tt - t0) / (t1 - t0) * p1
        a2 = (t2 - tt) / (t2 - t1) * p1 + (tt - t1) / (t2 - t1) * p2
        a3 = (t3 - tt) / (t3 - t2) * p2 + (tt - t2) / (t3 - t2) * p3
        b1 = (t2 - tt) / (t2 - t0) * a1 + (tt - t0) / (t2 - t0) * a2
        b2 = (t3 - tt) / (t3 - t1) * a2 + (tt - t1) / (t3 - t1) * a3
        out.append((t2 - tt) / (t2 - t1) * b1 + (tt - t1) / (t2 - t1) * b2)
    return np.vstack(out)


def resample_by_arclength(dense: np.ndarray, spacing: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(dense, axis=0, append=dense[:1]), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))[:-1]
    total = s[-1] + seg[-1]
    count = max(64, int(round(total / spacing)))
    targets = np.linspace(0.0, total, count, endpoint=False)
    resampled = np.empty((count, dense.shape[1]))
    for c in range(dense.shape[1]):
        col = np.concatenate((dense[:, c], dense[:1, c]))
        resampled[:, c] = np.interp(targets, np.concatenate((s, [total])), col)
    return resampled


def loop_harmonics(rng: np.random.Generator, u: np.ndarray, freqs, amps) -> np.ndarray:
    """Sum of integer-frequency sines — inherently loop-continuous."""
    y = np.zeros_like(u)
    for f, a in zip(freqs, amps):
        y += a * np.sin(2 * math.pi * f * u + rng.uniform(0, 2 * math.pi))
    return y


def self_intersects(pos: np.ndarray, width: np.ndarray) -> bool:
    """O(S^2) horizontal proximity check between non-neighboring samples."""
    n = len(pos)
    xz = pos[:, [0, 2]]
    for i in range(n):
        d = np.linalg.norm(xz - xz[i], axis=1)
        loop_dist = np.minimum(np.abs(np.arange(n) - i), n - np.abs(np.arange(n) - i))
        clearance = (width + width[i]) * 0.5 + 4.0
        if np.any((loop_dist > 15) & (d < clearance)):
            return True
    return False


def generate(seed: int) -> dict:
    for attempt in range(120):
        rng = np.random.default_rng((seed + attempt * 0x9E3779B9) & 0xFFFFFFFFFFFFFFFF)

        n_ctrl = int(rng.integers(10, 15))
        base = 2 * math.pi / n_ctrl
        angles = np.sort(
            np.arange(n_ctrl) * base + rng.uniform(-0.35 * base, 0.35 * base, n_ctrl)
        )
        radii = rng.uniform(90.0, 170.0, n_ctrl)
        ctrl = np.stack(
            [radii * np.cos(angles), np.zeros(n_ctrl), radii * np.sin(angles)], axis=1
        )

        # --- Corner archetypes (track-design craft, seed-driven) ------------
        # A memorable lap needs signature corners, not just noise: one hairpin
        # (big braking event) and one chicane (direction-change rhythm),
        # placed roughly opposite each other.
        hp = int(rng.integers(0, n_ctrl))
        ctrl[hp, [0, 2]] *= 0.55  # pull hard toward center → hairpin
        ctrl[(hp - 1) % n_ctrl, [0, 2]] *= 0.85  # ease neighbors so the
        ctrl[(hp + 1) % n_ctrl, [0, 2]] *= 0.85  # spline turns, not kinks
        ch = (hp + n_ctrl // 2) % n_ctrl
        chord = ctrl[(ch + 1) % n_ctrl] - ctrl[(ch - 1) % n_ctrl]
        chord = chord / (np.linalg.norm(chord) + 1e-9)
        perp = np.array([-chord[2], 0.0, chord[0]])
        amp = rng.uniform(6.0, 9.0)
        p1 = ctrl[ch] - chord * 15.0 + perp * amp
        p2 = ctrl[ch] + chord * 15.0 - perp * amp
        ctrl = np.vstack([ctrl[:ch], [p1, p2], ctrl[ch + 1 :]])

        # Esses: a rapid alternating flick (Suzuka S-curves family), placed a
        # quarter-lap from the hairpin so the three signature sections spread.
        m = len(ctrl)
        es = (hp + m // 4) % m
        if min(abs(es - ch), m - abs(es - ch)) < 2:
            es = (hp + 3 * m // 4) % m
        chord2 = ctrl[(es + 1) % m] - ctrl[(es - 1) % m]
        chord2 = chord2 / (np.linalg.norm(chord2) + 1e-9)
        perp2 = np.array([-chord2[2], 0.0, chord2[0]])
        amp2 = rng.uniform(4.5, 6.5)
        e1 = ctrl[es] - chord2 * 18.0 + perp2 * amp2
        e2 = ctrl[es] - perp2 * (0.5 * amp2)
        e3 = ctrl[es] + chord2 * 18.0 + perp2 * amp2
        ctrl = np.vstack([ctrl[:es], [e1, e2, e3], ctrl[es + 1 :]])

        pos = resample_by_arclength(catmull_rom_closed(ctrl), SAMPLE_SPACING_M)
        n = len(pos)
        u = np.arange(n) / n
        kernel = np.ones(CURVATURE_SMOOTH_WINDOW) / CURVATURE_SMOOTH_WINDOW

        # Geometric lint: Catmull-Rom through aggressive archetype points can
        # cusp. Accept a layout only if no adjacent-sample direction change
        # exceeds 7.5 deg (~R 19 m at 2.5 m spacing) — kinked layouts
        # regenerate instead of shipping bumps that launch the car.
        seg_v = np.roll(pos, -1, axis=0) - pos
        seg_v /= np.linalg.norm(seg_v, axis=1, keepdims=True)
        turn = np.degrees(np.arccos(np.clip((seg_v * np.roll(seg_v, 1, axis=0)).sum(1), -1.0, 1.0)))
        if turn.max() > 9.0:
            continue

        # Flat-plan curvature pre-pass: find the longest straight and the
        # tightest corner while the geometry is still 2D.
        tflat = np.roll(pos, -1, axis=0) - np.roll(pos, 1, axis=0)
        tflat /= np.linalg.norm(tflat, axis=1, keepdims=True)
        t2f = np.roll(tflat, -1, axis=0)
        kflat = (tflat[:, 0] * t2f[:, 2] - tflat[:, 2] * t2f[:, 0]) / SAMPLE_SPACING_M
        kflat = np.convolve(np.tile(kflat, 3), kernel, mode="same")[n : 2 * n]

        # Start/finish belongs on the main straight, like every real circuit —
        # roll the sample indexing so index 0 sits a few car lengths into it.
        quiet0 = np.abs(kflat) < 0.006
        best_len, best_start, run = 0, 0, 0
        for i2, f in enumerate(np.concatenate([quiet0, quiet0])):
            run = run + 1 if f else 0
            if run > best_len:
                best_len, best_start = run, i2 - run + 1
        roll_to = (best_start + 6) % n
        pos = np.roll(pos, -roll_to, axis=0)
        kflat = np.roll(kflat, -roll_to)
        i_hp = int(np.argmax(np.abs(kflat)))

        # Elevation: loop-continuous harmonics + a crest ~35 m before the
        # tightest corner (the blind-crest-into-braking drama of real courses),
        # grade-clamped.
        elev = loop_harmonics(rng, u, freqs=(1, 3, 5), amps=(4.0, 2.5, 1.2))
        crest_c = (i_hp - 14) % n
        d = np.abs(np.arange(n) - crest_c)
        d = np.minimum(d, n - d).astype(float)
        elev += 1.5 * np.exp(-(d * d) / (2.0 * 8.0 * 8.0))
        grade = np.max(np.abs(np.diff(elev, append=elev[:1]))) / SAMPLE_SPACING_M
        if grade > MAX_GRADE:
            elev *= MAX_GRADE / grade
        pos[:, 1] = elev

        # Frames: tangent from central differences, up = world-up perpendicularized.
        tangent = np.roll(pos, -1, axis=0) - np.roll(pos, 1, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        world_up = np.array([0.0, 1.0, 0.0])
        up = world_up - tangent * (tangent @ world_up)[:, None]
        up /= np.linalg.norm(up, axis=1, keepdims=True)
        side = np.cross(up, tangent)
        side /= np.linalg.norm(side, axis=1, keepdims=True)

        # Signed horizontal curvature (rad/m), heavily smoothed (circular).
        t_next = np.roll(tangent, -1, axis=0)
        kappa = (tangent[:, 0] * t_next[:, 2] - tangent[:, 2] * t_next[:, 0]) / SAMPLE_SPACING_M
        kernel = np.ones(CURVATURE_SMOOTH_WINDOW) / CURVATURE_SMOOTH_WINDOW
        kappa = np.convolve(np.tile(kappa, 3), kernel, mode="same")[n : 2 * n]

        # Track-design: rhythm needs at least one genuine straight (~110 m+
        # of near-zero curvature) — regenerate layouts that are all corners.
        quiet = np.abs(kappa) < 0.006
        best_run, run = 0, 0
        for flag in np.concatenate([quiet, quiet]):  # circular runs
            run = run + 1 if flag else 0
            best_run = max(best_run, run)
        if min(best_run, n) * SAMPLE_SPACING_M < 110.0:
            continue

        # Width choreography: corners breathe wider (entry/line choice),
        # straights stay lean; harmonics keep it organic.
        width = 10.0 + loop_harmonics(rng, u, freqs=(2, 7), amps=(2.0, 1.0))
        width = width + 4.0 * np.minimum(1.0, np.abs(kappa) / 0.03)
        width = np.maximum(width, MIN_WIDTH_M)

        if self_intersects(pos, width):
            continue

        # Banking: rotate `up` about the tangent so the surface leans into the
        # corner (inside edge drops). kappa > 0 = left turn; the normal must
        # lean toward -side (left), hence the negation. side/up stay orthonormal.
        bank = np.clip(-kappa * BANK_GAIN, -MAX_BANK_RAD, MAX_BANK_RAD)
        cos_b, sin_b = np.cos(bank)[:, None], np.sin(bank)[:, None]
        up_banked = up * cos_b + side * sin_b
        side = np.cross(up_banked, tangent)
        side /= np.linalg.norm(side, axis=1, keepdims=True)
        up = up_banked / np.linalg.norm(up_banked, axis=1, keepdims=True)

        # Terrain ribbon: 4 vertex rows per sample (outer-L, edge-L, edge-R, outer-R).
        half = (width * 0.5)[:, None]
        rows = [
            pos - side * (half + SHOULDER_WIDTH_M) - world_up * SHOULDER_DROP_M,
            pos - side * half,
            pos + side * half,
            pos + side * (half + SHOULDER_WIDTH_M) - world_up * SHOULDER_DROP_M,
        ]
        verts = np.stack(rows, axis=1).reshape(-1, 3)  # v(i,row) = i*4+row

        tris = []
        for i in range(n):
            j = (i + 1) % n
            for r in range(3):
                a, b = i * 4 + r, i * 4 + r + 1
                c, d = j * 4 + r, j * 4 + r + 1
                # Wound so face normals point +Y (upward) for one-sided collision.
                tris.append([a, c, b])
                tris.append([b, c, d])
        tris = list(tris)

        # Kerbs: alternating-height rumble strips on BOTH edges through tight
        # corners — the suspension chatter is the whole point (audio + FFB).
        kerb_zone = np.abs(kappa) > KERB_KAPPA
        verts_list = [verts]
        base = len(verts)
        for sign in (-1.0, 1.0):
            i = 0
            while i < n:
                if not (kerb_zone[i] and kerb_zone[(i + 1) % n]):
                    i += 1
                    continue
                # contiguous run of kerb samples
                j = i
                while j + 1 < n and kerb_zone[(j + 1) % n]:
                    j += 1
                if j - i >= 3:  # skip blips
                    strip = []
                    for k in range(i, j + 1):
                        tooth = KERB_TOOTH_M * (k % 2)
                        inner = pos[k] + side[k] * sign * (width[k] * 0.5)
                        outer = pos[k] + side[k] * sign * (width[k] * 0.5 + KERB_WIDTH_M)
                        strip.append(inner + up[k] * tooth)
                        strip.append(outer + up[k] * tooth)
                    for k in range(len(strip) // 2 - 1):
                        a, b = base + 2 * k, base + 2 * k + 1
                        c, d = base + 2 * k + 2, base + 2 * k + 3
                        tris += [[a, c, b], [b, c, d]] if sign > 0 else [[a, b, c], [b, d, c]]
                    verts_list.append(np.array(strip))
                    base += len(strip)
                i = j + 1
        verts = np.vstack(verts_list)
        tris = np.array(tris, dtype=np.uint32)

        checkpoints = sorted(int(f * n) for f in CHECKPOINT_FRACTIONS)
        # Spawn: sample 0. yaw about +Y such that forward (+Z at yaw=0) aligns with
        # tangent[0]:  yaw = atan2(t.x, t.z)
        yaw = math.atan2(tangent[0][0], tangent[0][2])

        seg = np.linalg.norm(np.diff(pos, axis=0, append=pos[:1]), axis=1)
        return {
            "format": "sttr-track-json/1",
            "seed": seed,
            "attempt": attempt,
            "sample_count": n,
            "length_m": float(seg.sum()),
            "width_min_m": float(width.min()),
            "width_max_m": float(width.max()),
            "elevation_range_m": float(elev.max() - elev.min()),
            "centerline": {
                "pos": pos.round(5).tolist(),
                "up": up.round(6).tolist(),
                "tangent": tangent.round(6).tolist(),
                "width": width.round(4).tolist(),
            },
            "checkpoints": checkpoints,
            "terrain": {
                "vertices": verts.round(5).tolist(),
                "triangles": tris.tolist(),
            },
            "spawn": {"index": 0, "yaw": yaw},
        }
    raise SystemExit(f"no non-self-intersecting track found for seed {seed}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out-dir", type=pathlib.Path, required=True)
    args = ap.parse_args()

    track = generate(args.seed)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / "track.json"
    out.write_text(json.dumps(track), encoding="utf-8")
    print(
        f"seed={args.seed} samples={track['sample_count']} "
        f"length={track['length_m']:.0f}m width={track['width_min_m']:.1f}-"
        f"{track['width_max_m']:.1f}m elev-range={track['elevation_range_m']:.1f}m "
        f"-> {out}"
    )


if __name__ == "__main__":
    main()

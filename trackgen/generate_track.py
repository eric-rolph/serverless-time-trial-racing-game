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


def catmull_rom_closed(points: np.ndarray, samples_per_seg: int = 200) -> np.ndarray:
    """Uniform Catmull-Rom through a closed control polygon."""
    n = len(points)
    ts = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)
    out = []
    for i in range(n):
        p0, p1, p2, p3 = (points[(i + k - 1) % n] for k in range(4))
        t = ts[:, None]
        out.append(
            0.5
            * (
                2 * p1
                + (-p0 + p2) * t
                + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2
                + (-p0 + 3 * p1 - 3 * p2 + p3) * t**3
            )
        )
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
    for attempt in range(50):
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

        pos = resample_by_arclength(catmull_rom_closed(ctrl), SAMPLE_SPACING_M)
        n = len(pos)
        u = np.arange(n) / n

        # Elevation: loop-continuous harmonics, grade-clamped.
        elev = loop_harmonics(rng, u, freqs=(1, 3, 5), amps=(4.0, 2.5, 1.2))
        grade = np.max(np.abs(np.diff(elev, append=elev[:1]))) / SAMPLE_SPACING_M
        if grade > MAX_GRADE:
            elev *= MAX_GRADE / grade
        pos[:, 1] = elev

        width = 11.0 + loop_harmonics(rng, u, freqs=(2, 7), amps=(3.0, 1.5))
        width = np.maximum(width, MIN_WIDTH_M)

        if self_intersects(pos, width):
            continue

        # Frames: tangent from central differences, up = world-up perpendicularized.
        tangent = np.roll(pos, -1, axis=0) - np.roll(pos, 1, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        world_up = np.array([0.0, 1.0, 0.0])
        up = world_up - tangent * (tangent @ world_up)[:, None]
        up /= np.linalg.norm(up, axis=1, keepdims=True)
        side = np.cross(up, tangent)
        side /= np.linalg.norm(side, axis=1, keepdims=True)

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

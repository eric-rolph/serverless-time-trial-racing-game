#!/usr/bin/env python3
"""Apron regen gates (TERRAIN.md §1) — run against a generated track.json.

Checks (exit non-zero on any failure):

1. APRON SELF-INTERSECTION — no apron vertex may poke above another
   segment's road/shoulder surface. Methodology: for every apron vertex
   (owner sample i) and every centerline sample j that is NON-ADJACENT
   (wrap-aware circular index distance > 40 — the same window the generator's
   safe-width W uses), express the vertex in j's frame using the xz-normalized
   side/tangent. The vertex is "laterally within j's corridor" when
   |lateral| <= width_j/2 + 8 (road + shoulder — the full solid surface) and
   |longitudinal| <= 1.5 m (> half the 2.5 m sample spacing, so consecutive
   samples give overlapping full coverage along the road). For such pairs the
   segment's surface height at that lateral offset is reconstructed with the
   generator's own formulas (banked road plane inside width/2; linear
   shoulder ramp to -1.2 m at width/2 + 8), and the check asserts
   vertex_y < surface_y + 0.05. The signed worst margin (max of
   vertex_y - surface_y) is reported; a construction-correct apron keeps it
   well below zero because W = clamp(gap/2, 10, 45) plus the cosine falloff
   has dropped the apron far below any corridor it can reach.

2. PROP CLEARANCE — every prop's collision box stays >= 1.2 m clear of the
   ribbon (drivable surface). Methodology: the four yaw-rotated horizontal
   corners of each prop box are tested against the ribbon as a ruled surface:
   distance = min over all centerline segments of the xz point-to-segment
   distance, minus the linearly interpolated width/2 at the closest point.

3. APRON STATS — min/max/mean safe width W (from the generator's "apron"
   block) and the measured worst height discontinuity between the apron's
   outer row and the ground quad plane (must be < 0.05 m; by construction it
   is exactly APRON_ZFIGHT_EPS_M = 0.001 m — the stated z-fight choice: far
   row 1 mm below, quad exactly at ground_y).

Usage: python check_apron.py ../assets/tracks/circuit1/track.json
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

from generate_track import (
    APRON_ADJ_WINDOW,
    APRON_ROWS,
    APRON_ZFIGHT_EPS_M,
    SHOULDER_DROP_M,
    SHOULDER_WIDTH_M,
)

LON_WINDOW_M = 1.5  # > sample spacing / 2 -> full longitudinal coverage
POKE_TOL_M = 0.05
PROP_CLEAR_M = 1.2


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("track_json", type=pathlib.Path)
    args = ap.parse_args()
    track = json.loads(args.track_json.read_text(encoding="utf-8"))

    n = track["sample_count"]
    pos = np.array(track["centerline"]["pos"])
    up = np.array(track["centerline"]["up"])
    tangent = np.array(track["centerline"]["tangent"])
    width = np.array(track["centerline"]["width"])
    verts = np.array(track["terrain"]["vertices"])
    ground_y = track["ground_y"]
    side = np.cross(up, tangent)
    side /= np.linalg.norm(side, axis=1, keepdims=True)

    if len(verts) < 12 * n + 4 or "apron" not in track:
        sys.exit("no apron block found (need >= 12*S verts + ground quad)")
    apron = verts[4 * n : 12 * n].reshape(n, 2 * APRON_ROWS, 3)

    failures = []

    # ---- 0. mesh integrity: every triangle up-facing and non-degenerate ---
    # The kernel's off-corridor wheel raycast is BACK-FACE CULLED (physics/
    # NOTES.md): a folded/flipped apron patch is invisible to the down-ray and
    # silently reintroduces the cliff. Zero tolerance.
    tris = np.array(track["terrain"]["triangles"])
    ta, tb, tc = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    nrm = np.cross(tb - ta, tc - ta)
    area2 = np.linalg.norm(nrm, axis=1)
    degen = int((area2 < 1e-9).sum())
    ny_min = float((nrm[:, 1] / np.maximum(area2, 1e-12)).min())
    print(f"[0] mesh integrity: {len(tris)} tris, degenerate = {degen}, min normal.y = {ny_min:.4f} (assert > 0)")
    if degen or ny_min <= 0.0:
        failures.append(f"mesh integrity: {degen} degenerate tris, min normal.y {ny_min:.4f}")

    # ---- 1. apron self-intersection ------------------------------------
    side_h = np.hypot(side[:, 0], side[:, 2])            # xz shrink of side
    side_hat = side[:, [0, 2]] / side_h[:, None]
    tan_h = np.hypot(tangent[:, 0], tangent[:, 2])
    tan_hat = tangent[:, [0, 2]] / tan_h[:, None]
    half = width * 0.5

    av = apron.reshape(-1, 3)                             # (8n, 3)
    owner = np.repeat(np.arange(n), 2 * APRON_ROWS)
    worst = -np.inf
    worst_info = None
    pairs = 0
    chunk = 1024
    for c0 in range(0, len(av), chunk):
        v = av[c0 : c0 + chunk]
        ow = owner[c0 : c0 + chunk]
        d_idx = np.abs(ow[:, None] - np.arange(n)[None, :])
        nonadj = np.minimum(d_idx, n - d_idx) > APRON_ADJ_WINDOW
        dx = v[:, None, 0] - pos[None, :, 0]
        dz = v[:, None, 2] - pos[None, :, 2]
        lat = dx * side_hat[None, :, 0] + dz * side_hat[None, :, 1]
        lon = dx * tan_hat[None, :, 0] + dz * tan_hat[None, :, 1]
        in_cor = (
            nonadj
            & (np.abs(lat) <= half[None, :] + SHOULDER_WIDTH_M)
            & (np.abs(lon) <= LON_WINDOW_M)
        )
        if not in_cor.any():
            continue
        vi, ji = np.nonzero(in_cor)
        pairs += len(vi)
        t = lat[vi, ji] / side_h[ji]                      # 3D lateral param
        t = np.clip(t, -(half[ji] + SHOULDER_WIDTH_M), half[ji] + SHOULDER_WIDTH_M)
        road_y = pos[ji, 1] + side[ji, 1] * t
        over = np.maximum(0.0, np.abs(t) - half[ji])      # into the shoulder
        surf = road_y - SHOULDER_DROP_M * over / SHOULDER_WIDTH_M
        margin = v[vi, 1] - surf
        k = int(np.argmax(margin))
        if margin[k] > worst:
            worst = float(margin[k])
            worst_info = (c0 + vi[k], int(ji[k]))
        if np.any(margin >= POKE_TOL_M):
            bad = int(np.sum(margin >= POKE_TOL_M))
            failures.append(f"apron intersection: {bad} vertices >= {POKE_TOL_M} m above another corridor")
    print(
        f"[1] apron self-intersection: {pairs} vertex-in-other-corridor pairs, "
        f"worst margin (vertex_y - surface_y) = {worst:+.3f} m "
        f"(assert < +{POKE_TOL_M})"
        + (f" at apron vert {worst_info[0]} vs sample {worst_info[1]}" if worst_info else "")
    )
    if pairs == 0:
        print("    (no apron vertex reaches any non-adjacent corridor)")

    # ---- 2. prop clearance ----------------------------------------------
    props = track.get("props", [])
    seg_a = pos[:, [0, 2]]
    seg_b = np.roll(pos, -1, axis=0)[:, [0, 2]]
    seg_v = seg_b - seg_a
    seg_len2 = np.maximum((seg_v ** 2).sum(1), 1e-12)
    w_a, w_b = width, np.roll(width, -1)
    min_clear = np.inf
    for p in props:
        px, _, pz = p["pos"]
        hx, _, hz = p["half"]
        cy, sy = np.cos(p["yaw"]), np.sin(p["yaw"])
        for sx in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                dxl, dzl = sx * hx, sz * hz
                cx = px + dxl * cy + dzl * sy             # R_y(yaw) rotation
                cz = pz - dxl * sy + dzl * cy
                rel = np.array([cx, cz]) - seg_a
                tt = np.clip((rel * seg_v).sum(1) / seg_len2, 0.0, 1.0)
                closest = seg_a + seg_v * tt[:, None]
                dist = np.hypot(cx - closest[:, 0], cz - closest[:, 1])
                wq = (w_a * (1 - tt) + w_b * tt) * 0.5
                clear = float((dist - wq).min())
                min_clear = min(min_clear, clear)
    print(f"[2] prop clearance: {len(props)} props, min corner clearance from ribbon = {min_clear:.2f} m (assert >= {PROP_CLEAR_M})")
    if props and min_clear < PROP_CLEAR_M:
        failures.append(f"prop clearance {min_clear:.2f} m < {PROP_CLEAR_M} m")

    # ---- 3. apron stats ---------------------------------------------------
    a = track["apron"]
    far = apron[:, [APRON_ROWS - 1, 2 * APRON_ROWS - 1], 1]  # outer row y, both sides
    step = float(np.abs(far - ground_y).max())
    print(
        f"[3] apron width W: min={a['width_min_m']:.2f} m  max={a['width_max_m']:.2f} m  "
        f"mean={a['width_mean_m']:.2f} m"
    )
    print(
        f"    outer-edge height discontinuity vs ground quad: worst = {step:.4f} m "
        f"(assert < 0.05; construction = {APRON_ZFIGHT_EPS_M})"
    )
    if step >= 0.05:
        failures.append(f"apron outer-edge step {step:.4f} m >= 0.05 m")

    if failures:
        sys.exit("FAIL: " + "; ".join(failures))
    print("ALL APRON GATES PASS")


if __name__ == "__main__":
    main()

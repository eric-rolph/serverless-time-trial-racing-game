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
        # Banking gain must NOT saturate on near-straights (the old linear
        # gain 110 hit max bank at kappa=0.0006 ~ R=1.6km, so the road was
        # fully banked almost everywhere and FLIPPED 8 deg across every
        # curvature zero-crossing - measured 18 g vertical kicks). tanh
        # referenced to real-corner curvature + extra smoothing = gentle
        # straights, full bank only in genuine corners, soft transitions.
        kappa_b = np.convolve(np.tile(kappa, 3), np.ones(25) / 25.0, mode="same")[n : 2 * n]
        bank = -MAX_BANK_RAD * np.tanh(kappa_b / 0.015)
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


# =============================================================================
# Circuit composer — deterministic, archetype-based layouts (TRK1 v2 + props)
#
# Unlike generate() (seeded noise + regeneration), the composer builds a
# specific circuit from a layout list. Each archetype is a control-point
# emitter: it appends densely spaced centerline points along an exact
# curvature profile (linear-curvature "clothoid" ramps between arcs, so the
# heading is C1 and the lint gate cannot cusp). The composer closes the loop
# exactly by solving the lengths of the three long straights (2 position
# equations, 3 unknowns, min-norm), with the heading budget closed by
# construction (all section angles sum to exactly +-360 deg).
#
# Elevation is anchor-based: archetypes contribute (arc-length, height)
# anchors that are linearly interpolated around the loop and then
# Gaussian-smoothed (C1+), replacing the harmonics model for circuit mode.
# =============================================================================

DEG = math.pi / 180.0


def _clothoid(x: float, z: float, heading: float, k0: float, k1: float,
              length: float, step: float):
    """Integrate one linear-curvature segment.

    Heading is analytic (theta(s) = h + k0*s + (k1-k0)*s^2/2L), so the end
    heading is EXACT: h + (k0+k1)/2 * L. Position uses midpoint quadrature at
    `step` — sub-millimeter over any section at step=1.
    """
    n = max(2, int(math.ceil(length / step)))
    edges = np.linspace(0.0, length, n + 1)
    mids = 0.5 * (edges[:-1] + edges[1:])
    dk = (k1 - k0) / length
    th = heading + k0 * mids + 0.5 * dk * mids * mids
    ds = np.diff(edges)
    xs = x + np.cumsum(np.cos(th) * ds)
    zs = z + np.cumsum(np.sin(th) * ds)
    h_end = heading + 0.5 * (k0 + k1) * length
    return np.stack([xs, zs], axis=1), (float(xs[-1]), float(zs[-1]), h_end)


# --- archetype emitters ------------------------------------------------------
# Each returns a list of (kappa_start, kappa_end, length_m) segments whose
# signed heading change is exactly the requested angle. direction: +1 = left
# (kappa > 0), -1 = right.

def arch_straight(length_m: float):
    return [(0.0, 0.0, float(length_m))]


def arch_turn(direction: int, radius: float, angle_deg: float, ramp: float = 18.0):
    """Single constant-radius corner with clothoid entry/exit ramps."""
    k = direction / radius
    a = angle_deg * DEG
    hold = (a - abs(k) * ramp) / abs(k)  # ramps eat |k|*ramp of angle total
    if hold <= 1.0:
        raise ValueError(f"turn R{radius} {angle_deg}deg too short for ramp {ramp}")
    return [(0.0, k, ramp), (k, k, hold), (k, 0.0, ramp)]


def arch_chicane(first_dir: int, radius: float = 30.0, angle: float = 52.0,
                 link: float = 14.0, ramp: float = 13.0):
    """Tight direction flick: L-R (first_dir=+1) or R-L. Net heading 0."""
    return (arch_turn(first_dir, radius, angle, ramp)
            + arch_straight(link)
            + arch_turn(-first_dir, radius, angle, ramp))


def arch_esses(legs, link: float = 10.0, ramp: float = 24.0):
    """Flowing alternating sweepers. legs = [(direction, radius, angle_deg)].
    Net heading = sum of signed leg angles (the composer budgets it)."""
    segs = []
    for j, (d, r, a) in enumerate(legs):
        if j:
            segs += arch_straight(link)
        segs += arch_turn(d, r, a, ramp)
    return segs


def arch_double_apex(direction: int, angle_total: float = 150.0,
                     r_apex: float = 38.0, r_relief: float = 140.0,
                     ramp: float = 20.0, mid_ramp: float = 18.0,
                     apex_frac: float = 0.42):
    """One sweeping corner with TWO curvature peaks (~r_apex) and a mild
    relief (~r_relief) between — driven as a single arc. The apex holds are
    sized so the 15-sample smoothed |kappa| crosses KERB_KAPPA at both
    clipping points (kerbs paint there)."""
    ka = direction / r_apex
    kr = direction / r_relief
    ramp_angle = abs(ka) * ramp + (abs(ka) + abs(kr)) * mid_ramp
    rem = angle_total * DEG - ramp_angle
    if rem <= 0.05:
        raise ValueError("double_apex angle too small for its ramps")
    a_apex = apex_frac * rem
    a_relief = rem - 2.0 * a_apex
    return [
        (0.0, ka, ramp),
        (ka, ka, a_apex / abs(ka)),
        (ka, kr, mid_ramp),
        (kr, kr, a_relief / abs(kr)),
        (kr, ka, mid_ramp),
        (ka, ka, a_apex / abs(ka)),
        (ka, 0.0, ramp),
    ]


def arch_spiral(direction: int, r_in: float, r_out: float, angle_total: float,
                ramp_in: float = 20.0, ramp_out: float = 16.0):
    """Monotonic radius change over the corner (decreasing_radius when
    r_out < r_in, increasing_radius when r_out > r_in). Curvature is a single
    linear ramp between the two radii — monotonic and smooth, no cusps."""
    k_in = direction / r_in
    k_out = direction / r_out
    ramp_angle = 0.5 * abs(k_in) * ramp_in + 0.5 * abs(k_out) * ramp_out
    rem = angle_total * DEG - ramp_angle
    if rem <= 0.05:
        raise ValueError("spiral angle too small for its ramps")
    length = rem / (0.5 * (abs(k_in) + abs(k_out)))
    return [(0.0, k_in, ramp_in), (k_in, k_out, length), (k_out, 0.0, ramp_out)]


def arch_hairpin(direction: int, radius: float = 17.0, angle: float = 180.0,
                 ramp: float = 20.0):
    return arch_turn(direction, radius, angle, ramp)


def arch_corkscrew(first_dir: int, a1: float, a2: float, radius: float = 32.0,
                   link: float = 14.0, ramp: float = 14.0):
    """Tight direction change pair; the drama (blind crest + drop) comes from
    the elevation anchors the circuit recipe attaches to this section."""
    return (arch_turn(first_dir, radius, a1, ramp)
            + arch_straight(link)
            + arch_turn(-first_dir, radius, a2, ramp))


def arch_kink(direction: int, angle: float = 38.0, radius: float = 90.0,
              ramp: float = 16.0):
    return arch_turn(direction, radius, angle, ramp)


def arch_gentle90(direction: int, radius: float = 70.0, angle: float = 90.0,
                  ramp: float = 20.0):
    return arch_turn(direction, radius, angle, ramp)


# --- composer ----------------------------------------------------------------

def compose(layout, step: float = 1.0):
    """layout: list of (name, segs, tags). Returns (pts Nx2, end_pose,
    sections). Starts at origin heading +X; section s-ranges recorded."""
    x = z = h = s = 0.0
    pts = [np.array([[0.0, 0.0]])]
    sections = []
    for name, segs, tags in layout:
        s0, h0 = s, h
        for k0, k1, length in segs:
            p, (x, z, h) = _clothoid(x, z, h, k0, k1, length, step)
            pts.append(p)
            s += length
        sections.append(dict(name=name, s0=s0, s1=s, h0=h0, h1=h, **tags))
    return np.vstack(pts), (x, z, h), sections


def _solve_box_closure(A: np.ndarray, target_shift: np.ndarray, cur: np.ndarray,
                       lo: np.ndarray, hi: np.ndarray, prefer: np.ndarray):
    """Solve A @ dL = target_shift with cur+dL in [lo, hi], minimizing the
    distance of cur+dL to `prefer`. A is 2xN (N>=3). Returns dL or None.

    2 equations, N unknowns: the solution set is an (N-2)-dim affine space.
    We take the particular min-norm solution and search the null space on a
    coarse-to-fine grid for a feasible, target-preferring point — small N,
    fully vectorized, deterministic.
    """
    # POCS: alternate projection between the affine solution space
    # {dL : A dL = b} and the box [lo-cur, hi-cur], seeded at the point of
    # the affine space nearest to (prefer - cur). Converges to a point in the
    # intersection whenever it is non-empty.
    pinv = np.linalg.pinv(A)
    box_lo, box_hi = lo - cur, hi - cur

    def affine_project(x):
        return x - pinv @ (A @ x - target_shift)

    dL = affine_project(prefer - cur)
    for _ in range(3000):
        clipped = np.clip(dL, box_lo, box_hi)
        if np.allclose(clipped, dL, atol=1e-10):
            break
        dL = affine_project(clipped)
    # Land exactly inside the box; accept a small affine residual — the outer
    # close_loop iteration recomposes and re-solves, absorbing it.
    dL = np.clip(dL, box_lo, box_hi)
    if np.linalg.norm(A @ dL - target_shift) > 0.05:
        return None
    return dL


def close_loop(build_layout, lengths: dict, bounds: dict, iters: int = 5,
               step: float = 1.0, adjustable=("main", "brake", "back", "settle", "run_home")):
    """Solve the adjustable straight lengths so the composed loop returns
    exactly to the origin. The heading budget must already close (checked);
    position closure is linear in the straight lengths, so this converges in
    one pass + roundoff cleanup iterations. Straights are kept inside their
    bounds and as close to the requested lengths as feasible.
    """
    L = dict(lengths)
    prefer = np.array([lengths[k] for k in adjustable])
    lo = np.array([bounds[k][0] for k in adjustable])
    hi = np.array([bounds[k][1] for k in adjustable])
    pts = end = sections = None
    sec_of = {"main": "main_b", "brake": "brake", "back": "back",
              "settle": "settle", "link_hp": "link_hp", "link1": "link1",
              "run_home": "run_home"}
    for _ in range(iters):
        layout = build_layout(L)
        pts, end, sections = compose(layout, step=step)
        x, z, h = end
        turns = h / (2.0 * math.pi)
        if abs(abs(turns) - 1.0) > 1e-6:
            raise ValueError(f"heading budget does not close: {h / DEG:.3f} deg")
        by_name = {sec["name"]: sec for sec in sections}
        A = np.stack(
            [np.array([math.cos(by_name[sec_of[k]]["h0"]), math.sin(by_name[sec_of[k]]["h0"])])
             for k in adjustable], axis=1)  # 2 x N
        err = np.array([x, z])
        if np.linalg.norm(err) < 5e-4:
            break
        cur = np.array([L[k] for k in adjustable])
        dL = _solve_box_closure(A, -err, cur, lo, hi, prefer)
        if dL is None:
            raise ValueError("closure infeasible within straight bounds")
        for k, key in enumerate(adjustable):
            L[key] += float(dL[k])
    x, z, _ = end
    if math.hypot(x, z) > 5e-3:
        raise ValueError(f"closure residual too large: {math.hypot(x, z):.4f} m")
    for key in adjustable:
        klo, khi = bounds[key]
        if not klo - 1e-6 <= L[key] <= khi + 1e-6:
            raise ValueError(f"closure pushed {key} to {L[key]:.1f} m (bounds {klo}-{khi})")
    return L, pts, end, sections


def elevation_from_anchors(anchors, total_len: float, s_samples: np.ndarray,
                           sigma_m: float = 6.0):
    """Periodic linear interpolation through (s, y) anchors, then circular
    Gaussian smoothing — C1+ everywhere including the seam wrap."""
    anchors = sorted(anchors)
    s_a = np.array([a[0] for a in anchors])
    y_a = np.array([a[1] for a in anchors])
    # periodic pad
    s_ext = np.concatenate([s_a - total_len, s_a, s_a + total_len])
    y_ext = np.tile(y_a, 3)
    y = np.interp(s_samples, s_ext, y_ext)
    n = len(y)
    spacing = total_len / n
    half_w = max(1, int(round(3.0 * sigma_m / spacing)))
    xs = np.arange(-half_w, half_w + 1) * spacing
    kern = np.exp(-0.5 * (xs / sigma_m) ** 2)
    kern /= kern.sum()
    return np.convolve(np.tile(y, 3), kern, mode="same")[n:2 * n]


# --- circuit assembly (frames / banking / kerbs / ribbon / ground / props) ---

GROUND_DROP_M = 1.35  # shared formula: ground_y = min(centerline y) - 1.35
TIRE_HALF = (1.1, 0.55, 1.1)
ARMCO_HALF = (2.0, 0.38, 0.15)
ARMCO_PITCH = 4.15  # 4 m segment + 0.15 m gap, along the offset curve

# Apron (TERRAIN.md §1): 4 extra vertex rows per side beyond the shoulders,
# blending shoulder-edge height -> ground_y with a C1 cosine falloff — the
# collision mesh the wheels raycast off-corridor (kernel wave, TERRAIN.md §2).
# CIRCUIT/v2 PATH ONLY: generate() and its ribbon are frozen (dev5 regen
# byte-identity is a release gate).
APRON_ROWS = 4
APRON_W_MIN_M = 10.0
APRON_W_MAX_M = 45.0
APRON_ADJ_WINDOW = 40  # samples: circular index distance <= 40 = "adjacent"
APRON_SMOOTH_WINDOW = 15  # moving-average window for W along the centerline
# Geometric row spacing (ratio 2): rows cluster near the shoulder where the
# cosine profile bends; gaps of W*(1,2,4,8)/15.
APRON_ROW_FRACTIONS = (1.0 / 15.0, 3.0 / 15.0, 7.0 / 15.0, 1.0)
# Z-fight guard (chosen option): the far apron row sits 1 mm BELOW ground_y;
# the ground quad stays exactly at ground_y as today. The 1 mm step is the
# "worst remaining height discontinuity" at the apron outer edge.
APRON_ZFIGHT_EPS_M = 0.001


APRON_FOLD_MARGIN = 0.8  # fraction of the fold-limit radius the apron may use
APRON_W_FLOOR_M = 1.0  # absolute floor (keeps the 4 rows non-degenerate)


def apron_safe_width(pos: np.ndarray, side: np.ndarray, half: np.ndarray):
    """Per-sample, per-side safe apron width W (n x 2, columns = left/right).

    gap = horizontal distance from this sample's shoulder edge to the nearest
    NON-ADJACENT sample's shoulder edge (either side, wrap-aware; adjacent =
    circular index distance <= APRON_ADJ_WINDOW). W = clamp(gap/2, 10, 45),
    then circularly moving-averaged (~15 samples) so apron edges don't zigzag.

    FOLD CAP (self-intersection guard the gap term cannot see): on the INSIDE
    of a corner the per-sample offset rays converge at the local center of
    curvature — any lateral offset beyond 1/|kappa| from the centerline sweeps
    retrograde and the ruled surface folds over itself (flipped, back-face-
    culled triangles the wheel raycast would miss). On the converging side W
    is therefore additionally capped at APRON_FOLD_MARGIN * (1/|kappa_max| -
    lat_edge), where |kappa_max| is the worst raw per-sample curvature within
    the smoothing half-window. This cap MAY undercut the 10 m clamp floor
    (e.g. the hairpin inside, edge radius ~2 m — the pre-apron skirt there was
    a vertical wall; a steep narrow apron strictly improves it). The cap is
    re-applied after smoothing so the moving average cannot lift W back over
    the fold limit.
    """
    n = len(pos)
    edge_l = (pos - side * (half + SHOULDER_WIDTH_M))[:, [0, 2]]
    edge_r = (pos + side * (half + SHOULDER_WIDTH_M))[:, [0, 2]]
    pts = np.stack([edge_l, edge_r], axis=1).reshape(-1, 2)  # (2n,2): L0,R0,L1,R1,...
    owner = np.repeat(np.arange(n), 2)
    diff = np.abs(owner[:, None] - owner[None, :])
    circ = np.minimum(diff, n - diff)
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    d[circ <= APRON_ADJ_WINDOW] = np.inf
    gap = d.min(axis=1).reshape(n, 2)

    # Fold cap. Raw (unsmoothed) curvature magnitude + the side it converges
    # toward (sign convention-free: dot the tangent derivative with side).
    tangent = np.roll(pos, -1, axis=0) - np.roll(pos, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    t_next = np.roll(tangent, -1, axis=0)
    kmag = np.abs(tangent[:, 0] * t_next[:, 2] - tangent[:, 2] * t_next[:, 0]) / SAMPLE_SPACING_M
    dtan = np.roll(tangent, -1, axis=0) - np.roll(tangent, 1, axis=0)
    conv_r = (dtan[:, 0] * side[:, 0] + dtan[:, 2] * side[:, 2]) > 0.0  # curves toward +side
    k_l = np.where(~conv_r, kmag, 0.0)  # curvature converging on the left col
    k_r = np.where(conv_r, kmag, 0.0)
    hw = APRON_SMOOTH_WINDOW // 2
    for shift in range(-hw, hw + 1):
        if shift:
            k_l = np.maximum(k_l, np.roll(np.where(~conv_r, kmag, 0.0), shift))
            k_r = np.maximum(k_r, np.roll(np.where(conv_r, kmag, 0.0), shift))
    lat_edge = half[:, 0] + SHOULDER_WIDTH_M
    with np.errstate(divide="ignore"):
        cap = np.stack(
            [
                np.where(k_l > 1e-9, APRON_FOLD_MARGIN * (1.0 / np.maximum(k_l, 1e-9) - lat_edge), np.inf),
                np.where(k_r > 1e-9, APRON_FOLD_MARGIN * (1.0 / np.maximum(k_r, 1e-9) - lat_edge), np.inf),
            ],
            axis=1,
        )
    cap = np.maximum(cap, APRON_W_FLOOR_M)

    w = np.clip(gap * 0.5, APRON_W_MIN_M, APRON_W_MAX_M)
    w = np.minimum(w, cap)
    kern = np.ones(APRON_SMOOTH_WINDOW) / APRON_SMOOTH_WINDOW
    for c in range(2):
        w[:, c] = np.convolve(np.tile(w[:, c], 3), kern, mode="same")[n:2 * n]
    w = np.minimum(w, cap)  # smoothing must not lift W back over the fold limit
    return np.maximum(w, APRON_W_FLOOR_M)


def _shoulder_surface(pos_i, side_i, half_w: float, lateral: float):
    """Exact mesh-shoulder surface point at |lateral| in [half, half+8] —
    linear interp between the edge row and the outer row (same math as the
    ribbon vertices), so props sit ON the collision surface."""
    over = max(0.0, lateral - half_w)
    p = pos_i + side_i * lateral
    return p - np.array([0.0, 1.0, 0.0]) * (SHOULDER_DROP_M * over / SHOULDER_WIDTH_M)


def _place_tire_row(props, pos, side, tan, width, i0, i1, sign, count, lateral_want=11.0):
    """Row of tire stacks along the outside edge, `count` stacks with ~1.5 m
    gaps (pitch = 2*1.1 + 1.5 = 3.7 m along the edge)."""
    n = len(pos)
    pitch = 2.0 * TIRE_HALF[0] + 1.5
    span = (i1 - i0) % n
    step = max(1, int(round(pitch / SAMPLE_SPACING_M)))
    idxs = [(i0 + int(round(j * step))) % n for j in range(count)]
    for i in idxs:
        half_w = width[i] * 0.5
        lat = min(max(lateral_want, half_w + 3.2), half_w + 6.5)
        p = _shoulder_surface(pos[i], side[i] * sign, half_w, lat)
        yaw = math.atan2(tan[i][0], tan[i][2])
        props.append(dict(type=0, pos=[float(p[0]), float(p[1] + TIRE_HALF[1] - 0.1), float(p[2])],
                          yaw=float(yaw), half=list(TIRE_HALF)))


def _place_armco_run(props, pos, side, width, i0, i1, sign, lateral_off=3.0):
    """Continuous Armco: 4 m segments pitched along the OFFSET curve (outer
    edge + lateral_off), yaw following the local edge direction."""
    n = len(pos)
    span = (i1 - i0) % n
    offset_pts = []
    for j in range(span + 1):
        i = (i0 + j) % n
        half_w = width[i] * 0.5
        lat = half_w + lateral_off
        offset_pts.append(_shoulder_surface(pos[i], side[i] * sign, half_w, lat))
    offset_pts = np.array(offset_pts)
    d = np.diff(offset_pts[:, [0, 2]], axis=0)
    seg_len = np.hypot(d[:, 0], d[:, 1])
    s_off = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = s_off[-1]
    n_seg = int(total // ARMCO_PITCH)
    for j in range(n_seg):
        sc = (j + 0.5) * ARMCO_PITCH
        if sc >= total:
            break
        k = int(np.searchsorted(s_off, sc)) - 1
        k = min(max(k, 0), len(seg_len) - 1)
        t = (sc - s_off[k]) / max(seg_len[k], 1e-9)
        p = offset_pts[k] * (1 - t) + offset_pts[k + 1] * t
        dxz = offset_pts[k + 1] - offset_pts[k]
        yaw = math.atan2(-dxz[2], dxz[0])  # local +X (long axis) -> edge dir
        props.append(dict(type=1, pos=[float(p[0]), float(p[1] + ARMCO_HALF[1] - 0.05), float(p[2])],
                          yaw=float(yaw), half=list(ARMCO_HALF)))


def assemble_circuit(pos: np.ndarray, sections, seed: int) -> dict:
    """Frames, banking, kerbs, terrain ribbon, ground quad, props — the same
    pipeline generate() uses, driven by composed geometry + section tags."""
    n = len(pos)
    spacing = None  # actual, computed below

    seg = np.linalg.norm(np.diff(pos[:, [0, 2]], axis=0, append=pos[:1, [0, 2]]), axis=1)
    total_len_flat = float(seg.sum())
    spacing = total_len_flat / n

    # Frames (identical construction to generate()).
    tangent = np.roll(pos, -1, axis=0) - np.roll(pos, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    world_up = np.array([0.0, 1.0, 0.0])
    up = world_up - tangent * (tangent @ world_up)[:, None]
    up /= np.linalg.norm(up, axis=1, keepdims=True)
    side = np.cross(up, tangent)
    side /= np.linalg.norm(side, axis=1, keepdims=True)

    t_next = np.roll(tangent, -1, axis=0)
    kappa = (tangent[:, 0] * t_next[:, 2] - tangent[:, 2] * t_next[:, 0]) / SAMPLE_SPACING_M
    kernel = np.ones(CURVATURE_SMOOTH_WINDOW) / CURVATURE_SMOOTH_WINDOW
    kappa = np.convolve(np.tile(kappa, 3), kernel, mode="same")[n:2 * n]

    # Width choreography: corners breathe wider, straights stay lean.
    width = 10.0 + 4.0 * np.minimum(1.0, np.abs(kappa) / 0.03)
    width = np.maximum(width, MIN_WIDTH_M)

    # Banking (identical formula to generate()).
    kappa_b = np.convolve(np.tile(kappa, 3), np.ones(25) / 25.0, mode="same")[n:2 * n]
    bank = -MAX_BANK_RAD * np.tanh(kappa_b / 0.015)
    cos_b, sin_b = np.cos(bank)[:, None], np.sin(bank)[:, None]
    up_banked = up * cos_b + side * sin_b
    side = np.cross(up_banked, tangent)
    side /= np.linalg.norm(side, axis=1, keepdims=True)
    up = up_banked / np.linalg.norm(up_banked, axis=1, keepdims=True)

    # Terrain ribbon.
    half = (width * 0.5)[:, None]
    rows = [
        pos - side * (half + SHOULDER_WIDTH_M) - world_up * SHOULDER_DROP_M,
        pos - side * half,
        pos + side * half,
        pos + side * (half + SHOULDER_WIDTH_M) - world_up * SHOULDER_DROP_M,
    ]
    verts = np.stack(rows, axis=1).reshape(-1, 3)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        for r in range(3):
            a, b = i * 4 + r, i * 4 + r + 1
            c, d = j * 4 + r, j * 4 + r + 1
            tris.append([a, c, b])
            tris.append([b, c, d])

    # Apron (TERRAIN.md §1 — kills the cliff): 4 vertex rows per side beyond
    # the shoulder edge, height h(d) = ground_y + (h_edge - ground_y) *
    # 0.5*(1 + cos(pi*d/W)) — C1 at both ends. W is the per-sample per-side
    # safe width so aprons in tight loop interiors meet mid-gap (valley)
    # instead of crossing the opposing corridor. Vertex layout: the apron
    # block sits at [4n, 12n) — v(i, side, row) = 4n + i*8 + side*4 + row
    # (side 0 = left, 1 = right; row 0..3 outward) — so the client can find
    # it deterministically; kerb verts append AFTER the apron block.
    ground_y = float(pos[:, 1].min()) - GROUND_DROP_M
    w_safe = apron_safe_width(pos, side, half)
    dir_xz = side[:, [0, 2]] / np.linalg.norm(side[:, [0, 2]], axis=1, keepdims=True)
    fr = np.array(APRON_ROW_FRACTIONS)
    blend = 0.5 * (1.0 + np.cos(math.pi * fr))
    apron = np.empty((n, 2 * APRON_ROWS, 3))
    for scol, sgn in ((0, -1.0), (1, 1.0)):
        edge = rows[0] if scol == 0 else rows[3]  # shoulder outer row (n,3)
        for k in range(APRON_ROWS):
            lat = fr[k] * w_safe[:, scol]  # horizontal distance beyond the edge
            if k == APRON_ROWS - 1:
                y = np.full(n, ground_y - APRON_ZFIGHT_EPS_M)  # z-fight guard
            else:
                y = ground_y + (edge[:, 1] - ground_y) * blend[k]
            col = scol * APRON_ROWS + k
            apron[:, col, 0] = edge[:, 0] + sgn * dir_xz[:, 0] * lat
            apron[:, col, 1] = y
            apron[:, col, 2] = edge[:, 2] + sgn * dir_xz[:, 1] * lat
    apron_base = len(verts)  # == 4n
    verts = np.vstack([verts, apron.reshape(-1, 3)])
    for i in range(n):
        j = (i + 1) % n
        for scol, sgn in ((0, -1.0), (1, 1.0)):
            outer_row = 0 if scol == 0 else 3
            prev_i, prev_j = i * 4 + outer_row, j * 4 + outer_row
            for k in range(APRON_ROWS):
                a, c = prev_i, prev_j
                b = apron_base + i * 8 + scol * 4 + k
                d = apron_base + j * 8 + scol * 4 + k
                # a/c = inner pair, b/d = outer pair; same winding rule as the
                # kerbs so face normals point +Y on both sides.
                tris += [[a, c, b], [b, c, d]] if sgn > 0 else [[a, b, c], [b, d, c]]
                prev_i, prev_j = b, d
    apron_stats = {
        "width_min_m": float(w_safe.min()),
        "width_max_m": float(w_safe.max()),
        "width_mean_m": float(w_safe.mean()),
        "outer_edge_step_m": APRON_ZFIGHT_EPS_M,
    }

    # Kerbs (identical to generate(): tooth 0.02 — MUST stay in sync with
    # road.c's analytic band; "heavy" kerbs are made by sustaining |kappa|
    # deep past KERB_KAPPA so the band paints long and fully).
    kerb_zone = np.abs(kappa) > KERB_KAPPA
    verts_list = [verts]
    base = len(verts)
    for sign in (-1.0, 1.0):
        i = 0
        while i < n:
            if not (kerb_zone[i] and kerb_zone[(i + 1) % n]):
                i += 1
                continue
            j = i
            while j + 1 < n and kerb_zone[(j + 1) % n]:
                j += 1
            if j - i >= 3:
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

    # Ground quad: the solid landscape, baked into the COLLISION mesh at
    # ground_y (shared formula with road.c fallback and ambience.js disc;
    # computed above for the apron). The quad stays exactly at ground_y — the
    # apron's far row sits APRON_ZFIGHT_EPS_M below it (stated z-fight choice).
    centroid = pos[:, [0, 2]].mean(axis=0)
    max_r = float(np.max(np.hypot(pos[:, 0] - centroid[0], pos[:, 2] - centroid[1])))
    h_side = max_r + 120.0
    gbase = len(verts)
    gverts = np.array([
        [centroid[0] - h_side, ground_y, centroid[1] - h_side],
        [centroid[0] + h_side, ground_y, centroid[1] - h_side],
        [centroid[0] - h_side, ground_y, centroid[1] + h_side],
        [centroid[0] + h_side, ground_y, centroid[1] + h_side],
    ])
    verts = np.vstack([verts, gverts])
    # wound so face normals point +Y: (v0, v2, v1) and (v1, v2, v3)
    tris += [[gbase + 0, gbase + 2, gbase + 1], [gbase + 1, gbase + 2, gbase + 3]]
    tris = np.array(tris, dtype=np.uint32)

    # --- props from section tags ---
    def sec_range(name):
        sec = next(s for s in sections if s["name"] == name)
        return int(sec["s0"] / spacing) % n, int(sec["s1"] / spacing) % n

    props = []
    for sec in sections:
        i0, i1 = int(sec["s0"] / spacing) % n, int(sec["s1"] / spacing) % n
        span = (i1 - i0) % n
        for rule in sec.get("props", []):
            kind = rule["kind"]
            # rule f0/f1: fraction of the section's sample range
            a = (i0 + int(rule.get("f0", 0.0) * span)) % n
            b = (i0 + int(rule.get("f1", 1.0) * span)) % n
            sign = float(rule["side"])
            if kind == "tires":
                _place_tire_row(props, pos, side, tangent, width, a, b, sign,
                                rule.get("count", 6), rule.get("lateral", 11.0))
            elif kind == "armco":
                _place_armco_run(props, pos, side, width, a, b, sign,
                                 rule.get("lateral_off", 3.0))
    if len(props) > 4096:
        raise ValueError(f"prop budget blown: {len(props)}")

    checkpoints = sorted(int(f * n) for f in CHECKPOINT_FRACTIONS)
    yaw = math.atan2(tangent[0][0], tangent[0][2])
    seg3 = np.linalg.norm(np.diff(pos, axis=0, append=pos[:1]), axis=1)

    return {
        "format": "sttr-track-json/1",
        "seed": seed,
        "attempt": 0,
        "sample_count": n,
        "length_m": float(seg3.sum()),
        "width_min_m": float(width.min()),
        "width_max_m": float(width.max()),
        "elevation_range_m": float(pos[:, 1].max() - pos[:, 1].min()),
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
        "ground_y": ground_y,
        "apron": apron_stats,
        "props": props,
        "sections": [
            {"name": s["name"], "s0": round(s["s0"], 2), "s1": round(s["s1"], 2),
             "i0": int(s["s0"] / spacing) % n, "i1": int(s["s1"] / spacing) % n}
            for s in sections
        ],
    }


# --- circuit1 recipe ---------------------------------------------------------
# Heading budget (deg, + = left): chicane 0, glue1 +48, esses +55 net,
# glue2 +57, double-apex -150, dec-radius +140, hairpin +180, inc-radius -110,
# corkscrew +5 net, kink +45, gentle90 +90  ==>  sum exactly +360.
# The glue sweepers (R>=110, gentle) climb the east side so the braking
# straight heads WNW and the back straight SW — that is what lets a loop with
# this mandated corner sequence close at all (see design notes in the report).
# Position closure is solved at build time over five straights: main (b+a),
# brake, back, settle (post-corkscrew), run_home (the south descent between
# kink and gentle90).

CIRCUIT1 = {
    "seed": 20260709,
    "chicane": {"first": +1, "radius": 30.0, "angle": 52.0, "link": 12.0},
    "glue1": {"direction": +1, "radius": 88.0, "angle": 60.0},
    "esses_legs": [(+1, 114.0, 41.0), (-1, 124.0, 23.5),
                   (+1, 110.0, 41.0), (-1, 118.0, 23.5)],  # net +35
    "glue2": {"direction": +1, "radius": 78.0, "angle": 85.0},
    "da": {"direction": -1, "angle": 150.0, "r_relief": 125.0},
    "dr": {"direction": +1, "angle": 140.0, "r_in": 90.0, "r_out": 35.0},
    "hairpin": {"direction": +1, "radius": 17.0},
    "ir": {"direction": -1, "angle": 125.0, "r_in": 40.0, "r_out": 120.0},
    "corkscrew": {"first": +1, "a1": 53.0, "a2": 43.0, "radius": 32.0},
    "kink": {"direction": +1, "angle": 35.0, "radius": 85.0},
    "g90": {"direction": +1, "radius": 52.0},
    "straights": {"main": 620.0, "brake": 240.0, "back": 410.0,
                  "link1": 20.0, "link_da": 16.0, "link_hp": 18.0,
                  "settle": 90.0, "run_home": 55.0},
}

CIRCUIT1_BOUNDS = {"main": (545.0, 700.0), "brake": (200.0, 330.0),
                   "back": (370.0, 500.0), "settle": (50.0, 260.0),
                   "run_home": (30.0, 260.0)}

CIRCUIT1_ADJUSTABLE = ("main", "brake", "back", "settle", "run_home")


def circuit1_layout(P, L):
    """Build the layout list for the given straight lengths L."""
    main_b = L["main"] * 0.5
    main_a = L["main"] - main_b
    c = P["chicane"]
    return [
        ("main_b", arch_straight(main_b), {}),
        ("chicane", arch_chicane(c["first"], c["radius"], c["angle"], c["link"]),
         {"props": [
             {"kind": "tires", "side": -c["first"], "f0": 0.28, "f1": 0.50, "count": 6},
             {"kind": "tires", "side": c["first"], "f0": 0.78, "f1": 1.00, "count": 6},
         ]}),
        ("link1", arch_straight(L["link1"]), {}),
        ("glue1", arch_turn(P["glue1"]["direction"], P["glue1"]["radius"],
                            P["glue1"]["angle"], ramp=22.0), {}),
        ("esses", arch_esses(P["esses_legs"]), {}),
        ("glue2", arch_turn(P["glue2"]["direction"], P["glue2"]["radius"],
                            P["glue2"]["angle"], ramp=22.0), {}),
        ("double_apex", arch_double_apex(P["da"]["direction"], P["da"]["angle"],
                                         r_relief=P["da"].get("r_relief", 140.0)),
         {"props": [{"kind": "armco", "side": -P["da"]["direction"]}]}),
        ("link_da", arch_straight(L["link_da"]), {}),
        ("dec_radius", arch_spiral(P["dr"]["direction"], P["dr"]["r_in"],
                                   P["dr"]["r_out"], P["dr"]["angle"]),
         {"props": [{"kind": "armco", "side": -P["dr"]["direction"]}]}),
        ("brake", arch_straight(L["brake"]), {}),
        ("hairpin", arch_hairpin(P["hairpin"]["direction"], P["hairpin"]["radius"]),
         {"props": [
             {"kind": "tires", "side": -P["hairpin"]["direction"], "f0": 0.05, "f1": 0.45, "count": 7},
             {"kind": "tires", "side": -P["hairpin"]["direction"], "f0": 0.55, "f1": 0.95, "count": 7},
         ]}),
        ("link_hp", arch_straight(L["link_hp"]), {}),
        ("inc_radius", arch_spiral(P["ir"]["direction"], P["ir"]["r_in"],
                                   P["ir"]["r_out"], P["ir"]["angle"]), {}),
        ("back", arch_straight(L["back"]), {}),
        ("corkscrew", arch_corkscrew(P["corkscrew"]["first"], P["corkscrew"]["a1"],
                                     P["corkscrew"]["a2"], P["corkscrew"]["radius"]),
         {"props": [
             {"kind": "armco", "side": +1},
             {"kind": "armco", "side": -1},
             {"kind": "tires", "side": -P["corkscrew"]["first"], "f0": 0.08, "f1": 0.40, "count": 5},
             {"kind": "tires", "side": P["corkscrew"]["first"], "f0": 0.60, "f1": 0.95, "count": 5},
         ]}),
        ("settle", arch_straight(L["settle"]), {}),
        ("kink", arch_kink(P["kink"]["direction"], P["kink"]["angle"],
                           P["kink"].get("radius", 90.0)), {}),
        ("run_home", arch_straight(L["run_home"]), {}),
        ("g90", arch_gentle90(P["g90"]["direction"], P["g90"].get("radius", 70.0)), {}),
        ("main_a", arch_straight(main_a), {}),
    ]


def circuit1_elevation(sections, total_len):
    """Anchor table: rolling +-6 m at <=4% grade everywhere, except the
    corkscrew: +2 m blind-crest rise at turn-in then a 12 m drop (peaks ~8%,
    steepest mid-chicane) settling over ~120 m, then a gentle climb home."""
    S = {sec["name"]: sec for sec in sections}

    def mid(name, f=0.5):
        sec = S[name]
        return sec["s0"] * (1.0 - f) + sec["s1"] * f

    a = []
    a.append((0.0, 0.0))                                   # seam
    a.append((mid("main_b", 0.6), 1.0))                    # gentle roll <=1.5%
    a.append((S["chicane"]["s0"], 0.6))
    a.append((S["chicane"]["s1"], 0.8))
    a.append((mid("esses", 0.5), 2.2))
    a.append((S["esses"]["s1"], 3.0))
    a.append((mid("double_apex", 0.6), 3.8))
    a.append((S["dec_radius"]["s0"], 2.5))
    a.append((S["dec_radius"]["s1"], 0.5))
    a.append((mid("brake", 0.6), -0.5))
    a.append((S["hairpin"]["s0"], -1.5))
    a.append((S["hairpin"]["s1"], -1.5))
    a.append((S["inc_radius"]["s1"], -0.5))
    a.append((mid("back", 0.5), 1.5))
    a.append((S["back"]["s1"] - 80.0, 4.0))                # crest approach
    t0 = S["corkscrew"]["s0"]
    a.append((t0, 6.0))                                    # blind crest AT turn-in
    a.append((t0 + 28.0, 5.3))
    a.append((t0 + 58.0, 2.9))                             # ~8%
    a.append((t0 + 88.0, 0.5))                             # ~8% (mid-chicane)
    a.append((t0 + 118.0, -1.9))                           # ~8%
    a.append((t0 + 150.0, -4.0))
    settle_end = min(t0 + 240.0, S["kink"]["s0"] - 5.0)
    a.append((min(t0 + 190.0, settle_end - 25.0), -5.4))
    a.append((settle_end, -6.0))                           # 12 m total drop
    a.append((S["kink"]["s1"], -5.2))
    a.append((mid("run_home", 0.7), -4.6))
    a.append((S["g90"]["s1"], -3.4))                       # climb back <=2%
    return a


def generate_circuit1() -> dict:
    P = dict(CIRCUIT1)
    L0 = dict(P["straights"])

    def build(L):
        return circuit1_layout(P, L)

    L, dense, end, sections = close_loop(build, L0, CIRCUIT1_BOUNDS, step=1.0,
                                         adjustable=CIRCUIT1_ADJUSTABLE)

    pos = resample_by_arclength(
        np.stack([dense[:, 0], np.zeros(len(dense)), dense[:, 1]], axis=1),
        SAMPLE_SPACING_M,
    )
    n = len(pos)
    total_len = sections[-1]["s1"]

    # Lint gate (same as generate()).
    seg_v = np.roll(pos, -1, axis=0) - pos
    seg_v /= np.linalg.norm(seg_v, axis=1, keepdims=True)
    turn = np.degrees(np.arccos(np.clip((seg_v * np.roll(seg_v, 1, axis=0)).sum(1), -1.0, 1.0)))
    if turn.max() > 9.0:
        raise SystemExit(f"lint gate failed: worst per-sample turn {turn.max():.2f} deg")

    # Elevation from anchors, C1-smoothed; grade audit.
    s_samples = np.arange(n) * (total_len / n)
    elev = elevation_from_anchors(circuit1_elevation(sections, total_len), total_len, s_samples)
    pos[:, 1] = elev

    grade = np.abs(np.diff(elev, append=elev[:1])) / (total_len / n)
    cs = next(s for s in sections if s["name"] == "corkscrew")
    ck0 = int((cs["s0"] - 30) / (total_len / n))
    ck1 = int((cs["s1"] + 260) / (total_len / n))
    in_ck = np.zeros(n, dtype=bool)
    in_ck[ck0:min(ck1, n)] = True
    g_out = grade[~in_ck].max()
    g_ck = grade[in_ck].max() if in_ck.any() else 0.0
    if g_out > 0.045:
        raise SystemExit(f"grade gate failed outside corkscrew: {g_out * 100:.1f}%")
    if g_ck > 0.095:
        raise SystemExit(f"corkscrew grade too steep: {g_ck * 100:.1f}%")

    track = assemble_circuit(pos, sections, P["seed"])

    width = np.array(track["centerline"]["width"])
    if self_intersects(pos, width):
        raise SystemExit("circuit self-intersects")

    track["circuit"] = {
        "name": "circuit1",
        "straights_solved": {k: round(v, 2) for k, v in L.items()},
        "lint_worst_turn_deg": float(turn.max()),
        "grade_max_outside_corkscrew": float(g_out),
        "grade_max_corkscrew": float(g_ck),
    }
    return track


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--circuit", type=str, default=None, choices=["circuit1"])
    ap.add_argument("--out-dir", type=pathlib.Path, required=True)
    args = ap.parse_args()

    if args.circuit == "circuit1":
        track = generate_circuit1()
    else:
        if args.seed is None:
            raise SystemExit("--seed required unless --circuit is given")
        track = generate(args.seed)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / "track.json"
    out.write_text(json.dumps(track), encoding="utf-8")
    print(
        f"seed={track['seed']} samples={track['sample_count']} "
        f"length={track['length_m']:.0f}m width={track['width_min_m']:.1f}-"
        f"{track['width_max_m']:.1f}m elev-range={track['elevation_range_m']:.1f}m "
        f"props={len(track.get('props', []))} -> {out}"
    )
    if "apron" in track:
        a = track["apron"]
        print(
            f"apron: W min={a['width_min_m']:.2f}m max={a['width_max_m']:.2f}m "
            f"mean={a['width_mean_m']:.2f}m outer-edge step={a['outer_edge_step_m']:.3f}m"
        )


if __name__ == "__main__":
    main()

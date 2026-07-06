#!/usr/bin/env python3
"""Pack track.json into the TRK1 binary format (docs/CONTRACTS.md §8).

Little-endian throughout. Prints the FNV-1a-64 track hash (the identity used in
LAPLOG headers, KV keys, and URLs).

Usage: python pack_track.py ../assets/tracks/dev/track.json
"""

from __future__ import annotations

import argparse
import json
import pathlib
import struct

FNV_OFFSET = 0xCBF29CE484222325
FNV_PRIME = 0x100000001B3


def fnv1a64(data: bytes) -> int:
    h = FNV_OFFSET
    for b in data:
        h ^= b
        h = (h * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF
    return h


def pack(track: dict) -> bytes:
    cl = track["centerline"]
    pos, up, tan, width = cl["pos"], cl["up"], cl["tangent"], cl["width"]
    verts = track["terrain"]["vertices"]
    tris = track["terrain"]["triangles"]
    checkpoints = track["checkpoints"]
    s, v, t, c = len(pos), len(verts), len(tris), len(checkpoints)

    out = bytearray()
    out += b"STRK"
    out += struct.pack("<HH", 1, c)
    out += struct.pack("<Q", track["seed"] & 0xFFFFFFFFFFFFFFFF)
    out += struct.pack("<III", s, v, t)
    for i in range(s):
        out += struct.pack("<10f", *pos[i], *up[i], *tan[i], width[i])
    out += struct.pack(f"<{c}I", *checkpoints)
    for vert in verts:
        out += struct.pack("<3f", *vert)
    for tri in tris:
        out += struct.pack("<3I", *tri)
    out += struct.pack("<Iff", track["spawn"]["index"], track["spawn"]["yaw"], 0.0)
    return bytes(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("track_json", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path, default=None)
    args = ap.parse_args()

    track = json.loads(args.track_json.read_text(encoding="utf-8"))
    blob = pack(track)
    out = args.out or args.track_json.with_name("track.bin")
    out.write_bytes(blob)

    h = fnv1a64(blob)
    meta = {
        "track_hash": f"{h:016x}",
        "seed": track["seed"],
        "bytes": len(blob),
        "sample_count": track["sample_count"],
        "length_m": track["length_m"],
    }
    out.with_name("track.meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"packed {len(blob)} bytes -> {out}  track_hash={h:016x}")


if __name__ == "__main__":
    main()

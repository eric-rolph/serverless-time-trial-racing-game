#!/usr/bin/env bash
# build_wasm.sh — build physics/out/sim.wasm, the single deterministic physics
# authority (CONTRACTS §1, ADR-001, ADR-008).
#
# Constraints enforced here:
#   * -O2, -ffp-contract=off, never any fast-math flag
#   * -sSTANDALONE_WASM=1 --no-entry (reactor module, no JS glue)
#   * scalar math (BOX3D_DISABLE_SIMD → no wasm SIMD128)
#   * -DNDEBUG turns B3_ASSERT into a no-op (see box3d/include/box3d/base.h)
#   * box3d sources compiled directly (list mirrors BOX3D_SOURCE_FILES in
#     physics/box3d/src/CMakeLists.txt) — no emcmake needed
#
# Usage: ./build_wasm.sh   (from physics/; EMCC env var overrides discovery)

set -euo pipefail
cd "$(dirname "$0")"

# --- locate emcc ---
if [ -n "${EMCC:-}" ]; then
    :
elif command -v emcc >/dev/null 2>&1; then
    EMCC=emcc
elif [ -f "$HOME/emsdk/upstream/emscripten/emcc" ]; then
    EMCC="$HOME/emsdk/upstream/emscripten/emcc"
else
    echo "error: emcc not found (install emsdk or set EMCC)" >&2
    exit 1
fi
echo "using EMCC=$EMCC"

mkdir -p out

# --- box3d sources: keep in sync with BOX3D_SOURCE_FILES in box3d/src/CMakeLists.txt ---
BOX3D_SRC=(
    aabb.c arena_allocator.c bitset.c block_allocator.c body.c broad_phase.c
    capsule.c compound.c constraint_graph.c contact.c contact_solver.c
    convex_manifold.c core.c distance.c distance_joint.c dynamic_tree.c
    height_field.c hull.c id_pool.c island.c joint.c manifold.c
    math_functions.c mesh.c mesh_contact.c motor_joint.c mover.c
    parallel_for.c parallel_joint.c physics_world.c prismatic_joint.c
    recording.c recording_replay.c world_snapshot.c revolute_joint.c
    scheduler.c sensor.c shape.c simd.c solver.c solver_set.c sphere.c
    spherical_joint.c table.c timer.c triangle_manifold.c types.c
    weld_joint.c wheel_joint.c
)

SRC=( src/sim.c src/road.c src/vehicle.c src/wasm_api.c )
for f in "${BOX3D_SRC[@]}"; do
    SRC+=( "box3d/src/$f" )
done

# CONTRACTS §1.1 export list, underscore-prefixed for EXPORTED_FUNCTIONS.
# memory is exported automatically by STANDALONE_WASM.
EXPORTS=_sim_abi_version,_sim_alloc,_sim_load_track,_sim_reset,_sim_step,_sim_replay,_sim_state_ptr,_sim_state_size,_sim_state_hash_lo,_sim_state_hash_hi,_sim_lap_time_ticks,_sim_ffb_torque,_sim_tire_temp,_sim_damage

"$EMCC" \
    -O2 \
    -std=gnu17 \
    -ffp-contract=off \
    -DNDEBUG \
    -DBOX3D_DISABLE_SIMD \
    -Iinclude \
    -Ibox3d/include \
    -Ibox3d/src \
    -sSTANDALONE_WASM=1 \
    --no-entry \
    -sEXPORTED_FUNCTIONS="$EXPORTS" \
    -sINITIAL_MEMORY=33554432 \
    -sALLOW_MEMORY_GROWTH=1 \
    "${SRC[@]}" \
    -o out/sim.wasm

echo "built out/sim.wasm"
ls -la out/sim.wasm

# Show the import surface so hot-path WASI leakage is caught early.
if command -v wasm-objdump >/dev/null 2>&1; then
    wasm-objdump -x out/sim.wasm | sed -n '/Import\[/,/^$/p' || true
fi

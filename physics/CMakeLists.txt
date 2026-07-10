# physics/ — NATIVE test build only. The release artifact is built by
# build_wasm.sh (emcc); native builds exist for unit testing and debugging
# (CONTRACTS §1). Configure with clang + Ninja:
#   cmake -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
#         -DCMAKE_BUILD_TYPE=RelWithDebInfo -B build .

cmake_minimum_required(VERSION 3.22)
project(sim_kernel LANGUAGES C CXX)

# --- Box3D (pinned submodule), scalar + single-threaded per ADR-002 ---
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_VALIDATE OFF CACHE BOOL "" FORCE)
set(BOX3D_DISABLE_SIMD ON CACHE BOOL "" FORCE)
add_subdirectory(box3d)

# --- Strict FP: no contraction, never fast-math (CONTRACTS §1) ---
if (CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-ffp-contract=off)
endif()

# --- sim kernel static library ---
add_library(sim STATIC
    src/sim.c
    src/road.c
    src/vehicle.c
    src/wasm_api.c
)
target_include_directories(sim PUBLIC include)
target_link_libraries(sim PUBLIC box3d)
set_target_properties(sim PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

# --- tests ---
enable_testing()

add_executable(test_determinism tests/test_determinism.c tests/make_test_track.c)
target_link_libraries(test_determinism PRIVATE sim)
set_target_properties(test_determinism PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME determinism COMMAND test_determinism)

add_executable(test_vehicle tests/test_vehicle.c tests/make_test_track.c)
target_link_libraries(test_vehicle PRIVATE sim)
set_target_properties(test_vehicle PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME vehicle COMMAND test_vehicle)

# test_tire sweeps the internal brush-patch function directly (vehicle.h
# lives in src/, deliberately not part of the public include/ surface).
add_executable(test_tire tests/test_tire.c)
target_link_libraries(test_tire PRIVATE sim)
target_include_directories(test_tire PRIVATE src)
set_target_properties(test_tire PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME tire COMMAND test_tire)

# test_road drives road_query / vehicle_tire_thermal directly (road.h and
# vehicle.h live in src/, deliberately not part of the public include/ surface).
add_executable(test_road tests/test_road.c)
target_link_libraries(test_road PRIVATE sim)
target_include_directories(test_road PRIVATE src)
set_target_properties(test_road PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME road COMMAND test_road)

# test_suspension drives vehicle_suspension_step / vehicle_wheel_kinematics /
# vehicle_camber_thrust_sigma directly (docs/SUSPENSION.md §5.2) plus a full
# sim settle on the flat oval.
add_executable(test_suspension tests/test_suspension.c tests/make_test_track.c)
target_link_libraries(test_suspension PRIVATE sim)
target_include_directories(test_suspension PRIVATE src)
set_target_properties(test_suspension PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME suspension COMMAND test_suspension)

# test_damage drives the crash-damage model: vehicle_apply_impact / effect
# readouts / vehicle_reset directly (vehicle.h lives in src/) plus a full clean
# lap on the flat oval asserting zero damage (docs/SOFTBODY.md Phase 2).
add_executable(test_damage tests/test_damage.c tests/make_test_track.c)
target_link_libraries(test_damage PRIVATE sim)
target_include_directories(test_damage PRIVATE src)
set_target_properties(test_damage PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME damage COMMAND test_damage)

# test_track_v2 covers the TRK1 v2 format (static props, CONTRACTS §8) and the
# off-corridor grass fallback (docs/ROAD-SURFACE.md §6): parse/reject matrix,
# chassis-vs-prop crash damage, road_query grass/legacy modes with a bit-exact
# pre-change capture, and a full sim drive off the corridor onto grass.
add_executable(test_track_v2 tests/test_track_v2.c tests/make_test_track.c)
target_link_libraries(test_track_v2 PRIVATE sim)
target_include_directories(test_track_v2 PRIVATE src)
set_target_properties(test_track_v2 PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME track_v2 COMMAND test_track_v2)

# test_drivetrain covers the gearbox/engine/LSD wave (docs/DRIVETRAIN.md §7
# gate 2): ratio table, edge-triggered shifts, cut timing in ticks, limiter,
# engine braking, LSD split, reverse rule, over-rev protection, and the
# shift-heavy step==step==replay crown gate. Drives internal helpers
# (vehicle.h lives in src/) plus the sim on a big flat circle.
add_executable(test_drivetrain tests/test_drivetrain.c tests/make_test_track.c)
target_link_libraries(test_drivetrain PRIVATE sim)
target_include_directories(test_drivetrain PRIVATE src)
set_target_properties(test_drivetrain PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME drivetrain COMMAND test_drivetrain)

# Launch harness (docs/DRIVETRAIN.md §7 gate 4): 0-100 km/h, 1st-gear
# wheelspin, gear/top-speed table, brake-bias stopping-distance check.
add_executable(harness_launch tests/harness_launch.c tests/make_test_track.c)
target_link_libraries(harness_launch PRIVATE sim)
target_include_directories(harness_launch PRIVATE src)
set_target_properties(harness_launch PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME launch COMMAND harness_launch)

# Skidpad harness (docs/DRIVETRAIN.md §7 gate 5): steady-state 1.05-1.10 g,
# mild limit understeer, rollover threshold > 1.25 g.
add_executable(harness_skidpad tests/harness_skidpad.c tests/make_test_track.c)
target_link_libraries(harness_skidpad PRIVATE sim)
target_include_directories(harness_skidpad PRIVATE src)
set_target_properties(harness_skidpad PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
add_test(NAME skidpad COMMAND harness_skidpad)

if (UNIX)
    target_link_libraries(test_determinism PRIVATE m)
    target_link_libraries(test_vehicle PRIVATE m)
    target_link_libraries(test_tire PRIVATE m)
    target_link_libraries(test_road PRIVATE m)
    target_link_libraries(test_suspension PRIVATE m)
    target_link_libraries(test_damage PRIVATE m)
    target_link_libraries(test_track_v2 PRIVATE m)
    target_link_libraries(test_drivetrain PRIVATE m)
    target_link_libraries(harness_launch PRIVATE m)
    target_link_libraries(harness_skidpad PRIVATE m)
endif()

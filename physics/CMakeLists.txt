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

if (UNIX)
    target_link_libraries(test_determinism PRIVATE m)
    target_link_libraries(test_vehicle PRIVATE m)
    target_link_libraries(test_tire PRIVATE m)
    target_link_libraries(test_road PRIVATE m)
endif()

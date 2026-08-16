cmake_minimum_required(VERSION 3.12.0)
project(box3d_wrapper)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

# box3d is OPTIONAL. If external/box3d is absent, this is a no-op and everything downstream
# compiles with WITH_BOX3D_BINDING=0 — a build without the dependency is byte-identical to one
# from before this dependency existed. See SIMVAL/PhysicsEngineDiscussion/PHYSICS_ENGINE_ANALYSIS.md
# section 7.
if(NOT BOX3D_FOUND)
    message(STATUS "box3d: not present at ${AIRSIM_ROOT}/external/box3d — URDF/Box3D support disabled")
    return()
endif()

# Build only the static library: no samples (they need sokol/imgui, which are not vendored), no
# unit tests, no benchmarks. The engine's own suite is run separately by
# SIMVAL/urdf_physics/verify_box3d.sh, which is where the pin is verified.
set(BOX3D_SAMPLES    OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)

# ⚠ Large-world mode. This affects ABI and changes box3d's determinism hashes, so it is a pinned
# decision, not a runtime option. Resolved by measurement at Gate 1: double costs 0.98-1.03x of
# float on robot-sized scenes (within noise) with identical accuracy, so the SIMD argument for
# float does not apply here. See SIMVAL/urdf_physics/RESULTS.md, M7.
set(BOX3D_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)

add_subdirectory("${AIRSIM_ROOT}/external/box3d" "${CMAKE_CURRENT_BINARY_DIR}/box3d")

cmake_minimum_required(VERSION 3.16.0)
project(mujoco_wrapper)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

# MuJoCo is OPTIONAL and ADDITIVE. Absent => no-op, WITH_MUJOCO_BINDING=0, and Box3D continues to
# be the URDF backend exactly as before. Present => "PhysicsEngine": "MuJoCo" becomes selectable
# per vehicle in settings. Neither engine replaces the other; see external/mujoco/VENDORED.md.
#
# ⚠ OFF FOR build.sh, ON ONLY FOR build_thirdparty.sh. This directory is skipped by the ordinary build
# even when the source is vendored, because MuJoCo must NOT be compiled with the host toolchain:
# see cmake-modules/UnrealToolchain.cmake for the __isoc23_* symbols that produces and why no
# compiler flag can suppress them. build_thirdparty.sh configures this same tree a second time with
# -DAIRSIM_BUILD_MUJOCO=ON and Unreal's clang, then stages the archive into AirLib/deps/mujoco.
#
# Consequence worth stating plainly: `./build.sh` alone never produces a MuJoCo-capable plugin. It
# produces a Box3D-only one, exactly as it did before MuJoCo existed. Run ./build_thirdparty.sh once,
# then ./build.sh.
option(AIRSIM_BUILD_MUJOCO "Build the vendored MuJoCo. Requires Unreal's toolchain - use build_thirdparty.sh." OFF)

if(NOT AIRSIM_BUILD_MUJOCO)
    return()
endif()

if(NOT MUJOCO_SOURCE_FOUND)
    message(STATUS "mujoco: not vendored at ${AIRSIM_ROOT}/external/mujoco — nothing to build")
    return()
endif()

# ⚠ Build the library ONLY. simulate/ needs GLFW, the samples need more still, and the test suite
# pulls googletest and abseil over the network — none of which belong in a plugin build. MuJoCo's
# own suite is run separately against the pin, the way verify_box3d.sh does for box3d.
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_STUDIO   OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
set(MUJOCO_USE_FILAMENT   OFF CACHE BOOL "" FORCE)

# ⚠ STATIC, like box3d. MuJoCo defaults to a shared library; a .so would have to be staged next to
# the plugin binary and resolved at load, which is a packaging problem box3d does not have and this
# project has no mechanism for. A static archive is linked into libUnrealEditor-AirSim.so and there
# is nothing to ship.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# ⚠ NO LTO. MuJoCo turns on CMAKE_INTERPROCEDURAL_OPTIMIZATION for every non-Debug build, which
# makes `ar` produce an archive of LLVM IR BITCODE rather than object files. Nothing complains at
# build time: the archive is created, `file` calls it "current ar archive", and only a consumer
# reports "file format not recognized" — or, linked into a UE plugin, it fails at EDITOR STARTUP
# with "file too short". Confirmed here by extracting a member: `LLVM IR bitcode`.
#
# This is the second of the two traps UnrealRoboticsLab's third_party/build_all.sh warns about, and
# it is worse than the first because the build looks successful. MuJoCo exposes a clean opt-out, so
# no patch is needed.
set(MUJOCO_ENABLE_LTO OFF CACHE BOOL "" FORCE)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)

# ⚠ C++ ABI. MuJoCo is C++ (Box3D is C, which is why that dependency never raised this). It MUST be
# compiled with the same clang + libc++ as the rest of AirLib, or the UE plugin link fails with a
# wall of undefined std::* symbols — and with some toolchain combinations the system compiler emits
# LTO bitcode that loads as "file too short" at editor startup instead of failing loudly.
# build.sh exports CC/CXX=clang and CommonSetup.cmake adds -stdlib=libc++, so inheriting this
# project's toolchain is what keeps it correct. Recorded because it cost the UnrealRoboticsLab
# authors enough to write it into their build script.
# ⚠ Silence OUR warning policy for THIRD-PARTY sources, but keep the ABI flags.
#
# CommonSetup adds -Wall -Wextra for AirLib's own code; MuJoCo's build adds -Werror for its own.
# The combination is not one either project tests: -Wextra supplies -Wunused-parameter, MuJoCo's
# -Werror promotes it, and the build dies in plugin/obj_decoder/obj_decoder.cc on an unused `vfs`
# argument that upstream compiles cleanly. Holding a vendored dependency to our warning standard
# just means it cannot be updated.
#
# -w disables warnings entirely, so -Werror has nothing left to promote. It is appended LAST so it
# wins, and -stdlib=libc++ is deliberately left in place — that one is not a style choice, it is
# what makes the archive linkable into the UE plugin.
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# ⚠ NO -U_GNU_SOURCE HERE, and that is the interesting part. The obvious cure for the
# __isoc23_sscanf / __isoc23_strtol symbols this build used to emit is the one already applied to
# tinyxml2.cpp in cmake/AirLib/CMakeLists.txt — and it does not work for MuJoCo. libc++ requires
# _GNU_SOURCE (locale_t, strtoll_l, strtod_l, pthread_cond_clockwait), so removing it stops the
# build outright; measured, 20 errors in lodepng and tinyobjloader. Restoring _POSIX_C_SOURCE,
# _XOPEN_SOURCE and _DEFAULT_SOURCE by name gets past locale_t and then dies on the rest, which
# have no feature-test switch at all.
#
# The fix is therefore the sysroot, not a flag: see cmake-modules/UnrealToolchain.cmake, which
# build_thirdparty.sh passes as -DCMAKE_TOOLCHAIN_FILE. Recorded here because "why is there no
# -U_GNU_SOURCE next to the -w?" is a reasonable question with a non-obvious answer.

# ⚠ Neutralise MuJoCo's install/export rules.
#
# We consume MuJoCo in-tree via add_subdirectory and never `install` it, so its packaging rules are
# dead weight — but they are not harmless. `install(EXPORT mujoco ...)` demands that every target
# mujoco links be part of an export set, and once the library is STATIC its private dependencies
# (miniz, lodepng) become link dependencies rather than being absorbed. Configuration then fails
# with "requires target miniz that is not in any export set".
#
# Shadowing install() with an empty function applies to this directory and everything below it, and
# leaves the vendored source untouched — better than patching upstream for something we simply do
# not use.
function(install)
endfunction()

# export() too — qhull, one of MuJoCo's fetched dependencies, calls it directly at its own
# CMakeLists.txt:743 and fails for the same reason once targets are static.
function(export)
endfunction()

add_subdirectory("${AIRSIM_ROOT}/external/mujoco" "${CMAKE_CURRENT_BINARY_DIR}/mujoco")

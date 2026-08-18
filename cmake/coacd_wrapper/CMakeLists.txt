cmake_minimum_required(VERSION 3.24.0)
project(coacd_wrapper)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

# CoACD is OPTIONAL and shared by BOTH backends. Absent => WITH_COACD_BINDING=0, and every mesh
# keeps the single-convex-hull behaviour it has today: correct for convex shapes, fatter than it
# looks for concave ones. Nothing regresses without it. See external/coacd/VENDORED.md.
#
# ⚠ OFF FOR build.sh, ON ONLY FOR build_thirdparty.sh — same rule and same reason as MuJoCo. It is
# C++, so it must be compiled against Unreal's sysroot rather than the host's; see
# cmake-modules/UnrealToolchain.cmake.
option(AIRSIM_BUILD_COACD "Build the vendored CoACD. Requires Unreal's toolchain - use build_thirdparty.sh." OFF)

if(NOT AIRSIM_BUILD_COACD)
    return()
endif()

if(NOT COACD_SOURCE_FOUND)
    message(STATUS "coacd: not vendored at ${AIRSIM_ROOT}/external/coacd — nothing to build")
    return()
endif()

# ⚠ WITH_3RD_PARTY_LIBS=ON, and this is the expensive choice made deliberately. OFF avoids OpenVDB,
# Boost, TBB, zlib and spdlog entirely — but it does not make CoACD degrade on a non-manifold mesh,
# it makes it THROW ("The mesh is not a 2-manifold!"). Measured with
# urdf_physics/tools/mesh_manifold_check.cpp, which applies CoACD's own edge test: 1 of 10 Go2
# meshes is a closed 2-manifold. The OFF build would refuse nine of the quadruped's ten meshes.
#
# ON routes them through ManifoldPreprocess, which voxelises via OpenVDB and re-extracts a closed
# surface. Slow, hence the decomposition cache on our side.
set(WITH_3RD_PARTY_LIBS ON CACHE BOOL "" FORCE)

# ⚠ Boost's own CMakeLists still ask for compatibility with CMake < 3.5, which CMake 4 REMOVED
# (not deprecated - removed), so configuration dies in libs/predef with "Compatibility with
# CMake < 3.5 has been removed". This is a property of the dependency's age, not of anything we
# do, and CoACD pins Boost, so it cannot be fixed by moving the pin forward.
#
# CMake documents this exact escape hatch in the error message it prints. Scoped to this
# directory and below, so nothing else in the build relaxes its policy floor.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

# Static, for the same packaging reason as box3d and MuJoCo: a .so would have to be staged beside
# the plugin binary and resolved at load, which this project has no mechanism for.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(USE_STATIC_DEPENDENCIES ON CACHE BOOL "" FORCE)

# ⚠ No LTO, for the reason spelled out at length in cmake/mujoco_wrapper/CMakeLists.txt: an archive
# of LLVM bitcode is created without complaint and only fails at editor startup with "file too
# short". Set here too rather than assumed, because OpenVDB's build enables IPO independently.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)

# OpenVDB pulls Blosc and zlib by default; neither is wanted and both are more fetching.
set(USE_BLOSC OFF CACHE BOOL "" FORCE)
set(OPENVDB_ENABLE_UNINSTALL OFF CACHE BOOL "" FORCE)
set(OPENVDB_CORE_SHARED OFF CACHE BOOL "" FORCE)
set(OPENVDB_CORE_STATIC ON CACHE BOOL "" FORCE)
set(OPENVDB_BUILD_BINARIES OFF CACHE BOOL "" FORCE)
set(TBB_TEST OFF CACHE BOOL "" FORCE)

# ⚠ Boost.Iostreams' zlib support OFF. Nothing here needs it — OpenVDB's USE_ZLIB and USE_BLOSC are
# already off above, and CoACD itself does not compress anything. Leaving it on makes Boost go
# looking for a zlib, and the only one on this machine is the HOST's shared libz (see the find_root
# note in cmake-modules/UnrealToolchain.cmake for why that must never be linked into the plugin).
# Turning the feature off is better than fetching a zlib to satisfy a dependency we do not want.
set(BOOST_IOSTREAMS_ENABLE_ZLIB  OFF CACHE BOOL "" FORCE)
set(BOOST_IOSTREAMS_ENABLE_BZIP2 OFF CACHE BOOL "" FORCE)
set(BOOST_IOSTREAMS_ENABLE_LZMA  OFF CACHE BOOL "" FORCE)
set(BOOST_IOSTREAMS_ENABLE_ZSTD  OFF CACHE BOOL "" FORCE)

# ⚠ Silence OUR warning policy for THIRD-PARTY sources while keeping the ABI flags. CommonSetup
# adds -Wall -Wextra for AirLib's own code; CoACD adds -Wall -fpermissive and OpenVDB has its own
# ideas. Holding a vendored dependency to our warning standard just means it cannot be updated.
# -stdlib=libc++ is deliberately left in place - that one is not a style choice, it is what makes
# the archive linkable into the UE plugin.
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# ⚠ Neutralise install/export, exactly as for MuJoCo: `install(EXPORT ...)` demands every linked
# target be part of an export set, and once the libraries are static their private dependencies
# become link dependencies rather than being absorbed. Shadowing applies to this directory and
# below, and leaves the vendored source untouched.
function(install)
endfunction()

function(export)
endfunction()

# ⚠ CoACD's packaging needs a source PATCH, not a shadow, and this is why. It calls
# configure_package_config_file on ${CMAKE_SOURCE_DIR}/cmake/Config.cmake.in — CMAKE_SOURCE_DIR
# being the TOP-LEVEL source directory, which for us is `cmake`, not external/coacd. So it looks
# for cmake/cmake/Config.cmake.in and configuration dies. That is an upstream bug for every
# add_subdirectory consumer, not something our layout provoked.
#
# Shadowing it the way install() and export() are shadowed above does NOT work: CoACD runs
# `include(CMakePackageConfigHelpers)` afterwards, which redefines the function and discards the
# shadow. Hence COACD_INSTALL, a one-line gate in the vendored CMakeLists that defaults ON so a
# standalone build is unaffected. Recorded in external/coacd/VENDORED.md.
set(COACD_INSTALL OFF CACHE BOOL "" FORCE)

# ⚠ CoACD also builds a SHARED library (_coacd) and a command-line executable (main) that we do not
# want: the .so would have to be shipped beside the plugin, and the executable pulls a main() into
# a tree that has no use for it. Neither is needed for the static `coacd` target, and neither is
# built because build_thirdparty.sh names the `coacd` target explicitly rather than building all.

add_subdirectory("${AIRSIM_ROOT}/external/coacd" "${CMAKE_CURRENT_BINARY_DIR}/coacd")

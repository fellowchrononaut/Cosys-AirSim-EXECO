# Build a dependency with UNREAL'S OWN clang and sysroot instead of the host's.
#
# ⚠ WHY THIS EXISTS. build.sh compiles everything against the HOST toolchain (clang 18 + glibc
# 2.39 here), and Unreal then links the result with its bundled sysroot (glibc 2.28). That works
# only for as long as no translation unit uses a symbol the host glibc introduced after 2.28.
# cmake/AirLib/CMakeLists.txt already carries a one-file workaround for the first such symbol, and
# predicted the rest:
#
#     "ANY AirLib translation unit compiled on a glibc >= 2.38 host that calls sscanf/strtol-family
#      functions will fail the same way ... The structural fix is to build AirLib against Unreal's
#      bundled sysroot."
#
# MuJoCo is where that prediction came true. Since glibc 2.38, <stdio.h> and <stdlib.h> redirect
# sscanf -> __isoc23_sscanf and strtol -> __isoc23_strtol whenever _ISOC2X_SOURCE is on, and
# _GNU_SOURCE (which clang defines by default in C++ mode) turns it on. Measured on the host-built
# archives: libmujoco.a 2 such symbols, libtinyobjloader.a 1, libtinyxml2.a 1. Every one of them
# would have failed the plugin link with
#
#     ld.lld: error: undefined symbol: __isoc23_sscanf
#
# ⚠ AND THE OBVIOUS WORKAROUND DOES NOT WORK HERE, which is why this file exists rather than one
# more flag. -U_GNU_SOURCE is what rescued tinyxml2.cpp, but applied to MuJoCo it fails to compile
# at all: libc++ needs _GNU_SOURCE for locale_t/uselocale (<locale_guard.h>), and re-enabling
# _POSIX_C_SOURCE/_XOPEN_SOURCE/_DEFAULT_SOURCE by name still leaves strtoll_l, strtod_l and
# pthread_cond_clockwait undeclared - those are GNU extensions with no other switch. libc++ and
# _GNU_SOURCE are not separable, so the source-level fix is a dead end and the toolchain is the
# only lever left.
#
# Compiling against the 2.28 headers means the newer names are never emitted in the first place.
# Verified before adopting: a probe TU calling sscanf and strtol emits plain `sscanf`/`strtol`.
# UnrealRoboticsLab reaches the same conclusion in third_party/build_all.sh (--engine).
#
# ⚠ NOT a cross-compile. Host and target are both x86_64 Linux, so CMAKE_SYSTEM_NAME is left alone
# deliberately: setting it puts CMake in cross-compiling mode, where find_package(Threads) and
# friends start consulting CMAKE_FIND_ROOT_PATH and fail for no useful reason. Only the compiler
# and the sysroot change.
#
# Point UE_TOOLCHAIN at .../HostLinux/Linux_x64/<v_clang-...>/x86_64-unknown-linux-gnu.
# build_thirdparty.sh globs for it, so this is normally set for you.

if(NOT DEFINED UE_TOOLCHAIN)
    set(UE_TOOLCHAIN "$ENV{UE_TOOLCHAIN}")
endif()

if(NOT UE_TOOLCHAIN OR NOT EXISTS "${UE_TOOLCHAIN}/bin/clang++")
    message(FATAL_ERROR
        "UE_TOOLCHAIN must point at Unreal's bundled Linux toolchain, e.g.\n"
        "  <UE_ROOT>/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v25_clang-18.1.0-rockylinux8/x86_64-unknown-linux-gnu\n"
        "Got: '${UE_TOOLCHAIN}'")
endif()

set(CMAKE_C_COMPILER   "${UE_TOOLCHAIN}/bin/clang")
set(CMAKE_CXX_COMPILER "${UE_TOOLCHAIN}/bin/clang++")
set(CMAKE_SYSROOT      "${UE_TOOLCHAIN}")

# ⚠ llvm-ar/llvm-ranlib from the SAME toolchain, not the host's. The host binutils ar works today,
# but archives are the one artefact this build hands to Unreal, and a mismatched ar is exactly how
# the LTO-bitcode failure in cmake/mujoco_wrapper/CMakeLists.txt presents: an archive that is
# created without complaint and only fails at editor startup.
set(CMAKE_AR     "${UE_TOOLCHAIN}/bin/llvm-ar"     CACHE FILEPATH "" FORCE)
set(CMAKE_RANLIB "${UE_TOOLCHAIN}/bin/llvm-ranlib" CACHE FILEPATH "" FORCE)

# ⚠ CONFINE find_package TO THE SYSROOT. Setting the compiler and sysroot is not enough on its own:
# find_package still searches the HOST's /usr, and it will happily hand a third-party build a
# host library. Caught in the act — CoACD's zlib.cmake does find_package(ZLIB QUIET) and got:
#
#     ZLIB_INCLUDE_DIR : /usr/include
#     ZLIB_LIBRARY     : /usr/lib/x86_64-linux-gnu/libz.so
#
# a SHARED library built against glibc 2.39, which is exactly what this file exists to keep out of
# the plugin. It only surfaced because --sysroot then removed /usr/include from the include path
# and the compile failed on a missing zlib.h. Had the header been reachable it would have linked
# and the problem would have moved to runtime.
#
# PROGRAM stays NEVER: cmake, git and the like are host tools and are meant to be found on the
# host. LIBRARY and INCLUDE become ONLY, so nothing outside the sysroot can be linked or included.
#
# ⚠ CMAKE_SYSTEM_NAME is still deliberately unset. Host and target are both x86_64 Linux, and
# declaring a cross-compile makes find_package(Threads) and friends consult CMAKE_FIND_ROOT_PATH
# for things they should not. CMAKE_FIND_ROOT_PATH is honoured either way, so the confinement
# below does not need cross-compiling mode to work.
set(CMAKE_FIND_ROOT_PATH "${UE_TOOLCHAIN}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

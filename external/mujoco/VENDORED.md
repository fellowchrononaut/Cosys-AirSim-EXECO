# MuJoCo, vendored

Source: https://github.com/google-deepmind/mujoco
Pin:    3dd2bc3b52e6697d751a9f4bf1b2c3a3b1c83bf2
Tag:    3.11.0-127-g3dd2bc3b
Date:   2026-08-18
Licence: Apache-2.0 (see LICENSE)

Vendored as a plain copy, matching `external/box3d`, so a checkout of this fork builds without
extra submodule steps. Copied 2026-08-18 with `test/ doc/ python/ mjx/ unity/ wasm/ dist/`
excluded — none are needed to build the C library and together they are most of the 254 MB.

⚠ Build it with the SAME toolchain as the rest of AirLib: clang with `-stdlib=libc++`, which
`build.sh` and `cmake-modules/CommonSetup.cmake` already arrange. MuJoCo is C++; linking a
libstdc++ build into the UE plugin produces a wall of `std::*` undefined symbols, and some
toolchain combinations emit LTO bitcode that fails at editor load with "file too short". Box3D
never showed this because Box3D is C. Learned from UnrealRoboticsLab's `third_party/build_all.sh`.

## Local patches

Upstream is otherwise unmodified. Re-apply these on update:

1. **`CMakeLists.txt` — honour `BUILD_SHARED_LIBS`.** Upstream hardcodes
   `add_library(mujoco SHARED ...)`. We build a static archive so it links into
   `libUnrealEditor-AirSim.so` with nothing to stage beside the editor binary, matching how box3d
   is already handled. Marked in-file with `LOCAL PATCH (ExecoSim, 2026-08-18)`.

## Build-time network access

MuJoCo's CMake fetches its own dependencies (miniz, lodepng, ccd, qhull, tinyobjloader, tinyxml2)
via `FindOrFetch`. They land in `cmake/.fetchcontent-cache/`. ⚠ The first configure therefore needs
network access, unlike box3d which is fully vendored. The cache makes later builds offline.

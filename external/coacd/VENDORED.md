# CoACD — vendored

Approximate **convex decomposition**. Turns one concave triangle mesh into several convex parts.

    upstream  https://github.com/SarahWeiii/CoACD
    pin       c7436bf437e7713d5a931d45a9c79429f854563a  (2026-04-07)
    licence   MIT — see LICENSE

## Why it is here

Both physics backends represent a moving concave shape as **one convex hull**, and both are wrong
in the same way. The code has said so for a while:

* `AirLib/src/urdf/backends/Box3DRobot.cpp:330` — *"⚠ ONE CONVEX HULL per `<mesh>` … a C-shaped
  bracket collides as a solid block … The real alternative is convex decomposition (CoACD/VHACD)."*
  Box3D's mesh shapes are **static-only**, so a moving link can never use its triangles however
  hard we try. Decomposition is the only route.
* The level mirror's own report — *"they are also convex hulls, so a concave one is fatter than it
  looks"* — for kinematic bodies, for the same reason.
* MuJoCo needs it more broadly: it replaces **any** mesh with its convex hull for collision, so
  without decomposition a mirrored level becomes one convex blob per object, with doorways and
  interiors filled solid. Measured on the Blocks map: 172 mirrored shapes carrying 39,696
  triangles — every one a concave tri-mesh.

## ⚠ Built WITH_3RD_PARTY_LIBS=ON, and that is not the cheap option

`WITH_3RD_PARTY_LIBS=OFF` avoids OpenVDB, Boost, TBB, zlib and spdlog entirely. It was rejected on
evidence, not preference. Without those libraries CoACD does not degrade on a non-manifold mesh —
it refuses:

```cpp
#else
  bool is_manifold = IsManifold(m);
  if (!is_manifold)
    throw std::runtime_error("The mesh is not a 2-manifold!");
#endif
```

Measured with `urdf_physics/tools/mesh_manifold_check.cpp`, which applies the same edge test CoACD
does, so it predicts the verdict rather than approximating it:

| model | closed 2-manifolds |
|---|---|
| Unitree Go2 | **1 of 10** |
| ExoMy | 2 of 2 |

The Go2's `base.dae` has 190 non-manifold edges and 7 boundary edges in 77,928 triangles — nearly
closed, and `IsManifold` is binary, so nearly does not count. The `OFF` build would throw on nine
of the quadruped's ten meshes. `ON` routes them through `ManifoldPreprocess`, which voxelises via
OpenVDB and re-extracts a closed surface.

**Consequence to plan around:** essentially every Go2 mesh takes the slow voxelisation path. The
decomposition cache is therefore load-bearing, not an optimisation.

## Build

Not built by `build.sh`. Like MuJoCo it is built out of tree against **Unreal's** clang and
sysroot — see `cmake/cmake-modules/UnrealToolchain.cmake` for why no compiler flag substitutes:

    ./build_thirdparty.sh              # mujoco + coacd
    ./build_thirdparty.sh --only coacd

## ⚠ Its dependencies are FETCHED, not vendored

`cmake/{zlib,boost,openvdb,spdlog}.cmake` clone over the network at configure time (OpenVDB pinned
to v8.2.0; TBB arrives through it). So this directory alone does **not** pin the build — the same
caveat recorded in `external/mujoco/VENDORED.md`, and the same weak point. The build script points
FetchContent at `cmake/.fetchcontent-cache` so a second configure does not re-clone.

## Local changes

One, and it is gated so a standalone build is unaffected — see the `COACD_INSTALL` block in
`CMakeLists.txt`. Upstream's packaging reads `${CMAKE_SOURCE_DIR}/cmake/Config.cmake.in`, and
`CMAKE_SOURCE_DIR` is the **top-level** source dir, not CoACD's own — so configuration fails for
every `add_subdirectory` consumer, not just ours. The whole install/package-config section is now
behind an option defaulting **ON**.

Removed from the copy, none of it referenced by the library build:

| removed | why |
|---|---|
| `.git` | vendored, not a submodule |
| `assets/`, `docker/` | sample meshes and container recipes |
| `3rd/cdt/CDT/tests/` | 13 MB of test-expectation data. `CMakeLists.txt:76` uses only `3rd/cdt/CDT/include`, so none of it is compiled, and it was four fifths of the vendored size. |

The library sources, headers and CMake are byte-for-byte upstream apart from the gate above.

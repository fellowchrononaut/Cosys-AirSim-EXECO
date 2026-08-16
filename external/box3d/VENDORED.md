# box3d — vendored

Committed rather than fetched, deliberately. See
`../../../PhysicsEngineDiscussion/PHYSICS_ENGINE_ANALYSIS.md` R1: box3d is **v0.1.0** with breaking
API churn (`b3Body_GetLocalCenterOfMass` → `b3Body_GetLocalCenter`, `b3World_Dump` removed, and a
`b3Body_CollideMover` signature change, all within nine commits). ExecoSim archives datasets against
this code, so "the physics engine is exactly these bytes" is worth the repo weight. A `setup.sh`
download would depend on GitHub still serving the SHA and on nobody editing the URL.

| | |
|---|---|
| Upstream | https://github.com/erincatto/box3d |
| Commit | `e961bfb7bf9123188eb0addc71194c9d7af60e41` (also in `PINNED_COMMIT.txt`) |
| Date | 2026-07-11 |
| Version | v0.1.0 (the only tag) |
| License | MIT — see `LICENSE` |

## What was removed from the upstream tree

Only what the static library and its test suite need is kept, to hold the committed size down:

| Removed | Size | Why |
|---|---|---|
| `samples/` | 5.3 MB | Interactive demo app; needs sokol/imgui |
| `extern/sokol` | 2.0 MB | Only used by `samples/` |
| `data/` | 1.1 MB | Sample meshes; nothing in `src/` or `test/` reads them |
| `docs/images/` | 592 KB | Manual illustrations; the `.md` text is kept |
| `benchmark/` | 72 KB | Standalone benchmark harness |
| `.github/` | — | Upstream CI config |

Kept: `src/`, `include/`, `test/`, `shared/` (needed by `test/`), `docs/*.md`, `CMakeLists.txt`,
`LICENSE`, `README.md`. **`docs/loose_ends.md` is kept on purpose** — its "Limitations" list is
cited directly by the analysis doc and by the backend source.

## Upgrading

Do not bump this casually. An upstream update is a scheduled, tested migration:
re-run `SIMVAL/urdf_physics/verify_box3d.sh` against the new commit, then the Gate 1 suite in both
precision modes, and diff `include/box3d/*.h` for renames before touching anything else.

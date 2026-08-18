// Minimal STL reader, and the URDF `package://` path resolution that goes with it.
//
// Two Gate 2 jobs need mesh geometry and neither can proceed without it:
//   1. R2's consistency audit, which compares what Box3D drives on (<collision>) against what the
//      sensors trace (<visual>). Without mesh extents that comparison cannot be made on a real
//      robot — and on ExoMy every <visual> is a mesh, so it would measure nothing.
//   2. <mesh> collision, currently refused outright. Box3D's b3CreateHull takes points, so a
//      single convex hull per link needs a vertex list and no new dependency.
//
// Deliberately not a general asset pipeline: STL only, no normals, no materials, no scene graph.
// OBJ/DAE and convex *decomposition* are separate, later decisions.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <string>
#include <vector>

namespace urdf {

struct MeshData {
    std::vector<Vec3> vertices;  ///< triangle soup: 3 per face, in the mesh's own frame
    size_t triangles = 0;
};

/// Load a mesh by FORMAT, chosen from the file extension. This is the entry point; prefer it over
/// the per-format loaders so a new format reaches every consumer at once.
///
/// Supported: `.stl` (binary/ASCII), `.dae` (Collada), `.obj` (Wavefront).
///
/// ⚠ One loader, three consumers, and that is the reason format support lives here rather than in
/// the renderer: the Box3D collision hull (Box3DRobot.cpp), the R2 collision audit
/// (UrdfCollisionAudit.cpp) and the Unreal procedural mesh (UrdfBotPawn.cpp) all read the same
/// MeshData. A format added to only one of them produces a robot that is drawn but not collidable,
/// or audited against geometry it does not have — differences no screenshot reveals.
///
/// ⚠ Positions only, deliberately. STL has no vertex normals, and the render path already computes
/// its own (with optional decimation and smoothing), so carrying normals or materials through here
/// would add a second source of truth for shading without a consumer that wants it.
///
/// Throws std::runtime_error naming the file and the reason. An unknown extension is an error, not
/// a silent empty mesh.
MeshData loadMesh(const std::string& path);

/// Load a binary or ASCII STL. Throws std::runtime_error naming the file on failure.
///
/// The two formats are told apart by size rather than by the "solid" header, because binary STLs
/// written by several common exporters begin with the ASCII magic word and would otherwise be
/// misparsed into an empty mesh — a silent wrong answer rather than an error.
MeshData loadStl(const std::string& path);

/// Load a Collada (.dae) mesh: `<triangles>`, `<polylist>` and `<tristrips>`-free geometry.
///
/// ⚠ `<unit>` and `<up_axis>` are read and APPLIED. Collada declares its own scale and handedness,
/// and a reader that ignores them yields a robot that is silently the wrong size or lying on its
/// side — geometry that looks plausible alone and is wrong against everything else in the scene.
///
/// ⚠ The visual-scene graph is walked and node transforms are composed. Many exporters put the
/// placement of a part in `<node>` rather than baking it into the vertices, so ignoring transforms
/// collapses a multi-part assembly onto one origin. If the file declares no visual scene, every
/// geometry is emitted untransformed — the only sane reading, and the common case for the
/// single-part meshes URDFs usually reference.
MeshData loadCollada(const std::string& path);

/// Load a Wavefront `.obj`. Positions and faces only; `usemtl`/`.mtl` are ignored, as materials
/// have no consumer here. Polygons are fan-triangulated and negative (relative) indices honoured.
MeshData loadObj(const std::string& path);

/// Resolve a URDF mesh filename to something openable.
///
/// URDF meshes are conventionally `package://<pkg>/<rest>`, which only means anything with a ROS
/// package path to resolve against. `search_roots` are tried in order: for each, both
/// `<root>/<pkg>/<rest>` and `<root>/<rest>`. A plain relative path resolves against the directory
/// holding the URDF. Returns empty if nothing exists — callers report the miss rather than
/// substituting a placeholder, because a robot silently missing its geometry is the failure mode
/// this workstream keeps finding.
std::string resolveMeshPath(const std::string& filename, const std::string& urdf_dir,
                            const std::vector<std::string>& search_roots = {});

} // namespace urdf

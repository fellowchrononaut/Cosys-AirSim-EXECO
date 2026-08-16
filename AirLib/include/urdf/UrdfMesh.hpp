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

/// Load a binary or ASCII STL. Throws std::runtime_error naming the file on failure.
///
/// The two formats are told apart by size rather than by the "solid" header, because binary STLs
/// written by several common exporters begin with the ASCII magic word and would otherwise be
/// misparsed into an empty mesh — a silent wrong answer rather than an error.
MeshData loadStl(const std::string& path);

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

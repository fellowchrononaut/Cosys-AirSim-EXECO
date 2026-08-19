// Minimal STL reader, and the URDF `package://` path resolution that goes with it.
//
// Two Gate 2 jobs need mesh geometry and neither can proceed without it:
//   1. R2's consistency audit, which compares what Box3D drives on (<collision>) against what the
//      sensors trace (<visual>). Without mesh extents that comparison cannot be made on a real
//      robot — and on ExoMy every <visual> is a mesh, so it would measure nothing.
//   2. <mesh> collision, currently refused outright. Box3D's b3CreateHull takes points, so a
//      single convex hull per link needs a vertex list and no new dependency.
//
// Deliberately not a general asset pipeline: positions, triangles and per-material colour. No
// normals, no UVs, no textures, no skinning. USD is out of scope — it is a composition engine
// (layers, references, variants, binary crate), not a file format a reader of this size can parse.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <string>
#include <vector>

namespace urdf {

/// One run of consecutive triangles sharing a material.
///
/// ⚠ A section is a RANGE into MeshData::vertices, never a copy. That is the property that keeps
/// this additive: the vertex array stays one contiguous triangle soup, so the Box3D collision hull,
/// the CoACD decomposition and the R2 audit all continue to read it exactly as before and needed no
/// change when materials arrived. Anything that reorders or splits the vertex array breaks them
/// silently, because they iterate it whole.
struct MeshSection {
    size_t first_triangle = 0;   ///< index of the first triangle (vertex index / 3)
    size_t triangle_count = 0;
    Material material;           ///< material.present is false when the file declared none
};

struct MeshData {
    std::vector<Vec3> vertices;  ///< triangle soup: 3 per face, in the mesh's own frame
    size_t triangles = 0;

    /// Per-material runs, in file order. EMPTY means "the format carried no materials" — which is
    /// every STL and is not the same thing as one section with no colour, because a caller may
    /// legitimately want to know the difference. Use drawSections() rather than testing for empty.
    ///
    /// ⚠ Adjacent runs with an identical material are merged; non-adjacent ones are not. Two nodes
    /// instancing the same geometry therefore yield two sections of the same colour — an extra draw
    /// call, never a wrong colour. Merging them would mean reordering the vertex array, which the
    /// note on MeshSection forbids.
    std::vector<MeshSection> sections;

    /// Sections as a renderer should draw them: the declared list, or a single implicit section
    /// spanning the whole mesh when the format carried none. Both render paths go through this so
    /// the "empty means one" rule exists once rather than in each of them.
    std::vector<MeshSection> drawSections() const;
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
/// ⚠ Positions and per-material COLOUR, and nothing else. Normals are still not carried: the render
/// path computes its own (with optional decimation and smoothing), so carrying them would add a
/// second source of truth for shading. Colour is different — it exists nowhere else. A URDF
/// <material> is one flat colour for a whole link, so a mesh whose own file declares five materials
/// has no way to express four of them, and dropping them is what made the Go2 and the Scout render
/// as featureless white blobs while their real colours sat unread in the .dae.
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
///
/// ⚠ MATERIALS ARE RESOLVED THROUGH THE BIND, NOT BY NAME. `<triangles material="X">` names a
/// SYMBOL that is scoped to the instance, and the symbol need not be the material's id:
///
///     <triangles material="ref_Mesh004">                       <- symbol
///     <instance_material symbol="ref_Mesh004" target="#mat_Mesh004"/>   <- in <bind_material>
///     <material id="mat_Mesh004"><instance_effect url="#effect_Mesh004"/></material>
///     <effect id="effect_Mesh004">...<diffuse><color>0 0 0 1</color>
///
/// Treating the symbol as a material id happens to work on the Unitree Go2, whose exporter made
/// them identical, and resolves NOTHING on the AgileX Scout, whose exporter did not. A shortcut
/// here is therefore correct on whichever robot is tested first and silently untinted on the other.
///
/// ⚠ `<lambert>`, `<phong>`, `<blinn>` and `<constant>` all appear in robot .dae files in this
/// project — Go2 uses lambert, Scout uses phong — so handling one of them loses a whole robot.
/// `<diffuse>` holding a `<texture>` rather than a `<color>` yields no colour: the texture name is
/// recorded on Material::texture and the section is left unpainted, never guessed at.
///
/// ⚠ `<transparency>` is deliberately IGNORED and alpha is taken as 1. Its meaning inverts between
/// the A_ONE and RGB_ZERO opaque modes, and Scout declares `<transparency>1.0</transparency>` on an
/// opaque black panel — read under the wrong mode that panel disappears. Honouring it would also
/// need a translucent material, which the render path does not have.
MeshData loadCollada(const std::string& path);

/// Load a Wavefront `.obj`. Positions and faces, plus per-material sections from `usemtl` and the
/// `mtllib` it names. Polygons are fan-triangulated and negative (relative) indices honoured.
///
/// ⚠ The `.mtl` is resolved beside the `.obj` and a missing one is NOT an error — an OBJ that names
/// a library it did not ship with is common, and the geometry is still perfectly usable. It leaves
/// the sections unpainted, which the caller can see, rather than failing the load.
///
/// Only `Kd` (diffuse) and `d`/`Tr` (alpha) are read. `map_Kd` is recorded on Material::texture and
/// not loaded; there is no image decoder here.
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

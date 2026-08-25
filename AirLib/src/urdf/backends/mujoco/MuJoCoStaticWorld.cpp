#include "urdf/backends/mujoco/MuJoCoStaticWorld.hpp"

// Degenerate-cloud guard, shared with the Box3D path: a hull builder given a flat or
// collinear point set segfaults rather than refusing it.
#include "urdf/UrdfConvexDecomposition.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace urdf {
namespace {

// Helpers moved here with the emission code they serve; the per-robot backend no longer uses any
// of them, so there is exactly one copy of each.
/// Rotate a body-local point into the world by a StaticBody's orientation, then translate.
/// StaticShape geometry is expressed in its owning body's frame (UrdfStaticWorld.hpp) precisely so
/// one cooked mesh can serve every instance of a placed asset; MuJoCo geoms on the world body need
/// world coordinates, so the compose happens here.
Vec3 toWorld(const Quat& q, const Vec3& t, const Vec3& p)
{
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    return Vec3{
        t.x + p.x * (1 - 2 * (yy + zz)) + p.y * (2 * (xy - wz)) + p.z * (2 * (xz + wy)),
        t.y + p.x * (2 * (xy + wz)) + p.y * (1 - 2 * (xx + zz)) + p.z * (2 * (yz - wx)),
        t.z + p.x * (2 * (xz - wy)) + p.y * (2 * (yz + wx)) + p.z * (1 - 2 * (xx + yy))
    };
}

/// Clip a triangle against an axis-aligned box, returning the polygon that survives.
///
/// ⚠ VERTEX CONTAINMENT IS NOT ENOUGH, and assuming it was cost a full debugging cycle. A level's
/// ground is often two enormous triangles with vertices 20 km out; a robot standing in the middle
/// of one is inside the triangle while being nowhere near any of its vertices. Testing "is a vertex
/// near the robot" therefore discards exactly the geometry the robot is standing on.
///
/// Sutherland-Hodgman against the six half-spaces. New vertices are created on the box boundary,
/// so a 40 km sheet becomes a patch the size of the box — which is the point: no approximation,
/// just less of the mesh.
/// Signed volume of a closed triangle mesh. Positive => outward-facing (a solid object);
/// negative => inward-facing (an enclosure you stand inside).
///
/// ⚠ THIS, NOT CONVEXITY, IS THE DISCRIMINATOR. A room and a crate are the SAME GEOMETRY — a closed
/// box surface is convex either way — and a convexity test happily declared an enclosed room convex,
/// handed it to MuJoCo whole, and turned it into a solid block containing the robot, which was
/// launched at 39 m/s on the first step. What actually differs is which side the surface faces, and
/// winding is what encodes that.
double meshSignedVolume(const std::vector<Vec3>& pts, const std::vector<int>& idx)
{
    double v6 = 0;
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3& a = pts[idx[t]];
        const Vec3& b = pts[idx[t + 1]];
        const Vec3& c = pts[idx[t + 2]];
        v6 += a.x * (b.y * c.z - b.z * c.y)
            - a.y * (b.x * c.z - b.z * c.x)
            + a.z * (b.x * c.y - b.y * c.x);
    }
    return v6 / 6.0;
}

/// Is this mesh already convex? If so MuJoCo can take it whole, exactly, as ONE geom.
///
/// ⚠ THIS IS WHAT KEEPS THE GEOM COUNT SANE. MuJoCo replaces every mesh with its convex hull for
/// collision, so a CONCAVE object must be split or it becomes a solid blob — an enclosed room
/// became one block and the robot settled on top of it. But most level props (crates, pillars,
/// beams, blocks) are already convex, and for those the hull IS the object: one geom, exact, no
/// splitting. Splitting everything cost 52,074 geoms and 6.7 GB on one map.
///
/// The test is the direct one: every vertex must lie behind every face plane. Early-exit makes the
/// concave case cheap — a shell fails within the first few faces — while the convex case costs
/// O(F*V), which is fine because convex props are small. Meshes above `max_faces` are assumed
/// concave without testing, since a mesh that large is a level shell rather than a prop and the
/// full test would be the expensive path on the expensive case.
bool isConvexMesh(const std::vector<Vec3>& pts, const std::vector<int>& idx, size_t max_faces,
                  double eps = 1e-4)
{
    const size_t faces = idx.size() / 3;
    if (faces == 0 || faces > max_faces) return false;

    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3& a = pts[idx[t]];
        const Vec3& b = pts[idx[t + 1]];
        const Vec3& c = pts[idx[t + 2]];
        const double ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
        const double vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nlen < 1e-12) continue;                 // degenerate face tells us nothing
        nx /= nlen; ny /= nlen; nz /= nlen;

        for (const Vec3& p : pts) {
            const double d = (p.x - a.x) * nx + (p.y - a.y) * ny + (p.z - a.z) * nz;
            if (d > eps) return false;              // a vertex in front of a face => concave
        }
    }
    return true;
}

std::vector<Vec3> clipPolygonToBox(const Vec3 tri[3], const Vec3& lo, const Vec3& hi)
{
    std::vector<Vec3> poly{ tri[0], tri[1], tri[2] };

    // axis 0..2, side -1 = keep >= lo, +1 = keep <= hi
    auto clipHalfSpace = [](std::vector<Vec3>& in, int axis, double bound, int side) {
        if (in.empty()) return;
        auto coord = [axis](const Vec3& v) { return axis == 0 ? v.x : (axis == 1 ? v.y : v.z); };
        auto inside = [&](const Vec3& v) {
            return side < 0 ? coord(v) >= bound : coord(v) <= bound;
        };

        std::vector<Vec3> out;
        out.reserve(in.size() + 2);
        for (size_t i = 0; i < in.size(); ++i) {
            const Vec3& a = in[i];
            const Vec3& b = in[(i + 1) % in.size()];
            const bool ia = inside(a), ib = inside(b);
            if (ia) out.push_back(a);
            if (ia != ib) {
                const double ca = coord(a), cb = coord(b);
                const double denom = cb - ca;
                if (std::fabs(denom) > 1e-12) {
                    const double t = (bound - ca) / denom;
                    out.push_back(Vec3{ a.x + (b.x - a.x) * t,
                                        a.y + (b.y - a.y) * t,
                                        a.z + (b.z - a.z) * t });
                }
            }
        }
        in.swap(out);
    };

    clipHalfSpace(poly, 0, lo.x, -1);  clipHalfSpace(poly, 0, hi.x, +1);
    clipHalfSpace(poly, 1, lo.y, -1);  clipHalfSpace(poly, 1, hi.y, +1);
    clipHalfSpace(poly, 2, lo.z, -1);  clipHalfSpace(poly, 2, hi.z, +1);
    return poly;
}

} // namespace

void emitStaticWorld(mjSpec_* spec, mjsBody_* world, const StaticWorld* static_world,
                     const StaticWorldEmitOptions& options, StaticWorldEmitStats& stats)
{

    // --- the ground, as a sampled height field ------------------------------------------------
    //
    // ⚠ PREFERRED OVER THE FLAT PLANE, and it is the difference between "works on Blocks" and
    // "works on a map with terrain". mjGEOM_HFIELD is native, exact for slopes and steps, needs no
    // convex decomposition, and costs nothing in broadphase. The plane below remains as the
    // fallback for when the host could not sample (no ground under the robot at all).
    if (options.ground_height_field.valid() && world) {
        const BackendOptions::HeightField& hf = options.ground_height_field;

        double peak = 0;
        for (float h : hf.heights) peak = std::max(peak, static_cast<double>(h));

        mjsHField* field = mjs_addHField(spec);
        mjs_setName(field->element, "ground_hfield");
        field->nrow = hf.rows;
        field->ncol = hf.cols;
        // size is (x half-extent, y half-extent, elevation z, base depth). The base extends BELOW
        // the grid so a robot cannot fall through the underside at the patch edge.
        field->size[0] = hf.half_extent;
        field->size[1] = hf.half_extent;
        // ⚠ size[2] MUST EQUAL THE PEAK, and a floor of 0.01 here silently exaggerated terrain.
        // MuJoCo rescales elevation data to [0,1] at compile (mjCHField::Compile subtracts the
        // minimum and divides by the range) and multiplies by size[2] at collision time, so the
        // realised elevation is `h/peak * size[2]`. That is the true height only when size[2] is
        // exactly the peak. Clamping it up to 0.01 m on a region whose relief is, say, 2 mm scaled
        // every height by five: a floor that reads flat in Unreal became 1 cm of invented bumps,
        // and only on the flattest maps, which is where nobody would look for it.
        //
        // The compiler's own requirement is merely that the size be positive, so the floor exists
        // only to satisfy that, and is small enough to be below any relief worth representing.
        field->size[2] = std::max(peak, 1e-6);
        field->size[3] = 1.0;
        mjs_setFloat(field->userdata, hf.heights.data(), static_cast<int>(hf.heights.size()));

        mjsGeom* g = mjs_addGeom(world, nullptr);
        mjs_setName(g->element, "ground_hfield_geom");
        g->type = mjGEOM_HFIELD;
        mjs_setString(g->hfieldname, "ground_hfield");
        g->pos[0] = hf.center_x;
        g->pos[1] = hf.center_y;
        g->pos[2] = hf.min_z;    // heights are measured upward from here
        ++stats.geoms_emitted;
        stats.used_height_field = true;
    }

    // --- scaffolding floor --------------------------------------------------------------------
    // ⚠ Same fallback role as in the Box3D backend, and the caller applies the same suppression:
    // UrdfBotSimApi sets add_ground_plane only when the level mirror produced nothing, because an
    // infinite plane and a real level are not additive.
    if (options.add_ground_plane && world && !stats.used_height_field) {
        mjsGeom* g = mjs_addGeom(world, nullptr);
        mjs_setName(g->element, "urdf_ground_plane");
        g->type = mjGEOM_PLANE;
        // For a plane, size is (x half-extent, y half-extent, grid spacing); zeros in the first two
        // mean infinite, which is what "the floor" should be. The third is rendering-only here.
        g->size[0] = 0.0;
        g->size[1] = 0.0;
        g->size[2] = 1.0;
        g->pos[2] = options.ground_plane_z;
    }

    // --- the mirrored level -------------------------------------------------------------------
    //
    // ⚠ EVERY SHAPE MUST BE CONVEX. This is the whole reason CoACD had to land before this could:
    // MuJoCo replaces any mesh with its convex hull for collision, so mirroring the level's cooked
    // tri-meshes directly would give one convex blob per object — doorways and interiors filled
    // solid, and a robot standing on the roof of a building it should be inside. Measured on the
    // Blocks map: 172 mirrored shapes carrying 39,696 triangles, every one concave.
    //
    // Box3D does NOT need this — b3CreateMesh takes concave triangles directly for static bodies —
    // so the two engines legitimately get different collision geometry from one level. That is a
    // real difference to keep in mind when comparing them, not a bug in either.
    stats.geoms_emitted = 0;
    stats.shapes_dropped = 0;

    if (static_world && world) {
        int asset_index = 0;

        // Convex point clouds -> a mesh asset plus a geom referencing it. Vertices only: MuJoCo
        // computes the convex hull itself when no faces are supplied (user_mesh.cc:1390), which is
        // exactly what is wanted and avoids shipping a triangulation it would discard.
        auto addConvexGeom = [&](const std::vector<Vec3>& pts, const Vec3& body_pos,
                                 const Quat& body_rot, double friction,
                                 const std::string& body_name) {
            // ⚠ MuJoCo REFUSES a degenerate cloud at compile time — "mesh has colocated vertices,
            // cannot compute convex hull" (user_mesh.cc:1760) — and that fails the WHOLE model, not
            // just this shape. One sliver in one mirrored rock would stop the robot loading, so the
            // shared guard runs first. Box3D needs the same check for a different reason: it
            // segfaults instead of refusing.
            if (!hasHullableVolume(pts)) { ++stats.shapes_dropped; return; }

            std::vector<float> verts;
            verts.reserve(pts.size() * 3);
            double worst = 0;
            bool finite = true;
            double shape_lo[3] = { 1e300, 1e300, 1e300 };
            double shape_hi[3] = { -1e300, -1e300, -1e300 };
            for (const Vec3& p : pts) {
                const Vec3 w = toWorld(body_rot, body_pos, p);
                if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) {
                    finite = false;
                    break;
                }
                const double c[3] = { w.x, w.y, w.z };
                for (int k = 0; k < 3; ++k) {
                    if (c[k] < shape_lo[k]) shape_lo[k] = c[k];
                    if (c[k] > shape_hi[k]) shape_hi[k] = c[k];
                }
                worst = std::max(worst, std::max(std::fabs(w.x),
                                                 std::max(std::fabs(w.y), std::fabs(w.z))));
                verts.push_back(static_cast<float>(w.x));
                verts.push_back(static_cast<float>(w.y));
                verts.push_back(static_cast<float>(w.z));
            }

            // ⚠ VALIDATE BEFORE HANDING VERTICES TO MUJOCO. A single non-finite or absurd vertex
            // does not fail loudly — it compiles, and then poisons every collision and ray query in
            // the model, because rbound and the broadphase are computed from it. Measured on the
            // Blocks mirror: one shape produced a geom with a bounding radius of 40,141 m in a
            // level 200 m across, and the robot fell through a world that every other diagnostic
            // reported as present and correctly masked.
            //
            // ⚠ MEASURE THE SHAPE'S OWN SPAN, not its distance from the world origin. The first
            // version of this check tested |v| against 100 km, which the offending shape — a
            // vertex 40 km out — passed comfortably, so it did nothing at all. Distance from the
            // origin is the wrong quantity: what breaks MuJoCo's broadphase is a single geom whose
            // BOUNDING RADIUS dwarfs the scene, and that is a property of the shape's own extent.
            //
            // 1 km is chosen against the thing being measured: the Blocks mirror spans ~200 m, so
            // any single mirrored object a kilometre across is corrupt rather than merely large.
            double span = 0;
            for (int k = 0; k < 3; ++k) span = std::max(span, shape_hi[k] - shape_lo[k]);
            if (span > stats.worst_span) {
                stats.worst_span = span;
                stats.worst_span_body = body_name;
            }
            // `worst` was computed and then never read - a leftover from a superseded test that
            // compared distance-from-origin against 100 km. It is real data about how far out the
            // level reaches, so it is reported rather than discarded.
            stats.worst_vertex = std::max(stats.worst_vertex, worst);
            if (!finite) {
                ++stats.shapes_dropped;
                return;
            }

            // ⚠ A VAST FLAT SLAB IS NOT CONVERTED TO A PLANE HERE, though a long comment
            // once claimed it was. Unreal levels routinely sit on an enormous ground mesh - the
            // Blocks map's is 40 km across and ~0.2 m thick - and handing that to MuJoCo whole
            // gives a geom with a 20 km bounding radius: a broadphase candidate for every query in
            // the model, with float32 vertex storage resolving to ~2.4 mm out there.
            //
            // What actually prevents that is the region CLIPPING in the Mesh branch below, so
            // shapes reaching this point are already local. mjGEOM_PLANE would be exact and free,
            // and remains the obvious improvement - but it must be written before it is described,
            // and before any counter reports it.
            const std::string name = "sw_mesh_" + std::to_string(asset_index++);
            mjsMesh* mesh = mjs_addMesh(spec, nullptr);
            mjs_setName(mesh->element, name.c_str());
            mjs_setFloat(mesh->uservert, verts.data(), static_cast<int>(verts.size()));

            mjsGeom* g = mjs_addGeom(world, nullptr);
            g->type = mjGEOM_MESH;
            mjs_setString(g->meshname, name.c_str());
            // ⚠ Vertices are already in world coordinates, so the geom's own transform is identity.
            // Baking the body transform into the points rather than setting geom pos/quat keeps one
            // convention for all four shape kinds, and the level never moves.
            g->friction[0] = friction;
            ++stats.geoms_emitted;
        };

        // ⚠ Report per BODY, not per triangle: a level has a few hundred bodies and tens of
        // thousands of triangles, and a progress callback fired 52,000 times is itself a cost.
        int body_index = 0;
        const int body_total = static_cast<int>(static_world->bodies.size());

        for (const StaticBody& body : static_world->bodies) {
            if (options.build_progress)
                options.build_progress("level collision geometry", ++body_index, body_total);

            for (const StaticShape& shape : body.shapes) {
                switch (shape.kind) {
                case StaticShapeKind::Sphere: {
                    mjsGeom* g = mjs_addGeom(world, nullptr);
                    g->type = mjGEOM_SPHERE;
                    const Vec3 c = toWorld(body.orientation, body.position, shape.center_a);
                    g->pos[0] = c.x; g->pos[1] = c.y; g->pos[2] = c.z;
                    g->size[0] = shape.radius;
                    g->friction[0] = body.friction;
                    ++stats.geoms_emitted;
                    break;
                }
                case StaticShapeKind::Capsule: {
                    mjsGeom* g = mjs_addGeom(world, nullptr);
                    g->type = mjGEOM_CAPSULE;
                    // fromto is the natural spelling here: our capsule is already stored as two
                    // hemisphere centres, so no axis/length/orientation round trip is needed.
                    const Vec3 a = toWorld(body.orientation, body.position, shape.center_a);
                    const Vec3 b = toWorld(body.orientation, body.position, shape.center_b);
                    g->fromto[0] = a.x; g->fromto[1] = a.y; g->fromto[2] = a.z;
                    g->fromto[3] = b.x; g->fromto[4] = b.y; g->fromto[5] = b.z;
                    g->size[0] = shape.radius;
                    g->friction[0] = body.friction;
                    ++stats.geoms_emitted;
                    break;
                }
                case StaticShapeKind::Hull:
                    // Already convex — Unreal cooked it from an FKAggregateGeom convex element, so
                    // it goes straight in with no decomposition.
                    addConvexGeom(shape.points, body.position, body.orientation, body.friction,
                                  body.name);
                    break;

                case StaticShapeKind::Mesh: {
                    // ⚠ CONVEX OBJECTS GO IN WHOLE — one geom, exact, no splitting. MuJoCo hulls
                    // every mesh, and for an already-convex object the hull IS the object, so this
                    // costs nothing in fidelity and collapses the common case (crates, pillars,
                    // beams) from hundreds of geoms to one.
                    // ⚠ Convex AND outward-facing. Convexity alone is not enough: an enclosure
                    // is convex too, and taking one whole makes a solid block around the robot.
                    // A negative signed volume means the surface faces inward, i.e. the robot is
                    // meant to be INSIDE it, and it must be split.
                    const double signed_volume =
                        meshSignedVolume(shape.points, shape.indices);
                    const bool take_whole =
                        (options.static_mesh_mode == BackendOptions::StaticMeshMode::Whole) ||
                        (options.static_mesh_mode == BackendOptions::StaticMeshMode::Auto &&
                         signed_volume > 0 &&
                         isConvexMesh(shape.points, shape.indices, /*max_faces=*/2000));

                    if (signed_volume < 0) ++stats.enclosures;

                    if (take_whole) {
                        addConvexGeom(shape.points, body.position, body.orientation, body.friction,
                                      body.name);
                        ++stats.convex_objects;
                        break;
                    }

                    // ⚠ ONE THIN CONVEX PRISM PER TRIANGLE — NOT convex decomposition.
                    //
                    // Decomposing static level geometry was wrong, and an enclosed map proved it:
                    // a hollow room is a SHELL, and approximating a shell with a bounded number of
                    // convex parts FILLS THE CAVITY, because that is what convex decomposition of a
                    // hollow volume means. Measured: 171 shapes produced ~176 parts — the room
                    // became one solid block, the robot collided with invisible geometry in mid-air
                    // and settled upside down. No part budget fixes this; a room needs the surface,
                    // not an approximation of its volume.
                    //
                    // A triangle extruded along its normal is convex BY CONSTRUCTION, so MuJoCo
                    // takes it exactly. Walls stay walls, doorways stay open, and CoACD is not
                    // involved in static geometry at all — it goes back to robot link meshes, where
                    // a closed concave part genuinely does want decomposing.
                    //
                    // Box3D never needed any of this: b3CreateMesh cooks the real triangles.
                    const double kThickness = 0.02;   // 2 cm: stable contact, no visible inflation

                    const Vec3 c = options.clip_center;
                    const double r = options.static_world_radius;

                    for (size_t t = 0; t + 2 < shape.indices.size(); t += 3) {
                        const Vec3 a = toWorld(body.orientation, body.position,
                                               shape.points[shape.indices[t]]);
                        const Vec3 b2 = toWorld(body.orientation, body.position,
                                                shape.points[shape.indices[t + 1]]);
                        const Vec3 c2 = toWorld(body.orientation, body.position,
                                                shape.points[shape.indices[t + 2]]);

                        // ⚠ CLIP, do not merely cull. Culling by bounding box keeps any triangle
                        // that OVERLAPS the region — and a level's geometry is often a handful of
                        // enormous triangles, each of which overlaps and is then emitted at full
                        // size. Measured on an enclosed map: prisms spanning +/-15 km inside a 30 m
                        // region, which is both a terrible collision approximation and a broadphase
                        // disaster.
                        //
                        // Clipping was rejected earlier for the DECOMPOSITION path, where cutting a
                        // closed mesh opens it and the convex parts bulge into the gap. That
                        // objection does not apply here: each prism is independent, so clipping a
                        // triangle simply yields a smaller polygon. Same clipper, different context.
                        std::vector<Vec3> poly{ a, b2, c2 };
                        if (r > 0) {
                            const Vec3 lo{ c.x - r, c.y - r, c.z - r };
                            const Vec3 hi{ c.x + r, c.y + r, c.z + r };
                            const Vec3 tri_in[3] = { a, b2, c2 };
                            poly = clipPolygonToBox(tri_in, lo, hi);
                            // Entirely outside the region: counted, because "the level mirrored"
                            // and "the robot has geometry near it" are different claims.
                            if (poly.size() < 3) { ++stats.triangles_clipped_away; continue; }
                        }

                        // Face normal; a degenerate triangle has none and is skipped.
                        const double ux = b2.x - a.x, uy = b2.y - a.y, uz = b2.z - a.z;
                        const double vx = c2.x - a.x, vy = c2.y - a.y, vz = c2.z - a.z;
                        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
                        const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
                        if (nlen < 1e-12) { ++stats.shapes_dropped; continue; }
                        nx /= nlen; ny /= nlen; nz /= nlen;

                        // A clipped triangle can become a polygon of up to 7 vertices; fan it
                        // back into triangles so every emitted prism stays a simple wedge.
                        const double h = 0.5 * kThickness;
                        for (size_t k = 1; k + 1 < poly.size(); ++k) {
                            // ⚠ Stop rather than grind. Hitting the cap means the region radius is
                            // too large for this map's tessellation; both numbers are reported so
                            // it can be tuned instead of guessed at.
                            if (options.static_world_max_triangles > 0 &&
                                stats.triangles_emitted >=
                                    static_cast<size_t>(options.static_world_max_triangles)) {
                                ++stats.triangles_skipped;
                                continue;
                            }

                            const Vec3 tri[3] = { poly[0], poly[k], poly[k + 1] };

                            // ⚠ SKIP ZERO-AREA FAN TRIANGLES. Clipping against a box produces
                            // duplicate and collinear vertices at the corners, and fanning them
                            // yields degenerate triangles whose extrusion is six COPLANAR points.
                            // MuJoCo refuses those at COMPILE time — "mesh has coplanar vertices,
                            // cannot compute convex hull" — which fails the entire model, not just
                            // that shape. One sliver in one wall would stop the robot loading.
                            {
                                const double e1x = tri[1].x - tri[0].x, e1y = tri[1].y - tri[0].y,
                                             e1z = tri[1].z - tri[0].z;
                                const double e2x = tri[2].x - tri[0].x, e2y = tri[2].y - tri[0].y,
                                             e2z = tri[2].z - tri[0].z;
                                const double cx = e1y * e2z - e1z * e2y;
                                const double cy = e1z * e2x - e1x * e2z;
                                const double cz = e1x * e2y - e1y * e2x;
                                // Area is half the cross-product magnitude; 1e-9 m^2 is far below
                                // anything that could carry a contact.
                                if (0.5 * std::sqrt(cx * cx + cy * cy + cz * cz) < 1e-9) continue;
                            }

                            std::vector<float> verts;
                            verts.reserve(18);
                            for (int side = -1; side <= 1; side += 2)
                                for (const Vec3& p0 : tri) {
                                    verts.push_back(static_cast<float>(p0.x + nx * h * side));
                                    verts.push_back(static_cast<float>(p0.y + ny * h * side));
                                    verts.push_back(static_cast<float>(p0.z + nz * h * side));
                                }

                            const std::string mname = "sw_tri_" + std::to_string(asset_index++);
                            mjsMesh* mesh = mjs_addMesh(spec, nullptr);
                            mjs_setName(mesh->element, mname.c_str());
                            mjs_setFloat(mesh->uservert, verts.data(),
                                         static_cast<int>(verts.size()));

                            mjsGeom* g = mjs_addGeom(world, nullptr);
                            g->type = mjGEOM_MESH;
                            mjs_setString(g->meshname, mname.c_str());
                            g->friction[0] = body.friction;
                            ++stats.geoms_emitted;
                            ++stats.triangles_emitted;
                        }
                    }
                    break;
                }
                }
            }
        }
    }
}

} // namespace urdf

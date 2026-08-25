#include "urdf/backends/Box3DCollisionReadback.hpp"

#include "urdf/backends/Box3DRobot.hpp"

#include <box3d/box3d.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace urdf {
namespace {

/// Walk one Box3D hull's half-edge faces into a triangle list, in hull-local coordinates.
///
/// ⚠ Box3D hull faces are convex POLYGONS, not triangles — the half-edge ring is the only place
/// their winding is recorded. Fanning the ring here keeps the drawn wireframe the solver's actual
/// face set rather than a re-hulled approximation of its point cloud.
void appendHullTriangles(const b3HullData* hull, CollisionShape& geom)
{
    if (hull == nullptr) return;
    const b3Vec3* points = b3GetHullPoints(hull);
    const b3HullHalfEdge* edges = b3GetHullEdges(hull);
    const b3HullFace* faces = b3GetHullFaces(hull);
    if (points == nullptr || edges == nullptr || faces == nullptr) return;

    geom.vertices.reserve(static_cast<size_t>(hull->vertexCount));
    for (int i = 0; i < hull->vertexCount; ++i)
        geom.vertices.push_back(Vec3{ points[i].x, points[i].y, points[i].z });

    for (int f = 0; f < hull->faceCount; ++f) {
        const uint8_t start = faces[f].edge;
        uint8_t edge = edges[start].next;
        const uint8_t root = edges[start].origin;
        // Fan from the face's first vertex; stop when the ring closes, and bound the walk so a
        // corrupt half-edge table cannot spin here forever.
        for (int guard = 0; guard < hull->edgeCount && edges[edge].next != start; ++guard) {
            geom.indices.push_back(root);
            geom.indices.push_back(edges[edge].origin);
            geom.indices.push_back(edges[edges[edge].next].origin);
            edge = edges[edge].next;
        }
    }
}

Vec3 toVec3(const b3Vec3& v) { return Vec3{ v.x, v.y, v.z }; }

bool tooFar(const CollisionDebugFilter& filter, const Vec3& position, double reach)
{
    if (!(filter.radius > 0)) return false;
    const double dx = position.x - filter.center.x;
    const double dy = position.y - filter.center.y;
    const double bound = filter.radius + reach;
    return dx * dx + dy * dy > bound * bound;
}

} // namespace

void readBox3DRobotCollision(const b3urdf::Box3DRobot& robot, const std::string& prefix,
                             const CollisionDebugFilter& filter, CollisionDebugSnapshot& out)
{
    if (!robot.isBuilt()) return;

    for (size_t l = 0; l < robot.linkCount(); ++l) {
        const b3BodyId body = robot.bodyId(l);
        if (!b3Body_IsValid(body)) continue;
        const b3WorldTransform tf = b3Body_GetTransform(body);
        const Vec3 body_pos{ tf.p.x, tf.p.y, tf.p.z };
        const Quat body_rot{ tf.q.v.x, tf.q.v.y, tf.q.v.z, tf.q.s };

        const int shape_count = b3Body_GetShapeCount(body);
        if (shape_count <= 0) continue;
        std::vector<b3ShapeId> shapes(static_cast<size_t>(shape_count));
        const int got = b3Body_GetShapes(body, shapes.data(), shape_count);

        for (int s = 0; s < got; ++s) {
            if (out.geoms.size() >= filter.max_geoms) { ++out.omitted; continue; }

            CollisionShape geom;
            geom.provenance = CollisionShape::Provenance::Realised;
            geom.label = prefix + robot.linkName(l);
            geom.is_world = false;
            // ⚠ Shape geometry is BODY-LOCAL in Box3D, so the body transform is the geom frame and
            // the points go in untouched. Baking the transform into the points instead would
            // silently lose the rotation for a hull created transformed.
            geom.position = body_pos;
            geom.orientation = body_rot;

            switch (b3Shape_GetType(shapes[s])) {
            case b3_sphereShape: {
                const b3Sphere sphere = b3Shape_GetSphere(shapes[s]);
                geom.kind = CollisionShape::Kind::Sphere;
                geom.radius = sphere.radius;
                // The centre is BODY-LOCAL and stays that way, alongside the capsule's two centres
                // below: one convention for where a shape sits inside its body.
                geom.vertices.push_back(toVec3(sphere.center));
                break;
            }
            case b3_capsuleShape: {
                const b3Capsule capsule = b3Shape_GetCapsule(shapes[s]);
                geom.kind = CollisionShape::Kind::Capsule;
                geom.radius = capsule.radius;
                // Endpoints kept verbatim rather than reduced to an axis and a length: the drawing
                // side needs both centres and any reduction round-trips badly.
                geom.vertices.push_back(toVec3(capsule.center1));
                geom.vertices.push_back(toVec3(capsule.center2));
                break;
            }
            case b3_hullShape:
                geom.kind = CollisionShape::Kind::Mesh;
                appendHullTriangles(b3Shape_GetHull(shapes[s]), geom);
                break;
            default:
                // Meshes, height fields and compounds on a robot link. Box3D's mesh shape is
                // static-only, so a link should never carry one; counted rather than assumed
                // impossible.
                ++out.omitted;
                {
                    const std::string name =
                        "b3ShapeType(" +
                        std::to_string(static_cast<int>(b3Shape_GetType(shapes[s]))) + ")";
                    if (std::find(out.omitted_kinds.begin(), out.omitted_kinds.end(), name) ==
                        out.omitted_kinds.end())
                        out.omitted_kinds.push_back(name);
                }
                continue;
            }

            // ⚠ REACH FROM THE SHAPE, never a constant. A fixed guess here made a 0.5 m radius keep
            // a robot two metres away, which is the filter quietly not filtering — the same failure
            // as culling level triangles by their vertices. Shape geometry is body-local, so its
            // own extent is exactly the reach.
            double reach = geom.radius;
            for (const Vec3& v : geom.vertices)
                reach = std::max(reach, geom.radius + std::sqrt(v.x * v.x + v.y * v.y));
            if (tooFar(filter, geom.position, reach)) continue;
            out.geoms.push_back(std::move(geom));
        }
    }
}

void readBox3DStaticWorld(const StaticWorld& level, const CollisionDebugFilter& filter,
                          CollisionDebugSnapshot& out)
{
    for (const StaticBody& body : level.bodies) {
        for (const StaticShape& shape : body.shapes) {
            if (out.geoms.size() >= filter.max_geoms) { ++out.omitted; continue; }

            CollisionShape geom;
            geom.provenance = CollisionShape::Provenance::Submitted;
            geom.label = body.name.empty() ? std::string("level") : body.name;
            geom.is_world = true;
            geom.position = body.position;
            geom.orientation = body.orientation;

            switch (shape.kind) {
            case StaticShapeKind::Sphere:
                geom.kind = CollisionShape::Kind::Sphere;
                geom.radius = shape.radius;
                geom.vertices.push_back(shape.center_a);
                break;
            case StaticShapeKind::Capsule:
                geom.kind = CollisionShape::Kind::Capsule;
                geom.radius = shape.radius;
                geom.vertices.push_back(shape.center_a);
                geom.vertices.push_back(shape.center_b);
                break;
            case StaticShapeKind::Hull:
            case StaticShapeKind::Mesh:
                geom.kind = CollisionShape::Kind::Mesh;
                geom.vertices = shape.points;
                geom.indices = shape.indices;
                break;
            }

            // A level mesh can be kilometres across, so its own extent is the reach that decides
            // whether the region touches it — the same lesson as the triangle clipper.
            double reach = 0;
            for (const Vec3& p : geom.vertices)
                reach = std::max(reach, std::max(std::fabs(p.x), std::fabs(p.y)));
            if (tooFar(filter, geom.position, reach + shape.radius)) continue;
            out.geoms.push_back(std::move(geom));
        }
    }
}

void describeBox3DColliders(const b3urdf::Box3DRobot& robot, const std::string& prefix,
                            PhysicsColliderSet& out)
{
    out = PhysicsColliderSet();
    out.backend = "box3d";
    if (!robot.isBuilt())
        return;

    for (size_t l = 0; l < robot.linkCount(); ++l) {
        const b3BodyId body = robot.bodyId(l);
        if (!b3Body_IsValid(body)) {
            // Named, not merely missing: a sidecar told nothing about this link would register a
            // robot with a silent hole in it.
            out.undescribed.push_back(prefix + robot.linkName(l) +
                                      " (Box3D realised no body for this link)");
            continue;
        }

        PhysicsColliderDescriptor collider;
        collider.stable_id = prefix + robot.linkName(l);
        collider.link_index = l;
        collider.link_name = robot.linkName(l);

        const b3WorldTransform tf = b3Body_GetTransform(body);
        collider.position = Vec3{ tf.p.x, tf.p.y, tf.p.z };
        collider.orientation = Quat{ tf.q.v.x, tf.q.v.y, tf.q.v.z, tf.q.s };
        const b3Vec3 v = b3Body_GetLinearVelocity(body);
        const b3Vec3 w = b3Body_GetAngularVelocity(body);
        collider.linear_velocity = Vec3{ v.x, v.y, v.z };
        collider.angular_velocity = Vec3{ w.x, w.y, w.z };

        const b3MassData mass_data = b3Body_GetMassData(body);
        LinkInertialDescriptor& inertial = collider.inertial;
        inertial.mass = mass_data.mass;
        inertial.com_local = Vec3{ mass_data.center.x, mass_data.center.y, mass_data.center.z };
        // b3Matrix3 is three COLUMNS (cx, cy, cz); the descriptor is row-major. Transposing at the
        // boundary rather than shipping Box3D's layout keeps one convention for both backends.
        const b3Matrix3& I = mass_data.inertia;
        inertial.inertia_local[0] = I.cx.x; inertial.inertia_local[1] = I.cy.x;
        inertial.inertia_local[2] = I.cz.x;
        inertial.inertia_local[3] = I.cx.y; inertial.inertia_local[4] = I.cy.y;
        inertial.inertia_local[5] = I.cz.y;
        inertial.inertia_local[6] = I.cx.z; inertial.inertia_local[7] = I.cy.z;
        inertial.inertia_local[8] = I.cz.z;
        inertial.provenance = InertiaProvenance::SolverRealised;
        inertial.is_articulated_effective_inertia = false;
        // Box3D exposes no articulated mean inverse inertia, unlike MuJoCo's body_invweight0.
        inertial.has_mean_inverse_inertia = false;

        // Body-local shapes, through the one readback. `readBox3DRobotCollision` reports the body
        // transform as each shape's frame; a descriptor wants them in the body's own frame, and
        // for Box3D that is the identity, because its shape geometry is already body-local.
        CollisionDebugFilter everything;
        CollisionDebugSnapshot geoms;
        {
            // One link at a time would need a per-link accessor Box3D does not have, so the whole
            // robot is walked and filtered by label - cheap, and this runs at scenario build.
            readBox3DRobotCollision(robot, prefix, everything, geoms);
            for (CollisionShape& shape : geoms.geoms) {
                if (shape.label != collider.stable_id) continue;
                shape.position = Vec3{ 0, 0, 0 };
                shape.orientation = Quat{ 0, 0, 0, 1 };
                collider.shapes.push_back(std::move(shape));
            }
        }

        if (b3Body_GetShapeCount(body) > 0) {
            std::vector<b3ShapeId> shapes(static_cast<size_t>(b3Body_GetShapeCount(body)));
            const int got = b3Body_GetShapes(body, shapes.data(), static_cast<int>(shapes.size()));
            if (got > 0) {
                collider.material.friction = b3Shape_GetFriction(shapes[0]);
                collider.material.restitution = b3Shape_GetRestitution(shapes[0]);
                collider.material.reported = true;
            }
        }

        // ⚠ KinematicOneWay for every link, for the same reason as MuJoCo: plan §11.1 gates
        // DynamicTwoWay on an effective inertia neither backend can supply, not on plumbing. Note
        // that a link Box3D drives kinematically (below a cosmetic <mimic>) reaches the same role
        // by a DIFFERENT route - it has no dynamics to receive a reaction into at all - and that
        // distinction will matter when two-way lands, so it is recorded rather than collapsed.
        collider.role = CouplingRole::KinematicOneWay;
        if (robot.isLinkKinematic(l))
            out.undescribed.push_back(collider.stable_id +
                                      " (kinematic under a cosmetic <mimic>: one-way regardless of "
                                      "what the effective-inertia spike concludes)");

        out.colliders.push_back(std::move(collider));
    }
}

} // namespace urdf

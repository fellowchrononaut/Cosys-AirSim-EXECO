#include "urdf/backends/mujoco/MuJoCoCollisionReadback.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <functional>
#include <string>

namespace urdf {

void readMuJoCoCollisionGeometry(const mjModel_* model, const mjData_* data,
                                 const CollisionDebugFilter& filter,
                                 const MuJoCoBodyLabeller& label, CollisionDebugSnapshot& out)
{
    if (model == nullptr || data == nullptr)
        return;

    const bool clip = filter.radius > 0;
    const double r2 = filter.radius * filter.radius;

    for (int g = 0; g < model->ngeom; ++g) {
        // ⚠ A GEOM THAT COLLIDES WITH NOTHING IS NOT COLLISION GEOMETRY. MuJoCo uses the same array
        // for visual-only geoms; drawing those as though they were physical is precisely the
        // confusion this readback exists to remove.
        if (model->geom_contype[g] == 0 && model->geom_conaffinity[g] == 0)
            continue;

        bool is_world = false;
        const std::string owner = label(model->geom_bodyid[g], is_world);
        if (is_world && !filter.include_world) continue;
        if (!is_world && !filter.include_robots) continue;

        const double* xp = data->geom_xpos + 3 * g;
        if (clip) {
            const double dx = xp[0] - filter.center.x;
            const double dy = xp[1] - filter.center.y;
            // ⚠ Against the geom's BOUNDING SPHERE, not its centre. The level's ground prisms are
            // metres wide; testing centres alone punches a hole in the overlay exactly where the
            // robot is standing.
            const double reach = filter.radius + model->geom_rbound[g];
            if (dx * dx + dy * dy > reach * reach && dx * dx + dy * dy > r2) continue;
        }

        if (out.geoms.size() >= filter.max_geoms) {
            ++out.omitted;
            continue;
        }

        CollisionShape geom;
        geom.provenance = CollisionShape::Provenance::Realised;
        geom.label = owner;
        geom.is_world = is_world;
        geom.margin = model->geom_margin[g];
        geom.position = Vec3{ xp[0], xp[1], xp[2] };
        {
            mjtNum quat[4];
            mju_mat2Quat(quat, data->geom_xmat + 9 * g);
            geom.orientation = Quat{ quat[1], quat[2], quat[3], quat[0] };
        }

        const double* size = model->geom_size + 3 * g;
        switch (model->geom_type[g]) {
        case mjGEOM_SPHERE:
            geom.kind = CollisionShape::Kind::Sphere;
            geom.radius = size[0];
            break;
        case mjGEOM_CAPSULE:
            geom.kind = CollisionShape::Kind::Capsule;
            geom.radius = size[0];
            geom.half_length = size[1];
            break;
        case mjGEOM_CYLINDER:
            geom.kind = CollisionShape::Kind::Cylinder;
            geom.radius = size[0];
            geom.half_length = size[1];
            break;
        case mjGEOM_BOX:
            geom.kind = CollisionShape::Kind::Box;
            geom.half_extents = Vec3{ size[0], size[1], size[2] };
            break;
        case mjGEOM_PLANE:
            geom.kind = CollisionShape::Kind::Plane;
            // Zero means infinite; give the drawing side something finite to work with while
            // keeping the fact that it is a stand-in visible through the kind.
            geom.half_extents = Vec3{ size[0] > 0 ? size[0] : 50.0,
                                      size[1] > 0 ? size[1] : 50.0, 0.0 };
            break;
        case mjGEOM_HFIELD: {
            const int hid = model->geom_dataid[g];
            if (hid < 0) { ++out.omitted; continue; }
            geom.kind = CollisionShape::Kind::HeightField;
            geom.rows = model->hfield_nrow[hid];
            geom.cols = model->hfield_ncol[hid];
            const double* hs = model->hfield_size + 4 * hid;
            geom.half_extents = Vec3{ hs[0], hs[1], 0.0 };
            // ⚠ DENORMALISE. MuJoCo rescales elevation data to [0,1] at compile
            // (mjCHField::Compile) and multiplies by size[2] at collision time. Handing the raw
            // array out would draw terrain flattened to a metre of relief regardless of the map.
            const float* raw = model->hfield_data + model->hfield_adr[hid];
            const size_t count = static_cast<size_t>(geom.rows) * geom.cols;
            geom.heights.resize(count);
            for (size_t i = 0; i < count; ++i)
                geom.heights[i] = static_cast<float>(raw[i] * hs[2]);
            break;
        }
        case mjGEOM_MESH: {
            const int mid = model->geom_dataid[g];
            if (mid < 0) { ++out.omitted; continue; }
            geom.kind = CollisionShape::Kind::Mesh;
            // ⚠ THE CONVEX HULL, NOT WHAT WE SUBMITTED. MuJoCo runs qhull on a vertex-only mesh and
            // stores the result as the mesh's own faces (user_mesh.cc:1386), so this is the surface
            // it will actually collide against - including wherever that is fatter than the asset.
            const int vadr = model->mesh_vertadr[mid];
            const int vnum = model->mesh_vertnum[mid];
            const int fadr = model->mesh_faceadr[mid];
            const int fnum = model->mesh_facenum[mid];
            geom.vertices.reserve(static_cast<size_t>(vnum));
            for (int v = 0; v < vnum; ++v) {
                const float* p = model->mesh_vert + 3 * (vadr + v);
                geom.vertices.push_back(Vec3{ p[0], p[1], p[2] });
            }
            geom.indices.reserve(static_cast<size_t>(fnum) * 3);
            for (int f = 0; f < fnum; ++f) {
                const int* tri = model->mesh_face + 3 * (fadr + f);
                geom.indices.push_back(tri[0]);
                geom.indices.push_back(tri[1]);
                geom.indices.push_back(tri[2]);
            }
            break;
        }
        default:
            // Ellipsoids, SDFs and anything MuJoCo grows later. Counted and NAMED, so a gap in the
            // overlay is explainable instead of merely puzzling.
            //
            // MuJoCo ships no geom-type-to-string helper, and inventing a table that could fall out
            // of step with mjtGeom is worse than reporting the enum value: the number is checkable
            // against the header, a stale name is not.
            ++out.omitted;
            {
                const std::string name = "mjtGeom(" + std::to_string(model->geom_type[g]) + ")";
                if (std::find(out.omitted_kinds.begin(), out.omitted_kinds.end(), name) ==
                    out.omitted_kinds.end())
                    out.omitted_kinds.push_back(name);
            }
            continue;
        }

        out.geoms.push_back(std::move(geom));
    }
}

namespace {

/// MuJoCo stores a body's inertia as a diagonal in a principal frame (`body_inertia` about
/// `body_ipos`/`body_iquat`). Rotate it back into the body frame: I = R * diag * R^T.
///
/// ⚠ Done ONCE, here, rather than handing both spellings to consumers. Box3D keeps a dense tensor
/// and MuJoCo a diagonal plus a frame; a descriptor that exported whichever the backend happened to
/// hold would make every downstream reduction branch on the backend, which is how two engines start
/// disagreeing about the same robot.
void diagonalToBodyFrame(const mjtNum* diag, const mjtNum* quat, double out[9])
{
    mjtNum R[9];
    mju_quat2Mat(R, quat);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += R[3 * r + k] * diag[k] * R[3 * c + k];
            out[3 * r + c] = sum;
        }
}

} // namespace

void describeMuJoCoColliders(const mjModel_* model, const mjData_* data,
                             const std::function<bool(int bodyid)>& owns,
                             const MuJoCoBodyLabeller& stable_id, PhysicsColliderSet& out)
{
    out = PhysicsColliderSet();
    out.backend = "mujoco";
    if (model == nullptr || data == nullptr)
        return;

    for (int b = 1; b < model->nbody; ++b) {   // body 0 is the world by MuJoCo's construction
        if (owns && !owns(b))
            continue;

        bool is_world = false;
        PhysicsColliderDescriptor collider;
        collider.stable_id = stable_id(b, is_world);
        collider.link_index = static_cast<size_t>(b);
        {
            const char* name = mj_id2name(model, mjOBJ_BODY, b);
            collider.link_name = name ? name : "<unnamed>";
        }

        collider.position = Vec3{ data->xpos[3 * b], data->xpos[3 * b + 1], data->xpos[3 * b + 2] };
        collider.orientation = Quat{ data->xquat[4 * b + 1], data->xquat[4 * b + 2],
                                     data->xquat[4 * b + 3], data->xquat[4 * b] };

        // ⚠ mj_objectVelocity with flg_local=0, so the twist is world-ALIGNED, matching invariant
        // 0. Reading `cvel` directly would give a velocity about the body's c-frame origin rather
        // than the body, which is the same numbers meaning something else.
        {
            mjtNum velocity[6];
            mj_objectVelocity(model, data, mjOBJ_BODY, b, velocity, /*flg_local=*/0);
            collider.angular_velocity = Vec3{ velocity[0], velocity[1], velocity[2] };
            collider.linear_velocity = Vec3{ velocity[3], velocity[4], velocity[5] };
        }

        LinkInertialDescriptor& inertial = collider.inertial;
        inertial.mass = model->body_mass[b];
        inertial.com_local = Vec3{ model->body_ipos[3 * b], model->body_ipos[3 * b + 1],
                                   model->body_ipos[3 * b + 2] };
        diagonalToBodyFrame(model->body_inertia + 3 * b, model->body_iquat + 4 * b,
                            inertial.inertia_local);
        inertial.provenance = InertiaProvenance::SolverRealised;
        inertial.is_articulated_effective_inertia = false;

        // Articulated, and configuration-independent: MuJoCo evaluates it at qpos0. Recorded for
        // the effective-inertia spike, explicitly not used as one.
        if (model->body_invweight0 != nullptr) {
            inertial.has_mean_inverse_inertia = true;
            inertial.mean_inverse_mass = model->body_invweight0[2 * b];
            inertial.mean_inverse_rotational_inertia = model->body_invweight0[2 * b + 1];
        }

        // Body-local geometry. The overlay wants world poses; a collider registration wants the
        // shapes in the frame of the body being registered, so the body transform is divided out
        // here rather than by every consumer.
        //
        // ⚠ Reuses the one geom walk rather than repeating the mjtGeom switch — a second copy is
        // how the view and the registry would start describing different shapes. It costs one pass
        // over `ngeom` per body, which is fine because this runs at scenario build, not per step.
        {
            CollisionDebugFilter everything;
            everything.include_world = true;
            everything.include_robots = true;
            CollisionDebugSnapshot geoms;
            readMuJoCoCollisionGeometry(model, data, everything,
                                        [b](int id, bool& world) {
                                            world = false;
                                            return id == b ? std::string("self")
                                                           : std::string("other");
                                        },
                                        geoms);

            mjtNum body_mat[9];
            mju_quat2Mat(body_mat, data->xquat + 4 * b);
            const mjtNum body_conj[4] = { data->xquat[4 * b], -data->xquat[4 * b + 1],
                                          -data->xquat[4 * b + 2], -data->xquat[4 * b + 3] };

            for (CollisionShape& shape : geoms.geoms) {
                if (shape.label != "self") continue;

                // world -> body-local: p_local = R_body^T * (p_world - t_body)
                const Vec3 d{ shape.position.x - collider.position.x,
                              shape.position.y - collider.position.y,
                              shape.position.z - collider.position.z };
                shape.position = Vec3{
                    body_mat[0] * d.x + body_mat[3] * d.y + body_mat[6] * d.z,
                    body_mat[1] * d.x + body_mat[4] * d.y + body_mat[7] * d.z,
                    body_mat[2] * d.x + body_mat[5] * d.y + body_mat[8] * d.z
                };

                const mjtNum shape_quat[4] = { shape.orientation.w, shape.orientation.x,
                                               shape.orientation.y, shape.orientation.z };
                mjtNum local_quat[4];
                mju_mulQuat(local_quat, body_conj, shape_quat);
                shape.orientation = Quat{ local_quat[1], local_quat[2], local_quat[3],
                                          local_quat[0] };

                shape.label = collider.stable_id;
                collider.shapes.push_back(std::move(shape));
            }
        }

        // ⚠ KinematicOneWay, unconditionally, and NOT because this code is unfinished. Plan §11.1
        // decides that M2 is kinematic to MPM precisely because the effective inertia above is not
        // available; promoting a link to DynamicTwoWay is gated on that spike, not on plumbing.
        collider.role = CouplingRole::KinematicOneWay;

        out.colliders.push_back(std::move(collider));
    }
}

} // namespace urdf

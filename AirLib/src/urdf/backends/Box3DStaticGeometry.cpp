#include "urdf/backends/Box3DStaticGeometry.hpp"

#include "urdf/backends/Box3DMath.hpp"

#include <chrono>
#include <map>
#include <mutex>

namespace b3urdf {

namespace {

/// Hull vertex budget, matching Box3DRobot's — a level's convex pieces are not more detailed than
/// a robot's, and one number is easier to reason about than two.
constexpr int kMaxHullVertices = 64;

std::mutex& cacheMutex()
{
    static std::mutex m;
    return m;
}

/// Live cooks, keyed on the identity of the StaticWorld they were cooked from. Weak, so the cook
/// dies with the last robot standing on it — a level swap must not keep the old level's triangles
/// resident forever.
std::map<const urdf::StaticWorld*, std::weak_ptr<const Box3DStaticGeometry>>& cache()
{
    static std::map<const urdf::StaticWorld*, std::weak_ptr<const Box3DStaticGeometry>> c;
    return c;
}

std::vector<b3Vec3> toB3Points(const std::vector<urdf::Vec3>& in)
{
    std::vector<b3Vec3> out;
    out.reserve(in.size());
    for (const urdf::Vec3& v : in)
        out.push_back(toB3(v));
    return out;
}

} // namespace

std::shared_ptr<const Box3DStaticGeometry> Box3DStaticGeometry::acquire(
    std::shared_ptr<const urdf::StaticWorld> world)
{
    if (!world || world->bodies.empty()) return nullptr;

    std::lock_guard<std::mutex> guard(cacheMutex());

    auto& c = cache();
    const auto it = c.find(world.get());
    if (it != c.end()) {
        if (std::shared_ptr<const Box3DStaticGeometry> live = it->second.lock())
            return live;  // the shared cook — this is where the 17.4 ms is saved
        c.erase(it);
    }

    // Not make_shared: the constructor is private, and a public factory that could be bypassed
    // would defeat the point of keying the cache on identity.
    std::shared_ptr<const Box3DStaticGeometry> cooked(new Box3DStaticGeometry(world));
    c[world.get()] = cooked;
    return cooked;
}

Box3DStaticGeometry::Box3DStaticGeometry(std::shared_ptr<const urdf::StaticWorld> world)
    : source_(std::move(world))
{
    cook();
}

Box3DStaticGeometry::~Box3DStaticGeometry()
{
    {
        std::lock_guard<std::mutex> guard(cacheMutex());
        const auto it = cache().find(source_.get());
        // Only erase our own entry: a cook for the same pointer could already have replaced it.
        if (it != cache().end() && it->second.expired()) cache().erase(it);
    }

    // ⚠ Every b3World holding shapes that reference these meshes must already be destroyed. That
    // is guaranteed by ownership, not by convention: Box3DRobot holds a shared_ptr to this object
    // and destroys its world in its own destructor, so this runs only after the last such world is
    // gone. Verified in tests/test_static_geometry.cpp that b3DestroyWorld leaves the mesh intact,
    // which is what makes that ordering the only one that has to hold.
    for (b3MeshData* m : meshes_)
        b3DestroyMesh(m);
}

void Box3DStaticGeometry::cook()
{
    const auto t0 = std::chrono::steady_clock::now();

    for (const urdf::StaticBody& body : source_->bodies) {
        for (const urdf::StaticShape& shape : body.shapes) {
            // Hulls are validated here even though they are built per world, so that a level's
            // rejected-shape count is a property of the cook and not something that grows every
            // time another robot attaches. A count that triples with three robots would be read as
            // "the level got worse", which is exactly the plausible-wrong-answer failure mode.
            if (shape.kind == urdf::StaticShapeKind::Hull) {
                if (shape.points.size() < 4) {
                    ++rejected_;
                    continue;
                }
                const std::vector<b3Vec3> pts = toB3Points(shape.points);
                b3HullData* probe =
                    b3CreateHull(pts.data(), static_cast<int>(pts.size()), kMaxHullVertices);
                if (probe == nullptr)
                    ++rejected_;
                else
                    b3DestroyHull(probe);
                continue;
            }

            if (shape.kind != urdf::StaticShapeKind::Mesh) continue;

            if (shape.points.size() < 3 || shape.indices.size() < 3) {
                ++rejected_;
                meshes_.push_back(nullptr);
                continue;
            }

            std::vector<b3Vec3> vertices = toB3Points(shape.points);
            std::vector<int32_t> indices(shape.indices.begin(), shape.indices.end());

            b3MeshDef def{};
            def.vertices = vertices.data();
            def.indices = indices.data();
            def.materialIndices = nullptr;
            def.weldTolerance = 0.0f;
            def.vertexCount = static_cast<int>(vertices.size());
            def.triangleCount = static_cast<int>(indices.size() / 3);
            def.weldVertices = false;
            def.useMedianSplit = false;
            // Adjacency information. Without it a body sliding across a tri-mesh catches on the
            // internal edges between triangles — the classic "ghost collision" — which on a level
            // floor reads as a rover randomly tripping over nothing.
            def.identifyEdges = true;

            b3MeshData* mesh = b3CreateMesh(&def, nullptr, 0);
            if (mesh == nullptr) ++rejected_;

            // A null is pushed on failure so the index stays in step with the mesh shapes
            // encountered during attachTo's walk. Losing that alignment would attach the wrong
            // geometry to the wrong actor, which looks like a level that is subtly mis-built.
            meshes_.push_back(mesh);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    cook_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void Box3DStaticGeometry::attachTo(b3WorldId world) const
{
    size_t mesh_index = 0;

    for (const urdf::StaticBody& body : source_->bodies) {
        if (body.shapes.empty()) continue;

        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_staticBody;
        bd.position = toB3Pos(body.position);
        bd.rotation = b3Quat{ b3Vec3{ static_cast<float>(body.orientation.x),
                                      static_cast<float>(body.orientation.y),
                                      static_cast<float>(body.orientation.z) },
                              static_cast<float>(body.orientation.w) };
        bd.name = body.name.c_str();

        const b3BodyId id = b3CreateBody(world, &bd);

        b3ShapeDef sd = b3DefaultShapeDef();
        // ⚠ TAGGED SO ONE LINK CAN OPT OUT OF IT (plan D10). Without a distinct category the
        // mirrored terrain is indistinguishable from every other shape, and "ignore the rigid
        // ground but keep colliding with everything else" cannot be expressed.
        sd.filter = b3urdf::filters::staticWorld();
        sd.baseMaterial.friction = static_cast<float>(body.friction);
        sd.baseMaterial.restitution = static_cast<float>(body.restitution);
        // Static geometry never contributes mass, and letting shapes touch a static body's mass
        // data is how a "static" object acquires an inertia tensor and stops being static.
        sd.updateBodyMass = false;

        for (const urdf::StaticShape& shape : body.shapes) {
            switch (shape.kind) {
            case urdf::StaticShapeKind::Mesh: {
                b3MeshData* mesh = (mesh_index < meshes_.size()) ? meshes_[mesh_index] : nullptr;
                ++mesh_index;
                if (mesh == nullptr) break;  // already counted as rejected at cook time
                // Scale is baked into the vertices by the mirror, so the shape scale is unit.
                b3CreateMeshShape(id, &sd, mesh, b3Vec3_one);
                break;
            }
            case urdf::StaticShapeKind::Hull: {
                if (shape.points.size() < 4) break;  // counted once, at cook time
                const std::vector<b3Vec3> pts = toB3Points(shape.points);
                // ⚠ Unlike a mesh, b3CreateHullShape CLONES the hull, so this one is built per
                // world and freed immediately. Caching it would leak; keeping it alive to match
                // the mesh path would be a misunderstanding of the API rather than a saving.
                b3HullData* hull =
                    b3CreateHull(pts.data(), static_cast<int>(pts.size()), kMaxHullVertices);
                if (hull == nullptr) break;
                b3CreateHullShape(id, &sd, hull);
                b3DestroyHull(hull);
                break;
            }
            case urdf::StaticShapeKind::Sphere: {
                b3Sphere s;
                s.center = toB3(shape.center_a);
                s.radius = static_cast<float>(shape.radius);
                b3CreateSphereShape(id, &sd, &s);
                break;
            }
            case urdf::StaticShapeKind::Capsule: {
                b3Capsule c;
                c.center1 = toB3(shape.center_a);
                c.center2 = toB3(shape.center_b);
                c.radius = static_cast<float>(shape.radius);
                b3CreateCapsuleShape(id, &sd, &c);
                break;
            }
            }
        }
    }
}

} // namespace b3urdf

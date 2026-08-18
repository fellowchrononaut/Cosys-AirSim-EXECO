#include "urdf/UrdfCollisionAudit.hpp"

#include "urdf/UrdfMesh.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace urdf {
namespace {

/// Rotation matrix from URDF fixed-axis rpy, composed R = Rz(yaw) Ry(pitch) Rx(roll).
void rpyToMatrix(const Vec3& rpy, double m[9])
{
    const double cr = std::cos(rpy.x), sr = std::sin(rpy.x);
    const double cp = std::cos(rpy.y), sp = std::sin(rpy.y);
    const double cy = std::cos(rpy.z), sy = std::sin(rpy.z);
    m[0] = cy * cp; m[1] = cy * sp * sr - sy * cr; m[2] = cy * sp * cr + sy * sr;
    m[3] = sy * cp; m[4] = sy * sp * sr + cy * cr; m[5] = sy * sp * cr - cy * sr;
    m[6] = -sp;     m[7] = cp * sr;                m[8] = cp * cr;
}

Vec3 transformPoint(const Origin& o, const Vec3& p)
{
    double m[9];
    rpyToMatrix(o.rpy, m);
    return Vec3{ m[0] * p.x + m[1] * p.y + m[2] * p.z + o.xyz.x,
                 m[3] * p.x + m[4] * p.y + m[5] * p.z + o.xyz.y,
                 m[6] * p.x + m[7] * p.y + m[8] * p.z + o.xyz.z };
}

/// Bounds of one geometry, expressed in the link frame.
///
/// Primitives are enumerated at their corners and transformed, which is exact for boxes and a
/// tight-enough envelope for cylinders and spheres under rotation. Meshes are transformed
/// vertex-by-vertex.
void accumulateGeometry(const Geometry& g, const Origin& origin, const std::string& urdf_dir,
                        const std::vector<std::string>& roots, Bounds& out, bool& unresolved)
{
    switch (g.type) {
    case GeometryType::Box: {
        const double hx = g.box_size.x * 0.5, hy = g.box_size.y * 0.5, hz = g.box_size.z * 0.5;
        for (int i = 0; i < 8; ++i)
            out.expand(transformPoint(origin, Vec3{ (i & 1) ? hx : -hx,
                                                    (i & 2) ? hy : -hy,
                                                    (i & 4) ? hz : -hz }));
        break;
    }
    case GeometryType::Cylinder: {
        // Envelope of the cylinder: the corners of the box that contains it. Conservative under
        // rotation, which is the right direction for a divergence screen.
        const double r = g.radius, h = g.length * 0.5;
        for (int i = 0; i < 8; ++i)
            out.expand(transformPoint(origin, Vec3{ (i & 1) ? r : -r,
                                                    (i & 2) ? r : -r,
                                                    (i & 4) ? h : -h }));
        break;
    }
    case GeometryType::Sphere: {
        const double r = g.radius;
        for (int i = 0; i < 8; ++i)
            out.expand(transformPoint(origin, Vec3{ (i & 1) ? r : -r,
                                                    (i & 2) ? r : -r,
                                                    (i & 4) ? r : -r }));
        break;
    }
    case GeometryType::Mesh: {
        const std::string path = resolveMeshPath(g.mesh_filename, urdf_dir, roots);
        if (path.empty()) {
            unresolved = true;
            return;
        }
        try {
            const MeshData mesh = loadMesh(path);
            for (const Vec3& v : mesh.vertices)
                out.expand(transformPoint(origin, Vec3{ v.x * g.mesh_scale.x,
                                                        v.y * g.mesh_scale.y,
                                                        v.z * g.mesh_scale.z }));
        }
        catch (const std::exception&) {
            // A mesh that exists but cannot be read is the same kind of gap as one that is
            // missing, and must not be confused with a link that legitimately has no geometry.
            unresolved = true;
        }
        break;
    }
    }
}

double overlapVolume(const Bounds& a, const Bounds& b)
{
    if (!a.valid || !b.valid) return 0;
    const double dx = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
    const double dy = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
    const double dz = std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z);
    if (dx <= 0 || dy <= 0 || dz <= 0) return 0;
    return dx * dy * dz;
}

std::string fixed(double v, int places)
{
    std::ostringstream s;
    s << std::fixed << std::setprecision(places) << v;
    return s.str();
}

} // namespace

void Bounds::expand(const Vec3& p)
{
    if (!valid) {
        min = max = p;
        valid = true;
        return;
    }
    min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
}

Vec3 Bounds::extents() const
{
    if (!valid) return Vec3{ 0, 0, 0 };
    return Vec3{ max.x - min.x, max.y - min.y, max.z - min.z };
}

double Bounds::volume() const
{
    const Vec3 e = extents();
    return e.x * e.y * e.z;
}

double Bounds::radius() const
{
    const Vec3 e = extents();
    return 0.5 * std::max(e.x, std::max(e.y, e.z));
}

CollisionAudit auditCollisionConsistency(const Robot& model, const std::string& urdf_dir,
                                         const std::vector<std::string>& search_roots)
{
    CollisionAudit a;
    a.robot = model.name;
    a.links_total = static_cast<int>(model.links.size());

    for (size_t i = 0; i < model.links.size(); ++i) {
        const Link& l = model.links[i];
        LinkAudit la;
        la.link = static_cast<int>(i);
        la.name = l.name;
        la.has_collision = !l.collisions.empty();
        la.has_visual = !l.visuals.empty();
        la.mass = l.has_inertial ? l.inertial.mass : 0.0;

        bool unresolved = false;
        for (const Collision& c : l.collisions)
            accumulateGeometry(c.geometry, c.origin, urdf_dir, search_roots, la.collision, unresolved);
        for (const Visual& v : l.visuals)
            accumulateGeometry(v.geometry, v.origin, urdf_dir, search_roots, la.visual, unresolved);
        la.mesh_unresolved = unresolved;

        // How much of what the sensors can see is outside anything the solver can push on.
        if (la.visual.valid) {
            const double vis = la.visual.volume();
            if (vis > 0)
                la.uncovered_fraction = std::max(0.0, 1.0 - overlapVolume(la.visual, la.collision) / vis);
            else
                la.uncovered_fraction = la.collision.valid ? 0.0 : 1.0;
        }

        a.mass_total += la.mass;
        if (la.has_collision) {
            ++a.links_with_collision;
            a.mass_with_collision += la.mass;
        }
        if (la.has_visual) ++a.links_with_visual;
        if (la.has_visual && !la.has_collision) ++a.links_visual_only;
        if (la.mesh_unresolved) ++a.links_unresolved_mesh;

        if (la.uncovered_fraction > a.worst_uncovered_fraction) {
            a.worst_uncovered_fraction = la.uncovered_fraction;
            a.worst_link = la.name;
        }

        a.links.push_back(std::move(la));
    }

    return a;
}

std::string CollisionAudit::report() const
{
    std::ostringstream s;
    s << "R2 collision-consistency audit - robot '" << robot << "'\n"
      << "  Box3D drives on <collision>; Unreal sensors trace <visual>. This is where they differ.\n\n";

    s << "  " << std::left << std::setw(28) << "link" << std::setw(11) << "collision"
      << std::setw(9) << "visual" << std::setw(11) << "mass(g)" << "uncovered\n";
    s << "  " << std::string(66, '-') << "\n";

    for (const LinkAudit& l : links) {
        // Only links that actually diverge, plus anything with a broken mesh. A full listing of a
        // 23-link rover buries the six interesting rows.
        const bool interesting = (l.has_visual && !l.has_collision) || l.mesh_unresolved ||
                                 l.uncovered_fraction > 0.5;
        if (!interesting) continue;
        s << "  " << std::left << std::setw(28) << l.name
          << std::setw(11) << (l.has_collision ? "yes" : "NO")
          << std::setw(9) << (l.has_visual ? "yes" : "no")
          << std::setw(11) << fixed(l.mass * 1000.0, 1)
          << fixed(l.uncovered_fraction * 100.0, 1) << " %"
          << (l.mesh_unresolved ? "   <mesh NOT FOUND>" : "") << "\n";
    }

    s << "\n  links                     : " << links_total
      << "\n  with <collision>          : " << links_with_collision
      << "  (" << fixed(100.0 * links_with_collision / std::max(links_total, 1), 1) << " %)"
      << "\n  visible but un-simulated  : " << links_visual_only
      << "\n  unresolved <mesh>         : " << links_unresolved_mesh
      << "\n  mass on collidable links  : " << fixed(mass_with_collision, 4) << " / "
      << fixed(mass_total, 4) << " kg"
      << "\n  mass with NO collision    : " << fixed(100.0 * uncollidable_mass_fraction(), 1) << " %"
      << "\n  worst-diverging link      : " << (worst_link.empty() ? "(none)" : worst_link)
      << "  (" << fixed(100.0 * worst_uncovered_fraction, 1) << " % of its visual volume "
      << "outside any collision shape)\n";

    if (links_visual_only > 0)
        s << "\n  => " << links_visual_only << " link(s) will be seen by LiDAR and passed through by "
             "the solver.\n     This is R2. It is not necessarily wrong - it is what the URDF says - "
             "but it must be\n     known before trusting either the contact behaviour or the range "
             "returns.\n";

    return s.str();
}

} // namespace urdf

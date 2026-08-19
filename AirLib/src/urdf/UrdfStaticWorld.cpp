// OBJ export for mirrored level geometry.
//
// ⚠ This file exists ONLY so the two engines' collision geometry can be looked at side by side.
// Box3D cooks the level's real triangles; MuJoCo can only take convex shapes and therefore
// approximates. Judging that approximation by argument failed repeatedly today — exporting both
// and opening them in a mesh viewer does not.
#include "urdf/UrdfStaticWorld.hpp"

#include <fstream>
#include <string>


namespace urdf {

bool writeStaticWorldObj(const StaticWorld& world, const std::string& path)
{
    std::ofstream f(path);
    if (!f) return false;

    f << "# Box3D static collision geometry - the level's REAL triangles, cooked concavely.\n";
    f << "# bodies=" << world.bodies.size() << " shapes=" << world.shapeCount()
      << " triangles=" << world.triangleCount() << "\n";

    auto toWorld = [](const Quat& q, const Vec3& t, const Vec3& p) {
        const double x = q.x, y = q.y, z = q.z, w = q.w;
        const double xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z;
        const double wx = w*x, wy = w*y, wz = w*z;
        return Vec3{ t.x + p.x*(1-2*(yy+zz)) + p.y*(2*(xy-wz))   + p.z*(2*(xz+wy)),
                     t.y + p.x*(2*(xy+wz))   + p.y*(1-2*(xx+zz)) + p.z*(2*(yz-wx)),
                     t.z + p.x*(2*(xz-wy))   + p.y*(2*(yz+wx))   + p.z*(1-2*(xx+yy)) };
    };

    int vbase = 1;
    int body_index = 0;
    for (const StaticBody& b : world.bodies) {
        int shape_index = 0;
        for (const StaticShape& sh : b.shapes) {
            f << "o body" << body_index << "_shape" << shape_index
              << "_kind" << static_cast<int>(sh.kind) << "_" << b.name << "\n";
            for (const Vec3& p : sh.points) {
                const Vec3 w = toWorld(b.orientation, b.position, p);
                f << "v " << w.x << " " << w.y << " " << w.z << "\n";
            }
            if (!sh.indices.empty()) {
                for (size_t t = 0; t + 2 < sh.indices.size(); t += 3)
                    f << "f " << (vbase + sh.indices[t]) << " " << (vbase + sh.indices[t + 1])
                      << " " << (vbase + sh.indices[t + 2]) << "\n";
            }
            // A Hull has points but no faces; the point cloud alone still shows its extent.
            vbase += static_cast<int>(sh.points.size());
            ++shape_index;
        }
        ++body_index;
    }
    return true;
}

} // namespace urdf

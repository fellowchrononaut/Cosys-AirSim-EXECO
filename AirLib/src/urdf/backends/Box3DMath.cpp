#include "urdf/backends/Box3DMath.hpp"

namespace b3urdf {

void rotateInertia(const double in[9], const b3Quat& q, double out[9])
{
    // Quaternion -> rotation matrix, row-major.
    const double x = q.v.x, y = q.v.y, z = q.v.z, w = q.s;
    const double R[9] = {
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)
    };

    // tmp = R * I
    double tmp[9];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += R[r * 3 + k] * in[k * 3 + c];
            tmp[r * 3 + c] = s;
        }

    // out = tmp * R^T
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += tmp[r * 3 + k] * R[c * 3 + k];
            out[r * 3 + c] = s;
        }
}

} // namespace b3urdf

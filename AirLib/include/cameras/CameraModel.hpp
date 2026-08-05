// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_cameras_CameraModel_hpp
#define airsim_core_cameras_CameraModel_hpp

// Generic camera models — unprojection only (pixel -> ray).
//
// Header-only and dependency-free on purpose: cmake/AirLib/CMakeLists.txt globs a
// *named* list of src/ subdirectories, so a .cpp added under AirLib/src/cameras/ would
// never be compiled and nothing would report it. Everything here stays in the header.
//
// The raymap is built once per camera at configuration time, so an iterative
// unprojection costs nothing at run time; the models are not contorted to avoid it.
//
// ---------------------------------------------------------------------------------
// CONVENTIONS — the Python reference in tools/raymap_check.py repeats these verbatim
// ---------------------------------------------------------------------------------
//
// Pixel centres are at INTEGER image coordinates: pixel (px, py) has its centre at
// continuous image coordinates (u, v) = (px, py). This is the OpenCV / Kalibr /
// ScanNet++ convention, i.e. the convention the cx, cy we are handed are expressed
// in, so a calibration can be transcribed rather than converted. It is NOT the
// graphics convention (SV_Position, which puts centres at half-integers); converting
// between the two is a texel-addressing concern for the consumer, not a change to the
// numbers stored here.
//
// Rays are in the right-handed optical camera frame: +x right along an image row,
// +y down an image column, +z forward out of the lens. The camera-to-world transform,
// including the optical -> Unreal axis change, is applied by the consumer; keeping the
// raymap in the calibration's own frame is what makes it checkable against OpenCV.
//
// Directions are unit length. A ray the model cannot unproject (outside its valid
// domain — real for wide-FOV Double Sphere at the image corners) is returned invalid
// and stored with an all-zero direction.
//
// Per ADR-001 a ray is (origin, direction) and never a direction alone. For every
// model here the origin is constant across the image, so the values are redundant
// today. They are stored anyway.

#include <cmath>
#include <limits>
#include <string>

namespace msr
{
namespace airlib
{
    namespace cameras
    {

        static constexpr double kPi = 3.14159265358979323846;

        enum class CameraModelType
        {
            None = 0, //no CameraModel block — the existing pinhole path, untouched
            Pinhole,
            KannalaBrandt, //OPENCV_FISHEYE, k1..k4
            DoubleSphere, //Kalibr 'ds', [xi, alpha, fx, fy, cx, cy]
            Raymap //per-pixel (origin, direction) loaded from a file
        };

        inline const char* toString(CameraModelType type)
        {
            switch (type) {
            case CameraModelType::Pinhole:
                return "Pinhole";
            case CameraModelType::KannalaBrandt:
                return "KannalaBrandt";
            case CameraModelType::DoubleSphere:
                return "DoubleSphere";
            case CameraModelType::Raymap:
                return "Raymap";
            default:
                return "None";
            }
        }

        // Kannala-Brandt has no closed-form inverse: theta must be recovered from the
        // distorted radius by root finding. The raymap is built once, offline, so the
        // tolerance is set by what is representable rather than by any time budget:
        // 1e-12 rad against a focal length of ~617 px/rad is ~6e-10 px, four orders
        // below float32 storage quantisation and six below the sub-pixel gate.
        struct SolverSettings
        {
            double tolerance = 1e-12; //radians, on the residual and on the step
            int max_iterations = 50; //safeguarded Newton needs <10 on real calibrations
        };

        struct CameraModelParams
        {
            CameraModelType type = CameraModelType::None;

            unsigned int width = 0;
            unsigned int height = 0;

            //intrinsics; NaN means "not supplied" (Pinhole then derives them from fov_degrees)
            double fx = std::numeric_limits<double>::quiet_NaN();
            double fy = std::numeric_limits<double>::quiet_NaN();
            double cx = std::numeric_limits<double>::quiet_NaN();
            double cy = std::numeric_limits<double>::quiet_NaN();

            //Kannala-Brandt (OPENCV_FISHEYE)
            double k1 = 0.0, k2 = 0.0, k3 = 0.0, k4 = 0.0;

            //Double Sphere (Kalibr order is [xi, alpha, fx, fy, cx, cy])
            double xi = 0.0;
            double alpha = 0.0;

            //Pinhole only: horizontal FOV, the existing FOV_Degrees behaviour
            double fov_degrees = std::numeric_limits<double>::quiet_NaN();

            //Type == Raymap
            std::string raymap_path;

            SolverSettings solver;

            //Derived by resolveParams, not read from settings: the largest incidence angle
            //at which the Kannala-Brandt polynomial is still invertible, i.e. its first
            //stationary point. See kbMaxTheta. pi is the fallback for an unresolved params.
            double kb_max_theta = kPi;
        };

        struct Ray
        {
            //ADR-001: (origin, direction), never direction alone
            double ox = 0.0, oy = 0.0, oz = 0.0;
            double dx = 0.0, dy = 0.0, dz = 0.0;
            bool valid = false;
        };

        namespace detail
        {

            inline bool isSet(double v)
            {
                return !std::isnan(v);
            }

            //theta_d = theta * (1 + k1 t^2 + k2 t^4 + k3 t^6 + k4 t^8)
            inline double kbForward(const CameraModelParams& p, double theta)
            {
                const double t2 = theta * theta;
                return theta * (1.0 + t2 * (p.k1 + t2 * (p.k2 + t2 * (p.k3 + t2 * p.k4))));
            }

            inline double kbForwardDerivative(const CameraModelParams& p, double theta)
            {
                const double t2 = theta * theta;
                return 1.0 + t2 * (3.0 * p.k1 + t2 * (5.0 * p.k2 + t2 * (7.0 * p.k3 + t2 * 9.0 * p.k4)));
            }

            // The largest incidence angle at which theta -> theta_d is still invertible:
            // the first stationary point of the KB polynomial on (0, pi], or pi if there
            // is none.
            //
            // This is not a nicety. For the ScanNet++ DSLR calibration the polynomial
            // turns over near theta = 2.7 rad, and f(pi) = 1.60 is BELOW f at the image
            // corner (theta_d = 1.71) even though the corner's true root sits at
            // theta = 1.475, comfortably inside the monotonic range. Bracketing on
            // [0, pi] therefore rejects 18401 perfectly valid corner texels (measured).
            // Beyond the stationary point the model is genuinely non-invertible — two
            // incidence angles share one radius — so [0, kbMaxTheta] is both the correct
            // bracket and the honest valid domain.
            inline double kbMaxTheta(const CameraModelParams& p)
            {
                const int kScanSteps = 256;
                double previous = 0.0; //f'(0) == 1 > 0 always
                for (int i = 1; i <= kScanSteps; ++i) {
                    const double theta = kPi * i / kScanSteps;
                    if (kbForwardDerivative(p, theta) <= 0.0) {
                        double lo = previous, hi = theta;
                        for (int j = 0; j < 64; ++j) {
                            const double mid = 0.5 * (lo + hi);
                            if (kbForwardDerivative(p, mid) > 0.0)
                                lo = mid;
                            else
                                hi = mid;
                        }
                        return lo;
                    }
                    previous = theta;
                }
                return kPi;
            }

            // Safeguarded Newton: Newton where it behaves, bisection where it does not.
            // Plain Newton is enough for every calibration we hold, but the KB polynomial
            // is not monotonic for arbitrary k1..k4 and a raymap must not contain a
            // silently wrong ray, so the bracket [0, kb_max_theta] is maintained
            // throughout and a step that leaves it is replaced by a bisection step.
            inline bool kbSolveTheta(const CameraModelParams& p, double theta_d,
                                     double& theta_out, int& iterations_out)
            {
                iterations_out = 0;
                if (theta_d <= 0.0) {
                    theta_out = 0.0;
                    return true;
                }

                const double kThetaMax = p.kb_max_theta;
                double lo = 0.0, hi = kThetaMax;
                if (kbForward(p, hi) < theta_d)
                    return false; //past the lens' invertible incidence angle

                double theta = theta_d < kThetaMax ? theta_d : kThetaMax;
                for (int i = 0; i < p.solver.max_iterations; ++i) {
                    ++iterations_out;
                    const double f = kbForward(p, theta) - theta_d;
                    if (std::abs(f) <= p.solver.tolerance) {
                        theta_out = theta;
                        return true;
                    }
                    if (f > 0.0)
                        hi = theta;
                    else
                        lo = theta;
                    if (hi - lo <= p.solver.tolerance) {
                        theta_out = 0.5 * (lo + hi); //bracket collapsed: as good as it gets
                        return true;
                    }

                    const double df = kbForwardDerivative(p, theta);
                    double next = df > 0.0 ? theta - f / df : 0.5 * (lo + hi);
                    if (!(next > lo && next < hi))
                        next = 0.5 * (lo + hi); //Newton left the bracket — bisect instead
                    theta = next;
                }
                theta_out = theta;
                return std::abs(kbForward(p, theta) - theta_d) <= p.solver.tolerance;
            }

        } //namespace detail

        // Fills in whatever the schema allows to be omitted and rejects what cannot be.
        // Returns false with a message rather than throwing, so the raymap tool and the
        // settings parser can each report in their own idiom.
        inline bool resolveParams(CameraModelParams& p, std::string& error)
        {
            if (p.type == CameraModelType::None) {
                error = "camera model type is None";
                return false;
            }
            if (p.type == CameraModelType::Raymap) {
                if (p.raymap_path.empty()) {
                    error = "Raymap camera model needs a Path";
                    return false;
                }
                return true;
            }
            if (p.width == 0 || p.height == 0) {
                error = "CameraModel needs a non-zero Width and Height";
                return false;
            }

            if (p.type == CameraModelType::Pinhole) {
                // Reduction to the existing FOV_Degrees behaviour. Unreal maps NDC x in
                // [-1,1] onto continuous viewport x in [0,W]; pixel px covers [px, px+1]
                // so its centre is at continuous x = px + 0.5. Under the integer-pixel-
                // centre convention above, pixel coordinate = continuous - 0.5, so NDC 0
                // lands on W/2 - 0.5 = (W-1)/2 — and fx = (W/2) / tan(fov/2) exactly.
                // Square pixels (fy == fx) is what an Unreal perspective capture does;
                // the vertical FOV follows from the aspect ratio.
                if (!detail::isSet(p.fx) || !detail::isSet(p.fy)) {
                    if (!detail::isSet(p.fov_degrees)) {
                        error = "Pinhole CameraModel needs either fx/fy or FOV_Degrees";
                        return false;
                    }
                    if (!(p.fov_degrees > 0.0) || p.fov_degrees >= 180.0) {
                        error = "Pinhole FOV_Degrees must be in (0, 180)";
                        return false;
                    }
                    const double f = 0.5 * static_cast<double>(p.width) /
                                     std::tan(0.5 * p.fov_degrees * 3.14159265358979323846 / 180.0);
                    p.fx = f;
                    p.fy = f;
                }
                if (!detail::isSet(p.cx))
                    p.cx = 0.5 * (static_cast<double>(p.width) - 1.0);
                if (!detail::isSet(p.cy))
                    p.cy = 0.5 * (static_cast<double>(p.height) - 1.0);
            }

            if (!detail::isSet(p.fx) || !detail::isSet(p.fy) ||
                !detail::isSet(p.cx) || !detail::isSet(p.cy)) {
                error = std::string(toString(p.type)) + " CameraModel needs fx, fy, cx and cy";
                return false;
            }
            if (p.fx == 0.0 || p.fy == 0.0) {
                error = "CameraModel fx and fy must be non-zero";
                return false;
            }
            if (p.type == CameraModelType::KannalaBrandt)
                p.kb_max_theta = detail::kbMaxTheta(p);
            if (p.type == CameraModelType::DoubleSphere) {
                if (!(p.alpha >= 0.0 && p.alpha <= 1.0)) {
                    error = "DoubleSphere alpha must be in [0, 1]";
                    return false;
                }
                if (p.alpha == 1.0) {
                    error = "DoubleSphere alpha must be < 1";
                    return false;
                }
            }
            return true;
        }

        // ---------------------------------------------------------------------------
        // unprojection — (u, v) in pixel coordinates, integer coordinates are centres
        // ---------------------------------------------------------------------------

        inline Ray unprojectPinhole(const CameraModelParams& p, double u, double v)
        {
            Ray ray;
            const double mx = (u - p.cx) / p.fx;
            const double my = (v - p.cy) / p.fy;
            const double n = std::sqrt(mx * mx + my * my + 1.0);
            ray.dx = mx / n;
            ray.dy = my / n;
            ray.dz = 1.0 / n;
            ray.valid = true;
            return ray;
        }

        inline Ray unprojectKannalaBrandt(const CameraModelParams& p, double u, double v)
        {
            Ray ray;
            const double mx = (u - p.cx) / p.fx;
            const double my = (v - p.cy) / p.fy;
            const double theta_d = std::sqrt(mx * mx + my * my); //the distorted radius IS theta_d

            if (theta_d == 0.0) {
                ray.dz = 1.0;
                ray.valid = true;
                return ray;
            }

            double theta = 0.0;
            int iterations = 0;
            if (!detail::kbSolveTheta(p, theta_d, theta, iterations))
                return ray; //invalid: zero direction

            const double s = std::sin(theta) / theta_d;
            ray.dx = s * mx;
            ray.dy = s * my;
            ray.dz = std::cos(theta);
            ray.valid = true;
            return ray;
        }

        // Usenko, Demmel & Cremers, "The Double Sphere Camera Model" (3DV 2018), the
        // closed-form unprojection of eqs. (47)-(49). Closed form is the model's main
        // advantage over KB and the reason both of our real rigs are calibrated in it.
        inline Ray unprojectDoubleSphere(const CameraModelParams& p, double u, double v)
        {
            Ray ray;
            const double mx = (u - p.cx) / p.fx;
            const double my = (v - p.cy) / p.fy;
            const double r2 = mx * mx + my * my;

            //valid domain: for alpha > 0.5 the image plane is bounded
            if (p.alpha > 0.5 && r2 >= 1.0 / (2.0 * p.alpha - 1.0))
                return ray;

            const double under = 1.0 - (2.0 * p.alpha - 1.0) * r2;
            if (under < 0.0)
                return ray;

            const double mz = (1.0 - p.alpha * p.alpha * r2) /
                              (p.alpha * std::sqrt(under) + 1.0 - p.alpha);

            const double inner = mz * mz + (1.0 - p.xi * p.xi) * r2;
            if (inner < 0.0)
                return ray;

            const double denom = mz * mz + r2;
            if (denom == 0.0)
                return ray;

            const double k = (mz * p.xi + std::sqrt(inner)) / denom;
            double dx = k * mx;
            double dy = k * my;
            double dz = k * mz - p.xi;

            const double n = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (!(n > 0.0))
                return ray;

            //analytically unit already; normalising only removes accumulated round-off
            ray.dx = dx / n;
            ray.dy = dy / n;
            ray.dz = dz / n;
            ray.valid = true;
            return ray;
        }

        inline Ray unproject(const CameraModelParams& p, double u, double v)
        {
            switch (p.type) {
            case CameraModelType::Pinhole:
                return unprojectPinhole(p, u, v);
            case CameraModelType::KannalaBrandt:
                return unprojectKannalaBrandt(p, u, v);
            case CameraModelType::DoubleSphere:
                return unprojectDoubleSphere(p, u, v);
            default:
                return Ray(); //Raymap is loaded, not evaluated; None has no rays
            }
        }
    }
}
} //namespace

#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_cameras_Raymap_hpp
#define airsim_core_cameras_Raymap_hpp

// Per-texel viewing rays for a generic camera, plus the debug dump that makes the
// camera maths checkable offline.
//
// Header-only for the same reason as CameraModel.hpp — see the note there.
//
// ADR-001: six floats per texel, (origin, direction), never direction alone. For every
// model in scope today the origin is constant across the image so the values are
// redundant now; they are stored anyway, because the cost of getting a released file
// format wrong is a migration and the cost of carrying three floats is not. A later
// bCentralCamera fast path may let a shader skip the origin *fetch*; it may not change
// the *format*.
//
// ---------------------------------------------------------------------------------
// DUMP FORMAT (little endian; tools/raymap_check.py reads exactly this)
// ---------------------------------------------------------------------------------
//   off  size  content
//     0     8  magic  "AIRRAYM1" (ASCII, no NUL terminator)
//     8     4  uint32 version   = 1
//    12     4  uint32 width
//    16     4  uint32 height
//    20     4  uint32 channels  = 6
//    24     4  uint32 dtype     = 0 (float32) | 1 (float64)
//    28     4  uint32 flags     = bit0 set -> central camera (all origins equal)
//    32     4  uint32 reserved0 = 0
//    36     4  uint32 reserved1 = 0
//    40   ...  width*height*6 values, row major (y outer, x inner),
//              per texel: ox, oy, oz, dx, dy, dz
//
// float32 is the default because it is what the raymap texture will hold, so a check
// run against a float32 dump measures the error the renderer will actually see.
// float64 exists to separate model error from storage quantisation when that matters.
//
// A texel whose direction is all zero is one the model could not unproject: outside
// its valid domain. That is a real case, not a failure — a 1328x1328 Double Sphere
// fisheye has image corners outside the lens circle.

#include "CameraModel.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{
    namespace cameras
    {

        struct Raymap
        {
            static constexpr unsigned int kChannels = 6; //ADR-001: origin + direction

            unsigned int width = 0;
            unsigned int height = 0;
            bool central = true; //all origins identical — a fetch hint, never a format change
            std::vector<double> data; //width * height * kChannels, row major

            void resize(unsigned int w, unsigned int h)
            {
                width = w;
                height = h;
                data.assign(static_cast<size_t>(w) * h * kChannels, 0.0);
            }

            size_t index(unsigned int x, unsigned int y) const
            {
                return (static_cast<size_t>(y) * width + x) * kChannels;
            }

            Ray at(unsigned int x, unsigned int y) const
            {
                const double* t = &data[index(x, y)];
                Ray ray;
                ray.ox = t[0];
                ray.oy = t[1];
                ray.oz = t[2];
                ray.dx = t[3];
                ray.dy = t[4];
                ray.dz = t[5];
                ray.valid = !(t[3] == 0.0 && t[4] == 0.0 && t[5] == 0.0);
                return ray;
            }

            void set(unsigned int x, unsigned int y, const Ray& ray)
            {
                double* t = &data[index(x, y)];
                t[0] = ray.ox;
                t[1] = ray.oy;
                t[2] = ray.oz;
                if (ray.valid) {
                    t[3] = ray.dx;
                    t[4] = ray.dy;
                    t[5] = ray.dz;
                }
                else {
                    t[3] = t[4] = t[5] = 0.0; //invalid texels carry a zero direction
                }
            }
        };

        struct RaymapStats
        {
            size_t texels = 0;
            size_t invalid = 0; //outside the model's valid domain
        };

        namespace detail
        {

            inline void writeU32(std::ostream& s, uint32_t v)
            {
                //little endian, written byte by byte so the file does not depend on the host
                unsigned char b[4] = { static_cast<unsigned char>(v & 0xFFu),
                                       static_cast<unsigned char>((v >> 8) & 0xFFu),
                                       static_cast<unsigned char>((v >> 16) & 0xFFu),
                                       static_cast<unsigned char>((v >> 24) & 0xFFu) };
                s.write(reinterpret_cast<const char*>(b), 4);
            }

            inline uint32_t readU32(const unsigned char* b)
            {
                return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
                       (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
            }

        } //namespace detail

        static constexpr char kRaymapMagic[9] = "AIRRAYM1";
        static constexpr uint32_t kRaymapVersion = 1;
        static constexpr size_t kRaymapHeaderBytes = 40;

        // Builds the raymap for a resolved CameraModelParams. Type == Raymap loads the
        // file instead; that is the escape hatch for any calibration the named models
        // cannot express, and the only way to guarantee pixel-exact agreement with how a
        // GEER checkpoint was trained.
        inline bool buildRaymap(const CameraModelParams& params, Raymap& out,
                                RaymapStats& stats, std::string& error);

        inline bool readRaymap(const std::string& path, Raymap& out, std::string& error)
        {
            std::ifstream f(path, std::ios::binary);
            if (!f) {
                error = "cannot open raymap file " + path;
                return false;
            }
            unsigned char header[kRaymapHeaderBytes];
            f.read(reinterpret_cast<char*>(header), kRaymapHeaderBytes);
            if (f.gcount() != static_cast<std::streamsize>(kRaymapHeaderBytes)) {
                error = "raymap file too short for a header: " + path;
                return false;
            }
            if (std::memcmp(header, kRaymapMagic, 8) != 0) {
                error = "not a raymap file (bad magic): " + path;
                return false;
            }
            const uint32_t version = detail::readU32(header + 8);
            const uint32_t width = detail::readU32(header + 12);
            const uint32_t height = detail::readU32(header + 16);
            const uint32_t channels = detail::readU32(header + 20);
            const uint32_t dtype = detail::readU32(header + 24);
            const uint32_t flags = detail::readU32(header + 28);
            if (version != kRaymapVersion) {
                error = "unsupported raymap version in " + path;
                return false;
            }
            if (channels != Raymap::kChannels) {
                error = "raymap must carry 6 channels per texel (ADR-001): " + path;
                return false;
            }
            if (dtype > 1 || width == 0 || height == 0) {
                error = "bad raymap header in " + path;
                return false;
            }

            out.resize(width, height);
            out.central = (flags & 1u) != 0;
            const size_t count = out.data.size();
            if (dtype == 0) {
                std::vector<float> buf(count);
                f.read(reinterpret_cast<char*>(buf.data()),
                       static_cast<std::streamsize>(count * sizeof(float)));
                if (f.gcount() != static_cast<std::streamsize>(count * sizeof(float))) {
                    error = "raymap payload truncated in " + path;
                    return false;
                }
                for (size_t i = 0; i < count; ++i)
                    out.data[i] = static_cast<double>(buf[i]);
            }
            else {
                f.read(reinterpret_cast<char*>(out.data.data()),
                       static_cast<std::streamsize>(count * sizeof(double)));
                if (f.gcount() != static_cast<std::streamsize>(count * sizeof(double))) {
                    error = "raymap payload truncated in " + path;
                    return false;
                }
            }
            return true;
        }

        inline bool writeRaymap(const Raymap& map, const std::string& path,
                                bool as_float64, std::string& error)
        {
            std::ofstream f(path, std::ios::binary);
            if (!f) {
                error = "cannot open " + path + " for writing";
                return false;
            }
            f.write(kRaymapMagic, 8);
            detail::writeU32(f, kRaymapVersion);
            detail::writeU32(f, map.width);
            detail::writeU32(f, map.height);
            detail::writeU32(f, Raymap::kChannels);
            detail::writeU32(f, as_float64 ? 1u : 0u);
            detail::writeU32(f, map.central ? 1u : 0u);
            detail::writeU32(f, 0u);
            detail::writeU32(f, 0u);

            if (as_float64) {
                f.write(reinterpret_cast<const char*>(map.data.data()),
                        static_cast<std::streamsize>(map.data.size() * sizeof(double)));
            }
            else {
                std::vector<float> buf(map.data.size());
                for (size_t i = 0; i < map.data.size(); ++i)
                    buf[i] = static_cast<float>(map.data[i]);
                f.write(reinterpret_cast<const char*>(buf.data()),
                        static_cast<std::streamsize>(buf.size() * sizeof(float)));
            }
            if (!f) {
                error = "write failed for " + path;
                return false;
            }
            return true;
        }

        inline bool buildRaymap(const CameraModelParams& params, Raymap& out,
                                RaymapStats& stats, std::string& error)
        {
            CameraModelParams p = params;
            if (!resolveParams(p, error))
                return false;

            if (p.type == CameraModelType::Raymap) {
                if (!readRaymap(p.raymap_path, out, error))
                    return false;
                stats.texels = static_cast<size_t>(out.width) * out.height;
                stats.invalid = 0;
                for (unsigned int y = 0; y < out.height; ++y)
                    for (unsigned int x = 0; x < out.width; ++x)
                        if (!out.at(x, y).valid)
                            ++stats.invalid;
                return true;
            }

            out.resize(p.width, p.height);
            out.central = true; //every model in the v1 set is central; the format is not
            stats.texels = static_cast<size_t>(p.width) * p.height;
            stats.invalid = 0;

            for (unsigned int y = 0; y < p.height; ++y) {
                for (unsigned int x = 0; x < p.width; ++x) {
                    //integer pixel coordinates ARE pixel centres — see CameraModel.hpp
                    Ray ray = unproject(p, static_cast<double>(x), static_cast<double>(y));
                    //origin is the optical centre; constant here, per-texel by construction
                    ray.ox = 0.0;
                    ray.oy = 0.0;
                    ray.oz = 0.0;
                    if (!ray.valid)
                        ++stats.invalid;
                    out.set(x, y, ray);
                }
            }
            return true;
        }
    }
}
} //namespace

#endif

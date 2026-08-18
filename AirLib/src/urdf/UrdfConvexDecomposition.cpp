#include "urdf/UrdfConvexDecomposition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <array>
#include <chrono>

#if WITH_COACD_BINDING
#include "coacd.h"
#endif

namespace urdf {
namespace {

/// ⚠ Bump on ANY change to the cache format OR to how a decomposition is produced. The key covers
/// the mesh and the options; it cannot cover "we changed which library we call, or fixed a bug in
/// the welding". Without this a stale entry from an older, wrong implementation is served forever
/// and looks like the new code failing to take effect.
constexpr uint32_t kCacheFormatVersion = 1;

/// FNV-1a. Chosen because it is ten lines and needs no dependency: the cache key must not be a
/// reason to link a hashing library into AirLib. Collisions are a cache-correctness risk rather
/// than a security one, and 64 bits over a few thousand meshes is far past adequate.
inline void hashBytes(uint64_t& h, const void* data, size_t n)
{
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
}

/// ⚠ Quantise before hashing. Vertices arrive as doubles from three different loaders (STL, DAE,
/// OBJ) and the same mesh re-exported can differ in the last bit, which would miss the cache every
/// time while looking like the cache simply not working. 1 micrometre is far below any collision
/// tolerance we care about.
inline long long quantise(double v)
{
    return static_cast<long long>(v * 1e6 + (v < 0 ? -0.5 : 0.5));
}

std::string cachePath(const std::string& dir, uint64_t key)
{
    char name[32];
    std::snprintf(name, sizeof(name), "%016llx.cvx", static_cast<unsigned long long>(key));
    return dir + "/" + name;
}

bool readCache(const std::string& path, std::vector<ConvexPart>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    uint32_t version = 0;
    uint32_t part_count = 0;
    f.read(reinterpret_cast<char*>(&version), sizeof(version));
    f.read(reinterpret_cast<char*>(&part_count), sizeof(part_count));
    if (!f || version != kCacheFormatVersion) return false;

    // A cap rather than trust: the file is ours, but a truncated or half-written one would
    // otherwise ask for an absurd allocation before failing.
    if (part_count > 100000u) return false;

    std::vector<ConvexPart> parts(part_count);
    for (uint32_t i = 0; i < part_count; ++i) {
        uint32_t n = 0;
        f.read(reinterpret_cast<char*>(&n), sizeof(n));
        if (!f || n > 10000000u) return false;
        parts[i].points.resize(n);
        f.read(reinterpret_cast<char*>(parts[i].points.data()),
               static_cast<std::streamsize>(n * sizeof(Vec3)));
        if (!f) return false;
    }
    out = std::move(parts);
    return true;
}

void writeCache(const std::string& path, const std::vector<ConvexPart>& parts)
{
    // ⚠ Write to a temporary and rename. Two robots loading the same level decompose the same
    // meshes at the same time, and a half-written file that is then READ AS VALID would poison the
    // cache permanently — the expensive failure this whole cache exists to avoid. rename() within
    // a directory is atomic.
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return;   // an unwritable cache is a performance problem, never a correctness one

        const uint32_t version = kCacheFormatVersion;
        const uint32_t count = static_cast<uint32_t>(parts.size());
        f.write(reinterpret_cast<const char*>(&version), sizeof(version));
        f.write(reinterpret_cast<const char*>(&count), sizeof(count));
        for (const ConvexPart& p : parts) {
            const uint32_t n = static_cast<uint32_t>(p.points.size());
            f.write(reinterpret_cast<const char*>(&n), sizeof(n));
            f.write(reinterpret_cast<const char*>(p.points.data()),
                    static_cast<std::streamsize>(n * sizeof(Vec3)));
        }
        if (!f) { std::remove(tmp.c_str()); return; }
    }
    std::rename(tmp.c_str(), path.c_str());
}

DecompositionResult singleHull(const std::vector<Vec3>& vertices, std::string note)
{
    DecompositionResult r;
    r.parts.resize(1);
    r.parts[0].points = vertices;
    r.decomposed = false;
    r.note = std::move(note);
    return r;
}

} // namespace

uint64_t decompositionCacheKey(const std::vector<Vec3>& vertices, const std::vector<int>& indices,
                               const DecompositionOptions& opts)
{
    uint64_t h = 14695981039346656037ULL;
    hashBytes(h, &kCacheFormatVersion, sizeof(kCacheFormatVersion));

    for (const Vec3& v : vertices) {
        const long long q[3] = { quantise(v.x), quantise(v.y), quantise(v.z) };
        hashBytes(h, q, sizeof(q));
    }
    if (!indices.empty())
        hashBytes(h, indices.data(), indices.size() * sizeof(int));

    // ⚠ The OPTIONS are part of the key, not just the mesh. Changing the threshold must produce a
    // different entry, or a re-tuned robot silently keeps the geometry it was cooked with and the
    // new setting appears to do nothing.
    const long long t = quantise(opts.threshold);
    hashBytes(h, &t, sizeof(t));
    hashBytes(h, &opts.max_hulls, sizeof(opts.max_hulls));
    return h;
}

DecompositionResult decomposeConvex(const std::vector<Vec3>& vertices,
                                    const std::vector<int>& indices,
                                    const DecompositionOptions& opts)
{
    if (!opts.enabled)
        return singleHull(vertices, "convex decomposition disabled by settings");
    if (vertices.size() < 4 || indices.size() < 12)
        return singleHull(vertices, "too small to decompose - fewer than 4 triangles");

#if !WITH_COACD_BINDING
    return singleHull(vertices,
                      "this build has no CoACD (WITH_COACD_BINDING=0) - run ./build_thirdparty.sh, "
                      "then ./build.sh");
#else
    const uint64_t key = decompositionCacheKey(vertices, indices, opts);

    if (!opts.cache_dir.empty()) {
        std::vector<ConvexPart> cached;
        if (readCache(cachePath(opts.cache_dir, key), cached)) {
            DecompositionResult r;
            r.parts = std::move(cached);
            r.decomposed = true;
            r.from_cache = true;
            r.note = "from cache";
            return r;
        }
    }

    // ⚠ THE C API, NOT THE C++ ONE, and this is an ABI decision rather than a style preference.
    // coacd::CoACD takes and returns std::vector<std::array<double,3>>. libcoacd.a is compiled with
    // UNREAL'S clang and libc++ (cmake-modules/UnrealToolchain.cmake); this translation unit is
    // compiled by build.sh with the HOST clang and libc++. Passing standard-library containers
    // across that boundary makes correctness depend on two libc++ builds agreeing about their
    // internal layout — true today, unverifiable, and exactly the kind of assumption that fails
    // silently after a toolchain bump.
    //
    // CoACD_run takes plain pointers and counts. There is nothing left to disagree about, and the
    // cost is the flat parameter list below. MuJoCo raised no such question only because its whole
    // public API is C.
    std::vector<double> verts(vertices.size() * 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        verts[3 * i + 0] = vertices[i].x;
        verts[3 * i + 1] = vertices[i].y;
        verts[3 * i + 2] = vertices[i].z;
    }
    std::vector<int> tris(indices.begin(), indices.end());

    CoACD_Mesh in;
    in.vertices_ptr = verts.data();
    in.vertices_count = static_cast<uint64_t>(vertices.size());
    in.triangles_ptr = tris.data();
    in.triangles_count = static_cast<uint64_t>(tris.size() / 3);

    const auto t0 = std::chrono::steady_clock::now();
    CoACD_MeshArray out;
    out.meshes_ptr = nullptr;
    out.meshes_count = 0;
    try {
        // Everything after max_hulls is upstream's default, spelled out because the C entry point
        // has no defaults. preprocess_auto matters most: it runs the OpenVDB manifold repair only
        // when the mesh needs it, and 9 of the Go2's 10 meshes do.
        out = CoACD_run(in, opts.threshold, opts.max_hulls, preprocess_auto,
                        /*prep_resolution=*/50, /*sample_resolution=*/2000, /*mcts_nodes=*/20,
                        /*mcts_iteration=*/150, /*mcts_max_depth=*/3, /*pca=*/false,
                        /*merge=*/true, /*decimate=*/false, /*max_ch_vertex=*/256,
                        /*extrude=*/false, /*extrude_margin=*/0.01, apx_ch, /*seed=*/0u,
                        /*real_metric=*/false);
    }
    catch (const std::exception& e) {
        // ⚠ Falling back rather than propagating, and saying so. CoACD throws on a mesh it cannot
        // handle — "The mesh is not a 2-manifold!" when built without OpenVDB, and other cases
        // besides. A link that loses its decomposition is a fidelity loss; a link that fails to
        // load at all is a robot that does not exist. The first is much the better failure, so
        // long as it is not silent.
        return singleHull(vertices, std::string("CoACD failed (") + e.what() +
                                        ") - falling back to a single convex hull");
    }
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (out.meshes_count == 0 || out.meshes_ptr == nullptr) {
        CoACD_freeMeshArray(out);
        return singleHull(vertices, "CoACD returned no parts - falling back to a single hull");
    }

    DecompositionResult r;
    r.parts.resize(static_cast<size_t>(out.meshes_count));
    for (uint64_t i = 0; i < out.meshes_count; ++i) {
        const CoACD_Mesh& m = out.meshes_ptr[i];
        ConvexPart& part = r.parts[static_cast<size_t>(i)];
        part.points.reserve(static_cast<size_t>(m.vertices_count));
        for (uint64_t v = 0; v < m.vertices_count; ++v)
            part.points.push_back(Vec3{ m.vertices_ptr[3 * v + 0], m.vertices_ptr[3 * v + 1],
                                        m.vertices_ptr[3 * v + 2] });
    }
    // ⚠ Freed through CoACD's own deallocator. The array was allocated inside a library built with
    // a different toolchain, so freeing it here would cross exactly the boundary the C API exists
    // to avoid.
    CoACD_freeMeshArray(out);

    r.decomposed = true;
    r.seconds = secs;
    r.note = "decomposed";

    if (!opts.cache_dir.empty())
        writeCache(cachePath(opts.cache_dir, key), r.parts);

    return r;
#endif
}

DecompositionResult decomposeConvexSoup(const std::vector<Vec3>& soup,
                                        const DecompositionOptions& opts)
{
    // ⚠ Welding is not a tidy-up, it is what makes the input a mesh. urdf::MeshData is a triangle
    // soup, and exporters split vertices at UV and normal seams besides, so a perfectly closed
    // surface arrives as thousands of disconnected triangles. CoACD reads connectivity from shared
    // indices; without this it sees no shared edges and decomposes nothing sensible.
    std::vector<Vec3> vertices;
    std::vector<int> indices;
    std::map<std::array<long long, 3>, int> weld;

    vertices.reserve(soup.size() / 2);
    indices.reserve(soup.size());

    std::vector<int> canon(soup.size());
    for (size_t i = 0; i < soup.size(); ++i) {
        const std::array<long long, 3> k{ quantise(soup[i].x), quantise(soup[i].y),
                                          quantise(soup[i].z) };
        auto it = weld.find(k);
        if (it != weld.end()) { canon[i] = it->second; continue; }
        const int idx = static_cast<int>(vertices.size());
        vertices.push_back(soup[i]);
        weld.emplace(k, idx);
        canon[i] = idx;
    }

    for (size_t t = 0; t + 2 < soup.size(); t += 3) {
        const int a = canon[t], b = canon[t + 1], c = canon[t + 2];
        if (a == b || b == c || a == c) continue;   // degenerate once welded
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }

    return decomposeConvex(vertices, indices, opts);
}

} // namespace urdf

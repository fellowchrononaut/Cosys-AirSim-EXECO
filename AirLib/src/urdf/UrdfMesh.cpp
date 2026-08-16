#include "urdf/UrdfMesh.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

namespace urdf {
namespace {

bool fileExists(const std::string& p)
{
    struct stat st;
    return !p.empty() && ::stat(p.c_str(), &st) == 0 && (st.st_mode & S_IFREG);
}

long long fileSize(const std::string& p)
{
    struct stat st;
    if (::stat(p.c_str(), &st) != 0) return -1;
    return static_cast<long long>(st.st_size);
}

std::string dirOf(const std::string& p)
{
    const size_t slash = p.find_last_of('/');
    return slash == std::string::npos ? std::string(".") : p.substr(0, slash);
}

MeshData loadStlBinary(std::ifstream& in, const std::string& path, uint32_t tri_count)
{
    MeshData m;
    m.triangles = tri_count;
    m.vertices.reserve(static_cast<size_t>(tri_count) * 3);

    // Each facet: 3 float normal, 9 float vertices, 2 byte attribute = 50 bytes.
    std::vector<char> buf(50);
    for (uint32_t t = 0; t < tri_count; ++t) {
        in.read(buf.data(), 50);
        if (in.gcount() != 50)
            throw std::runtime_error(path + ": truncated binary STL at triangle " +
                                     std::to_string(t) + " of " + std::to_string(tri_count));
        for (int v = 0; v < 3; ++v) {
            float xyz[3];
            std::memcpy(xyz, buf.data() + 12 + v * 12, 12);
            m.vertices.push_back(Vec3{ xyz[0], xyz[1], xyz[2] });
        }
    }
    return m;
}

MeshData loadStlAscii(const std::string& path)
{
    std::ifstream in(path);
    if (!in) throw std::runtime_error(path + ": cannot open mesh");

    MeshData m;
    std::string tok;
    while (in >> tok) {
        if (tok != "vertex") continue;
        double x = 0, y = 0, z = 0;
        if (!(in >> x >> y >> z))
            throw std::runtime_error(path + ": malformed 'vertex' record in ASCII STL");
        m.vertices.push_back(Vec3{ x, y, z });
    }
    if (m.vertices.size() % 3 != 0)
        throw std::runtime_error(path + ": ASCII STL vertex count " +
                                 std::to_string(m.vertices.size()) + " is not a multiple of 3");
    m.triangles = m.vertices.size() / 3;
    return m;
}

} // namespace

MeshData loadStl(const std::string& path)
{
    const long long size = fileSize(path);
    if (size < 0) throw std::runtime_error(path + ": cannot stat mesh file");

    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error(path + ": cannot open mesh");

    // 80-byte header, then a uint32 triangle count.
    char header[80] = { 0 };
    in.read(header, 80);
    uint32_t tri_count = 0;
    if (in.gcount() == 80 && in.read(reinterpret_cast<char*>(&tri_count), 4)) {
        // ⚠ Do not decide on the "solid" magic word: several exporters write it at the head of a
        // *binary* file, and trusting it yields an empty mesh with no error — a silently wrong
        // answer. The declared triangle count against the real file length is unambiguous.
        const long long expected = 84 + 50LL * static_cast<long long>(tri_count);
        if (tri_count > 0 && expected == size) return loadStlBinary(in, path, tri_count);
    }

    return loadStlAscii(path);
}

std::string resolveMeshPath(const std::string& filename, const std::string& urdf_dir,
                            const std::vector<std::string>& search_roots)
{
    if (filename.empty()) return {};

    if (filename.rfind("file://", 0) == 0) {
        const std::string p = filename.substr(7);
        return fileExists(p) ? p : std::string();
    }

    if (filename.rfind("package://", 0) == 0) {
        const std::string rest = filename.substr(10);         // "<pkg>/<path...>"
        const size_t slash = rest.find('/');
        const std::string pkg = slash == std::string::npos ? rest : rest.substr(0, slash);
        const std::string tail = slash == std::string::npos ? std::string() : rest.substr(slash + 1);

        std::vector<std::string> roots = search_roots;
        // The directory holding the URDF, and its parents, are where a self-contained model
        // package almost always sits.
        std::string d = urdf_dir;
        for (int up = 0; up < 4 && !d.empty() && d != "/"; ++up) {
            roots.push_back(d);
            d = dirOf(d);
        }

        for (const std::string& root : roots) {
            for (const std::string& cand : { root + "/" + pkg + "/" + tail, root + "/" + tail })
                if (fileExists(cand)) return cand;
        }
        return {};
    }

    if (!filename.empty() && filename[0] == '/') return fileExists(filename) ? filename : std::string();

    const std::string rel = urdf_dir + "/" + filename;
    return fileExists(rel) ? rel : std::string();
}

} // namespace urdf

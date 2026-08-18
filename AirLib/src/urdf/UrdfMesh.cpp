#include "urdf/UrdfMesh.hpp"

#include "tinyxml2.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
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

MeshData loadObj(const std::string& path)
{
    std::ifstream in(path);
    if (!in) throw std::runtime_error(path + ": cannot open mesh");

    std::vector<Vec3> pos;
    MeshData out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.size() < 2) continue;
        if (line[0] == 'v' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream ls(line.substr(1));
            Vec3 v{};
            if (ls >> v.x >> v.y >> v.z) pos.push_back(v);
        }
        else if (line[0] == 'f' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream ls(line.substr(1));
            std::string tok;
            std::vector<int> face;
            while (ls >> tok) {
                // "v", "v/vt", "v//vn", "v/vt/vn" - only the position index is wanted here.
                const size_t slash = tok.find('/');
                const std::string vs = slash == std::string::npos ? tok : tok.substr(0, slash);
                if (vs.empty()) continue;
                int idx = 0;
                try { idx = std::stoi(vs); } catch (...) { continue; }
                // ⚠ OBJ indices are 1-based, and NEGATIVE indices are relative to the end of the
                // vertex list so far. Treating a negative index as an error would reject files
                // several exporters emit; treating it as positive would silently pick the wrong
                // vertex, which is worse.
                if (idx < 0) idx = static_cast<int>(pos.size()) + idx;
                else idx -= 1;
                if (idx >= 0 && idx < static_cast<int>(pos.size())) face.push_back(idx);
            }
            // Fan-triangulate. OBJ faces are coplanar convex polygons by convention, which is
            // exactly when a fan is correct.
            for (size_t k = 2; k < face.size(); ++k) {
                out.vertices.push_back(pos[face[0]]);
                out.vertices.push_back(pos[face[k - 1]]);
                out.vertices.push_back(pos[face[k]]);
                ++out.triangles;
            }
        }
    }
    if (out.triangles == 0) throw std::runtime_error(path + ": OBJ contains no faces");
    return out;
}

// --- Collada ---------------------------------------------------------------------------------
namespace {

using Mat4 = std::array<double, 16>;  // row-major

Mat4 identity4()
{
    Mat4 m{};
    m[0] = m[5] = m[10] = m[15] = 1.0;
    return m;
}

Mat4 mul4(const Mat4& a, const Mat4& b)
{
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[i * 4 + k] * b[k * 4 + j];
            r[i * 4 + j] = s;
        }
    return r;
}

Vec3 xform(const Mat4& m, const Vec3& v)
{
    return Vec3{ m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3],
                 m[4] * v.x + m[5] * v.y + m[6] * v.z + m[7],
                 m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11] };
}

std::vector<double> numbers(const char* text)
{
    std::vector<double> v;
    if (!text) return v;
    std::istringstream ss(text);
    double d;
    while (ss >> d) v.push_back(d);
    return v;
}

std::vector<int> integers(const char* text)
{
    std::vector<int> v;
    if (!text) return v;
    std::istringstream ss(text);
    long long d;
    while (ss >> d) v.push_back(static_cast<int>(d));
    return v;
}

/// One geometry's triangles, in the geometry's own frame.
using GeomMap = std::map<std::string, std::vector<Vec3>>;

void readGeometries(tinyxml2::XMLElement* root, const std::string& path, GeomMap& out)
{
    std::map<std::string, std::vector<double>> sources;
    std::map<std::string, std::string> vertices;  // vertices id -> POSITION source id

    for (auto* lib = root->FirstChildElement("library_geometries"); lib;
         lib = lib->NextSiblingElement("library_geometries")) {
        for (auto* geom = lib->FirstChildElement("geometry"); geom;
             geom = geom->NextSiblingElement("geometry")) {
            auto* mesh = geom->FirstChildElement("mesh");
            if (!mesh) continue;

            sources.clear();
            vertices.clear();
            for (auto* src = mesh->FirstChildElement("source"); src;
                 src = src->NextSiblingElement("source")) {
                auto* fa = src->FirstChildElement("float_array");
                if (fa && src->Attribute("id"))
                    sources["#" + std::string(src->Attribute("id"))] = numbers(fa->GetText());
            }
            for (auto* v = mesh->FirstChildElement("vertices"); v;
                 v = v->NextSiblingElement("vertices")) {
                for (auto* inp = v->FirstChildElement("input"); inp;
                     inp = inp->NextSiblingElement("input")) {
                    const char* sem = inp->Attribute("semantic");
                    if (sem && std::string(sem) == "POSITION" && v->Attribute("id") &&
                        inp->Attribute("source"))
                        vertices["#" + std::string(v->Attribute("id"))] = inp->Attribute("source");
                }
            }

            std::vector<Vec3> tris;
            for (auto* prim = mesh->FirstChildElement(); prim; prim = prim->NextSiblingElement()) {
                const std::string tag = prim->Name();
                const bool is_tri = (tag == "triangles");
                const bool is_poly = (tag == "polylist" || tag == "polygons");
                if (tag == "tristrips" || tag == "trifans")
                    throw std::runtime_error(path + ": <" + tag +
                                             "> is not supported; re-export as triangles");
                if (!is_tri && !is_poly) continue;

                int stride = 1, pos_off = 0;
                std::string pos_src;
                for (auto* inp = prim->FirstChildElement("input"); inp;
                     inp = inp->NextSiblingElement("input")) {
                    const int off = inp->IntAttribute("offset", 0);
                    stride = std::max(stride, off + 1);
                    const char* sem = inp->Attribute("semantic");
                    const char* src = inp->Attribute("source");
                    if (!sem || !src) continue;
                    if (std::string(sem) == "VERTEX") {
                        pos_off = off;
                        auto it = vertices.find(src);
                        pos_src = (it != vertices.end()) ? it->second : std::string();
                    }
                    else if (std::string(sem) == "POSITION" && pos_src.empty()) {
                        pos_off = off;
                        pos_src = src;
                    }
                }
                auto sit = sources.find(pos_src);
                if (sit == sources.end())
                    throw std::runtime_error(path + ": <" + tag +
                                             "> has no resolvable POSITION source");
                const std::vector<double>& pts = sit->second;

                auto* p_el = prim->FirstChildElement("p");
                if (!p_el) continue;
                const std::vector<int> idx = integers(p_el->GetText());

                auto emit = [&](int i) {
                    const size_t b = static_cast<size_t>(i) * 3;
                    if (b + 2 < pts.size()) tris.push_back(Vec3{ pts[b], pts[b + 1], pts[b + 2] });
                };

                if (is_tri) {
                    const size_t n = idx.size() / (3 * static_cast<size_t>(stride));
                    for (size_t t = 0; t < n; ++t)
                        for (int k = 0; k < 3; ++k)
                            emit(idx[(t * 3 + k) * stride + pos_off]);
                }
                else {
                    // <polylist> carries a per-face vertex count; fan-triangulate each face.
                    std::vector<int> vcount;
                    if (auto* vc = prim->FirstChildElement("vcount"))
                        vcount = integers(vc->GetText());
                    size_t cursor = 0;
                    for (int fv : vcount) {
                        for (int k = 2; k < fv; ++k) {
                            emit(idx[(cursor + 0) * stride + pos_off]);
                            emit(idx[(cursor + k - 1) * stride + pos_off]);
                            emit(idx[(cursor + k) * stride + pos_off]);
                        }
                        cursor += static_cast<size_t>(fv);
                    }
                }
            }
            if (geom->Attribute("id") && !tris.empty())
                out["#" + std::string(geom->Attribute("id"))] = std::move(tris);
        }
    }
}

/// Walk a <node> subtree, composing transforms, emitting every <instance_geometry> found.
void walkNode(tinyxml2::XMLElement* node, Mat4 parent, const GeomMap& geoms, MeshData& out,
              bool& any_instanced)
{
    Mat4 m = parent;
    for (auto* e = node->FirstChildElement(); e; e = e->NextSiblingElement()) {
        const std::string tag = e->Name();
        if (tag == "matrix") {
            const std::vector<double> v = numbers(e->GetText());
            if (v.size() == 16) {
                Mat4 t{};
                std::copy(v.begin(), v.end(), t.begin());
                m = mul4(m, t);
            }
        }
        else if (tag == "translate") {
            const std::vector<double> v = numbers(e->GetText());
            if (v.size() == 3) {
                Mat4 t = identity4();
                t[3] = v[0]; t[7] = v[1]; t[11] = v[2];
                m = mul4(m, t);
            }
        }
        else if (tag == "scale") {
            const std::vector<double> v = numbers(e->GetText());
            if (v.size() == 3) {
                Mat4 t = identity4();
                t[0] = v[0]; t[5] = v[1]; t[10] = v[2];
                m = mul4(m, t);
            }
        }
        else if (tag == "rotate") {
            const std::vector<double> v = numbers(e->GetText());
            if (v.size() == 4 && v[3] != 0.0) {
                const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (n > 1e-12) {
                    const double x = v[0] / n, y = v[1] / n, z = v[2] / n;
                    const double a = v[3] * 3.14159265358979323846 / 180.0;  // Collada: DEGREES
                    const double c = std::cos(a), s = std::sin(a), k = 1 - c;
                    Mat4 t = identity4();
                    t[0] = c + x * x * k;     t[1] = x * y * k - z * s; t[2] = x * z * k + y * s;
                    t[4] = y * x * k + z * s; t[5] = c + y * y * k;     t[6] = y * z * k - x * s;
                    t[8] = z * x * k - y * s; t[9] = z * y * k + x * s; t[10] = c + z * z * k;
                    m = mul4(m, t);
                }
            }
        }
    }

    for (auto* ig = node->FirstChildElement("instance_geometry"); ig;
         ig = ig->NextSiblingElement("instance_geometry")) {
        const char* url = ig->Attribute("url");
        if (!url) continue;
        auto it = geoms.find(url);
        if (it == geoms.end()) continue;
        any_instanced = true;
        for (const Vec3& v : it->second) {
            out.vertices.push_back(xform(m, v));
        }
    }

    for (auto* child = node->FirstChildElement("node"); child;
         child = child->NextSiblingElement("node"))
        walkNode(child, m, geoms, out, any_instanced);
}

} // namespace

MeshData loadCollada(const std::string& path)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error(path + ": Collada parse error: " +
                                 std::string(doc.ErrorStr() ? doc.ErrorStr() : "?"));
    auto* root = doc.FirstChildElement("COLLADA");
    if (!root) throw std::runtime_error(path + ": no <COLLADA> root element");

    // ⚠ Read <unit> and <up_axis> BEFORE anything else uses the numbers. Collada declares its own
    // scale and handedness; a reader that ignores them produces a robot silently the wrong size or
    // on its side. URDF is metres, Z-up, so both are corrected to that here.
    double unit = 1.0;
    std::string up = "Y_UP";
    if (auto* asset = root->FirstChildElement("asset")) {
        if (auto* u = asset->FirstChildElement("unit")) unit = u->DoubleAttribute("meter", 1.0);
        if (auto* a = asset->FirstChildElement("up_axis"))
            if (const char* t = a->GetText()) up = t;
    }

    GeomMap geoms;
    readGeometries(root, path, geoms);
    if (geoms.empty()) throw std::runtime_error(path + ": Collada contains no triangle geometry");

    MeshData out;
    bool any_instanced = false;
    for (auto* lib = root->FirstChildElement("library_visual_scenes"); lib;
         lib = lib->NextSiblingElement("library_visual_scenes"))
        for (auto* scene = lib->FirstChildElement("visual_scene"); scene;
             scene = scene->NextSiblingElement("visual_scene"))
            for (auto* node = scene->FirstChildElement("node"); node;
                 node = node->NextSiblingElement("node"))
                walkNode(node, identity4(), geoms, out, any_instanced);

    // ⚠ No visual scene, or a scene that instances nothing: emit every geometry untransformed.
    // That is the only sane reading, and it is the common case for the single-part meshes a URDF
    // references. Failing here instead would reject files that are perfectly usable.
    if (!any_instanced) {
        out.vertices.clear();
        for (const auto& g : geoms)
            out.vertices.insert(out.vertices.end(), g.second.begin(), g.second.end());
    }

    for (Vec3& v : out.vertices) {
        v.x *= unit; v.y *= unit; v.z *= unit;
        if (up == "Y_UP") { const double y = v.y; v.y = -v.z; v.z = y; }
        else if (up == "X_UP") { const double x = v.x; v.x = v.y; v.y = x; }
    }

    out.triangles = out.vertices.size() / 3;
    if (out.triangles == 0) throw std::runtime_error(path + ": Collada produced no triangles");
    return out;
}

MeshData loadMesh(const std::string& path)
{
    std::string ext;
    const size_t dot = path.find_last_of('.');
    if (dot != std::string::npos) ext = path.substr(dot + 1);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if (ext == "stl") return loadStl(path);
    if (ext == "dae") return loadCollada(path);
    if (ext == "obj") return loadObj(path);

    // ⚠ An unknown extension is an ERROR, never an empty mesh. A link that silently loses its
    // geometry is invisible to the renderer AND untraceable by every sensor, and the only symptom
    // is a robot that is partly not there.
    throw std::runtime_error(path + ": unsupported mesh format '." + ext +
                             "'; supported: .stl, .dae, .obj");
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

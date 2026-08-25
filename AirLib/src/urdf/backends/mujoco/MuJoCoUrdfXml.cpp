#include "urdf/backends/mujoco/MuJoCoUrdfXml.hpp"

#include "urdf/UrdfMesh.hpp"

#include <cctype>
#include <cstddef>

namespace urdf {

std::string resolveMeshPathsForMuJoCo(const std::string& urdf_xml, const std::string& urdf_dir,
                                      const std::vector<std::string>& search_roots,
                                      std::vector<std::string>* unresolved)
{
    // A targeted rewrite rather than a parse-and-reserialise: MuJoCo reads the ORIGINAL bytes, and
    // round-tripping the document through another writer would create a second, quietly divergent
    // definition of the robot. Only the attribute values change.
    static const std::string kAttribute = "filename=\"";

    std::string out;
    out.reserve(urdf_xml.size() + 256);

    std::size_t cursor = 0;
    while (true) {
        const std::size_t start = urdf_xml.find(kAttribute, cursor);
        if (start == std::string::npos) {
            out.append(urdf_xml, cursor, std::string::npos);
            break;
        }
        const std::size_t value_start = start + kAttribute.size();
        const std::size_t value_end = urdf_xml.find('"', value_start);
        if (value_end == std::string::npos) {
            out.append(urdf_xml, cursor, std::string::npos);
            break;
        }

        // ⚠ ONLY <mesh filename="...">. A URDF also carries <gazebo><plugin filename="lib*.so">,
        // and reporting those as unresolved MESHES is a false alarm that trains an operator to
        // ignore the real one. Walk back to the owning tag and check it.
        std::size_t tag = urdf_xml.rfind('<', start);
        bool is_mesh = false;
        if (tag != std::string::npos) {
            std::size_t name = tag + 1;
            while (name < urdf_xml.size() && std::isspace(static_cast<unsigned char>(urdf_xml[name])))
                ++name;
            is_mesh = urdf_xml.compare(name, 4, "mesh") == 0 &&
                      name + 4 < urdf_xml.size() &&
                      (std::isspace(static_cast<unsigned char>(urdf_xml[name + 4])) ||
                       urdf_xml[name + 4] == '/' || urdf_xml[name + 4] == '>');
        }

        const std::string filename = urdf_xml.substr(value_start, value_end - value_start);
        const std::string resolved =
            is_mesh ? resolveMeshPath(filename, urdf_dir, search_roots) : filename;

        out.append(urdf_xml, cursor, value_start - cursor);
        if (!is_mesh) {
            out.append(filename);
        }
        else if (resolved.empty()) {
            // Left exactly as written. MuJoCo will drop it, and the caller reports which.
            out.append(filename);
            if (unresolved != nullptr) unresolved->push_back(filename);
        }
        else {
            out.append(resolved);
        }
        cursor = value_end;
    }

    return out;
}

} // namespace urdf

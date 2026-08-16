#pragma once

#include "urdf/UrdfModel.hpp"

#include <stdexcept>
#include <string>

namespace urdf {

class ParseError : public std::runtime_error {
public:
    explicit ParseError(const std::string& what) : std::runtime_error(what) {}
};

/// Parse a URDF file. Throws ParseError with a message naming the offending element.
///
/// Supported: <robot>, <link> (multiple <collision>, one <inertial>), <joint> of every URDF type,
/// <origin>, <axis>, <limit>, <dynamics>, and box/cylinder/sphere/mesh geometry.
///
/// Deliberately NOT supported, and rejected loudly rather than silently ignored:
///   - xacro (run it through `xacro` first)
///   - <mimic> and <safety_controller> — parsed by UrdfSim but never acted on; accepting them
///     here would repeat that class of silent no-op
///   - <transmission>, <gazebo> extensions
Robot parseFile(const std::string& path);

/// Same, from a string. Used by the tests so fixtures need no files on disk.
Robot parseString(const std::string& xml, const std::string& source_name = "<string>");

} // namespace urdf

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
/// Supported: <robot>, <link> (multiple <collision> and <visual>, one <inertial>), <joint> of
/// every URDF type, <origin>, <axis>, <limit>, <dynamics>, <mimic>, and box/cylinder/sphere/mesh
/// geometry.
///
/// Deliberately NOT supported, and rejected loudly rather than silently ignored:
///   - xacro (run it through `xacro` first)
///   - <safety_controller> — parsed by UrdfSim but never acted on; accepting it here would
///     repeat that class of silent no-op
///   - <transmission>, <gazebo> extensions
///
/// ⚠ <mimic> is *parsed* here but not *resolved* here. Whether the coupling can be honoured, and
/// how, depends on whether the joint carries load — a backend question, answered in UrdfMimic.hpp
/// and enforced when the robot is built. The parser's contract is to record the constraint in full
/// (source, multiplier, offset) so no later stage has to guess. What is forbidden is UrdfSim's
/// behaviour: storing it and never applying it.
struct ParseOptions {
    /// Reserved. No parse-time opt-outs exist at present: everything this parser accepts, it
    /// records faithfully, and every judgement call about what can be *simulated* belongs to the
    /// backend. Kept as a struct so adding one later is not an API break.
};

Robot parseFile(const std::string& path, const ParseOptions& options = {});

/// Same, from a string. Used by the tests so fixtures need no files on disk.
Robot parseString(const std::string& xml, const std::string& source_name = "<string>",
                  const ParseOptions& options = {});

} // namespace urdf

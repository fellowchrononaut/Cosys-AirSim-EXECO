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
///   - <safety_controller> — parsed by UrdfSim but never acted on; accepting it here would
///     repeat that class of silent no-op
///   - <mimic> — same reasoning, but see ParseOptions::ignore_mimic for a deliberate opt-out
///   - <transmission>, <gazebo> extensions
/// Opt-ins for URDF features this loader cannot yet honour.
///
/// The default is to REFUSE such a file rather than load a robot that quietly differs from it.
/// A caller who knows the feature is cosmetic can enable it here; every affected joint is then
/// flagged (Joint::mimic_ignored) so the omission stays enumerable rather than invisible.
struct ParseOptions {
    /// Load <mimic> joints as ordinary free joints of their stated type. Box3D has no gear or
    /// coupled-joint constraint, so the coupling genuinely cannot be represented today.
    bool ignore_mimic = false;
};

Robot parseFile(const std::string& path, const ParseOptions& options = {});

/// Same, from a string. Used by the tests so fixtures need no files on disk.
Robot parseString(const std::string& xml, const std::string& source_name = "<string>",
                  const ParseOptions& options = {});

} // namespace urdf

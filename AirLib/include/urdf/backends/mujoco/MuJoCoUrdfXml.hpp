// Making a ROS-style URDF readable by MuJoCo's own XML parser.
//
// ⚠ WHY THIS EXISTS. MuJoCo builds its model from the URDF TEXT, using its own reader — and that
// reader has no notion of ROS packages and no search-path list, so every
// `filename="package://pkg/..."` is a mesh it cannot open. It does not fail loudly: it drops the
// geometry and compiles a model whose links have no collision at all. Measured on the ExoMy, whose
// 23 collision elements are ALL package:// URIs: the robot loads, renders, reports sane state, and
// falls through the floor.
//
// Our own parser already resolves these (urdf::resolveMeshPath). This rewrites the same references
// in the XML so MuJoCo receives paths it can open, rather than teaching MuJoCo about ROS.
#pragma once

#include <string>
#include <vector>

namespace urdf {

/// Rewrite every `filename="..."` in `urdf_xml` to an absolute path.
///
/// `urdf_dir` is the directory holding the URDF; `search_roots` are the ROS-package roots a
/// `package://` reference resolves against. Anything that cannot be resolved is left untouched and
/// appended to `unresolved` — reported, never silently substituted, because a robot quietly missing
/// its collision geometry is the exact failure this guards.
std::string resolveMeshPathsForMuJoCo(const std::string& urdf_xml, const std::string& urdf_dir,
                                      const std::vector<std::string>& search_roots,
                                      std::vector<std::string>* unresolved);

} // namespace urdf

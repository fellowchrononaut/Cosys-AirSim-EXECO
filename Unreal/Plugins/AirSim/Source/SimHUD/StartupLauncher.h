#pragma once

#include <string>

namespace airsim_startup
{
    // Opens the editor-modal MVP used when -settings= is absent. Returns true only when the user
    // presses Launch; cancellation is reported separately so callers do not fall through to the
    // legacy default-file path.
    bool RunStartupLauncher(std::string& settings_text, bool& cancelled);
}

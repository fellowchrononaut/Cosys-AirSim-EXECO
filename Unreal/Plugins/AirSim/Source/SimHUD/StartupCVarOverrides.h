#pragma once

#include <string>
#include <vector>

// Validates and applies the optional top-level StartupCVars object from the already parsed
// AirSim settings document. Values are applied through IConsoleVariable::Set only; this API never
// constructs or executes a console command. Throws std::invalid_argument on any refused override.
namespace airsim_startup
{
    struct AppliedCVar
    {
        std::string name;
        std::string value;
        std::string type;
    };

    std::vector<AppliedCVar> ValidateStartupCVars();
    std::vector<AppliedCVar> ApplyStartupCVars();
    void RestoreStartupCVars();
}

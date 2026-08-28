#include "StartupCVarOverrides.h"

#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include "common/AirSimSettings.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace airsim_startup
{
namespace
{
    using msr::airlib::AirSimSettings;

    struct Pending
    {
        IConsoleVariable* variable;
        AppliedCVar record;
        int32 integer = 0;
        float number = 0.0f;
        FString string;
        enum class Kind { Integer, Number, String } kind;
    };

    struct PreviousValue
    {
        IConsoleVariable* variable;
        EConsoleVariableFlags priority;
        Pending::Kind kind;
        int32 integer = 0;
        float number = 0.0f;
        FString string;
    };
    std::vector<PreviousValue> GPreviousValues;
}

static std::vector<Pending> BuildPending()
{
    const auto& authored_cvars = AirSimSettings::singleton().startup_cvars;
    std::vector<Pending> pending;
    pending.reserve(authored_cvars.size());

    for (const auto& authored_pair : authored_cvars) {
        const std::string& name = authored_pair.first;
        const AirSimSettings::StartupCVarSetting& authored = authored_pair.second;
        IConsoleVariable* variable = IConsoleManager::Get().FindConsoleVariable(UTF8_TO_TCHAR(name.c_str()));
        if (variable == nullptr)
            throw std::invalid_argument("StartupCVars entry '" + name + "' is not a registered console variable (commands are not allowed)");

        Pending entry;
        entry.variable = variable;
        entry.record.name = name;
        if (authored.value_type == AirSimSettings::StartupCVarSetting::ValueType::Integer) {
            entry.integer = static_cast<int32>(authored.integer_value);
            if (variable->IsVariableInt()) {
                entry.kind = Pending::Kind::Integer;
                entry.record.type = "integer";
            }
            else if (variable->IsVariableFloat()) {
                entry.number = static_cast<float>(entry.integer);
                entry.kind = Pending::Kind::Number;
                entry.record.type = "number";
            }
            else {
                throw std::invalid_argument("StartupCVars entry '" + name + "' must use a string value for this CVar");
            }
            entry.record.value = std::to_string(entry.integer);
        }
        else if (authored.value_type == AirSimSettings::StartupCVarSetting::ValueType::Number) {
            const double value = authored.number_value;
            if (!std::isfinite(value) || value < -std::numeric_limits<float>::max() || value > std::numeric_limits<float>::max())
                throw std::invalid_argument("StartupCVars entry '" + name + "' is outside the supported float range");
            if (!variable->IsVariableFloat())
                throw std::invalid_argument("StartupCVars entry '" + name + "' must use an integer value for this CVar");
            entry.number = static_cast<float>(value);
            entry.kind = Pending::Kind::Number;
            entry.record.type = "number";
            entry.record.value = std::to_string(entry.number);
        }
        else if (authored.value_type == AirSimSettings::StartupCVarSetting::ValueType::String) {
            if (!variable->IsVariableString())
                throw std::invalid_argument("StartupCVars entry '" + name + "' must use a numeric value for this CVar");
            entry.string = UTF8_TO_TCHAR(authored.string_value.c_str());
            entry.kind = Pending::Kind::String;
            entry.record.type = "string";
            entry.record.value = TCHAR_TO_UTF8(*entry.string);
        }
        pending.push_back(std::move(entry));
    }

    return pending;
}

std::vector<AppliedCVar> ValidateStartupCVars()
{
    const std::vector<Pending> pending = BuildPending();
    std::vector<AppliedCVar> records;
    records.reserve(pending.size());
    for (const Pending& entry : pending)
        records.push_back(entry.record);
    return records;
}

std::vector<AppliedCVar> ApplyStartupCVars()
{
    const std::vector<Pending> pending = BuildPending();
    std::vector<AppliedCVar> applied;
    applied.reserve(pending.size());
    // BuildPending validates every entry before changing any process-global CVar. A malformed
    // entry later in the object therefore cannot leave a partially applied launch behind.
    for (const Pending& entry : pending) {
        const bool already_saved = std::any_of(GPreviousValues.begin(), GPreviousValues.end(),
            [&entry](const PreviousValue& previous) { return previous.variable == entry.variable; });
        if (!already_saved) {
            PreviousValue previous;
            previous.variable = entry.variable;
            previous.priority = static_cast<EConsoleVariableFlags>(entry.variable->GetFlags() & ECVF_SetByMask);
            previous.kind = entry.kind;
            switch (entry.kind) {
            case Pending::Kind::Integer: previous.integer = entry.variable->GetInt(); break;
            case Pending::Kind::Number: previous.number = entry.variable->GetFloat(); break;
            case Pending::Kind::String: previous.string = entry.variable->GetString(); break;
            }
            GPreviousValues.push_back(std::move(previous));
        }
        switch (entry.kind) {
        case Pending::Kind::Integer:
            entry.variable->Set(entry.integer, ECVF_SetByConsole);
            break;
        case Pending::Kind::Number:
            entry.variable->Set(entry.number, ECVF_SetByConsole);
            break;
        case Pending::Kind::String:
            entry.variable->Set(*entry.string, ECVF_SetByConsole);
            break;
        }
        applied.push_back(entry.record);
    }
    return applied;
}

void RestoreStartupCVars()
{
    for (auto it = GPreviousValues.rbegin(); it != GPreviousValues.rend(); ++it) {
        it->variable->Unset(ECVF_SetByConsole);
        // Preserve the exact source priority, including ECVF_SetByConstructor (zero). Promoting
        // constructor/default values to SetByCode would leave a stronger source after cleanup.
        const EConsoleVariableFlags priority = it->priority;
        switch (it->kind) {
        case Pending::Kind::Integer: it->variable->Set(it->integer, priority); break;
        case Pending::Kind::Number: it->variable->Set(it->number, priority); break;
        case Pending::Kind::String: it->variable->Set(*it->string, priority); break;
        }
    }
    GPreviousValues.clear();
}
}

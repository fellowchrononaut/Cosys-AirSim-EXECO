#include "StartupProfileState.h"

#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"

namespace
{
    const TCHAR* ProfileSection = TEXT("AirSimStartupLauncher");
    const TCHAR* LastProfileKey = TEXT("LastProfile");
    const TCHAR* RecentProfilesKey = TEXT("RecentProfiles");
}

// Keep a linkable definition for toolchains that still apply pre-C++17 constexpr
// odr-use rules to the public cap used by the launcher and its tests.
constexpr int32 FStartupProfileState::MaxRecentProfiles;

FString FStartupProfileState::NormalizePath(const FString& path)
{
    if (path.IsEmpty())
        return FString();
    FString normalized = FPaths::ConvertRelativePathToFull(path);
    FPaths::NormalizeFilename(normalized);
    return normalized;
}

void FStartupProfileState::SetLoaded(const FString& path, const FString& text)
{
    CurrentPath = NormalizePath(path);
    CurrentText = text;
    SavedText = text;
    LastProfile = CurrentPath;
    if (!CurrentPath.IsEmpty())
        AddRecentPath(CurrentPath);
}

void FStartupProfileState::SetCurrentText(const FString& text)
{
    CurrentText = text;
}

void FStartupProfileState::MarkSaved()
{
    SavedText = CurrentText;
}

bool FStartupProfileState::IsDirty() const
{
    return CurrentText != SavedText;
}

void FStartupProfileState::AddRecentPath(const FString& path)
{
    const FString normalized = NormalizePath(path);
    if (normalized.IsEmpty())
        return;
    RecentPaths.Remove(normalized);
    RecentPaths.Insert(normalized, 0);
    while (RecentPaths.Num() > MaxRecentProfiles)
        RecentPaths.Pop();
}

void FStartupProfileState::RemoveRecentPath(const FString& path)
{
    RecentPaths.Remove(NormalizePath(path));
}

void FStartupProfilePersistence::Load(FStartupProfileState& state)
{
    if (GConfig == nullptr)
        return;
    GConfig->GetString(ProfileSection, LastProfileKey, state.LastProfile, GGameUserSettingsIni);
    state.LastProfile = FStartupProfileState::NormalizePath(state.LastProfile);
    TArray<FString> persisted;
    GConfig->GetArray(ProfileSection, RecentProfilesKey, persisted, GGameUserSettingsIni);
    for (int32 index = persisted.Num() - 1; index >= 0; --index)
    {
        const FString& path = persisted[index];
        const FString normalized = FStartupProfileState::NormalizePath(path);
        if (!normalized.IsEmpty() && FPaths::FileExists(normalized))
            state.AddRecentPath(normalized);
    }
}

void FStartupProfilePersistence::Save(const FStartupProfileState& state)
{
    if (GConfig == nullptr)
        return;
    GConfig->SetString(ProfileSection, LastProfileKey, *state.LastProfile, GGameUserSettingsIni);
    TArray<FString> recent;
    for (const FString& path : state.RecentPaths)
        if (FPaths::FileExists(path))
            recent.Add(path);
    GConfig->SetArray(ProfileSection, RecentProfilesKey, recent, GGameUserSettingsIni);
    GConfig->Flush(false, GGameUserSettingsIni);
}

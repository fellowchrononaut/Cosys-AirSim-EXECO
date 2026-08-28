#pragma once

#include "CoreMinimal.h"

/** Pure in-memory state for the interactive startup-launcher profile workflow. */
class FStartupProfileState
{
public:
    static constexpr int32 MaxRecentProfiles = 8;

    FString CurrentPath;
    FString CurrentText;
    FString SavedText;
    FString LastProfile;
    TArray<FString> RecentPaths;

    void SetLoaded(const FString& path, const FString& text);
    void SetCurrentText(const FString& text);
    void MarkSaved();
    bool IsDirty() const;
    void AddRecentPath(const FString& path);
    void RemoveRecentPath(const FString& path);

    static FString NormalizePath(const FString& path);
};

/** Persistence adapter using Unreal's per-user GameUserSettings config, never project files. */
class FStartupProfilePersistence
{
public:
    static void Load(FStartupProfileState& state);
    static void Save(const FStartupProfileState& state);
};

#include "../StartupProfileState.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupProfileStateTransitions,
    "AirSim.StartupLauncher.ProfileState.Transitions",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupProfileStateTransitions::RunTest(const FString&)
{
    FStartupProfileState state;
    state.SetLoaded(TEXT("/tmp/profile-a.json"), TEXT("one"));
    TestFalse(TEXT("Loaded snapshot starts clean"), state.IsDirty());
    state.SetCurrentText(TEXT("two"));
    TestTrue(TEXT("Editing changes dirty state"), state.IsDirty());
    state.MarkSaved();
    TestFalse(TEXT("MarkSaved updates the snapshot"), state.IsDirty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupProfileStateRecentPaths,
    "AirSim.StartupLauncher.ProfileState.RecentPaths",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupProfileStateRecentPaths::RunTest(const FString&)
{
    FStartupProfileState state;
    state.AddRecentPath(TEXT("/tmp/profile-a.json"));
    state.AddRecentPath(TEXT("/tmp/profile-b.json"));
    state.AddRecentPath(TEXT("/tmp/profile-a.json"));
    TestEqual(TEXT("Most recent path is first"), state.RecentPaths[0], FStartupProfileState::NormalizePath(TEXT("/tmp/profile-a.json")));
    TestEqual(TEXT("Duplicate path is removed"), state.RecentPaths.Num(), 2);
    for (int32 index = 0; index < 10; ++index)
        state.AddRecentPath(FString::Printf(TEXT("/tmp/profile-%d.json"), index));
    TestEqual(TEXT("Recent paths are capped"), state.RecentPaths.Num(), FStartupProfileState::MaxRecentProfiles);
    return true;
}

#endif

// Launching and stopping the Newton MPM sidecar from the simulator.
//
// ⚠ ONE PLACE, SHARED BY THE RUNTIME AND THE EDITOR PANEL. Both launch the same script: the
// runtime for auto-start when the settings select PhysicsEngine "NewtonSidecar", the panel for its
// Start/Stop/Restart buttons. The path used to be derived separately in each — the panel from the
// plugin directory, the simulator from the project directory — which is two copies of "four levels
// up" that stay correct until somebody moves a folder and then fail in only one of the two places:
// the button working while auto-start silently does nothing, or the reverse.
//
// ⚠ A PLAIN HEADER, DELIBERATELY. This was first declared in SimModeWorldBase.h and
// UnrealHeaderTool rejected it outright — "Found 'AIRSIM_API' when expecting class while parsing
// class" — because that header is a UObject header and UHT parses it. A free function belongs
// somewhere UHT does not read.
#pragma once

#include "CoreMinimal.h"

/// Resolved from `airsim.NewtonSidecarControlScript` when set, otherwise from the project
/// directory. Does not check that the file exists; callers report that themselves, because
/// "missing" means something different to a button than it does to an automatic launch.
AIRSIM_API FString ResolveNewtonSidecarControlScript();

/// Run `tools/sidecar_ctl.sh <Verb>` detached. Returns false only when the script could not be
/// launched at all — never when the sidecar itself refused, because that answer takes up to
/// fifteen seconds and arrives on the wire rather than from the process.
///
/// ⚠ `bTakeOwnership` DECIDES WHETHER THE EDITOR WILL LATER KILL IT, and the distinction matters:
/// a sidecar the operator started from a terminal — for a probe run, or to keep a bed alive across
/// editor restarts — must not be stopped by an editor that had nothing to do with it. Only a
/// launch that came from this process claims ownership.
AIRSIM_API bool RunNewtonSidecarControl(const FString& Verb, bool bTakeOwnership,
                                        FString& OutMessage);

/// Stop the sidecar, but ONLY if this process started it. Called on editor shutdown.
///
/// ⚠ THIS COVERS THE CLEAN EXIT ONLY. A crashed editor never runs it, which is why an
/// editor-started sidecar is also given `--parent-pid`: it watches for that pid to disappear and
/// exits on its own. Two abandoned sidecars were found by hand on 2026-08-26, one after 32
/// minutes, each holding GPU memory for a simulator that had stopped.
AIRSIM_API void StopNewtonSidecarIfOwned();

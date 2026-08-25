// Drawing the solver's own collision geometry on top of the Unreal level it approximates.
//
// ⚠ WHY THIS IS A VIEW AND NOTHING ELSE. Every collision defect in this workstream has been an
// argument about geometry conducted entirely through counters — "172 shapes mirrored", "6 collision
// geoms realised", "2704/2704 traces hit" — and two of those numbers were true while being about
// the wrong place. A wireframe over the level cannot be true about the wrong place: it is either
// on the thing it should be on, or it visibly is not. Nothing in the simulation may read it.
//
// ⚠ SOLVER FRAME IN, UNREAL FRAME OUT, exactly once, through UrdfTransform. The Y mirror between
// URDF/FLU and Unreal has already produced one silent bug in this workstream (a height field
// sampled 120 m from where it was reported), and the only defence is that there is one conversion
// and every consumer uses it.
#pragma once

#include "CoreMinimal.h"

#include "urdf/UrdfCollisionDebug.hpp"

class UWorld;

namespace PhysicsCollisionDebugDraw {

/// Is the overlay switched on? Driven by `airsim.PhysicsDebugDraw`, so it can be toggled inside a
/// running PIE session without a relaunch.
bool IsEnabled();

/// Build the read-back filter from the console variables and the operator's viewpoint.
///
/// `FocusUnreal` is where the camera is: the overlay follows what is being looked at, which is what
/// makes a radius usable at all. Pass `FVector::ZeroVector` and the radius cvar will be ignored.
urdf::CollisionDebugFilter MakeFilter(const FVector& FocusUnreal, float WorldToMeters,
                                      bool bHaveFocus);

/// Draw one snapshot for a single frame.
void Draw(UWorld* World, const urdf::CollisionDebugSnapshot& Snapshot, float WorldToMeters);

/// Report what the snapshot contained, at most once every few seconds.
///
/// ⚠ Includes the LARGEST geom and its owner. "The collision looks bigger than the asset" is a
/// measurable claim, and this is the number that measures it — a wheel whose realised hull spans
/// 0.4 m when the wheel is 0.2 m across says so here before anyone has to squint at a wireframe.
void LogSummary(const urdf::CollisionDebugSnapshot& Snapshot);

} // namespace PhysicsCollisionDebugDraw

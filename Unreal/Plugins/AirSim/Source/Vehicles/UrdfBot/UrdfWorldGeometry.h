// Mirroring the Unreal level's collision into a solver-agnostic urdf::StaticWorld.
//
// This is the game-thread half of analysis doc §6.1's recommendation — "the plugin keeps a
// game-thread module whose only jobs are (i) mirroring Unreal collision geometry into Box3D once at
// BeginPlay ... Nothing game-thread-side steps." It runs once, produces plain data, and never sees
// the solver. The extraction is adapted from Box3DUnreal's `Box3DStaticGeometry` (MIT), which is
// the reference implementation of this exact job.
//
// ⚠ One deliberate departure from that reference: it extracts only an actor's **root** primitive
// component. That is fine for a plugin whose actors are purpose-built colliders, and wrong for a
// general Unreal map, where geometry routinely hangs off non-root components. This walks every
// primitive component on every actor.
//
// ⚠ Scope is the **whole level**, per §6.0c, not a radius around the robot. A radius-limited
// mirror would make the physical world depend on which robot you are — two rovers driving the same
// route would hit different obstacles — which is the class of silent divergence this workstream
// exists to avoid. It is affordable because the cook is shared across robots (Box3DStaticGeometry).
//
// ⚠ What is mirrored is Unreal's **collision**, and the sensors trace Unreal's collision too. So
// unlike R2 — where Box3D drives on `<collision>` hulls while LiDAR sees `<visual>` meshes — the
// level does *not* have two divergent representations. The robot and the world are asymmetric in
// exactly one direction, and that asymmetry is R2's, not this file's.
#pragma once

#include "CoreMinimal.h"

/// ⚠ UAirBlueprintLib::LogMessage is **on-screen only** — every UE_LOG inside it is commented out,
/// and it keys messages by prefix so two messages sharing one prefix overwrite each other. That is
/// fine for a one-line status and useless for a diagnostic report: the report scrolls off, is never
/// written to Blocks.log, and cannot be read back after the fact. Everything in this workstream
/// depends on the operator being able to see these numbers, so the urdfbot's diagnostics go to a
/// real log category as well as to the screen.
DECLARE_LOG_CATEGORY_EXTERN(LogUrdfBot, Log, All);

#include "urdf/UrdfStaticWorld.hpp"

#include <memory>

class UWorld;

namespace UrdfWorldGeometry {

/// Which cooked collision to take from a component.
enum class ECollisionSource : uint8 {
    /// Complex tri-mesh where the asset has one, else the simple primitives. The right default:
    /// tri-mesh is what a level's floors and walls actually are.
    Auto,
    /// AggGeom convex/box/sphere/capsule only. Cheaper and coarser.
    Simple,
    /// Cooked tri-mesh only; components without one are skipped and named.
    Complex,
};

/// A mirrored level plus the live Unreal components behind its kinematic bodies.
///
/// The two travel together because `KinematicSources[i]` drives `World->kinematic[i]`: they are
/// index-aligned, and separating them would let that alignment rot silently — which would push the
/// wrong obstacle to the wrong place, an error that looks like a physics bug rather than a
/// bookkeeping one.
struct FMirrorResult {
    std::shared_ptr<const urdf::StaticWorld> World;

    /// Index-aligned with `World->kinematic`. Weak, because a mirrored actor may be destroyed
    /// mid-run; a stale entry stops being pushed rather than crashing.
    TArray<TWeakObjectPtr<UPrimitiveComponent>> KinematicSources;
};

struct FMirrorOptions {
    ECollisionSource Source = ECollisionSource::Auto;

    /// If non-empty, only actors carrying one of these tags are mirrored. The escape hatch for a
    /// large map where mirroring everything is genuinely too much — and it is opt-in, so the
    /// default stays "the whole level" and a partial mirror is always something the operator asked
    /// for rather than something that happened.
    TArray<FName> RequiredTags;

    /// Actors carrying any of these are never mirrored, tags or not.
    TArray<FName> ExcludedTags;

    /// Mirror components whose mobility is Movable, **as kinematic bodies that track their Unreal
    /// pose every frame**.
    ///
    /// Previously these were either skipped, or (with this on) frozen at their load-time pose —
    /// which was a footgun: a lift that rose in Unreal stayed down in Box3D, so the robot rode an
    /// invisible platform or fell through a visible one. They are now tracked, so a movable
    /// obstacle behaves like an obstacle.
    ///
    /// ⚠ Tracking is **one-directional**. The robot is pushed by these bodies; they are never
    /// pushed back, because their pose is dictated from Unreal. Momentum is not conserved across
    /// that boundary. Two-way contact needs §6.0c stage 3.
    ///
    /// ⚠ Kinematic bodies cannot use tri-meshes (Box3D mesh shapes are static-only), so these are
    /// mirrored as convex hulls and are **fatter than they look**.
    bool bIncludeMovable = false;

    /// Also mirror other vehicles' pawns as kinematic bodies, so a Box3D robot feels a drone or a
    /// husky drive into it (§6.0b). A robot never mirrors itself — that exclusion is applied per
    /// robot at registration, not here, because the mirror is shared between robots.
    bool bIncludeOtherVehicles = false;

    /// The channel a robot is assumed to collide on. A component is mirrored only if it blocks
    /// this channel, which is what makes the mirror agree with what the level author intended to
    /// be solid.
    ECollisionChannel BlockingChannel = ECC_Pawn;

    /// If a strict pass mirrors **nothing**, retry once accepting any component that blocks any of
    /// the standard solid channels.
    ///
    /// Not laziness about picking the right channel: a level's collision setup is the level
    /// author's choice and this code cannot know it. The alternative is an operator staring at an
    /// empty physics world and a rebuild cycle per guess. The fallback is loud — it logs that it
    /// engaged — so a relaxed mirror is never mistaken for a clean one.
    bool bRelaxedFallback = true;

    /// Collect ONLY kinematic bodies (vehicles and movable components); emit no static geometry.
    ///
    /// ⚠ This exists because the shared level mirror is taken too early to ever contain a URDF
    /// robot. The mirror runs once, during the FIRST urdfbot's initialisation, and is memoised per
    /// UWorld — so at the moment it is built no urdfbot link components exist yet, not even the
    /// building robot's own. Measured 2026-08-17 with three rovers: every one of them tracked the
    /// same 7 kinematic bodies (the husky and the drone, which already existed), and not one of the
    /// 69 URDF link components. That, and not any architectural limit, is why Box3D robots did not
    /// interact with each other while they did interact with Chaos and FastPhysics vehicles.
    ///
    /// The static half must stay memoised — its pointer identity is what makes the cook shared —
    /// so the fix is to re-collect only the kinematic half, later, when every pawn exists.
    bool bKinematicOnly = false;

    /// Mirror landscape terrain by reading its Chaos heightfield directly.
    ///
    /// ⚠ Terrain does NOT arrive through any of the generic paths.
    /// `ULandscapeHeightfieldCollisionComponent` does not implement
    /// `IInterface_CollisionDataProvider` — the interface appears nowhere in UE 5.6's whole
    /// Landscape module — and it carries no `AggGeom`. So a landscape looks to the generic
    /// extractors exactly like a component with no collision, in every map, and the robot is
    /// handed a world with buildings and props but no ground. That reads at runtime as "the
    /// mirror does not work", when in fact everything except the thing you stand on mirrored.
    bool bIncludeLandscape = true;

    /// Mirror EVERY instance of an instanced/foliage mesh component, not just the base asset.
    ///
    /// ⚠ Off, an `InstancedStaticMeshComponent` is mirrored by the generic path as ONE body at
    /// the component's own transform — which for foliage is the InstancedFoliageActor's origin.
    /// That is worse than skipping it: a phantom tree at the map origin and no collision on any
    /// of the real ones. Modern maps put most of their solid clutter behind ISM/HISM/PCG, so
    /// this is the difference between mirroring a level and mirroring its skeleton.
    bool bIncludeInstancedMeshes = true;

    /// Level-wide ceiling on mirrored instances. A dense foliage map can hold hundreds of
    /// thousands; each one costs a body plus a copy of its geometry. Hitting the cap is reported
    /// loudly rather than silently truncating the world.
    int32 MaxInstances = 50000;

    double DefaultFriction = 0.7;
    double DefaultRestitution = 0.0;
};

/// What the mirror did. Logged at load like the R2 audit, and for the same reason: a level that
/// mirrored 3 of 400 colliders produces a robot that drives through most of the map while looking
/// entirely normal, and the only way to know is to be told the number.
struct FMirrorStats {
    int32 ActorsConsidered = 0;
    int32 ComponentsConsidered = 0;
    int32 ComponentsMirrored = 0;
    int32 ComponentsNoCollisionData = 0;
    int32 ComponentsSkippedMovable = 0;
    int32 ComponentsKinematic = 0;
    int32 ComponentsVehicle = 0;
    int32 ComponentsSkippedNotBlocking = 0;

    /// Terrain, counted separately from everything else. A mirror can be "357 components, 7.2 M
    /// triangles" and still contain no ground at all, so the ground gets its own line.
    int32 LandscapeComponentsMirrored = 0;
    int32 LandscapeComponentsSkipped = 0;

    /// Instanced/foliage geometry, likewise counted apart: "1 component mirrored" and "1 component
    /// mirrored, 4 812 instances" are wildly different worlds and used to print identically.
    int32 InstancedComponents = 0;
    int32 InstancesMirrored = 0;
    int32 InstancesSkippedOverBudget = 0;

    /// The strict pass found nothing and the relaxed pass ran. The mirror is usable, but the
    /// level's collision channels are not what this code assumed, which is worth fixing properly.
    bool bUsedRelaxedFallback = false;
    int64 Triangles = 0;
    int32 Shapes = 0;
    double ElapsedMs = 0.0;

    /// Up to a handful of names, so a warning can say *which* movable colliders were skipped.
    TArray<FString> SkippedMovableNames;

    /// Why individual components were rejected, sampled. Counters say *how many* were lost; these
    /// say *why*, which is the difference between "the level did not mirror" and a diagnosis.
    TArray<FString> RejectionSamples;

    FString Report() const;
};

/// Mirror `World`'s static collision. Game thread only — `GetPhysicsTriMeshData` needs CPU-side
/// collision data and the actor iteration is not thread-safe.
FMirrorResult MirrorLevel(UWorld* World, float WorldToMeters, const FMirrorOptions& Options,
                          FMirrorStats& OutStats);

/// `MirrorLevel`, memoised per UWorld.
///
/// The memo is the whole point rather than an optimisation: every robot in the scene must receive
/// **the same `shared_ptr`**, because that pointer's identity is what `Box3DStaticGeometry`'s cache
/// keys on. Mirror per robot and each one cooks the level again (17.4 ms per 80 k triangles, and it
/// is the robots' *worlds* that then hold N copies of the triangles). Mirror once and hand the same
/// pointer round, and every robot after the first attaches for microseconds.
///
/// Held weakly, so the mirror dies with the level rather than pinning a torn-down map's triangles.
/// `OutStats` is filled only when a mirror actually ran; `bOutFromCache` says which happened.
const FMirrorResult& MirrorLevelShared(UWorld* World, float WorldToMeters,
                                       const FMirrorOptions& Options, FMirrorStats& OutStats,
                                       bool& bOutFromCache);

/// Kinematic-only mirror, deliberately NOT memoised.
///
/// The companion to `MirrorLevelShared`, for the half of the mirror that must be re-collected after
/// every vehicle exists rather than during the first robot's initialisation — see
/// `FMirrorOptions::bKinematicOnly` for what that fixes and how it was measured.
///
/// Uncached on purpose: the static cook is shared because its cost is real and its content is
/// fixed, whereas the set of vehicles is neither. Each robot needs its own kinematic instances
/// anyway (each is pushed independently), so there is nothing to share here.
///
/// ⚠ GAME THREAD ONLY, like `MirrorLevel` — it iterates actors and reads cooked collision.
FMirrorResult MirrorVehicles(UWorld* World, float WorldToMeters, const FMirrorOptions& Options,
                             FMirrorStats& OutStats);

} // namespace UrdfWorldGeometry

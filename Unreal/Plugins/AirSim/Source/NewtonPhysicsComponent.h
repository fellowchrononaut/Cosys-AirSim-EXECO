// Per-actor control over what the Newton sidecar does with this object.
//
// ⚠ WHY A COMPONENT RATHER THAN MORE SCAN RULES. Without one, whether an object reaches the sand
// solver is decided by a heuristic sweep of the level — "is it movable", "does it carry this tag" —
// and the two failure directions are both silent. An object the operator wanted is absent and the
// sand simply ignores it; an object they did not want is mirrored and quietly costs a solver
// rebuild. Neither says anything. Attaching this component makes the intent explicit and local to
// the thing it concerns, and the sweep stops being the authority for that actor.
//
// ⚠ IT IS AN OVERRIDE, NOT A FILTER. An actor WITHOUT this component keeps whatever the level-wide
// scan decides (UrdfMirrorMovable, required tags, and so on) — adding the component to one crate
// must not change the meaning of every other crate. An actor WITH it is governed by it alone, in
// both directions: ticked means mirror even if the scan would have skipped it, unticked means do
// not mirror even if the scan would have taken it. "I ticked the box and nothing happened" and "I
// unticked it and it was mirrored anyway" are both bugs this shape rules out.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "NewtonPhysicsComponent.generated.h"

/// What the sidecar should do with this actor.
UENUM(BlueprintType)
enum class ENewtonPhysicsRole : uint8
{
    /// The actor's pose is pushed to the sidecar every tick. Unreal keeps deciding where it goes;
    /// the robot and the sand are pushed BY it and can never push it back.
    ///
    /// ⚠ ONE-DIRECTIONAL, and the asymmetry is visible: a crate dropped into the bed ploughs the
    /// sand and then falls through it, because the thing deciding the crate's motion is Unreal's
    /// own physics and it has never heard of the sand.
    MirrorToSidecar UMETA(DisplayName = "Mirror into the sidecar (one-way)"),

    /// ⚠ NOT IMPLEMENTED YET — see the class comment. The actor would be SOLVED by Newton alongside
    /// the robot and the sand, and Unreal would render the pose that comes back, exactly as a
    /// sidecar-owned vehicle already does. That is the only way a dropped object can REST on sand.
    /// Selecting it today is refused at load rather than silently downgraded to MirrorToSidecar,
    /// because a crate that sinks when the operator asked for one that floats is precisely the kind
    /// of quiet wrong answer this workstream exists to avoid.
    SolveInSidecar UMETA(DisplayName = "Solve in the sidecar (two-way) - NOT IMPLEMENTED"),
};

/// Attach to any actor with collision to control how the Newton MPM sidecar treats it.
UCLASS(ClassGroup = (EXECOsim), meta = (BlueprintSpawnableComponent),
       HideCategories = (Activation, Collision, Cooking, Tags, ComponentReplication, Variable))
class AIRSIM_API UNewtonPhysicsComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UNewtonPhysicsComponent();

    /// ⚠ THE OVERRIDE ITSELF. Untick to keep this actor OUT of the sidecar even when the level-wide
    /// scan would have taken it; tick to put it in even when the scan would have skipped it.
    /// An actor without this component is unaffected either way.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton Physics",
              meta = (DisplayName = "Mirror in sidecar"))
    bool bMirrorInSidecar = true;

    /// What the sidecar does with it. ⚠ Only MirrorToSidecar works today; SolveInSidecar is
    /// refused at load rather than silently treated as a mirror.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton Physics",
              meta = (DisplayName = "Role", EditCondition = "bMirrorInSidecar"))
    ENewtonPhysicsRole Role = ENewtonPhysicsRole::MirrorToSidecar;

    /// ⚠ SOLID TO THE ROBOT AND SOLID TO THE SAND ARE DIFFERENT QUESTIONS, and they fail
    /// separately. A mirrored object goes into the rigid solver AND becomes an MPM collider, and on
    /// 2026-08-27 an operator had exactly the split case: the Scout collided with a mirrored sphere
    /// while the sand ignored it entirely. Being able to say which one you want makes that a
    /// setting rather than a mystery — and turning the sand half off is also the cheap way to keep
    /// a large moving object in the scene without paying for it in the MPM contact solve.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton Physics",
              meta = (DisplayName = "Interact with MPM sand",
                      EditCondition = "bMirrorInSidecar"))
    bool bInteractWithMpm = true;

    /// Solid to the robot's rigid solver. ⚠ Untick BOTH this and the sand box and the object is
    /// mirrored to no purpose; the loader says so rather than registering a body nothing can touch.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton Physics",
              meta = (DisplayName = "Collide with robots",
                      EditCondition = "bMirrorInSidecar"))
    bool bCollideWithRobots = true;

    /// Friction the sand and the robot feel against this object. ⚠ Separate from Unreal's own
    /// physical material, deliberately: that one governs Chaos contacts in the level, this one
    /// governs a different solver, and quietly reusing one number for both would make a change in
    /// either place move the other.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Newton Physics",
              meta = (DisplayName = "Friction", ClampMin = "0.0", ClampMax = "3.0",
                      EditCondition = "bMirrorInSidecar"))
    float Friction = 0.7f;

    /// What one actor asks for. Returns false when it carries no component, in which case the
    /// caller must fall back to the level-wide scan rather than assume either answer.
    struct FResolved {
        bool bMirror = true;
        ENewtonPhysicsRole Role = ENewtonPhysicsRole::MirrorToSidecar;
        bool bInteractWithMpm = true;
        bool bCollideWithRobots = true;
        float Friction = 0.7f;
    };
    static bool ResolveFor(const AActor* Actor, FResolved& Out);
};

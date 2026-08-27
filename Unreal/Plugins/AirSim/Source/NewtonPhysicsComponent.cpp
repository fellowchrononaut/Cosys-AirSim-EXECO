#include "NewtonPhysicsComponent.h"

#include "GameFramework/Actor.h"

UNewtonPhysicsComponent::UNewtonPhysicsComponent()
{
    // ⚠ NO TICK. This component holds authored intent and nothing else; the mirroring is driven by
    // the vehicle's own game-thread hook, which already runs at the right point in the frame with
    // the physics lock held. A component that ticked would be a second place where mirroring could
    // happen, at a different point in the frame, for no benefit.
    PrimaryComponentTick.bCanEverTick = false;
}

bool UNewtonPhysicsComponent::ResolveFor(const AActor* Actor, FResolved& Out)
{
    if (Actor == nullptr)
        return false;
    const UNewtonPhysicsComponent* Component =
        Actor->FindComponentByClass<UNewtonPhysicsComponent>();
    if (Component == nullptr)
        return false;   // ⚠ NOT "do not mirror" — the caller falls back to the level-wide scan.

    Out.bMirror = Component->bMirrorInSidecar;
    Out.Role = Component->Role;
    Out.bInteractWithMpm = Component->bInteractWithMpm;
    Out.bCollideWithRobots = Component->bCollideWithRobots;
    Out.Friction = Component->Friction;
    return true;
}

// An empty actor attached to one URDF link, so a sensor can be mounted on that link.
//
// Why this exists rather than a change to the sensor classes: every Unreal sensor takes an
// `AActor*` and derives its world pose from that actor's transform composed with the relative pose
// in settings (UnrealLidarSensor.cpp:29 and its six siblings). Mounting a sensor on a link
// therefore needs an actor whose transform *is* the link's.
//
// Attaching an empty actor to the link component gets that for free — Unreal propagates the
// transform — and leaves all seven sensor implementations untouched. The alternative, threading a
// USceneComponent* through every sensor constructor, changes far more code to reach the same
// place, and would have to be redone for every sensor added later.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "UrdfLinkMount.generated.h"

UCLASS()
class AIRSIM_API AUrdfLinkMount : public AActor
{
    GENERATED_BODY()

public:
    AUrdfLinkMount();
};

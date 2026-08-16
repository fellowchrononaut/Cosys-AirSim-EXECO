#include "UrdfLinkMount.h"

#include "Components/SceneComponent.h"

AUrdfLinkMount::AUrdfLinkMount()
{
    // No tick: the transform is maintained by attachment, not by this actor.
    PrimaryActorTick.bCanEverTick = false;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("MountRoot"));
    RootComponent->SetMobility(EComponentMobility::Movable);
}

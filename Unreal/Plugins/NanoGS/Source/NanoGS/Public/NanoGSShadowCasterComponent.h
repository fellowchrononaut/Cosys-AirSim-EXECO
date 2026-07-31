// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NanoGSShadowCasterComponent.generated.h"

/**
 * Marker component for gs.ShadowMode = Proxy Mesh: attach this to an actor whose primitive
 * components (static mesh, etc.) should cast shadows onto Gaussian splats. The shadow-capture
 * manager (UNanoGSShadowManagerSubsystem) scans the world for actors carrying this component and
 * feeds their primitive components into per-light depth-only scene captures.
 *
 * This exists because UNanoGSLightingSettings is a UDeveloperSettings (project-wide singleton
 * config object) and can't cleanly hold a reference to a level-specific actor — a level-placed
 * marker component is the idiomatic way to designate "this mesh casts splat shadows" per level.
 */
UCLASS(ClassGroup = (Rendering), meta = (BlueprintSpawnableComponent))
class NANOGS_API UNanoGSShadowCasterComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UNanoGSShadowCasterComponent(const FObjectInitializer& ObjectInitializer);
};

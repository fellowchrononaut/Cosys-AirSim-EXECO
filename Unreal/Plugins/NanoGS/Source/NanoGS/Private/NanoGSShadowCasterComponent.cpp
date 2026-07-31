// Copyright Epic Games, Inc. All Rights Reserved.

#include "NanoGSShadowCasterComponent.h"

UNanoGSShadowCasterComponent::UNanoGSShadowCasterComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = false;
	bAutoActivate = true;
}

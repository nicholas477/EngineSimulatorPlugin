// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "EngineSimulatorEngineInterface.generated.h"

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UEngineSimulatorEngineInterface : public UInterface
{
	GENERATED_BODY()
};

/**
 * Interface for any UClass that has an engine simulator instance.
 * 
 * Used by EngineSimulatorAudioComponent to produce audio from a UObject that has an engine simulator instance
 */
class ENGINESIMULATORPLUGIN_API IEngineSimulatorEngineInterface
{
	GENERATED_BODY()
public:
	virtual TSharedPtr<class IEngineSimulatorInterface> GetEngineSimulator() const = 0;
};

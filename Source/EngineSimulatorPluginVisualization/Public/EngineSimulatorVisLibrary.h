// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "EngineSimulatorVisLibrary.generated.h"

class UUserWidget;

/**
 * 
 */
UCLASS()
class ENGINESIMULATORPLUGINVISUALIZATION_API UEngineSimulatorVisLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Visualization Library")
		static void SetWindowWidget(UUserWidget* Widget, bool bOnTop = false);
};

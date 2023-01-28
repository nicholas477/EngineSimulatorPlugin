// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include "DSP/Osc.h"
#include "EngineSimulatorAudioComponent.generated.h"

// ========================================================================
// UEngineSimulatorAudioComponent
// Synth component class which implements USynthComponent
// This is a simple hello-world type example which generates a sine-wave
// tone using a DSP oscillator class and implements a single function to set
// the frequency. To enable example:
// 1. Ensure "SignalProcessing" is added to project's .Build.cs in PrivateDependencyModuleNames
// 2. Enable macro below that includes code utilizing SignalProcessing Oscilator
// ========================================================================

#define SYNTHCOMPONENT_EX_OSCILATOR_ENABLED 0

class IEngineSimulatorInterface;
class UEngineSimulatorWheeledVehicleMovementComponent;

UCLASS(ClassGroup = "Engine Simulator", meta = (BlueprintSpawnableComponent))
class ENGINESIMULATORPLUGIN_API UEngineSimulatorAudioComponent : public USynthComponent
{
	GENERATED_BODY()

public:
	UEngineSimulatorAudioComponent(const FObjectInitializer& ObjectInitializer);

	// Called when synth is created
	virtual bool Init(int32& InSampleRate) override;

	// Called to generate more audio
	virtual int32 OnGenerateAudio(float* OutAudio, int32 NumSamples) override;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Engine Simulator Audio Component")
		TWeakObjectPtr<UEngineSimulatorWheeledVehicleMovementComponent> EngineComponent;

	TSharedPtr<IEngineSimulatorInterface> EngineSimulator;
};
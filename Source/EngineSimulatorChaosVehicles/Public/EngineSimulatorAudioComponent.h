// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include "EngineSimulatorAudioComponent.generated.h"

class IEngineSimulatorInterface;
class UEngineSimulatorWheeledVehicleMovementComponent;
class UAudioBus;

UCLASS(ClassGroup = "Engine Simulator", meta = (BlueprintSpawnableComponent))
class ENGINESIMULATORCHAOSVEHICLES_API UEngineSimulatorAudioComponent : public USynthComponent
{
	GENERATED_BODY()

public:
	UEngineSimulatorAudioComponent(const FObjectInitializer& ObjectInitializer);

	virtual void OnComponentCreated() override;

	// Called when synth is created
	virtual bool Init(int32& InSampleRate) override;

	// Called to generate more audio
	virtual int32 OnGenerateAudio(float* OutAudio, int32 NumSamples) override;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	// If true, then this component will look for the engine simulator movement component on its owner
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Engine Simulator Audio Component")
		bool bAutomaticallySetEngineComponent;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Engine Simulator Audio Component", meta=(EditCondition="!bAutomaticallySetEngineComponent"))
		TWeakObjectPtr<UEngineSimulatorWheeledVehicleMovementComponent> EngineComponent;

	// Disables sound output, routes it through an audio bus instead for further processing.
	// Use this if you want to apply metasound filters
	//
	// NOTE: This is currently not functional
	UPROPERTY(BlueprintReadOnly, EditAnywhere, Category = "Engine Simulator Audio Component")
		bool bOutputToAudioBus;

	// Use this if you want to run the sound through metasounds
	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Audio Component")
		UAudioBus* AudioBus;

	TSharedPtr<IEngineSimulatorInterface> EngineSimulator;
};
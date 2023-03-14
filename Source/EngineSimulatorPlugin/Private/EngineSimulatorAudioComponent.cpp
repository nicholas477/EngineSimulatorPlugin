// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorAudioComponent.h"

#include "EngineSimulatorWheeledVehicleMovementComponent.h"

UEngineSimulatorAudioComponent::UEngineSimulatorAudioComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	SetComponentTickEnabled(true);
	bAutoActivate = true;

	bAutomaticallySetEngineComponent = true;
}

bool UEngineSimulatorAudioComponent::Init(int32& InSampleRate)
{
	check(IsInGameThread());

	if (bAutomaticallySetEngineComponent)
	{
		auto* VHMC = Cast<UEngineSimulatorWheeledVehicleMovementComponent>(GetOwner()->GetComponentByClass(UEngineSimulatorWheeledVehicleMovementComponent::StaticClass()));
		if (VHMC)
		{
			EngineComponent = VHMC;
		}
	}

	NumChannels = 1;
	InSampleRate = 44000;
	return true;
}

int32 UEngineSimulatorAudioComponent::OnGenerateAudio(float* OutAudio, int32 NumSamples)
{
	if (EngineSimulator.IsValid())
	{
		int32 GeneratedSound = EngineSimulator->GenerateAudio(OutAudio, NumSamples);

		if (GeneratedSound < NumSamples)
		{
			UE_LOG(LogTemp, Warning, TEXT("UEngineSimulatorAudioComponent::OnGenerateAudio: Buffer Underrun, not enough samples. Is EngineSimulator ticking fast enough?"))
		}

		return GeneratedSound;
	}

	return NumSamples;
}

void UEngineSimulatorAudioComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	EngineSimulator = EngineComponent.IsValid() ? EngineComponent->GetEngineSimulator() : nullptr;
}

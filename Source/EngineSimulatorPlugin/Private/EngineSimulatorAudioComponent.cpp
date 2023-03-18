// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorAudioComponent.h"

#include "EngineSimulatorWheeledVehicleMovementComponent.h"
#include "Sound/AudioBus.h"
#include "AudioMixerDevice.h"

UEngineSimulatorAudioComponent::UEngineSimulatorAudioComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	SetComponentTickEnabled(true);
	bAutoActivate = true;

	bAutomaticallySetEngineComponent = true;

	bOutputToAudioBus = false;
}

void UEngineSimulatorAudioComponent::OnComponentCreated()
{
	Super::OnComponentCreated();

	if (bOutputToAudioBus)
	{
		AudioBus = NewObject<UAudioBus>(this);
		FSoundSourceBusSendInfo BusSendInfo;
		BusSendInfo.AudioBus = AudioBus;
		if (BusSends.Num() == 0)
		{
			BusSends.Add(BusSendInfo);
		}
		else
		{
			BusSends[0] = BusSendInfo;
		}
	}
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

	EngineSimulator = EngineComponent.IsValid() ? EngineComponent->GetEngineSimulator() : nullptr;

	SetOutputToBusOnly(bOutputToAudioBus);
	if (AudioBus)
	{
		if (Audio::FMixerDevice* MixerDevice = FAudioDeviceManager::GetAudioMixerDeviceFromWorldContext(this))
		{
			uint32 AudioBusId = AudioBus->GetUniqueID();
			int32 AudioBusNumChannels = (int32)AudioBus->AudioBusChannels + 1;
			MixerDevice->StartAudioBus(AudioBusId, AudioBusNumChannels, false);
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

// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorComponent.h"

UEngineSimulatorComponent::UEngineSimulatorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UEngineSimulatorComponent::BeginPlay()
{
	Super::BeginPlay();

	EngineSimulatorThread = MakeUnique<FEngineSimulatorThread>(FEngineSimulatorParameters());
}

void UEngineSimulatorComponent::BeginDestroy()
{
	EngineSimulatorThread.Reset();

	Super::BeginDestroy();
}

// Called every frame
void UEngineSimulatorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (EngineSimulatorThread)
	{
		EngineSimulatorOutput = EngineSimulatorThread->GetEngineOutput();
		EngineSimulatorThread->SetEngineInput(EngineSimulatorInput);
	}
}

void UEngineSimulatorComponent::GearUp()
{
	CurrentGear = FMath::Clamp(CurrentGear + 1, -1, EngineSimulatorOutput.NumGears - 1);
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->EnqueueUpdate([=](IEngineSimulatorInterface* EngineInterface)
		{
			EngineInterface->SetGear(CurrentGear);
		});
	}

}

void UEngineSimulatorComponent::GearDown(bool bNewGearDown)
{
	CurrentGear = FMath::Clamp(CurrentGear - 1, -1, EngineSimulatorOutput.NumGears - 1);
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->EnqueueUpdate([=](IEngineSimulatorInterface* EngineInterface)
		{
			EngineInterface->SetGear(CurrentGear);
		});
	}
}

void UEngineSimulatorComponent::SetClutchPressure(float Pressure)
{
	ClutchPressure = Pressure;
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->EnqueueUpdate([=](IEngineSimulatorInterface* EngineInterface)
		{
			EngineInterface->SetClutchPressure(Pressure);
		});
	}
}

void UEngineSimulatorComponent::SetStarterEnabled(bool bEnabled)
{
	bStarterEnabled = bEnabled;
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->EnqueueUpdate([=](IEngineSimulatorInterface* EngineInterface)
		{
			EngineInterface->SetStarterEnabled(bEnabled);
		});
	}
}

void UEngineSimulatorComponent::RespawnEngine()
{
	EngineSimulatorThread = MakeUnique<FEngineSimulatorThread>(FEngineSimulatorParameters());

	EngineSimulatorThread->EnqueueUpdate([=](IEngineSimulatorInterface* EngineInterface)
	{
		EngineInterface->SetIgnitionEnabled(true);
		EngineInterface->SetStarterEnabled(bStarterAutomaticallyEnabled);
	});

	bStarterEnabled = bStarterAutomaticallyEnabled;
	CurrentGear = -1;
}

TSharedPtr<class IEngineSimulatorInterface> UEngineSimulatorComponent::GetEngineSimulator() const
{
	if (EngineSimulatorThread)
	{
		return EngineSimulatorThread->GetEngineSimulator();
	}

	return nullptr;
}

#if WITH_GAMEPLAY_DEBUGGER
void UEngineSimulatorComponent::DescribeSelfToGameplayDebugger(FGameplayDebuggerCategory* GameplayDebugger) const
{
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->PrintGameplayDebuggerInfo(GameplayDebugger);
	}
}
#endif


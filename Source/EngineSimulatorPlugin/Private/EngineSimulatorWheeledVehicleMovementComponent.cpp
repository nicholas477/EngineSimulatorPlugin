// Fill out your copyright notice in the Description page of Project Settings.

#include "EngineSimulatorWheeledVehicleMovementComponent.h"
#include "EngineSimulatorWheeledVehicleSimulation.h"
#include "ChaosVehicleMovementComponent.h"
#include "VehicleUtility.h"
#include "Sound/SoundWaveProcedural.h"
#include "ChaosVehicleManager.h"
#include "EngineSimulator.h"

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebuggerCategory.h"
#endif

UEngineSimulatorWheeledVehicleMovementComponent::UEngineSimulatorWheeledVehicleMovementComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;

	//SleepThreshold = 0; // Disable vehicle sleep. This is required
}

void UEngineSimulatorWheeledVehicleMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (VehicleSimulationPT.Get())
	{
		UEngineSimulatorWheeledVehicleSimulation* VS = ((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get());
		LastEngineSimulatorOutput = VS->GetLastOutput();
		UE_LOG(LogTemp, Warning, TEXT("Output! %lf"), LastEngineSimulatorOutput.LastFrameTime);

		if (PVehicleOutput)
		{
			PVehicleOutput->CurrentGear = LastEngineSimulatorOutput.CurrentGear + 1;
			PVehicleOutput->TargetGear = LastEngineSimulatorOutput.CurrentGear + 1;
			PVehicleOutput->EngineRPM = LastEngineSimulatorOutput.RPM;
		}

		EngineSetup.MaxRPM = LastEngineSimulatorOutput.Redline;
	}
}

void UEngineSimulatorWheeledVehicleMovementComponent::SetEngineSimChangeGearUp(bool bNewGearUp)
{
	if (VehicleSimulationPT && bNewGearUp)
	{
		if (CurrentGear != FMath::Clamp(CurrentGear + 1, -1, LastEngineSimulatorOutput.NumGears - 1))
		{
			CurrentGear = FMath::Clamp(CurrentGear + 1, -1, LastEngineSimulatorOutput.NumGears - 1);
			((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
			{
				EngineInterface->SetGear(CurrentGear);
			});
		}
	}
}

void UEngineSimulatorWheeledVehicleMovementComponent::SetEngineSimChangeGearDown(bool bNewGearDown)
{
	if (VehicleSimulationPT && bNewGearDown)
	{
		CurrentGear = FMath::Clamp(CurrentGear - 1, -1, LastEngineSimulatorOutput.NumGears - 1);
		((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
		{
			EngineInterface->SetGear(CurrentGear);
		});
	}
}

void UEngineSimulatorWheeledVehicleMovementComponent::RespawnEngine()
{
	FEngineSimulatorParameters EngineParameters;
	EngineParameters.bShowGUI = false;

	// Make the Vehicle Simulation class that will be updated from the physics thread async callback
	((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->Reset(EngineParameters);

	((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
	{
		EngineInterface->SetIgnitionEnabled(true);
		EngineInterface->SetStarterEnabled(bStarterAutomaticallyEnabled);
	});

	bStarterEnabled = bStarterAutomaticallyEnabled;
	CurrentGear = -1;
}

void UEngineSimulatorWheeledVehicleMovementComponent::SetClutchPressure(float Pressure)
{
	ClutchPressure = Pressure;
	if (VehicleSimulationPT)
	{
		((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
			{
				EngineInterface->SetClutchPressure(Pressure);
			}
		);
	}
}

void UEngineSimulatorWheeledVehicleMovementComponent::SetStarterEnabled(bool bEnabled)
{
	bStarterEnabled = bEnabled;
	if (VehicleSimulationPT)
	{
		((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
			{
				EngineInterface->SetStarterEnabled(bStarterEnabled);
			}
		);
	}
}

void UEngineSimulatorWheeledVehicleMovementComponent::CreateVehicle()
{
	Super::CreateVehicle();

	((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
	{
		EngineInterface->SetIgnitionEnabled(true);
		EngineInterface->SetStarterEnabled(bStarterAutomaticallyEnabled);
	});

	bStarterEnabled = bStarterAutomaticallyEnabled;
}

TUniquePtr<Chaos::FSimpleWheeledVehicle> UEngineSimulatorWheeledVehicleMovementComponent::CreatePhysicsVehicle()
{
	FEngineSimulatorParameters EngineParameters;
	EngineParameters.bShowGUI = false;

	// Make the Vehicle Simulation class that will be updated from the physics thread async callback
	VehicleSimulationPT = MakeUnique<UEngineSimulatorWheeledVehicleSimulation>(EngineParameters);

	((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->AsyncUpdateSimulation([=](IEngineSimulatorInterface* EngineInterface)
	{
		EngineInterface->SetIgnitionEnabled(true);
		EngineInterface->SetStarterEnabled(bStarterAutomaticallyEnabled);
	});

	bStarterEnabled = bStarterAutomaticallyEnabled;
	CurrentGear = -1;

	return UChaosVehicleMovementComponent::CreatePhysicsVehicle();
}

TSharedPtr<IEngineSimulatorInterface> UEngineSimulatorWheeledVehicleMovementComponent::GetEngineSimulator() const
{
	if (VehicleSimulationPT)
	{
		return ((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->GetEngineSimulator();
	}

	return nullptr;
}

void UEngineSimulatorWheeledVehicleMovementComponent::ParallelUpdate(float DeltaSeconds)
{
	Super::ParallelUpdate(DeltaSeconds);

	if (PVehicleOutput)
	{
		UEngineSimulatorWheeledVehicleSimulation* VS = ((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get());
		auto LastOutput = VS->GetLastOutput();

		PVehicleOutput->CurrentGear = LastOutput.CurrentGear + 1;
		PVehicleOutput->TargetGear = LastOutput.CurrentGear + 1;
		PVehicleOutput->EngineRPM = LastOutput.RPM;
	}
}

#if WITH_GAMEPLAY_DEBUGGER
void UEngineSimulatorWheeledVehicleMovementComponent::DescribeSelfToGameplayDebugger(FGameplayDebuggerCategory* DebuggerCategory) const
{
	if (VehicleSimulationPT)
	{
		((UEngineSimulatorWheeledVehicleSimulation*)VehicleSimulationPT.Get())->PrintGameplayDebuggerInfo(DebuggerCategory);
	}
}
#endif

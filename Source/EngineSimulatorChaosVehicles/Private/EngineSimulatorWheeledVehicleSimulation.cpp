// Fill out your copyright notice in the Description page of Project Settings.

#include "EngineSimulatorWheeledVehicleSimulation.h"
#include "EngineSimulatorWheeledVehicleMovementComponent.h"
#include "ChaosVehicleMovementComponent.h"
#include "EngineSimulator.h"
#include "VehicleUtility.h"
#include "Sound/SoundWaveProcedural.h"
#include "ChaosVehicleManager.h"

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebuggerCategory.h"
#endif // WITH_GAMEPLAY_DEBUGGER

UEngineSimulatorWheeledVehicleSimulation::UEngineSimulatorWheeledVehicleSimulation(const FEngineSimulatorParameters& InParameters)
	: Parameters(InParameters)
{
}

void UEngineSimulatorWheeledVehicleSimulation::Init(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicleIn)
{
	UChaosWheeledVehicleSimulation::Init(PVehicleIn);

	EngineSimulatorThread = MakeUnique<FEngineSimulatorThread>(Parameters);
}

void UEngineSimulatorWheeledVehicleSimulation::ProcessMechanicalSimulation(float DeltaTime)
{
	if (EngineSimulatorThread)
	{
		FEngineSimulatorOutput SimulationOutput = EngineSimulatorThread->GetEngineOutput();
		{
			FScopeLock Lock(&LastOutputMutex);
			LastOutput = SimulationOutput;
		}

		auto& PTransmission = PVehicle->GetTransmission();

		// Sample the wheel rotation
		bool bWheelsInContact = false;
		float WheelRPM = 0;
		int32 SampledWheels = 0;
		for (int I = 0; I < PVehicle->Wheels.Num(); I++)
		{
			if (PVehicle->Wheels[I].EngineEnabled
				&& PVehicle->Wheels[I].bInContact)
			{
				WheelRPM += PVehicle->Wheels[I].GetWheelRPM();
				bWheelsInContact = true;
				SampledWheels++;
			}
		}

		// Average RPM between the wheels sampled
		WheelRPM /= SampledWheels > 0 ? SampledWheels : 1;

		float DynoSpeed = WheelRPM * PTransmission.Setup().FinalDriveRatio;

		FEngineSimulatorInput NextInput;
		NextInput.DeltaTime = DeltaTime;
		NextInput.InContactWithGround = bWheelsInContact;
		NextInput.EngineRPM = DynoSpeed;
		EngineSimulatorThread->SetEngineInput(NextInput);

		// apply drive torque to wheels
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		{
			Chaos::FSimpleWheelSim& PWheel = PVehicle->Wheels[WheelIdx];
			if (PWheel.Setup().EngineEnabled)
			{
				float OutWheelTorque = Chaos::TorqueMToCm(SimulationOutput.Torque * PTransmission.Setup().FinalDriveRatio) * PWheel.Setup().TorqueRatio;
				PWheel.SetDriveTorque(OutWheelTorque);
			}
			else
			{
				PWheel.SetDriveTorque(0.f);
			}
		}

	}
}

void UEngineSimulatorWheeledVehicleSimulation::ApplyInput(const FControlInputs& ControlInputs, float DeltaTime)
{
	UChaosWheeledVehicleSimulation::ApplyInput(ControlInputs, DeltaTime);

	FControlInputs ModifiedInputs = ControlInputs;

	AsyncUpdateSimulation([ThrottleInput = ModifiedInputs.ThrottleInput](IEngineSimulatorInterface* EngineInterface)
	{
		EngineInterface->SetSpeedControl(ThrottleInput * ThrottleInput);
	});
}

void UEngineSimulatorWheeledVehicleSimulation::AsyncUpdateSimulation(TFunction<void(IEngineSimulatorInterface*)> InCallable)
{
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->EnqueueUpdate(InCallable);
	}
}

FEngineSimulatorOutput UEngineSimulatorWheeledVehicleSimulation::GetLastOutput()
{
	if (EngineSimulatorThread)
	{
		return EngineSimulatorThread->GetEngineOutput();
	}
	else
	{
		FScopeLock Lock(&LastOutputMutex);
		return LastOutput;
	}
}

void UEngineSimulatorWheeledVehicleSimulation::Reset(const FEngineSimulatorParameters& InParameters)
{
	EngineSimulatorThread = MakeUnique<FEngineSimulatorThread>(InParameters);
}

#if WITH_GAMEPLAY_DEBUGGER
void UEngineSimulatorWheeledVehicleSimulation::PrintGameplayDebuggerInfo(FGameplayDebuggerCategory* GameplayDebugger)
{
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->PrintGameplayDebuggerInfo(GameplayDebugger);
	}
}
#endif

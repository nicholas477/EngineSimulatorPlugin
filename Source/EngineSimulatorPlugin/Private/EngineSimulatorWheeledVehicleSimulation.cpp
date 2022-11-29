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

DECLARE_STATS_GROUP(TEXT("EngineSimulatorPlugin"), STATGROUP_EngineSimulatorPlugin, STATGROUP_Advanced);
DECLARE_CYCLE_STAT(TEXT("EngineThread:UpdateSimulation"), STAT_EngineSimulatorPlugin_UpdateSimulation, STATGROUP_EngineSimulatorPlugin);

FEngineSimulatorThread::FEngineSimulatorThread(const FEngineSimulatorParameters& InParameters)
	: bStopRequested(false)
	, Semaphore(FGenericPlatformProcess::GetSynchEventFromPool(false))
	, Parameters(InParameters)
	, Thread(FRunnableThread::Create(this, TEXT("Engine Simulator Thread"))) // TODO: change this later
{
	
}

FEngineSimulatorThread::~FEngineSimulatorThread()
{
	if (Thread)
	{
		Thread->Kill(true);
		delete Thread;
		Thread = nullptr;
	}

	FGenericPlatformProcess::ReturnSynchEventToPool(Semaphore);
	Semaphore = nullptr;
}

bool FEngineSimulatorThread::Init()
{
	return true;
}

uint32 FEngineSimulatorThread::Run()
{
	EngineSimulator = CreateEngine(Parameters);

	while (!bStopRequested)
	{
		Semaphore->Wait();
		if (bStopRequested)
		{
			break;
		}
		else
		{
			FEngineSimulatorInput ThisInput;
			{
				FScopeLock Lock(&InputMutex);
				ThisInput = Input;
			}

			{
				SCOPE_CYCLE_COUNTER(STAT_EngineSimulatorPlugin_UpdateSimulation);
				float DynoSpeed = ThisInput.EngineRPM * (EngineSimulator->GetGearRatio() == 0.f ? 1000000.f : EngineSimulator->GetGearRatio());
				EngineSimulator->SetDynoSpeed(DynoSpeed);
				EngineSimulator->SetDynoEnabled(ThisInput.InContactWithGround);

				TFunction<void(IEngineSimulatorInterface*)> SimulationUpdate;
				while (UpdateQueue.Dequeue(SimulationUpdate))
				{
					SimulationUpdate(EngineSimulator.Get());
				}

				EngineSimulator->Simulate(ThisInput.DeltaTime);

				float TransmissionTorque = EngineSimulator->GetFilteredDynoTorque() * EngineSimulator->GetGearRatio();

#if WITH_GAMEPLAY_DEBUGGER
				GameplayDebuggerPrint = [
						bHasEngine = EngineSimulator->HasEngine(),
						EngineName = EngineSimulator->GetName(),
						T = TransmissionTorque,
						RPM = EngineSimulator->GetRPM(),
						Speed = EngineSimulator->GetSpeed(),
						DynoSpeed = DynoSpeed,
						Grounded = ThisInput.InContactWithGround
					](FGameplayDebuggerCategory* GameplayDebugger)
				{
					if (bHasEngine)
					{
						GameplayDebugger->AddTextLine(
							FString::Printf(TEXT("{yellow}Engine: {white}%s"), *EngineName)
						);
						GameplayDebugger->AddTextLine(
							FString::Printf(TEXT("\t{yellow}Torque at the wheel: {white}%f"), T)
						);
						GameplayDebugger->AddTextLine(
							FString::Printf(TEXT("\t{yellow}RPM: {white}%f"), RPM)
						);
						GameplayDebugger->AddTextLine(
							FString::Printf(TEXT("\t{yellow}Dyno RPM: {white}%f"), DynoSpeed)
						);
						if (!Grounded)
						{
							GameplayDebugger->AddTextLine("\t{green}Engine in air, dyno disabled");
						}
					}
					else
					{
						GameplayDebugger->AddTextLine("{red}FAILED TO LOAD ENGINE");
					}
				};
#endif

				{
					FScopeLock Lock(&OutputMutex);
					Output.Torque = TransmissionTorque;
					Output.RPM = EngineSimulator->GetRPM();
					Output.Redline = EngineSimulator->GetRedLine();
					Output.Horsepower = EngineSimulator->GetDynoPower();
					Output.Name = EngineSimulator->GetName();
					Output.NumGears = EngineSimulator->GetGearCount();
					Output.FrameCounter = ThisInput.FrameCounter + 1;
				}
			}
		}
	}

	EngineSimulator.Reset();

	return 0;
}

void FEngineSimulatorThread::Stop()
{
	bStopRequested = true;
	Semaphore->Trigger();
	Thread->WaitForCompletion();
}

void FEngineSimulatorThread::Exit()
{
	//Stop();
}

void FEngineSimulatorThread::Trigger()
{
	Semaphore->Trigger();
}

UEngineSimulatorWheeledVehicleSimulation::UEngineSimulatorWheeledVehicleSimulation(TArray<class UChaosVehicleWheel*>& WheelsIn, 
	const FEngineSimulatorParameters& InParameters)
	: UChaosWheeledVehicleSimulation(WheelsIn)
	, Parameters(InParameters)
{
	EngineSimulatorThread = MakeUnique<FEngineSimulatorThread>(InParameters);
}

void UEngineSimulatorWheeledVehicleSimulation::ProcessMechanicalSimulation(float DeltaTime)
{
	if (EngineSimulatorThread)
	{
		// Retrieve output from the last frame
		FEngineSimulatorOutput SimulationOutput;
		{
			FScopeLock Lock(&EngineSimulatorThread->OutputMutex);
			SimulationOutput = EngineSimulatorThread->Output;
		}

		{
			FScopeLock Lock(&LastOutputMutex);
			LastOutput = SimulationOutput;
		}

		auto& PTransmission = PVehicle->GetTransmission();

		bool bWheelsInContact = false;
		float WheelRPM = 0;
		int32 SampledWheels = 0;
		for (int I = 0; I < PVehicle->Wheels.Num(); I++)
		{
			if (PVehicle->Wheels[I].EngineEnabled 
				&& PVehicle->Wheels[I].bInContact 
				&& !PVehicle->Wheels[I].IsSlipping())
			{
				WheelRPM += PVehicle->Wheels[I].GetWheelRPM();
				bWheelsInContact = true;
				SampledWheels++;
			}
		}

		if (SampledWheels > 0)
		{
			WheelRPM /= SampledWheels;
		}

		float DynoSpeed = WheelRPM * PTransmission.Setup().FinalDriveRatio;

		// Do input here...
		{
			EngineSimulatorThread->InputMutex.Lock();
			EngineSimulatorThread->Input.DeltaTime = DeltaTime;
			EngineSimulatorThread->Input.InContactWithGround = bWheelsInContact;
			//if (bWheelRPMWasSet)
			{
				EngineSimulatorThread->Input.EngineRPM = DynoSpeed;
			}
			EngineSimulatorThread->Input.FrameCounter = GFrameCounter;
			EngineSimulatorThread->InputMutex.Unlock();
		}
		EngineSimulatorThread->Trigger();

		// apply drive torque to wheels
		for (int WheelIdx = 0; WheelIdx < Wheels.Num(); WheelIdx++)
		{
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			//PWheel.bInContact = true; // fuck you epic
			if (PWheel.Setup().EngineEnabled)
			{
				float OutWheelTorque = Chaos::TorqueMToCm(SimulationOutput.Torque * PTransmission.Setup().FinalDriveRatio) * PWheel.Setup().TorqueRatio;
				//if (FMath::Sign(PWheel.GetWheelRPM()) < 0.f)
				//{
				//	PWheel.SetDriveTorque(FMath::Abs(OutWheelTorque));
				//}
				//else
				//{
					PWheel.SetDriveTorque(OutWheelTorque);
				//}
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

	//PEngine.SetThrottle(ModifiedInputs.ThrottleInput * ModifiedInputs.ThrottleInput);
	AsyncUpdateSimulation([ThrottleInput = ModifiedInputs.ThrottleInput](IEngineSimulatorInterface* EngineInterface)
	{
		//UE_LOG(LogTemp, Warning, TEXT("Setting engine speed: %f"), ThrottleInput * ThrottleInput);
		EngineInterface->SetSpeedControl(ThrottleInput * ThrottleInput);
	});
}

void UEngineSimulatorWheeledVehicleSimulation::AsyncUpdateSimulation(TFunction<void(IEngineSimulatorInterface*)> InCallable)
{
	if (EngineSimulatorThread)
	{
		EngineSimulatorThread->UpdateQueue.Enqueue(InCallable);
	}
}

FEngineSimulatorOutput UEngineSimulatorWheeledVehicleSimulation::GetLastOutput()
{
	FScopeLock Lock(&LastOutputMutex);
	return LastOutput;
}

void UEngineSimulatorWheeledVehicleSimulation::Reset(const FEngineSimulatorParameters& InParameters)
{
	EngineSimulatorThread = MakeUnique<FEngineSimulatorThread>(InParameters);
}

#if WITH_GAMEPLAY_DEBUGGER
void UEngineSimulatorWheeledVehicleSimulation::PrintGameplayDebuggerInfo(FGameplayDebuggerCategory* GameplayDebugger)
{
	if (EngineSimulatorThread && EngineSimulatorThread->GameplayDebuggerPrint)
	{
		EngineSimulatorThread->GameplayDebuggerPrint(GameplayDebugger);
	}
}
#endif

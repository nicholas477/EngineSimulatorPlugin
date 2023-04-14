// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorThread.h"

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebuggerCategory.h"
#endif // WITH_GAMEPLAY_DEBUGGER


FEngineSimulatorThread::FEngineSimulatorThread(const FEngineSimulatorParameters& InParameters)
	: bStopRequested(false)
	, Semaphore(FGenericPlatformProcess::GetSynchEventFromPool(false))
	, Parameters(InParameters)
	, Thread(FRunnableThread::Create(this, *GetThreadName()))
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

void FEngineSimulatorThread::SetEngineInput(const FEngineSimulatorInput& InInput)
{
	FScopeLock Lock(&InputMutex);
	Input = InInput;
}

FEngineSimulatorOutput FEngineSimulatorThread::GetEngineOutput()
{
	FScopeLock Lock(&OutputMutex);
	return Output;
}

void FEngineSimulatorThread::EnqueueUpdate(const TFunction<void(IEngineSimulatorInterface*)>& UpdateFunction)
{
	UpdateQueue.Enqueue(UpdateFunction);
}

bool FEngineSimulatorThread::Init()
{
	return true;
}

DECLARE_STATS_GROUP(TEXT("EngineSimulatorPlugin"), STATGROUP_EngineSimulatorPlugin, STATGROUP_Advanced);
DECLARE_CYCLE_STAT(TEXT("EngineThread:UpdateSimulation"), STAT_EngineSimulatorPlugin_UpdateSimulation, STATGROUP_EngineSimulatorPlugin);
DECLARE_FLOAT_COUNTER_STAT_EXTERN(TEXT("Simulation Frame Time (ms)"), STAT_EngineSimulatorFrame, STATGROUP_EngineSimulatorPlugin, );
DECLARE_FLOAT_COUNTER_STAT_EXTERN(TEXT("Simulation Input Delta Time (ms)"), STAT_EngineSimulatorInputDeltaTime, STATGROUP_EngineSimulatorPlugin, );

DEFINE_STAT(STAT_EngineSimulatorFrame);
DEFINE_STAT(STAT_EngineSimulatorInputDeltaTime);

uint32 FEngineSimulatorThread::Run()
{
	EngineSimulator = CreateEngine(Parameters);

	LastSimulationTime = FPlatformTime::Seconds();

	const double SemaphoreWaitTime = 8.0 / 1000; // Wait time before next time

	while (!bStopRequested)
	{
		double WaitTime = (LastSimulationTime + SemaphoreWaitTime) - FPlatformTime::Seconds();
		if (WaitTime > 0)
		{
			if (Semaphore->Wait(FTimespan::FromSeconds(WaitTime)))
			{
				if (bStopRequested)
				{
					break;
				}
			}
		}

		if (bStopRequested)
		{
			break;
		}
		else
		{
			Tick();
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

#if WITH_GAMEPLAY_DEBUGGER
void FEngineSimulatorThread::PrintGameplayDebuggerInfo(FGameplayDebuggerCategory* GameplayDebugger)
{
	if (GameplayDebuggerPrint)
	{
		GameplayDebuggerPrint(GameplayDebugger);
	}
}
#endif

void FEngineSimulatorThread::Tick()
{
	FEngineSimulatorInput ThisInput;
	{
		FScopeLock Lock(&InputMutex);
		ThisInput = Input;
	}

	ThisInput.DeltaTime = FPlatformTime::Seconds() - LastSimulationTime;
	LastSimulationTime = FPlatformTime::Seconds();

	{
		SCOPE_CYCLE_COUNTER(STAT_EngineSimulatorPlugin_UpdateSimulation);
		float DynoSpeed = ThisInput.EngineRPM * (EngineSimulator->GetGearRatio() == 0.f ? 1000000.f : EngineSimulator->GetGearRatio());
		EngineSimulator->SetDynoSpeed(DynoSpeed);
		EngineSimulator->SetDynoEnabled(ThisInput.bDynoEnabled);

		TFunction<void(IEngineSimulatorInterface*)> SimulationUpdate;
		while (UpdateQueue.Dequeue(SimulationUpdate))
		{
			SimulationUpdate(EngineSimulator.Get());
		}

		SET_FLOAT_STAT(STAT_EngineSimulatorInputDeltaTime, ThisInput.DeltaTime * 1000);
		double SimulationStartTime = FPlatformTime::Seconds();

		EngineSimulator->Simulate(ThisInput.DeltaTime);

		double SimulationTime = FPlatformTime::Seconds() - SimulationStartTime;
		SET_FLOAT_STAT(STAT_EngineSimulatorFrame, SimulationTime * 1000);

		float TransmissionTorque = EngineSimulator->GetFilteredDynoTorque() * EngineSimulator->GetGearRatio();

		FillOutDebugData(TransmissionTorque, DynoSpeed, ThisInput);

		FEngineSimulatorOutput NewOutput;
		NewOutput.Torque = TransmissionTorque;
		NewOutput.RPM = EngineSimulator->GetRPM();
		NewOutput.Redline = EngineSimulator->GetRedLine();
		NewOutput.Horsepower = EngineSimulator->GetDynoPower();
		NewOutput.Name = EngineSimulator->GetName();
		NewOutput.CurrentGear = EngineSimulator->GetGear();
		NewOutput.NumGears = EngineSimulator->GetGearCount();
		NewOutput.LastFrameTime = FPlatformTime::Seconds();
		UpdateEngineOutput(NewOutput);
	}
}

void FEngineSimulatorThread::FillOutDebugData(float TransmissionTorque, float DynoSpeed, const FEngineSimulatorInput& ThisInput)
{
#if WITH_GAMEPLAY_DEBUGGER
	GameplayDebuggerPrint = [
		bHasEngine = EngineSimulator->HasEngine(),
		EngineName = EngineSimulator->GetName(),
		T = TransmissionTorque,
		RPM = EngineSimulator->GetRPM(),
		Speed = EngineSimulator->GetSpeed(),
		DynoSpeed = DynoSpeed,
		Grounded = ThisInput.bDynoEnabled,
		StarterEnabled = EngineSimulator->IsStarterEnabled(),
		IgnitionEnabled = EngineSimulator->IsIgnitionEnabled()
	](FGameplayDebuggerCategory* GameplayDebugger)
	{
		GameplayDebugger->AddTextLine("hgello");

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
			GameplayDebugger->AddTextLine(
				FString::Printf(TEXT("\t{yellow}Starter Enabled: {white}%s"), StarterEnabled ? TEXT("true") : TEXT("false"))
			);
			GameplayDebugger->AddTextLine(
				FString::Printf(TEXT("\t{yellow}Ignition Enabled: {white}%s"), IgnitionEnabled ? TEXT("true") : TEXT("false"))
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
}

FString FEngineSimulatorThread::GetThreadName()
{
	static int32 ThreadNum = 0;
	return FString::Printf(TEXT("Engine Simulator Thread %d"), ThreadNum++);
}

void FEngineSimulatorThread::UpdateEngineOutput(const FEngineSimulatorOutput& InOutput)
{
	FScopeLock Lock(&OutputMutex);
	uint64 LastFrameNumber = Output.FrameCounter;
	Output = InOutput;
	Output.FrameCounter = LastFrameNumber + 1;
}

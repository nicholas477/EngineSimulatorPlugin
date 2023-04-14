// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "EngineSimulator.h"
#include "EngineSimulatorThread.generated.h"

class IEngineSimulatorInterface;
class FGameplayDebuggerCategory;

USTRUCT(BlueprintType)
struct ENGINESIMULATORPLUGIN_API FEngineSimulatorInput
{
	GENERATED_USTRUCT_BODY()

	// Currently not used
	UPROPERTY()
		float DeltaTime = 1.0f / 60.f;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Engine Simulator Input")
		float EngineRPM = 0.f;

	// When enabled, then the engine RPM is being set from the input, and torque is being read out from the engine
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Engine Simulator Input")
		bool bDynoEnabled = true;
};

USTRUCT(BlueprintType)
struct ENGINESIMULATORPLUGIN_API FEngineSimulatorOutput
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		float Torque = 0; // Newtons per meter

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		float RPM = 0;

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		float Redline = 0;

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		float Horsepower = 0;

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		FString Name;

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		int32 CurrentGear = -1;

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		int32 NumGears = 1;

	uint64 FrameCounter = 0;

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Output")
		double LastFrameTime = 0.f;
};

class ENGINESIMULATORPLUGIN_API FEngineSimulatorThread : public FRunnable
{
public:
	// Constructor, create the thread by calling this
	FEngineSimulatorThread(const FEngineSimulatorParameters& InParameters);
	virtual ~FEngineSimulatorThread() override;

	void SetEngineInput(const FEngineSimulatorInput& InInput);
	FEngineSimulatorOutput GetEngineOutput();

	void Trigger();
	void PrintGameplayDebuggerInfo(FGameplayDebuggerCategory* GameplayDebugger);

	TSharedPtr<IEngineSimulatorInterface> GetEngineSimulator() const { return EngineSimulator; }

	void EnqueueUpdate(const TFunction<void(IEngineSimulatorInterface*)>& UpdateFunction);

protected:
	// Overriden from FRunnable
	bool Init() override;
	uint32 Run() override;
	void Stop() override;
	void Exit() override;

	virtual void Tick();
	virtual void FillOutDebugData(float TransmissionTorque, float DynoSpeed, const struct FEngineSimulatorInput& ThisInput);

	FString GetThreadName();

	void UpdateEngineOutput(const FEngineSimulatorOutput& InOutput);

	TAtomic<bool> bStopRequested;

	FCriticalSection InputMutex;
	FEngineSimulatorInput Input;

	FCriticalSection OutputMutex;
	FEngineSimulatorOutput Output;

	FEvent* Semaphore;
	TSharedPtr<IEngineSimulatorInterface> EngineSimulator;
	FEngineSimulatorParameters Parameters;

	TQueue<TFunction<void(IEngineSimulatorInterface*)>, EQueueMode::Mpsc> UpdateQueue;

	FRunnableThread* Thread;

	TAtomic<double> LastSimulationTime;

#if WITH_GAMEPLAY_DEBUGGER
	TFunction<void(FGameplayDebuggerCategory*)> GameplayDebuggerPrint;
#endif
};
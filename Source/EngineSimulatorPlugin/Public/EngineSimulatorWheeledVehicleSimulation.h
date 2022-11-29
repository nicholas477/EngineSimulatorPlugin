// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "HAL/Runnable.h"
#include "EngineSimulator.h"
#include "EngineSimulatorWheeledVehicleSimulation.generated.h"

class IEngineSimulatorInterface;
class USoundWaveProcedural;

struct FEngineSimulatorInput
{
	float DeltaTime = 1.0f / 60.f;
	float EngineRPM = 0.f;
	bool InContactWithGround = true;
	uint64 FrameCounter = 0;
};

USTRUCT(BlueprintType)
struct FEngineSimulatorOutput
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		float Torque = 0; // Newtons per meter

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		float RPM = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		float Redline = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		float Horsepower = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		FString Name;

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		int32 CurrentGear = -1;

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Output")
		int32 NumGears = 1;

	uint64 FrameCounter = 0;
};

class FGameplayDebuggerCategory;

class FEngineSimulatorThread : public FRunnable
{
public:

	// Constructor, create the thread by calling this
	FEngineSimulatorThread(const FEngineSimulatorParameters& InParameters);

	// Destructor
	virtual ~FEngineSimulatorThread() override;


	// Overriden from FRunnable
	// Do not call these functions youself, that will happen automatically
	bool Init() override; // Do your setup here, allocate memory, ect.
	uint32 Run() override; // Main data processing happens here
	void Stop() override; // Clean up any memory you allocated here
	void Exit() override;

	void Trigger();
	void PrintGameplayDebuggerInfo(FGameplayDebuggerCategory* GameplayDebugger);

protected:
	TAtomic<bool> bStopRequested;

	FCriticalSection InputMutex;
	FEngineSimulatorInput Input;

	FCriticalSection OutputMutex;
	FEngineSimulatorOutput Output;

	FEvent* Semaphore;
	TUniquePtr<IEngineSimulatorInterface> EngineSimulator;
	FEngineSimulatorParameters Parameters;

	TQueue<TFunction<void(IEngineSimulatorInterface*)>, EQueueMode::Mpsc> UpdateQueue;

	FRunnableThread* Thread;

#if WITH_GAMEPLAY_DEBUGGER
	TFunction<void(FGameplayDebuggerCategory*)> GameplayDebuggerPrint;
#endif
	friend class UEngineSimulatorWheeledVehicleSimulation;
};

class UEngineSimulatorWheeledVehicleSimulation : public UChaosWheeledVehicleSimulation
{
public:
	UEngineSimulatorWheeledVehicleSimulation(TArray<class UChaosVehicleWheel*>& WheelsIn, const FEngineSimulatorParameters& InParameters);
	virtual ~UEngineSimulatorWheeledVehicleSimulation() = default;

	/** Update the engine/transmission simulation */
	virtual void ProcessMechanicalSimulation(float DeltaTime) override;

	/** Pass control Input to the vehicle systems */
	virtual void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) override;

	void AsyncUpdateSimulation(TFunction<void(IEngineSimulatorInterface*)> InCallable);

	FEngineSimulatorOutput GetLastOutput();

	// Destroys the engine and the engine thread and remakes them
	void Reset(const FEngineSimulatorParameters& InParameters);

#if WITH_GAMEPLAY_DEBUGGER
	void PrintGameplayDebuggerInfo(FGameplayDebuggerCategory* GameplayDebugger);
#endif

protected:
	TUniquePtr<FEngineSimulatorThread> EngineSimulatorThread;

	FEngineSimulatorParameters Parameters;

	FEngineSimulatorOutput LastOutput;
	mutable FCriticalSection LastOutputMutex;

	bool bStarterEnabled;
	bool bDynoEnabled;
	bool bIgnitionEnabled;
};
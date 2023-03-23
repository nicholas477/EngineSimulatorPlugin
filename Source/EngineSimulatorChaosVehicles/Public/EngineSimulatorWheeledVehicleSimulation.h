// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "EngineSimulatorThread.h"

class UEngineSimulatorWheeledVehicleSimulation : public UChaosWheeledVehicleSimulation
{
public:
	UEngineSimulatorWheeledVehicleSimulation(const FEngineSimulatorParameters& InParameters);
	virtual ~UEngineSimulatorWheeledVehicleSimulation() = default;

	virtual void Init(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicleIn) override;

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

	TSharedPtr<IEngineSimulatorInterface> GetEngineSimulator() const { return EngineSimulatorThread ? EngineSimulatorThread->GetEngineSimulator() : nullptr; }

protected:
	TUniquePtr<FEngineSimulatorThread> EngineSimulatorThread;

	FEngineSimulatorParameters Parameters;

	FEngineSimulatorOutput LastOutput;
	mutable FCriticalSection LastOutputMutex;

	bool bStarterEnabled;
	bool bDynoEnabled;
	bool bIgnitionEnabled;
};
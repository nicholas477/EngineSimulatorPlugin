// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "EngineSimulatorWheeledVehicleSimulation.h"
#include "EngineSimulatorWheeledVehicleMovementComponent.generated.h"

class USoundWave;
class USoundWaveProcedural;

UCLASS(ClassGroup = "Engine Simulator", meta = (BlueprintSpawnableComponent))
class ENGINESIMULATORPLUGIN_API UEngineSimulatorWheeledVehicleMovementComponent : public UChaosWheeledVehicleMovementComponent
{
	GENERATED_UCLASS_BODY()

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Set the user input for gear up */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|EngineSimulatorVehicleMovement")
	void SetEngineSimChangeGearUp(bool bNewGearUp);

	/** Set the user input for gear down */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|EngineSimulatorVehicleMovement")
	void SetEngineSimChangeGearDown(bool bNewGearDown);

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Vehicle Component")
		int32 CurrentGear = -1;

	// Set clutch pressure (0 - 1)
	UFUNCTION(BlueprintCallable, Category = "Game|Components|EngineSimulatorVehicleMovement")
		void SetClutchPressure(float Pressure);

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Vehicle Component")
		float ClutchPressure = 1.f;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|EngineSimulatorVehicleMovement")
		void SetStarterEnabled(bool bEnabled);

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Vehicle Component")
		bool bStarterEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine Simulator Vehicle Component")
		bool bStarterAutomaticallyEnabled = true;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|EngineSimulatorVehicleMovement")
		void RespawnEngine();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Engine Simulator Vehicle Movement")
		USoundWaveProcedural* OutputEngineSound;

	UPROPERTY(BlueprintReadOnly, Transient, Category = "Engine Simulator Vehicle Movement")
		FEngineSimulatorOutput LastEngineSimulatorOutput;

public:
	virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() override;

	TSharedPtr<IEngineSimulatorInterface> GetEngineSimulator() const;

	// Get output data from Physics Thread
	virtual void ParallelUpdate(float DeltaSeconds);

#if WITH_GAMEPLAY_DEBUGGER
	virtual void DescribeSelfToGameplayDebugger(class FGameplayDebuggerCategory* DebuggerCategory) const;
#endif // WITH_GAMEPLAY_DEBUGGER
};
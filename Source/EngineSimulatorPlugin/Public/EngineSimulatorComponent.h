// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "EngineSimulatorEngineInterface.h"
#include "EngineSimulatorThread.h"
#include "EngineSimulatorComponent.generated.h"

/**
 * Runs engine simulator. This is not a movement component, it does not turn the actor into a vehicle.
 */
UCLASS( ClassGroup=("Engine Simulator"), meta = (BlueprintSpawnableComponent))
class ENGINESIMULATORPLUGIN_API UEngineSimulatorComponent : public UActorComponent, public IEngineSimulatorEngineInterface
{
	GENERATED_BODY()

public:	
	UEngineSimulatorComponent();

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

public:	
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	virtual TSharedPtr<class IEngineSimulatorInterface> GetEngineSimulator() const override;

#if WITH_GAMEPLAY_DEBUGGER
	virtual void DescribeSelfToGameplayDebugger(class FGameplayDebuggerCategory* DebuggerCategory) const;
#endif // WITH_GAMEPLAY_DEBUGGER

	UPROPERTY(BlueprintReadOnly, VisibleAnywhere, Category = "Engine Simulator Component")
		FEngineSimulatorOutput EngineSimulatorOutput;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Engine Simulator Component")
		FEngineSimulatorInput EngineSimulatorInput;

	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Component")
		void GearUp();

	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Component")
		void GearDown(bool bNewGearDown);

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Component")
		int32 CurrentGear = -1;

	// Set clutch pressure (0 - 1)
	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Component")
		void SetClutchPressure(float Pressure);

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Component")
		float ClutchPressure = 1.f;

	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Component")
		void SetStarterEnabled(bool bEnabled);

	UPROPERTY(BlueprintReadOnly, Category = "Engine Simulator Component")
		bool bStarterEnabled = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine Simulator Component")
		bool bStarterAutomaticallyEnabled = true;

	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Component")
		void RespawnEngine();

protected:
	TUniquePtr<FEngineSimulatorThread> EngineSimulatorThread;
};

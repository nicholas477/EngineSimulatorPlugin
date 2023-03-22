// Copyright Epic Games, Inc. All Rights Reserved.

#include "GameplayDebugger/GameplayDebuggerCategory_EngineSimulatorChaosVehicles.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "EngineSimulatorWheeledVehicleMovementComponent.h"

FGameplayDebuggerCategory_EngineSimulatorChaosVehicles::FGameplayDebuggerCategory_EngineSimulatorChaosVehicles()
{
}

void FGameplayDebuggerCategory_EngineSimulatorChaosVehicles::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
    if (this == nullptr)
        return;

    if (APawn* DebugActorPawn = OwnerPC->GetPawn())
    {
        using ESWHMC = UEngineSimulatorWheeledVehicleMovementComponent;
        if (auto* MovementComponent = Cast<ESWHMC>(DebugActorPawn->GetComponentByClass(ESWHMC::StaticClass())))
        {
            MovementComponent->DescribeSelfToGameplayDebugger(this);
        }
    }
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_EngineSimulatorChaosVehicles::MakeInstance()
{
    return MakeShareable(new FGameplayDebuggerCategory_EngineSimulatorChaosVehicles());
}

#endif // WITH_GAMEPLAY_DEBUGGER

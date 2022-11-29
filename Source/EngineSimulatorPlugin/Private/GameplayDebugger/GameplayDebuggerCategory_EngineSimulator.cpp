// Copyright Epic Games, Inc. All Rights Reserved.

#include "GameplayDebugger/GameplayDebuggerCategory_EngineSimulator.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "EngineSimulatorWheeledVehicleMovementComponent.h"

FGameplayDebuggerCategory_EngineSimulator::FGameplayDebuggerCategory_EngineSimulator()
{
}

void FGameplayDebuggerCategory_EngineSimulator::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
    if (this == nullptr)
        return;

    if (APawn* DebugActorPawn = Cast<APawn>(DebugActor))
    {
        using ESWHMC = UEngineSimulatorWheeledVehicleMovementComponent;
        if (auto* MovementComponent = Cast<ESWHMC>(DebugActorPawn->GetComponentByClass(ESWHMC::StaticClass())))
        {
            MovementComponent->DescribeSelfToGameplayDebugger(this);
        }
    }
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_EngineSimulator::MakeInstance()
{
    return MakeShareable(new FGameplayDebuggerCategory_EngineSimulator());
}

#endif // WITH_GAMEPLAY_DEBUGGER

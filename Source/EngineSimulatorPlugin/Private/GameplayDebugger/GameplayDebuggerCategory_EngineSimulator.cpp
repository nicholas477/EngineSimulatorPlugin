// Copyright Epic Games, Inc. All Rights Reserved.

#include "GameplayDebugger/GameplayDebuggerCategory_EngineSimulator.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"

#include "EngineSimulatorComponent.h"

FGameplayDebuggerCategory_EngineSimulator::FGameplayDebuggerCategory_EngineSimulator()
{
}

void FGameplayDebuggerCategory_EngineSimulator::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
    if (this == nullptr)
        return;

    if (APawn* DebugActorPawn = OwnerPC->GetPawn())
    {
        if (auto* EngineSimulator = Cast<UEngineSimulatorComponent>(DebugActorPawn->GetComponentByClass(UEngineSimulatorComponent::StaticClass())))
        {
            EngineSimulator->DescribeSelfToGameplayDebugger(this);
        }
    }
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_EngineSimulator::MakeInstance()
{
    return MakeShareable(new FGameplayDebuggerCategory_EngineSimulator());
}

#endif // WITH_GAMEPLAY_DEBUGGER

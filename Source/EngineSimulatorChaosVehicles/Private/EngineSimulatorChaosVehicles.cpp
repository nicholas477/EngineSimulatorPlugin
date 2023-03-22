// Copyright Epic Games, Inc. All Rights Reserved.

#include "EngineSimulatorChaosVehicles.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebugger.h"
#include "GameplayDebugger/GameplayDebuggerCategory_EngineSimulatorChaosVehicles.h"
#endif // WITH_GAMEPLAY_DEBUGGER

#define LOCTEXT_NAMESPACE "FEngineSimulatorChaosVehiclesModule"

void FEngineSimulatorChaosVehiclesModule::StartupModule()
{
#if WITH_GAMEPLAY_DEBUGGER
	IGameplayDebugger& GameplayDebuggerModule = IGameplayDebugger::Get();
	GameplayDebuggerModule.RegisterCategory("Engine Simulator Chaos Vehicles", IGameplayDebugger::FOnGetCategory::CreateStatic(&FGameplayDebuggerCategory_EngineSimulatorChaosVehicles::MakeInstance), EGameplayDebuggerCategoryState::EnabledInGameAndSimulate, 5);
	GameplayDebuggerModule.NotifyCategoriesChanged();
#endif
}

void FEngineSimulatorChaosVehiclesModule::ShutdownModule()
{
#if WITH_GAMEPLAY_DEBUGGER
	if (IGameplayDebugger::IsAvailable())
	{
		IGameplayDebugger& GameplayDebuggerModule = IGameplayDebugger::Get();
		GameplayDebuggerModule.UnregisterCategory("Engine Simulator Chaos Vehicles");
		GameplayDebuggerModule.NotifyCategoriesChanged();
	}
#endif
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FEngineSimulatorChaosVehiclesModule, EngineSimulatorChaosVehicles)

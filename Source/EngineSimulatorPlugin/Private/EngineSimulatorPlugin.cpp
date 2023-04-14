// Copyright Epic Games, Inc. All Rights Reserved.

#include "EngineSimulatorPlugin.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"

#if WITH_GAMEPLAY_DEBUGGER
#include "GameplayDebugger.h"
#include "GameplayDebugger/GameplayDebuggerCategory_EngineSimulator.h"
#endif // WITH_GAMEPLAY_DEBUGGER

#define LOCTEXT_NAMESPACE "FEngineSimulatorPluginModule"

void FEngineSimulatorPluginModule::StartupModule()
{
#if WITH_GAMEPLAY_DEBUGGER
	IGameplayDebugger& GameplayDebuggerModule = IGameplayDebugger::Get();
	GameplayDebuggerModule.RegisterCategory("Engine Simulator", IGameplayDebugger::FOnGetCategory::CreateStatic(&FGameplayDebuggerCategory_EngineSimulator::MakeInstance), EGameplayDebuggerCategoryState::EnabledInGameAndSimulate, 5);
	GameplayDebuggerModule.NotifyCategoriesChanged();
#endif
}

void FEngineSimulatorPluginModule::ShutdownModule()
{
#if WITH_GAMEPLAY_DEBUGGER
	if (IGameplayDebugger::IsAvailable())
	{
		IGameplayDebugger& GameplayDebuggerModule = IGameplayDebugger::Get();
		GameplayDebuggerModule.UnregisterCategory("Engine Simulator");
		GameplayDebuggerModule.NotifyCategoriesChanged();
	}
#endif
}

FString FEngineSimulatorPluginModule::GetAssetDirectory()
{
#if WITH_EDITOR
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("EngineSimulatorPlugin"));

	check(Plugin.IsValid());

	return FPaths::Combine(Plugin->GetBaseDir(), TEXT("Resources"), TEXT("assets"));
#else
	return FPaths::Combine(FPaths::ProjectDir(), TEXT("assets"));
#endif
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FEngineSimulatorPluginModule, EngineSimulatorPlugin)

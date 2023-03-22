// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class EngineSimulatorChaosVehicles : ModuleRules
{
	public EngineSimulatorChaosVehicles(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
                "CoreUObject",
				"Engine",
                "Projects",
                "ChaosVehicles",
				"EngineSimulatorPlugin",
                "AudioMixer"
            }
		);

        PrivateDependencyModuleNames.AddRange(
			new string[]
			{
                "ChaosVehiclesCore",
                "ChaosVehiclesEngine"
            }
		);

        if (Target.bBuildDeveloperTools || (Target.Configuration != UnrealTargetConfiguration.Shipping && Target.Configuration != UnrealTargetConfiguration.Test))
        {
            PrivateDependencyModuleNames.Add("GameplayDebugger");
            PublicDefinitions.Add("WITH_GAMEPLAY_DEBUGGER=1");
        }
        else
        {
            PublicDefinitions.Add("WITH_GAMEPLAY_DEBUGGER=0");
        }
    }
}

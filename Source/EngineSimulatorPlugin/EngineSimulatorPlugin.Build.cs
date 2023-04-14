// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class EngineSimulatorPlugin : ModuleRules
{
	public EngineSimulatorPlugin(ReadOnlyTargetRules Target) : base(Target)
	{
        bUseUnity = false;
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
                "CoreUObject",
				"Engine",
                "Projects",
                "ChaosVehicles",
                "AudioMixer",
                "EngineSim"
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

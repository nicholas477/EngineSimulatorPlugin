// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class EngineSimulatorPluginVisualization : ModuleRules
{
	public EngineSimulatorPluginVisualization(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
                "CoreUObject",
				"Engine",
				"EngineSimulatorPlugin",
				"DeveloperSettings",
				"UMG"
            }
		);

		PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "Slate",
				"SlateCore"
            }
        );
    }
}

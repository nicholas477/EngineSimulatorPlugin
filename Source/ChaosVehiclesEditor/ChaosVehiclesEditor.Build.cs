// Copyright Epic Games, Inc. All Rights Reserved.

namespace UnrealBuildTool.Rules
{
	public class ChaosVehiclesEditor : ModuleRules
	{
        public ChaosVehiclesEditor(ReadOnlyTargetRules Target) : base(Target)
		{
			PrivateIncludePaths.Add("ChaosVehiclesEditor/Private");
            PublicIncludePaths.Add(ModuleDirectory + "/Public");

			PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
					"Slate",
					"SlateCore",
					"Engine",
					"EditorFramework",
					"UnrealEd",
					"PropertyEditor",
					"AnimGraphRuntime",
					"AnimGraph",
					"BlueprintGraph",
					"ToolMenus",
					"PhysicsCore",
					"ChaosVehiclesCore",
					"ChaosVehiclesEngine",
					"ChaosVehicles"
				}
				);

			PrivateDefinitions.Add("CHAOS_INCLUDE_LEVEL_1=1");
		}
	}
}

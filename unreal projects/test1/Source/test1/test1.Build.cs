// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class test1 : ModuleRules
{
	public test1(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput" });

		PrivateDependencyModuleNames.AddRange(new string[] { "RenderCore", "RHI", "Slate", "SlateCore", "Json" });
		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.Add("UnrealEd"); // Editor-only undo transactions.
		}

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}

// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

class FEngineSimulatorPluginModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static FString GetAssetDirectory();
private:
	/** Handle to the test dll we will load */
	//void*	ExampleLibraryHandle;
};

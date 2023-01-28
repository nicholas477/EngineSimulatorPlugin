// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

class SWindow;

class FEngineSimulatorPluginVisualizationModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	TSharedPtr<SWindow> GetWindow() const { return Window; };

protected:
	TSharedPtr<SWindow> Window;
};

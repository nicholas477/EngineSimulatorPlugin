// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorVisLibrary.h"

#include "EngineSimulatorPluginVisualization.h"
#include "Modules/ModuleManager.h"
#include "Framework/Application/SlateApplication.h"
#include "Blueprint/UserWidget.h"

#define LOCTEXT_NAMESPACE "FEngineSimulatorPluginVisualizationModule"

void UEngineSimulatorVisLibrary::SetWindowWidget(UUserWidget* Widget, bool bOnTop)
{
	auto WidgetRef = Widget->TakeWidget();
	auto Window = SNew(SWindow)
		.ClientSize(FVector2D(640, 480))
		.Title(LOCTEXT("VisualizationWindowTitle", "Engine Simulator Visualization"))
		.SizingRule(ESizingRule::Autosized)
		.SupportsMaximize(true)
		.SupportsMinimize(true)
		.HasCloseButton(true)
		.IsTopmostWindow(bOnTop)
		[WidgetRef];

	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().AddWindow(Window, true);
	}
}

#undef LOCTEXT_NAMESPACE
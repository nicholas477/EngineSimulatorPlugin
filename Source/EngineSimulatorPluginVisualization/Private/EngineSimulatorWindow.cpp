// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorWindow.h"

#include "Blueprint/UserWidget.h"

UEngineSimulatorWindow::UEngineSimulatorWindow()
{
	Size = FVector2D(640, 480);
	bSupportsMaximize = true;
	bSupportsMinimize = true;
	bHasCloseButton = true;
	bOnTop = true;
}

void UEngineSimulatorWindow::ShowWindow()
{
	if (!Widget)
	{
		return;
	}

	auto WidgetRef = Widget->TakeWidget();
	Window = SNew(SWindow)
		.ClientSize(Size)
		.Title(Title)
		.SizingRule((ESizingRule)SizingRule)
		.SupportsMaximize(bSupportsMaximize)
		.SupportsMinimize(bSupportsMinimize)
		.HasCloseButton(bHasCloseButton)
		.IsTopmostWindow(bOnTop)
		[WidgetRef];

	if (Window && FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().AddWindow(Window.ToSharedRef(), true);
	}
}

void UEngineSimulatorWindow::BeginDestroy()
{
	Super::BeginDestroy();

	if (Window)
	{
		Window->RequestDestroyWindow();
		Window = nullptr;
	}
}
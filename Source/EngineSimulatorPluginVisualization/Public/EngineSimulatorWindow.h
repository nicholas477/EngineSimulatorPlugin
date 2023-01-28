// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "EngineSimulatorWindow.generated.h"

class UUserWidget;
class SWindow;

UENUM(BlueprintType)
enum class EEngineSimulatorWindowSizingRule : uint8
{
	/* The windows size fixed and cannot be resized **/
	FixedSize,

	/** The window size is computed from its content and cannot be resized by users */
	Autosized,

	/** The window can be resized by users */
	UserSized,
};


/**
 * 
 */
UCLASS(BlueprintType, Blueprintable)
class ENGINESIMULATORPLUGINVISUALIZATION_API UEngineSimulatorWindow : public UObject
{
	GENERATED_BODY()

public:
	UEngineSimulatorWindow();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="Engine Simulator Window", meta = (ExposeOnSpawn = true))
		FVector2D Size;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		bool bSupportsMaximize;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		bool bSupportsMinimize;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		bool bHasCloseButton;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		bool bOnTop;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		UUserWidget* Widget;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		FText Title;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Engine Simulator Window", meta = (ExposeOnSpawn = true))
		EEngineSimulatorWindowSizingRule SizingRule;

	UFUNCTION(BlueprintCallable, Category = "Engine Simulator Window")
		void ShowWindow();

	virtual void BeginDestroy() override;

protected:
	TSharedPtr<SWindow> Window;
};

// Copyright Epic Games, Inc. All Rights Reserved.

#include "EngineSimulatorWheeledVehiclePawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "EngineSimulatorWheeledVehicleMovementComponent.h"
#include "DisplayDebugHelpers.h"

FName AEngineSimulatorWheeledVehiclePawn::VehicleMovementComponentName(TEXT("VehicleMovementComp"));
FName AEngineSimulatorWheeledVehiclePawn::VehicleMeshComponentName(TEXT("VehicleMesh"));

AEngineSimulatorWheeledVehiclePawn::AEngineSimulatorWheeledVehiclePawn(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(VehicleMeshComponentName);
	Mesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
	Mesh->BodyInstance.bSimulatePhysics = false;
	Mesh->BodyInstance.bNotifyRigidBodyCollision = true;
	Mesh->BodyInstance.bUseCCD = true;
	Mesh->bBlendPhysics = true;
	Mesh->SetGenerateOverlapEvents(true);
	Mesh->SetCanEverAffectNavigation(false);
	RootComponent = Mesh;

	VehicleMovementComponent = CreateDefaultSubobject<UEngineSimulatorWheeledVehicleMovementComponent>(VehicleMovementComponentName);
	VehicleMovementComponent->SetIsReplicated(true); // Enable replication by default
	VehicleMovementComponent->UpdatedComponent = Mesh;
}

void AEngineSimulatorWheeledVehiclePawn::DisplayDebug(UCanvas* Canvas, const FDebugDisplayInfo& DebugDisplay, float& YL, float& YPos)
{
	static FName NAME_Vehicle = FName(TEXT("Vehicle"));

	Super::DisplayDebug(Canvas, DebugDisplay, YL, YPos);
}

class UEngineSimulatorWheeledVehicleMovementComponent* AEngineSimulatorWheeledVehiclePawn::GetVehicleMovementComponent() const
{
	return VehicleMovementComponent;
}


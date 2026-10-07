#include "CamryAssembly.h"
#include "DoorPoseComponent.h"
#include "DoorInspectionComponent.h"
#include "Components/TextRenderComponent.h"

#if WITH_EDITOR
#include "ScopedTransaction.h"
#endif

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

ACamryAssembly::ACamryAssembly()
{
	PrimaryActorTick.bCanEverTick = false;

	VehicleRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VehicleRoot"));
	SetRootComponent(VehicleRoot);
	VehicleRoot->SetMobility(EComponentMobility::Movable);

	Body = CreateDefaultSubobject<USceneComponent>(TEXT("Body"));
	LeftDoor = CreateDefaultSubobject<USceneComponent>(TEXT("LeftDoor"));
	RightDoor = CreateDefaultSubobject<USceneComponent>(TEXT("RightDoor"));
	for (USceneComponent* Group : {Body.Get(), LeftDoor.Get(), RightDoor.Get()})
	{
		Group->SetupAttachment(VehicleRoot);
		Group->SetMobility(EComponentMobility::Movable);
	}

	// Approved preview coordinates in Unreal centimeters. These are not physical hinges.
	LeftDoor->SetRelativeLocation(FVector(51.828956604003906, -83.02676010131836, 62.81541442871094));
	RightDoor->SetRelativeLocation(FVector(51.827301025390625, 83.08061599731445, 62.81299114227295));
	LeftDoorPose = CreateDefaultSubobject<UDoorPoseComponent>(TEXT("LeftDoorPose"));
	RightDoorPose = CreateDefaultSubobject<UDoorPoseComponent>(TEXT("RightDoorPose"));
	LeftDoorPose->Configure(LeftDoor, Body);
	RightDoorPose->Configure(RightDoor, Body);
	LeftDoorInspection=CreateDefaultSubobject<UDoorInspectionComponent>(TEXT("LeftDoorInspection"));
	LeftDoorInspection->SetupAttachment(Body);
	TArray<UTextRenderComponent*> InspectionLabels;
	for (int32 i=0;i<6;++i)
	{
		UTextRenderComponent* Label=CreateDefaultSubobject<UTextRenderComponent>(FName(*FString::Printf(TEXT("InspectionLabel%d"),i+1)));
		Label->SetupAttachment(Body);
		Label->SetMobility(EComponentMobility::Movable);
		Label->SetRelativeRotation(FRotator(0,-90,0));
		Label->SetWorldSize(1.5f);
		Label->SetTextRenderColor(FColor::White);
		Label->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Label->SetVisibility(false);
		InspectionLabels.Add(Label);
	}
	LeftDoorInspection->Configure(LeftDoorPose,Body,InspectionLabels);
	const FVector ImportedMeshOrigin(250.0, 0.0, 0.0);

	struct FMeshEntry
	{
		USceneComponent* Parent;
		const TCHAR* AssetName;
		const TCHAR* Folder;
	};
	const TCHAR* BodyFolder = TEXT("/Game/DigitalTwin/Camry/Body/");
	const TCHAR* DoorFolder = TEXT("/Game/DigitalTwin/Camry/FrontDoorPreview/Parts/");
	const FMeshEntry Entries[] = {
		{Body, TEXT("Camry_BodySidesFenders"), BodyFolder},
		{Body, TEXT("Camry_FloorRearStructure"), BodyFolder},
		{Body, TEXT("Camry_FrontStructure"), BodyFolder},
		{Body, TEXT("Camry_PillarsSills"), BodyFolder},
		{Body, TEXT("Camry_Roof"), BodyFolder},
		{LeftDoor, TEXT("p2000352_395_frontdoorouter"), DoorFolder},
		{LeftDoor, TEXT("p2000358_394_frontdoorinnerfront"), DoorFolder},
		{LeftDoor, TEXT("p2000359_393_frontdoorinnerrar"), DoorFolder},
		{RightDoor, TEXT("p2000551_706_frontdoorinnerrearR"), DoorFolder},
		{RightDoor, TEXT("p2000552_707_frontdoorinnerfrontR"), DoorFolder},
		{RightDoor, TEXT("p2000553_708_frontdoorouterR"), DoorFolder},
	};

	for (const FMeshEntry& Entry : Entries)
	{
		const FString ComponentName = Entry.Parent->GetName() + TEXT("_") + Entry.AssetName;
		UStaticMeshComponent* Mesh = CreateDefaultSubobject<UStaticMeshComponent>(FName(*ComponentName));
		Mesh->SetupAttachment(Entry.Parent);
		Mesh->SetMobility(EComponentMobility::Movable);
		Mesh->SetRelativeLocation(ImportedMeshOrigin - Entry.Parent->GetRelativeLocation());
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Mesh->SetGenerateOverlapEvents(false);
		Mesh->ComponentTags.Add(Entry.Parent->GetFName());

		const FString AssetName = FString(TEXT("SM_")) + Entry.AssetName;
		const FString AssetPath = FString(Entry.Folder) + AssetName + TEXT(".") + AssetName;
		ConstructorHelpers::FObjectFinder<UStaticMesh> MeshAsset(*AssetPath);
		if (MeshAsset.Succeeded())
		{
			Mesh->SetStaticMesh(MeshAsset.Object);
		}
		MeshComponents.Add(Mesh);
	}
}

UDoorPoseComponent* ACamryAssembly::GetDoorPoseController(ECamryDoorSide Side) const
{
	return Side == ECamryDoorSide::Left ? LeftDoorPose.Get() : RightDoorPose.Get();
}

void ACamryAssembly::CaptureSelectedDoorReference()
{
#if WITH_EDITOR
	const FScopedTransaction Transaction(NSLOCTEXT("Camry", "CaptureReference", "Save door reference pose"));
#endif
	const bool bSuccess = GetDoorPoseController(SelectedDoor)->CaptureReferencePose();
	LastPoseCommandResult = bSuccess ? TEXT("Reference saved. Save the level to persist it.") : TEXT("Reference save failed. Check Output Log.");
}

void ACamryAssembly::ApplySelectedDoorOffset()
{
#if WITH_EDITOR
	const FScopedTransaction Transaction(NSLOCTEXT("Camry", "ApplyOffset", "Apply door offset"));
#endif
	const bool bSuccess = GetDoorPoseController(SelectedDoor)->ApplyReferenceOffsetMm(PreviewOffsetMm);
	LastPoseCommandResult = bSuccess ? TEXT("Offset applied from reference (mm, body axes).") : TEXT("Move failed. Save a valid reference first; check Output Log.");
}

void ACamryAssembly::RestoreSelectedDoorReference()
{
#if WITH_EDITOR
	const FScopedTransaction Transaction(NSLOCTEXT("Camry", "RestoreReference", "Restore door reference pose"));
#endif
	const bool bSuccess = GetDoorPoseController(SelectedDoor)->RestoreReferencePose();
	LastPoseCommandResult = bSuccess ? TEXT("Reference pose restored.") : TEXT("Restore failed. Save a valid reference first; check Output Log.");
}

void ACamryAssembly::RefreshDoorInspection() { if (LeftDoorInspection) LeftDoorInspection->RefreshMeasurements(); }
void ACamryAssembly::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	RefreshDoorInspection();
}

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CamryAssembly.generated.h"

class USceneComponent;
class UStaticMeshComponent;
class UDoorPoseComponent;
class UDoorInspectionComponent;

UENUM()
enum class ECamryDoorSide : uint8
{
	Left,
	Right
};

/** Native vehicle assembly. Door origins are geometry centers, not hinge axes. */
UCLASS(NotBlueprintable)
class TEST1_API ACamryAssembly : public AActor
{

	GENERATED_BODY()

public:
	ACamryAssembly();

	USceneComponent* GetLeftDoor() const { return LeftDoor.Get(); }
	USceneComponent* GetBodyFrame() const { return Body.Get(); }
	USceneComponent* GetRightDoor() const { return RightDoor.Get(); }
	UDoorPoseComponent* GetDoorPoseController(ECamryDoorSide Side) const;

	UPROPERTY(EditInstanceOnly, Category = "Camry|Door Control")
	ECamryDoorSide SelectedDoor = ECamryDoorSide::Left;

	/** Offset from the saved reference along body X/Y/Z axes, not world or door axes. */
	UPROPERTY(EditInstanceOnly, Category = "Camry|Door Control", meta = (Units = "mm"))
	FVector PreviewOffsetMm = FVector(0.0, -100.0, 0.0);

	UPROPERTY(VisibleInstanceOnly, Transient, Category = "Camry|Door Control")
	FString LastPoseCommandResult;

	UFUNCTION(CallInEditor, Category = "Camry|Door Control", meta = (DisplayName = "1. Save Current Pose as Reference"))
	void CaptureSelectedDoorReference();

	UFUNCTION(CallInEditor, Category = "Camry|Door Control", meta = (DisplayName = "2. Apply Offset (mm)"))
	void ApplySelectedDoorOffset();

	UFUNCTION(CallInEditor, Category = "Camry|Door Control", meta = (DisplayName = "3. Restore Reference Pose"))
	void RestoreSelectedDoorReference();

	UFUNCTION(CallInEditor, Category="Camry|Inspection", meta=(DisplayName="Refresh Door Inspection"))
	void RefreshDoorInspection();
	virtual void OnConstruction(const FTransform& Transform) override;

private:
	UPROPERTY(VisibleAnywhere, Category="Camry|Inspection")
	TObjectPtr<UDoorInspectionComponent> LeftDoorInspection;
	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TObjectPtr<UDoorPoseComponent> LeftDoorPose;

	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TObjectPtr<UDoorPoseComponent> RightDoorPose;

	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TObjectPtr<USceneComponent> VehicleRoot;

	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TObjectPtr<USceneComponent> Body;

	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TObjectPtr<USceneComponent> LeftDoor;

	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TObjectPtr<USceneComponent> RightDoor;

	UPROPERTY(VisibleAnywhere, Category = "Camry|Components")
	TArray<TObjectPtr<UStaticMeshComponent>> MeshComponents;
};

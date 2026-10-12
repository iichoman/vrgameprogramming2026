#pragma once

#include "CoreMinimal.h"
#include "Components/PrimitiveComponent.h"
#include "DoorInspectionComponent.generated.h"

class UDoorPoseComponent;
class UTextRenderComponent;

USTRUCT()
struct FDoorInspectionPoint
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere) FName Name;
	UPROPERTY(EditAnywhere) FVector DoorPointLocalCm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere) FVector BodyPointLocalCm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere) FVector ReferenceDoorPointBodyCm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere) FVector GapDirectionBody = FVector::ForwardVector;
	UPROPERTY(EditAnywhere) FVector FlushDirectionBody = -FVector::RightVector;
};

USTRUCT()
struct FDoorInspectionResult
{
	GENERATED_BODY()
	UPROPERTY(VisibleAnywhere) FName Name;
	UPROPERTY(VisibleAnywhere) double GapChangeMm = 0;
	UPROPERTY(VisibleAnywhere) double FlushChangeMm = 0;
	FVector DoorWorld = FVector::ZeroVector;
	FVector BodyWorld = FVector::ZeroVector;
	FVector GapWorld = FVector::ZeroVector;
	FVector FlushWorld = FVector::ZeroVector;
};

/** Fixed reference stations, independent of sensors. Signed projections, not nearest-surface gaps. */
UCLASS(NotBlueprintable)
class TEST1_API UDoorInspectionComponent : public UPrimitiveComponent
{
	GENERATED_BODY()
public:
	UDoorInspectionComponent();
	void Configure(UDoorPoseComponent* InPose, USceneComponent* InBody, const TArray<UTextRenderComponent*>& InLabels);
	UFUNCTION() bool RefreshMeasurements();
	/** Pure virtual gauge evaluation; does not change the door, baseline or displayed results. */
	bool EvaluateDoorPose(const FTransform& DoorWorld, TArray<FDoorInspectionResult>& Out) const;
	UPROPERTY(EditInstanceOnly, Category="Inspection") TArray<FDoorInspectionPoint> Points;
	UPROPERTY(EditAnywhere, Category="Inspection") bool bShowInspection = true;
	UPROPERTY(VisibleAnywhere, Category="Inspection") FString MeasurementModel = TEXT("Approximate signed changes from baseline. Virtual body anchors; not OEM gaps or sensor readings.");
	UPROPERTY(VisibleInstanceOnly, Transient, Category="Inspection") bool bMeasurementsValid = false;
	UPROPERTY(VisibleInstanceOnly, Transient, Category="Inspection") TArray<FDoorInspectionResult> Measurements;
	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
protected:
	virtual void OnRegister() override;
	virtual void OnUnregister() override;
private:
	void HandlePoseChanged(UDoorPoseComponent* Changed);
	UPROPERTY() TObjectPtr<UDoorPoseComponent> Pose;
	UPROPERTY() TObjectPtr<USceneComponent> BodyFrame;
	UPROPERTY() TArray<TObjectPtr<UTextRenderComponent>> Labels;
};

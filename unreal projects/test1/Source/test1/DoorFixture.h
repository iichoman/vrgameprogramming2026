#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DoorFixture.generated.h"

class ACamryAssembly;
class UDoorPoseComponent;
class UStaticMeshComponent;
class UTextRenderComponent;
class UArrowComponent;
class UStaticMesh;

UENUM()
enum class EDoorFixtureRole : uint8 { Locator, Support, Stopper, Clamp };

/** Geometry study, not a rated fixture design. Coordinates use the saved door frame. */
USTRUCT()
struct FDoorFixtureContact
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere) FName Name;
	UPROPERTY(EditAnywhere) EDoorFixtureRole Role = EDoorFixtureRole::Locator;
	UPROPERTY(EditAnywhere) int32 PartId = 0;
	UPROPERTY(EditAnywhere, meta=(Units="cm")) FVector DoorPointLocalCm = FVector::ZeroVector;
	/** Points away from the panel toward the fixture. Pressing is the opposite direction. */
	UPROPERTY(EditAnywhere) FVector OutwardNormalLocal = FVector::RightVector;
	/** Full lateral X/Y widths and normal Z depth. Illustrative dimensions, not approved tooling. */
	UPROPERTY(EditAnywhere, meta=(Units="mm")) FVector PreviewSizeMm = FVector(16,16,8);
	UPROPERTY(EditAnywhere, meta=(Units="mm", ClampMin="0")) double PreviewStemLengthMm = 80;
	/** Placement adjustment in fixture axes; does not move or deform the door. */
	UPROPERTY(EditAnywhere, meta=(Units="mm")) FVector OffsetMm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere) bool bRegionOnly = false;
	UPROPERTY(EditAnywhere, meta=(MultiLine="true")) FString Notes;
};

USTRUCT()
struct FDoorFixtureContactResult
{
	GENERATED_BODY()
	UPROPERTY(VisibleAnywhere) FName Name;
	UPROPERTY(VisibleAnywhere) FVector FixtureWorldCm = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere) FVector DoorWorldCm = FVector::ZeroVector;
	/** Signed displacement of the material point from the fixture marker, not contact penetration. */
	UPROPERTY(VisibleAnywhere, meta=(Units="mm")) double NormalDisplacementMm = 0;
	UPROPERTY(VisibleAnywhere, meta=(Units="mm")) double TangentialDisplacementMm = 0;
};

/** Native, non-physical fixture layout. Stays with the body while the door moves independently. */
UCLASS(NotBlueprintable)
class TEST1_API ADoorFixture : public AActor
{
	GENERATED_BODY()
public:
	ADoorFixture();
	UPROPERTY(EditInstanceOnly, Category="Fixture|Layout") TObjectPtr<ACamryAssembly> Assembly;
	UPROPERTY(EditInstanceOnly, Category="Fixture|Layout") TArray<FDoorFixtureContact> Contacts;
	UPROPERTY(EditInstanceOnly, Category="Fixture|Layout") FString LayoutSource;
	UPROPERTY(EditAnywhere, Category="Fixture|Display") bool bShowFixture = true;
	UPROPERTY(EditAnywhere, Category="Fixture|Display") bool bShowLabels = true;
	/** Study clearance only: displaces each fixture marker outward, never commands the door. */
	UPROPERTY(EditAnywhere, Category="Fixture|Display", meta=(Units="mm", ClampMin="0")) double PreviewRetractionMm = 0;
	UPROPERTY(VisibleInstanceOnly, Transient, Category="Fixture|Results") bool bLayoutValid = false;
	UPROPERTY(VisibleInstanceOnly, Transient, Category="Fixture|Results") FString LastStatus;
	UPROPERTY(VisibleInstanceOnly, Transient, Category="Fixture|Results") TArray<FDoorFixtureContactResult> Results;

	// Reflected for editor automation, not Blueprint exposure.
	UFUNCTION() bool AlignToDoorReference();
	UFUNCTION() bool RefreshFixture();
	UFUNCTION() FVector GetContactWorldPosition(FName ContactName) const;
	UFUNCTION(CallInEditor, Category="Fixture|Actions", meta=(DisplayName="1. Align Fixture to Saved Door Reference"))
	void AlignFixtureButton();
	UFUNCTION(CallInEditor, Category="Fixture|Actions", meta=(DisplayName="2. Refresh Fixture Preview"))
	void RefreshFixtureButton();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void PostRegisterAllComponents() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override { return true; }
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	static constexpr int32 MaxContacts = 12;
	void BindPose();
	void PoseChanged(UDoorPoseComponent* Pose);
	void HidePreview();
	bool Fail(const FString& Reason);
	UPROPERTY() TObjectPtr<USceneComponent> FixtureRoot;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Pads;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Stems;
	UPROPERTY() TArray<TObjectPtr<UArrowComponent>> Directions;
	UPROPERTY() TArray<TObjectPtr<UTextRenderComponent>> Labels;
	UPROPERTY() TObjectPtr<UStaticMesh> CylinderMesh;
	UPROPERTY() TObjectPtr<UStaticMesh> CubeMesh;
	TWeakObjectPtr<UDoorPoseComponent> BoundPose;
};

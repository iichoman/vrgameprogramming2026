#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VehicleProcessTest.generated.h"

class UStaticMesh;
class UStaticMeshComponent;
class SWidget;

USTRUCT()
struct FVehicleModuleMesh
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, Category="Module") TObjectPtr<UStaticMesh> Mesh;
    UPROPERTY(EditAnywhere, Category="Module") bool bPreviewVisible = true;
};

/** Process grouping of retained FE surfaces, not a validated manufacturing BOM. */
UCLASS(NotBlueprintable)
class TEST1_API AVehicleProcessModule : public AActor
{
    GENERATED_BODY()
public:
    AVehicleProcessModule();
    UPROPERTY(EditInstanceOnly, Category="Module") FName ModuleId;
    UPROPERTY(EditInstanceOnly, Category="Module") TObjectPtr<AActor> VehicleFrame;
    UPROPERTY(EditInstanceOnly, Category="Module") TArray<FVehicleModuleMesh> MeshParts;
    UPROPERTY(EditInstanceOnly, Category="Module") FVector MeshOffsetCm = FVector::ZeroVector;
    UPROPERTY(EditInstanceOnly, Category="Module") bool bWheelMotionEnabled = false;
    UPROPERTY(VisibleInstanceOnly, Category="Module") bool bHasReferencePose = false;
    UPROPERTY(VisibleInstanceOnly, Category="Module") FTransform ReferencePoseInVehicle;

    UFUNCTION() void RebuildVisuals();
    UFUNCTION() bool CaptureReferencePose();
    /** Absolute translation relative to immutable saved pose, in vehicle axes. */
    UFUNCTION() bool CommandOffsetMm(FVector OffsetMm);
    UFUNCTION() bool RestoreReference();
    UFUNCTION() bool ResetToReference();
    UFUNCTION() FVector GetCurrentOffsetMm() const;
    UFUNCTION() FVector GetTargetOffsetMm() const { return TargetOffsetCm * 10.0; }
    UFUNCTION() int32 GetVisualCount() const { return Visuals.Num(); }
    bool IsSettled() const;
    void AdvanceMotion(double Seconds);
    virtual void Tick(float DeltaSeconds) override;
protected:
    virtual void BeginPlay() override;
private:
    bool ValidFrame() const;
    bool ApplyOffsetCm(const FVector& OffsetCm);
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Visuals;
    FVector TargetOffsetCm = FVector::ZeroVector;
    FVector VelocityCm = FVector::ZeroVector;
    FVector IntegralCm = FVector::ZeroVector;
    double Remainder = 0.0;
};

/** Native runtime controls for the four wheels; no production/fastening claims. */
UCLASS(NotBlueprintable)
class TEST1_API AVehicleProcessTestDriver : public AActor
{
    GENERATED_BODY()
public:
    AVehicleProcessTestDriver();
    UPROPERTY(EditInstanceOnly, Category="Test") TArray<TObjectPtr<AVehicleProcessModule>> Wheels;
    virtual void Tick(float DeltaSeconds) override;
protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
private:
    void CreatePanel();
    void RunRuntimeChecks();
    void EnableLook();
    void DisableLook();
    TSharedPtr<SWidget> Panel;
    bool bLooking = false;
    bool bCaptureRequested = false;
    bool bCaptureDeployed = false;
    bool bCaptureTaken = false;
    bool bInputConfigured = false;
    double CaptureClock = 0.0;
};

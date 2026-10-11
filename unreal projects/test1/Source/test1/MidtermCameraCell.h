#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MidtermCameraCell.generated.h"

class ACamryAssembly;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UDoorPoseComponent;

UENUM()
enum class EMidtermCellState : uint8 { Idle, ApproachPickup, Grip, Carry, Verify, Complete, Stopped };

/** Fixed-base robot carries a door between two computed TCP poses. Grip and placement are kinematic. */
UCLASS(NotBlueprintable)
class TEST1_API AMidtermCameraCell : public AActor
{
	GENERATED_BODY()
public:
	AMidtermCameraCell();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(EditInstanceOnly, Category="Midterm|Links") TObjectPtr<ACamryAssembly> Assembly;
	UPROPERTY(EditInstanceOnly, Category="Midterm|Links") TObjectPtr<AActor> Robot;
	UPROPERTY(EditAnywhere, Category="Midterm|Demo", meta=(Units="mm", ClampMin="0")) double InjectedErrorMm = 5.0;
	UPROPERTY(EditAnywhere, Category="Midterm|Demo", meta=(Units="mm", ClampMin="0")) double StopThresholdMm = 2.0;
	UPROPERTY(EditAnywhere, Category="Midterm|Demo", meta=(Units="cm", ClampMin="10")) double PickupOutwardDistanceCm = 100.0;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") EMidtermCellState State = EMidtermCellState::Idle;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") double CameraMeasuredErrorMm = 0.0;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") double PickupReachErrorCm = 0.0;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") double MountReachErrorCm = 0.0;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") FString Status;
	UPROPERTY(VisibleAnywhere, Category="Midterm|Camera") TObjectPtr<USceneCaptureComponent2D> Camera;
	UPROPERTY(Transient, VisibleInstanceOnly, Category="Midterm|Camera") TObjectPtr<UTextureRenderTarget2D> CameraFeed;

	UFUNCTION() void StartNormal();
	UFUNCTION() void StartMisaligned();
	UFUNCTION() void ResetDemo();

private:
	void Start(bool bMisaligned);
	bool ConfigureSensor();
	bool ConfigureCarry();
	bool ObserveDoor();
	bool SolveTarget(const FVector& TargetWorld, TArray<double>& OutAngles, double& OutErrorCm);
	void SetJointAngles(const TArray<double>& Angles);
	void MoveDoorTo(const FVector& WorldLocation);
	void UpdateSequence(float DeltaSeconds);
	void ShowStatus() const;
	void HandleNormalKey();
	void HandleFaultKey();
	void HandleResetKey();
	TArray<TWeakObjectPtr<USceneComponent>> Joints;
	TArray<FRotator> RestRotations;
	TWeakObjectPtr<USceneComponent> Tool;
	TWeakObjectPtr<UDoorPoseComponent> DoorPose;
	TArray<double> PickupAngles;
	TArray<double> MountAngles;
	FTransform ReferenceDoorWorld = FTransform::Identity;
	FVector ReferenceMarkerWorld = FVector::ZeroVector;
	FVector PickupWorld = FVector::ZeroVector;
	FVector GripOffsetWorld = FVector(0, 0, 130);
	double StateSeconds = 0.0;
	bool bSensorReady = false;
	bool bCarryReady = false;
	bool bFaultRequested = false;
	bool bGripped = false;
};

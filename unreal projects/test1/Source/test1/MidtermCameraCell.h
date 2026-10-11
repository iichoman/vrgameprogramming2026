#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MidtermCameraCell.generated.h"

class ACamryAssembly;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

UENUM()
enum class EMidtermCellState : uint8 { Idle, Verify, UpperHinge, LowerHinge, Complete, Stopped };

/** Two-scenario classroom demo. Camera observes a known virtual landmark; no image recognition. */
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
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") EMidtermCellState State = EMidtermCellState::Idle;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") double CameraMeasuredErrorMm = 0.0;
	UPROPERTY(VisibleInstanceOnly, Category="Midterm|Demo") FString Status;
	UPROPERTY(VisibleAnywhere, Category="Midterm|Camera") TObjectPtr<USceneCaptureComponent2D> Camera;
	UPROPERTY(Transient, VisibleInstanceOnly, Category="Midterm|Camera") TObjectPtr<UTextureRenderTarget2D> CameraFeed;

	UFUNCTION() void StartNormal();
	UFUNCTION() void StartMisaligned();
	UFUNCTION() void ResetDemo();

private:
	void Start(bool bMisaligned);
	bool ConfigureSensor();
	bool ObserveDoor();
	void DriveRobot(float DeltaSeconds);
	void SetJointPose(double Alpha, bool bLower);
	void ShowStatus() const;
	void HandleNormalKey();
	void HandleFaultKey();
	void HandleResetKey();
	TArray<TWeakObjectPtr<USceneComponent>> Joints;
	TArray<FRotator> RestRotations;
	FVector ReferenceMarkerWorld = FVector::ZeroVector;
	double StateSeconds = 0.0;
	bool bSensorReady = false;
};

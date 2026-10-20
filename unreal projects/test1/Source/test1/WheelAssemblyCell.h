#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WheelAssemblyCell.generated.h"

class ACamryAssembly;
class ARobotArm;
class UStaticMeshComponent;

UENUM(BlueprintType)
enum class EWheelScenario : uint8 { Normal, SensorBias, DisplacedWheel, UnreachableWheel, ReversedWheel, MissingWheel, Custom };
UENUM(BlueprintType)
enum class EWheelControlMode : uint8 { Basic, IndependentVerification };
UENUM(BlueprintType)
enum class EWheelOutcome : uint8 { NotEvaluated, Pass, QualityFail, Blocked, Stopped };
UENUM(BlueprintType)
enum class EWheelStage : uint8 { Idle, ApproachPickup, Grip, Lift, Transfer, ApproachMount, Align, Fasten, Release, Retract, Reposition, Complete, Fault, Stopped };

struct FWheelLane
{
	int32 WheelIndex = INDEX_NONE;
	int32 Completed = 0;
	EWheelStage Stage = EWheelStage::Idle;
	FTransform Pickup;
	FTransform WheelInTCP;
	FTransform TargetWheel;
	FTransform TargetTCP;
	FTransform LastCarried;
	double StageTime = 0;
	double StableTime = 0;
	int32 Corrections = 0;
	bool bHeld = false;
	bool bFastened = false;
};

/** Two independently driven arms; each installs the front and rear wheel on its side. */
UCLASS(NotBlueprintable)
class TEST1_API AWheelAssemblyCell : public AActor
{
	GENERATED_BODY()
public:
	AWheelAssemblyCell();
	UPROPERTY(EditInstanceOnly, Category="Wheel Cell|References") TObjectPtr<ACamryAssembly> Assembly;
	UPROPERTY(EditInstanceOnly, Category="Wheel Cell|References") TObjectPtr<ARobotArm> LeftRobot;
	UPROPERTY(EditInstanceOnly, Category="Wheel Cell|References") TObjectPtr<ARobotArm> RightRobot;
	/** Body coordinates, cm. Order: left front, left rear, right front, right rear. */
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") double FrontXcm = 125;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") double RearXcm = -125;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") double HubHalfTrackCm = 90;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") double HubHeightCm = 38;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") double PickupHalfTrackCm = 225;
	/** Grasp transform in wheel local coordinates. Calibrate for each robot. */
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") FTransform LeftTCPInWheel = FTransform::Identity;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") FTransform RightTCPInWheel = FTransform::Identity;
	/** Calibrated joint target for moving from the front station to rear pickup. */
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") TArray<double> LeftRearApproachAnglesDegrees;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") TArray<double> RightRearApproachAnglesDegrees;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double PositionToleranceMm = 2;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double OrientationToleranceDeg = 0.5;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double QualityToleranceMm = 2;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double QualityOrientationDeg = 0.5;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double SettleSeconds = 0.25;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double FasteningSeconds = 0.5;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double StageTimeoutSeconds = 30;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") double FixedStepSeconds = 1.0 / 120.0;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") EWheelScenario Scenario = EWheelScenario::Normal;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") EWheelControlMode ControlMode = EWheelControlMode::IndependentVerification;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment", meta=(Units="mm")) FVector SensorBiasMm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment", meta=(Units="mm")) FVector InitialOffsetMm = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") FRotator InitialRotationDeg = FRotator::ZeroRotator;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") int32 FaultWheelIndex = 0;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") bool bWheelPresent = true;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") bool bAlignmentSensorValid = true;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") bool bVerificationSensorValid = true;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") double MaxPickupDisplacementMm = 500;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Experiment") double MaxPickupRotationDeg = 30;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") EWheelOutcome Outcome = EWheelOutcome::NotEvaluated;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") FString Status;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") int32 MountedCount = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") double SimulationTimeSeconds = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") TArray<double> MountPositionErrorMm;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") TArray<double> MountOrientationErrorDeg;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|State") TArray<bool> FastenedWheels;
	UFUNCTION(CallInEditor, Category="Wheel Cell|Actions") bool ResetCycle();
	UFUNCTION(CallInEditor, Category="Wheel Cell|Actions") bool StartCycle();
	UFUNCTION(CallInEditor, Category="Wheel Cell|Actions") void StopCycle();
	UFUNCTION() bool SelectScenario(EWheelScenario Preset, EWheelControlMode Mode);
	UFUNCTION() bool AdvanceSimulation(double DeltaSeconds);
	UFUNCTION() FTransform GetMountWorld(int32 Index) const;
	UFUNCTION() FTransform GetPickupWorld(int32 Index) const;
	UFUNCTION() FString GetLaneStage(bool bLeft) const;
	UFUNCTION() FTransform GetWheelWorld(int32 Index) const;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
private:
	UPROPERTY() TObjectPtr<USceneComponent> CellRoot;
	UPROPERTY() TArray<TObjectPtr<USceneComponent>> WheelRoots;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> TireVisuals;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> HubVisuals;
	FWheelLane Lanes[2];
	FTransform BodyAtStart;
	FTransform RobotAtStart[2];
	double Accumulator = 0;
	bool bRunning = false;
	bool ValidateSetup();
	void Fail(const FString& Reason);
	void SetStage(int32 Side, EWheelStage Next, const FString& Message);
	bool MoveTo(int32 Side, EWheelStage Next, const FTransform& WheelTarget);
	void StepLane(int32 Side, double Dt);
	void RefreshQuality();
	ARobotArm* RobotFor(int32 Side) const;
	FTransform TCPFor(int32 Side) const;
	FTransform OffsetOutward(const FTransform& Pose, int32 Side, double Centimeters) const;
	bool BeginWheel(int32 Side);
};

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DoorAssemblyCell.generated.h"

class ACamryAssembly;
class ARobotArm;
class UDoorPoseComponent;
class UDoorInspectionComponent;

// BlueprintType is reflection metadata required by Unreal's Python enum wrapper.
// All gameplay and UI remain native C++; no Blueprint assets or graphs are used.
UENUM(BlueprintType)
enum class EDoorCellControlMode : uint8 { Basic, IndependentVerification };
UENUM(BlueprintType)
enum class EDoorCellScenario : uint8 { Normal, SensorBias, DisplacedDoor, UnreachableDoor, ReversedDoor, MissingDoor, Custom };
UENUM(BlueprintType)
enum class EDoorCellOutcome : uint8 { NotEvaluated, Pass, QualityFail, Blocked, Stopped };

USTRUCT()
struct FDoorCellStationQuality
{
	GENERATED_BODY()
	UPROPERTY(VisibleAnywhere) FName Name;
	UPROPERTY(VisibleAnywhere) double GapChangeMm=0;
	UPROPERTY(VisibleAnywhere) double FlushChangeMm=0;
	UPROPERTY(VisibleAnywhere) bool bPassed=false;
};

UENUM()
enum class EDoorAssemblyState : uint8
{
	Idle, ApproachPickup, Grip, Lift, Transfer, ApproachMount, Align, Fasten, Release, Retract, Complete, Stopped, Fault
};

/** Door assembly process using measured simulated joint states, not a transform animation. */
UCLASS(NotBlueprintable)
class TEST1_API ADoorAssemblyCell : public AActor
{
	GENERATED_BODY()
public:
	ADoorAssemblyCell();
	UPROPERTY(EditInstanceOnly, Category="Cell|References") TObjectPtr<ACamryAssembly> Assembly;
	UPROPERTY(EditInstanceOnly, Category="Cell|References") TObjectPtr<ARobotArm> Robot;
	/** TCP relative to door, including grasp orientation. Unreal centimeters. */
	UPROPERTY(EditAnywhere, Category="Cell|Setup") FTransform TCPInDoorCm = FTransform::Identity;
	UPROPERTY(EditAnywhere, Category="Cell|Setup", meta=(Units="cm")) FVector PickupOffsetBodyCm = FVector(0,-100,20);
	UPROPERTY(EditAnywhere, Category="Cell|Setup", meta=(Units="cm")) FVector ApproachOffsetBodyCm = FVector(0,-25,0);
	UPROPERTY(EditAnywhere, Category="Cell|Setup", meta=(Units="cm")) FVector LiftOffsetBodyCm = FVector(0,0,25);
	UPROPERTY(EditAnywhere, Category="Cell|Setup", meta=(Units="cm")) FVector TransferOffsetBodyCm = FVector(0,-50,25);
	UPROPERTY(EditAnywhere, Category="Cell|Setup") bool bAutoStart = false;
	UPROPERTY(EditAnywhere, Category="Cell|Control") double PositionToleranceMm = 2;
	UPROPERTY(EditAnywhere, Category="Cell|Control") double OrientationToleranceDeg = 0.5;
	UPROPERTY(EditAnywhere, Category="Cell|Control") double SettleSeconds = 0.3;
	UPROPERTY(EditAnywhere, Category="Cell|Control") double StageTimeoutSeconds = 30;
	UPROPERTY(EditAnywhere, Category="Cell|Control") double FasteningSeconds = 1;
	UPROPERTY(EditAnywhere, Category="Cell|Control") double FixedStepSeconds = 1.0/120.0;
	UPROPERTY(EditAnywhere, Category="Cell|Recording") bool bRecordCSV = true;
	UPROPERTY(EditAnywhere, Category="Cell|Experiment") EDoorCellControlMode ControlMode=EDoorCellControlMode::IndependentVerification;
	UPROPERTY(EditAnywhere, Category="Cell|Experiment") EDoorCellScenario Scenario=EDoorCellScenario::Normal;
	/** Explicit experimental initial pose, applied by Reset; Start reads the actual placed door. */
	UPROPERTY(EditAnywhere, Category="Cell|Placement", meta=(Units="mm")) FVector InitialDoorOffsetMm=FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category="Cell|Placement") FRotator InitialDoorRotationDeg=FRotator::ZeroRotator;
	UPROPERTY(EditAnywhere, Category="Cell|Placement") bool bDoorPresent=true;
	UPROPERTY(EditAnywhere, Category="Cell|Placement") bool bPreservePlacedDoorOnBeginPlay=true;
	/** Engineering assumptions for the current gripper's recovery window, not manufacturer limits. */
	UPROPERTY(EditAnywhere, Category="Cell|Placement") double MaxPickupDisplacementMm=500;
	UPROPERTY(EditAnywhere, Category="Cell|Placement") double MaxPickupRotationDeg=30;
	UPROPERTY(EditAnywhere, Category="Cell|Sensors", meta=(Units="mm")) FVector AlignmentSensorBiasMm=FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category="Cell|Sensors") FRotator AlignmentSensorRotationBiasDeg=FRotator::ZeroRotator;
	UPROPERTY(EditAnywhere, Category="Cell|Sensors", meta=(Units="mm")) FVector VerificationSensorBiasMm=FVector::ZeroVector;
	UPROPERTY(EditAnywhere, Category="Cell|Sensors") FRotator VerificationSensorRotationBiasDeg=FRotator::ZeroRotator;
	UPROPERTY(EditAnywhere, Category="Cell|Sensors") bool bAlignmentSensorValid=true;
	UPROPERTY(EditAnywhere, Category="Cell|Sensors") bool bVerificationSensorValid=true;
	UPROPERTY(EditAnywhere, Category="Cell|Quality") double GapToleranceMm=2;
	UPROPERTY(EditAnywhere, Category="Cell|Quality") double FlushToleranceMm=2;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Quality") TArray<FDoorCellStationQuality> StationQuality;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Quality") EDoorCellOutcome Outcome=EDoorCellOutcome::NotEvaluated;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Quality") bool bQualityValid=false;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Quality") bool bQualityPassed=false;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Sensors") double AlignmentPositionErrorMm=0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Sensors") double AlignmentOrientationErrorDeg=0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Sensors") double VerificationPositionErrorMm=0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Sensors") FTransform AlignmentMeasuredDoorWorld;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Sensors") FTransform VerificationMeasuredDoorWorld;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|Experiment") FString RunMetadataFile;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") EDoorAssemblyState State = EDoorAssemblyState::Idle;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") FString Status;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") double PositionErrorMm = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") double OrientationErrorDeg = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") double SimulationTimeSeconds = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") bool bDoorHeld = false;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") bool bDoorFastened = false;
	UPROPERTY(VisibleAnywhere, Transient, Category="Cell|State") FString LogFile;
	UFUNCTION(CallInEditor, Category="Cell|Actions") bool StartCycle();
	UFUNCTION(CallInEditor, Category="Cell|Actions") void StopCycle();
	UFUNCTION(CallInEditor, Category="Cell|Actions") bool ResetCycle();
	/** Runs the same fixed-step controller in tests and runtime. Does not save a map. */
	UFUNCTION() bool AdvanceSimulation(double DeltaSeconds);
	UFUNCTION() FString GetStateLabel() const;
	UFUNCTION() FTransform GetMountDoorWorld() const;
	UFUNCTION() FTransform GetPickupDoorWorld() const;
	UFUNCTION() bool SelectScenario(EDoorCellScenario Preset, EDoorCellControlMode Mode);
	UFUNCTION() bool RefreshQuality();
	UFUNCTION() FString GetQualitySummary() const;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
private:
	TWeakObjectPtr<UDoorPoseComponent> Pose;
	TWeakObjectPtr<UDoorInspectionComponent> Inspection;
	FTransform MountInBody;
	FTransform BodyAtStart;
	FTransform RobotBaseAtStart;
	FTransform DoorInTCP;
	FTransform FastenedInBody;
	FTransform TargetTCPWorld;
	FTransform PickupAtStart;
	FTransform TargetDoorWorld;
	FTransform LastCarriedDoorWorld;
	int32 AlignmentCorrections=0;
	double Accumulator = 0;
	double StateTime = 0;
	double StableTime = 0;
	double RecordTime = 0;
	FString CSVBuffer;
	bool ValidateSetup();
	bool EnterMove(EDoorAssemblyState Next, const FTransform& DoorTarget);
	void EnterState(EDoorAssemblyState Next, const FString& Message);
	void FixedUpdate(double Dt);
	void Fail(const FString& Reason);
	FTransform OffsetDoor(const FTransform& World, const FVector& BodyOffset) const;
	void MeasureTCP();
	void RecordSample();
	void FlushRecording();
	bool MeasureSensors();
	bool AlignmentAccepted() const;
	bool VerifyBeforeFasten();
	bool CheckPickup();
	FTransform SimulatePoseSensor(const FVector& BiasMm, const FRotator& BiasDeg) const;
	void WriteRunMetadata();
};

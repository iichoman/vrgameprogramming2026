#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DoorAssemblyCell.generated.h"

class ACamryAssembly;
class ARobotArm;
class UDoorPoseComponent;

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
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
private:
	TWeakObjectPtr<UDoorPoseComponent> Pose;
	FTransform MountInBody;
	FTransform BodyAtStart;
	FTransform RobotBaseAtStart;
	FTransform DoorInTCP;
	FTransform FastenedInBody;
	FTransform TargetTCPWorld;
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
};

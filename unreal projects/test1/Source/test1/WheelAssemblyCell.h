#pragma once

#include "CoreMinimal.h"
#include "FactoryTaskModel.h"
#include "GameFramework/Actor.h"
#include "Components/ActorComponent.h"
#include "WheelAssemblyCell.generated.h"

class ACamryAssembly;
class ARobotArm;
class UStaticMeshComponent;
class USphereComponent;
class AWheelAssemblyCell;

USTRUCT(BlueprintType)
struct FWheelSensorReading
{
	GENERATED_BODY()
	UPROPERTY(VisibleAnywhere) bool bValid = false;
	UPROPERTY(VisibleAnywhere) bool bDetected = false;
	UPROPERTY(VisibleAnywhere) FTransform Pose = FTransform::Identity;
	UPROPERTY(VisibleAnywhere) double MeasuredAt = -1;
	bool Fresh(double Now, double MaxAge) const { return bValid && MeasuredAt >= 0 && Now-MeasuredAt <= MaxAge; }
};

USTRUCT(BlueprintType)
struct FWheelHubScanReading
{
	GENERATED_BODY()
	UPROPERTY(VisibleAnywhere) bool bValid = false;
	UPROPERTY(VisibleAnywhere) int32 HitCount = 0;
	UPROPERTY(VisibleAnywhere) double MeasuredAt = -1;
	UPROPERTY(VisibleAnywhere) double Confidence = 0;
	UPROPERTY(VisibleAnywhere) double ResidualMm = 0;
	UPROPERTY(VisibleAnywhere) FTransform MeasuredWorld = FTransform::Identity;
	UPROPERTY(VisibleAnywhere) FString TargetPort = TEXT("hub_fl");
	UPROPERTY(VisibleAnywhere) FString FailureReason;
	UPROPERTY(VisibleAnywhere) FVector ScanOriginWorld = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere) TArray<FVector> RayEndsWorld;
	UPROPERTY(VisibleAnywhere) TArray<FVector> HitPointsWorld;
};

/** Virtual probes read the scene and gripper plant; they never issue arm commands. */
UCLASS(ClassGroup=(DigitalTwin), meta=(BlueprintSpawnableComponent))
class TEST1_API UWheelRobotSensorBank : public UActorComponent
{
	GENERATED_BODY()
public:
	UPROPERTY(EditAnywhere) double SamplePeriod = .05;
	UPROPERTY(EditAnywhere) double MaxAge = .25;
	UPROPERTY(VisibleAnywhere) FWheelSensorReading Presence;
	UPROPERTY(VisibleAnywhere) FWheelSensorReading PickupPose;
	UPROPERTY(VisibleAnywhere) FWheelSensorReading Gripped;
	UPROPERTY(VisibleAnywhere) FWheelSensorReading MountPose;
	// Optional camera/perception adapter; owned per sensor bank, never shared mutable state.
    TSharedPtr<IFactoryPartObservationProvider> ObservationProvider;
    bool bPublishPickupPose = true;
	bool bGripActuatorFault = false;
	void Reset();
	virtual void Sample(double Now, const AWheelAssemblyCell* Cell, int32 WheelIndex);
	void CommandGrip(bool bClose) { bGripClosed = bClose; }
	bool IsGripCommandedClosed() const { return bGripClosed; }
private:
	double LastSample = -1;
	int32 LastWheelIndex = INDEX_NONE;
	bool bGripClosed = false;
};

UENUM(BlueprintType)
enum class EWheelScenario : uint8 { Normal, SensorBias, DisplacedWheel, UnreachableWheel, ReversedWheel, MissingWheel, Custom };
UENUM(BlueprintType)
enum class EWheelControlMode : uint8 { Basic, IndependentVerification };
UENUM(BlueprintType)
enum class EWheelOutcome : uint8 { NotEvaluated, Pass, QualityFail, Blocked, Stopped };
UENUM(BlueprintType)
enum class EWheelStage : uint8 { Idle, ApproachPickup, Grip, Lift, Transfer, ApproachMount, Align, Fasten, Release, Retract, Reposition, WaitSupply, WaitPose, Complete, Fault, Stopped };

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
    FVector GripVelocity = FVector::ZeroVector;
    double DemandN = 0, AppliedForceN = 0, CapacityN = 0, SlipMm = 0;
	double StageTime = 0;
	double StableTime = 0;
	int32 Corrections = 0;
	int32 GripAttempts = 0;
	double GripCommandTime = 0;
	bool bGripCommanded = false;
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
    UPROPERTY(EditAnywhere, Category="Wheel Cell|Laser") bool bLaserScanFrontLeftHub = false;
    UPROPERTY(EditAnywhere, Category="Wheel Cell|Laser") bool bBlockFrontLeftLaser = false;
    UPROPERTY(EditAnywhere, Category="Wheel Cell|Laser", meta=(Units="cm")) FVector FrontLeftHubOffsetCm = FVector::ZeroVector;
    UPROPERTY(EditAnywhere, Category="Wheel Cell|Laser", meta=(Units="mm")) double LaserOutlierMm = 0;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") FWheelHubScanReading FrontLeftHubScan;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") FWheelHubScanReading FrontLeftDockScan;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") FWheelHubScanReading FrontLeftPreMountScan;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") FWheelHubScanReading FrontLeftFinalWheelScan;
    bool bPhysicalLeftGrip = false;
    bool bLowFrictionGrip = false;
    double FrontLeftWheelMassKg = 18.0;
    double FrontLeftWheelSizeScale = 1.0;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") double FrontLeftWheelMeasuredDiameterCm = 0;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") bool bFrontLeftWheelSizePassed = false;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") FVector WheelSizeNearHitWorld = FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Laser") FVector WheelSizeFarHitWorld = FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Grip") bool bLeftWheelSlipFall = false;
    UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Grip") double LeftWheelSlipFallSeconds = 0;
    FVector LeftWheelSlipVelocityCmps = FVector::ZeroVector;
    void AdvanceLeftWheelSlipFall(double DeltaSeconds);
    const FWheelLane& GetLeftGripState() const { return Lanes[0]; }
    // Runtime adapter bindings, ordered LF/LR/RF/RR. Empty preserves standalone demos.
    UPROPERTY(Transient) TArray<TObjectPtr<ARobotArm>> TaskRobots;
    UPROPERTY(Transient) TArray<TObjectPtr<UWheelRobotSensorBank>> TaskSensors;
    TArray<double> RearTransit[4];
    TArray<int32> TaskPartIndices;
    TMap<int32,FVector> SupplyOffsetsCm;
    int32 PartIndex(int32 Target) const { return TaskPartIndices.IsValidIndex(Target)?TaskPartIndices[Target]:Target; }
    FTransform GetPhysicalPartWorld(int32 Index) const;
    bool SetPhysicalPartWorld(int32 Index,const FTransform& Pose);
    ARobotArm* RobotForWheel(int32 Index) const;
    bool UsesTaskBindings() const { return TaskRobots.Num()==4; }
    bool PrepareTaskLanes();
    int32 LaneCount() const { return UsesTaskBindings()?4:2; }
    bool LaneEnabled(int32 Lane) const { return !UsesTaskBindings()||!TaskQueue[Lane].IsEmpty(); }
    UWheelRobotSensorBank* SensorsFor(int32 Lane) const;
    bool SensorDrivenLane(int32 Lane) const;
    int32 GeometricSide(int32 Lane) const { return UsesTaskBindings()?(Lanes[Lane].WheelIndex>=0?Lanes[Lane].WheelIndex/2:(!TaskQueue[Lane].IsEmpty()?TaskQueue[Lane][0]/2:Lane%2)):Lane; }
    TArray<int32> TaskQueue[4];
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Control") bool bExternallyDriven = false;
	/** Enable measured decisions on the left robot in the sensor assembly map. */
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Sensors") bool bSensorDrivenLeftRobot = false;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Sensors") bool bAutoRecoverLeftRobot = false;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Sensors", meta=(Units="s")) double LeftRecoveryTimeoutSeconds = 5;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Sensors", meta=(Units="s")) double LeftRecoveryStableSeconds = .15;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Sensors") int32 MaxAutoGripAttempts = 8;
	UPROPERTY(VisibleAnywhere, Category="Wheel Cell|Sensors") TObjectPtr<UWheelRobotSensorBank> LeftSensors;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Sensors") int32 LeftPickupReplans = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Sensors") int32 LeftGripRetries = 0;
	/** Optional measured module centers, LF/LR/RF/RR, in body centimeters. */
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Geometry") TArray<FVector> MountCentersCm;
	UFUNCTION() void UpdateFastenedWheels();
	/** CAD carriage is moved with the vehicle; rails and robots remain fixed. */
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Transport") bool bEnableTransport = false;
	UPROPERTY(EditInstanceOnly, Category="Wheel Cell|Transport") TObjectPtr<AActor> TransportCarriage;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Transport", meta=(Units="cm")) FVector TransportStartOffsetCm = FVector(-600, 0, 0);
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Transport", meta=(Units="s")) double TransportSeconds = 8;
	UPROPERTY(EditAnywhere, Category="Wheel Cell|Transport", meta=(Units="s")) double DockSettleSeconds = 1;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Transport") bool bTransporting = false;
	UPROPERTY(VisibleAnywhere, Transient, Category="Wheel Cell|Transport") double TransportProgress = 0;
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
	FTransform GetMountTargetWorld(int32 Index) const;
	bool ScanFrontLeftHub(bool bScanWheel=false);
	UFUNCTION() FTransform GetPickupWorld(int32 Index) const;
	UFUNCTION() FString GetLaneStage(bool bLeft) const;
	UFUNCTION() FTransform GetWheelWorld(int32 Index) const;
	EWheelStage GetLeftStage() const { return Lanes[0].Stage; }
	int32 GetActiveLeftWheelIndex() const { return Lanes[0].WheelIndex; }
	UFUNCTION() bool IsWheelVisible(int32 Index) const;
	UFUNCTION() FString GetLeftSensorSummary() const;
	FString GetLaserInspectionSummary() const;
	UFUNCTION() void InjectLeftWheelMissing();
	UFUNCTION() void InjectLeftWheelDisplacement(FVector OffsetCm);
	void SetLeftWheelSupplyVisible(int32 Index, bool bVisible);
	void DisplaceLeftWheel(int32 Index, FVector OffsetCm);
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
private:
	UPROPERTY() TObjectPtr<USceneComponent> CellRoot;
	UPROPERTY() TObjectPtr<USphereComponent> FrontLeftHubProxy;
	UPROPERTY() TArray<TObjectPtr<USceneComponent>> WheelRoots;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> TireVisuals;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> HubVisuals;
	FWheelLane Lanes[4];
	FTransform FastenedInBody[4];
	FTransform BodyAtStart;
	FTransform RobotAtStart[4];
	double Accumulator = 0;
	bool bRunning = false;
	bool bStationCaptured = false;
	FTransform VehicleAtStation;
	FTransform CarriageAtStation;
	FTransform ExpectedVehicle;
	FTransform ExpectedCarriage;
	double TransportTime = 0;
	void SetTransportProgress(double Progress);
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
	bool ScanFrontLeftWheelSize();
	void WaitForLeftSensor(int32 Side, EWheelStage WaitStage, const FString& Message);
};

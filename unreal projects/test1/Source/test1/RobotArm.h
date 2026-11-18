#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RobotControlMath.h"
#include "RobotTorquePlant.h"
#include "RobotMotionPlanner.h"
#include "RobotMotionTrajectory.h"
#include "WheelContactSettings.h"
#include "RobotArm.generated.h"

class UStaticMesh;
class UStaticMeshComponent;

USTRUCT()
struct FRobotJointSetup
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere) FName Name;
	UPROPERTY(EditAnywhere) FTransform HomeLocalCm = FTransform::Identity;
	UPROPERTY(EditAnywhere) FVector AxisLocal = FVector::UpVector;
	UPROPERTY(EditAnywhere) double MinimumDegrees = -170;
	UPROPERTY(EditAnywhere) double MaximumDegrees = 170;
	UPROPERTY(EditAnywhere) double MaxVelocityDegrees = 45;
	UPROPERTY(EditAnywhere) double MaxAccelerationDegrees = 90;
};

USTRUCT()
struct FRobotMeshBinding
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere) TObjectPtr<UStaticMesh> Mesh;
	/** -1 = base; otherwise A1..A6 index 0..5. */
	UPROPERTY(EditAnywhere) int32 JointIndex = -1;
	UPROPERTY(EditAnywhere) FTransform RelativePoseCm = FTransform::Identity;
};

/** Meshes copied from the source robot; motion comes exclusively from the servo state. */
UCLASS(NotBlueprintable)
class TEST1_API ARobotArm : public AActor
{
	GENERATED_BODY()
public:
	ARobotArm();
	UPROPERTY(EditAnywhere, Category="Robot|Calibration") TArray<FRobotJointSetup> Joints;
	UPROPERTY(EditAnywhere, Category="Robot|Calibration") TArray<FRobotMeshBinding> MeshBindings;
	UPROPERTY(EditAnywhere, Category="Robot|Calibration") FTransform ToolInLastJointCm = FTransform::Identity;
	UPROPERTY(EditAnywhere, Category="Robot|Calibration") TArray<double> InitialAnglesDegrees;
	UPROPERTY(EditAnywhere, Category="Robot|Control") double Kp = 5;
	UPROPERTY(EditAnywhere, Category="Robot|Control") double Ki = 0.15;
	UPROPERTY(EditAnywhere, Category="Robot|Control") double Kd = 0.15;
	UPROPERTY(EditAnywhere, Category="Robot|Control") double MotorResponseSeconds = 0.06;
	UPROPERTY(VisibleAnywhere, Transient, Category="Robot|State") TArray<double> CurrentAnglesDegrees;
	UPROPERTY(VisibleAnywhere, Transient, Category="Robot|State") TArray<double> TargetAnglesDegrees;
	UPROPERTY(VisibleAnywhere, Transient, Category="Robot|State") FString LastStatus;
	UPROPERTY(EditAnywhere, Category="Robot|Calibration", meta=(MultiLine="true")) FString CalibrationSource;
	UFUNCTION() bool InitializeArm();
	UFUNCTION() FTransform GetTCPWorld() const;
	UFUNCTION() bool CommandTCPWorld(const FTransform& TargetWorld);
    /** Accepts a queued fixed-orientation straight move; no tilt/detour fallback. */
    bool CommandLinearTCPWorld(const FTransform& TargetWorld);
    bool IsMotionPlanning() const { return PendingMotion.IsValid()||(SegmentDuration>0&&!bMotionBlocked&&!MotionScene.IsGeometryReady()); }
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	/** Geometry context supplied by the task; touching a part is not a quality verdict. */
	void SetMotionPart(USceneComponent* Part,bool Held);
	TArray<double> PreferredIKSeedDegrees;
	UPROPERTY(VisibleAnywhere, Transient, Category="Robot|Motion") double PlannedMotionSeconds=0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Robot|Motion") int32 PlannedWaypoints=0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Robot|Motion") bool bMotionBlocked=false;
	/** Collision-checked joint-space move. Angles are degrees, tracked by the existing PID. */
	UFUNCTION() bool CommandJointDegrees(const TArray<double>& AnglesDegrees);
	UFUNCTION() FTransform GetTargetTCPWorld() const;
	UFUNCTION() bool StepControl(double DeltaSeconds);
	UFUNCTION() void Halt();
	UFUNCTION() bool IsAtJointTarget(double ToleranceDegrees = 0.1) const;
	UFUNCTION() bool SetInitialAngles(const TArray<double>& AnglesDegrees);
	/** Kinematic link bounds used by the dropped-wheel collision sweep. */
	void SetDroppedWheelCollision(bool bEnabled);
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	const TArray<FRobotJointDefinition>& GetDefinitions() const { return Definitions; }
	const TArray<double>& GetAnglesRadians() const { return Angles; }
    bool bTorqueDynamics=false;
    UPROPERTY(EditAnywhere, Category="Robot|Tool") FWheelJawSettings WheelJaw;
    double TorqueLimitScale=1.0;
    // Assumed link parameters in each joint frame; replace with OEM CAD/inertia data.
    UPROPERTY(EditAnywhere, Category="Robot|Dynamics") TArray<double> LinkMassKg={80,65,45,15,10,5};
    UPROPERTY(EditAnywhere, Category="Robot|Dynamics") TArray<FVector> LinkCOMCm={FVector(0,0,20),FVector(30,0,0),FVector(30,0,0),FVector(0,0,10),FVector(0,0,8),FVector(0,0,5)};
    UPROPERTY(EditAnywhere, Category="Robot|Dynamics") TArray<double> JointInertiaKgM2={40,40,40,4,4,4};
    UPROPERTY(EditAnywhere, Category="Robot|Dynamics") TArray<double> JointDriveLimitNm={4000,4000,4000,400,400,400};
    FVector ToolReactionForceN=FVector::ZeroVector, ToolReactionTorqueNm=FVector::ZeroVector, ForcePointWorldCm=FVector::ZeroVector;
    TArray<double> AppliedJointTorqueNm;
    int32 LastIKIterations = 0;
private:
	UPROPERTY() TObjectPtr<USceneComponent> ArmRoot;
	UPROPERTY() TArray<TObjectPtr<USceneComponent>> JointNodes;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Visuals;
	UPROPERTY() TArray<TObjectPtr<class UBoxComponent>> DropCollisionBoxes;
	TArray<FRobotJointDefinition> Definitions;
	TArray<FRobotJointState> Controllers;
	TArray<double> Angles;
	TArray<double> Targets;
	TArray<double> SegmentStart;
	double SegmentElapsed = 0;
	double SegmentDuration = 0;
	bool bInitialized = false;
	FRobotMotionScene MotionScene;
	TWeakObjectPtr<USceneComponent> MotionPart;
	bool bMotionHolding=false;
	TArray<TArray<double>> MotionPath;
	TArray<double> MotionDurations;
	int32 MotionIndex=0;
	double SceneAge=0;
	TSharedPtr<FRobotMotionRequest> PendingMotion;
	FRobotMotionTrajectory ActiveTrajectory;
	bool QueueTCPMotion(const FTransform& World,bool bLinear);
	bool AdvanceMotionPlanning();
	bool PlanJointMove(const TArray<double>& Goal);
	bool SegmentSettled(double ToleranceDegrees) const;
	void UpdateVisuals();
};

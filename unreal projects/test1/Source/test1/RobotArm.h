#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RobotControlMath.h"
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
	/** Smooth joint-space transit between calibrated station poses. Angles are degrees. */
	UFUNCTION() bool CommandJointDegrees(const TArray<double>& AnglesDegrees);
	UFUNCTION() FTransform GetTargetTCPWorld() const;
	UFUNCTION() bool StepControl(double DeltaSeconds);
	UFUNCTION() void Halt();
	UFUNCTION() bool IsAtJointTarget(double ToleranceDegrees = 0.1) const;
	UFUNCTION() bool SetInitialAngles(const TArray<double>& AnglesDegrees);
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	const TArray<FRobotJointDefinition>& GetDefinitions() const { return Definitions; }
	const TArray<double>& GetAnglesRadians() const { return Angles; }
	int32 LastIKIterations = 0;
private:
	UPROPERTY() TObjectPtr<USceneComponent> ArmRoot;
	UPROPERTY() TArray<TObjectPtr<USceneComponent>> JointNodes;
	UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Visuals;
	TArray<FRobotJointDefinition> Definitions;
	TArray<FRobotJointState> Controllers;
	TArray<double> Angles;
	TArray<double> Targets;
	TArray<double> SegmentStart;
	double SegmentElapsed = 0;
	double SegmentDuration = 0;
	bool bInitialized = false;
	void UpdateVisuals();
};

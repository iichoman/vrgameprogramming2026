#pragma once

#include "CoreMinimal.h"

/** A revolute joint. HomeLocal describes its pivot at q=0 relative to its parent.
 * All kinematic lengths are Unreal centimeters; angles are radians. AxisLocal is
 * expressed in the joint's home frame, and transforms must have unit scale.
 */
struct TEST1_API FRobotJointDefinition
{
	FTransform HomeLocal = FTransform::Identity;
	FVector AxisLocal = FVector::UpVector;
	double MinRadians = -PI;
	double MaxRadians = PI;
	double MaxVelocityRadians = 1.0;
	double MaxAccelerationRadians = 2.0;
};

enum class ERobotIKStatus : uint8
{
	Success,
	InvalidInput,
	JointLimit,
	NoConvergence
};

struct TEST1_API FRobotIKSettings
{
	double PositionToleranceCm = 0.2;
	double OrientationToleranceRadians = 0.008726646259971648; // 0.5 degrees.
	double OrientationWeightCm = 30.0;
	double DampingCm = 2.0;
	double MaxStepRadians = 0.15;
	int32 MaxIterations = 160;
	int32 MaxRestarts = 3;
};

struct TEST1_API FRobotIKSolution
{
	ERobotIKStatus Status = ERobotIKStatus::InvalidInput;
	TArray<double> JointRadians;
	int32 Iterations = 0;
	double PositionErrorCm = TNumericLimits<double>::Max();
	double OrientationErrorRadians = TNumericLimits<double>::Max();

	bool IsSuccess() const { return Status == ERobotIKStatus::Success; }
};

/** Full rigid-pose kinematics. The target and result are relative to the robot base.
 * This solves geometry and joint limits; collision and process interlocks belong
 * to the cell controller. A failed solution must never authorize attachment.
 */
struct TEST1_API FRobotKinematics
{
	static bool ForwardKinematics(
		const TArray<FRobotJointDefinition>& Joints,
		const TArray<double>& JointRadians,
		const FTransform& ToolLocal,
		FTransform& OutTCP,
		TArray<FTransform>* OutJointTransforms = nullptr);

	static FRobotIKSolution SolveInverseKinematics(
		const TArray<FRobotJointDefinition>& Joints,
		const FTransform& ToolLocal,
		const FTransform& TargetBaseLocal,
		const TArray<double>& SeedRadians,
		const FRobotIKSettings& Settings = FRobotIKSettings());

	/** Shortest target/current quaternion difference, represented as a base-frame
	 * rotation vector. Its length is the angular error in radians.
	 */
	static FVector RotationError(const FQuat& Target, const FQuat& Current);
	static const TCHAR* StatusText(ERobotIKStatus Status);
};

/** Velocity-command PID, followed by a first-order, acceleration-limited servo.
 * This deliberately simplified plant does not claim torque/load simulation.
 * Gains: Kp [1/s], Ki [1/s^2], Kd dimensionless; integral [rad*s].
 */
struct TEST1_API FRobotJointControllerSettings
{
	double Kp = 5.0;
	double Ki = 0.15;
	double Kd = 0.15;
	double VelocityResponseSeconds = 0.06;
	double IntegralLimitRadiansSeconds = 0.4;
};

struct TEST1_API FRobotJointState
{
	double AngleRadians = 0.0;
	double VelocityRadians = 0.0;
	double IntegralErrorSeconds = 0.0;
	double DesiredVelocityRadians = 0.0;
};

struct TEST1_API FRobotJointController
{
	/** No target snapping. Returns false without changing state for invalid input.
	 * Call at a fixed simulation interval, e.g. 1/120 s. Derivative acts on measured
	 * velocity to avoid a target-step kick; saturation uses conditional integration.
	 */
	static bool Step(
		const FRobotJointDefinition& Joint,
		const FRobotJointControllerSettings& Settings,
		double TargetRadians,
		double FeedforwardRadiansPerSecond,
		double DeltaSeconds,
		FRobotJointState& InOutState);
};

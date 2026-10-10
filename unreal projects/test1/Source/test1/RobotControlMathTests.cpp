#if WITH_DEV_AUTOMATION_TESTS

#include "RobotControlMath.h"
#include "Misc/AutomationTest.h"

namespace RobotControlMathTests
{
	TArray<FRobotJointDefinition> MakeSixAxisChain()
	{
		TArray<FRobotJointDefinition> Joints;
		Joints.SetNum(6);
		Joints[0].HomeLocal.SetTranslation(FVector(0.0, 0.0, 40.0));
		Joints[0].AxisLocal = FVector::UpVector;
		Joints[1].AxisLocal = FVector::YAxisVector;
		Joints[2].HomeLocal.SetTranslation(FVector(70.0, 0.0, 0.0));
		Joints[2].AxisLocal = FVector::YAxisVector;
		Joints[3].HomeLocal.SetTranslation(FVector(55.0, 0.0, 0.0));
		Joints[3].AxisLocal = FVector::XAxisVector;
		Joints[4].AxisLocal = FVector::YAxisVector;
		Joints[5].HomeLocal.SetTranslation(FVector(20.0, 0.0, 0.0));
		Joints[5].AxisLocal = FVector::XAxisVector;
		return Joints;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotForwardKinematicsTest,
	"test1.RobotControl.ForwardKinematics", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRobotForwardKinematicsTest::RunTest(const FString& Parameters)
{
	TArray<FRobotJointDefinition> Joints;
	Joints.SetNum(2);
	Joints[0].HomeLocal.SetTranslation(FVector(10.0, 0.0, 0.0));
	Joints[1].HomeLocal.SetTranslation(FVector(50.0, 0.0, 0.0));
	const FTransform Tool(FQuat::Identity, FVector(25.0, 0.0, 0.0));
	FTransform TCP;
	TArray<FTransform> JointTransforms;
	TestTrue(TEXT("FK accepts rigid centimeter inputs"), FRobotKinematics::ForwardKinematics(
		Joints, TArray<double>({UE_DOUBLE_PI * 0.5, 0.0}), Tool, TCP, &JointTransforms));
	TestTrue(TEXT("Parent rotation moves 50 cm link and 25 cm tool"),
		TCP.GetTranslation().Equals(FVector(10.0, 75.0, 0.0), 0.000001));
	TestEqual(TEXT("One base-local result per joint"), JointTransforms.Num(), 2);
	TestTrue(TEXT("TCP rotation agrees with serial chain"),
		FRobotKinematics::RotationError(FQuat(FVector::UpVector, UE_DOUBLE_PI * 0.5), TCP.GetRotation()).Length() < 0.000001);

	// Nonidentity home orientation defines the local actuation axis. It must not
	// accidentally rotate around an untransformed parent-frame axis.
	Joints.SetNum(1);
	Joints[0].HomeLocal = FTransform(FQuat(FVector::YAxisVector, UE_DOUBLE_PI * 0.5));
	TestTrue(TEXT("Rotated home frame is valid"), FRobotKinematics::ForwardKinematics(
		Joints, TArray<double>({UE_DOUBLE_PI * 0.5}), FTransform(FQuat::Identity, FVector(10.0, 0.0, 0.0)), TCP));
	const FVector Expected = (Joints[0].HomeLocal.GetRotation() * FQuat(FVector::UpVector, UE_DOUBLE_PI * 0.5))
		.RotateVector(FVector(10.0, 0.0, 0.0));
	TestTrue(TEXT("Actuation is in the joint home frame"), TCP.GetTranslation().Equals(Expected, 0.000001));
	Joints[0].HomeLocal.SetScale3D(FVector(2.0));
	TestFalse(TEXT("Scaled geometry must be baked before kinematics"), FRobotKinematics::ForwardKinematics(
		Joints, TArray<double>({0.0}), Tool, TCP));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotInverseKinematicsTest,
	"test1.RobotControl.InverseKinematics", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRobotInverseKinematicsTest::RunTest(const FString& Parameters)
{
	const TArray<FRobotJointDefinition> Joints = RobotControlMathTests::MakeSixAxisChain();
	const FTransform Tool(FQuat::Identity, FVector(15.0, 0.0, 0.0));
	FTransform Target;
	const TArray<double> KnownAngles({0.35, -0.55, 0.85, 0.25, -0.40, 0.20});
	TestTrue(TEXT("Construct reachable six-dimensional target"),
		FRobotKinematics::ForwardKinematics(Joints, KnownAngles, Tool, Target));
	FRobotIKSettings Settings;
	Settings.PositionToleranceCm = 0.02;
	Settings.OrientationToleranceRadians = 0.001;
	const TArray<double> Seed({0.10, -0.20, 0.25, 0.05, -0.15, 0.05});
	const FRobotIKSolution Solution = FRobotKinematics::SolveInverseKinematics(Joints, Tool, Target, Seed, Settings);
	TestTrue(TEXT("IK converges on position AND orientation"), Solution.IsSuccess());
	TestTrue(TEXT("Position error within configured centimeter tolerance"), Solution.PositionErrorCm <= Settings.PositionToleranceCm);
	TestTrue(TEXT("Orientation error within configured radian tolerance"), Solution.OrientationErrorRadians <= Settings.OrientationToleranceRadians);
	FTransform Reconstructed;
	if (FRobotKinematics::ForwardKinematics(Joints, Solution.JointRadians, Tool, Reconstructed))
	{
		TestTrue(TEXT("IK solution independently reconstructs requested position"),
			FVector::Distance(Reconstructed.GetTranslation(), Target.GetTranslation()) <= Settings.PositionToleranceCm);
		TestTrue(TEXT("IK solution independently reconstructs requested rotation"),
			FRobotKinematics::RotationError(Target.GetRotation(), Reconstructed.GetRotation()).Length() <= Settings.OrientationToleranceRadians);
	}
	else
	{
		AddError(TEXT("Returned solution must be valid FK input"));
	}
	for (int32 Joint = 0; Joint < Solution.JointRadians.Num(); ++Joint)
	{
		TestTrue(TEXT("IK respects mechanical joint interval"), Solution.JointRadians[Joint] >= Joints[Joint].MinRadians
			&& Solution.JointRadians[Joint] <= Joints[Joint].MaxRadians);
	}
	const FTransform FarTarget(FQuat::Identity, FVector(10000.0, 0.0, 0.0));
	const FRobotIKSolution Unreachable = FRobotKinematics::SolveInverseKinematics(Joints, Tool, FarTarget, Seed, Settings);
	TestFalse(TEXT("Out-of-reach pose cannot authorize an assembly"), Unreachable.IsSuccess());
	TestTrue(TEXT("Unreachable pose reports finite residual"), FMath::IsFinite(Unreachable.PositionErrorCm)
		&& Unreachable.PositionErrorCm > 9000.0);
	const FRobotIKSolution WrongCount = FRobotKinematics::SolveInverseKinematics(
		Joints, Tool, Target, TArray<double>({0.0}), Settings);
	TestTrue(TEXT("Malformed seed is rejected"), WrongCount.Status == ERobotIKStatus::InvalidInput);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJointServoTest,
	"test1.RobotControl.JointServo", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRobotJointServoTest::RunTest(const FString& Parameters)
{
	FRobotJointDefinition Joint;
	Joint.MinRadians = -1.5;
	Joint.MaxRadians = 1.5;
	Joint.MaxVelocityRadians = 0.8;
	Joint.MaxAccelerationRadians = 1.6;
	FRobotJointControllerSettings Settings;
	FRobotJointState State;
	constexpr double Dt = 1.0 / 120.0;
	constexpr double Target = 1.0;
	bool bSpeedBounded = true;
	bool bAccelerationBounded = true;
	bool bNoAngleSnap = true;
	bool bAllStepsValid = true;
	for (int32 Sample = 0; Sample < 1200; ++Sample)
	{
		const double PreviousAngle = State.AngleRadians;
		const double PreviousVelocity = State.VelocityRadians;
		bAllStepsValid &= FRobotJointController::Step(Joint, Settings, Target, 0.0, Dt, State);
		bSpeedBounded &= FMath::Abs(State.VelocityRadians) <= Joint.MaxVelocityRadians + 1e-10;
		bAccelerationBounded &= FMath::Abs(State.VelocityRadians - PreviousVelocity) <= Joint.MaxAccelerationRadians * Dt + 1e-10;
		bNoAngleSnap &= FMath::Abs(State.AngleRadians - PreviousAngle) <= Joint.MaxVelocityRadians * Dt + 1e-10;
	}
	TestTrue(TEXT("Fixed-step servo accepts valid inputs"), bAllStepsValid);
	TestTrue(TEXT("Velocity saturates at physical configured limit"), bSpeedBounded);
	TestTrue(TEXT("First-order plant respects acceleration limit"), bAccelerationBounded);
	TestTrue(TEXT("Motion integrates speed, never applies target angles directly"), bNoAngleSnap);
	TestTrue(TEXT("Controller settles to commanded angle"), FMath::Abs(Target - State.AngleRadians) < 0.01);
	TestTrue(TEXT("Settled velocity is small"), FMath::Abs(State.VelocityRadians) < 0.002);
	TestTrue(TEXT("Integral remains bounded"), FMath::Abs(State.IntegralErrorSeconds) <= Settings.IntegralLimitRadiansSeconds);
	const double PreviousAngle = State.AngleRadians;
	TestFalse(TEXT("Out-of-range angle command is rejected"), FRobotJointController::Step(Joint, Settings, 2.0, 0.0, Dt, State));
	TestEqual(TEXT("Rejected input leaves angle unchanged"), State.AngleRadians, PreviousAngle);
	TestFalse(TEXT("Zero dt is rejected"), FRobotJointController::Step(Joint, Settings, 0.0, 0.0, 0.0, State));
	return true;
}

#endif

#include "RobotControlMath.h"

namespace RobotControlMath
{
	constexpr int32 PoseDimension = 6;
	constexpr double DifferenceStep = 0.0001;

	bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	bool IsRigidTransform(const FTransform& Value)
	{
		const FQuat& Rotation = Value.GetRotation();
		return IsFiniteVector(Value.GetTranslation()) && IsFiniteVector(Value.GetScale3D())
			&& Value.GetScale3D().Equals(FVector::OneVector, 0.00001)
			&& FMath::IsFinite(Rotation.X) && FMath::IsFinite(Rotation.Y)
			&& FMath::IsFinite(Rotation.Z) && FMath::IsFinite(Rotation.W)
			&& FMath::Abs(Rotation.SizeSquared() - 1.0) < 0.0001;
	}

	bool IsJointValid(const FRobotJointDefinition& Joint)
	{
		return IsRigidTransform(Joint.HomeLocal) && IsFiniteVector(Joint.AxisLocal)
			&& FMath::Abs(Joint.AxisLocal.SizeSquared() - 1.0) < 0.0001
			&& FMath::IsFinite(Joint.MinRadians) && FMath::IsFinite(Joint.MaxRadians)
			&& Joint.MinRadians <= Joint.MaxRadians
			&& FMath::IsFinite(Joint.MaxVelocityRadians) && Joint.MaxVelocityRadians > 0.0
			&& FMath::IsFinite(Joint.MaxAccelerationRadians) && Joint.MaxAccelerationRadians > 0.0;
	}

	bool IsChainValid(const TArray<FRobotJointDefinition>& Joints,
		const TArray<double>& Angles, const FTransform& Tool)
	{
		if (Joints.IsEmpty() || Joints.Num() != Angles.Num() || !IsRigidTransform(Tool))
		{
			return false;
		}
		for (int32 Index = 0; Index < Joints.Num(); ++Index)
		{
			if (!IsJointValid(Joints[Index]) || !FMath::IsFinite(Angles[Index]))
			{
				return false;
			}
		}
		return true;
	}

	void PoseResidual(const FTransform& Target, const FTransform& Current,
		double OrientationWeight, double OutResidual[PoseDimension],
		double& OutPositionError, double& OutOrientationError)
	{
		const FVector Position = Target.GetTranslation() - Current.GetTranslation();
		const FVector Orientation = FRobotKinematics::RotationError(Target.GetRotation(), Current.GetRotation());
		OutPositionError = Position.Length();
		OutOrientationError = Orientation.Length();
		OutResidual[0] = Position.X;
		OutResidual[1] = Position.Y;
		OutResidual[2] = Position.Z;
		OutResidual[3] = Orientation.X * OrientationWeight;
		OutResidual[4] = Orientation.Y * OrientationWeight;
		OutResidual[5] = Orientation.Z * OrientationWeight;
	}

	double ResidualCost(const double Residual[PoseDimension])
	{
		double Cost = 0.0;
		for (int32 Row = 0; Row < PoseDimension; ++Row)
		{
			Cost += Residual[Row] * Residual[Row];
		}
		return Cost;
	}

	// Partial-pivot Gaussian elimination on a tiny damped, positive-definite system.
	bool SolvePoseSystem(double Matrix[PoseDimension][PoseDimension],
		double RHS[PoseDimension], double Out[PoseDimension])
	{
		for (int32 Column = 0; Column < PoseDimension; ++Column)
		{
			int32 PivotRow = Column;
			for (int32 Row = Column + 1; Row < PoseDimension; ++Row)
			{
				if (FMath::Abs(Matrix[Row][Column]) > FMath::Abs(Matrix[PivotRow][Column]))
				{
					PivotRow = Row;
				}
			}
			if (!FMath::IsFinite(Matrix[PivotRow][Column]) || FMath::Abs(Matrix[PivotRow][Column]) < 1e-12)
			{
				return false;
			}
			if (PivotRow != Column)
			{
				for (int32 Entry = Column; Entry < PoseDimension; ++Entry)
				{
					Swap(Matrix[PivotRow][Entry], Matrix[Column][Entry]);
				}
				Swap(RHS[PivotRow], RHS[Column]);
			}
			for (int32 Row = Column + 1; Row < PoseDimension; ++Row)
			{
				const double Factor = Matrix[Row][Column] / Matrix[Column][Column];
				for (int32 Entry = Column; Entry < PoseDimension; ++Entry)
				{
					Matrix[Row][Entry] -= Factor * Matrix[Column][Entry];
				}
				RHS[Row] -= Factor * RHS[Column];
			}
		}
		for (int32 Row = PoseDimension - 1; Row >= 0; --Row)
		{
			double Value = RHS[Row];
			for (int32 Column = Row + 1; Column < PoseDimension; ++Column)
			{
				Value -= Matrix[Row][Column] * Out[Column];
			}
			Out[Row] = Value / Matrix[Row][Row];
			if (!FMath::IsFinite(Out[Row]))
			{
				return false;
			}
		}
		return true;
	}
}

bool FRobotKinematics::ForwardKinematics(const TArray<FRobotJointDefinition>& Joints,
	const TArray<double>& JointRadians, const FTransform& ToolLocal,
	FTransform& OutTCP, TArray<FTransform>* OutJointTransforms)
{
	if (!RobotControlMath::IsChainValid(Joints, JointRadians, ToolLocal))
	{
		return false;
	}
	FTransform Parent = FTransform::Identity;
	if (OutJointTransforms)
	{
		OutJointTransforms->Reset(Joints.Num());
	}
	for (int32 Index = 0; Index < Joints.Num(); ++Index)
	{
		const FRobotJointDefinition& Joint = Joints[Index];
		const FQuat Rotation = Joint.HomeLocal.GetRotation() * FQuat(Joint.AxisLocal, JointRadians[Index]);
		const FTransform Local(Rotation, Joint.HomeLocal.GetTranslation(), FVector::OneVector);
		Parent = Local * Parent; // Unreal applies Local, then Parent.
		Parent.NormalizeRotation();
		if (OutJointTransforms)
		{
			OutJointTransforms->Add(Parent);
		}
	}
	OutTCP = ToolLocal * Parent;
	OutTCP.NormalizeRotation();
	return RobotControlMath::IsRigidTransform(OutTCP);
}

FVector FRobotKinematics::RotationError(const FQuat& Target, const FQuat& Current)
{
	FQuat Difference = Target * Current.Inverse();
	Difference.Normalize();
	if (Difference.W < 0.0)
	{
		Difference.X = -Difference.X;
		Difference.Y = -Difference.Y;
		Difference.Z = -Difference.Z;
		Difference.W = -Difference.W;
	}
	const FVector Imaginary(Difference.X, Difference.Y, Difference.Z);
	const double SineHalfAngle = Imaginary.Length();
	if (SineHalfAngle < 1e-10)
	{
		return 2.0 * Imaginary;
	}
	const double Angle = 2.0 * FMath::Atan2(SineHalfAngle, FMath::Clamp(Difference.W, 0.0, 1.0));
	return Imaginary * (Angle / SineHalfAngle);
}

FRobotIKSolution FRobotKinematics::SolveInverseKinematics(const TArray<FRobotJointDefinition>& Joints,
	const FTransform& ToolLocal, const FTransform& TargetBaseLocal,
	const TArray<double>& SeedRadians, const FRobotIKSettings& Settings)
{
	using namespace RobotControlMath;
	FRobotIKSolution Result;
	if (!IsChainValid(Joints, SeedRadians, ToolLocal) || !IsRigidTransform(TargetBaseLocal)
		|| !FMath::IsFinite(Settings.PositionToleranceCm) || Settings.PositionToleranceCm <= 0.0
		|| !FMath::IsFinite(Settings.OrientationToleranceRadians) || Settings.OrientationToleranceRadians <= 0.0
		|| !FMath::IsFinite(Settings.OrientationWeightCm) || Settings.OrientationWeightCm <= 0.0
		|| !FMath::IsFinite(Settings.DampingCm) || Settings.DampingCm <= 0.0
		|| !FMath::IsFinite(Settings.MaxStepRadians) || Settings.MaxStepRadians <= 0.0
		|| Settings.MaxIterations <= 0 || Settings.MaxRestarts < 0 || Settings.MaxRestarts > 16)
	{
		return Result;
	}
	Result.Status = ERobotIKStatus::NoConvergence;
	double BestCost = TNumericLimits<double>::Max();
	bool bBestAtLimit = false;

	for (int32 Attempt = 0; Attempt <= Settings.MaxRestarts; ++Attempt)
	{
		TArray<double> Angles = SeedRadians;
		for (int32 Joint = 0; Joint < Joints.Num(); ++Joint)
		{
			// Deterministic restarts break straight-arm singularities without introducing
			// random run-to-run motion. A warm seed remains the first attempt.
			const double Offset = Attempt == 0 ? 0.0
				: FMath::Sin((Joint + 1) * (Attempt + 1) * 1.61803398875) * (0.25 * Attempt);
			Angles[Joint] = FMath::Clamp(Angles[Joint] + Offset, Joints[Joint].MinRadians, Joints[Joint].MaxRadians);
		}
		int32 StalledIterations = 0;
		for (int32 Iteration = 0; Iteration < Settings.MaxIterations; ++Iteration)
		{
			++Result.Iterations;
			FTransform Current;
			if (!ForwardKinematics(Joints, Angles, ToolLocal, Current))
			{
				return FRobotIKSolution();
			}
			double Residual[PoseDimension];
			double PositionError;
			double OrientationError;
			PoseResidual(TargetBaseLocal, Current, Settings.OrientationWeightCm,
				Residual, PositionError, OrientationError);
			const double Cost = ResidualCost(Residual);
			if (Cost < BestCost)
			{
				BestCost = Cost;
				Result.JointRadians = Angles;
				Result.PositionErrorCm = PositionError;
				Result.OrientationErrorRadians = OrientationError;
				bBestAtLimit = false;
				for (int32 Joint = 0; Joint < Joints.Num(); ++Joint)
				{
					bBestAtLimit |= FMath::Abs(Angles[Joint] - Joints[Joint].MinRadians) < 0.00001
						|| FMath::Abs(Angles[Joint] - Joints[Joint].MaxRadians) < 0.00001;
				}
			}
			if (PositionError <= Settings.PositionToleranceCm && OrientationError <= Settings.OrientationToleranceRadians)
			{
				Result.Status = ERobotIKStatus::Success;
				Result.JointRadians = Angles;
				Result.PositionErrorCm = PositionError;
				Result.OrientationErrorRadians = OrientationError;
				return Result;
			}

			TArray<TArray<double>> Jacobian;
			Jacobian.SetNum(PoseDimension);
			for (int32 Row = 0; Row < PoseDimension; ++Row)
			{
				Jacobian[Row].Init(0.0, Joints.Num());
			}
			for (int32 Joint = 0; Joint < Joints.Num(); ++Joint)
			{
				TArray<double> Perturbed = Angles;
				double Step = FMath::Min(DifferenceStep, Joints[Joint].MaxRadians - Angles[Joint]);
				if (Step < DifferenceStep * 0.1)
				{
					Step = -FMath::Min(DifferenceStep, Angles[Joint] - Joints[Joint].MinRadians);
				}
				if (FMath::Abs(Step) < 1e-12)
				{
					continue; // Fixed joint.
				}
				Perturbed[Joint] += Step;
				FTransform PerturbedTCP;
				if (!ForwardKinematics(Joints, Perturbed, ToolLocal, PerturbedTCP))
				{
					return FRobotIKSolution();
				}
				const FVector Translation = (PerturbedTCP.GetTranslation() - Current.GetTranslation()) / Step;
				const FVector Rotation = RotationError(PerturbedTCP.GetRotation(), Current.GetRotation())
					* (Settings.OrientationWeightCm / Step);
				Jacobian[0][Joint] = Translation.X;
				Jacobian[1][Joint] = Translation.Y;
				Jacobian[2][Joint] = Translation.Z;
				Jacobian[3][Joint] = Rotation.X;
				Jacobian[4][Joint] = Rotation.Y;
				Jacobian[5][Joint] = Rotation.Z;
			}
			double Normal[PoseDimension][PoseDimension] = {};
			double RHS[PoseDimension];
			for (int32 Row = 0; Row < PoseDimension; ++Row)
			{
				RHS[Row] = Residual[Row];
				for (int32 Column = 0; Column < PoseDimension; ++Column)
				{
					for (int32 Joint = 0; Joint < Joints.Num(); ++Joint)
					{
						Normal[Row][Column] += Jacobian[Row][Joint] * Jacobian[Column][Joint];
					}
				}
				Normal[Row][Row] += FMath::Square(Settings.DampingCm);
			}
			double PoseStep[PoseDimension] = {};
			if (!SolvePoseSystem(Normal, RHS, PoseStep))
			{
				break;
			}
			TArray<double> Delta;
			Delta.Init(0.0, Joints.Num());
			double LargestStep = 0.0;
			for (int32 Joint = 0; Joint < Joints.Num(); ++Joint)
			{
				for (int32 Row = 0; Row < PoseDimension; ++Row)
				{
					Delta[Joint] += Jacobian[Row][Joint] * PoseStep[Row];
				}
				LargestStep = FMath::Max(LargestStep, FMath::Abs(Delta[Joint]));
			}
			const double StepScale = LargestStep > Settings.MaxStepRadians
				? Settings.MaxStepRadians / LargestStep : 1.0;
			bool bImproved = false;
			for (int32 Backtrack = 0; Backtrack < 8; ++Backtrack)
			{
				const double Scale = StepScale * FMath::Pow(0.5, Backtrack);
				TArray<double> Candidate = Angles;
				for (int32 Joint = 0; Joint < Joints.Num(); ++Joint)
				{
					Candidate[Joint] = FMath::Clamp(Angles[Joint] + Delta[Joint] * Scale,
						Joints[Joint].MinRadians, Joints[Joint].MaxRadians);
				}
				FTransform CandidateTCP;
				ForwardKinematics(Joints, Candidate, ToolLocal, CandidateTCP);
				double CandidateResidual[PoseDimension];
				double CandidatePositionError;
				double CandidateOrientationError;
				PoseResidual(TargetBaseLocal, CandidateTCP, Settings.OrientationWeightCm,
					CandidateResidual, CandidatePositionError, CandidateOrientationError);
				if (ResidualCost(CandidateResidual) < Cost - 1e-12)
				{
					Angles = MoveTemp(Candidate);
					bImproved = true;
					break;
				}
			}
			StalledIterations = bImproved ? 0 : StalledIterations + 1;
			if (StalledIterations >= 4)
			{
				break;
			}
		}
	}
	Result.Status = bBestAtLimit ? ERobotIKStatus::JointLimit : ERobotIKStatus::NoConvergence;
	return Result;
}

const TCHAR* FRobotKinematics::StatusText(ERobotIKStatus Status)
{
	switch (Status)
	{
	case ERobotIKStatus::Success: return TEXT("Converged");
	case ERobotIKStatus::InvalidInput: return TEXT("Invalid kinematic input");
	case ERobotIKStatus::JointLimit: return TEXT("Joint limit / target not reached");
	default: return TEXT("Target not reached");
	}
}

bool FRobotJointController::Step(const FRobotJointDefinition& Joint,
	const FRobotJointControllerSettings& Settings, double TargetRadians,
	double FeedforwardRadiansPerSecond, double DeltaSeconds, FRobotJointState& InOutState)
{
	if (!RobotControlMath::IsJointValid(Joint)
		|| !FMath::IsFinite(TargetRadians) || TargetRadians < Joint.MinRadians - 1e-8 || TargetRadians > Joint.MaxRadians + 1e-8
		|| !FMath::IsFinite(FeedforwardRadiansPerSecond) || !FMath::IsFinite(DeltaSeconds) || DeltaSeconds <= 0.0 || DeltaSeconds > 0.25
		|| !FMath::IsFinite(Settings.Kp) || Settings.Kp < 0.0
		|| !FMath::IsFinite(Settings.Ki) || Settings.Ki < 0.0
		|| !FMath::IsFinite(Settings.Kd) || Settings.Kd < 0.0
		|| !FMath::IsFinite(Settings.VelocityResponseSeconds) || Settings.VelocityResponseSeconds < 0.0
		|| !FMath::IsFinite(Settings.IntegralLimitRadiansSeconds) || Settings.IntegralLimitRadiansSeconds < 0.0
		|| !FMath::IsFinite(InOutState.AngleRadians) || !FMath::IsFinite(InOutState.VelocityRadians)
		|| !FMath::IsFinite(InOutState.IntegralErrorSeconds)
		|| InOutState.AngleRadians < Joint.MinRadians - 1e-8 || InOutState.AngleRadians > Joint.MaxRadians + 1e-8)
	{
		return false;
	}
	FRobotJointState State = InOutState;
	TargetRadians = FMath::Clamp(TargetRadians, Joint.MinRadians, Joint.MaxRadians);
	const double Error = TargetRadians - State.AngleRadians;
	const double CandidateIntegral = FMath::Clamp(State.IntegralErrorSeconds + Error * DeltaSeconds,
		-Settings.IntegralLimitRadiansSeconds, Settings.IntegralLimitRadiansSeconds);
	const double BaseOutput = Settings.Kp * Error - Settings.Kd * State.VelocityRadians + FeedforwardRadiansPerSecond;
	const double CandidateOutput = BaseOutput + Settings.Ki * CandidateIntegral;
	// Allow integration when unsaturated or when its sign unwinds saturation.
	if (FMath::Abs(CandidateOutput) <= Joint.MaxVelocityRadians || CandidateOutput * Error <= 0.0)
	{
		State.IntegralErrorSeconds = CandidateIntegral;
	}
	State.DesiredVelocityRadians = FMath::Clamp(BaseOutput + Settings.Ki * State.IntegralErrorSeconds,
		-Joint.MaxVelocityRadians, Joint.MaxVelocityRadians);
	const double Response = Settings.VelocityResponseSeconds > 1e-9
		? 1.0 - FMath::Exp(-DeltaSeconds / Settings.VelocityResponseSeconds) : 1.0;
	const double PlantVelocity = State.VelocityRadians + Response * (State.DesiredVelocityRadians - State.VelocityRadians);
	const double NewVelocity = FMath::Clamp(PlantVelocity,
		State.VelocityRadians - Joint.MaxAccelerationRadians * DeltaSeconds,
		State.VelocityRadians + Joint.MaxAccelerationRadians * DeltaSeconds);
	const double NewAngle = State.AngleRadians + (State.VelocityRadians + NewVelocity) * 0.5 * DeltaSeconds;
	State.AngleRadians = FMath::Clamp(NewAngle, Joint.MinRadians, Joint.MaxRadians);
	State.VelocityRadians = FMath::Clamp(NewVelocity, -Joint.MaxVelocityRadians, Joint.MaxVelocityRadians);
	if (State.AngleRadians <= Joint.MinRadians && State.VelocityRadians < 0.0)
	{
		State.VelocityRadians = 0.0;
		State.IntegralErrorSeconds = FMath::Max(0.0, State.IntegralErrorSeconds);
	}
	if (State.AngleRadians >= Joint.MaxRadians && State.VelocityRadians > 0.0)
	{
		State.VelocityRadians = 0.0;
		State.IntegralErrorSeconds = FMath::Min(0.0, State.IntegralErrorSeconds);
	}
	if (!FMath::IsFinite(State.AngleRadians) || !FMath::IsFinite(State.VelocityRadians)
		|| !FMath::IsFinite(State.IntegralErrorSeconds) || !FMath::IsFinite(State.DesiredVelocityRadians))
	{
		return false;
	}
	InOutState = State;
	return true;
}

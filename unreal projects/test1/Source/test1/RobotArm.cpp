#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"

ARobotArm::ARobotArm()
{
	PrimaryActorTick.bCanEverTick = false;
	ArmRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RobotBase"));
	SetRootComponent(ArmRoot);
	ArmRoot->SetMobility(EComponentMobility::Movable);
	for (int32 i=0;i<6;++i)
	{
		auto* Node = CreateDefaultSubobject<USceneComponent>(FName(*FString::Printf(TEXT("JointA%d"),i+1)));
		Node->SetupAttachment(i ? JointNodes[i-1].Get() : ArmRoot.Get());
		Node->SetMobility(EComponentMobility::Movable);
		JointNodes.Add(Node);
	}
	for (int32 i=0;i<24;++i)
	{
		auto* Mesh = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("RobotVisual%02d"),i)));
		Mesh->SetupAttachment(ArmRoot);
		Mesh->SetMobility(EComponentMobility::Movable);
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Mesh->SetGenerateOverlapEvents(false);
		Mesh->SetVisibility(false);
		Visuals.Add(Mesh);
	}
}

void ARobotArm::OnConstruction(const FTransform& Transform) { Super::OnConstruction(Transform); InitializeArm(); }
void ARobotArm::BeginPlay() { Super::BeginPlay(); if (!bInitialized) InitializeArm(); }

bool ARobotArm::InitializeArm()
{
	bInitialized = false;
	Definitions.Reset(); Angles.Reset(); Targets.Reset(); Controllers.Reset();
	if (Joints.Num()!=6 || InitialAnglesDegrees.Num()!=6 || MeshBindings.Num()>Visuals.Num()
		|| !GetActorTransform().IsValid() || !GetActorScale3D().Equals(FVector::OneVector,1.e-6))
	{ LastStatus=TEXT("Six calibrated joints, six initial angles, and unit actor scale required."); return false; }
	for (int32 i=0;i<6;++i)
	{
		const auto& J=Joints[i];
		FRobotJointDefinition D;
		D.HomeLocal=J.HomeLocalCm; D.AxisLocal=J.AxisLocal;
		D.MinRadians=FMath::DegreesToRadians(J.MinimumDegrees); D.MaxRadians=FMath::DegreesToRadians(J.MaximumDegrees);
		D.MaxVelocityRadians=FMath::DegreesToRadians(J.MaxVelocityDegrees);
		D.MaxAccelerationRadians=FMath::DegreesToRadians(J.MaxAccelerationDegrees);
		const double Q=FMath::DegreesToRadians(InitialAnglesDegrees[i]);
		if (!D.HomeLocal.IsValid() || !D.HomeLocal.GetScale3D().Equals(FVector::OneVector,1.e-6)
			|| D.AxisLocal.ContainsNaN() || D.AxisLocal.IsNearlyZero() || !FMath::IsFinite(Q)
			|| Q<D.MinRadians || Q>D.MaxRadians || D.MinRadians>=D.MaxRadians
			|| D.MaxVelocityRadians<=0 || D.MaxAccelerationRadians<=0)
		{ LastStatus=TEXT("Invalid joint calibration or initial angle."); return false; }
		D.AxisLocal.Normalize(); Definitions.Add(D); Angles.Add(Q); Targets.Add(Q);
		FRobotJointState Controller; Controller.AngleRadians=Q; Controllers.Add(Controller);
	}
	FTransform Test;
	if (!FRobotKinematics::ForwardKinematics(Definitions,Angles,ToolInLastJointCm,Test))
	{ LastStatus=TEXT("Invalid TCP calibration."); return false; }
	for (int32 i=0;i<Visuals.Num();++i)
	{
		const bool Active=MeshBindings.IsValidIndex(i) && MeshBindings[i].Mesh;
		Visuals[i]->SetVisibility(Active);
		if (!Active) continue;
		const auto& B=MeshBindings[i];
		if (B.JointIndex < -1 || B.JointIndex>5 || !B.RelativePoseCm.IsValid())
		{ LastStatus=TEXT("Invalid mesh-to-joint binding."); return false; }
		Visuals[i]->AttachToComponent(B.JointIndex<0 ? ArmRoot.Get() : JointNodes[B.JointIndex].Get(),FAttachmentTransformRules::KeepRelativeTransform);
		Visuals[i]->SetStaticMesh(B.Mesh); Visuals[i]->SetRelativeTransform(B.RelativePoseCm);
	}
	SegmentDuration=0; SegmentElapsed=0; bInitialized=true; UpdateVisuals();
	LastStatus=TEXT("Calibrated. PID velocity servo with simplified motor response."); return true;
}

bool ARobotArm::SetInitialAngles(const TArray<double>& Values)
{ InitialAnglesDegrees=Values; return InitializeArm(); }

FTransform ARobotArm::GetTCPWorld() const
{
	FTransform Local;
	return bInitialized && FRobotKinematics::ForwardKinematics(Definitions,Angles,ToolInLastJointCm,Local)
		? Local*GetActorTransform() : GetActorTransform();
}

bool ARobotArm::CommandTCPWorld(const FTransform& World)
{
	if (!bInitialized || !World.IsValid() || !World.GetScale3D().Equals(FVector::OneVector,1.e-6))
	{ LastStatus=TEXT("Invalid TCP command or uninitialized robot."); return false; }
	FRobotIKSettings IK; IK.PositionToleranceCm=0.03; IK.OrientationToleranceRadians=0.001;
	const auto Solution=FRobotKinematics::SolveInverseKinematics(Definitions,ToolInLastJointCm,
		World.GetRelativeTransform(GetActorTransform()),Angles,IK);
	LastIKIterations=Solution.Iterations;
	if (Solution.Status!=ERobotIKStatus::Success)
	{
		LastStatus=FString::Printf(TEXT("IK failed: status %d, position %.2f mm, orientation %.2f deg."),
			int32(Solution.Status),Solution.PositionErrorCm*10,FMath::RadiansToDegrees(Solution.OrientationErrorRadians));
		return false;
	}
	SegmentStart=Angles; Targets=Solution.JointRadians; SegmentElapsed=0; SegmentDuration=0.25;
	for (int32 i=0;i<6;++i)
	{
		const double Distance=FMath::Abs(Targets[i]-Angles[i]);
		// Quintic smoothstep has peak normalized speed 1.875 and acceleration < 6.
		SegmentDuration=FMath::Max(SegmentDuration,1.875*Distance/Definitions[i].MaxVelocityRadians);
		SegmentDuration=FMath::Max(SegmentDuration,FMath::Sqrt(6*Distance/Definitions[i].MaxAccelerationRadians));
	}
	LastStatus=TEXT("IK solved; tracking joint trajectory with PID."); UpdateVisuals(); return true;
}

bool ARobotArm::CommandJointDegrees(const TArray<double>& AnglesDegrees)
{
	if (!bInitialized || AnglesDegrees.Num() != Definitions.Num())
	{ LastStatus = TEXT("Invalid joint-space target."); return false; }
	TArray<double> NewTargets;
	for (int32 Index = 0; Index < AnglesDegrees.Num(); ++Index)
	{
		const double Value = FMath::DegreesToRadians(AnglesDegrees[Index]);
		if (!FMath::IsFinite(Value) || Value < Definitions[Index].MinRadians || Value > Definitions[Index].MaxRadians)
		{ LastStatus = TEXT("Joint-space target exceeds calibrated range."); return false; }
		NewTargets.Add(Value);
	}
	SegmentStart = Angles; Targets = MoveTemp(NewTargets); SegmentElapsed = 0; SegmentDuration = 0.25;
	for (int32 Index = 0; Index < Targets.Num(); ++Index)
	{
		const double Travel = FMath::Abs(Targets[Index] - Angles[Index]);
		SegmentDuration = FMath::Max(SegmentDuration, 1.875 * Travel / Definitions[Index].MaxVelocityRadians);
		SegmentDuration = FMath::Max(SegmentDuration, FMath::Sqrt(6 * Travel / Definitions[Index].MaxAccelerationRadians));
	}
	LastStatus = TEXT("Tracking calibrated joint-space transit with PID.");
	UpdateVisuals(); return true;
}

FTransform ARobotArm::GetTargetTCPWorld() const
{
	FTransform Local;
	return bInitialized && FRobotKinematics::ForwardKinematics(Definitions, Targets, ToolInLastJointCm, Local)
		? Local * GetActorTransform() : GetActorTransform();
}

bool ARobotArm::StepControl(double Dt)
{
	if (!bInitialized || !FMath::IsFinite(Dt) || Dt<=0 || Dt>0.1
		|| !FMath::IsFinite(Kp) || !FMath::IsFinite(Ki) || !FMath::IsFinite(Kd) || Kp<0 || Ki<0 || Kd<0
		|| !FMath::IsFinite(MotorResponseSeconds) || MotorResponseSeconds<=0) return false;
	SegmentElapsed+=Dt;
	const double S=SegmentDuration>0 ? FMath::Clamp(SegmentElapsed/SegmentDuration,0.,1.) : 1.;
	const double Blend=S*S*S*(10+S*(-15+6*S));
	const double Rate=SegmentDuration>0 && S<1 ? 30*S*S*(1-S)*(1-S)/SegmentDuration : 0;
	FRobotJointControllerSettings Settings; Settings.Kp=Kp; Settings.Ki=Ki; Settings.Kd=Kd; Settings.VelocityResponseSeconds=MotorResponseSeconds;
	for (int32 i=0;i<6;++i)
	{
		const double Start=SegmentStart.IsValidIndex(i) ? SegmentStart[i] : Targets[i];
		const double Goal=Start+(Targets[i]-Start)*Blend;
		if (!FRobotJointController::Step(Definitions[i],Settings,Goal,(Targets[i]-Start)*Rate,Dt,Controllers[i])) return false;
		Angles[i]=Controllers[i].AngleRadians;
	}
	UpdateVisuals(); return true;
}

void ARobotArm::Halt()
{
	Targets=Angles; SegmentStart=Angles; SegmentDuration=0; SegmentElapsed=0;
	for (auto& C:Controllers) { C.VelocityRadians=0; C.IntegralErrorSeconds=0; }
	LastStatus=TEXT("Stopped: simulated drive hold. Reset required."); UpdateVisuals();
}

bool ARobotArm::IsAtJointTarget(double ToleranceDegrees) const
{
	if (!bInitialized || Angles.Num()!=Targets.Num() || SegmentElapsed<SegmentDuration) return false;
	for (int32 i=0;i<Angles.Num();++i)
		if (FMath::Abs(Angles[i]-Targets[i])>FMath::DegreesToRadians(ToleranceDegrees)
			|| FMath::Abs(Controllers[i].VelocityRadians)>FMath::DegreesToRadians(0.2)) return false;
	return true;
}

void ARobotArm::UpdateVisuals()
{
	CurrentAnglesDegrees.Reset(); TargetAnglesDegrees.Reset();
	for (int32 i=0;i<Angles.Num();++i)
	{
		FTransform Relative=Definitions[i].HomeLocal;
		Relative.SetRotation((Relative.GetRotation()*FQuat(Definitions[i].AxisLocal,Angles[i])).GetNormalized());
		JointNodes[i]->SetRelativeTransform(Relative);
		CurrentAnglesDegrees.Add(FMath::RadiansToDegrees(Angles[i])); TargetAnglesDegrees.Add(FMath::RadiansToDegrees(Targets[i]));
	}
}

#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/BoxComponent.h"
#include "HAL/PlatformTime.h"
#include "CoreGlobals.h"
#include "Engine/World.h"
#include "Misc/ScopeExit.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "RobotCommercialProfile.h"

namespace
{
// Explicit test-runner opt-in: a blocking simulation loop cannot advance the
// rendering frame counter. Interactive PIE always retains its frame budget.
bool SynchronousMotionTest()
{
    static const bool Enabled=FApp::IsUnattended()&&FParse::Param(FCommandLine::Get(),TEXT("RobotMotionSynchronousTest"));
    return Enabled;
}
}

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
		auto* Box = CreateDefaultSubobject<UBoxComponent>(FName(*FString::Printf(TEXT("DropCollision%02d"),i)));
		Box->SetupAttachment(Mesh);
		Box->SetMobility(EComponentMobility::Movable);
		Box->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Box->SetCollisionResponseToAllChannels(ECR_Ignore);
		Box->SetCollisionResponseToChannel(ECC_PhysicsBody,ECR_Block);
		DropCollisionBoxes.Add(Box);
	}
}

void ARobotArm::OnConstruction(const FTransform& Transform) { Super::OnConstruction(Transform); InitializeArm(); }
void ARobotArm::BeginPlay() { Super::BeginPlay(); if (!bInitialized) InitializeArm(); }

bool ARobotArm::InitializeArm()
{
	bInitialized = false;
    PendingMotion.Reset();ActiveTrajectory=FRobotMotionTrajectory();
    MotionPath.Reset();MotionDurations.Reset();MotionIndex=0;MotionScene=FRobotMotionScene();SceneAge=0;
    MotionPart.Reset(); bMotionHolding=false; bMotionBlocked=false; PreferredIKSeedDegrees.Reset();
	Definitions.Reset(); Angles.Reset(); Targets.Reset(); Controllers.Reset();
	if (Joints.Num()!=6 || InitialAnglesDegrees.Num()!=6 || MeshBindings.Num()>Visuals.Num()
		|| !GetActorTransform().IsValid() || !GetActorScale3D().Equals(FVector::OneVector,1.e-6))
	{ LastStatus=TEXT("Six calibrated joints, six initial angles, and unit actor scale required."); return false; }
    const auto PreviousSetup=Joints;
    if(!RobotCommercialProfile::Apply(*this,LastStatus))return false;
    bool InitializationCommitted=false;
    ON_SCOPE_EXIT{if(!InitializationCommitted)Joints=PreviousSetup;};
    bool ReferenceEnvelopeChanged=false;
    for(int I=0;I<6;++I)ReferenceEnvelopeChanged|=PreviousSetup[I].MinimumDegrees!=Joints[I].MinimumDegrees||PreviousSetup[I].MaximumDegrees!=Joints[I].MaximumDegrees;
    bool NeedsInitialSolve=false;
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
			|| D.MinRadians>=D.MaxRadians
			|| D.MaxVelocityRadians<=0 || D.MaxAccelerationRadians<=0)
		{ LastStatus=TEXT("Invalid joint calibration or initial angle."); return false; }
        NeedsInitialSolve|=Q<D.MinRadians-1.e-8||Q>D.MaxRadians+1.e-8;
        if((Q<D.MinRadians-1.e-8||Q>D.MaxRadians+1.e-8)&&!ReferenceEnvelopeChanged)
        {LastStatus=TEXT("초기 관절 각도가 설정 범위 밖입니다.");return false;}
		D.AxisLocal.Normalize(); Definitions.Add(D); Angles.Add(Q); Targets.Add(Q);
		FRobotJointState Controller; Controller.AngleRadians=Q; Controllers.Add(Controller);
	}
	FTransform Test;
	if (!FRobotKinematics::ForwardKinematics(Definitions,Angles,ToolInLastJointCm,Test))
	{ LastStatus=TEXT("Invalid TCP calibration."); return false; }
    if(NeedsInitialSolve)
    {
        // Reset/configuration only: preserve the requested TCP if a previously
        // saved demo branch lies outside the selected reference envelope.
        FRobotIKSettings Settings;Settings.MaxRestarts=8;Settings.PositionToleranceCm=.01;Settings.OrientationToleranceRadians=.0002;
        auto Sol=FRobotKinematics::SolveInverseKinematics(Definitions,ToolInLastJointCm,Test,Angles,Settings);
        if(!Sol.IsSuccess())
        {
            LastStatus=TEXT("초기 자세가 참조 관절 범위 밖이며 동일 TCP의 IK 해를 찾지 못했습니다.");
            UE_LOG(LogTemp,Warning,TEXT("MOTION_INITIAL_REJECT %s q=%s residual=%.3fcm"),*GetName(),*FString::JoinBy(InitialAnglesDegrees,TEXT(","),[](double Q){return FString::Printf(TEXT("%.2f"),Q);}),Sol.PositionErrorCm);
            return false;
        }
        Angles=Targets=Sol.JointRadians;
        for(int I=0;I<6;++I){InitialAnglesDegrees[I]=FMath::RadiansToDegrees(Angles[I]);Controllers[I].AngleRadians=Angles[I];}
    }
    // A saved demo home can fit A5 while leaving no room for the next pickup.
    // At RESET only, prefer an equivalent wrist configuration with more A5
    // travel. Never assume an imported wrist convention: FK must prove that
    // both position and orientation are unchanged before accepting a branch.
    if(CalibrationSource.StartsWith(TEXT("robot_calibration.json:"))
        &&FMath::Min(Angles[4]-Definitions[4].MinRadians,Definitions[4].MaxRadians-Angles[4])<FMath::DegreesToRadians(15.))
    {
        auto Best=Angles;double BestCost=TNumericLimits<double>::Max();
        for(double A:{-UE_DOUBLE_PI,UE_DOUBLE_PI})for(double B:{-UE_DOUBLE_PI,UE_DOUBLE_PI})
        {
            auto Candidate=Angles;Candidate[3]+=A;Candidate[4]=UE_DOUBLE_PI-Candidate[4];Candidate[5]+=B;
            bool Valid=true;for(int I=0;I<6;++I)Valid&=Candidate[I]>=Definitions[I].MinRadians&&Candidate[I]<=Definitions[I].MaxRadians;
            const double Margin=FMath::Min(Candidate[4]-Definitions[4].MinRadians,Definitions[4].MaxRadians-Candidate[4]);
            FTransform Pose;
            if(!Valid||Margin<FMath::DegreesToRadians(30.)||!FRobotKinematics::ForwardKinematics(Definitions,Candidate,ToolInLastJointCm,Pose)
                ||!Pose.GetLocation().Equals(Test.GetLocation(),.001)||Pose.GetRotation().AngularDistance(Test.GetRotation())>.00001)continue;
            const double Cost=FMath::Abs(Candidate[3])+FMath::Abs(Candidate[5]);
            if(Cost<BestCost){Best=Candidate;BestCost=Cost;}
        }
        if(BestCost<TNumericLimits<double>::Max())
        {
            Angles=Targets=Best;
            for(int I=0;I<6;++I){InitialAnglesDegrees[I]=FMath::RadiansToDegrees(Angles[I]);Controllers[I].AngleRadians=Angles[I];}
        }
    }
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
		const FBox Bounds=B.Mesh->GetBoundingBox();
		DropCollisionBoxes[i]->SetBoxExtent(Bounds.GetExtent()*0.85);
		DropCollisionBoxes[i]->SetRelativeLocation(Bounds.GetCenter());
	}
	SegmentDuration=0; SegmentElapsed=0; bInitialized=true; InitializationCommitted=true; UpdateVisuals();
	LastStatus=TEXT("Calibrated. PID velocity servo with simplified motor response."); return true;
}

void ARobotArm::SetDroppedWheelCollision(bool bEnabled)
{
	for(int32 I=0;I<DropCollisionBoxes.Num();++I)
		DropCollisionBoxes[I]->SetCollisionEnabled(bEnabled && MeshBindings.IsValidIndex(I) && MeshBindings[I].Mesh
			? ECollisionEnabled::QueryOnly : ECollisionEnabled::NoCollision);
}

bool ARobotArm::SetInitialAngles(const TArray<double>& Values)
{ InitialAnglesDegrees=Values; return InitializeArm(); }

FTransform ARobotArm::GetTCPWorld() const
{
	FTransform Local;
	return bInitialized && FRobotKinematics::ForwardKinematics(Definitions,Angles,ToolInLastJointCm,Local)
		? Local*GetActorTransform() : GetActorTransform();
}

void ARobotArm::SetMotionPart(USceneComponent* Part,bool Held)
{
    MotionPart=Part;bMotionHolding=Held;
}
#include "RobotMotionPlanning.inl"

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
    return PlanJointMove(NewTargets);
}

FTransform ARobotArm::GetTargetTCPWorld() const
{
	if(PendingMotion)return PendingMotion->Goal;
	FTransform Local;
	return bInitialized && FRobotKinematics::ForwardKinematics(Definitions, MotionPath.IsEmpty()?Targets:MotionPath.Last(), ToolInLastJointCm, Local)
		? Local * GetActorTransform() : GetActorTransform();
}

bool ARobotArm::StepControl(double Dt)
{
	if (!bInitialized || !FMath::IsFinite(Dt) || Dt<=0 || Dt>0.1
		|| !FMath::IsFinite(Kp) || !FMath::IsFinite(Ki) || !FMath::IsFinite(Kd) || Kp<0 || Ki<0 || Kd<0
		|| !FMath::IsFinite(MotorResponseSeconds) || MotorResponseSeconds<=0) return false;
    if(bMotionBlocked)return false;
    SceneAge+=Dt;
    if(SceneAge>=.1){MotionScene.Capture(*this,MotionPart.Get(),bMotionHolding);SceneAge=0;}
    MotionScene.RefreshTransforms();
    // Cold CAD acceleration structures are built off-thread before planning.
    // Hold the current pose until ready, rather than counting asset preparation
    // against the IK budget or accepting unchecked geometry. Headless tests wait.
    if(!MotionScene.PrepareGeometry(!FApp::CanEverRender()||SynchronousMotionTest()))
    {LastStatus=TEXT("충돌 형상 준비 중 · 로봇 대기");return true;}
    if(PendingMotion&&!AdvanceMotionPlanning())return false;
    const auto PreviousAngles=Angles;const auto PreviousControllers=Controllers;
    SegmentElapsed+=Dt;
    TArray<double> Reference=Targets,Feed;Feed.Init(0,Targets.Num());
    if(ActiveTrajectory.Samples.Num()>1)ActiveTrajectory.Reference(SegmentElapsed,Reference,Feed);
	FRobotJointControllerSettings Settings; Settings.Kp=Kp; Settings.Ki=Ki; Settings.Kd=Kd; Settings.VelocityResponseSeconds=MotorResponseSeconds;
    TArray<FTransform> JointFrames;FTransform TCP;
    if(bTorqueDynamics&&!FRobotKinematics::ForwardKinematics(Definitions,Angles,ToolInLastJointCm,TCP,&JointFrames))return false;
    AppliedJointTorqueNm.SetNumZeroed(6);
    for (int32 i=0;i<6;++i)
    {
        const double Goal=Reference[i];
        if(bTorqueDynamics)
        {
            const auto Frame=JointFrames[i]*GetActorTransform();
            const FVector Axis=Frame.TransformVectorNoScale(Definitions[i].AxisLocal);
            double Load=FVector::DotProduct(Axis,FVector::CrossProduct((ForcePointWorldCm-Frame.GetLocation())/100,ToolReactionForceN)+ToolReactionTorqueNm);
            if(LinkMassKg.Num()!=6||LinkCOMCm.Num()!=6||JointInertiaKgM2.Num()!=6||JointDriveLimitNm.Num()!=6)return false;
            double Inertia=JointInertiaKgM2[i];
            for(int Link=i;Link<6;++Link)
            {
                if(!FMath::IsFinite(LinkMassKg[Link])||LinkMassKg[Link]<=0||LinkCOMCm[Link].ContainsNaN())return false;
                const FVector R=((JointFrames[Link]*GetActorTransform()).TransformPosition(LinkCOMCm[Link])-Frame.GetLocation())/100;
                Load+=FVector::DotProduct(Axis,FVector::CrossProduct(R,FVector(0,0,-9.81*LinkMassKg[Link])));
                Inertia+=LinkMassKg[Link]*FVector::CrossProduct(R,Axis).SizeSquared();
            }
            if(!FRobotTorquePlant::Step(Definitions[i],Goal,Feed[i],Inertia,
                JointDriveLimitNm[i]*TorqueLimitScale,Load,Dt,Controllers[i],AppliedJointTorqueNm[i]))return false;
        }
        else if (!FRobotJointController::Step(Definitions[i],Settings,Goal,Feed[i],Dt,Controllers[i])) return false;
		Angles[i]=Controllers[i].AngleRadians;
	}
    FString Collision;
    FTransform ActualTCP;FRobotKinematics::ForwardKinematics(Definitions,Angles,ToolInLastJointCm,ActualTCP);
    const bool InCorridor=ActiveTrajectory.InCorridor(ActualTCP*GetActorTransform());
    if(!InCorridor||!MotionScene.Clear(*this,Angles,Collision))
    {
        if(!InCorridor)Collision=TEXT("삽입 자세/직선 허용 오차 초과 · 강제 체결 차단");
        Angles=PreviousAngles;Controllers=PreviousControllers;Halt();bMotionBlocked=true;
        for(auto& C:Controllers)C.VelocityRadians=0;
        LastStatus=TEXT("운행 중 간섭 정지 · ")+Collision;
        UE_LOG(LogTemp,Warning,TEXT("MOTION_STOP %s %s"),*GetName(),*LastStatus);return false;
    }
    UpdateVisuals(); return true;
}

void ARobotArm::Halt()
{
    PendingMotion.Reset();ActiveTrajectory=FRobotMotionTrajectory();
	MotionPath.Reset();
    Targets=Angles; SegmentStart=Angles; SegmentDuration=0; SegmentElapsed=0;
	for (auto& C:Controllers) { if(!bTorqueDynamics)C.VelocityRadians=0; C.IntegralErrorSeconds=0; }
	LastStatus=TEXT("Stopped: simulated drive hold. Reset required."); UpdateVisuals();
}

bool ARobotArm::IsAtJointTarget(double ToleranceDegrees) const
{
	return !IsMotionPlanning()&&!bMotionBlocked&&MotionIndex+1>=MotionPath.Num()&&SegmentSettled(ToleranceDegrees);
}

void ARobotArm::EndPlay(const EEndPlayReason::Type Reason)
{
    PendingMotion.Reset();ActiveTrajectory=FRobotMotionTrajectory();MotionScene=FRobotMotionScene();
    Super::EndPlay(Reason);
}

bool ARobotArm::SegmentSettled(double ToleranceDegrees) const
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
		CurrentAnglesDegrees.Add(FMath::RadiansToDegrees(Angles[i])); TargetAnglesDegrees.Add(FMath::RadiansToDegrees(MotionPath.IsEmpty()?Targets[i]:MotionPath.Last()[i]));
	}
}

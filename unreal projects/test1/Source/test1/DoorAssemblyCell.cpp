#include "DoorAssemblyCell.h"
#include "AssemblyModelContracts.h"
#include "ReducedPartPhysics.h"
#include "RobotArm.h"
#include "DoorGripperGeometry.h"
#include "CamryAssembly.h"
#include "DoorPoseComponent.h"
#include "DoorInspectionComponent.h"
#include "VehicleProcessTest.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Components/SceneComponent.h"
#include "Components/BoxComponent.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"

ADoorAssemblyCell::ADoorAssemblyCell()
{
	PrimaryActorTick.bCanEverTick=true;
	RootComponent=CreateDefaultSubobject<USceneComponent>(TEXT("CellRoot"));
	DoorSizeProxy=CreateDefaultSubobject<UBoxComponent>(TEXT("DoorSizeScanProxy"));
	DoorSizeProxy->SetupAttachment(RootComponent);
	DoorSizeProxy->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	DoorSizeProxy->SetCollisionResponseToAllChannels(ECR_Ignore);
	DoorSizeProxy->SetCollisionResponseToChannel(ECC_Visibility,ECR_Block);
	DoorSizeProxy->SetHiddenInGame(true);
}

FString ADoorAssemblyCell::GetStateLabel() const
{
	return StaticEnum<EDoorAssemblyState>()->GetNameStringByValue(int64(State));
}

FString ADoorAssemblyCell::GetOperationTaskId() const
{
    if(TargetModule) return TargetModule->ModuleId.ToString().ToLower();
    return bRightDoor ? TEXT("door_fr") : OperationTaskId;
}

bool ADoorAssemblyCell::ScanDoorSize()
{
	if(!Pose.IsValid()||!DoorSizeProxy||DoorSizeScale<=0)return false;
	FBox Bounds(ForceInit);
    const FTransform Frame(Pose->GetControlledDoor()->GetComponentQuat(),Pose->GetControlledDoor()->GetComponentLocation());
    TArray<USceneComponent*> DoorParts;
    Pose->GetControlledDoor()->GetChildrenComponents(true,DoorParts);
    DoorParts.Add(Pose->GetControlledDoor());
    // Empty legacy components have zero bounds at their pivot, not physical surfaces.
    for(auto* C:DoorParts)if(auto* P=Cast<UStaticMeshComponent>(C))
        if(P->GetStaticMesh()&&!P->bHiddenInGame)
            Bounds+=P->CalcBounds(P->GetComponentTransform().GetRelativeTransform(Frame)).GetBox();
	if(!Bounds.IsValid)return false;
	const FVector Center=Frame.TransformPosition(Bounds.GetCenter()),Extent=Bounds.GetExtent();
    const FVector Axis=Frame.TransformVectorNoScale(FVector::ForwardVector);
    NominalDoorWidthCm=RecipeDoorWidthCm>0 ? RecipeDoorWidthCm : AssemblyRecipes::CamryDoorWidthCm(GetOperationTaskId());
	DoorSizeProxy->SetBoxExtent(Extent);
	DoorSizeProxy->SetWorldLocation(Center);
	DoorSizeProxy->SetWorldRotation(Frame.GetRotation());
	FHitResult Left,Right;
	const double Reach=Extent.X+100;
	const bool HitLeft=DoorSizeProxy->LineTraceComponent(Left,Center-Axis*Reach,Center+Axis*Reach,FCollisionQueryParams(SCENE_QUERY_STAT(DoorSizeLaser),false));
	const bool HitRight=DoorSizeProxy->LineTraceComponent(Right,Center+Axis*Reach,Center-Axis*Reach,FCollisionQueryParams(SCENE_QUERY_STAT(DoorSizeLaser),false));
	DoorSizeLeftHitWorld=HitLeft?Left.ImpactPoint:FVector::ZeroVector;
	DoorSizeRightHitWorld=HitRight?Right.ImpactPoint:FVector::ZeroVector;
	MeasuredDoorWidthCm=HitLeft&&HitRight?FVector::Distance(Left.ImpactPoint,Right.ImpactPoint):0;
	DoorSizeMeasuredAt=SimulationTimeSeconds;
	bDoorSizePassed=FAssemblyDimensionCriterion{NominalDoorWidthCm,DoorWidthToleranceCm}.Accepts(MeasuredDoorWidthCm,HitLeft&&HitRight);
	return bDoorSizePassed;
}

bool ADoorAssemblyCell::ValidateSetup()
{
	if (!IsValid(Assembly) || !IsValid(Robot) || !Assembly->GetBodyFrame())
	{ Status=TEXT("Assign the vehicle assembly and native robot."); return false; }
	Pose=Assembly->GetDoorPoseController(bRightDoor?ECamryDoorSide::Right:ECamryDoorSide::Left);
	if (IsValid(TargetModule))
	{
		if (!TargetModule->bHasReferencePose || TargetModule->VehicleFrame != Assembly || !TargetModule->GetRootComponent())
		{ Status=TEXT("Door module has no saved vehicle-frame mounting pose."); return false; }
		if (!ModulePose) ModulePose=NewObject<UDoorPoseComponent>(this);
        Pose=ModulePose.Get();
        Pose->Configure(TargetModule->GetRootComponent(), Assembly->GetBodyFrame());
		const FTransform MountWorld=TargetModule->ReferencePoseInVehicle*Assembly->GetActorTransform();
		if (!Pose->SetReferencePoseInBody(MountWorld.GetRelativeTransform(Assembly->GetBodyFrame()->GetComponentTransform())))
		{ Status=TEXT("Door module mounting pose is invalid."); return false; }
	}
	else Pose->Configure(bRightDoor?Assembly->GetRightDoor():Assembly->GetLeftDoor(), Assembly->GetBodyFrame());
	if (!Pose.IsValid() || !Pose->HasReferencePose() || !Pose->GetControlledDoor())
	{ Status=TEXT("A serialized normal left-door reference is required."); return false; }
	const double Values[]={PositionToleranceMm,OrientationToleranceDeg,SettleSeconds,StageTimeoutSeconds,FasteningSeconds,FixedStepSeconds};
	for (double V:Values) if (!FMath::IsFinite(V) || V<=0)
	{ Status=TEXT("Control tolerances and times must be finite and positive."); return false; }
	if (FixedStepSeconds>0.02 || !TCPInDoorCm.IsValid() || !TCPInDoorCm.GetScale3D().Equals(FVector::OneVector,1.e-6)
		|| PickupOffsetBodyCm.ContainsNaN() || ApproachOffsetBodyCm.ContainsNaN()
		|| LiftOffsetBodyCm.ContainsNaN() || TransferOffsetBodyCm.ContainsNaN())
	{ Status=TEXT("Invalid grasp, waypoint, or control period (maximum 20 ms)."); return false; }
	BodyAtStart=Assembly->GetBodyFrame()->GetComponentTransform();
	if (!BodyAtStart.IsValid() || !BodyAtStart.GetScale3D().Equals(FVector::OneVector,1.e-6))
	{ Status=TEXT("Vehicle body requires a finite rigid transform with unit scale."); return false; }
	MountInBody=Pose->GetReferencePoseInBody();
	RobotBaseAtStart=Robot->GetActorTransform();
	Inspection=InspectionOverride?InspectionOverride.Get():Assembly->FindComponentByClass<UDoorInspectionComponent>();
	if (!IsValid(TargetModule) && (!Inspection.IsValid() || Inspection->Points.Num()!=6))
	{ Status=TEXT("Six registered inspection stations are required."); return false; }
	const double Extra[]={GapToleranceMm,FlushToleranceMm,MaxPickupDisplacementMm,MaxPickupRotationDeg};
	for (double V:Extra) if (!FMath::IsFinite(V) || V<=0)
	{ Status=TEXT("Quality tolerances and pickup recovery limits must be finite and positive."); return false; }
	if (InitialDoorOffsetMm.ContainsNaN() || InitialDoorRotationDeg.ContainsNaN()
		|| AlignmentSensorBiasMm.ContainsNaN() || AlignmentSensorRotationBiasDeg.ContainsNaN()
		|| VerificationSensorBiasMm.ContainsNaN() || VerificationSensorRotationBiasDeg.ContainsNaN())
	{ Status=TEXT("Placement and sensor offsets must be finite."); return false; }
	return true;
}

FTransform ADoorAssemblyCell::GetMountDoorWorld() const
{
    if(!IsValid(Assembly)||!Assembly->GetBodyFrame())return FTransform::Identity;
    if (IsValid(TargetModule) && TargetModule->bHasReferencePose)
        return TargetModule->ReferencePoseInVehicle*Assembly->GetActorTransform();
    const auto* Reference=Assembly->GetDoorPoseController(bRightDoor?ECamryDoorSide::Right:ECamryDoorSide::Left);
    return Reference&&Reference->HasReferencePose()?Reference->GetReferencePoseInBody()*Assembly->GetBodyFrame()->GetComponentTransform():FTransform::Identity;
}

FTransform ADoorAssemblyCell::OffsetDoor(const FTransform& World,const FVector& Offset) const
{
	FTransform Result=World;
	Result.AddToTranslation(Assembly->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(Offset));
	return Result;
}

FTransform ADoorAssemblyCell::GetPickupDoorWorld() const
{
	return IsValid(Assembly) && Assembly->GetBodyFrame() ? OffsetDoor(GetMountDoorWorld(),PickupOffsetBodyCm) : FTransform::Identity;
}

bool ADoorAssemblyCell::ResetCycle()
{
    if(Robot)DoorGripperGeometry::Configure(*Robot);
    Operations.Reset();PhysicalDoor=FDoorRigidBody();bSupportCommand=false;
    if(Robot){Robot->ToolReactionForceN=Robot->ToolReactionTorqueNm=FVector::ZeroVector;}
	for (const auto& Entry:OriginalDoorMeshScales) if(Entry.Key.IsValid())Entry.Key->SetRelativeScale3D(Entry.Value);
	OriginalDoorMeshScales.Empty();
	if (Robot) Robot->Halt();
	FlushRecording();
	if (!ValidateSetup() || !Robot->InitializeArm())
	{ State=EDoorAssemblyState::Fault; if (Robot) Status+=TEXT(" ")+Robot->LastStatus; return false; }
	bDoorHeld=false; bDoorFastened=false;
	Outcome=EDoorCellOutcome::NotEvaluated; AlignmentCorrections=0; LogFile.Empty(); RunMetadataFile.Empty();
	SimulationTimeSeconds=0; Accumulator=0; StateTime=0; StableTime=0;
	LastAlignmentSampleTime=-1;bLastAlignmentSampleValid=false;
	FTransform Initial=GetPickupDoorWorld();
	Initial.AddToTranslation(BodyAtStart.TransformVectorNoScale(InitialDoorOffsetMm/10));
	Initial.SetRotation((BodyAtStart.GetRotation()*InitialDoorRotationDeg.Quaternion()*BodyAtStart.GetRotation().Inverse()*Initial.GetRotation()).GetNormalized());
	if (!Pose->SetDoorWorldPose(Initial)) { Fail(TEXT("Could not place door in the pickup station.")); return false; }
	TArray<USceneComponent*> DoorChildren; Pose->GetControlledDoor()->GetChildrenComponents(true,DoorChildren);
	for (auto* C:DoorChildren) if (auto* P=Cast<UPrimitiveComponent>(C)) P->SetVisibility(bDoorPresent);
	MeasuredDoorWidthCm=NominalDoorWidthCm=0;bDoorSizePassed=false;
	DoorSizeMeasuredAt=-1;
	DoorSizeLeftHitWorld=DoorSizeRightHitWorld=FVector::ZeroVector;
	if (DoorSizeScale!=1.0)
	{
		for(auto* C:DoorChildren)if(auto* P=Cast<UPrimitiveComponent>(C))
		{OriginalDoorMeshScales.Add(P,P->GetRelativeScale3D());P->SetRelativeScale3D(P->GetRelativeScale3D()*DoorSizeScale);}
	}
	RefreshQuality(); MeasureSensors();
	TargetTCPWorld=Robot->GetTCPWorld(); MeasureTCP();
	EnterState(EDoorAssemblyState::Idle,TEXT("Ready. Enter/Start runs the cycle; Space stops; R resets."));
	return true;
}

void ADoorAssemblyCell::FinishOperation(EFactoryCommandStatus Control)
{
    Operations.Finish(Operations.Active.Id,Control,
        bUsePhysicalDynamics&&PhysicalDoor.bSupportContact
            ? (PhysicalDoor.SupportDeflectionMm<=2?EAssemblyQuality::Pass:EAssemblyQuality::Fail) : EAssemblyQuality::NotEvaluated,
        !bUsePhysicalDynamics||PhysicalDoor.bValid,Status);
}
bool ADoorAssemblyCell::PrepareNextOperation()
{
    if(Operations.bActive||bDoorHeld||!(State==EDoorAssemblyState::Complete||State==EDoorAssemblyState::Idle))return false;
    if(!ValidateSetup())return false;
    PhysicalDoor=FDoorRigidBody();bSupportCommand=false;bDoorFastened=false;
    Outcome=EDoorCellOutcome::NotEvaluated;AlignmentCorrections=0;Accumulator=StateTime=StableTime=0;
    DoorSizeMeasuredAt=LastAlignmentSampleTime=-1;MeasuredDoorWidthCm=0;
    EnterState(EDoorAssemblyState::Idle,TEXT("다음 부품 요청 준비 · 로봇/부품 자세 유지"));return true;
}
bool ADoorAssemblyCell::StartOperation(const FAssemblyOperationRequest& Request)
{
    const bool Bound=RequestBinding.TaskId==GetOperationTaskId();
    const bool Matches=!Bound||(Request.ProductId==RequestBinding.ProductId&&Request.PartId==RequestBinding.PartId
        &&Request.EquipmentId==RequestBinding.EquipmentId&&Request.SourcePort==RequestBinding.SourcePort&&Request.TargetPort==RequestBinding.TargetPort);
    if(State!=EDoorAssemblyState::Idle||!Matches||Request.TaskId!=GetOperationTaskId()||!Operations.Accept(Request))
    {Status=TEXT("작업 요청 거부: 대상/세대/순서/중복/점유 확인");return false;}
    return StartCycle();
}

bool ADoorAssemblyCell::StartCycle()
{
    if (State!=EDoorAssemblyState::Idle) { Status=TEXT("Reset to Idle before starting a new cycle."); return false; }
    if(!Operations.bActive)
    {
        auto R=RequestBinding.TaskId==GetOperationTaskId()?RequestBinding:FAssemblyOperationRequest();R.Id={Operations.Epoch,Operations.LastSequence+1};R.TaskId=GetOperationTaskId();
        if(R.ProductId.IsEmpty())R.ProductId=Assembly?Assembly->GetPathName():TEXT("standalone");
        if(R.PartId.IsEmpty())R.PartId=TEXT("part_")+R.TaskId;
        if(R.EquipmentId.IsEmpty())R.EquipmentId=GetPathName();
        if(R.SourcePort.IsEmpty())R.SourcePort=TEXT("supply_")+R.TaskId;
        if(R.TargetPort.IsEmpty())R.TargetPort=R.TaskId;
        return StartOperation(R);
    }
    if(bUsePhysicalDynamics&&!PhysicsSettings.Valid()){PhysicalDoor.bValid=false;Fail(TEXT("도어 물성 무효"));return false;}
    if (!ValidateSetup()) { Fail(Status); return false; }
    Robot->bTorqueDynamics=bUsePhysicalDynamics;
	SimulationTimeSeconds=0; RecordTime=0; Accumulator=0;
	if (bRecordCSV)
	{
		LogFile=FPaths::Combine(FPaths::ProjectSavedDir(),TEXT("DoorAssemblyRuns"),FGuid::NewGuid().ToString(EGuidFormats::Digits)+TEXT(".csv"));
		CSVBuffer=TEXT("time_s,state,held,fastened,tcp_x_cm,tcp_y_cm,tcp_z_cm,target_x_cm,target_y_cm,target_z_cm,position_error_mm,orientation_error_deg,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,door_mount_error_mm,align_sensor_error_mm,verify_sensor_error_mm,quality_valid,quality_pass,outcome");
		for (int32 I=1;I<=6;++I) CSVBuffer+=FString::Printf(TEXT(",p%d_gap_mm,p%d_flush_mm,p%d_pass"),I,I,I);
		CSVBuffer+=TEXT("\n");
	}
	PickupAtStart=Pose->GetControlledDoor()->GetComponentTransform();
	if(!ScanDoorSize())
	{Fail(FString::Printf(TEXT("차문 규격 검사 실패 · 레이저 폭 %.1fcm / 기준 %.1fcm ± 1.0cm"),MeasuredDoorWidthCm,NominalDoorWidthCm));return false;}
	WriteRunMetadata();
	if (!CheckPickup()) { Fail(Status); return false; }
	if (!MeasureSensors()) { Fail(TEXT("Invalid alignment sensor input.")); return false; }
	return EnterMove(EDoorAssemblyState::ApproachPickup,OffsetDoor(PickupAtStart,ApproachOffsetBodyCm));
}

void ADoorAssemblyCell::StopCycle()
{
	if (State==EDoorAssemblyState::Idle || State==EDoorAssemblyState::Complete || State==EDoorAssemblyState::Fault || State==EDoorAssemblyState::Stopped) return;
	Robot->Halt(); Outcome=EDoorCellOutcome::Stopped; EnterState(EDoorAssemblyState::Stopped,TEXT("Cycle stopped; grasp/fastening state retained. Reset to recover."));
    FinishOperation(EFactoryCommandStatus::Cancelled);
	Accumulator=0; RecordSample(); FlushRecording();
}

void ADoorAssemblyCell::Fail(const FString& Reason)
{
	// Reason can refer to Robot->LastStatus, which Halt changes.
	const FString FailureReason=Reason;
	if (Robot) Robot->Halt();
	Outcome=EDoorCellOutcome::Blocked;
	EnterState(EDoorAssemblyState::Fault,FailureReason); FinishOperation(EFactoryCommandStatus::Failed); RecordSample(); FlushRecording();
}

void ADoorAssemblyCell::EnterState(EDoorAssemblyState Next,const FString& Message)
{ State=Next; StateTime=0; StableTime=0; Status=Message; }

bool ADoorAssemblyCell::EnterMove(EDoorAssemblyState Next,const FTransform& DoorTarget)
{
	TargetDoorWorld=DoorTarget;
	TargetTCPWorld=(bDoorHeld && !bDoorFastened ? DoorInTCP.Inverse() : TCPInDoorCm)*DoorTarget;
    Robot->SetMotionPart(Pose.IsValid()?Pose->GetControlledDoor():nullptr,bDoorHeld&&!bDoorFastened);
    const bool Linear=Next==EDoorAssemblyState::Align||Next==EDoorAssemblyState::Retract
        ||(Next==EDoorAssemblyState::ApproachMount&&State==EDoorAssemblyState::Align);
    if(Next==EDoorAssemblyState::Retract)TargetTCPWorld.SetRotation(Robot->GetTCPWorld().GetRotation());
	if (!(Linear?Robot->CommandLinearTCPWorld(TargetTCPWorld):Robot->CommandTCPWorld(TargetTCPWorld)))
    {
        Fail(FString::Printf(TEXT("%s %s · 목표 %s / 로봇 %s · %s"),
            TargetModule ? *TargetModule->ModuleId.ToString() : (bRightDoor ? TEXT("Door_FR") : TEXT("Door_FL")),
            *StaticEnum<EDoorAssemblyState>()->GetNameStringByValue(int64(Next)), *TargetTCPWorld.GetLocation().ToString(),
            *Robot->GetActorLocation().ToString(), *Robot->LastStatus));
        return false;
    }
	EnterState(Next,TEXT("Tracking IK joint target with PID.")); MeasureTCP(); RecordSample(); return true;
}

void ADoorAssemblyCell::MeasureTCP()
{
	if (!Robot) return;
	const FTransform Current=Robot->GetTCPWorld();
	PositionErrorMm=FVector::Distance(Current.GetLocation(),TargetTCPWorld.GetLocation())*10;
	OrientationErrorDeg=FMath::RadiansToDegrees(Current.GetRotation().AngularDistance(TargetTCPWorld.GetRotation()));
}

bool ADoorAssemblyCell::AdvanceSimulation(double Delta)
{
	if (!FMath::IsFinite(Delta) || Delta<0 || Delta>1 || !FMath::IsFinite(FixedStepSeconds) || FixedStepSeconds<=0 || FixedStepSeconds>0.02) return false;
	if (!bUsePhysicalDynamics && bDoorFastened && Pose.IsValid() && IsValid(Assembly) && Assembly->GetBodyFrame())
		Pose->SetDoorWorldPose(FastenedInBody*Assembly->GetBodyFrame()->GetComponentTransform());
	if (Pose.IsValid()) { RefreshQuality(); MeasureSensors(); }
    if(State==EDoorAssemblyState::Fault||State==EDoorAssemblyState::Stopped||State==EDoorAssemblyState::Complete)
    {
        if(bUsePhysicalDynamics&&PhysicalDoor.bActive)
        {
            Accumulator+=Delta;
            while(Accumulator>=FixedStepSeconds){Accumulator-=FixedStepSeconds;SimulationTimeSeconds+=FixedStepSeconds;
                if(Robot)Robot->StepControl(FixedStepSeconds);if(!StepDoorPhysics(FixedStepSeconds))return false;}
            RefreshQuality();MeasureSensors();
        }
        return State!=EDoorAssemblyState::Fault;
    }
    if (State==EDoorAssemblyState::Idle) return true;
	Accumulator+=Delta;
	while (Accumulator+1.e-10>=FixedStepSeconds)
	{
		Accumulator-=FixedStepSeconds; FixedUpdate(FixedStepSeconds);
		if (State==EDoorAssemblyState::Fault || State==EDoorAssemblyState::Stopped || State==EDoorAssemblyState::Complete) { Accumulator=0; break; }
	}
	return State!=EDoorAssemblyState::Fault;
}

void ADoorAssemblyCell::FixedUpdate(double Dt)
{
	SimulationTimeSeconds+=Dt;
	if (!IsValid(Robot) || !Pose.IsValid() || !IsValid(Assembly)) { Fail(TEXT("Lost robot or door reference.")); return; }
    const bool WasPlanning=Robot->IsMotionPlanning();
    if(!WasPlanning)StateTime+=Dt;
	if (!Assembly->GetBodyFrame()->GetComponentTransform().Equals(BodyAtStart,1.e-6)
		|| !Robot->GetActorTransform().Equals(RobotBaseAtStart,1.e-6))
	{ Fail(TEXT("Body or robot base moved during active cycle. Stop and recalibrate targets.")); return; }
	if (bDoorHeld && !bDoorFastened && !Pose->GetControlledDoor()->GetComponentTransform().Equals(LastCarriedDoorWorld,1.e-5))
	{ Fail(TEXT("Door moved externally while held. Stop and revalidate grasp.")); return; }
	if (!bDoorHeld && !bDoorFastened && (State==EDoorAssemblyState::ApproachPickup || State==EDoorAssemblyState::Grip)
		&& !Pose->GetControlledDoor()->GetComponentTransform().Equals(PickupAtStart,1.e-5))
	{ Fail(TEXT("Door placement changed during pickup. Reset and re-detect.")); return; }
	if (!Robot->StepControl(Dt)) { Fail(Robot->LastStatus); return; }
    if(bUsePhysicalDynamics&&PhysicalDoor.bActive)
    {if(!StepDoorPhysics(Dt)){Fail(TEXT("도어 물리 적분 무효"));return;}if(State==EDoorAssemblyState::Fault)return;}
    if (!bUsePhysicalDynamics && bDoorHeld && !bDoorFastened && !Pose->SetDoorWorldPose(DoorInTCP*Robot->GetTCPWorld()))
	{ Fail(TEXT("Door transport pose rejected.")); return; }
	if (!bUsePhysicalDynamics && bDoorFastened) Pose->SetDoorWorldPose(FastenedInBody*Assembly->GetBodyFrame()->GetComponentTransform());
	LastCarriedDoorWorld=Pose->GetControlledDoor()->GetComponentTransform();
	if (!RefreshQuality() || !MeasureSensors()) { Fail(TEXT("Invalid quality geometry or alignment sensor.")); return; }
	MeasureTCP();
    if(State==EDoorAssemblyState::Align&&bDoorHeld
        &&FMath::RadiansToDegrees(LastCarriedDoorWorld.GetRotation().AngularDistance(TargetDoorWorld.GetRotation()))>OrientationToleranceDeg)
    {Fail(TEXT("삽입 중 도어 각도 이탈 · 체결 차단"));return;}
    if(Robot->IsMotionPlanning())
    {StableTime=0;Status=GetStateLabel()+TEXT(" · ")+Robot->LastStatus;return;}
    if(WasPlanning)Status=GetStateLabel()+TEXT(" · ")+Robot->LastStatus;
	const bool Settled=PositionErrorMm<=PositionToleranceMm && OrientationErrorDeg<=OrientationToleranceDeg && Robot->IsAtJointTarget();
	StableTime=Settled ? StableTime+Dt : 0;
	if (StateTime>StageTimeoutSeconds) { Fail(TEXT("Stage timed out before measured motion settled.")); return; }
	RecordTime+=Dt;
	if (RecordTime>=0.05) { RecordSample(); RecordTime=0; }
    if (State==EDoorAssemblyState::Fasten)
    {
        if(bUsePhysicalDynamics)
        {
            if(!PhysicalDoor.bSupportContact){Fail(TEXT("수신 지지 미접촉 · 해제 차단"));return;}
            if(PhysicalDoor.VelocityMps.Size()<0.002&&PhysicalDoor.SupportDeflectionMm<=2)
            {bDoorHeld=false;EnterState(EDoorAssemblyState::Release,TEXT("받침 하중 인계 · 해제 후 안정 확인"));}
            return;
        }
		if (!Settled) { Fail(TEXT("Lost alignment while fastening.")); return; }
		if (!AlignmentAccepted()) { Fail(TEXT("Alignment sensor lost alignment while fastening.")); return; }
		if (!VerifyBeforeFasten()) return;
		if (StateTime>=FasteningSeconds)
		{
			FastenedInBody=Pose->GetControlledDoor()->GetComponentTransform().GetRelativeTransform(Assembly->GetBodyFrame()->GetComponentTransform());
			bDoorFastened=true; EnterState(EDoorAssemblyState::Release,TEXT("Ideal rigid fastening completed; retaining achieved pose."));
		}
		return;
	}
    if (State==EDoorAssemblyState::Release)
    {
        if(bUsePhysicalDynamics)
        {
            if(!PhysicalDoor.bSupportContact||PhysicalDoor.SupportDeflectionMm>10){Fail(TEXT("해제 후 지지 실패"));return;}
            if(PhysicalDoor.VelocityMps.Size()>=0.002||PhysicalDoor.SupportDeflectionMm>2||StateTime<0.3)return;
            bDoorFastened=true; // Compatibility flag: supported placement, NOT bolt quality.
        }
        bDoorHeld=false;
		EnterMove(EDoorAssemblyState::Retract,OffsetDoor(GetMountDoorWorld(),ApproachOffsetBodyCm)); return;
	}
	if (StableTime<SettleSeconds) return;
	switch (State)
	{
	case EDoorAssemblyState::ApproachPickup:
		EnterMove(EDoorAssemblyState::Grip,PickupAtStart); break;
	case EDoorAssemblyState::Grip:
		if (FVector::Distance((TCPInDoorCm*Pose->GetControlledDoor()->GetComponentTransform()).GetLocation(),Robot->GetTCPWorld().GetLocation())*10>PositionToleranceMm
			|| FMath::RadiansToDegrees((TCPInDoorCm*PickupAtStart).GetRotation().AngularDistance(Robot->GetTCPWorld().GetRotation()))>OrientationToleranceDeg)
		{ Fail(TEXT("Grasp contact does not match the detected door pose.")); break; }
		DoorInTCP=Pose->GetControlledDoor()->GetComponentTransform().GetRelativeTransform(Robot->GetTCPWorld());
		LastCarriedDoorWorld=Pose->GetControlledDoor()->GetComponentTransform();
        if(bUsePhysicalDynamics)
        {
            FBox B(ForceInit);TArray<USceneComponent*> MassComponents;Pose->GetControlledDoor()->GetChildrenComponents(true,MassComponents);
            for(auto* C:MassComponents)if(auto* M=Cast<UStaticMeshComponent>(C))if(M->GetStaticMesh()&&!M->bHiddenInGame)B+=M->Bounds.GetBox();
            const auto DoorPose=Pose->GetControlledDoor()->GetComponentTransform();
            BodyInDoor=FTransform(DoorPose.InverseTransformPosition(B.IsValid?B.GetCenter():DoorPose.GetLocation()));
            PhysicalDoor=FDoorRigidBody();PhysicalDoor.Pose=BodyInDoor*DoorPose;PhysicalDoor.bActive=true;
        }
        bDoorHeld=true; EnterMove(EDoorAssemblyState::Lift,OffsetDoor(PickupAtStart,LiftOffsetBodyCm)); break;
	case EDoorAssemblyState::Lift:
		EnterMove(EDoorAssemblyState::Transfer,OffsetDoor(GetMountDoorWorld(),TransferOffsetBodyCm)); break;
	case EDoorAssemblyState::Transfer:
		EnterMove(EDoorAssemblyState::ApproachMount,OffsetDoor(GetMountDoorWorld(),ApproachOffsetBodyCm)); break;
	case EDoorAssemblyState::ApproachMount:
    {
        // All translation/attitude corrections happen outside the mating plane.
        // Only the axial translation remains when entering Align.
        const FTransform Outside=OffsetDoor(GetMountDoorWorld(),ApproachOffsetBodyCm);
        const FVector Residual=Outside.GetLocation()-AlignmentMeasuredDoorWorld.GetLocation();
        const double Angle=FMath::RadiansToDegrees(Outside.GetRotation().AngularDistance(AlignmentMeasuredDoorWorld.GetRotation()));
        if(!bLastAlignmentSampleValid){Fail(TEXT("삽입 전 정렬 센서 무효"));break;}
        if(Residual.Size()*10>PositionToleranceMm||Angle>OrientationToleranceDeg)
        {
            if(++AlignmentCorrections>8){Fail(TEXT("장착 바깥 사전 정렬 실패"));break;}
            FTransform Corrected=TargetDoorWorld;Corrected.AddToTranslation(Residual);
            Corrected.SetRotation((Outside.GetRotation()*AlignmentMeasuredDoorWorld.GetRotation().Inverse()*Corrected.GetRotation()).GetNormalized());
            EnterMove(EDoorAssemblyState::ApproachMount,Corrected);break;
        }
        FTransform Insert=TargetDoorWorld;
        Insert.AddToTranslation(-BodyAtStart.TransformVectorNoScale(ApproachOffsetBodyCm));
        EnterMove(EDoorAssemblyState::Align,Insert);break;
    }
	case EDoorAssemblyState::Align:
	{
		if (!AlignmentAccepted())
		{
			if (++AlignmentCorrections>8) { Fail(TEXT("Sensor-driven alignment did not converge within correction limit.")); break; }
            // Withdraw along the same axis before correcting; never rotate or
            // slide a partially inserted panel through its mating surfaces.
            FTransform Outside=TargetDoorWorld;
            Outside.AddToTranslation(BodyAtStart.TransformVectorNoScale(ApproachOffsetBodyCm));
            EnterMove(EDoorAssemblyState::ApproachMount,Outside);break;
		}
		if (!VerifyBeforeFasten()) break;
        bSupportCommand=bUsePhysicalDynamics;
        EnterState(EDoorAssemblyState::Fasten,bUsePhysicalDynamics?TEXT("받침 닫힘 명령 · 실제 지지 확인"):TEXT("Measured alignment accepted; ideal fastening in progress.")); break;
	}
	case EDoorAssemblyState::Retract:
		RefreshQuality(); Outcome=bQualityPassed ? EDoorCellOutcome::Pass : EDoorCellOutcome::QualityFail;
		EnterState(EDoorAssemblyState::Complete,bQualityPassed
			? (IsValid(TargetModule) ? TEXT("Complete: independent pose check PASS; module door retained and robot retracted.") : TEXT("Complete: six-station quality PASS; door retained and robot retracted."))
			: TEXT("Complete: assembly QUALITY FAIL; inspect measured door pose."));
    FinishOperation(EFactoryCommandStatus::Completed);
		Assembly->RefreshDoorInspection(); RecordSample(); FlushRecording(); break;
	default: break;
	}
}

void ADoorAssemblyCell::RecordSample()
{
	if (!bRecordCSV || CSVBuffer.IsEmpty() || !Robot || !Pose.IsValid()) return;
	const FVector P=Robot->GetTCPWorld().GetLocation(), T=TargetTCPWorld.GetLocation();
	CSVBuffer+=FString::Printf(TEXT("%.6f,%s,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f"),
		SimulationTimeSeconds,*GetStateLabel(),bDoorHeld,bDoorFastened,P.X,P.Y,P.Z,T.X,T.Y,T.Z,PositionErrorMm,OrientationErrorDeg);
	for (double Q:Robot->CurrentAnglesDegrees) CSVBuffer+=FString::Printf(TEXT(",%.6f"),Q);
	CSVBuffer+=FString::Printf(TEXT(",%.6f,%.6f,%.6f,%d,%d,%s"),FVector::Distance(Pose->GetControlledDoor()->GetComponentLocation(),GetMountDoorWorld().GetLocation())*10,
		AlignmentPositionErrorMm,VerificationPositionErrorMm,bQualityValid,bQualityPassed,*StaticEnum<EDoorCellOutcome>()->GetNameStringByValue(int64(Outcome)));
	for (int32 I=0;I<6;++I)
	{
		const FDoorCellStationQuality Q=StationQuality.IsValidIndex(I) ? StationQuality[I] : FDoorCellStationQuality();
		CSVBuffer+=FString::Printf(TEXT(",%.6f,%.6f,%d"),Q.GapChangeMm,Q.FlushChangeMm,Q.bPassed);
	}
	CSVBuffer+=TEXT("\n");
}

void ADoorAssemblyCell::FlushRecording()
{
	if (CSVBuffer.IsEmpty() || LogFile.IsEmpty()) return;
	WriteRunMetadata();
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(LogFile),true);
	if (!FFileHelper::SaveStringToFile(CSVBuffer,*LogFile,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		UE_LOG(LogTemp,Warning,TEXT("Could not save assembly log: %s"),*LogFile);
	CSVBuffer.Reset();
}

void ADoorAssemblyCell::BeginPlay()
{
	Super::BeginPlay();
	if (bExternallyDriven) return;
	const bool Preserve=bPreservePlacedDoorOnBeginPlay && (Scenario==EDoorCellScenario::Normal || Scenario==EDoorCellScenario::Custom)
		&& IsValid(Assembly) && Assembly->GetLeftDoor();
	const FTransform Placed=Preserve ? Assembly->GetLeftDoor()->GetComponentTransform() : FTransform::Identity;
	if (!ResetCycle()) return;
	if (Preserve) { Pose->SetDoorWorldPose(Placed); RefreshQuality(); MeasureSensors(); }
	if (bAutoStart) StartCycle();
}
void ADoorAssemblyCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bExternallyDriven) return;
	if (DeltaSeconds>1) { if (Robot) StopCycle(); Status=TEXT("Large frame stall: stopped; reset required."); return; }
	AdvanceSimulation(DeltaSeconds);
}
void ADoorAssemblyCell::EndPlay(const EEndPlayReason::Type Reason)
{ FlushRecording(); Super::EndPlay(Reason); }

bool ADoorAssemblyCell::SelectScenario(EDoorCellScenario Preset, EDoorCellControlMode Mode)
{
	if (State!=EDoorAssemblyState::Idle && State!=EDoorAssemblyState::Complete
		&& State!=EDoorAssemblyState::Fault && State!=EDoorAssemblyState::Stopped) return false;
	Scenario=Preset; ControlMode=Mode;
	InitialDoorOffsetMm=FVector::ZeroVector; InitialDoorRotationDeg=FRotator::ZeroRotator;
	AlignmentSensorBiasMm=FVector::ZeroVector; AlignmentSensorRotationBiasDeg=FRotator::ZeroRotator;
	VerificationSensorBiasMm=FVector::ZeroVector; VerificationSensorRotationBiasDeg=FRotator::ZeroRotator;
	bAlignmentSensorValid=true; bVerificationSensorValid=true; bDoorPresent=true;
	switch (Preset)
	{
	case EDoorCellScenario::SensorBias: AlignmentSensorBiasMm=FVector(0,5,0); break;
	case EDoorCellScenario::DisplacedDoor: InitialDoorOffsetMm=FVector(20,-30,10); InitialDoorRotationDeg=FRotator(0,2,0); break;
	case EDoorCellScenario::UnreachableDoor: InitialDoorOffsetMm=FVector(0,-100000,0); break;
	case EDoorCellScenario::ReversedDoor: InitialDoorRotationDeg=FRotator(0,0,180); break;
	case EDoorCellScenario::MissingDoor: bDoorPresent=false; break;
	default: break;
	}
	return ResetCycle();
}

bool ADoorAssemblyCell::CheckPickup()
{
	if (!bDoorPresent) { Status=TEXT("Pickup blocked: door is missing."); return false; }
	if (!PickupAtStart.IsValid() || !PickupAtStart.GetScale3D().Equals(FVector::OneVector,1.e-6))
	{ Status=TEXT("Pickup blocked: invalid or scaled door pose."); return false; }
	const FTransform Expected=GetPickupDoorWorld();
	if (FVector::Distance(PickupAtStart.GetLocation(),Expected.GetLocation())*10>MaxPickupDisplacementMm)
	{ Status=TEXT("Pickup blocked: door outside configured recovery window; reposition it."); return false; }
	if (FMath::RadiansToDegrees(PickupAtStart.GetRotation().AngularDistance(Expected.GetRotation()))>MaxPickupRotationDeg)
	{ Status=TEXT("Pickup blocked: door reversed or outside assumed gripper orientation window."); return false; }
	// Preflight uses immutable FK/IK inputs and never changes servo angles or targets.
	TArray<double> Seed=Robot->GetAnglesRadians();
	const TArray<FTransform> Targets={OffsetDoor(PickupAtStart,ApproachOffsetBodyCm),PickupAtStart,OffsetDoor(PickupAtStart,LiftOffsetBodyCm)};
	FRobotIKSettings IK; IK.PositionToleranceCm=0.03; IK.OrientationToleranceRadians=0.001;
	for (const auto& Door:Targets)
	{
		const auto Result=FRobotKinematics::SolveInverseKinematics(Robot->GetDefinitions(),Robot->ToolInLastJointCm,
			(TCPInDoorCm*Door).GetRelativeTransform(Robot->GetActorTransform()),Seed,IK);
		if (!Result.IsSuccess()) { Status=TEXT("Pickup blocked: IK cannot reach approach, grasp or lift pose."); return false; }
		Seed=Result.JointRadians;
	}
	return true;
}

FTransform ADoorAssemblyCell::SimulatePoseSensor(const FVector& BiasMm,const FRotator& BiasDeg) const
{
	const FTransform Body=Assembly->GetBodyFrame()->GetComponentTransform();
	FTransform Measured=Pose->GetControlledDoor()->GetComponentTransform().GetRelativeTransform(Body);
	Measured.AddToTranslation(BiasMm/10);
	Measured.SetRotation((BiasDeg.Quaternion()*Measured.GetRotation()).GetNormalized());
	return Measured*Body;
}

bool ADoorAssemblyCell::MeasureSensors()
{
	bLastAlignmentSampleValid=false;
	if (!Pose.IsValid() || !IsValid(Assembly) || !bAlignmentSensorValid) return false;
	AlignmentMeasuredDoorWorld=SimulatePoseSensor(AlignmentSensorBiasMm,AlignmentSensorRotationBiasDeg);
	VerificationMeasuredDoorWorld=SimulatePoseSensor(VerificationSensorBiasMm,VerificationSensorRotationBiasDeg);
	const FTransform Mount=GetMountDoorWorld();
	AlignmentPositionErrorMm=FVector::Distance(AlignmentMeasuredDoorWorld.GetLocation(),Mount.GetLocation())*10;
	AlignmentOrientationErrorDeg=FMath::RadiansToDegrees(AlignmentMeasuredDoorWorld.GetRotation().AngularDistance(Mount.GetRotation()));
	VerificationPositionErrorMm=FVector::Distance(VerificationMeasuredDoorWorld.GetLocation(),Mount.GetLocation())*10;
	LastAlignmentSampleTime=SimulationTimeSeconds;
	bLastAlignmentSampleValid=AlignmentMeasuredDoorWorld.IsValid() && VerificationMeasuredDoorWorld.IsValid();
	return bLastAlignmentSampleValid;
}

bool ADoorAssemblyCell::AlignmentAccepted() const
{
	return bAlignmentSensorValid && AlignmentPositionErrorMm<=PositionToleranceMm && AlignmentOrientationErrorDeg<=OrientationToleranceDeg;
}

bool ADoorAssemblyCell::VerifyBeforeFasten()
{
	if (ControlMode==EDoorCellControlMode::Basic) return true;
	if (!bVerificationSensorValid) { Fail(TEXT("Independent verification sensor is invalid; fastening blocked.")); return false; }
	TArray<FDoorInspectionResult> Measured;
	if (!IsValid(TargetModule) && (!Inspection.IsValid() || !Inspection->EvaluateDoorPose(VerificationMeasuredDoorWorld,Measured) || Measured.Num()!=6))
	{ Fail(TEXT("Independent six-station verification unavailable; fastening blocked.")); return false; }
	for (const auto& Q:Measured)
		if (FMath::Abs(Q.GapChangeMm)>GapToleranceMm || FMath::Abs(Q.FlushChangeMm)>FlushToleranceMm)
		{ Fail(FString::Printf(TEXT("Independent verification blocked fastening at %s: gap %+.2f, flush %+.2f mm."),*Q.Name.ToString(),Q.GapChangeMm,Q.FlushChangeMm)); return false; }
	if (VerificationPositionErrorMm>PositionToleranceMm
		|| FMath::RadiansToDegrees(VerificationMeasuredDoorWorld.GetRotation().AngularDistance(GetMountDoorWorld().GetRotation()))>OrientationToleranceDeg)
	{ Fail(TEXT("Independent verification rejected measured door alignment.")); return false; }
	return true;
}

bool ADoorAssemblyCell::RefreshQuality()
{
	StationQuality.Reset(); bQualityPassed=false; bQualityValid=false;
	if (IsValid(TargetModule))
	{
		if (!Pose.IsValid() || !Pose->GetControlledDoor() || !bDoorPresent) return false;
		const FTransform Actual=Pose->GetControlledDoor()->GetComponentTransform();
		const FTransform Mount=GetMountDoorWorld();
		bQualityValid=Actual.IsValid() && Mount.IsValid();
		bQualityPassed=bQualityValid && FVector::Distance(Actual.GetLocation(),Mount.GetLocation())*10<=PositionToleranceMm
			&& FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Mount.GetRotation()))<=OrientationToleranceDeg;
		return bQualityValid;
	}
	if (!Inspection.IsValid() || !Inspection->RefreshMeasurements() || Inspection->Measurements.Num()!=6 || !bDoorPresent) return false;
	bQualityValid=true; bQualityPassed=true;
	for (const auto& R:Inspection->Measurements)
	{
		FDoorCellStationQuality Q; Q.Name=R.Name; Q.GapChangeMm=R.GapChangeMm; Q.FlushChangeMm=R.FlushChangeMm;
		Q.bPassed=FMath::Abs(Q.GapChangeMm)<=GapToleranceMm && FMath::Abs(Q.FlushChangeMm)<=FlushToleranceMm;
		bQualityPassed&=Q.bPassed; StationQuality.Add(Q);
	}
	return true;
}

FString ADoorAssemblyCell::GetQualitySummary() const
{
	if (IsValid(TargetModule))
		return FString::Printf(TEXT("Module %s / %s\nOutcome: %s\nAlignment sensor %s: %.2f mm / %.2f deg\nIndependent pose sensor %s: %.2f mm\nMount pose check: %s (limits %.2f mm / %.2f deg)"),
			*TargetModule->ModuleId.ToString(),*StaticEnum<EDoorCellControlMode>()->GetNameStringByValue(int64(ControlMode)),
			*StaticEnum<EDoorCellOutcome>()->GetNameStringByValue(int64(Outcome)),bAlignmentSensorValid?TEXT("valid"):TEXT("INVALID"),
			AlignmentPositionErrorMm,AlignmentOrientationErrorDeg,bVerificationSensorValid?TEXT("valid"):TEXT("INVALID"),
			VerificationPositionErrorMm,bQualityValid?(bQualityPassed?TEXT("PASS"):TEXT("FAIL")):TEXT("INVALID"),
			PositionToleranceMm,OrientationToleranceDeg);
	FString Text=FString::Printf(TEXT("Mode: %s / %s\nOutcome: %s\nAlignment sensor %s: %.2f mm / %.2f deg\nIndependent sensor %s: %.2f mm\nSix-station gauge: %s (limits %.2f / %.2f mm)"),
		*StaticEnum<EDoorCellControlMode>()->GetNameStringByValue(int64(ControlMode)),
		*StaticEnum<EDoorCellScenario>()->GetNameStringByValue(int64(Scenario)),
		*StaticEnum<EDoorCellOutcome>()->GetNameStringByValue(int64(Outcome)),bAlignmentSensorValid ? TEXT("valid") : TEXT("INVALID"),AlignmentPositionErrorMm,AlignmentOrientationErrorDeg,
		bVerificationSensorValid ? TEXT("valid") : TEXT("INVALID"),VerificationPositionErrorMm,bQualityValid ? (bQualityPassed ? TEXT("PASS") : TEXT("FAIL")) : TEXT("INVALID"),GapToleranceMm,FlushToleranceMm);
	for (const auto& Q:StationQuality) Text+=FString::Printf(TEXT("\n%s  dG:%+.2f  dF:%+.2f  %s"),*Q.Name.ToString(),Q.GapChangeMm,Q.FlushChangeMm,Q.bPassed ? TEXT("PASS") : TEXT("FAIL"));
	return Text;
}

void ADoorAssemblyCell::WriteRunMetadata()
{
	if (!bRecordCSV || LogFile.IsEmpty() || !IsValid(Robot)) return;
	RunMetadataFile=FPaths::ChangeExtension(LogFile,TEXT("json"));
	TSharedRef<FJsonObject> J=MakeShared<FJsonObject>();
	J->SetStringField(TEXT("schema"),TEXT("door-cell-experiment-v1"));
	J->SetStringField(TEXT("scenario"),StaticEnum<EDoorCellScenario>()->GetNameStringByValue(int64(Scenario)));
	J->SetStringField(TEXT("mode"),StaticEnum<EDoorCellControlMode>()->GetNameStringByValue(int64(ControlMode)));
	J->SetStringField(TEXT("state"),GetStateLabel()); J->SetStringField(TEXT("status"),Status);
	J->SetStringField(TEXT("outcome"),StaticEnum<EDoorCellOutcome>()->GetNameStringByValue(int64(Outcome)));
	J->SetStringField(TEXT("model"),IsValid(TargetModule)
		? TEXT("Process-module door; virtual pose sensors with body-frame biases; mount-pose verification; ideal rigid grasp/fastening. No collision, load or hinge mechanics or physical gap gauges.")
		: TEXT("Perfect virtual pickup detection; pose sensors with body-frame constant biases; signed virtual gap/flush gauges; ideal rigid grasp/fastening. No collision, load or hinge mechanics."));
	J->SetStringField(TEXT("robot_calibration"),Robot->CalibrationSource);
	const auto VectorField=[&](const TCHAR* Name,const FVector& V)
	{ TArray<TSharedPtr<FJsonValue>> A; A.Add(MakeShared<FJsonValueNumber>(V.X)); A.Add(MakeShared<FJsonValueNumber>(V.Y)); A.Add(MakeShared<FJsonValueNumber>(V.Z)); J->SetArrayField(Name,A); };
	VectorField(TEXT("alignment_sensor_bias_mm"),AlignmentSensorBiasMm);
	VectorField(TEXT("alignment_sensor_rotation_pitch_yaw_roll_deg"),FVector(AlignmentSensorRotationBiasDeg.Pitch,AlignmentSensorRotationBiasDeg.Yaw,AlignmentSensorRotationBiasDeg.Roll));
	VectorField(TEXT("verification_sensor_bias_mm"),VerificationSensorBiasMm);
	VectorField(TEXT("verification_sensor_rotation_pitch_yaw_roll_deg"),FVector(VerificationSensorRotationBiasDeg.Pitch,VerificationSensorRotationBiasDeg.Yaw,VerificationSensorRotationBiasDeg.Roll));
	VectorField(TEXT("initial_offset_mm"),InitialDoorOffsetMm); VectorField(TEXT("pickup_world_cm"),PickupAtStart.GetLocation());
	VectorField(TEXT("pickup_rotation_pitch_yaw_roll_deg"),FVector(PickupAtStart.Rotator().Pitch,PickupAtStart.Rotator().Yaw,PickupAtStart.Rotator().Roll));
	J->SetBoolField(TEXT("door_present"),bDoorPresent); J->SetBoolField(TEXT("quality_valid"),bQualityValid); J->SetBoolField(TEXT("quality_pass"),bQualityPassed);
	J->SetNumberField(TEXT("gap_tolerance_mm"),GapToleranceMm); J->SetNumberField(TEXT("flush_tolerance_mm"),FlushToleranceMm);
	J->SetNumberField(TEXT("position_tolerance_mm"),PositionToleranceMm); J->SetNumberField(TEXT("orientation_tolerance_deg"),OrientationToleranceDeg);
	J->SetNumberField(TEXT("fixed_step_s"),FixedStepSeconds); J->SetNumberField(TEXT("simulation_time_s"),SimulationTimeSeconds);
	J->SetNumberField(TEXT("kp"),Robot->Kp); J->SetNumberField(TEXT("ki"),Robot->Ki); J->SetNumberField(TEXT("kd"),Robot->Kd);
	TArray<TSharedPtr<FJsonValue>> Stations;
	for (const auto& Q:StationQuality)
	{ auto S=MakeShared<FJsonObject>(); S->SetStringField(TEXT("name"),Q.Name.ToString()); S->SetNumberField(TEXT("gap_change_mm"),Q.GapChangeMm); S->SetNumberField(TEXT("flush_change_mm"),Q.FlushChangeMm); S->SetBoolField(TEXT("pass"),Q.bPassed); Stations.Add(MakeShared<FJsonValueObject>(S)); }
	J->SetArrayField(TEXT("stations"),Stations);
	FString Json; FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Json));
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(RunMetadataFile),true);
	FFileHelper::SaveStringToFile(Json,*RunMetadataFile,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}


bool ADoorAssemblyCell::StepDoorPhysics(double Dt)
{
    if(!Pose.IsValid()||!IsValid(Robot))return false;
    const auto Goal=BodyInDoor*DoorInTCP*Robot->GetTCPWorld();
    const FVector Before=PhysicalDoor.Pose.GetLocation();
    if(!PhysicalDoor.Step(PhysicsSettings,Goal,BodyInDoor*GetMountDoorWorld(),bDoorHeld,bSupportCommand,Dt))return false;
    Robot->ToolReactionForceN=-PhysicalDoor.GripForceN;Robot->ToolReactionTorqueNm=-PhysicalDoor.GripTorqueNm;
    Robot->ForcePointWorldCm=PhysicalDoor.Pose.GetLocation();
    if(!bDoorHeld&&!PhysicalDoor.bSupportContact&&GetWorld())
    {
        FCollisionQueryParams Params(SCENE_QUERY_STAT(DoorDrop),false,this);Params.AddIgnoredActor(Assembly);Params.AddIgnoredActor(Robot);
        if(TargetModule)Params.AddIgnoredActor(TargetModule);
        FHitResult Hit;
        if(GetWorld()->SweepSingleByChannel(Hit,Before,PhysicalDoor.Pose.GetLocation(),FQuat::Identity,ECC_PhysicsBody,FCollisionShape::MakeSphere(20),Params))
        {
            PhysicalDoor.Pose.SetLocation(Hit.bStartPenetrating?Before+Hit.Normal*(Hit.PenetrationDepth+0.01):Hit.Location+Hit.ImpactNormal*0.01);
            PhysicalDoor.VelocityMps=ReducedPartPhysics::ContactVelocity(PhysicalDoor.VelocityMps,Hit.ImpactNormal);
        }
    }
    if(!Pose->SetDoorWorldPose(BodyInDoor.Inverse()*PhysicalDoor.Pose))return false;
    LastCarriedDoorWorld=Pose->GetControlledDoor()->GetComponentTransform();
    if(bDoorHeld&&PhysicalDoor.SlipMm>50)
    {bDoorHeld=false;Fail(TEXT("도어 파지 유지력 초과 · 이탈/낙하 · 후속 공정 차단"));}
    return true;
}

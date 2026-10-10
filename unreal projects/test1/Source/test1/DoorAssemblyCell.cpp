#include "DoorAssemblyCell.h"
#include "RobotArm.h"
#include "CamryAssembly.h"
#include "DoorPoseComponent.h"
#include "Components/SceneComponent.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

ADoorAssemblyCell::ADoorAssemblyCell()
{
	PrimaryActorTick.bCanEverTick=true;
	RootComponent=CreateDefaultSubobject<USceneComponent>(TEXT("CellRoot"));
}

FString ADoorAssemblyCell::GetStateLabel() const
{
	return StaticEnum<EDoorAssemblyState>()->GetNameStringByValue(int64(State));
}

bool ADoorAssemblyCell::ValidateSetup()
{
	if (!IsValid(Assembly) || !IsValid(Robot) || !Assembly->GetBodyFrame())
	{ Status=TEXT("Assign the vehicle assembly and native robot."); return false; }
	Pose=Assembly->GetDoorPoseController(ECamryDoorSide::Left);
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
	return true;
}

FTransform ADoorAssemblyCell::GetMountDoorWorld() const
{
	return IsValid(Assembly) && Assembly->GetBodyFrame()
		? MountInBody*Assembly->GetBodyFrame()->GetComponentTransform() : FTransform::Identity;
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
	if (Robot) Robot->Halt();
	FlushRecording();
	if (!ValidateSetup() || !Robot->InitializeArm())
	{ State=EDoorAssemblyState::Fault; if (Robot) Status+=TEXT(" ")+Robot->LastStatus; return false; }
	bDoorHeld=false; bDoorFastened=false;
	SimulationTimeSeconds=0; Accumulator=0; StateTime=0; StableTime=0;
	if (!Pose->SetDoorWorldPose(GetPickupDoorWorld())) { Fail(TEXT("Could not place door in the pickup station.")); return false; }
	TargetTCPWorld=Robot->GetTCPWorld(); MeasureTCP();
	EnterState(EDoorAssemblyState::Idle,TEXT("Ready. Enter/Start runs the cycle; Space stops; R resets."));
	return true;
}

bool ADoorAssemblyCell::StartCycle()
{
	if (State!=EDoorAssemblyState::Idle) { Status=TEXT("Reset to Idle before starting a new cycle."); return false; }
	if (!ValidateSetup()) { Fail(Status); return false; }
	SimulationTimeSeconds=0; RecordTime=0; Accumulator=0;
	if (bRecordCSV)
	{
		LogFile=FPaths::Combine(FPaths::ProjectSavedDir(),TEXT("DoorAssemblyRuns"),FGuid::NewGuid().ToString(EGuidFormats::Digits)+TEXT(".csv"));
		CSVBuffer=TEXT("time_s,state,held,fastened,tcp_x_cm,tcp_y_cm,tcp_z_cm,target_x_cm,target_y_cm,target_z_cm,position_error_mm,orientation_error_deg,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,door_mount_error_mm\n");
	}
	return EnterMove(EDoorAssemblyState::ApproachPickup,OffsetDoor(GetPickupDoorWorld(),ApproachOffsetBodyCm));
}

void ADoorAssemblyCell::StopCycle()
{
	if (State==EDoorAssemblyState::Idle || State==EDoorAssemblyState::Complete || State==EDoorAssemblyState::Fault || State==EDoorAssemblyState::Stopped) return;
	Robot->Halt(); EnterState(EDoorAssemblyState::Stopped,TEXT("Cycle stopped; grasp/fastening state retained. Reset to recover."));
	Accumulator=0; RecordSample(); FlushRecording();
}

void ADoorAssemblyCell::Fail(const FString& Reason)
{
	// Reason can refer to Robot->LastStatus, which Halt changes.
	const FString FailureReason=Reason;
	if (Robot) Robot->Halt();
	EnterState(EDoorAssemblyState::Fault,FailureReason); RecordSample(); FlushRecording();
}

void ADoorAssemblyCell::EnterState(EDoorAssemblyState Next,const FString& Message)
{ State=Next; StateTime=0; StableTime=0; Status=Message; }

bool ADoorAssemblyCell::EnterMove(EDoorAssemblyState Next,const FTransform& DoorTarget)
{
	TargetTCPWorld=(bDoorHeld && !bDoorFastened ? DoorInTCP.Inverse() : TCPInDoorCm)*DoorTarget;
	if (!Robot->CommandTCPWorld(TargetTCPWorld)) { Fail(Robot->LastStatus); return false; }
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
	if (bDoorFastened && Pose.IsValid() && IsValid(Assembly) && Assembly->GetBodyFrame())
		Pose->SetDoorWorldPose(FastenedInBody*Assembly->GetBodyFrame()->GetComponentTransform());
	if (State==EDoorAssemblyState::Fault) return false;
	if (State==EDoorAssemblyState::Idle || State==EDoorAssemblyState::Stopped || State==EDoorAssemblyState::Complete) return true;
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
	SimulationTimeSeconds+=Dt; StateTime+=Dt;
	if (!IsValid(Robot) || !Pose.IsValid() || !IsValid(Assembly)) { Fail(TEXT("Lost robot or door reference.")); return; }
	if (!Assembly->GetBodyFrame()->GetComponentTransform().Equals(BodyAtStart,1.e-6)
		|| !Robot->GetActorTransform().Equals(RobotBaseAtStart,1.e-6))
	{ Fail(TEXT("Body or robot base moved during active cycle. Stop and recalibrate targets.")); return; }
	if (!Robot->StepControl(Dt)) { Fail(TEXT("Invalid servo input or integration failure.")); return; }
	if (bDoorHeld && !bDoorFastened && !Pose->SetDoorWorldPose(DoorInTCP*Robot->GetTCPWorld()))
	{ Fail(TEXT("Door transport pose rejected.")); return; }
	if (bDoorFastened) Pose->SetDoorWorldPose(FastenedInBody*Assembly->GetBodyFrame()->GetComponentTransform());
	MeasureTCP();
	const bool Settled=PositionErrorMm<=PositionToleranceMm && OrientationErrorDeg<=OrientationToleranceDeg && Robot->IsAtJointTarget();
	StableTime=Settled ? StableTime+Dt : 0;
	if (StateTime>StageTimeoutSeconds) { Fail(TEXT("Stage timed out before measured motion settled.")); return; }
	RecordTime+=Dt;
	if (RecordTime>=0.05) { RecordSample(); RecordTime=0; }
	if (State==EDoorAssemblyState::Fasten)
	{
		if (!Settled) { Fail(TEXT("Lost alignment while fastening.")); return; }
		const FTransform Door=Pose->GetControlledDoor()->GetComponentTransform(), Target=GetMountDoorWorld();
		if (FVector::Distance(Door.GetLocation(),Target.GetLocation())*10>PositionToleranceMm
			|| FMath::RadiansToDegrees(Door.GetRotation().AngularDistance(Target.GetRotation()))>OrientationToleranceDeg)
		{ Fail(TEXT("Independent door pose check lost alignment while fastening.")); return; }
		if (StateTime>=FasteningSeconds)
		{
			FastenedInBody=Pose->GetControlledDoor()->GetComponentTransform().GetRelativeTransform(Assembly->GetBodyFrame()->GetComponentTransform());
			bDoorFastened=true; EnterState(EDoorAssemblyState::Release,TEXT("Ideal rigid fastening completed; retaining achieved pose."));
		}
		return;
	}
	if (State==EDoorAssemblyState::Release)
	{
		bDoorHeld=false;
		EnterMove(EDoorAssemblyState::Retract,OffsetDoor(GetMountDoorWorld(),ApproachOffsetBodyCm)); return;
	}
	if (StableTime<SettleSeconds) return;
	switch (State)
	{
	case EDoorAssemblyState::ApproachPickup:
		EnterMove(EDoorAssemblyState::Grip,GetPickupDoorWorld()); break;
	case EDoorAssemblyState::Grip:
		DoorInTCP=Pose->GetControlledDoor()->GetComponentTransform().GetRelativeTransform(Robot->GetTCPWorld());
		bDoorHeld=true; EnterMove(EDoorAssemblyState::Lift,OffsetDoor(GetPickupDoorWorld(),LiftOffsetBodyCm)); break;
	case EDoorAssemblyState::Lift:
		EnterMove(EDoorAssemblyState::Transfer,OffsetDoor(GetMountDoorWorld(),TransferOffsetBodyCm)); break;
	case EDoorAssemblyState::Transfer:
		EnterMove(EDoorAssemblyState::ApproachMount,OffsetDoor(GetMountDoorWorld(),ApproachOffsetBodyCm)); break;
	case EDoorAssemblyState::ApproachMount:
		EnterMove(EDoorAssemblyState::Align,GetMountDoorWorld()); break;
	case EDoorAssemblyState::Align:
	{
		const FTransform Door=Pose->GetControlledDoor()->GetComponentTransform(), Target=GetMountDoorWorld();
		if (FVector::Distance(Door.GetLocation(),Target.GetLocation())*10>PositionToleranceMm
			|| FMath::RadiansToDegrees(Door.GetRotation().AngularDistance(Target.GetRotation()))>OrientationToleranceDeg)
		{ Fail(TEXT("Independent door pose check rejected fastening.")); break; }
		EnterState(EDoorAssemblyState::Fasten,TEXT("Alignment verified. Simulated rigid fastening in progress.")); break;
	}
	case EDoorAssemblyState::Retract:
		EnterState(EDoorAssemblyState::Complete,TEXT("Complete: door held by body; robot released and retracted."));
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
	CSVBuffer+=FString::Printf(TEXT(",%.6f\n"),FVector::Distance(Pose->GetControlledDoor()->GetComponentLocation(),GetMountDoorWorld().GetLocation())*10);
}

void ADoorAssemblyCell::FlushRecording()
{
	if (CSVBuffer.IsEmpty() || LogFile.IsEmpty()) return;
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(LogFile),true);
	if (!FFileHelper::SaveStringToFile(CSVBuffer,*LogFile,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		UE_LOG(LogTemp,Warning,TEXT("Could not save assembly log: %s"),*LogFile);
	CSVBuffer.Reset();
}

void ADoorAssemblyCell::BeginPlay()
{ Super::BeginPlay(); if (ResetCycle() && bAutoStart) StartCycle(); }
void ADoorAssemblyCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (DeltaSeconds>1) { if (Robot) StopCycle(); Status=TEXT("Large frame stall: stopped; reset required."); return; }
	AdvanceSimulation(DeltaSeconds);
}
void ADoorAssemblyCell::EndPlay(const EEndPlayReason::Type Reason)
{ FlushRecording(); Super::EndPlay(Reason); }

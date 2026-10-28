#include "DoorAssemblyCell.h"
#include "RobotArm.h"
#include "CamryAssembly.h"
#include "DoorPoseComponent.h"
#include "DoorInspectionComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"

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
	Inspection=Assembly->FindComponentByClass<UDoorInspectionComponent>();
	if (!Inspection.IsValid() || Inspection->Points.Num()!=6)
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
	Outcome=EDoorCellOutcome::NotEvaluated; AlignmentCorrections=0; LogFile.Empty(); RunMetadataFile.Empty();
	SimulationTimeSeconds=0; Accumulator=0; StateTime=0; StableTime=0;
	FTransform Initial=GetPickupDoorWorld();
	Initial.AddToTranslation(BodyAtStart.TransformVectorNoScale(InitialDoorOffsetMm/10));
	Initial.SetRotation((BodyAtStart.GetRotation()*InitialDoorRotationDeg.Quaternion()*BodyAtStart.GetRotation().Inverse()*Initial.GetRotation()).GetNormalized());
	if (!Pose->SetDoorWorldPose(Initial)) { Fail(TEXT("Could not place door in the pickup station.")); return false; }
	TArray<USceneComponent*> DoorChildren; Pose->GetControlledDoor()->GetChildrenComponents(true,DoorChildren);
	for (auto* C:DoorChildren) if (auto* P=Cast<UPrimitiveComponent>(C)) P->SetVisibility(bDoorPresent);
	RefreshQuality(); MeasureSensors();
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
		CSVBuffer=TEXT("time_s,state,held,fastened,tcp_x_cm,tcp_y_cm,tcp_z_cm,target_x_cm,target_y_cm,target_z_cm,position_error_mm,orientation_error_deg,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,door_mount_error_mm,align_sensor_error_mm,verify_sensor_error_mm,quality_valid,quality_pass,outcome");
		for (int32 I=1;I<=6;++I) CSVBuffer+=FString::Printf(TEXT(",p%d_gap_mm,p%d_flush_mm,p%d_pass"),I,I,I);
		CSVBuffer+=TEXT("\n");
	}
	PickupAtStart=Pose->GetControlledDoor()->GetComponentTransform();
	WriteRunMetadata();
	if (!CheckPickup()) { Fail(Status); return false; }
	if (!MeasureSensors()) { Fail(TEXT("Invalid alignment sensor input.")); return false; }
	return EnterMove(EDoorAssemblyState::ApproachPickup,OffsetDoor(PickupAtStart,ApproachOffsetBodyCm));
}

void ADoorAssemblyCell::StopCycle()
{
	if (State==EDoorAssemblyState::Idle || State==EDoorAssemblyState::Complete || State==EDoorAssemblyState::Fault || State==EDoorAssemblyState::Stopped) return;
	Robot->Halt(); Outcome=EDoorCellOutcome::Stopped; EnterState(EDoorAssemblyState::Stopped,TEXT("Cycle stopped; grasp/fastening state retained. Reset to recover."));
	Accumulator=0; RecordSample(); FlushRecording();
}

void ADoorAssemblyCell::Fail(const FString& Reason)
{
	// Reason can refer to Robot->LastStatus, which Halt changes.
	const FString FailureReason=Reason;
	if (Robot) Robot->Halt();
	Outcome=EDoorCellOutcome::Blocked;
	EnterState(EDoorAssemblyState::Fault,FailureReason); RecordSample(); FlushRecording();
}

void ADoorAssemblyCell::EnterState(EDoorAssemblyState Next,const FString& Message)
{ State=Next; StateTime=0; StableTime=0; Status=Message; }

bool ADoorAssemblyCell::EnterMove(EDoorAssemblyState Next,const FTransform& DoorTarget)
{
	TargetDoorWorld=DoorTarget;
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
	if (Pose.IsValid()) { RefreshQuality(); MeasureSensors(); }
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
	if (bDoorHeld && !bDoorFastened && !Pose->GetControlledDoor()->GetComponentTransform().Equals(LastCarriedDoorWorld,1.e-5))
	{ Fail(TEXT("Door moved externally while held. Stop and revalidate grasp.")); return; }
	if (!bDoorHeld && !bDoorFastened && (State==EDoorAssemblyState::ApproachPickup || State==EDoorAssemblyState::Grip)
		&& !Pose->GetControlledDoor()->GetComponentTransform().Equals(PickupAtStart,1.e-5))
	{ Fail(TEXT("Door placement changed during pickup. Reset and re-detect.")); return; }
	if (!Robot->StepControl(Dt)) { Fail(TEXT("Invalid servo input or integration failure.")); return; }
	if (bDoorHeld && !bDoorFastened && !Pose->SetDoorWorldPose(DoorInTCP*Robot->GetTCPWorld()))
	{ Fail(TEXT("Door transport pose rejected.")); return; }
	if (bDoorFastened) Pose->SetDoorWorldPose(FastenedInBody*Assembly->GetBodyFrame()->GetComponentTransform());
	LastCarriedDoorWorld=Pose->GetControlledDoor()->GetComponentTransform();
	if (!RefreshQuality() || !MeasureSensors()) { Fail(TEXT("Invalid quality geometry or alignment sensor.")); return; }
	MeasureTCP();
	const bool Settled=PositionErrorMm<=PositionToleranceMm && OrientationErrorDeg<=OrientationToleranceDeg && Robot->IsAtJointTarget();
	StableTime=Settled ? StableTime+Dt : 0;
	if (StateTime>StageTimeoutSeconds) { Fail(TEXT("Stage timed out before measured motion settled.")); return; }
	RecordTime+=Dt;
	if (RecordTime>=0.05) { RecordSample(); RecordTime=0; }
	if (State==EDoorAssemblyState::Fasten)
	{
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
		bDoorHeld=true; EnterMove(EDoorAssemblyState::Lift,OffsetDoor(PickupAtStart,LiftOffsetBodyCm)); break;
	case EDoorAssemblyState::Lift:
		EnterMove(EDoorAssemblyState::Transfer,OffsetDoor(GetMountDoorWorld(),TransferOffsetBodyCm)); break;
	case EDoorAssemblyState::Transfer:
		EnterMove(EDoorAssemblyState::ApproachMount,OffsetDoor(GetMountDoorWorld(),ApproachOffsetBodyCm)); break;
	case EDoorAssemblyState::ApproachMount:
		EnterMove(EDoorAssemblyState::Align,GetMountDoorWorld()); break;
	case EDoorAssemblyState::Align:
	{
		if (!AlignmentAccepted())
		{
			if (++AlignmentCorrections>8) { Fail(TEXT("Sensor-driven alignment did not converge within correction limit.")); break; }
			// Feedback uses only the sensor residual and previous commanded pose.
			// It never subtracts the configured fault bias or reads true quality to correct motion.
			const FTransform Mount=GetMountDoorWorld();
			FTransform Corrected=TargetDoorWorld;
			Corrected.AddToTranslation(Mount.GetLocation()-AlignmentMeasuredDoorWorld.GetLocation());
			Corrected.SetRotation((Mount.GetRotation()*AlignmentMeasuredDoorWorld.GetRotation().Inverse()*Corrected.GetRotation()).GetNormalized());
			EnterMove(EDoorAssemblyState::Align,Corrected); break;
		}
		if (!VerifyBeforeFasten()) break;
		EnterState(EDoorAssemblyState::Fasten,TEXT("Measured alignment accepted; ideal fastening in progress.")); break;
	}
	case EDoorAssemblyState::Retract:
		RefreshQuality(); Outcome=bQualityPassed ? EDoorCellOutcome::Pass : EDoorCellOutcome::QualityFail;
		EnterState(EDoorAssemblyState::Complete,bQualityPassed ? TEXT("Complete: six-station quality PASS; door retained and robot retracted.") : TEXT("Complete: assembly QUALITY FAIL; review station gap/flush results."));
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
	if (!Pose.IsValid() || !IsValid(Assembly) || !bAlignmentSensorValid) return false;
	AlignmentMeasuredDoorWorld=SimulatePoseSensor(AlignmentSensorBiasMm,AlignmentSensorRotationBiasDeg);
	VerificationMeasuredDoorWorld=SimulatePoseSensor(VerificationSensorBiasMm,VerificationSensorRotationBiasDeg);
	const FTransform Mount=GetMountDoorWorld();
	AlignmentPositionErrorMm=FVector::Distance(AlignmentMeasuredDoorWorld.GetLocation(),Mount.GetLocation())*10;
	AlignmentOrientationErrorDeg=FMath::RadiansToDegrees(AlignmentMeasuredDoorWorld.GetRotation().AngularDistance(Mount.GetRotation()));
	VerificationPositionErrorMm=FVector::Distance(VerificationMeasuredDoorWorld.GetLocation(),Mount.GetLocation())*10;
	return AlignmentMeasuredDoorWorld.IsValid() && VerificationMeasuredDoorWorld.IsValid();
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
	if (!Inspection.IsValid() || !Inspection->EvaluateDoorPose(VerificationMeasuredDoorWorld,Measured) || Measured.Num()!=6)
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
	J->SetStringField(TEXT("model"),TEXT("Perfect virtual pickup detection; pose sensors with body-frame constant biases; signed virtual gap/flush gauges; ideal rigid grasp/fastening. No collision, load or hinge mechanics."));
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

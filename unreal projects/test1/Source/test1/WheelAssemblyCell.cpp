#include "WheelAssemblyCell.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "UObject/ConstructorHelpers.h"

void UWheelRobotSensorBank::Reset()
{
	LastSample = -1; LastWheelIndex = INDEX_NONE; bGripClosed = false; bPublishPickupPose = true; bGripActuatorFault = false;
	Presence = PickupPose = Gripped = MountPose = FWheelSensorReading();
}

void UWheelRobotSensorBank::Sample(double Now, const AWheelAssemblyCell* Cell, int32 WheelIndex)
{
	if (LastWheelIndex == WheelIndex && LastSample >= 0 && Now - LastSample + 1.e-9 < SamplePeriod) return;
	LastSample = Now; LastWheelIndex = WheelIndex;
	const bool Ready = IsValid(Cell) && IsValid(Cell->LeftRobot) && WheelIndex >= 0 && WheelIndex < 2;
	const bool Present = Ready && Cell->IsWheelVisible(WheelIndex);
	Presence = FWheelSensorReading(); Presence.bValid = Ready; Presence.bDetected = Present; Presence.MeasuredAt = Now;
	if (bPublishPickupPose)
	{
		PickupPose.bValid = Present; PickupPose.bDetected = Present;
		PickupPose.Pose = Present ? Cell->GetWheelWorld(WheelIndex) : FTransform::Identity;
		PickupPose.MeasuredAt = Now;
	}
	const FTransform Wheel = Present ? Cell->GetWheelWorld(WheelIndex) : FTransform::Identity;
	const FTransform TCP = Ready ? Cell->LeftRobot->GetTCPWorld() : FTransform::Identity;
	const FTransform Contact = Ready ? Cell->LeftTCPInWheel * Wheel : FTransform::Identity;
	const bool AtContact = Present && FVector::Distance(Contact.GetLocation(), TCP.GetLocation()) * 10 <= Cell->PositionToleranceMm
		&& FMath::RadiansToDegrees(Contact.GetRotation().AngularDistance(TCP.GetRotation())) <= Cell->OrientationToleranceDeg;
	Gripped.bValid = Ready; Gripped.bDetected = bGripClosed && !bGripActuatorFault && AtContact; Gripped.MeasuredAt = Now;
	MountPose.bValid = Present; MountPose.bDetected = Present; MountPose.Pose = Wheel; MountPose.MeasuredAt = Now;
}

AWheelAssemblyCell::AWheelAssemblyCell()
{
	PrimaryActorTick.bCanEverTick = true;
	CellRoot = CreateDefaultSubobject<USceneComponent>(TEXT("WheelCellRoot"));
	SetRootComponent(CellRoot);
	LeftSensors = CreateDefaultSubobject<UWheelRobotSensorBank>(TEXT("LeftRobotSensors"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	for (int32 Index = 0; Index < 4; ++Index)
	{
		USceneComponent* Wheel = CreateDefaultSubobject<USceneComponent>(FName(*FString::Printf(TEXT("Wheel%d"), Index)));
		Wheel->SetupAttachment(CellRoot);
		Wheel->SetMobility(EComponentMobility::Movable);
		WheelRoots.Add(Wheel);
		UStaticMeshComponent* Tire = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("Tire%d"), Index)));
		Tire->SetupAttachment(Wheel);
		Tire->SetMobility(EComponentMobility::Movable);
		Tire->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Tire->SetGenerateOverlapEvents(false);
		Tire->SetRelativeRotation(FRotator(0, 0, 90));
		Tire->SetRelativeScale3D(FVector(0.66, 0.66, 0.16));
		if (Cylinder.Succeeded()) Tire->SetStaticMesh(Cylinder.Object);
		TireVisuals.Add(Tire);
		UStaticMeshComponent* Hub = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("WheelHub%d"), Index)));
		Hub->SetupAttachment(Wheel);
		Hub->SetMobility(EComponentMobility::Movable);
		Hub->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Hub->SetGenerateOverlapEvents(false);
		Hub->SetRelativeRotation(FRotator(0, 0, 90));
		Hub->SetRelativeLocation(FVector(0, Index < 2 ? -8.5 : 8.5, 0));
		Hub->SetRelativeScale3D(FVector(0.37, 0.37, 0.025));
		if (Cylinder.Succeeded()) Hub->SetStaticMesh(Cylinder.Object);
		HubVisuals.Add(Hub);
	}
}

ARobotArm* AWheelAssemblyCell::RobotFor(int32 Side) const { return Side == 0 ? LeftRobot.Get() : RightRobot.Get(); }
FTransform AWheelAssemblyCell::TCPFor(int32 Side) const { return Side == 0 ? LeftTCPInWheel : RightTCPInWheel; }

FTransform AWheelAssemblyCell::GetMountWorld(int32 Index) const
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || Index < 0 || Index > 3) return FTransform::Identity;
	const double X = Index % 2 == 0 ? FrontXcm : RearXcm;
	const double Y = Index < 2 ? -HubHalfTrackCm : HubHalfTrackCm;
	return FTransform(FRotator::ZeroRotator, MountCentersCm.Num() == 4 ? MountCentersCm[Index] : FVector(X, Y, HubHeightCm)) * Assembly->GetBodyFrame()->GetComponentTransform();
}

FTransform AWheelAssemblyCell::GetPickupWorld(int32 Index) const
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || Index < 0 || Index > 3) return FTransform::Identity;
	const double X = MountCentersCm.Num() == 4 ? MountCentersCm[Index].X : (Index % 2 == 0 ? FrontXcm : RearXcm);
	const double Y = Index < 2 ? -PickupHalfTrackCm : PickupHalfTrackCm;
	// Wheel racks stay at the station while the vehicle travels toward them.
	return FTransform(FRotator::ZeroRotator, FVector(X, Y, MountCentersCm.Num() == 4 ? MountCentersCm[Index].Z : HubHeightCm)) * (bEnableTransport && bStationCaptured ? BodyAtStart : Assembly->GetBodyFrame()->GetComponentTransform());
}

FTransform AWheelAssemblyCell::GetWheelWorld(int32 Index) const
{
	return WheelRoots.IsValidIndex(Index) && WheelRoots[Index] ? WheelRoots[Index]->GetComponentTransform() : FTransform::Identity;
}

bool AWheelAssemblyCell::IsWheelVisible(int32 Index) const
{
	return TireVisuals.IsValidIndex(Index) && IsValid(TireVisuals[Index]) && TireVisuals[Index]->IsVisible();
}

void AWheelAssemblyCell::InjectLeftWheelMissing()
{
	SetLeftWheelSupplyVisible(0, false);
}

void AWheelAssemblyCell::InjectLeftWheelDisplacement(FVector OffsetCm)
{
	DisplaceLeftWheel(0, OffsetCm);
}

void AWheelAssemblyCell::SetLeftWheelSupplyVisible(int32 Index, bool bVisible)
{
	if (Index < 0 || Index > 1 || (FastenedWheels.IsValidIndex(Index) && FastenedWheels[Index])) return;
	if (TireVisuals.IsValidIndex(Index)) TireVisuals[Index]->SetVisibility(bVisible);
	if (HubVisuals.IsValidIndex(Index)) HubVisuals[Index]->SetVisibility(bVisible);
}

void AWheelAssemblyCell::DisplaceLeftWheel(int32 Index, FVector OffsetCm)
{
	if (Index >= 0 && Index <= 1 && WheelRoots.IsValidIndex(Index) && WheelRoots[Index]
		&& !OffsetCm.ContainsNaN() && !(FastenedWheels.IsValidIndex(Index) && FastenedWheels[Index]))
		WheelRoots[Index]->AddWorldOffset(OffsetCm);
}

FString AWheelAssemblyCell::GetLeftSensorSummary() const
{
	if (!bSensorDrivenLeftRobot || !LeftSensors) return TEXT("왼팔 센서: 비활성");
	const double Now = SimulationTimeSeconds;
	const int32 Index = Lanes[0].WheelIndex;
	const bool PoseFresh = LeftSensors->PickupPose.Fresh(Now, LeftSensors->MaxAge);
	const bool GripFresh = LeftSensors->Gripped.Fresh(Now, LeftSensors->MaxAge);
	return FString::Printf(TEXT("왼팔 %s · 타이어 %d · 부품 %s · 위치 %s · 파지 %s · 장착 측정 %s · 재시도 %d"),
		*GetLaneStage(true), Index + 1,
		LeftSensors->Presence.Fresh(Now, LeftSensors->MaxAge) ? (LeftSensors->Presence.bDetected ? TEXT("있음") : TEXT("없음")) : TEXT("신호 지연"),
		PoseFresh ? TEXT("유효") : TEXT("무효/지연"), GripFresh ? (LeftSensors->Gripped.bDetected ? TEXT("확인") : TEXT("미확인")) : TEXT("지연"),
		LeftSensors->MountPose.Fresh(Now, LeftSensors->MaxAge) ? TEXT("유효") : TEXT("무효/지연"), Lanes[0].GripAttempts)
		+ FString::Printf(TEXT(" · 경로 재계산 %d · 파지 재시도 %d"), LeftPickupReplans, LeftGripRetries);
}

FTransform AWheelAssemblyCell::OffsetOutward(const FTransform& Pose, int32 Side, double Centimeters) const
{
	FTransform Result = Pose;
	Result.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, Side == 0 ? -Centimeters : Centimeters, 0)));
	return Result;
}

bool AWheelAssemblyCell::ValidateSetup()
{
	if (MountCentersCm.Num() != 0 && MountCentersCm.Num() != 4)
	{ Status = TEXT("Provide all four measured wheel centers or leave the array empty."); return false; }
	for (const FVector& Center : MountCentersCm) if (Center.ContainsNaN())
	{ Status = TEXT("Wheel centers must be finite."); return false; }
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || !IsValid(LeftRobot) || !IsValid(RightRobot) || LeftRobot == RightRobot)
	{ Status = TEXT("Assign one vehicle and two distinct calibrated robots."); return false; }
	const double Positive[] = {FrontXcm, HubHalfTrackCm, HubHeightCm, PickupHalfTrackCm, PositionToleranceMm,
		OrientationToleranceDeg, QualityToleranceMm, QualityOrientationDeg, SettleSeconds, FasteningSeconds,
		StageTimeoutSeconds, FixedStepSeconds, MaxPickupDisplacementMm, MaxPickupRotationDeg,
		LeftRecoveryTimeoutSeconds, LeftRecoveryStableSeconds};
	for (double Value : Positive) if (!FMath::IsFinite(Value) || Value <= 0)
	{ Status = TEXT("Wheel geometry, control times and tolerances must be positive finite values."); return false; }
	if (!FMath::IsFinite(RearXcm) || FixedStepSeconds > 0.02 || FaultWheelIndex < 0 || FaultWheelIndex > 3
		|| MaxAutoGripAttempts < 2
		|| SensorBiasMm.ContainsNaN() || InitialOffsetMm.ContainsNaN() || InitialRotationDeg.ContainsNaN()
		|| !LeftTCPInWheel.IsValid() || !RightTCPInWheel.IsValid()
		|| !LeftTCPInWheel.GetScale3D().Equals(FVector::OneVector, 1.e-6)
		|| !RightTCPInWheel.GetScale3D().Equals(FVector::OneVector, 1.e-6))
	{ Status = TEXT("Invalid wheel experiment or grasp calibration."); return false; }
	BodyAtStart = Assembly->GetBodyFrame()->GetComponentTransform();
	if (!BodyAtStart.IsValid() || !BodyAtStart.GetScale3D().Equals(FVector::OneVector, 1.e-6))
	{ Status = TEXT("Vehicle body must have a valid rigid transform."); return false; }
	for (int32 Side = 0; Side < 2; ++Side)
	{
		if ((Side == 0 ? LeftRearApproachAnglesDegrees : RightRearApproachAnglesDegrees).Num() != 6)
		{ Status = TEXT("Both rear station joint targets require six angles."); return false; }
		RobotAtStart[Side] = RobotFor(Side)->GetActorTransform();
		if (!RobotAtStart[Side].IsValid() || !RobotAtStart[Side].GetScale3D().Equals(FVector::OneVector, 1.e-6))
		{ Status = TEXT("Robot base must have a valid rigid transform."); return false; }
	}
	return true;
}

bool AWheelAssemblyCell::ResetCycle()
{
	if (LeftRobot) LeftRobot->Halt();
	if (RightRobot) RightRobot->Halt();
	bRunning = false;
	bTransporting = false;
	if (bStationCaptured && bEnableTransport && IsValid(Assembly) && IsValid(TransportCarriage))
	{
		Assembly->SetActorTransform(VehicleAtStation);
		TransportCarriage->SetActorTransform(CarriageAtStation);
	}
	if (!ValidateSetup() || !LeftRobot->InitializeArm() || !RightRobot->InitializeArm())
	{ Outcome = EWheelOutcome::Blocked; Status += TEXT(" Robot calibration failed or references missing."); return false; }
	if (bEnableTransport)
	{
		if (!IsValid(TransportCarriage) || TransportCarriage == Assembly || TransportCarriage == LeftRobot || TransportCarriage == RightRobot
			|| TransportStartOffsetCm.ContainsNaN() || TransportStartOffsetCm.IsNearlyZero()
			|| !FMath::IsFinite(TransportSeconds) || TransportSeconds <= 0
			|| !FMath::IsFinite(DockSettleSeconds) || DockSettleSeconds < 0)
		{ Outcome = EWheelOutcome::Blocked; Status = TEXT("Invalid transport carriage, travel or docking settings."); return false; }
		VehicleAtStation = Assembly->GetActorTransform();
		CarriageAtStation = TransportCarriage->GetActorTransform();
		bStationCaptured = true;
	}
	MountedCount = 0; SimulationTimeSeconds = 0; Accumulator = 0;
	LeftPickupReplans = LeftGripRetries = 0;
	Outcome = EWheelOutcome::NotEvaluated;
	MountPositionErrorMm.Init(0, 4); MountOrientationErrorDeg.Init(0, 4); FastenedWheels.Init(false, 4);
	for (int32 Index = 0; Index < 4; ++Index)
	{
		FTransform Initial = GetPickupWorld(Index);
		if (Index == FaultWheelIndex)
		{
			Initial.AddToTranslation(BodyAtStart.TransformVectorNoScale(InitialOffsetMm / 10.0));
			Initial.SetRotation((BodyAtStart.GetRotation() * InitialRotationDeg.Quaternion()
				* BodyAtStart.GetRotation().Inverse() * Initial.GetRotation()).GetNormalized());
		}
		WheelRoots[Index]->SetWorldTransform(Initial);
		const bool Visible = Index != FaultWheelIndex || bWheelPresent;
		TireVisuals[Index]->SetVisibility(Visible); HubVisuals[Index]->SetVisibility(Visible);
	}
	for (int32 Side = 0; Side < 2; ++Side) Lanes[Side] = FWheelLane();
	if (LeftSensors) LeftSensors->Reset();
	RefreshQuality();
	TransportTime = 0; TransportProgress = 0;
	if (bEnableTransport) SetTransportProgress(0);
	Status = TEXT("Ready: Enter starts both arms; Space stops; R resets.");
	return true;
}

bool AWheelAssemblyCell::StartCycle()
{
	if (bRunning || Outcome != EWheelOutcome::NotEvaluated) { Status = TEXT("Reset before starting a new cycle."); return false; }
	if (!bEnableTransport && !ValidateSetup()) { Fail(Status); return false; }
	if (bEnableTransport && (!bStationCaptured || !IsValid(Assembly) || !IsValid(TransportCarriage)
		|| !Assembly->GetActorTransform().Equals(ExpectedVehicle, 1.e-3)
		|| !TransportCarriage->GetActorTransform().Equals(ExpectedCarriage, 1.e-3)))
	{ Fail(TEXT("Transport setup changed; reset before starting.")); return false; }
	if (!bAlignmentSensorValid) { Fail(TEXT("Alignment sensor invalid; both robots remain stopped.")); return false; }
	for (int32 Index = 0; Index < 4; ++Index)
	{
		if (!(bSensorDrivenLeftRobot && Index < 2) && Index == FaultWheelIndex && !bWheelPresent) { Fail(TEXT("Pickup blocked: wheel missing.")); return false; }
		if (bSensorDrivenLeftRobot && Index < 2) continue;
		const FTransform Actual = GetWheelWorld(Index), Expected = GetPickupWorld(Index);
		if (!Actual.IsValid() || !Actual.GetScale3D().Equals(FVector::OneVector, 1.e-6)
			|| FVector::Distance(Actual.GetLocation(), Expected.GetLocation()) * 10 > MaxPickupDisplacementMm)
		{ Fail(TEXT("Pickup blocked: wheel outside recovery window.")); return false; }
		if (FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Expected.GetRotation())) > MaxPickupRotationDeg)
		{ Fail(TEXT("Pickup blocked: reversed wheel or invalid grasp orientation.")); return false; }
	}
	SimulationTimeSeconds = 0; Accumulator = 0; bRunning = true;
	if (bEnableTransport)
	{
		bTransporting = true; TransportTime = 0;
		Status = TEXT("차체 운반 중 — 도킹 완료 후 타이어 조립을 시작합니다.");
		return true;
	}
	for (int32 Side = 0; Side < 2; ++Side) if (!BeginWheel(Side)) return false;
	Status = TEXT("Both arms installing front wheels.");
	return true;
}

bool AWheelAssemblyCell::BeginWheel(int32 Side)
{
	FWheelLane& Lane = Lanes[Side];
	Lane.WheelIndex = Side * 2 + Lane.Completed;
	Lane.bHeld = false; Lane.bFastened = false; Lane.Corrections = 0;
	Lane.GripAttempts = 0; Lane.bGripCommanded = false; Lane.GripCommandTime = 0;
	if (Side == 0 && bSensorDrivenLeftRobot)
	{
		if (!LeftSensors) { Fail(TEXT("Left robot sensor bank missing.")); return false; }
		LeftSensors->CommandGrip(false);
		LeftSensors->Sample(SimulationTimeSeconds, this, Lane.WheelIndex);
		if (!LeftSensors->Presence.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge) || !LeftSensors->Presence.bDetected)
		{
			if (bAutoRecoverLeftRobot) { WaitForLeftSensor(EWheelStage::WaitSupply, TEXT("Left robot: waiting for wheel supply")); return true; }
			Fail(TEXT("Left robot: pickup presence sensor reports no wheel.")); return false;
		}
		if (!LeftSensors->PickupPose.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge))
		{
			if (bAutoRecoverLeftRobot) { WaitForLeftSensor(EWheelStage::WaitPose, TEXT("Left robot: waiting for pickup measurement")); return true; }
			Fail(TEXT("Left robot: pickup pose measurement unavailable.")); return false;
		}
		Lane.Pickup = LeftSensors->PickupPose.Pose;
	}
	else Lane.Pickup = GetWheelWorld(Lane.WheelIndex);
	return MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Lane.Pickup, Side, 25));
}

void AWheelAssemblyCell::SetStage(int32 Side, EWheelStage Next, const FString& Message)
{
	FWheelLane& Lane = Lanes[Side];
	Lane.Stage = Next; Lane.StageTime = 0; Lane.StableTime = 0;
	Status = FString::Printf(TEXT("%s arm: %s"), Side == 0 ? TEXT("Left") : TEXT("Right"), *Message);
}

bool AWheelAssemblyCell::MoveTo(int32 Side, EWheelStage Next, const FTransform& WheelTarget)
{
	FWheelLane& Lane = Lanes[Side];
	Lane.TargetWheel = WheelTarget;
	Lane.TargetTCP = (Lane.bHeld ? Lane.WheelInTCP.Inverse() : TCPFor(Side)) * WheelTarget;
	if (!RobotFor(Side)->CommandTCPWorld(Lane.TargetTCP))
	{
		Fail(FString::Printf(TEXT("%s arm IK: %s"), Side == 0 ? TEXT("Left") : TEXT("Right"), *RobotFor(Side)->LastStatus));
		return false;
	}
	SetStage(Side, Next, StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Next)));
	return true;
}

void AWheelAssemblyCell::WaitForLeftSensor(EWheelStage WaitStage, const FString& Message)
{
	LeftRobot->Halt();
	LeftSensors->CommandGrip(false);
	Lanes[0].bGripCommanded = false;
	SetStage(0, WaitStage, Message);
}

void AWheelAssemblyCell::Fail(const FString& Reason)
{
	const FString Saved = Reason;
	bRunning = false; bTransporting = false; Outcome = EWheelOutcome::Blocked;
	if (LeftRobot) LeftRobot->Halt();
	if (RightRobot) RightRobot->Halt();
	for (FWheelLane& Lane : Lanes) if (Lane.Stage != EWheelStage::Complete) Lane.Stage = EWheelStage::Fault;
	Status = Saved;
}

void AWheelAssemblyCell::StopCycle()
{
	if (!bRunning) return;
	bRunning = false; bTransporting = false; Outcome = EWheelOutcome::Stopped;
	LeftRobot->Halt(); RightRobot->Halt();
	for (FWheelLane& Lane : Lanes) if (Lane.Stage != EWheelStage::Complete) Lane.Stage = EWheelStage::Stopped;
	Status = TEXT("Stopped. Reset to restart; mounted wheels stay in place until reset.");
}

void AWheelAssemblyCell::RefreshQuality()
{
	if (MountPositionErrorMm.Num() != 4 || MountOrientationErrorDeg.Num() != 4) return;
	for (int32 Index = 0; Index < 4; ++Index)
	{
		const FTransform Mount = GetMountWorld(Index), Actual = GetWheelWorld(Index);
		MountPositionErrorMm[Index] = FVector::Distance(Actual.GetLocation(), Mount.GetLocation()) * 10;
		MountOrientationErrorDeg[Index] = FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Mount.GetRotation()));
	}
}

void AWheelAssemblyCell::StepLane(int32 Side, double Dt)
{
	FWheelLane& Lane = Lanes[Side];
	if (Lane.Stage == EWheelStage::Complete) return;
	ARobotArm* Robot = RobotFor(Side);
	const bool SensorDriven = Side == 0 && bSensorDrivenLeftRobot;
	if (SensorDriven && (Lane.Stage == EWheelStage::WaitSupply || Lane.Stage == EWheelStage::WaitPose))
	{
		Lane.StageTime += Dt;
		if (Lane.StageTime > LeftRecoveryTimeoutSeconds)
		{ Fail(TEXT("Left robot: supply or sensor recovery timed out.")); return; }
		const bool Ready = LeftSensors && LeftSensors->Presence.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge)
			&& LeftSensors->Presence.bDetected && LeftSensors->PickupPose.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge);
		Lane.StableTime = Ready ? Lane.StableTime + Dt : 0;
		if (Lane.StableTime < LeftRecoveryStableSeconds) return;
		const FTransform Measured = LeftSensors->PickupPose.Pose;
		const FTransform Expected = GetPickupWorld(Lane.WheelIndex);
		if (FVector::Distance(Measured.GetLocation(), Expected.GetLocation()) * 10 > MaxPickupDisplacementMm
			|| FMath::RadiansToDegrees(Measured.GetRotation().AngularDistance(Expected.GetRotation())) > MaxPickupRotationDeg)
		{ Fail(TEXT("Left robot: restored supply is outside pickup range.")); return; }
		Lane.Pickup = Measured;
		MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Measured, Side, 25));
		return;
	}
	if (SensorDriven)
	{
		if (!LeftSensors || !LeftSensors->Presence.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge) || !LeftSensors->Presence.bDetected)
		{
			if (bAutoRecoverLeftRobot && !Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip))
			{ WaitForLeftSensor(EWheelStage::WaitSupply, TEXT("Left robot: supply missing; request replacement")); return; }
			Fail(TEXT("Left robot: wheel missing or presence reading stale.")); return;
		}
		if (!LeftSensors->PickupPose.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge))
		{
			if (bAutoRecoverLeftRobot && !Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip))
			{ WaitForLeftSensor(EWheelStage::WaitPose, TEXT("Left robot: sensor link stale; waiting for new measurement")); return; }
			Fail(TEXT("Left robot: pickup pose reading stale.")); return;
		}
		if (Lane.bHeld && !Lane.bFastened && (!LeftSensors->Gripped.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge) || !LeftSensors->Gripped.bDetected))
		{ Fail(TEXT("Left robot: grip lost, both arms stopped.")); return; }
		if (!Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip)
			&& !LeftSensors->PickupPose.Pose.Equals(Lane.Pickup, 1.e-5))
		{
			const FTransform Measured = LeftSensors->PickupPose.Pose;
			const FTransform Expected = GetPickupWorld(Lane.WheelIndex);
			const double OffsetMm = FVector::Distance(Measured.GetLocation(), Expected.GetLocation()) * 10;
			const double AngleDeg = FMath::RadiansToDegrees(Measured.GetRotation().AngularDistance(Expected.GetRotation()));
			if (OffsetMm > MaxPickupDisplacementMm || AngleDeg > MaxPickupRotationDeg)
			{ Fail(TEXT("Left robot: measured wheel outside pickup recovery window.")); return; }
			Lane.Pickup = Measured; Lane.bGripCommanded = false; LeftSensors->CommandGrip(false); ++LeftPickupReplans;
			MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Measured, Side, 25));
			return;
		}
	}
	Lane.StageTime += Dt;
	if (Lane.StageTime > StageTimeoutSeconds)
	{
		const FTransform Actual = Robot->GetTCPWorld();
		Fail(FString::Printf(TEXT("%s %s timed out: TCP %.2f mm / %.2f deg; joints settled %d."),
			Side == 0 ? TEXT("Left") : TEXT("Right"),
			*StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Lane.Stage)),
			FVector::Distance(Actual.GetLocation(), Lane.TargetTCP.GetLocation()) * 10,
			FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Lane.TargetTCP.GetRotation())),
			Robot->IsAtJointTarget() ? 1 : 0));
		return;
	}
	if (Lane.bHeld && !Lane.bFastened && !GetWheelWorld(Lane.WheelIndex).Equals(Lane.LastCarried, 1.e-5))
	{ Fail(TEXT("Held wheel moved externally.")); return; }
	if (!SensorDriven && !Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip)
		&& !GetWheelWorld(Lane.WheelIndex).Equals(Lane.Pickup, 1.e-5))
	{ Fail(TEXT("Wheel moved during pickup.")); return; }
	if (!Robot->StepControl(Dt)) { Fail(TEXT("Robot servo failed.")); return; }
	if (Lane.bHeld && !Lane.bFastened) WheelRoots[Lane.WheelIndex]->SetWorldTransform(Lane.WheelInTCP * Robot->GetTCPWorld());
	Lane.LastCarried = GetWheelWorld(Lane.WheelIndex);
	const FTransform CurrentTCP = Robot->GetTCPWorld();
	const double TCPErrorMm = FVector::Distance(CurrentTCP.GetLocation(), Lane.TargetTCP.GetLocation()) * 10;
	const double TCPErrorDeg = FMath::RadiansToDegrees(CurrentTCP.GetRotation().AngularDistance(Lane.TargetTCP.GetRotation()));
	// A large station-to-station joint transit may retain a tiny PID oscillation.
	// The following IK command settles the exact pickup TCP before contact.
	const bool Settled = Lane.Stage == EWheelStage::Reposition
		? TCPErrorMm <= 5 && TCPErrorDeg <= 0.5
		: TCPErrorMm <= PositionToleranceMm && TCPErrorDeg <= OrientationToleranceDeg && Robot->IsAtJointTarget();
	Lane.StableTime = Settled ? Lane.StableTime + Dt : 0;
	if (Lane.Stage == EWheelStage::Fasten)
	{
		if (!Settled) { Fail(TEXT("Wheel alignment lost during fastening.")); return; }
		if (Lane.StageTime >= FasteningSeconds)
		{
			Lane.bFastened = true; FastenedWheels[Lane.WheelIndex] = true; ++MountedCount;
			FastenedInBody[Lane.WheelIndex] = GetWheelWorld(Lane.WheelIndex).GetRelativeTransform(Assembly->GetBodyFrame()->GetComponentTransform());
			SetStage(Side, EWheelStage::Release, TEXT("virtual fastening complete"));
		}
		return;
	}
	if (Lane.Stage == EWheelStage::Release)
	{
		if (SensorDriven) LeftSensors->CommandGrip(false);
		Lane.bHeld = false;
		MoveTo(Side, EWheelStage::Retract, OffsetOutward(GetWheelWorld(Lane.WheelIndex), Side, 25));
		return;
	}
	if (Lane.StableTime < SettleSeconds) return;
	switch (Lane.Stage)
	{
	case EWheelStage::ApproachPickup:
		MoveTo(Side, EWheelStage::Grip, Lane.Pickup); break;
	case EWheelStage::Grip:
	{
		if (SensorDriven)
		{
			if (!Lane.bGripCommanded)
			{
				LeftSensors->CommandGrip(true); Lane.bGripCommanded = true;
				Lane.GripCommandTime = SimulationTimeSeconds; ++Lane.GripAttempts;
				Status = TEXT("Left arm: gripper closed; waiting for grip sensor");
				break;
			}
			if (!LeftSensors->Gripped.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge))
			{ Fail(TEXT("Left robot: grip reading stale.")); break; }
			if (!LeftSensors->Gripped.bDetected)
			{
				if (SimulationTimeSeconds - Lane.GripCommandTime >= .3)
				{
					if (Lane.GripAttempts >= (bAutoRecoverLeftRobot ? MaxAutoGripAttempts : 2))
						Fail(TEXT("Left robot: grip recovery attempts exhausted."));
					else { LeftSensors->CommandGrip(false); Lane.bGripCommanded = false; ++LeftGripRetries; Status = TEXT("Left arm: grip not confirmed; retrying"); }
				}
				break;
			}
		}
		const FTransform Contact = TCPFor(Side) * Lane.Pickup;
		if (FVector::Distance(Contact.GetLocation(), CurrentTCP.GetLocation()) * 10 > PositionToleranceMm
			|| FMath::RadiansToDegrees(Contact.GetRotation().AngularDistance(CurrentTCP.GetRotation())) > OrientationToleranceDeg)
		{ Fail(TEXT("Gripper contact mismatch.")); break; }
		Lane.WheelInTCP = Lane.Pickup.GetRelativeTransform(CurrentTCP);
		Lane.bHeld = true; Lane.LastCarried = Lane.Pickup;
		FTransform Lift = Lane.Pickup;
		Lift.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, 0, 25)));
		MoveTo(Side, EWheelStage::Lift, Lift);
		break;
	}
	case EWheelStage::Lift:
	{
		FTransform Transfer = OffsetOutward(GetMountWorld(Lane.WheelIndex), Side, 55);
		Transfer.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, 0, 25)));
		MoveTo(Side, EWheelStage::Transfer, Transfer); break;
	}
	case EWheelStage::Transfer:
		MoveTo(Side, EWheelStage::ApproachMount, OffsetOutward(GetMountWorld(Lane.WheelIndex), Side, 25)); break;
	case EWheelStage::ApproachMount:
		MoveTo(Side, EWheelStage::Align, GetMountWorld(Lane.WheelIndex)); break;
	case EWheelStage::Align:
	{
		if (SensorDriven && !LeftSensors->MountPose.Fresh(SimulationTimeSeconds, LeftSensors->MaxAge))
		{ Fail(TEXT("Left robot: mount pose reading stale.")); break; }
		const FVector Measured = (SensorDriven ? LeftSensors->MountPose.Pose : GetWheelWorld(Lane.WheelIndex)).GetLocation()
			+ BodyAtStart.TransformVectorNoScale(SensorBiasMm / 10.0);
		const FVector Residual = GetMountWorld(Lane.WheelIndex).GetLocation() - Measured;
		if (Residual.Length() * 10 > PositionToleranceMm)
		{
			if (++Lane.Corrections > 8) { Fail(TEXT("Sensor alignment did not converge.")); break; }
			FTransform Corrected = Lane.TargetWheel;
			Corrected.AddToTranslation(Residual);
			MoveTo(Side, EWheelStage::Align, Corrected); break;
		}
		if (ControlMode == EWheelControlMode::IndependentVerification)
		{
			if (!bVerificationSensorValid) { Fail(TEXT("Independent verification sensor invalid; fastening blocked.")); break; }
			RefreshQuality();
			const FTransform Verified = SensorDriven ? LeftSensors->MountPose.Pose : GetWheelWorld(Lane.WheelIndex);
			const FTransform Mount = GetMountWorld(Lane.WheelIndex);
			const double VerifiedMm = FVector::Distance(Verified.GetLocation(), Mount.GetLocation()) * 10;
			const double VerifiedDeg = FMath::RadiansToDegrees(Verified.GetRotation().AngularDistance(Mount.GetRotation()));
			if (VerifiedMm > QualityToleranceMm || VerifiedDeg > QualityOrientationDeg)
			{ Fail(TEXT("Independent wheel alignment rejected; fastening blocked.")); break; }
		}
		SetStage(Side, EWheelStage::Fasten, TEXT("wheel aligned; virtual fastening")); break;
	}
	case EWheelStage::Retract:
		++Lane.Completed;
		if (Lane.Completed < 2)
		{
			const TArray<double>& RearAngles = Side == 0 ? LeftRearApproachAnglesDegrees : RightRearApproachAnglesDegrees;
			if (!Robot->CommandJointDegrees(RearAngles)) { Fail(Robot->LastStatus); break; }
			Lane.TargetTCP = Robot->GetTargetTCPWorld();
			SetStage(Side, EWheelStage::Reposition, TEXT("joint-space transit to rear station"));
		}
		else SetStage(Side, EWheelStage::Complete, TEXT("two wheels installed"));
		break;
	case EWheelStage::Reposition:
		BeginWheel(Side); break;
	default: break;
	}
}

bool AWheelAssemblyCell::AdvanceSimulation(double DeltaSeconds)
{
	if (!FMath::IsFinite(DeltaSeconds) || DeltaSeconds < 0 || DeltaSeconds > 1 || FixedStepSeconds <= 0 || FixedStepSeconds > 0.02) return false;
	UpdateFastenedWheels();
	if (!bRunning) return true;
	Accumulator += DeltaSeconds;
	while (bRunning && Accumulator + 1.e-10 >= FixedStepSeconds)
	{
		Accumulator -= FixedStepSeconds; SimulationTimeSeconds += FixedStepSeconds;
		if (bSensorDrivenLeftRobot && LeftSensors && !bTransporting && Lanes[0].WheelIndex >= 0)
			LeftSensors->Sample(SimulationTimeSeconds, this, Lanes[0].WheelIndex);
		if (bTransporting)
		{
			if (!IsValid(Assembly) || !IsValid(TransportCarriage) || !IsValid(LeftRobot) || !IsValid(RightRobot)
				// Scene components may elide sub-micron moves at the ends of the
				// smooth trajectory. Allow 0.01 mm, below assembly tolerances.
				|| !Assembly->GetActorTransform().Equals(ExpectedVehicle, 1.e-3)
				|| !TransportCarriage->GetActorTransform().Equals(ExpectedCarriage, 1.e-3)
				|| !LeftRobot->GetActorTransform().Equals(RobotAtStart[0], 1.e-6)
				|| !RightRobot->GetActorTransform().Equals(RobotAtStart[1], 1.e-6))
			{ Fail(TEXT("Vehicle, carriage or robot moved externally during transport.")); break; }
			TransportTime += FixedStepSeconds;
			const double T = FMath::Clamp(TransportTime / TransportSeconds, 0.0, 1.0);
			SetTransportProgress(T*T*T*(10 + T*(-15 + 6*T)));
			Status = T < 1 ? TEXT("차체 운반 중") : TEXT("차체 도킹 · 위치 안정화 중");
			RefreshQuality();
			if (TransportTime + 1.e-9 >= TransportSeconds + DockSettleSeconds)
			{
				bTransporting = false;
				for (int32 Side = 0; Side < 2; ++Side) if (!BeginWheel(Side)) break;
			}
			continue;
		}
		if (!IsValid(Assembly) || !Assembly->GetBodyFrame()
			|| !IsValid(LeftRobot) || !IsValid(RightRobot)
			|| (bEnableTransport && (!IsValid(TransportCarriage) || !TransportCarriage->GetActorTransform().Equals(CarriageAtStation, 1.e-3)))
			|| !Assembly->GetBodyFrame()->GetComponentTransform().Equals(BodyAtStart, bEnableTransport ? 1.e-3 : 1.e-6)
			|| !LeftRobot->GetActorTransform().Equals(RobotAtStart[0], 1.e-6)
			|| !RightRobot->GetActorTransform().Equals(RobotAtStart[1], 1.e-6))
		{ Fail(TEXT("Vehicle or robot base moved during assembly.")); break; }
		StepLane(0, FixedStepSeconds);
		if (bRunning) StepLane(1, FixedStepSeconds);
		RefreshQuality();
		if (bRunning && Lanes[0].Stage == EWheelStage::Complete && Lanes[1].Stage == EWheelStage::Complete)
		{
			bRunning = false;
			bool Passed = MountedCount == 4;
			for (int32 Index = 0; Index < 4; ++Index)
				Passed &= FastenedWheels[Index] && MountPositionErrorMm[Index] <= QualityToleranceMm
					&& MountOrientationErrorDeg[Index] <= QualityOrientationDeg;
			Outcome = Passed ? EWheelOutcome::Pass : EWheelOutcome::QualityFail;
			Status = Passed ? TEXT("PASS: four wheels installed and checked.") : TEXT("QUALITY FAIL: check individual wheel mount errors.");
		}
	}
	return Outcome != EWheelOutcome::Blocked;
}

bool AWheelAssemblyCell::SelectScenario(EWheelScenario Preset, EWheelControlMode Mode)
{
	if (bRunning) return false;
	Scenario = Preset; ControlMode = Mode;
	SensorBiasMm = FVector::ZeroVector; InitialOffsetMm = FVector::ZeroVector;
	InitialRotationDeg = FRotator::ZeroRotator; FaultWheelIndex = 0;
	bWheelPresent = true; bAlignmentSensorValid = true; bVerificationSensorValid = true;
	switch (Preset)
	{
	case EWheelScenario::SensorBias: SensorBiasMm = FVector(0, 5, 0); break;
	case EWheelScenario::DisplacedWheel: InitialOffsetMm = FVector(20, -30, 10); InitialRotationDeg = FRotator(0, 2, 0); break;
	case EWheelScenario::UnreachableWheel: InitialOffsetMm = FVector(0, -100000, 0); break;
	case EWheelScenario::ReversedWheel: InitialRotationDeg = FRotator(0, 0, 180); break;
	case EWheelScenario::MissingWheel: bWheelPresent = false; break;
	default: break;
	}
	return ResetCycle();
}

FString AWheelAssemblyCell::GetLaneStage(bool bLeft) const
{
	if (bTransporting) return TEXT("차체 도착 대기");
	return StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Lanes[bLeft ? 0 : 1].Stage));
}

void AWheelAssemblyCell::SetTransportProgress(double Progress)
{
	TransportProgress = Progress;
	const FVector Offset = BodyAtStart.TransformVectorNoScale(TransportStartOffsetCm) * (1 - Progress);
	ExpectedVehicle = VehicleAtStation; ExpectedVehicle.AddToTranslation(Offset);
	ExpectedCarriage = CarriageAtStation; ExpectedCarriage.AddToTranslation(Offset);
	Assembly->SetActorTransform(ExpectedVehicle);
	TransportCarriage->SetActorTransform(ExpectedCarriage);
}

void AWheelAssemblyCell::UpdateFastenedWheels()
{
	if (!Assembly || !Assembly->GetBodyFrame()) return;
	for (int32 I=0; I<4; ++I)
		if (FastenedWheels.IsValidIndex(I) && FastenedWheels[I])
			WheelRoots[I]->SetWorldTransform(FastenedInBody[I] * Assembly->GetBodyFrame()->GetComponentTransform());
}
void AWheelAssemblyCell::BeginPlay() { Super::BeginPlay(); if (!bExternallyDriven) ResetCycle(); }
void AWheelAssemblyCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bExternallyDriven) return;
	if (DeltaSeconds > 1 && bRunning) { StopCycle(); Status = TEXT("Large frame stall; reset required."); return; }
	AdvanceSimulation(DeltaSeconds);
}

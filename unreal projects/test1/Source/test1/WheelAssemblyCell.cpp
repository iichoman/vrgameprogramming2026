#include "WheelAssemblyCell.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "UObject/ConstructorHelpers.h"

AWheelAssemblyCell::AWheelAssemblyCell()
{
	PrimaryActorTick.bCanEverTick = true;
	CellRoot = CreateDefaultSubobject<USceneComponent>(TEXT("WheelCellRoot"));
	SetRootComponent(CellRoot);
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
	return FTransform(FRotator::ZeroRotator, FVector(X, Y, HubHeightCm)) * Assembly->GetBodyFrame()->GetComponentTransform();
}

FTransform AWheelAssemblyCell::GetPickupWorld(int32 Index) const
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || Index < 0 || Index > 3) return FTransform::Identity;
	const double X = Index % 2 == 0 ? FrontXcm : RearXcm;
	const double Y = Index < 2 ? -PickupHalfTrackCm : PickupHalfTrackCm;
	return FTransform(FRotator::ZeroRotator, FVector(X, Y, HubHeightCm)) * Assembly->GetBodyFrame()->GetComponentTransform();
}

FTransform AWheelAssemblyCell::GetWheelWorld(int32 Index) const
{
	return WheelRoots.IsValidIndex(Index) && WheelRoots[Index] ? WheelRoots[Index]->GetComponentTransform() : FTransform::Identity;
}

FTransform AWheelAssemblyCell::OffsetOutward(const FTransform& Pose, int32 Side, double Centimeters) const
{
	FTransform Result = Pose;
	Result.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, Side == 0 ? -Centimeters : Centimeters, 0)));
	return Result;
}

bool AWheelAssemblyCell::ValidateSetup()
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || !IsValid(LeftRobot) || !IsValid(RightRobot) || LeftRobot == RightRobot)
	{ Status = TEXT("Assign one vehicle and two distinct calibrated robots."); return false; }
	const double Positive[] = {FrontXcm, HubHalfTrackCm, HubHeightCm, PickupHalfTrackCm, PositionToleranceMm,
		OrientationToleranceDeg, QualityToleranceMm, QualityOrientationDeg, SettleSeconds, FasteningSeconds,
		StageTimeoutSeconds, FixedStepSeconds, MaxPickupDisplacementMm, MaxPickupRotationDeg};
	for (double Value : Positive) if (!FMath::IsFinite(Value) || Value <= 0)
	{ Status = TEXT("Wheel geometry, control times and tolerances must be positive finite values."); return false; }
	if (!FMath::IsFinite(RearXcm) || FixedStepSeconds > 0.02 || FaultWheelIndex < 0 || FaultWheelIndex > 3
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
	if (!ValidateSetup() || !LeftRobot->InitializeArm() || !RightRobot->InitializeArm())
	{ Outcome = EWheelOutcome::Blocked; Status += TEXT(" Robot calibration failed or references missing."); return false; }
	MountedCount = 0; SimulationTimeSeconds = 0; Accumulator = 0;
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
	RefreshQuality();
	Status = TEXT("Ready: Enter starts both arms; Space stops; R resets.");
	return true;
}

bool AWheelAssemblyCell::StartCycle()
{
	if (bRunning || Outcome != EWheelOutcome::NotEvaluated) { Status = TEXT("Reset before starting a new cycle."); return false; }
	if (!ValidateSetup()) { Fail(Status); return false; }
	if (!bAlignmentSensorValid) { Fail(TEXT("Alignment sensor invalid; both robots remain stopped.")); return false; }
	for (int32 Index = 0; Index < 4; ++Index)
	{
		if (Index == FaultWheelIndex && !bWheelPresent) { Fail(TEXT("Pickup blocked: wheel missing.")); return false; }
		const FTransform Actual = GetWheelWorld(Index), Expected = GetPickupWorld(Index);
		if (!Actual.IsValid() || !Actual.GetScale3D().Equals(FVector::OneVector, 1.e-6)
			|| FVector::Distance(Actual.GetLocation(), Expected.GetLocation()) * 10 > MaxPickupDisplacementMm)
		{ Fail(TEXT("Pickup blocked: wheel outside recovery window.")); return false; }
		if (FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Expected.GetRotation())) > MaxPickupRotationDeg)
		{ Fail(TEXT("Pickup blocked: reversed wheel or invalid grasp orientation.")); return false; }
	}
	SimulationTimeSeconds = 0; Accumulator = 0; bRunning = true;
	for (int32 Side = 0; Side < 2; ++Side) if (!BeginWheel(Side)) return false;
	Status = TEXT("Both arms installing front wheels.");
	return true;
}

bool AWheelAssemblyCell::BeginWheel(int32 Side)
{
	FWheelLane& Lane = Lanes[Side];
	Lane.WheelIndex = Side * 2 + Lane.Completed;
	Lane.Pickup = GetWheelWorld(Lane.WheelIndex);
	Lane.bHeld = false; Lane.bFastened = false; Lane.Corrections = 0;
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

void AWheelAssemblyCell::Fail(const FString& Reason)
{
	const FString Saved = Reason;
	bRunning = false; Outcome = EWheelOutcome::Blocked;
	if (LeftRobot) LeftRobot->Halt();
	if (RightRobot) RightRobot->Halt();
	for (FWheelLane& Lane : Lanes) if (Lane.Stage != EWheelStage::Complete) Lane.Stage = EWheelStage::Fault;
	Status = Saved;
}

void AWheelAssemblyCell::StopCycle()
{
	if (!bRunning) return;
	bRunning = false; Outcome = EWheelOutcome::Stopped;
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
	if (!Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip)
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
			SetStage(Side, EWheelStage::Release, TEXT("virtual fastening complete"));
		}
		return;
	}
	if (Lane.Stage == EWheelStage::Release)
	{
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
		const FVector Measured = GetWheelWorld(Lane.WheelIndex).GetLocation() + BodyAtStart.TransformVectorNoScale(SensorBiasMm / 10.0);
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
			if (MountPositionErrorMm[Lane.WheelIndex] > QualityToleranceMm
				|| MountOrientationErrorDeg[Lane.WheelIndex] > QualityOrientationDeg)
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
	if (!bRunning) return true;
	Accumulator += DeltaSeconds;
	while (bRunning && Accumulator + 1.e-10 >= FixedStepSeconds)
	{
		Accumulator -= FixedStepSeconds; SimulationTimeSeconds += FixedStepSeconds;
		if (!IsValid(Assembly) || !Assembly->GetBodyFrame()
			|| !Assembly->GetBodyFrame()->GetComponentTransform().Equals(BodyAtStart, 1.e-6)
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
	return StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Lanes[bLeft ? 0 : 1].Stage));
}

void AWheelAssemblyCell::BeginPlay() { Super::BeginPlay(); ResetCycle(); }
void AWheelAssemblyCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (DeltaSeconds > 1 && bRunning) { StopCycle(); Status = TEXT("Large frame stall; reset required."); return; }
	AdvanceSimulation(DeltaSeconds);
}

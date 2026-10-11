#include "MidtermCameraCell.h"
#include "CamryAssembly.h"
#include "DoorPoseComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "InputCoreTypes.h"
#include "GameFramework/PlayerController.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

AMidtermCameraCell::AMidtermCameraCell()
{
	PrimaryActorTick.bCanEverTick = true;
	// The existing robot Blueprint also binds number keys. Handle demo controls first.
	InputPriority = 1000;
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("DemoRoot"));
	SetRootComponent(Root);
	Camera = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("VirtualCamera"));
	Camera->SetupAttachment(Root);
	Camera->FOVAngle = 60.0f;
	Camera->bCaptureEveryFrame = true;
	Camera->bCaptureOnMovement = true;
	Status = TEXT("Idle: 1 normal, 2 door error, R reset");
}

void AMidtermCameraCell::BeginPlay()
{
	Super::BeginPlay();
	if (APlayerController* Player = GetWorld()->GetFirstPlayerController())
	{
		EnableInput(Player);
		InputComponent->Priority = InputPriority;
		InputComponent->BindKey(EKeys::One, IE_Pressed, this, &AMidtermCameraCell::HandleNormalKey);
		InputComponent->BindKey(EKeys::Two, IE_Pressed, this, &AMidtermCameraCell::HandleFaultKey);
		InputComponent->BindKey(EKeys::NumPadTwo, IE_Pressed, this, &AMidtermCameraCell::HandleFaultKey);
		InputComponent->BindKey(EKeys::P, IE_Pressed, this, &AMidtermCameraCell::HandleFaultKey);
		InputComponent->BindKey(EKeys::R, IE_Pressed, this, &AMidtermCameraCell::HandleResetKey);
	}
	if (Robot)
	{
		Robot->DisableInput(GetWorld()->GetFirstPlayerController());
		Robot->SetActorTickEnabled(false); // Demo owns the six joint commands while playing.
		TArray<USceneComponent*> Components;
		Robot->GetComponents<USceneComponent>(Components);
		for (int32 Index = 1; Index <= 6; ++Index)
		{
			const FName Wanted(*FString::Printf(TEXT("J_A%d"), Index));
			USceneComponent* Found = nullptr;
			for (USceneComponent* Component : Components)
			{
				if (Component && Component->GetFName() == Wanted) { Found = Component; break; }
			}
			Joints.Add(Found);
			RestRotations.Add(Found ? Found->GetRelativeRotation() : FRotator::ZeroRotator);
		}
		for (USceneComponent* Component : Components)
		{
			if (Component && Component->GetFName() == TEXT("TCP_Estimate")) { Tool = Component; break; }
		}
	}
	bSensorReady = ConfigureSensor();
	bCarryReady = ConfigureCarry();
	ResetDemo();
	if (FParse::Param(FCommandLine::Get(), TEXT("MidtermAutoTest")))
	{
		StartNormal();
		for (int32 Step = 0; Step < 100 && State != EMidtermCellState::Complete && State != EMidtermCellState::Stopped; ++Step) Tick(0.1f);
		const bool bNormal = State == EMidtermCellState::Complete && FMath::Abs(CameraMeasuredErrorMm) <= StopThresholdMm;
		StartMisaligned();
		for (int32 Step = 0; Step < 100 && State != EMidtermCellState::Complete && State != EMidtermCellState::Stopped; ++Step) Tick(0.1f);
		const bool bFault = State == EMidtermCellState::Stopped && FMath::Abs(CameraMeasuredErrorMm) > StopThresholdMm;
		ResetDemo();
		UE_LOG(LogTemp, Display, TEXT("MIDTERM_AUTO_TEST %s normal=%d fault=%d pickup_cm=%.2f mount_cm=%.2f"),
			bNormal && bFault && bCarryReady ? TEXT("PASS") : TEXT("FAIL"), bNormal, bFault, PickupReachErrorCm, MountReachErrorCm);
		FPlatformMisc::RequestExit(false);
	}
}

bool AMidtermCameraCell::ConfigureSensor()
{
	if (!Assembly || !Assembly->GetLeftDoor() || !Assembly->GetBodyFrame())
	{
		Status = TEXT("Missing Camry assembly / left door");
		return false;
	}
	UDoorPoseComponent* Pose = Assembly->GetDoorPoseController(ECamryDoorSide::Left);
	if (!Pose || !Pose->HasReferencePose())
	{
		Status = TEXT("Door reference pose is missing");
		return false;
	}
	Pose->RestoreReferencePose();
	DoorPose = Pose;
	ReferenceDoorWorld = Pose->GetReferencePoseInBody() * Assembly->GetBodyFrame()->GetComponentTransform();
	ReferenceMarkerWorld = Assembly->GetLeftDoor()->GetComponentLocation();
	const FTransform Body = Assembly->GetBodyFrame()->GetComponentTransform();
	const FVector CameraWorld = ReferenceMarkerWorld + Body.TransformVectorNoScale(FVector(120, -180, 70));
	Camera->SetWorldLocation(CameraWorld);
	Camera->SetWorldRotation((ReferenceMarkerWorld - CameraWorld).Rotation());
	CameraFeed = NewObject<UTextureRenderTarget2D>(this, TEXT("MidtermCameraFeed"));
	CameraFeed->RenderTargetFormat = RTF_RGBA8;
	CameraFeed->InitAutoFormat(1024, 1024);
	CameraFeed->UpdateResourceImmediate(true);
	Camera->TextureTarget = CameraFeed;
	return true;
}

void AMidtermCameraCell::SetJointAngles(const TArray<double>& Angles)
{
	if (Angles.Num() != 6 || Joints.Num() != 6) return;
	for (int32 Index = 0; Index < 6; ++Index)
	{
		if (!Joints[Index].IsValid()) continue;
		FRotator Offset = FRotator::ZeroRotator;
		if (Index == 0 || Index == 3 || Index == 5) Offset.Yaw = Angles[Index];
		else Offset.Pitch = Angles[Index];
		Joints[Index]->SetRelativeRotation(RestRotations[Index] + Offset);
	}
}

bool AMidtermCameraCell::SolveTarget(const FVector& TargetWorld, TArray<double>& OutAngles, double& OutErrorCm)
{
	if (!Tool.IsValid()) return false;
	TArray<double> Best; Best.Init(0.0, 6);
	OutErrorCm = TNumericLimits<double>::Max();
	const double Seeds[][3] = {{0,0,0},{90,0,0},{-90,0,0},{0,40,-40},{0,-40,40},{90,40,-40},{-90,-40,40}};
	for (const auto& Seed : Seeds)
	{
		TArray<double> Angles; Angles.Init(0.0, 6);
		Angles[0] = Seed[0]; Angles[1] = Seed[1]; Angles[2] = Seed[2];
		for (int32 Iteration = 0; Iteration < 120; ++Iteration)
		{
			SetJointAngles(Angles);
			const FVector Current = Tool->GetComponentLocation();
			const FVector Error = TargetWorld - Current;
			const double Distance = Error.Size();
			if (Distance < OutErrorCm) { OutErrorCm = Distance; Best = Angles; }
			if (Distance < 1.5) break;
			FVector Columns[6];
			for (int32 Axis = 0; Axis < 6; ++Axis)
			{
				Angles[Axis] += 0.5;
				SetJointAngles(Angles);
				Columns[Axis] = (Tool->GetComponentLocation() - Current) * 2.0;
				Angles[Axis] -= 0.5;
			}
			double A[3][4] = {};
			for (int32 Row = 0; Row < 3; ++Row)
			{
				for (int32 Col = 0; Col < 3; ++Col)
				{
					A[Row][Col] = Row == Col ? 0.25 : 0.0;
					for (const FVector& J : Columns) A[Row][Col] += J[Row] * J[Col];
				}
				A[Row][3] = Error[Row];
			}
			for (int32 Pivot = 0; Pivot < 3; ++Pivot)
			{
				const double Divisor = A[Pivot][Pivot];
				if (FMath::Abs(Divisor) < 1.e-9) break;
				for (int32 Col = Pivot; Col < 4; ++Col) A[Pivot][Col] /= Divisor;
				for (int32 Row = 0; Row < 3; ++Row) if (Row != Pivot)
				{
					const double Factor = A[Row][Pivot];
					for (int32 Col = Pivot; Col < 4; ++Col) A[Row][Col] -= Factor * A[Pivot][Col];
				}
			}
			const FVector Correction(A[0][3], A[1][3], A[2][3]);
			for (int32 Axis = 0; Axis < 6; ++Axis)
			{
				Angles[Axis] = FMath::Clamp(Angles[Axis] + FMath::Clamp(FVector::DotProduct(Columns[Axis], Correction), -8.0, 8.0), -150.0, 150.0);
			}
		}
	}
	OutAngles = Best;
	SetJointAngles(Best);
	return OutErrorCm <= 10.0;
}

bool AMidtermCameraCell::ConfigureCarry()
{
	if (!bSensorReady || !Robot || !Tool.IsValid() || !DoorPose.IsValid() || Joints.Num() != 6) return false;
	for (const auto& Joint : Joints) if (!Joint.IsValid()) return false;
	const FVector BodyY = Assembly->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(FVector::RightVector);
	PickupWorld = ReferenceMarkerWorld - BodyY * PickupOutwardDistanceCm;
	Robot->SetActorRotation(FRotator(0.0, 180.0, 0.0));
	const bool bPickup = SolveTarget(PickupWorld + GripOffsetWorld, PickupAngles, PickupReachErrorCm);
	const bool bMount = SolveTarget(ReferenceMarkerWorld + GripOffsetWorld, MountAngles, MountReachErrorCm);
	TArray<double> Rest; Rest.Init(0.0, 6); SetJointAngles(Rest);
	UE_LOG(LogTemp, Display, TEXT("MIDTERM_REACH pickup=%.2fcm mount=%.2fcm"), PickupReachErrorCm, MountReachErrorCm);
	return bPickup && bMount;
}

void AMidtermCameraCell::MoveDoorTo(const FVector& WorldLocation)
{
	if (!DoorPose.IsValid()) return;
	FTransform Pose = ReferenceDoorWorld;
	Pose.SetLocation(WorldLocation);
	DoorPose->SetDoorWorldPose(Pose);
}

bool AMidtermCameraCell::ObserveDoor()
{
	if (!bSensorReady || !Assembly || !Assembly->GetLeftDoor()) return false;
	// Ideal fiducial detector: project the known door landmark into the image plane.
	// The center is simulated rather than found by processing SceneCapture pixels.
	const FVector MarkerWorld = Assembly->GetLeftDoor()->GetComponentLocation();
	const FVector CameraSpace = Camera->GetComponentTransform().InverseTransformPosition(MarkerWorld);
	const double TanHalfFov = FMath::Tan(FMath::DegreesToRadians(Camera->FOVAngle * 0.5));
	if (CameraSpace.X <= 0.0 || FMath::Abs(CameraSpace.Y / CameraSpace.X) > TanHalfFov
		|| FMath::Abs(CameraSpace.Z / CameraSpace.X) > TanHalfFov)
	{
		Status = TEXT("STOP: door landmark outside camera view");
		return false;
	}
	// Estimate a subpixel image center (1/8 pixel), then solve for displacement
	// along the one permitted body-Y motion axis from calibrated bearing.
	constexpr double WidthPx = 1024.0;
	const double NormalizedImageX = CameraSpace.Y / CameraSpace.X;
	const double PixelX = FMath::RoundToDouble((NormalizedImageX / TanHalfFov + 1.0) * 0.5 * WidthPx * 8.0) / 8.0;
	const double MeasuredBearing = (PixelX * 2.0 / WidthPx - 1.0) * TanHalfFov;
	const FTransform CameraTransform = Camera->GetComponentTransform();
	const FVector ReferenceCamera = CameraTransform.InverseTransformPosition(ReferenceMarkerWorld);
	const FVector BodyYWorld = Assembly->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(FVector::RightVector);
	const FVector BodyYCamera = CameraTransform.InverseTransformVectorNoScale(BodyYWorld);
	const double Denominator = BodyYCamera.Y - MeasuredBearing * BodyYCamera.X;
	if (FMath::Abs(Denominator) < 1.e-6)
	{
		Status = TEXT("STOP: camera geometry cannot resolve door Y");
		return false;
	}
	CameraMeasuredErrorMm = 10.0 * (MeasuredBearing * ReferenceCamera.X - ReferenceCamera.Y) / Denominator;
	return FMath::IsFinite(CameraMeasuredErrorMm);
}

void AMidtermCameraCell::Start(bool bMisaligned)
{
	if (!bSensorReady || !bCarryReady) { State = EMidtermCellState::Stopped; Status = TEXT("STOP: camera or robot reach not ready"); return; }
	ResetDemo();
	bFaultRequested = bMisaligned;
	State = EMidtermCellState::ApproachPickup;
	StateSeconds = 0.0;
	Status = TEXT("Robot approaching staged door");
	UE_LOG(LogTemp, Display, TEXT("MIDTERM_DEMO_INPUT scenario=%s"), bMisaligned ? TEXT("fault") : TEXT("normal"));
}

void AMidtermCameraCell::StartNormal() { Start(false); }
void AMidtermCameraCell::StartMisaligned() { Start(true); }
void AMidtermCameraCell::HandleNormalKey() { StartNormal(); }
void AMidtermCameraCell::HandleFaultKey() { StartMisaligned(); }
void AMidtermCameraCell::HandleResetKey() { ResetDemo(); }

void AMidtermCameraCell::ResetDemo()
{
	if (DoorPose.IsValid()) MoveDoorTo(PickupWorld);
	TArray<double> Rest; Rest.Init(0.0, 6); SetJointAngles(Rest);
	State = EMidtermCellState::Idle;
	StateSeconds = 0.0;
	CameraMeasuredErrorMm = 0.0;
	bGripped = false;
	bFaultRequested = false;
	Status = bSensorReady && bCarryReady ? TEXT("Door staged: 1 normal, 2 fault, R reset") : TEXT("STOP: camera or robot reach not ready");
}

void AMidtermCameraCell::UpdateSequence(float DeltaSeconds)
{
	StateSeconds += DeltaSeconds;
	if (State == EMidtermCellState::ApproachPickup || State == EMidtermCellState::Carry)
	{
		const bool bCarrying = State == EMidtermCellState::Carry;
		const double Alpha = FMath::Clamp(StateSeconds / (bCarrying ? 3.0 : 2.0), 0.0, 1.0);
		TArray<double> Angles; Angles.Init(0.0, 6);
		for (int32 Index = 0; Index < 6; ++Index)
			Angles[Index] = bCarrying ? FMath::Lerp(PickupAngles[Index], MountAngles[Index], Alpha) : FMath::Lerp(0.0, PickupAngles[Index], Alpha);
		SetJointAngles(Angles);
		if (bGripped && Tool.IsValid()) MoveDoorTo(Tool->GetComponentLocation() - GripOffsetWorld);
		if (Alpha >= 1.0)
		{
			State = bCarrying ? EMidtermCellState::Verify : EMidtermCellState::Grip;
			StateSeconds = 0.0;
			Status = bCarrying ? TEXT("Door at body; camera checking alignment") : TEXT("Robot at pickup; gripping door");
			if (bCarrying && DoorPose.IsValid())
			{
				DoorPose->ApplyReferenceOffsetMm(FVector(0, bFaultRequested ? -InjectedErrorMm : 0.0, 0));
				bGripped = false;
			}
		}
	}
	else if (State == EMidtermCellState::Grip && StateSeconds >= 0.5)
	{
		bGripped = true;
		State = EMidtermCellState::Carry; StateSeconds = 0.0;
		Status = TEXT("Door gripped; carrying to body");
	}
	else if (State == EMidtermCellState::Verify && StateSeconds >= 0.25)
	{
		if (!ObserveDoor()) State = EMidtermCellState::Stopped;
		else if (FMath::Abs(CameraMeasuredErrorMm) > StopThresholdMm)
		{
			State = EMidtermCellState::Stopped;
			Status = FString::Printf(TEXT("STOP: door Y error %.2f mm > %.2f mm"), CameraMeasuredErrorMm, StopThresholdMm);
		}
		else { State = EMidtermCellState::Complete; Status = TEXT("Door placed at reference pose; camera PASS"); }
	}
}

void AMidtermCameraCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (State != EMidtermCellState::Idle && State != EMidtermCellState::Complete && State != EMidtermCellState::Stopped) UpdateSequence(DeltaSeconds);
	ShowStatus();
}

void AMidtermCameraCell::ShowStatus() const
{
	if (GEngine && GetWorld() && GetWorld()->IsGameWorld())
	{
		const bool bStopped = State == EMidtermCellState::Stopped;
		const FString Message = FString::Printf(TEXT("%s | %s | measured Y %.2f mm | limit %.2f mm | 1 normal 2/P fault R reset"),
			bStopped ? TEXT("SAFETY INTERLOCK: STOP") : TEXT("CAMERA CELL"),
			*Status, CameraMeasuredErrorMm, StopThresholdMm);
		GEngine->AddOnScreenDebugMessage(72001, 0.0f, bStopped ? FColor::Red : FColor::Cyan,
			Message, true, FVector2D(1.5f, 1.5f));
	}
}

void AMidtermCameraCell::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}

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
	}
	bSensorReady = ConfigureSensor();
	ResetDemo();
	if (FParse::Param(FCommandLine::Get(), TEXT("MidtermAutoTest")))
	{
		StartNormal();
		Tick(0.3f);
		const bool bNormal = State == EMidtermCellState::UpperHinge && FMath::Abs(CameraMeasuredErrorMm) <= StopThresholdMm;
		StartMisaligned();
		Tick(0.3f);
		const bool bFault = State == EMidtermCellState::Stopped && FMath::Abs(CameraMeasuredErrorMm) > StopThresholdMm;
		ResetDemo();
		UE_LOG(LogTemp, Display, TEXT("MIDTERM_AUTO_TEST %s normal=%d fault=%d joints=%d"),
			bNormal && bFault && Joints.Num() == 6 ? TEXT("PASS") : TEXT("FAIL"), bNormal, bFault, Joints.Num());
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
	if (!bSensorReady || !Assembly) { State = EMidtermCellState::Stopped; Status = TEXT("STOP: sensor not ready"); return; }
	UDoorPoseComponent* Pose = Assembly->GetDoorPoseController(ECamryDoorSide::Left);
	if (!Pose || !Pose->ApplyReferenceOffsetMm(FVector(0, bMisaligned ? -InjectedErrorMm : 0.0, 0)))
	{
		State = EMidtermCellState::Stopped; Status = TEXT("STOP: failed to position door"); return;
	}
	SetJointPose(0.0, false);
	State = EMidtermCellState::Verify;
	StateSeconds = 0.0;
	Status = bMisaligned
		? FString::Printf(TEXT("FAULT REQUESTED: door Y offset -%.1f mm; camera verifying"), InjectedErrorMm)
		: TEXT("NORMAL REQUESTED: camera verifying door position");
	UE_LOG(LogTemp, Display, TEXT("MIDTERM_DEMO_INPUT scenario=%s door_y_offset_mm=%.2f"),
		bMisaligned ? TEXT("fault") : TEXT("normal"), bMisaligned ? -InjectedErrorMm : 0.0);
}

void AMidtermCameraCell::StartNormal() { Start(false); }
void AMidtermCameraCell::StartMisaligned() { Start(true); }
void AMidtermCameraCell::HandleNormalKey() { StartNormal(); }
void AMidtermCameraCell::HandleFaultKey() { StartMisaligned(); }
void AMidtermCameraCell::HandleResetKey() { ResetDemo(); }

void AMidtermCameraCell::ResetDemo()
{
	if (Assembly)
	{
		if (UDoorPoseComponent* Pose = Assembly->GetDoorPoseController(ECamryDoorSide::Left)) Pose->RestoreReferencePose();
	}
	SetJointPose(0.0, false);
	State = EMidtermCellState::Idle;
	StateSeconds = 0.0;
	CameraMeasuredErrorMm = 0.0;
	Status = bSensorReady ? TEXT("Idle: 1 normal, 2 door error, R reset") : TEXT("STOP: sensor not ready");
}

void AMidtermCameraCell::SetJointPose(double Alpha, bool bLower)
{
	if (Joints.Num() != 6 || RestRotations.Num() != 6) return;
	// Small illustrative motion only. Exact hinge TCP poses still need calibration.
	for (int32 Index = 0; Index < 6; ++Index)
	{
		if (!Joints[Index].IsValid()) continue;
		FRotator Offset = FRotator::ZeroRotator;
		if (Index == 0) Offset.Yaw = bLower ? FMath::Lerp(11.0, 17.0, Alpha) : 11.0;
		if (Index == 1) Offset.Roll = bLower ? FMath::Lerp(8.0, -8.0, Alpha) : 8.0;
		if (Index == 2) Offset.Roll = bLower ? FMath::Lerp(-6.0, 6.0, Alpha) : -6.0;
		Joints[Index]->SetRelativeRotation(RestRotations[Index] + Offset * (bLower ? 1.0 : Alpha));
	}
}

void AMidtermCameraCell::DriveRobot(float DeltaSeconds)
{
	StateSeconds += DeltaSeconds;
	if (State == EMidtermCellState::UpperHinge)
	{
		SetJointPose(FMath::Clamp(StateSeconds / 2.0, 0.0, 1.0), false);
		if (StateSeconds >= 2.0) { State = EMidtermCellState::LowerHinge; StateSeconds = 0.0; Status = TEXT("Upper station complete; moving to lower station"); }
	}
	else if (State == EMidtermCellState::LowerHinge)
	{
		SetJointPose(FMath::Clamp(StateSeconds / 2.0, 0.0, 1.0), true);
		if (StateSeconds >= 2.0) { State = EMidtermCellState::Complete; Status = TEXT("Demo sequence complete (fastening events only)"); }
	}
}

void AMidtermCameraCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (State == EMidtermCellState::Verify)
	{
		StateSeconds += DeltaSeconds;
		if (StateSeconds >= 0.25)
		{
			if (!ObserveDoor()) { State = EMidtermCellState::Stopped; }
			else if (FMath::Abs(CameraMeasuredErrorMm) > StopThresholdMm)
			{
				State = EMidtermCellState::Stopped;
				Status = FString::Printf(TEXT("STOP: camera error %.2f mm > %.2f mm"), CameraMeasuredErrorMm, StopThresholdMm);
			}
			else { State = EMidtermCellState::UpperHinge; StateSeconds = 0.0; Status = TEXT("Camera PASS; upper station"); }
		}
	}
	else if (State == EMidtermCellState::UpperHinge || State == EMidtermCellState::LowerHinge) DriveRobot(DeltaSeconds);
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

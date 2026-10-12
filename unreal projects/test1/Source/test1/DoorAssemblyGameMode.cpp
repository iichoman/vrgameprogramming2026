#include "DoorAssemblyGameMode.h"

#include "DoorAssemblyCell.h"
#include "CamryAssembly.h"
#include "Camera/CameraActor.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SOverlay.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
	bool CanStopCell(const ADoorAssemblyCell* Cell)
	{
		return IsValid(Cell) && Cell->State != EDoorAssemblyState::Idle
			&& Cell->State != EDoorAssemblyState::Complete && Cell->State != EDoorAssemblyState::Fault
			&& Cell->State != EDoorAssemblyState::Stopped;
	}

	FText CellStatus(const TWeakObjectPtr<ADoorAssemblyPlayerController>& Controller)
	{
		const ADoorAssemblyCell* Cell = Controller.IsValid() ? Controller->GetAssemblyCell() : nullptr;
		return FText::FromString(IsValid(Cell) ? Cell->Status : TEXT("No DoorAssemblyCell found in this map."));
	}

	FString DoorMountMeasurements(const ADoorAssemblyCell* Cell)
	{
		if (!IsValid(Cell) || !IsValid(Cell->Assembly) || !IsValid(Cell->Assembly->GetLeftDoor()))
		{
			return TEXT("Door mount: unavailable");
		}
		const FTransform Door = Cell->Assembly->GetLeftDoor()->GetComponentTransform();
		const FTransform Mount = Cell->GetMountDoorWorld();
		return FString::Printf(TEXT("Door mount: %.2f mm / %.2f deg"),
			FVector::Distance(Door.GetLocation(), Mount.GetLocation()) * 10.0,
			FMath::RadiansToDegrees(Door.GetRotation().AngularDistance(Mount.GetRotation())));
	}

	FText CellMeasurements(const TWeakObjectPtr<ADoorAssemblyPlayerController>& Controller)
	{
		const ADoorAssemblyCell* Cell = Controller.IsValid() ? Controller->GetAssemblyCell() : nullptr;
		if (!IsValid(Cell))
		{
			return FText::FromString(TEXT("Waiting for assembly cell..."));
		}
		FString Measurements = FString::Printf(
			TEXT("State: %s\nElapsed: %.2f s\nTCP position error: %.2f mm\nTCP orientation error: %.2f deg\nDoor held: %s    Fastened: %s"),
			*Cell->GetStateLabel(), Cell->SimulationTimeSeconds, Cell->PositionErrorMm,
			Cell->OrientationErrorDeg, Cell->bDoorHeld ? TEXT("YES") : TEXT("NO"),
			Cell->bDoorFastened ? TEXT("YES") : TEXT("NO"));
		Measurements += TEXT("\n") + DoorMountMeasurements(Cell);
		return FText::FromString(Measurements);
	}

	class SDoorAssemblyPanel : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SDoorAssemblyPanel) {}
			SLATE_ARGUMENT(TWeakObjectPtr<ADoorAssemblyPlayerController>, Controller)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs)
		{
			Controller = InArgs._Controller;
			ButtonTextStyle = FCoreStyle::Get().GetWidgetStyle<FTextBlockStyle>("NormalText");
			ButtonTextStyle.SetFont(FCoreStyle::GetDefaultFontStyle("Bold", 22));
			const TWeakObjectPtr<ADoorAssemblyPlayerController> WeakController = Controller;
			const auto HasCell = [WeakController]()
			{
				return WeakController.IsValid() && IsValid(WeakController->GetAssemblyCell());
			};
			const auto CanStart = [WeakController]()
			{
				const ADoorAssemblyCell* Cell = WeakController.IsValid() ? WeakController->GetAssemblyCell() : nullptr;
				return IsValid(Cell) && Cell->State == EDoorAssemblyState::Idle;
			};
			const auto CanStop = [WeakController]()
			{
				return CanStopCell(WeakController.IsValid() ? WeakController->GetAssemblyCell() : nullptr);
			};

			ChildSlot
			[
				SNew(SOverlay)
				.Visibility(EVisibility::SelfHitTestInvisible)
				+ SOverlay::Slot()
				.HAlign(HAlign_Left)
				.VAlign(VAlign_Top)
				.Padding(18.0f)
				[
					SNew(SBox)
					// At the presentation viewport's 0.7 UI scale this is about 380 screen pixels.
					.WidthOverride(540.0f)
					[
						SNew(SBorder)
						.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
						.BorderBackgroundColor(FLinearColor(0.025f, 0.04f, 0.055f, 0.93f))
						.Padding(16.0f)
						[
							SNew(SVerticalBox)
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("DOOR ASSEMBLY CELL")))
								.Font(FCoreStyle::GetDefaultFontStyle("Bold", 30))
								.ColorAndOpacity(FLinearColor::White)
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 5.0f, 0.0f, 12.0f)
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("6-axis robot / measured motion and alignment")))
								.AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 18))
								.ColorAndOpacity(FLinearColor(0.6f, 0.83f, 0.95f))
							]
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0.0f, 0.0f, 6.0f, 0.0f)
								[
									SNew(SButton)
									.IsFocusable(false)
									.Text(FText::FromString(TEXT("Start")))
									.TextStyle(&ButtonTextStyle)
									.HAlign(HAlign_Center)
									.IsEnabled_Lambda(CanStart)
									.OnClicked_Lambda([WeakController]()
									{
										if (WeakController.IsValid()) WeakController->StartAssembly();
										return FReply::Handled();
									})
								]
								+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0.0f, 0.0f, 6.0f, 0.0f)
								[
									SNew(SButton)
									.IsFocusable(false)
									.Text(FText::FromString(TEXT("Stop")))
									.TextStyle(&ButtonTextStyle)
									.HAlign(HAlign_Center)
									.IsEnabled_Lambda(CanStop)
									.OnClicked_Lambda([WeakController]()
									{
										if (WeakController.IsValid()) WeakController->StopAssembly();
										return FReply::Handled();
									})
								]
								+ SHorizontalBox::Slot().FillWidth(1.0f)
								[
									SNew(SButton)
									.IsFocusable(false)
									.Text(FText::FromString(TEXT("Reset")))
									.TextStyle(&ButtonTextStyle)
									.HAlign(HAlign_Center)
									.IsEnabled_Lambda(HasCell)
									.OnClicked_Lambda([WeakController]()
									{
										if (WeakController.IsValid()) WeakController->ResetAssembly();
										return FReply::Handled();
									})
								]
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 14.0f, 0.0f, 10.0f)
							[
								SNew(STextBlock)
								.Text_Lambda([WeakController]() { return CellMeasurements(WeakController); })
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 22))
								.ColorAndOpacity(FLinearColor::White)
							]
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(STextBlock)
								.Text_Lambda([WeakController]() { return CellStatus(WeakController); })
								.AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 20))
								.ColorAndOpacity(FLinearColor(0.96f, 0.8f, 0.46f))
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 0.0f)
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("TCP errors: active motion target.\nDoor mount: saved nominal assembly pose.\nEnter: start / Space: stop / R: reset\nRight-drag: orbit / Wheel: zoom / Home: view")))
								.AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 18))
								.ColorAndOpacity(FLinearColor(0.72f, 0.76f, 0.8f))
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 0.0f)
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("PID velocity servo with simplified motor response; kinematic grasp / ideal rigid fastening. Contact forces and panel deformation not modeled.")))
								.AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 16))
								.ColorAndOpacity(FLinearColor(0.63f, 0.68f, 0.73f))
							]
						]
					]
				]
			];
		}

	private:
		TWeakObjectPtr<ADoorAssemblyPlayerController> Controller;
		FTextBlockStyle ButtonTextStyle;
	};
}

ADoorAssemblyPlayerController::ADoorAssemblyPlayerController()
{
	bAutoManageActiveCameraTarget = false;
	bShowMouseCursor = true;
	bEnableClickEvents = true;
}

void ADoorAssemblyPlayerController::BeginPlay()
{
	Super::BeginPlay();
	if (!IsLocalController()) return;

	FindAssemblyCell();
	AssemblyCamera = GetWorld()->SpawnActor<ACameraActor>();
	if (AssemblyCamera)
	{
		ResetCamera();
		SetViewTarget(AssemblyCamera);
	}

	if (GEngine && GEngine->GameViewport)
	{
		AssemblyPanel = SNew(SDoorAssemblyPanel)
			.Visibility(EVisibility::SelfHitTestInvisible)
			.Controller(TWeakObjectPtr<ADoorAssemblyPlayerController>(this));
		GEngine->GameViewport->AddViewportWidgetContent(AssemblyPanel.ToSharedRef(), 10);
	}

	FInputModeGameAndUI InputMode;
	InputMode.SetHideCursorDuringCapture(false);
	InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	SetInputMode(InputMode);
}

void ADoorAssemblyPlayerController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (AssemblyPanel.IsValid() && GEngine && GEngine->GameViewport)
	{
		GEngine->GameViewport->RemoveViewportWidgetContent(AssemblyPanel.ToSharedRef());
	}
	AssemblyPanel.Reset();
	if (IsValid(AssemblyCamera)) AssemblyCamera->Destroy();
	Super::EndPlay(EndPlayReason);
}

void ADoorAssemblyPlayerController::SetupInputComponent()
{
	Super::SetupInputComponent();
	if (!InputComponent) return;
	InputComponent->BindKey(EKeys::Enter, IE_Pressed, this, &ADoorAssemblyPlayerController::StartAssembly);
	InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &ADoorAssemblyPlayerController::StopAssembly);
	InputComponent->BindKey(EKeys::R, IE_Pressed, this, &ADoorAssemblyPlayerController::ResetAssembly);
	InputComponent->BindKey(EKeys::Home, IE_Pressed, this, &ADoorAssemblyPlayerController::ResetCamera);
	InputComponent->BindKey(EKeys::MouseScrollUp, IE_Pressed, this, &ADoorAssemblyPlayerController::ZoomIn);
	InputComponent->BindKey(EKeys::MouseScrollDown, IE_Pressed, this, &ADoorAssemblyPlayerController::ZoomOut);
}

void ADoorAssemblyPlayerController::PlayerTick(float DeltaSeconds)
{
	Super::PlayerTick(DeltaSeconds);
	if (!IsLocalController()) return;
	if (!IsValid(AssemblyCell)) FindAssemblyCell();
	if (IsInputKeyDown(EKeys::RightMouseButton))
	{
		float MouseX = 0.0f;
		float MouseY = 0.0f;
		if (GetMousePosition(MouseX, MouseY))
		{
			const FVector2D Position(MouseX, MouseY);
			if (bOrbitDragging)
			{
				const FVector2D Delta = Position - LastMousePosition;
				OrbitRotation.Yaw -= Delta.X * 0.25;
				OrbitRotation.Pitch = FMath::Clamp(OrbitRotation.Pitch + Delta.Y * 0.25, 3.0, 80.0);
				UpdateCamera();
			}
			LastMousePosition = Position;
			bOrbitDragging = true;
		}
	}
	else
	{
		bOrbitDragging = false;
	}
}

void ADoorAssemblyPlayerController::FindAssemblyCell()
{
	for (TActorIterator<ADoorAssemblyCell> It(GetWorld()); It; ++It)
	{
		AssemblyCell = *It;
		break;
	}
}

void ADoorAssemblyPlayerController::StartAssembly()
{
	if (IsValid(AssemblyCell) && AssemblyCell->State == EDoorAssemblyState::Idle) AssemblyCell->StartCycle();
}

void ADoorAssemblyPlayerController::StopAssembly()
{
	if (CanStopCell(AssemblyCell.Get())) AssemblyCell->StopCycle();
}

void ADoorAssemblyPlayerController::ResetAssembly()
{
	if (IsValid(AssemblyCell)) AssemblyCell->ResetCycle();
}

void ADoorAssemblyPlayerController::ResetCamera()
{
	const FVector Offset = FVector(320.0, -620.0, 300.0) - CameraFocus;
	OrbitDistance = Offset.Length();
	OrbitRotation = Offset.Rotation();
	UpdateCamera();
}

void ADoorAssemblyPlayerController::UpdateCamera()
{
	if (!IsValid(AssemblyCamera)) return;
	const FVector Location = CameraFocus + OrbitRotation.Vector() * OrbitDistance;
	AssemblyCamera->SetActorLocationAndRotation(Location, (CameraFocus - Location).Rotation());
}

void ADoorAssemblyPlayerController::ZoomIn()
{
	OrbitDistance = FMath::Max(260.0, OrbitDistance * 0.9);
	UpdateCamera();
}

void ADoorAssemblyPlayerController::ZoomOut()
{
	OrbitDistance = FMath::Min(1800.0, OrbitDistance / 0.9);
	UpdateCamera();
}

ADoorAssemblyGameMode::ADoorAssemblyGameMode()
{
	PlayerControllerClass = ADoorAssemblyPlayerController::StaticClass();
	DefaultPawnClass = nullptr;
	HUDClass = nullptr;
}

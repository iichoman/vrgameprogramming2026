#include "WheelAssemblyGameMode.h"
#include "WheelAssemblyCell.h"
#include "Camera/CameraActor.h"
#include "Components/InputComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SOverlay.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
	FString OutcomeLabel(EWheelOutcome Outcome)
	{
		switch (Outcome)
		{
		case EWheelOutcome::Pass: return TEXT("합격");
		case EWheelOutcome::QualityFail: return TEXT("품질 불합격");
		case EWheelOutcome::Blocked: return TEXT("작업 차단");
		case EWheelOutcome::Stopped: return TEXT("정지");
		default: return TEXT("미판정");
		}
	}

	FString ScenarioLabel(EWheelScenario Scenario)
	{
		switch (Scenario)
		{
		case EWheelScenario::Normal: return TEXT("정상");
		case EWheelScenario::SensorBias: return TEXT("센서 오차");
		case EWheelScenario::DisplacedWheel: return TEXT("휠 위치 틀어짐");
		case EWheelScenario::UnreachableWheel: return TEXT("범위 초과");
		case EWheelScenario::ReversedWheel: return TEXT("휠 뒤집힘");
		case EWheelScenario::MissingWheel: return TEXT("휠 없음");
		default: return TEXT("사용자 설정");
		}
	}

	TSharedRef<SWidget> PresetButton(TWeakObjectPtr<AWheelAssemblyPlayerController> Controller,
		const TCHAR* Label, EWheelScenario Scenario, EWheelControlMode Mode)
	{
		return SNew(SButton).IsFocusable(false).Text(FText::FromString(Label))
			.OnClicked_Lambda([Controller, Scenario, Mode]()
			{
			if (Controller.IsValid() && IsValid(Controller->GetCell())) Controller->GetCell()->SelectScenario(Scenario, Mode);
			return FReply::Handled();
			});
	}

	class SWheelAssemblyPanel : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SWheelAssemblyPanel) {}
			SLATE_ARGUMENT(TWeakObjectPtr<AWheelAssemblyPlayerController>, Controller)
		SLATE_END_ARGS()
		void Construct(const FArguments& Args)
		{
			Controller = Args._Controller;
			ChildSlot
			[
				SNew(SBorder).Padding(16).BorderBackgroundColor(FLinearColor(0.015f, 0.025f, 0.04f, 0.92f))
				[
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
						[SNew(STextBlock).Text(FText::FromString(TEXT("양쪽 로봇팔 · 4륜 조립"))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 24))]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[SNew(SButton).Text(FText::FromString(TEXT("시작"))).OnClicked_Lambda([this](){ if (Controller.IsValid()) Controller->StartAssembly(); return FReply::Handled(); })]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[SNew(SButton).Text(FText::FromString(TEXT("정지"))).OnClicked_Lambda([this](){ if (Controller.IsValid()) Controller->StopAssembly(); return FReply::Handled(); })]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[SNew(SButton).Text(FText::FromString(TEXT("초기화"))).OnClicked_Lambda([this](){ if (Controller.IsValid()) Controller->ResetAssembly(); return FReply::Handled(); })]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("정상"), EWheelScenario::Normal, EWheelControlMode::IndependentVerification)]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("센서 오차 · 기본"), EWheelScenario::SensorBias, EWheelControlMode::Basic)]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("센서 오차 · 검증"), EWheelScenario::SensorBias, EWheelControlMode::IndependentVerification)]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("위치 틀어짐"), EWheelScenario::DisplacedWheel, EWheelControlMode::IndependentVerification)]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("범위 초과"), EWheelScenario::UnreachableWheel, EWheelControlMode::IndependentVerification)]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("뒤집힘"), EWheelScenario::ReversedWheel, EWheelControlMode::IndependentVerification)]
							+ SHorizontalBox::Slot().AutoWidth().Padding(2)[PresetButton(Controller, TEXT("휠 없음"), EWheelScenario::MissingWheel, EWheelControlMode::IndependentVerification)]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
						[
							SNew(STextBlock).AutoWrapText(true).Text_Lambda([this]()
							{
								const AWheelAssemblyCell* Cell = Controller.IsValid() ? Controller->GetCell() : nullptr;
								if (!IsValid(Cell)) return FText::FromString(TEXT("휠 조립 셀을 찾을 수 없습니다."));
								return FText::FromString(FString::Printf(TEXT("상황: %s / 제어: %s / 결과: %s\n좌측: %s    우측: %s\n장착: %d/4    경과: %.2f초\n%s"),
									*ScenarioLabel(Cell->Scenario), Cell->ControlMode == EWheelControlMode::Basic ? TEXT("기본") : TEXT("독립 검증"),
									*OutcomeLabel(Cell->Outcome), *Cell->GetLaneStage(true), *Cell->GetLaneStage(false),
									Cell->MountedCount, Cell->SimulationTimeSeconds,
									*(Cell->bEnableTransport ? FString::Printf(TEXT("차체 운반 %.0f%% · %s"), Cell->TransportProgress * 100, *Cell->Status) : Cell->Status)));
							})
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
						[
							SNew(STextBlock).AutoWrapText(true).Text_Lambda([this]()
							{
								const AWheelAssemblyCell* Cell = Controller.IsValid() ? Controller->GetCell() : nullptr;
								if (!IsValid(Cell) || Cell->MountPositionErrorMm.Num() != 4) return FText::GetEmpty();
								FString Report = TEXT("장착 오차 (위치 mm / 방향 도):");
								const TCHAR* Names[] = {TEXT("좌측 앞"), TEXT("좌측 뒤"), TEXT("우측 앞"), TEXT("우측 뒤")};
								for (int32 Index = 0; Index < 4; ++Index)
									Report += FString::Printf(TEXT("\n%s  %.2f / %.2f  %s"), Names[Index], Cell->MountPositionErrorMm[Index],
										Cell->MountOrientationErrorDeg[Index], Cell->FastenedWheels[Index] ? TEXT("체결") : TEXT("미체결"));
								return FText::FromString(Report);
							})
						]
						+ SVerticalBox::Slot().AutoHeight()
						[SNew(STextBlock).AutoWrapText(true).Text(FText::FromString(TEXT("Enter 시작 / Space 정지 / R 초기화 · 우클릭 시점 회전 / 휠 확대\n임시 휠 형상과 가상 센서·체결을 사용합니다. 이송 장치는 후속 연결 대상입니다.")))]
					]
				]
			];
		}
	private:
		TWeakObjectPtr<AWheelAssemblyPlayerController> Controller;
	};
}

AWheelAssemblyPlayerController::AWheelAssemblyPlayerController()
{
	bAutoManageActiveCameraTarget = false;
	bShowMouseCursor = true;
	bEnableClickEvents = true;
}

void AWheelAssemblyPlayerController::BeginPlay()
{
	Super::BeginPlay();
	if (!IsLocalController()) return;
	for (TActorIterator<AWheelAssemblyCell> It(GetWorld()); It; ++It) { Cell = *It; break; }
	AssemblyCamera = GetWorld()->SpawnActor<ACameraActor>();
	if (AssemblyCamera) { ResetCamera(); SetViewTarget(AssemblyCamera); }
	if (GEngine && GEngine->GameViewport)
	{
		Panel = SNew(SWheelAssemblyPanel).Visibility(EVisibility::SelfHitTestInvisible)
			.Controller(TWeakObjectPtr<AWheelAssemblyPlayerController>(this));
		GEngine->GameViewport->AddViewportWidgetContent(Panel.ToSharedRef(), 10);
	}
	FInputModeGameAndUI InputMode;
	InputMode.SetHideCursorDuringCapture(false);
	InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	SetInputMode(InputMode);
}

void AWheelAssemblyPlayerController::EndPlay(const EEndPlayReason::Type Reason)
{
	if (Panel.IsValid() && GEngine && GEngine->GameViewport) GEngine->GameViewport->RemoveViewportWidgetContent(Panel.ToSharedRef());
	Panel.Reset();
	if (IsValid(AssemblyCamera)) AssemblyCamera->Destroy();
	Super::EndPlay(Reason);
}

void AWheelAssemblyPlayerController::SetupInputComponent()
{
	Super::SetupInputComponent();
	if (!InputComponent) return;
	InputComponent->BindKey(EKeys::Enter, IE_Pressed, this, &AWheelAssemblyPlayerController::StartAssembly);
	InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &AWheelAssemblyPlayerController::StopAssembly);
	InputComponent->BindKey(EKeys::R, IE_Pressed, this, &AWheelAssemblyPlayerController::ResetAssembly);
	InputComponent->BindKey(EKeys::Home, IE_Pressed, this, &AWheelAssemblyPlayerController::ResetCamera);
	InputComponent->BindKey(EKeys::MouseScrollUp, IE_Pressed, this, &AWheelAssemblyPlayerController::ZoomIn);
	InputComponent->BindKey(EKeys::MouseScrollDown, IE_Pressed, this, &AWheelAssemblyPlayerController::ZoomOut);
}

void AWheelAssemblyPlayerController::PlayerTick(float DeltaSeconds)
{
	Super::PlayerTick(DeltaSeconds);
	if (!IsLocalController()) return;
	if (IsInputKeyDown(EKeys::RightMouseButton))
	{
		float X = 0, Y = 0;
		if (GetMousePosition(X, Y))
		{
			const FVector2D Position(X, Y);
			if (bDragging)
			{
				const FVector2D Delta = Position - LastMouse;
				Orbit.Yaw -= Delta.X * 0.25;
				Orbit.Pitch = FMath::Clamp(Orbit.Pitch + Delta.Y * 0.25, 4.0, 80.0);
				UpdateCamera();
			}
			LastMouse = Position; bDragging = true;
		}
	}
	else bDragging = false;
}

void AWheelAssemblyPlayerController::StartAssembly() { if (IsValid(Cell)) Cell->StartCycle(); }
void AWheelAssemblyPlayerController::StopAssembly() { if (IsValid(Cell)) Cell->StopCycle(); }
void AWheelAssemblyPlayerController::ResetAssembly() { if (IsValid(Cell)) Cell->ResetCycle(); }
void AWheelAssemblyPlayerController::ResetCamera()
{
	const bool bTransport = IsValid(Cell) && Cell->bEnableTransport;
	Focus = bTransport ? FVector(-250, 0, 70) : FVector(0, 0, 70);
	const FVector Offset = bTransport ? FVector(850, -1200, 650) : FVector(430, -650, 320);
	Distance = Offset.Length(); Orbit = Offset.Rotation(); UpdateCamera();
}
void AWheelAssemblyPlayerController::UpdateCamera()
{
	if (!IsValid(AssemblyCamera)) return;
	const FVector Location = Focus + Orbit.Vector() * Distance;
	AssemblyCamera->SetActorLocationAndRotation(Location, (Focus - Location).Rotation());
}
void AWheelAssemblyPlayerController::ZoomIn() { Distance = FMath::Max(300.0, Distance * 0.9); UpdateCamera(); }
void AWheelAssemblyPlayerController::ZoomOut() { Distance = FMath::Min(1800.0, Distance / 0.9); UpdateCamera(); }

AWheelAssemblyGameMode::AWheelAssemblyGameMode()
{
	PlayerControllerClass = AWheelAssemblyPlayerController::StaticClass();
	DefaultPawnClass = nullptr;
	HUDClass = nullptr;
}

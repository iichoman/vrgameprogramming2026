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
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SOverlay.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
	// Display-only translations keep CSV/JSON identifiers and control diagnostics stable.
	FString KoreanLabel(const FString& Value)
	{
		static const TMap<FString, FString> Labels = {
			{TEXT("FrontLower"), TEXT("전방 하단")},
			{TEXT("FrontMiddle"), TEXT("전방 중앙")},
			{TEXT("FrontUpper"), TEXT("전방 상단")},
			{TEXT("RearLower"), TEXT("후방 하단")},
			{TEXT("RearMiddle"), TEXT("후방 중앙")},
			{TEXT("RearUpper"), TEXT("후방 상단")},
			{TEXT("Idle"), TEXT("대기")},
			{TEXT("ApproachPickup"), TEXT("문 파지 위치 접근")},
			{TEXT("Grip"), TEXT("문 파지")},
			{TEXT("Lift"), TEXT("문 들어 올리기")},
			{TEXT("Transfer"), TEXT("문 운반")},
			{TEXT("ApproachMount"), TEXT("장착 위치 접근")},
			{TEXT("Align"), TEXT("정밀 정렬")},
			{TEXT("Fasten"), TEXT("체결")},
			{TEXT("Release"), TEXT("파지 해제")},
			{TEXT("Retract"), TEXT("로봇 후퇴")},
			{TEXT("Complete"), TEXT("완료")},
			{TEXT("Stopped"), TEXT("정지")},
			{TEXT("Fault"), TEXT("오류 정지")},
			{TEXT("Basic"), TEXT("기본 제어")},
			{TEXT("IndependentVerification"), TEXT("독립 검증 제어")},
			{TEXT("Normal"), TEXT("정상")},
			{TEXT("SensorBias"), TEXT("센서 오차")},
			{TEXT("DisplacedDoor"), TEXT("배치 틀어짐")},
			{TEXT("UnreachableDoor"), TEXT("범위 초과")},
			{TEXT("ReversedDoor"), TEXT("문 뒤집힘")},
			{TEXT("MissingDoor"), TEXT("문 없음")},
			{TEXT("Custom"), TEXT("사용자 설정")},
			{TEXT("NotEvaluated"), TEXT("미판정")},
			{TEXT("Pass"), TEXT("합격")},
			{TEXT("QualityFail"), TEXT("품질 불합격")},
			{TEXT("Blocked"), TEXT("작업 차단")},
		};
		const FString* Label = Labels.Find(Value);
		return Label ? *Label : Value;
	}

	FString KoreanStatus(FString Value)
	{
		static const TPair<const TCHAR*, const TCHAR*> Translations[] = {
			{TEXT("Assign the vehicle assembly and native robot."), TEXT("차체와 로봇을 지정해 주세요.")},
			{TEXT("A serialized normal left-door reference is required."), TEXT("저장된 왼쪽 문 정상 기준 자세가 필요합니다.")},
			{TEXT("Control tolerances and times must be finite and positive."), TEXT("제어 허용 오차와 시간은 유효한 양수여야 합니다.")},
			{TEXT("Invalid grasp, waypoint, or control period (maximum 20 ms)."), TEXT("파지 자세, 경유점 또는 제어 주기가 잘못되었습니다. 제어 주기는 최대 20 ms입니다.")},
			{TEXT("Vehicle body requires a finite rigid transform with unit scale."), TEXT("차체 위치·회전이 유효해야 하며 크기 배율은 1이어야 합니다.")},
			{TEXT("Six registered inspection stations are required."), TEXT("검사 지점 6개를 등록해야 합니다.")},
			{TEXT("Quality tolerances and pickup recovery limits must be finite and positive."), TEXT("품질 허용 오차와 파지 복구 범위는 유효한 양수여야 합니다.")},
			{TEXT("Placement and sensor offsets must be finite."), TEXT("문 배치 및 센서 오차 설정값이 유효하지 않습니다.")},
			{TEXT("Could not place door in the pickup station."), TEXT("문을 파지 위치에 배치하지 못했습니다.")},
			{TEXT("Ready. Enter/Start runs the cycle; Space stops; R resets."), TEXT("준비 완료. Enter 또는 시작 버튼으로 실행하고, Space로 정지, R로 초기화합니다.")},
			{TEXT("Reset to Idle before starting a new cycle."), TEXT("새 작업을 시작하려면 초기화하여 대기 상태로 돌아가세요.")},
			{TEXT("Invalid alignment sensor input."), TEXT("정렬 센서 입력이 유효하지 않습니다.")},
			{TEXT("Cycle stopped; grasp/fastening state retained. Reset to recover."), TEXT("작업 정지. 파지·체결 상태를 유지합니다. 다시 실행하려면 초기화하세요.")},
			{TEXT("Tracking IK joint target with PID."), TEXT("IK로 계산한 관절 목표를 PID로 추종 중입니다.")},
			{TEXT("Lost robot or door reference."), TEXT("로봇 또는 문 연결이 끊겼습니다.")},
			{TEXT("Body or robot base moved during active cycle. Stop and recalibrate targets."), TEXT("작업 중 차체 또는 로봇 베이스가 이동했습니다. 정지 후 목표를 다시 보정하세요.")},
			{TEXT("Door moved externally while held. Stop and revalidate grasp."), TEXT("파지 중 외부 요인으로 문이 이동했습니다. 정지 후 파지 상태를 확인하세요.")},
			{TEXT("Door placement changed during pickup. Reset and re-detect."), TEXT("파지 접근 중 문 위치가 바뀌었습니다. 초기화 후 다시 감지하세요.")},
			{TEXT("Invalid servo input or integration failure."), TEXT("서보 입력 오류 또는 제어 계산 실패입니다.")},
			{TEXT("Door transport pose rejected."), TEXT("문 운반 자세를 적용할 수 없습니다.")},
			{TEXT("Invalid quality geometry or alignment sensor."), TEXT("품질 측정 형상 또는 정렬 센서가 유효하지 않습니다.")},
			{TEXT("Stage timed out before measured motion settled."), TEXT("동작이 안정되기 전에 단계 제한 시간을 초과했습니다.")},
			{TEXT("Lost alignment while fastening."), TEXT("체결 중 정렬이 틀어졌습니다.")},
			{TEXT("Alignment sensor lost alignment while fastening."), TEXT("체결 중 정렬 센서가 허용 범위 이탈을 감지했습니다.")},
			{TEXT("Ideal rigid fastening completed; retaining achieved pose."), TEXT("가상 강체 체결 완료. 도달한 문 자세를 유지합니다.")},
			{TEXT("Grasp contact does not match the detected door pose."), TEXT("파지 접촉 위치가 감지한 문 자세와 맞지 않습니다.")},
			{TEXT("Sensor-driven alignment did not converge within correction limit."), TEXT("센서 기반 정렬이 보정 횟수 제한 내에 수렴하지 못했습니다.")},
			{TEXT("Measured alignment accepted; ideal fastening in progress."), TEXT("측정된 정렬 상태를 확인했습니다. 가상 체결을 진행합니다.")},
			{TEXT("Complete: six-station quality PASS; door retained and robot retracted."), TEXT("완료: 6개 지점 품질 합격. 문을 고정하고 로봇이 후퇴했습니다.")},
			{TEXT("Complete: assembly QUALITY FAIL; review station gap/flush results."), TEXT("완료: 조립 품질 불합격. 지점별 간격·단차 결과를 확인하세요.")},
			{TEXT("Large frame stall: stopped; reset required."), TEXT("프레임 지연으로 정지했습니다. 초기화가 필요합니다.")},
			{TEXT("Pickup blocked: door is missing."), TEXT("파지 차단: 문이 없습니다.")},
			{TEXT("Pickup blocked: invalid or scaled door pose."), TEXT("파지 차단: 문 자세 또는 크기 배율이 잘못되었습니다.")},
			{TEXT("Pickup blocked: door outside configured recovery window; reposition it."), TEXT("파지 차단: 문이 설정된 복구 범위를 벗어났습니다. 다시 배치하세요.")},
			{TEXT("Pickup blocked: door reversed or outside assumed gripper orientation window."), TEXT("파지 차단: 문이 뒤집혔거나 파지 가능한 방향 범위를 벗어났습니다.")},
			{TEXT("Pickup blocked: IK cannot reach approach, grasp or lift pose."), TEXT("파지 차단: IK 계산상 접근·파지·들어 올리기 자세에 도달할 수 없습니다.")},
			{TEXT("Independent verification sensor is invalid; fastening blocked."), TEXT("독립 검증 센서가 유효하지 않아 체결을 차단했습니다.")},
			{TEXT("Independent six-station verification unavailable; fastening blocked."), TEXT("독립 검증의 6개 지점 측정이 불가능하여 체결을 차단했습니다.")},
			{TEXT("Independent verification rejected measured door alignment."), TEXT("독립 검증에서 문 정렬이 허용 범위를 벗어났습니다.")},
			{TEXT("Six calibrated joints, six initial angles, and unit actor scale required."), TEXT("보정된 관절 6개, 초기 각도 6개, 액터 크기 배율 1이 필요합니다.")},
			{TEXT("Invalid joint calibration or initial angle."), TEXT("관절 보정값 또는 초기 각도가 잘못되었습니다.")},
			{TEXT("Invalid TCP calibration."), TEXT("TCP 보정값이 잘못되었습니다.")},
			{TEXT("Invalid mesh-to-joint binding."), TEXT("메시와 관절 연결이 잘못되었습니다.")},
			{TEXT("Invalid TCP command or uninitialized robot."), TEXT("TCP 명령이 잘못되었거나 로봇이 초기화되지 않았습니다.")},
			{TEXT("Calibrated. PID velocity servo with simplified motor response."), TEXT("보정 완료. 단순화한 모터 응답과 PID 속도 서보를 사용합니다.")},
			{TEXT("IK solved; tracking joint trajectory with PID."), TEXT("IK 계산 완료. PID로 관절 궤적을 추종합니다.")},
			{TEXT("Stopped: simulated drive hold. Reset required."), TEXT("정지: 가상 구동 위치를 유지합니다. 초기화가 필요합니다.")},
			{TEXT("Independent verification blocked fastening at "), TEXT("독립 검증으로 체결 차단 — ")},
			{TEXT(": gap "), TEXT(": 간격 변화 ")},
			{TEXT(", flush "), TEXT(", 단차 변화 ")},
			{TEXT("IK failed: status "), TEXT("IK 실패: 상태 코드 ")},
			{TEXT(", position "), TEXT(", 위치 오차 ")},
			{TEXT(", orientation "), TEXT(", 방향 오차 ")},
			{TEXT(" deg."), TEXT("도.")},
		};
		for (const auto& Entry : Translations)
			Value.ReplaceInline(Entry.Key, Entry.Value, ESearchCase::CaseSensitive);
		for (const TCHAR* Station : {TEXT("FrontLower"), TEXT("FrontMiddle"), TEXT("FrontUpper"), TEXT("RearLower"), TEXT("RearMiddle"), TEXT("RearUpper")})
			Value.ReplaceInline(Station, *KoreanLabel(Station), ESearchCase::CaseSensitive);
		return Value;
	}

	FString KoreanQuality(const ADoorAssemblyCell* Cell)
	{
		if (!IsValid(Cell)) return TEXT("품질 측정 불가");
		FString Result = FString::Printf(
			TEXT("제어: %s / 상황: %s\n최종 판정: %s\n정렬 센서 %s: %.2f mm / %.2f도\n독립 센서 %s: %.2f mm\n6개 지점 품질: %s\n허용 변화량: 간격 ±%.2f / 단차 ±%.2f mm"),
			*KoreanLabel(StaticEnum<EDoorCellControlMode>()->GetNameStringByValue(int64(Cell->ControlMode))),
			*KoreanLabel(StaticEnum<EDoorCellScenario>()->GetNameStringByValue(int64(Cell->Scenario))),
			*KoreanLabel(StaticEnum<EDoorCellOutcome>()->GetNameStringByValue(int64(Cell->Outcome))),
			Cell->bAlignmentSensorValid ? TEXT("정상") : TEXT("오류"), Cell->AlignmentPositionErrorMm, Cell->AlignmentOrientationErrorDeg,
			Cell->bVerificationSensorValid ? TEXT("정상") : TEXT("오류"), Cell->VerificationPositionErrorMm,
			Cell->bQualityValid ? (Cell->bQualityPassed ? TEXT("합격") : TEXT("불합격")) : TEXT("측정 불가"), Cell->GapToleranceMm, Cell->FlushToleranceMm);
		for (int32 Index = 0; Index < Cell->StationQuality.Num(); ++Index)
		{
			const auto& Q = Cell->StationQuality[Index];
			Result += FString::Printf(TEXT("\n%s  간격:%+.2f  단차:%+.2f mm  %s"), *KoreanLabel(Q.Name.ToString()),
				Q.GapChangeMm, Q.FlushChangeMm, Q.bPassed ? TEXT("합격") : TEXT("불합격"));
		}
		return Result;
	}

	bool CanStopCell(const ADoorAssemblyCell* Cell)
	{
		return IsValid(Cell) && Cell->State != EDoorAssemblyState::Idle
			&& Cell->State != EDoorAssemblyState::Complete && Cell->State != EDoorAssemblyState::Fault
			&& Cell->State != EDoorAssemblyState::Stopped;
	}

	FText CellStatus(const TWeakObjectPtr<ADoorAssemblyPlayerController>& Controller)
	{
		const ADoorAssemblyCell* Cell = Controller.IsValid() ? Controller->GetAssemblyCell() : nullptr;
		return FText::FromString(IsValid(Cell) ? KoreanStatus(Cell->Status) : TEXT("맵에서 문 조립 셀을 찾을 수 없습니다."));
	}

	FString DoorMountMeasurements(const ADoorAssemblyCell* Cell)
	{
		if (!IsValid(Cell) || !IsValid(Cell->Assembly) || !IsValid(Cell->Assembly->GetLeftDoor()))
		{
			return TEXT("문 장착 오차: 측정 불가");
		}
		const FTransform Door = Cell->Assembly->GetLeftDoor()->GetComponentTransform();
		const FTransform Mount = Cell->GetMountDoorWorld();
		return FString::Printf(TEXT("문 장착 오차: %.2f mm / %.2f도"),
			FVector::Distance(Door.GetLocation(), Mount.GetLocation()) * 10.0,
			FMath::RadiansToDegrees(Door.GetRotation().AngularDistance(Mount.GetRotation())));
	}

	FText CellMeasurements(const TWeakObjectPtr<ADoorAssemblyPlayerController>& Controller)
	{
		const ADoorAssemblyCell* Cell = Controller.IsValid() ? Controller->GetAssemblyCell() : nullptr;
		if (!IsValid(Cell))
		{
			return FText::FromString(TEXT("조립 셀을 불러오는 중..."));
		}
		FString Measurements = FString::Printf(
			TEXT("진행 단계: %s\n경과 시간: %.2f초\nTCP 위치 오차: %.2f mm\nTCP 방향 오차: %.2f도\n문 파지: %s    체결: %s"),
			*KoreanLabel(Cell->GetStateLabel()), Cell->SimulationTimeSeconds, Cell->PositionErrorMm,
			Cell->OrientationErrorDeg, Cell->bDoorHeld ? TEXT("예") : TEXT("아니요"),
			Cell->bDoorFastened ? TEXT("예") : TEXT("아니요"));
		Measurements += TEXT("\n") + DoorMountMeasurements(Cell);
		return FText::FromString(Measurements);
	}

	TSharedRef<SWidget> ScenarioButton(const TWeakObjectPtr<ADoorAssemblyPlayerController>& Controller,
		const TCHAR* Label, EDoorCellScenario Scenario, EDoorCellControlMode Mode)
	{
		return SNew(SButton).IsFocusable(false).HAlign(HAlign_Center).Text(FText::FromString(Label))
			.IsEnabled_Lambda([Controller]()
			{
				const auto* C=Controller.IsValid() ? Controller->GetAssemblyCell() : nullptr;
				return IsValid(C) && !CanStopCell(C);
			})
			.OnClicked_Lambda([Controller,Scenario,Mode]()
			{
				if (Controller.IsValid() && Controller->GetAssemblyCell()) Controller->GetAssemblyCell()->SelectScenario(Scenario,Mode);
				return FReply::Handled();
			});
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
					.MaxDesiredHeight(780.0f)
					[
						SNew(SScrollBox)
						+ SScrollBox::Slot()
						[
						SNew(SBorder)
						.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
						.BorderBackgroundColor(FLinearColor(0.025f, 0.04f, 0.055f, 0.93f))
						.Padding(16.0f)
						[
							SNew(SVerticalBox)
							+ SVerticalBox::Slot().AutoHeight().Padding(0,0,0,6)
							[
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("정상"),EDoorCellScenario::Normal,EDoorCellControlMode::IndependentVerification)]
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("센서 오차 / 기본"),EDoorCellScenario::SensorBias,EDoorCellControlMode::Basic)]
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("센서 오차 / 검증"),EDoorCellScenario::SensorBias,EDoorCellControlMode::IndependentVerification)]
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0,0,0,8)
							[
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("배치 틀어짐"),EDoorCellScenario::DisplacedDoor,EDoorCellControlMode::IndependentVerification)]
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("범위 초과"),EDoorCellScenario::UnreachableDoor,EDoorCellControlMode::IndependentVerification)]
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("뒤집힘"),EDoorCellScenario::ReversedDoor,EDoorCellControlMode::IndependentVerification)]
								+ SHorizontalBox::Slot().FillWidth(1)[ScenarioButton(WeakController,TEXT("문 없음"),EDoorCellScenario::MissingDoor,EDoorCellControlMode::IndependentVerification)]
							]
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("로봇팔 문 조립 셀")))
								.Font(FCoreStyle::GetDefaultFontStyle("Bold", 30))
								.ColorAndOpacity(FLinearColor::White)
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 5.0f, 0.0f, 12.0f)
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("6축 로봇 · PID 구동 및 센서 기반 정렬")))
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
									.Text(FText::FromString(TEXT("시작")))
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
									.Text(FText::FromString(TEXT("정지")))
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
									.Text(FText::FromString(TEXT("초기화")))
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
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 18))
								.ColorAndOpacity(FLinearColor::White)
							]
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(STextBlock).AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular",16))
								.Text_Lambda([WeakController]()
								{ const auto* C=WeakController.IsValid() ? WeakController->GetAssemblyCell() : nullptr; return FText::FromString(KoreanQuality(C)); })
								.ColorAndOpacity_Lambda([WeakController]()
								{ const auto* C=WeakController.IsValid() ? WeakController->GetAssemblyCell() : nullptr; return FSlateColor(C && C->bQualityPassed ? FLinearColor(0.4f,1,0.6f) : FLinearColor(1,0.7f,0.3f)); })
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
								.Text(FText::FromString(TEXT("TCP 오차: 현재 이동 목표와의 차이\n문 장착 오차: 정상 장착 기준 자세와의 차이\nEnter: 시작 / Space: 정지 / R: 초기화\n우클릭 드래그: 시점 회전 / 휠: 확대·축소\nHome: 시점 초기화")))
								.AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 18))
								.ColorAndOpacity(FLinearColor(0.72f, 0.76f, 0.8f))
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 0.0f)
							[
								SNew(STextBlock)
								.Text(FText::FromString(TEXT("가상 자세 센서에 차체 좌표 기준의 일정한 오차를 적용합니다. 문 위치 감지는 이상적인 조건입니다. 간격·단차 변화량은 교육용 가상 측정값입니다. PID 모터 응답과 파지·체결을 단순화했으며, 충돌·하중·변형은 아직 반영하지 않습니다.")))
								.AutoWrapText(true)
								.Font(FCoreStyle::GetDefaultFontStyle("Regular", 16))
								.ColorAndOpacity(FLinearColor(0.63f, 0.68f, 0.73f))
							]
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

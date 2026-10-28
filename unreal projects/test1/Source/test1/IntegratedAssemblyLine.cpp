#include "IntegratedAssemblyLine.h"
#include "SensorAssemblyLine.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "Components/InputComponent.h"
#include "Camera/CameraActor.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
    FString StageLabel(EAssemblyLineStage Stage)
    {
        switch (Stage)
        {
        case EAssemblyLineStage::Idle: return TEXT("대기");
        case EAssemblyLineStage::Wheels: return TEXT("차체 반입 · 타이어 조립");
        case EAssemblyLineStage::Transfer: return TEXT("문 조립 구역 이송");
        case EAssemblyLineStage::Door: return TEXT("앞문 정렬 · 체결");
        case EAssemblyLineStage::Complete: return TEXT("완료 · 품질 합격");
        case EAssemblyLineStage::Stopped: return TEXT("운전자 정지");
        case EAssemblyLineStage::Fault: return TEXT("이상 감지 · 안전 정지");
        default: return TEXT("상태 확인 중");
        }
    }

    FString ProcessRail(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("○ 1 차체 반입  ─  ○ 2 타이어 조립  ─  ○ 3 구역 이송  ─  ○ 4 앞문 조립  ─  ○ 품질 판정");
        int32 Current = 0;
        if (Line->Stage == EAssemblyLineStage::Wheels)
            Current = Line->WheelCell && Line->WheelCell->bTransporting ? 0 : 1;
        else if (Line->Stage == EAssemblyLineStage::Transfer) Current = 2;
        else if (Line->Stage == EAssemblyLineStage::Door) Current = 3;
        else if (Line->Stage == EAssemblyLineStage::Complete) Current = 4;
        const TCHAR* Names[] = {TEXT("차체 반입"), TEXT("타이어 조립"), TEXT("구역 이송"), TEXT("앞문 조립"), TEXT("품질 판정")};
        FString Result;
        for (int32 Index=0; Index<5; ++Index)
        {
            if (Index) Result += TEXT("  ─  ");
            const bool Done = Line->Stage == EAssemblyLineStage::Complete || Index < Current;
            Result += FString::Printf(TEXT("%s %d %s"), Index == Current && Line->Stage != EAssemblyLineStage::Complete ? TEXT("●") : Done ? TEXT("✓") : TEXT("○"), Index+1, Names[Index]);
        }
        return Result;
    }

    FString CurrentOperation(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("조립 라인 액터를 찾는 중입니다.");
        switch (Line->Stage)
        {
        case EAssemblyLineStage::Idle: return TEXT("차체·로봇·센서의 초기 상태를 확인하고 시작 명령을 기다립니다.");
        case EAssemblyLineStage::Wheels:
            if (Line->WheelCell && Line->WheelCell->bTransporting) return TEXT("컨베이어가 차체를 작업 기준점으로 반입하고 도킹 여부를 확인합니다.");
            return FString::Printf(TEXT("좌·우 로봇이 타이어를 파지해 허브에 정렬·체결합니다. 현재 %d/4개 완료."), Line->WheelCell ? Line->WheelCell->MountedCount : 0);
        case EAssemblyLineStage::Transfer: return FString::Printf(TEXT("타이어 조립이 끝난 차체를 문 조립 구역으로 이송합니다. 진행률 %.0f%%."),Line->TransferProgress*100.0);
        case EAssemblyLineStage::Door:
            return FString::Printf(TEXT("문 로봇이 왼쪽 앞문을 파지·운반한 뒤 센서 오차를 보정하고 체결합니다. 현재: %s"),
                Line->DoorCell ? *Line->DoorCell->GetStateLabel() : TEXT("연결 확인 중"));
        case EAssemblyLineStage::Complete: return TEXT("타이어 4개와 왼쪽 앞문 체결을 마치고 최종 품질 판정을 통과했습니다.");
        case EAssemblyLineStage::Stopped: return TEXT("운전자 정지 명령으로 모든 구동 목표를 유지합니다. 원인 확인 후 초기화가 필요합니다.");
        case EAssemblyLineStage::Fault: return TEXT("이상을 감지하여 후속 공정을 차단하고 로봇과 이송 장치를 안전 정지했습니다.");
        default: return Line->Status;
        }
    }

    FString IncidentSummary(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("상태 데이터 없음");
        if (const auto* Autonomous = Cast<AAutonomousAssemblyLine>(Line))
        {
            if (!Autonomous->ActiveEvent.IsEmpty())
                return FString::Printf(TEXT("감지 이벤트: %s\n발생 %d/2 · 자동 복구 %d/2"), *Autonomous->ActiveEvent, Autonomous->InjectedEvents, Autonomous->RecoveredEvents);
        }
        if (Line->Stage == EAssemblyLineStage::Fault) return TEXT("발생 문제: ") + Line->Status;
        if (Line->Stage == EAssemblyLineStage::Stopped) return TEXT("발생 상황: 운전자 정지 명령");
        if (const auto* Sensor = Cast<ASensorAssemblyLine>(Line))
        {
            const bool Healthy = Sensor->Sensors && Sensor->Sensors->Healthy(Sensor->SensorTime);
            return Healthy ? TEXT("현재 감지된 이상 없음 · 도착/점유/위치/속도 센서 정상") : TEXT("센서 갱신 지연 또는 무효값 감지");
        }
        return TEXT("현재 감지된 이상 없음");
    }

    FString ResponseSummary(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("대응 판단 대기");
        if (Line->Stage == EAssemblyLineStage::Fault) return TEXT("현재 대응: 구동 정지 → 영향 공정 차단 → 원인 확인 → R로 안전 초기화");
        if (Line->Stage == EAssemblyLineStage::Stopped) return TEXT("현재 대응: 자세 유지 → 작업자 확인 → R로 초기 조건 복구");
        if (const auto* Autonomous = Cast<AAutonomousAssemblyLine>(Line))
        {
            if (Autonomous->InjectedEvents > Autonomous->RecoveredEvents)
                return TEXT("현재 대응: 해당 로봇 대기 → 공급/센서/그리퍼 복구 → 새 측정값 검증 → 자동 재시도");
        }
        if (Line->Stage == EAssemblyLineStage::Door) return TEXT("판단 기준: TCP 수렴 → 정렬 센서 허용 범위 → 독립 품질 검증 → 체결 승인");
        if (Line->Stage == EAssemblyLineStage::Wheels) return TEXT("판단 기준: 부품 존재·자세 확인 → 파지 확인 → 정렬 오차 확인 → 체결 승인");
        return TEXT("현재 대응: 공정 조건을 계속 감시하며 다음 단계 진입 조건을 확인합니다.");
    }

    FLinearColor StateColor(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line) || Line->Stage == EAssemblyLineStage::Fault) return FLinearColor(1.0f,.2f,.15f);
        if (Line->Stage == EAssemblyLineStage::Stopped) return FLinearColor(1.0f,.65f,.12f);
        if (Line->Stage == EAssemblyLineStage::Complete) return FLinearColor(.2f,1.0f,.45f);
        if (const auto* Autonomous = Cast<AAutonomousAssemblyLine>(Line); Autonomous && Autonomous->InjectedEvents > Autonomous->RecoveredEvents)
            return FLinearColor(1.0f,.65f,.12f);
        return FLinearColor(.18f,.75f,1.0f);
    }
}

AIntegratedAssemblyLine::AIntegratedAssemblyLine()
{
    PrimaryActorTick.bCanEverTick = true;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("LineRoot"));
}
void AIntegratedAssemblyLine::Fail(const FString& Reason)
{
    const FString Saved = Reason;
    if (WheelCell) WheelCell->StopCycle();
    if (DoorCell) DoorCell->StopCycle();
    Stage = EAssemblyLineStage::Fault;
    Status = Saved;
    bReady = false;
}
bool AIntegratedAssemblyLine::ResetLine()
{
    bReady = false;
    if (!IsValid(WheelCell) || !IsValid(DoorCell) || !IsValid(WheelCell->Assembly)
        || DoorCell->Assembly != WheelCell->Assembly || !IsValid(WheelCell->TransportCarriage)
        || !WheelCell->bEnableTransport || !FMath::IsFinite(TransferSeconds) || TransferSeconds <= 0
        || !FMath::IsFinite(DockSeconds) || DockSeconds < 0 || DoorStationOffsetCm.ContainsNaN())
    { Fail(TEXT("Invalid line references or transport configuration.")); return false; }
    WheelCell->bExternallyDriven = DoorCell->bExternallyDriven = true;
    WheelCell->StopCycle(); DoorCell->StopCycle();
    auto* Car = WheelCell->Assembly.Get();
    FTransform DoorStation = VehicleAtWheelStation;
    DoorStation.AddToTranslation(DoorStationOffsetCm);
    Car->SetActorTransform(DoorStation);
    if (!DoorCell->ResetCycle()) { Fail(DoorCell->Status); return false; }
    DoorSupplyWorld = Car->GetLeftDoor()->GetComponentTransform();
    Car->SetActorTransform(VehicleAtWheelStation);
    WheelCell->TransportCarriage->SetActorTransform(CarriageAtWheelStation);
    if (!WheelCell->ResetCycle()) { Fail(WheelCell->Status); return false; }
    Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
    ElapsedSeconds = TransferTime = TransferProgress = 0;
    Stage = EAssemblyLineStage::Idle;
    Status = TEXT("대기 · 차체 반입 → 타이어 4개 → 다음 구역 이송 → 왼쪽 앞문");
    bReady = true;
    return true;
}
bool AIntegratedAssemblyLine::StartLine()
{
    if (!bReady || Stage != EAssemblyLineStage::Idle) return false;
    if (!WheelCell->StartCycle()) { Fail(WheelCell->Status); return false; }
    Stage = EAssemblyLineStage::Wheels;
    return true;
}
void AIntegratedAssemblyLine::StopLine()
{
    if (Stage == EAssemblyLineStage::Idle || Stage == EAssemblyLineStage::Complete || Stage == EAssemblyLineStage::Fault) return;
    WheelCell->StopCycle(); DoorCell->StopCycle();
    Stage = EAssemblyLineStage::Stopped;
    Status = TEXT("정지 · 현재 자세 유지, R로 초기화 후 다시 시작");
    bReady = false;
}
bool AIntegratedAssemblyLine::AdvanceLine(double Seconds)
{
    if (!FMath::IsFinite(Seconds) || Seconds < 0 || Seconds > 1) { StopLine(); return false; }
    if (Stage == EAssemblyLineStage::Fault) return false;
    if (Stage == EAssemblyLineStage::Idle || Stage == EAssemblyLineStage::Stopped || Stage == EAssemblyLineStage::Complete) return true;
    if (!IsValid(WheelCell) || !IsValid(DoorCell) || !IsValid(WheelCell->Assembly) || !IsValid(WheelCell->TransportCarriage))
    { Fail(TEXT("Lost line reference.")); return false; }
    ElapsedSeconds += Seconds;
    auto* Car = WheelCell->Assembly.Get();
    auto* Carriage = WheelCell->TransportCarriage.Get();
    if (Stage == EAssemblyLineStage::Wheels)
    {
        const bool Ok = WheelCell->AdvanceSimulation(Seconds);
        Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
        Status = WheelCell->bTransporting ? TEXT("1/4 차체 반입") : TEXT("2/4 타이어 장착 · ") + WheelCell->Status;
        if (!Ok || WheelCell->Outcome == EWheelOutcome::Blocked || WheelCell->Outcome == EWheelOutcome::QualityFail || WheelCell->Outcome == EWheelOutcome::Stopped)
        { Fail(WheelCell->Status); return false; }
        if (WheelCell->Outcome == EWheelOutcome::Pass)
        {
            ExpectedVehicle = Car->GetActorTransform();
            ExpectedCarriage = Carriage->GetActorTransform();
            Stage = EAssemblyLineStage::Transfer;
        }
    }
    else if (Stage == EAssemblyLineStage::Transfer)
    {
        if (!Car->GetActorTransform().Equals(ExpectedVehicle,1.e-3) || !Carriage->GetActorTransform().Equals(ExpectedCarriage,1.e-3))
        { Fail(TEXT("Vehicle or carriage moved externally between stations.")); return false; }
        TransferTime += Seconds;
        const double T = FMath::Clamp(TransferTime / TransferSeconds,0.,1.);
        TransferProgress = T*T*T*(10 + T*(-15 + 6*T));
        ExpectedVehicle = VehicleAtWheelStation;
        ExpectedVehicle.AddToTranslation(DoorStationOffsetCm * TransferProgress);
        ExpectedCarriage = CarriageAtWheelStation;
        ExpectedCarriage.AddToTranslation(DoorStationOffsetCm * TransferProgress);
        Car->SetActorTransform(ExpectedVehicle);
        Carriage->SetActorTransform(ExpectedCarriage);
        WheelCell->UpdateFastenedWheels();
        Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
        Status = T < 1 ? TEXT("3/4 타이어 장착 차체를 문 작업 구역으로 이송") : TEXT("문 작업 구역 도킹 확인");
        if (TransferTime + 1.e-9 >= TransferSeconds + DockSeconds)
        {
            if (!DoorCell->StartCycle()) { Fail(DoorCell->Status); return false; }
            Stage = EAssemblyLineStage::Door;
        }
    }
    else if (Stage == EAssemblyLineStage::Door)
    {
        if (!Carriage->GetActorTransform().Equals(ExpectedCarriage,1.e-3))
        { Fail(TEXT("Carriage moved during door assembly.")); return false; }
        if (!DoorCell->AdvanceSimulation(Seconds) || DoorCell->Outcome == EDoorCellOutcome::QualityFail || DoorCell->Outcome == EDoorCellOutcome::Stopped)
        { Fail(DoorCell->Status); return false; }
        Status = TEXT("4/4 왼쪽 앞문 장착 · ") + DoorCell->GetStateLabel();
        if (DoorCell->State == EDoorAssemblyState::Complete)
        {
            if (DoorCell->Outcome != EDoorCellOutcome::Pass) { Fail(DoorCell->Status); return false; }
            Stage = EAssemblyLineStage::Complete;
            Status = TEXT("완료 · 타이어 4개와 왼쪽 앞문 장착·검사 합격");
        }
    }
    return Stage != EAssemblyLineStage::Fault;
}
FString AIntegratedAssemblyLine::GetSummary() const
{
    return FString::Printf(TEXT("통합 자동차 조립\n%s\n경과 %.1f초 | 타이어 %d/4 | 앞문 %s\n\nEnter 시작 / Space 정지 / R 초기화\n우클릭 회전 / 휠 확대·축소 / Home 전체 보기"),
        *Status, ElapsedSeconds, WheelCell ? WheelCell->MountedCount : 0,
        DoorCell && DoorCell->bDoorFastened ? TEXT("체결") : TEXT("대기"));
}
void AIntegratedAssemblyLine::BeginPlay() { Super::BeginPlay(); ResetLine(); }
void AIntegratedAssemblyLine::Tick(float Seconds) { Super::Tick(Seconds); AdvanceLine(Seconds); }

AIntegratedAssemblyController::AIntegratedAssemblyController() { bAutoManageActiveCameraTarget=false; bShowMouseCursor=true; }
void AIntegratedAssemblyController::BeginPlay()
{
    Super::BeginPlay();
    if (!IsLocalController()) return;
    for (TActorIterator<AIntegratedAssemblyLine> It(GetWorld()); It; ++It) { Line=*It; break; }
    AssemblyCamera=GetWorld()->SpawnActor<ACameraActor>();
    ResetCamera(); SetViewTarget(AssemblyCamera);
    Panel=SNew(SBox).HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(18).WidthOverride(650).MaxDesiredHeight(980)
    [SNew(SScrollBox)
        +SScrollBox::Slot()
        [SNew(SBorder).Padding(16).BorderBackgroundColor(FLinearColor(.012f,.022f,.035f,.96f))
        [SNew(SVerticalBox)
            +SVerticalBox::Slot().AutoHeight()
            [SNew(STextBlock).Text(FText::FromString(TEXT("DIGITAL TWIN · ASSEMBLY CONTROL")))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold",14)).ColorAndOpacity(FLinearColor(.35f,.72f,1.0f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,2,0,10)
            [SNew(STextBlock).Text(FText::FromString(TEXT("자동차 조립 공정 모니터")))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold",28)).ColorAndOpacity(FLinearColor::White)]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,10)
            [SNew(SBorder).Padding(10).BorderBackgroundColor_Lambda([this](){ return StateColor(Line)*FLinearColor(1,1,1,.22f); })
                [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(FString::Printf(TEXT("●  %s"),*StageLabel(Line ? Line->Stage : EAssemblyLineStage::Fault))); })
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold",22)).ColorAndOpacity_Lambda([this](){ return StateColor(Line); })]]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,12)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(ProcessRail(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Bold",15)).ColorAndOpacity(FLinearColor(.72f,.82f,.9f))]

            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,4)
            [SNew(STextBlock).Text(FText::FromString(TEXT("지금 하는 작업"))).Font(FCoreStyle::GetDefaultFontStyle("Bold",17)).ColorAndOpacity(FLinearColor(.4f,.82f,1.0f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,12)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(CurrentOperation(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",18)).ColorAndOpacity(FLinearColor::White)]

            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,4)
            [SNew(STextBlock).Text(FText::FromString(TEXT("이상 감지"))).Font(FCoreStyle::GetDefaultFontStyle("Bold",17)).ColorAndOpacity(FLinearColor(1.0f,.68f,.25f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,10)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(IncidentSummary(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",17)).ColorAndOpacity_Lambda([this](){ return StateColor(Line); })]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,4)
            [SNew(STextBlock).Text(FText::FromString(TEXT("시스템 판단 · 대응"))).Font(FCoreStyle::GetDefaultFontStyle("Bold",17)).ColorAndOpacity(FLinearColor(.45f,1.0f,.65f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,12)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(ResponseSummary(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",17)).ColorAndOpacity(FLinearColor(.83f,.92f,.86f))]

            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,8)
            [SNew(SBorder).Padding(10).BorderBackgroundColor(FLinearColor(.04f,.065f,.085f,.95f))
                [SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(Line ? Line->GetSummary() : TEXT("조립 라인 연결 대기")); })
                    .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",16)).ColorAndOpacity(FLinearColor(.82f,.88f,.92f))]]
            +SVerticalBox::Slot().AutoHeight().Padding(0,8,0,0)[SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(GetCameraLabel()); }).AutoWrapText(true).ColorAndOpacity(FLinearColor(.68f,.72f,.76f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,4,0,0)[SNew(SHorizontalBox)
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("전체"))).OnClicked_Lambda([this](){ResetCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("도킹 센서"))).OnClicked_Lambda([this](){ArrivalCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("왼팔"))).OnClicked_Lambda([this](){LeftRobotCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("오른팔"))).OnClicked_Lambda([this](){RightRobotCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("문 팔"))).OnClicked_Lambda([this](){DoorRobotCamera();return FReply::Handled();})]
            ]
            +SVerticalBox::Slot().AutoHeight().Padding(0,10,0,0)[SNew(SHorizontalBox).Visibility(Line && Line->GetClass()==ASensorAssemblyLine::StaticClass()?EVisibility::Visible:EVisibility::Collapsed)
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("정상"))).OnClicked_Lambda([this](){DemoNormal();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("이송 멈춤"))).OnClicked_Lambda([this](){DemoJam();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("위치 틀어짐"))).OnClicked_Lambda([this](){DemoMisDock();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("센서 끊김"))).OnClicked_Lambda([this](){DemoDropout();return FReply::Handled();})]
            ]
			+SVerticalBox::Slot().AutoHeight().Padding(0,4,0,0)[SNew(SHorizontalBox).Visibility(Line && Line->GetClass()==ASensorAssemblyLine::StaticClass()?EVisibility::Visible:EVisibility::Collapsed)
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("타이어 없음"))).OnClicked_Lambda([this](){DemoWheelMissing();return FReply::Handled();})]
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("타이어 위치"))).OnClicked_Lambda([this](){DemoWheelDisplaced();return FReply::Handled();})]
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("파지 실패"))).OnClicked_Lambda([this](){DemoGripFailure();return FReply::Handled();})]
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("타이어 센서 끊김"))).OnClicked_Lambda([this](){DemoWheelPoseDropout();return FReply::Handled();})]
			]
            +SVerticalBox::Slot().AutoHeight().Padding(0,12,0,0)[SNew(SHorizontalBox)
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("시작"))).OnClicked_Lambda([this](){StartAssembly();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("정지"))).OnClicked_Lambda([this](){StopAssembly();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("초기화"))).OnClicked_Lambda([this](){ResetAssembly();return FReply::Handled();})]
            ]
        ]]
    ];
    if (GEngine && GEngine->GameViewport) GEngine->GameViewport->AddViewportWidgetContent(Panel.ToSharedRef(),10);
    FInputModeGameAndUI Mode; Mode.SetHideCursorDuringCapture(false); SetInputMode(Mode);
}
void AIntegratedAssemblyController::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Panel.IsValid() && GEngine && GEngine->GameViewport) GEngine->GameViewport->RemoveViewportWidgetContent(Panel.ToSharedRef());
    Panel.Reset(); if (AssemblyCamera) AssemblyCamera->Destroy(); Super::EndPlay(Reason);
}
void AIntegratedAssemblyController::SetupInputComponent()
{
    Super::SetupInputComponent();
    InputComponent->BindKey(EKeys::Enter,IE_Pressed,this,&AIntegratedAssemblyController::StartAssembly);
    InputComponent->BindKey(EKeys::One,IE_Pressed,this,&AIntegratedAssemblyController::DemoNormal);
    InputComponent->BindKey(EKeys::Two,IE_Pressed,this,&AIntegratedAssemblyController::DemoJam);
    InputComponent->BindKey(EKeys::Three,IE_Pressed,this,&AIntegratedAssemblyController::DemoMisDock);
    InputComponent->BindKey(EKeys::Four,IE_Pressed,this,&AIntegratedAssemblyController::DemoDropout);
	InputComponent->BindKey(EKeys::Five,IE_Pressed,this,&AIntegratedAssemblyController::DemoWheelMissing);
	InputComponent->BindKey(EKeys::Six,IE_Pressed,this,&AIntegratedAssemblyController::DemoWheelDisplaced);
	InputComponent->BindKey(EKeys::Seven,IE_Pressed,this,&AIntegratedAssemblyController::DemoGripFailure);
	InputComponent->BindKey(EKeys::Eight,IE_Pressed,this,&AIntegratedAssemblyController::DemoWheelPoseDropout);
    InputComponent->BindKey(EKeys::SpaceBar,IE_Pressed,this,&AIntegratedAssemblyController::StopAssembly);
    InputComponent->BindKey(EKeys::R,IE_Pressed,this,&AIntegratedAssemblyController::ResetAssembly);
    InputComponent->BindKey(EKeys::Home,IE_Pressed,this,&AIntegratedAssemblyController::ResetCamera);
    InputComponent->BindKey(EKeys::F1,IE_Pressed,this,&AIntegratedAssemblyController::ResetCamera);
    InputComponent->BindKey(EKeys::F2,IE_Pressed,this,&AIntegratedAssemblyController::ArrivalCamera);
    InputComponent->BindKey(EKeys::F3,IE_Pressed,this,&AIntegratedAssemblyController::LeftRobotCamera);
    InputComponent->BindKey(EKeys::F4,IE_Pressed,this,&AIntegratedAssemblyController::RightRobotCamera);
    InputComponent->BindKey(EKeys::F5,IE_Pressed,this,&AIntegratedAssemblyController::DoorRobotCamera);
    InputComponent->BindKey(EKeys::MouseScrollUp,IE_Pressed,this,&AIntegratedAssemblyController::ZoomIn);
    InputComponent->BindKey(EKeys::MouseScrollDown,IE_Pressed,this,&AIntegratedAssemblyController::ZoomOut);
}
void AIntegratedAssemblyController::PlayerTick(float Seconds)
{
    Super::PlayerTick(Seconds);
    if (!IsLocalController() || !AssemblyCamera) return;
    const double Forward=(IsInputKeyDown(EKeys::W)?1.:0.)-(IsInputKeyDown(EKeys::S)?1.:0.);
    const double Right=(IsInputKeyDown(EKeys::D)?1.:0.)-(IsInputKeyDown(EKeys::A)?1.:0.);
    const double Up=(IsInputKeyDown(EKeys::E)?1.:0.)-(IsInputKeyDown(EKeys::Q)?1.:0.);
    if (Forward!=0 || Right!=0 || Up!=0)
    {
        CameraMode=ECameraMode::Free;
        const FRotator View=(Focus-AssemblyCamera->GetActorLocation()).Rotation();
        FVector FlatForward=View.Vector(); FlatForward.Z=0; FlatForward.Normalize();
        FVector FlatRight=View.RotateVector(FVector::RightVector); FlatRight.Z=0; FlatRight.Normalize();
        const double Speed=(IsInputKeyDown(EKeys::LeftShift)||IsInputKeyDown(EKeys::RightShift)?900.:350.)*Seconds;
        Focus+=(FlatForward*Forward+FlatRight*Right+FVector::UpVector*Up).GetClampedToMaxSize(1.)*Speed;
        UpdateCamera();
    }
    float X=0,Y=0;
    if (IsInputKeyDown(EKeys::RightMouseButton) && GetMousePosition(X,Y))
    {
        const FVector2D P(X,Y);
        if (bDragging) { CameraMode=ECameraMode::Free; const FVector2D D=P-LastMouse; Orbit.Yaw-=D.X*.25; Orbit.Pitch=FMath::Clamp(Orbit.Pitch+D.Y*.25,4.,80.); UpdateCamera(); }
        LastMouse=P; bDragging=true;
    }
    else bDragging=false;
    if (CameraMode!=ECameraMode::Free) UpdateCamera();
}
void AIntegratedAssemblyController::StartAssembly() { if(Line) Line->StartLine(); }
void AIntegratedAssemblyController::DemoNormal() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::Normal); }
void AIntegratedAssemblyController::DemoJam() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::Jam); }
void AIntegratedAssemblyController::DemoMisDock() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::MisDock); }
void AIntegratedAssemblyController::DemoDropout() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::PositionDropout); }
void AIntegratedAssemblyController::DemoWheelMissing() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::WheelMissing); }
void AIntegratedAssemblyController::DemoWheelDisplaced() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::WheelDisplaced); }
void AIntegratedAssemblyController::DemoGripFailure() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::GripFailure); }
void AIntegratedAssemblyController::DemoWheelPoseDropout() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::WheelPoseDropout); }
void AIntegratedAssemblyController::StopAssembly() { if(Line) Line->StopLine(); }
void AIntegratedAssemblyController::ResetAssembly() { if(Line) Line->ResetLine(); }
void AIntegratedAssemblyController::ResetCamera() { CameraMode=ECameraMode::Free; Focus=FVector(200,0,70); Orbit=FVector(550,-1600,1000).Rotation(); Distance=2000; UpdateCamera(); }
void AIntegratedAssemblyController::ArrivalCamera()
{
    if (!Line) return;
    CameraMode=ECameraMode::Arrival;
    Focus=Line->VehicleAtWheelStation.GetLocation()+FVector(-170,0,80);
    Orbit=FVector(-330,-480,250).Rotation(); Distance=640; UpdateCamera();
}
void AIntegratedAssemblyController::LeftRobotCamera() { CameraMode=ECameraMode::LeftRobot; UpdateCamera(); }
void AIntegratedAssemblyController::RightRobotCamera() { CameraMode=ECameraMode::RightRobot; UpdateCamera(); }
void AIntegratedAssemblyController::DoorRobotCamera() { CameraMode=ECameraMode::DoorRobot; UpdateCamera(); }
FString AIntegratedAssemblyController::GetCameraLabel() const
{
    const TCHAR* Names[]={TEXT("전체·자유 시점"),TEXT("도킹 센서 관찰"),TEXT("왼쪽 타이어 로봇 추적"),TEXT("오른쪽 타이어 로봇 추적"),TEXT("문 로봇 추적")};
    return FString::Printf(TEXT("시점: %s | F1~F5 전환 · WASD 이동 · Q/E 높이 · Shift 빠르게"),Names[int32(CameraMode)]);
}
void AIntegratedAssemblyController::FollowRobot(ARobotArm* Robot,double Side)
{
    if (!IsValid(Robot) || !AssemblyCamera) return;
    const FVector TCP=Robot->GetTCPWorld().GetLocation();
    Focus=TCP*.7+Robot->GetActorLocation()*.3+FVector(0,0,20);
    const FVector P=Focus+FVector(-220,Side*300,180);
    const FVector Offset=P-Focus;
    Orbit=Offset.Rotation(); Distance=Offset.Size();
    AssemblyCamera->SetActorLocationAndRotation(P,(Focus-P).Rotation());
}
void AIntegratedAssemblyController::UpdateCamera()
{
    if (!AssemblyCamera) return;
    if (Line && CameraMode==ECameraMode::LeftRobot && Line->WheelCell) { FollowRobot(Line->WheelCell->LeftRobot,-1); return; }
    if (Line && CameraMode==ECameraMode::RightRobot && Line->WheelCell) { FollowRobot(Line->WheelCell->RightRobot,1); return; }
    if (Line && CameraMode==ECameraMode::DoorRobot && Line->DoorCell) { FollowRobot(Line->DoorCell->Robot,-1); return; }
    const FVector P=Focus+Orbit.Vector()*Distance;
    AssemblyCamera->SetActorLocationAndRotation(P,(Focus-P).Rotation());
}
void AIntegratedAssemblyController::ZoomIn() { CameraMode=ECameraMode::Free; Distance=FMath::Max(150.,Distance*.9); UpdateCamera(); }
void AIntegratedAssemblyController::ZoomOut() { CameraMode=ECameraMode::Free; Distance=FMath::Min(5000.,Distance/.9); UpdateCamera(); }
AIntegratedAssemblyGameMode::AIntegratedAssemblyGameMode() { PlayerControllerClass=AIntegratedAssemblyController::StaticClass(); DefaultPawnClass=nullptr; HUDClass=nullptr; }

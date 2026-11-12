#include "FactoryRuntime.h"
#include "FactoryEquipmentRegistry.h"
#include "FactoryViewport.h"
#include "FactoryProcessRunner.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "Framework/Application/SlateApplication.h"
#include "Framework/Application/IInputProcessor.h"
#include "Engine/World.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Fonts/CompositeFont.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
// Scoped to this live factory UI, before the editor's Ctrl+Z command routing.
// No UObject layout changes: safe to install through Live Coding and restart Play.
class FFactoryShortcutInput final : public IInputProcessor, public TSharedFromThis<FFactoryShortcutInput>
{
public:
    FFactoryShortcutInput(AFactoryRuntime* R,TSharedRef<SWidget> P,TSharedRef<SFactoryViewport> V,bool Restore)
        : Runtime(R),Root(P),Surface(V),bRestoreFocus(Restore) {}
    static bool Contains(const TSharedPtr<SWidget>& Parent,const TSharedPtr<SWidget>& Child)
    {
        if(!Parent||!Child)return false;if(Parent==Child)return true;
        if(auto* Children=Parent->GetChildren())for(int32 I=0;I<Children->Num();++I)
            if(Contains(Children->GetChildAt(I),Child))return true;
        return false;
    }
    virtual void Tick(float,FSlateApplication& App,TSharedRef<ICursor>) override
    {
        if(!Runtime.IsValid()||!Root.IsValid()||!Runtime->GetWorld()||!Runtime->GetWorld()->IsGameWorld())
        {App.UnregisterInputPreProcessor(AsShared());return;}
        // Restore after Slate has attached the rebuilt tree, not just during removal/addition.
        if(bRestoreFocus)
        {
            bRestoreFocus=false;
            if(auto V=Surface.Pin())App.SetKeyboardFocus(V,EFocusCause::SetDirectly);
        }
    }
    virtual bool HandleKeyDownEvent(FSlateApplication& App,const FKeyEvent& E) override
    {
        if(!Runtime.IsValid()||!Root.IsValid()||!E.IsControlDown()||(E.GetKey()!=EKeys::Z&&E.GetKey()!=EKeys::Y))return false;
        const auto Focus=App.GetKeyboardFocusedWidget();
        if(!Contains(Root.Pin(),Focus))return false;
        // Text/numeric fields retain their own Undo and IME handling.
        if(Focus.IsValid()&&Focus->GetTypeAsString().Contains(TEXT("EditableText")))return false;
        if(!E.IsRepeat())Runtime->ViewportKey(E.GetKey(),true,E.IsShiftDown());
        return true;
    }
private:
    TWeakObjectPtr<AFactoryRuntime> Runtime;
    TWeakPtr<SWidget> Root;
    TWeakPtr<SFactoryViewport> Surface;
    bool bRestoreFocus;
};

struct FFactoryPanelChoice
{
    FString Id;
    FString Label;
    FFactoryPanelChoice(const FString& InId, const FString& InLabel) : Id(InId), Label(InLabel) {}
};

struct FFactoryNameDraft
{
    FString ItemId;
    FString Korean;
    FString English;
};

FString FactoryRoleLabel(const FString& Role)
{
    if (Role == TEXT("work")) return TEXT("작업 구역 (Work)");
    if (Role == TEXT("supply")) return TEXT("부품 공급 (Supply)");
    if (Role == TEXT("buffer")) return TEXT("대기 구역 (Buffer)");
    if (Role == TEXT("walkway")) return TEXT("보행 통로 (Walkway)");
    if (Role == TEXT("rework")) return TEXT("재작업 구역 (Rework)");
    if (Role == TEXT("reserve")) return TEXT("확장 공간 (Reserve)");
    return TEXT("용도 선택");
}
}

void AFactoryRuntime::CreatePanel()
{
    if (!GEngine || !GEngine->GameViewport) return;
    const uint64 Generation = ++PanelGeneration;
    const float SavedScrollOffset = PanelScroll.IsValid() ? PanelScroll->GetScrollOffset() : 0.0f;
    const bool RestoreViewportFocus=ViewportSurface.IsValid()&&(ViewportSurface->HasKeyboardFocus()||ViewportSurface->HasAnyUserFocus());
    // Keep the root and input surface attached while refreshing inspector controls.
    // Detaching the root invalidates Slate's focus/capture path during a drag.

    const FString FontFile = FPaths::ProjectContentDir() / TEXT("DigitalTwin/Fonts/NanumGothic.ttf");
    const bool bHasKoreanFont = IFileManager::Get().FileExists(*FontFile);
    TSharedPtr<const FCompositeFont> KoreanFont;
    if (bHasKoreanFont) KoreanFont = MakeShared<FCompositeFont>(FName(TEXT("NanumGothic")), FontFile,
        EFontHinting::Default, EFontLoadingPolicy::LazyLoad);
    const FSlateFontInfo Font = bHasKoreanFont ? FSlateFontInfo(KoreanFont, 14)
        : FCoreStyle::GetDefaultFontStyle("Regular", 14);
    const FSlateFontInfo HeadingFont = bHasKoreanFont ? FSlateFontInfo(KoreanFont, 17)
        : FCoreStyle::GetDefaultFontStyle("Bold", 17);
    const FLinearColor TextColor(.92f, .95f, .98f, 1);
    const FLinearColor Accent(.37f, .8f, .95f, 1);
    const FString PanelItemId = SelectedId;
    const TAttribute<bool> Editing = TAttribute<bool>::CreateLambda([this]() { return CanEdit(); });
    const TAttribute<bool> HasSelection = TAttribute<bool>::CreateLambda([this, PanelItemId]() {
        return CanEdit() && Selected() && SelectedId == PanelItemId;
    });
    const TAttribute<bool> ZoneSelected = TAttribute<bool>::CreateLambda([this, PanelItemId]() {
        const FFactoryLayoutItem* Item = Selected();
        return CanEdit() && Item && SelectedId == PanelItemId && Item->Kind == EFactoryItemKind::Zone;
    });
    TSharedRef<SVerticalBox> Rows = SNew(SVerticalBox);

    auto Label = [&Rows, Font, TextColor](const FString& Value)
    {
        Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(STextBlock).Font(Font)
            .ColorAndOpacity(TextColor).Text(FText::FromString(Value)).AutoWrapText(true)];
    };
    auto Heading = [&Rows, HeadingFont, Accent](const FString& Value)
    {
        Rows->AddSlot().AutoHeight().Padding(0, 12, 0, 5)[SNew(STextBlock).Font(HeadingFont)
            .ColorAndOpacity(Accent).Text(FText::FromString(Value)).AutoWrapText(true)];
    };
    auto Button = [this,Font](const FString& Caption, TFunction<void()> Action,
        const TAttribute<bool>& Enabled) -> TSharedRef<SWidget>
    {
        return SNew(SButton).IsFocusable(false).IsEnabled(Enabled).ContentPadding(FMargin(5, 6))
            .HAlign(HAlign_Center).OnClicked_Lambda([this,Action = MoveTemp(Action)]() {
                Action(); if(ViewportSurface.IsValid())FSlateApplication::Get().SetKeyboardFocus(ViewportSurface.ToSharedRef(),EFocusCause::SetDirectly); return FReply::Handled();
            })[SNew(STextBlock).Font(Font).Text(FText::FromString(Caption)).AutoWrapText(true)
                .Justification(ETextJustify::Center)];
    };
    auto Pair = [&Rows](TSharedRef<SWidget> Left, TSharedRef<SWidget> Right)
    {
        Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0)[Left]
            + SHorizontalBox::Slot().FillWidth(1).Padding(3, 0, 0, 0)[Right]];
    };
    auto Number = [this, Generation, &Rows, Font, TextColor](const FString& Caption, TFunction<TOptional<double>()> Get,
        TFunction<void(double)> Set, const TAttribute<bool>& Enabled, double Min, double Max)
    {
        Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(.48f).VAlign(VAlign_Center)[SNew(STextBlock).Font(Font)
                .ColorAndOpacity(TextColor).Text(FText::FromString(Caption)).AutoWrapText(true)]
            + SHorizontalBox::Slot().FillWidth(.52f)[SNew(SNumericEntryBox<double>).Font(Font)
                .IsEnabled(Enabled).AllowSpin(false).MinValue(Min).MaxValue(Max)
                .MinFractionalDigits(0).MaxFractionalDigits(3)
                .Value_Lambda([Get]() { return Get(); })
                .OnValueCommitted_Lambda([this, Generation, Get, Set](double Value, ETextCommit::Type Commit) {
                    if (PanelGeneration != Generation || Commit == ETextCommit::OnCleared) return;
                    const TOptional<double> Previous = Get();
                    if (Previous.IsSet() && !FMath::IsNearlyEqual(Previous.GetValue(), Value, 1.e-6)) Set(Value);
                })]];
    };

    Heading(bUseAssemblyIntegration?TEXT("공장·조립 통합 테스트"):TEXT("공장 배치·휠 작업"));
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(STextBlock).Font(Font).ColorAndOpacity(TextColor)
        .Text_Lambda([this]() { return FText::FromString(StateText()
            + (bDirty ? TEXT(" · 저장되지 않은 변경") : TEXT(""))); }).AutoWrapText(true)];

    Rows->AddSlot().AutoHeight().Padding(0, 5)[SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 3, 0)[
            Button(TEXT("작업 실행"), [this]() { StartCycle(); }, Editing)]
        + SHorizontalBox::Slot().FillWidth(1).Padding(3, 0)[
            Button(TEXT("정지"), [this]() { StopCycle(); }, TAttribute<bool>::CreateLambda([this]() {
                return RunState != EFactoryRunState::Editing && RunState != EFactoryRunState::Complete
                    && RunState != EFactoryRunState::Stopped && RunState != EFactoryRunState::Fault;
            }))]
        + SHorizontalBox::Slot().FillWidth(1).Padding(3, 0, 0, 0)[
            Button(TEXT("편집 복귀"), [this]() { ResetCycle(); }, TAttribute<bool>::CreateLambda([this]() {
                return !CanEdit();
            }))]];
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(STextBlock).Font(Font).ColorAndOpacity(TextColor)
        .Text_Lambda([this]() {
            if(bUseAssemblyIntegration)return FText::FromString(AssemblyMetrics());
            const FString Alignment = RunState == EFactoryRunState::Complete
                ? FString::Printf(TEXT("%.2fmm"), AlignmentMm) : TEXT("완료 후 표시");
            const FString PreviousCycle = LastCycleSeconds > 0
                ? FString::Printf(TEXT("%.2f초"), LastCycleSeconds) : TEXT("없음");
            return FText::FromString(FString::Printf(
                TEXT("경과 %.2f초 · 휠 이동 %.2fm\n정렬 오차 %s\n직전 완료 %s"),
                SimulationSeconds, TravelCm / 100.0, *Alignment, *PreviousCycle));
        }).AutoWrapText(true)];
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(STextBlock).Font(Font)
        .ColorAndOpacity(FLinearColor(.99f, .83f, .48f, 1)).Text_Lambda([this]() {
            return FText::FromString(Status);
        }).AutoWrapText(true)];

    Label(bUseAssemblyIntegration?TEXT("개별 로봇 편집 · 추가 후 작업 연결 · 차량/레일은 공통\nIK·PID / 물리 파지 검사는 왼쪽 앞 휠에 적용"):TEXT("IK·관절 PID / 체결·접촉 물리 미포함"));

    if(bUseAssemblyIntegration)
    {
        Heading(TEXT("이식된 센서 기능 확인"));
        Label(TEXT("물리 파지 시험: 정상 운반은 초록 휠 표식, 저마찰·과중량은 빨간 표식과 파지 해제 후 휠 낙하를 확인하세요. 질량·힘·미끄러짐 수치는 상단 공정 상태에 표시됩니다. 점질량 운반 시연이며 Chaos 충돌은 아닙니다."));
        Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(SCheckBox)
            .IsEnabled_Lambda([this](){return CanEdit()&&IsValid(TestAssemblyLine)&&IsValid(TestAssemblyLine->WheelCell);})
            .IsChecked_Lambda([this](){return IsValid(TestAssemblyLine)&&IsValid(TestAssemblyLine->WheelCell)&&TestAssemblyLine->WheelCell->bLaserScanFrontLeftHub?ECheckBoxState::Checked:ECheckBoxState::Unchecked;})
            .OnCheckStateChanged_Lambda([this](ECheckBoxState V){if(IsValid(TestAssemblyLine)&&IsValid(TestAssemblyLine->WheelCell))TestAssemblyLine->WheelCell->bLaserScanFrontLeftHub=V==ECheckBoxState::Checked;})
            [SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("왼쪽 앞 휠 레이저 3단계 검사 켜기")))]];
        Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(STextBlock).Font(Font).ColorAndOpacity(Accent)
            .Text_Lambda([this](){return FText::FromString(IsValid(TestAssemblyLine)&&IsValid(TestAssemblyLine->WheelCell)
                ?TestAssemblyLine->WheelCell->GetLaserInspectionSummary():TEXT("휠 검사 연결 없음"));}).AutoWrapText(true)];
        Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(SCheckBox)
            .IsChecked_Lambda([this](){return IsValid(TestAssemblyLine)&&TestAssemblyLine->bShowSensorVisualization?ECheckBoxState::Checked:ECheckBoxState::Unchecked;})
            .OnCheckStateChanged_Lambda([this](ECheckBoxState V){if(IsValid(TestAssemblyLine))TestAssemblyLine->bShowSensorVisualization=V==ECheckBoxState::Checked;})
            [SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("센서·휠 중심·장착점 표시"))) ]];
        Label(TEXT("흰색: 실제 휠 중심 / 노랑: 측정 위치\n빨강: 오래된 측정 / 초록: 장착 목표\n파랑 선: 위치 연결 표시"));
        Label(TEXT("검사 선택 → 센서 준비 후 상단 작업 실행\n왼쪽 앞 휠에서 선택한 사건을 재현합니다."));
        Label(TEXT("휠·차문 규격 검사는 부품 크기를 ±12% 바꾸고 공급 위치에서 레이저 측정한 폭/지름을 기준과 비교합니다. 빨간 측정선과 수치를 확인하세요. 차문 규격 검사는 휠 공정 후 시작합니다."));
        const TCHAR* Names[]={TEXT("센서 정상"),TEXT("부품 누락·보충"),TEXT("위치 센서 끊김"),TEXT("파지 실패·재시도"),TEXT("휠 위치 변화"),TEXT("기존 난수 사건"),TEXT("물리 파지 · 정상 운반"),TEXT("물리 파지 · 저마찰"),TEXT("물리 파지 · 과중량 휠"),TEXT("휠 규격 · 과대"),TEXT("휠 규격 · 과소"),TEXT("차문 규격 · 과대"),TEXT("차문 규격 · 과소"),TEXT("차체 도킹 초과 · 후진 복구")};
        for(int32 Kind=0;Kind<UE_ARRAY_COUNT(Names);++Kind)
            Rows->AddSlot().AutoHeight().Padding(0,2)[Button(Names[Kind],[this,Kind](){
                if(IsValid(TestAssemblyLine)){TestAssemblyLine->SetFactoryCheck(Kind);ResetCycle();}
            },Editing)];
        Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(STextBlock).Font(Font).ColorAndOpacity(Accent)
            .Text_Lambda([this](){
                const TCHAR* Names[]={TEXT("센서 정상"),TEXT("부품 누락·보충"),TEXT("위치 센서 끊김"),TEXT("파지 실패·재시도"),TEXT("휠 위치 변화"),TEXT("기존 난수 사건"),TEXT("물리 파지 · 정상 운반"),TEXT("물리 파지 · 저마찰"),TEXT("물리 파지 · 과중량 휠"),TEXT("휠 규격 · 과대"),TEXT("휠 규격 · 과소"),TEXT("차문 규격 · 과대"),TEXT("차문 규격 · 과소"),TEXT("차체 도킹 초과 · 후진 복구")};
                return FText::FromString(IsValid(TestAssemblyLine)?FString(TEXT("선택: "))+Names[TestAssemblyLine->GetFactoryCheck()]:TEXT("조립 라인 연결 없음"));
            }).AutoWrapText(true)];
    }
    if(!bUseAssemblyIntegration)
    {
    Label(TEXT("IK·관절 PID / 체결·접촉 물리 미포함"));
    Heading(TEXT("이식된 기능 검증"));
    for(int32 Toggle=0;Toggle<2;++Toggle)
        Rows->AddSlot().AutoHeight().Padding(0,2)[SNew(SCheckBox)
            .IsChecked_Lambda([this,Toggle](){return WheelProcess&&(Toggle==0?WheelProcess->bShowSensors:WheelProcess->bShowRoute)?ECheckBoxState::Checked:ECheckBoxState::Unchecked;})
            .OnCheckStateChanged_Lambda([this,Toggle](ECheckBoxState V){if(WheelProcess)(Toggle==0?WheelProcess->bShowSensors:WheelProcess->bShowRoute)=V==ECheckBoxState::Checked;})
            [SNew(STextBlock).Font(Font).Text(FText::FromString(Toggle==0?TEXT("센서·휠 중심·장착점 표시"):TEXT("작업 경로·현재 목표 표시")))]];
    Label(TEXT("청록 상자: 가상 센서 기준점\n흰색: 실제 휠 중심 / 노랑: 측정 위치\n빨강: 측정 지연·부품 없음\n초록: 장착점 / 툴 초록: 파지 확인\n파랑: 작업 경유점 / 자홍: 현재 목표\n경로 선은 관절 운동의 실제 궤적이 아닙니다."));
    Rows->AddSlot().AutoHeight().Padding(0,3)[Button(TEXT("센서·휠 작업 확대"),[this](){FocusWorkcell();},true)];
    Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(STextBlock).Font(Font).ColorAndOpacity(Accent)
        .Text_Lambda([this](){return FText::FromString(WheelProcess?WheelProcess->VisualizationSummary():TEXT(""));}).AutoWrapText(true)];
    Label(TEXT("연결된 로봇·공급대의 휠 작업으로 개별 기능을 확인합니다.\n전체 조립 시나리오·랜덤 사건은 실행하지 않습니다."));
    const TCHAR* FeatureNames[]={TEXT("센서 정상 확인"),TEXT("부품 없음 → 보충 요청"),TEXT("통신 끊김 → 대기"),TEXT("파지 실패 → 재시도"),TEXT("위치 변화 → 재계산"),TEXT("물리 파지 · 정상 운반"),TEXT("물리 파지 · 저마찰")};
    for(int32 Kind=0;Kind<UE_ARRAY_COUNT(FeatureNames);++Kind)
        Rows->AddSlot().AutoHeight().Padding(0,2)[Button(FeatureNames[Kind],[this,Kind](){
            if(WheelProcess)WheelProcess->StartFeatureCheck(Kind);
        },Editing)];
    Rows->AddSlot().AutoHeight().Padding(0,3)[Button(TEXT("공급·통신·그리퍼 복원"),[this](){
        if(WheelProcess)WheelProcess->RestoreFeatureInputs();
    },TAttribute<bool>::CreateLambda([this](){return WheelProcess&&WheelProcess->FeaturesEnabled()
        &&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;}))];
    Label(TEXT("누락된 휠은 예비 공급을 요청합니다.\n위치 센서 통신은 5초 안에 복원하세요.\n파지 실패는 재시도 표시 후 복원하세요.\n중단/완료 후 상단 ‘편집 복귀’로 다시 검사합니다."));
    Rows->AddSlot().AutoHeight().Padding(0,3)[Button(TEXT("운반 중 위치 센서 차단"),[this](){
        if(WheelProcess)WheelProcess->InterruptFeaturePose();
    },TAttribute<bool>::CreateLambda([this](){return WheelProcess&&WheelProcess->FeaturesEnabled()
        &&RunState>=EFactoryRunState::Lift&&RunState<=EFactoryRunState::Verify;}))];
    Label(TEXT("허브 판정: 중심 편차 3mm·축 오차 1° 이내에서 진입합니다. 안착은 중심 거리 0.5mm·축 오차 0.25° 이내로 0.15초 유지해야 합니다."));
    Rows->AddSlot().AutoHeight().Padding(0,3)[Button(TEXT("장착 중 허브 센서 차단"),[this](){
        if(WheelProcess)WheelProcess->InterruptHubSensor();
    },TAttribute<bool>::CreateLambda([this](){return WheelProcess&&WheelProcess->FeaturesEnabled()
        &&(RunState==EFactoryRunState::Align||RunState==EFactoryRunState::Verify);}))];
    Rows->AddSlot().AutoHeight().Padding(0,4)[SNew(STextBlock).Font(Font).ColorAndOpacity(Accent)
        .Text_Lambda([this](){return FText::FromString(WheelProcess?WheelProcess->FeatureSummary():TEXT("연결 없음"));}).AutoWrapText(true)];

    }

    Pair(Button(TEXT("실행 취소"),[this](){UndoEdit();},TAttribute<bool>::CreateLambda([this](){return CanUndo();})),
        Button(TEXT("다시 실행"),[this](){RedoEdit();},TAttribute<bool>::CreateLambda([this](){return CanRedo();})));

    Heading(TEXT("시점"));
    Pair(Button(TEXT("공장 전체"), [this]() { FitView(); }, true),
        Button(bUseAssemblyIntegration?TEXT("통합 작업 구역"):TEXT("휠 작업 구역"), [this]() { FocusWorkcell(); }, true));
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(SButton).IsFocusable(false).ContentPadding(6)
        .OnClicked_Lambda([this]() { ToggleView(); return FReply::Handled(); })[
            SNew(STextBlock).Font(Font).Justification(ETextJustify::Center).Text_Lambda([this]() {
                return FText::FromString(bTopView ? TEXT("입체 시점으로 보기") : TEXT("평면 시점으로 보기"));
            })]];
    Pair(Button(TEXT("확대 +"), [this]() { ZoomIn(); }, true),
        Button(TEXT("축소 -"), [this]() { ZoomOut(); }, true));
    Label(TEXT("좌클릭: 선택·이동 / 휠클릭: 평행 이동\n우클릭: 회전 / 휠: 확대·축소 / F: 초점\nCtrl+Z: 실행 취소 / Ctrl+Y: 다시 실행"));
    auto Check=[&Rows,Font,TextColor](const TCHAR* Caption,TFunction<bool()> Get,TFunction<void(bool)> Set)
    {
        Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(SCheckBox)
            .IsChecked_Lambda([Get](){return Get()?ECheckBoxState::Checked:ECheckBoxState::Unchecked;})
            .OnCheckStateChanged_Lambda([Set](ECheckBoxState V){Set(V==ECheckBoxState::Checked);})[
                SNew(STextBlock).Font(Font).ColorAndOpacity(TextColor).Text(FText::FromString(Caption))]];
    };
    Check(TEXT("구역 이름 표시"),[this](){return bZoneLabels;},[this](bool V){SetZoneLabels(V);});
    Check(TEXT("설비 이름 표시"),[this](){return bEquipmentLabels;},[this](bool V){SetEquipmentLabels(V);});
    Check(TEXT("공정 흐름 표시"),[this](){return bRouteVisible;},[this](bool V){SetRouteVisible(V);});

    Heading(TEXT("공장 크기"));
    Rows->AddSlot().AutoHeight().Padding(0,3)[Button(TEXT("공장 테두리 선택"),[this](){SelectHall();},Editing)];
    Label(TEXT("공장 또는 구역 선택 → 노란 손잡이 드래그"));
    Number(TEXT("길이 X (m)"), [this]() -> TOptional<double> { return Layout.LengthCm / 100.0; },
        [this](double Value) { SetHallSize(Value, Layout.WidthCm / 100.0); }, Editing, 12, 200);
    Number(TEXT("너비 Y (m)"), [this]() -> TOptional<double> { return Layout.WidthCm / 100.0; },
        [this](double Value) { SetHallSize(Layout.LengthCm / 100.0, Value); }, Editing, 12, 120);
    Label(TEXT("단위: m · Enter 또는 입력칸 이동으로 적용\n크기 변경 시 설비 크기·위치는 유지"));

    Heading(TEXT("선택한 구역·설비"));
    // SComboBox keeps a raw pointer to its source array. Captured shared pointers
    // deliberately keep each option list alive for the lifetime of its widget.
    auto Choices = MakeShared<TArray<TSharedPtr<FFactoryPanelChoice>>>();
    TSharedPtr<FFactoryPanelChoice> InitialChoice;
    for (const FFactoryLayoutItem& Item : Layout.Items)
    {
        auto Choice = MakeShared<FFactoryPanelChoice>(Item.Id, Item.Id + TEXT(" · ") + Item.DisplayName());
        Choices->Add(Choice);
        if (Item.Id == SelectedId) InitialChoice = Choice;
    }
    Rows->AddSlot().AutoHeight().Padding(0, 4)[SNew(SComboBox<TSharedPtr<FFactoryPanelChoice>>)
        .IsEnabled(Editing).OptionsSource(&Choices.Get()).InitiallySelectedItem(InitialChoice).MaxListHeight(320)
        .OnGenerateWidget_Lambda([Font, Choices](TSharedPtr<FFactoryPanelChoice> Choice) -> TSharedRef<SWidget> {
            return SNew(SBox).WidthOverride(295)[SNew(STextBlock).Font(Font)
                .Text(FText::FromString(Choice.IsValid() ? Choice->Label : FString())).AutoWrapText(true)];
        })
        .OnSelectionChanged_Lambda([this, Generation, Choices](TSharedPtr<FFactoryPanelChoice> Choice, ESelectInfo::Type) {
            if (PanelGeneration == Generation && Choice.IsValid() && CanEdit()) SelectItem(Choice->Id);
        })[SNew(STextBlock).Font(Font).Text_Lambda([this]() {
            const FFactoryLayoutItem* Item = Selected();
            return FText::FromString(Item ? Item->Id + TEXT(" · ") + Item->DisplayName() : TEXT("항목을 선택하세요"));
        }).AutoWrapText(true)]];

    Number(TEXT("중심 X (m)"), [this]() -> TOptional<double> {
        return Selected() ? TOptional<double>(Selected()->Position.X / 100.0) : TOptional<double>();
    }, [this, PanelItemId](double Value) {
        if (SelectedId == PanelItemId && Selected()) SetSelectedPosition(Value, Selected()->Position.Y / 100.0);
    },
        HasSelection, -1000, 1000);
    Number(TEXT("중심 Y (m)"), [this]() -> TOptional<double> {
        return Selected() ? TOptional<double>(Selected()->Position.Y / 100.0) : TOptional<double>();
    }, [this, PanelItemId](double Value) {
        if (SelectedId == PanelItemId && Selected()) SetSelectedPosition(Selected()->Position.X / 100.0, Value);
    },
        HasSelection, -1000, 1000);
    Number(TEXT("방향 (°)"), [this]() -> TOptional<double> {
        return Selected() ? TOptional<double>(Selected()->Yaw) : TOptional<double>();
    }, [this, PanelItemId](double Value) { if (SelectedId == PanelItemId) SetSelectedYaw(Value); }, HasSelection, -360, 360);
    Number(TEXT("구역 길이 (m)"), [this]() -> TOptional<double> {
        return Selected() && Selected()->Kind == EFactoryItemKind::Zone
            ? TOptional<double>(Selected()->Size.X / 100.0) : TOptional<double>();
    }, [this, PanelItemId](double Value) {
        if (SelectedId == PanelItemId && Selected()) SetSelectedSize(Value, Selected()->Size.Y / 100.0);
    },
        ZoneSelected, .2, 200);
    Number(TEXT("구역 너비 (m)"), [this]() -> TOptional<double> {
        return Selected() && Selected()->Kind == EFactoryItemKind::Zone
            ? TOptional<double>(Selected()->Size.Y / 100.0) : TOptional<double>();
    }, [this, PanelItemId](double Value) {
        if (SelectedId == PanelItemId && Selected()) SetSelectedSize(Selected()->Size.X / 100.0, Value);
    },
        ZoneSelected, .2, 200);

    Label(TEXT("이름: 한국어 / 영어"));
    // Commit both names together: rebuilding the panel on every focus-loss
    // would steal focus when the user tabs from the Korean field to English.
    const auto NameDraft = MakeShared<FFactoryNameDraft>();
    if (const FFactoryLayoutItem* Item = Selected())
    {
        NameDraft->ItemId = Item->Id;
        NameDraft->Korean = Item->NameKo;
        NameDraft->English = Item->NameEn;
    }
    auto ApplyNames = [this, Generation, NameDraft]() {
        const FFactoryLayoutItem* Item = Selected();
        if (PanelGeneration != Generation || !CanEdit() || !Item || Item->Id != NameDraft->ItemId) return;
        const FString Korean = NameDraft->Korean.TrimStartAndEnd();
        const FString English = NameDraft->English.TrimStartAndEnd();
        if (Korean != Item->NameKo || English != Item->NameEn) SetSelectedName(Korean, English);
    };
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(SEditableTextBox).Font(Font).IsEnabled(HasSelection)
        .HintText(FText::FromString(TEXT("한국어 이름"))).Text(FText::FromString(NameDraft->Korean))
        .OnTextChanged_Lambda([NameDraft](const FText& Value) { NameDraft->Korean = Value.ToString(); })
        .OnTextCommitted_Lambda([ApplyNames](const FText&, ETextCommit::Type Commit) {
            if (Commit == ETextCommit::OnEnter) ApplyNames();
        })];
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(SEditableTextBox).Font(Font).IsEnabled(HasSelection)
        .HintText(FText::FromString(TEXT("영어 이름"))).Text(FText::FromString(NameDraft->English))
        .OnTextChanged_Lambda([NameDraft](const FText& Value) { NameDraft->English = Value.ToString(); })
        .OnTextCommitted_Lambda([ApplyNames](const FText&, ETextCommit::Type Commit) {
            if (Commit == ETextCommit::OnEnter) ApplyNames();
        })];
    Rows->AddSlot().AutoHeight().Padding(0, 3)[Button(TEXT("이름 적용"), ApplyNames, HasSelection)];
    Label(TEXT("이름 두 칸을 편집한 뒤 '이름 적용' 또는 Enter"));

    auto RoleChoices = MakeShared<TArray<TSharedPtr<FFactoryPanelChoice>>>();
    TSharedPtr<FFactoryPanelChoice> InitialRole;
    for (const TCHAR* RoleId : { TEXT("work"), TEXT("supply"), TEXT("buffer"), TEXT("walkway"), TEXT("rework"), TEXT("reserve") })
    {
        auto Choice = MakeShared<FFactoryPanelChoice>(RoleId, FactoryRoleLabel(RoleId));
        RoleChoices->Add(Choice);
        if (Selected() && Selected()->Role == RoleId) InitialRole = Choice;
    }
    Rows->AddSlot().AutoHeight().Padding(0, 4)[SNew(SComboBox<TSharedPtr<FFactoryPanelChoice>>)
        .IsEnabled(ZoneSelected).OptionsSource(&RoleChoices.Get()).InitiallySelectedItem(InitialRole)
        .OnGenerateWidget_Lambda([Font, RoleChoices](TSharedPtr<FFactoryPanelChoice> Choice) -> TSharedRef<SWidget> {
            return SNew(STextBlock).Font(Font).Text(FText::FromString(Choice->Label));
        })
        .OnSelectionChanged_Lambda([this, Generation, RoleChoices, PanelItemId](TSharedPtr<FFactoryPanelChoice> Choice, ESelectInfo::Type) {
            if (PanelGeneration == Generation && Choice.IsValid() && CanEdit() && SelectedId == PanelItemId) SetSelectedRole(Choice->Id);
        })[SNew(STextBlock).Font(Font).Text_Lambda([this]() {
            return FText::FromString(Selected() && Selected()->Kind == EFactoryItemKind::Zone
                ? FactoryRoleLabel(Selected()->Role) : TEXT("구역 용도: 구역 선택 시 설정"));
        }).AutoWrapText(true)]];
    if(!bUseAssemblyIntegration)
    Pair(Button(TEXT("선택 항목 삭제"), [this]() { DeleteSelected(); }, HasSelection),
        Button(bUseAssemblyIntegration?TEXT("작업 부여"):TEXT("선택 설비 작업 연결"), [this]() { if(bUseAssemblyIntegration)OpenRobotPopup(SelectedId,FVector2D(600,120));else BindSelected(); }, TAttribute<bool>::CreateLambda([this]() {
            const FFactoryLayoutItem* Item = Selected();
            return CanEdit() && Item && (Item->Kind == EFactoryItemKind::VehicleStation
                || Item->Kind == EFactoryItemKind::WheelSupply || Item->Kind == EFactoryItemKind::CartesianHandler || Item->Kind == EFactoryItemKind::ArticulatedRobot);
        })));
    Rows->AddSlot().AutoHeight().Padding(0, 4)[SNew(STextBlock).Font(Font).ColorAndOpacity(TextColor)
        .Text_Lambda([this]() { return FText::FromString(BindingText()); }).AutoWrapText(true)];

    Heading(TEXT("공정 순서·동선"));
    Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(STextBlock).Font(Font).ColorAndOpacity(TextColor)
        .Text_Lambda([this](){return FText::FromString(RouteText());}).AutoWrapText(true)];
    Pair(Button(TEXT("선택 항목 추가"),[this](){RouteAppendSelected();},HasSelection),Button(TEXT("순서에서 제외"),[this](){RouteRemoveSelected();},HasSelection));
    Pair(Button(TEXT("앞 순서로 ↑"),[this](){RouteMoveSelected(-1);},HasSelection),Button(TEXT("뒤 순서로 ↓"),[this](){RouteMoveSelected(1);},HasSelection));
    Number(TEXT("곡선 굽힘 (m)"),[this]()->TOptional<double>{return Layout.RouteBendCm/100.;},[this](double V){SetRouteBend(V*100);},Editing,-20,20);
    Label(TEXT("화살표는 공정 순서 표시입니다. 차량 운반·컨베이어 구동 경로는 아닙니다."));

    Heading(TEXT("배치 저장·불러오기"));
    Pair(Button(TEXT("A 저장"), [this]() { SaveLayout(TEXT("A")); }, Editing),
        Button(TEXT("A 불러오기"), [this]() { LoadLayout(TEXT("A")); }, Editing));
    Pair(Button(TEXT("B 저장"), [this]() { SaveLayout(TEXT("B")); }, Editing),
        Button(TEXT("B 불러오기"), [this]() { LoadLayout(TEXT("B")); }, Editing));
    Rows->AddSlot().AutoHeight().Padding(0, 3)[Button(TEXT("기본 배치로 복원"), [this]() { RestoreDefault(); }, Editing)];
    Label(TEXT("저장 슬롯은 같은 이름으로 덮어씁니다.\n불러오기·기본 복원은 현재 편집 내용을 바꿉니다."));

    Heading(TEXT("배치 점검"));
    Rows->AddSlot().AutoHeight().Padding(0, 3)[SNew(STextBlock).Font(Font)
        .ColorAndOpacity(FLinearColor(.99f, .83f, .48f, 1)).Text_Lambda([this]() {
            return FText::FromString(ValidationText());
        }).AutoWrapText(true)];
    Label(TEXT("설계 검토용 모델 · 실제 공장 치수·안전 인증 아님"));

    TSharedRef<SVerticalBox> Palette=SNew(SVerticalBox);
    TSharedRef<SHorizontalBox> Tabs=SNew(SHorizontalBox);
    for(const TCHAR* Category:{TEXT("전체"),TEXT("구역"),TEXT("로봇"),TEXT("설비")})
    {
        const FString Name=Category;
        Tabs->AddSlot().AutoWidth().Padding(0,0,6,0)[Button((PaletteCategory==Name?TEXT("● "):TEXT(""))+Name,
            [this,Name](){PaletteCategory=Name;bNeedsPanel=true;},true)];
    }
    Tabs->AddSlot().FillWidth(1).Padding(12,0).VAlign(VAlign_Center)[SNew(SCheckBox).IsEnabled(Editing)
        .IsChecked_Lambda([this](){return bSnap?ECheckBoxState::Checked:ECheckBoxState::Unchecked;})
        .OnCheckStateChanged_Lambda([this](ECheckBoxState V){bSnap=V==ECheckBoxState::Checked;})[
            SNew(STextBlock).Font(Font).ColorAndOpacity(TextColor).Text(FText::FromString(TEXT("0.25m 격자 맞춤")))]];
    Tabs->AddSlot().AutoWidth().Padding(0,0,6,0)[Button(TEXT("선택 삭제 (Del)"),[this](){DeleteSelected();},TAttribute<bool>::CreateLambda([this](){return CanEdit()&&Selected()!=nullptr;}))];
    Tabs->AddSlot().AutoWidth().Padding(0,0,6,0)[Button(TEXT("공장 크기 조정"),[this](){SelectHall();FitView();},Editing)];
    Tabs->AddSlot().AutoWidth()[Button(TEXT("배치 취소 (Esc)"),[this](){CancelEdit();},Editing)];
    Palette->AddSlot().AutoHeight()[Tabs];
    TSharedRef<SHorizontalBox> Cards=SNew(SHorizontalBox);
    EnsureRegistry();
    for(const auto& D:Registry->All())
    {
        if(!IsPaletteTypeAllowed(D->TypeId.ToString())||D->TypeId==TEXT("assembly_wheel_right"))continue;
        if(PaletteCategory!=TEXT("전체")&&D->Category!=PaletteCategory)continue;
        const FString Id=D->TypeId.ToString();
        Cards->AddSlot().AutoWidth().Padding(0,8,8,5)[SNew(SBox).WidthOverride(170).HeightOverride(82)[
            SNew(SButton).IsFocusable(false).IsEnabled(Editing).ToolTipText(FText::FromString(D->Description))
                .OnClicked_Lambda([this,Id](){BeginPlacement(Id);return FReply::Handled();})[
                    SNew(STextBlock).Font(Font).Justification(ETextJustify::Center).AutoWrapText(true)
                    .Text(FText::FromString(D->TypeId==TEXT("assembly_wheel_left")?TEXT("휠 작업 로봇\n(Wheel Robot)"):D->NameKo+TEXT("\n(")+D->NameEn+TEXT(")")))]]];
    }
    Palette->AddSlot().AutoHeight()[SNew(SScrollBox).Orientation(Orient_Horizontal).ScrollBarAlwaysVisible(true)+SScrollBox::Slot()[Cards]];
    Palette->AddSlot().AutoHeight().Padding(0,3)[SNew(STextBlock).Font(Font).ColorAndOpacity(Accent)
        .Text_Lambda([this](){return FText::FromString(!CanEdit()?TEXT("실행 중에는 설비를 추가하거나 배치를 변경할 수 없습니다."):
            (!PendingTypeId.IsEmpty()?TEXT("배치 중 · 바닥 클릭 / 구역은 드래그 · Esc 취소"):TEXT("추가 도구 · 항목 선택 → 바닥 클릭으로 배치")));}).AutoWrapText(true)];
    const auto OldSurface=ViewportSurface;
    if(!ViewportSurface.IsValid())ViewportSurface=SNew(SFactoryViewport).Runtime(this);
    const bool bNewPanel = !Panel.IsValid();
    if (bNewPanel)
        Panel = SNew(SOverlay).Visibility(EVisibility::SelfHitTestInvisible)
            + SOverlay::Slot()[ViewportSurface.ToSharedRef()];
    const TSharedRef<SOverlay> Overlay = StaticCastSharedRef<SOverlay>(Panel.ToSharedRef());
    while (Overlay->GetNumWidgets() > 1)
        Overlay->RemoveSlot(Overlay->GetChildren()->GetChildAt(1));
    Overlay->AddSlot().HAlign(HAlign_Left).VAlign(VAlign_Fill).Padding(12)[
            SNew(SBox).WidthOverride(330)[SNew(SBorder).Padding(10)
                .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                .BorderBackgroundColor(FLinearColor(.025f,.035f,.05f,.96f))[
                    SAssignNew(PanelScroll,SScrollBox).ScrollBarAlwaysVisible(true)+SScrollBox::Slot()[Rows]]]];
    Overlay->AddSlot().HAlign(HAlign_Fill).VAlign(VAlign_Bottom).Padding(354,0,12,12)[
            SNew(SBorder).Padding(10).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                .BorderBackgroundColor(FLinearColor(.025f,.035f,.05f,.96f))[Palette]];
    if(bRobotPopup&&CanEdit()&&Selected())
    {
        auto Popup=SNew(SVerticalBox);
        Popup->AddSlot().AutoHeight()[SNew(STextBlock).Font(Font).Text(FText::FromString(Selected()->NameKo+TEXT(" · 작업 연결")))];
        Popup->AddSlot().AutoHeight().Padding(0,8)[SNew(STextBlock).Font(Font).AutoWrapText(true)
            .Text_Lambda([this](){return FText::FromString(PendingTaskId.IsEmpty()?TEXT("① 차체의 초록색 작업 지점을 클릭하세요."):TEXT("② 장착할 휠 또는 도어 부품을 클릭하세요."));})];
        Popup->AddSlot().AutoHeight()[SNew(STextBlock).Font(Font).AutoWrapText(true).Text_Lambda([this](){return FText::FromString(Status);})];
        for(const auto& Task:Layout.Tasks)if(Task.RobotId==SelectedId)
        {
            const auto* S=FactoryTasks::Find(Task.Id);const FString Id=Task.Id;
            Popup->AddSlot().AutoHeight().Padding(0,4)[Button((S?S->Label:Id)+TEXT(" · 연결 해제"),[this,Id](){UnassignTask(Id);},Editing)];
        }
        Popup->AddSlot().AutoHeight().Padding(0,6)[Button(TEXT("닫기 · Esc"),[this](){CloseRobotPopup();},true)];
        Overlay->AddSlot().HAlign(HAlign_Right).VAlign(VAlign_Top).Padding(12)[SNew(SBox).WidthOverride(320).MaxDesiredHeight(420)
            [SNew(SBorder).Padding(12).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(.035f,.07f,.10f,.98f))
                [SNew(SScrollBox)+SScrollBox::Slot()[Popup]]]];
    }
    PanelScroll->SetScrollOffset(SavedScrollOffset);
    if(bNewPanel)GEngine->GameViewport->AddViewportWidgetContent(Panel.ToSharedRef(),30);
    if(RestoreViewportFocus||!OldSurface.IsValid())FSlateApplication::Get().SetKeyboardFocus(ViewportSurface.ToSharedRef(),EFocusCause::SetDirectly);
    if(bNewPanel)FSlateApplication::Get().RegisterInputPreProcessor(
        MakeShared<FFactoryShortcutInput>(this,Panel.ToSharedRef(),ViewportSurface.ToSharedRef(),RestoreViewportFocus||!OldSurface.IsValid()),0);
    bNeedsPanel=false;
}

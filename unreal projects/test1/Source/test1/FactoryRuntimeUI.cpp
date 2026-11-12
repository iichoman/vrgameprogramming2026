#include "FactorySensorChart.h"
#include "FactoryRuntime.h"
#include "FactoryEquipmentRegistry.h"
#include "FactoryViewport.h"
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
    if(!GEngine||!GEngine->GameViewport)return;
    auto& Model=Inspection();const uint64 Generation=++PanelGeneration;
    const bool RestoreFocus=ViewportSurface.IsValid()&&(ViewportSurface->HasKeyboardFocus()||ViewportSurface->HasAnyUserFocus());
    const FString FontFile=FPaths::ProjectContentDir()/TEXT("DigitalTwin/Fonts/NanumGothic.ttf");
    auto Composite=MakeShared<FCompositeFont>(FName(TEXT("NanumGothic")),FontFile,EFontHinting::Default,EFontLoadingPolicy::LazyLoad);
    const FSlateFontInfo Font(Composite,12),HeadingFont(Composite,15),Small(Composite,10);
    const FLinearColor Text(.92f,.95f,.98f),Muted(.58f,.68f,.78f),Blue(.2f,.72f,1.f),Orange(1.f,.55f,.2f);
    const TAttribute<bool> Editing=TAttribute<bool>::CreateLambda([this](){return CanEdit();});
    auto Button=[this,Font](const FString& Label,TFunction<void()> Action,TAttribute<bool> Enabled=true)->TSharedRef<SWidget>
    {return SNew(SButton).IsFocusable(false).IsEnabled(Enabled).ContentPadding(FMargin(9,7)).OnClicked_Lambda([this,Action](){Action();if(ViewportSurface)FSlateApplication::Get().SetKeyboardFocus(ViewportSurface.ToSharedRef());return FReply::Handled();})[SNew(STextBlock).Font(Font).Text(FText::FromString(Label))];};
    auto Card=[](TSharedRef<SWidget> Child)->TSharedRef<SWidget>{return SNew(SBorder).Padding(12).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(.025f,.038f,.057f,.97f))[Child];};
    TSharedRef<SVerticalBox> Rows=SNew(SVerticalBox);
    auto Label=[&](const FString& Value,bool Heading=false){Rows->AddSlot().AutoHeight().Padding(0,Heading?9:4)[SNew(STextBlock).Font(Heading?HeadingFont:Font).ColorAndOpacity(Heading?Blue:Text).Text(FText::FromString(Value)).AutoWrapText(true)];};
    auto Dynamic=[&](TFunction<FString()> Get){Rows->AddSlot().AutoHeight().Padding(0,5)[SNew(STextBlock).Font(Font).ColorAndOpacity(Text).AutoWrapText(true).Text_Lambda([Get](){return FText::FromString(Get());})];};
    auto Number=[&](const FString& Caption,TFunction<double()> Get,TFunction<void(double)> Set,double Min,double Max)
    {Rows->AddSlot().AutoHeight().Padding(0,4)[SNew(SHorizontalBox)+SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center)[SNew(STextBlock).Font(Font).Text(FText::FromString(Caption))]
        +SHorizontalBox::Slot().FillWidth(1)[SNew(SNumericEntryBox<double>).Font(Font).IsEnabled(Editing).AllowSpin(false).MinValue(Min).MaxValue(Max).MaxFractionalDigits(3)
            .Value_Lambda([Get]()->TOptional<double>{return Get();}).OnValueCommitted_Lambda([this,Generation,Get,Set](double V,ETextCommit::Type Commit){if(Generation==PanelGeneration&&CanEdit()&&Commit!=ETextCommit::OnCleared&&!FMath::IsNearlyEqual(V,Get(),.00001))Set(V);})]];};
    auto Pair=[&](TSharedRef<SWidget> A,TSharedRef<SWidget> B){Rows->AddSlot().AutoHeight().Padding(0,4)[SNew(SHorizontalBox)+SHorizontalBox::Slot().FillWidth(1).Padding(0,0,3,0)[A]+SHorizontalBox::Slot().FillWidth(1).Padding(3,0,0,0)[B]];};

    // Top-level actions stay visible; property controls appear only for the active context.
    auto Top=SNew(SHorizontalBox);
    Top->AddSlot().FillWidth(1).VAlign(VAlign_Center)[SNew(STextBlock).Font(HeadingFont).ColorAndOpacity(Text).Text_Lambda([this](){return FText::FromString(TEXT("공장 시뮬레이터  ·  ")+StateText()+(bDirty?TEXT("  • 미저장"):TEXT("")));})];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("실행"),[this](){if(!StartCycle()){RefreshInspection();OpenDetails(EFactoryDetailTab::Problems);}},Editing)];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("정지"),[this](){StopCycle();},TAttribute<bool>::CreateLambda([this](){return RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;}))];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("편집 복귀"),[this](){ResetCycle();},TAttribute<bool>::CreateLambda([this](){return !CanEdit();}))];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("실행 취소"),[this](){UndoEdit();},TAttribute<bool>::CreateLambda([this](){return CanUndo();}))];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("다시 실행"),[this](){RedoEdit();},TAttribute<bool>::CreateLambda([this](){return CanRedo();}))];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("공장·저장"),[this](){OpenDetails(EFactoryDetailTab::Layout);})];
    Top->AddSlot().AutoWidth().Padding(3,0)[Button(TEXT("상세 정보"),[this](){Inspection().PanelOpen=!Inspection().PanelOpen;bNeedsPanel=true;})];

    if(Model.Tab==EFactoryDetailTab::Selection)
    {
        Label(TEXT("오브젝트"),true);
        auto Choices=MakeShared<TArray<TSharedPtr<FFactoryPanelChoice>>>();TSharedPtr<FFactoryPanelChoice> Initial;
        for(const auto& Id:InspectableObjects()){auto C=MakeShared<FFactoryPanelChoice>(Id,ObjectName(Id));Choices->Add(C);if(Id==CurrentObjectId())Initial=C;}
        Rows->AddSlot().AutoHeight().Padding(0,5)[SNew(SComboBox<TSharedPtr<FFactoryPanelChoice>>).OptionsSource(&Choices.Get()).InitiallySelectedItem(Initial).MaxListHeight(300)
            .OnGenerateWidget_Lambda([Choices,Font](TSharedPtr<FFactoryPanelChoice> C)->TSharedRef<SWidget>{return SNew(SBox).WidthOverride(300)[SNew(STextBlock).Font(Font).Text(FText::FromString(C->Label)).AutoWrapText(true)];})
            .OnSelectionChanged_Lambda([this,Choices,Generation](TSharedPtr<FFactoryPanelChoice> C,ESelectInfo::Type){if(C&&PanelGeneration==Generation)SelectObject(C->Id);})[
                SNew(STextBlock).Font(Font).AutoWrapText(true).Text(FText::FromString(CurrentObjectId().IsEmpty()?TEXT("이름으로 찾기…"):ObjectName(CurrentObjectId())))]];
        if(CurrentObjectId().IsEmpty())Label(TEXT("화면의 로봇·차체·부품을 클릭하거나 위 목록에서 선택하세요."));
        else
        {
            Label(ObjectName(CurrentObjectId()),true);
            Pair(Button(TEXT("선택 위치 보기 · F"),[this](){FocusSelected();}),Button(TEXT("선택 센서"),[this](){Inspection().ShowAllSensors=false;OpenDetails(EFactoryDetailTab::Sensors);}));
            if(const auto* Item=Selected())
            {
                const FString Id=Item->Id;
                if(Item->EffectiveTypeId().StartsWith(TEXT("assembly_")))
                {
                    if(bRobotPopup)Label(PendingTaskId.IsEmpty()?TEXT("① 차체의 초록색 작업 지점을 클릭하세요."):TEXT("② 연결할 실제 부품을 클릭하세요."));
                    else Rows->AddSlot().AutoHeight()[Button(TEXT("차체 지점 → 부품 연결"),[this,Id](){OpenRobotPopup(Id,FVector2D::ZeroVector);},Editing)];
                    for(const auto& T:Layout.Tasks)if(T.RobotId==Id){const FString TaskId=T.Id;const auto* S=FactoryTasks::Find(TaskId);Label((S?S->Label:TaskId)+TEXT(" ← ")+ObjectName(T.PartId));Rows->AddSlot().AutoHeight()[Button(TEXT("이 연결 해제"),[this,TaskId](){UnassignTask(TaskId);},Editing)];}
                    if(bRobotPopup)Dynamic([this](){return Status;});
                    if(bRobotPopup)Rows->AddSlot().AutoHeight().Padding(0,5)[Button(TEXT("연결 모드 종료 · Esc"),[this](){CloseRobotPopup();})];
                }
                Label(TEXT("배치"),true);
                Number(TEXT("X 위치 (m)"),[this](){return Selected()?Selected()->Position.X/100:0.;},[this,Id](double V){if(SelectedId==Id&&Selected())SetSelectedPosition(V,Selected()->Position.Y/100);},-1000,1000);
                Number(TEXT("Y 위치 (m)"),[this](){return Selected()?Selected()->Position.Y/100:0.;},[this,Id](double V){if(SelectedId==Id&&Selected())SetSelectedPosition(Selected()->Position.X/100,V);},-1000,1000);
                Number(TEXT("방향 (°)"),[this](){return Selected()?Selected()->Yaw:0.;},[this,Id](double V){if(SelectedId==Id)SetSelectedYaw(V);},-360,360);
                if(Item->Kind==EFactoryItemKind::Zone)
                {
                    Number(TEXT("길이 (m)"),[this](){return Selected()?Selected()->Size.X/100:0.;},[this,Id](double V){if(SelectedId==Id&&Selected())SetSelectedSize(V,Selected()->Size.Y/100);},.2,200);
                    Number(TEXT("너비 (m)"),[this](){return Selected()?Selected()->Size.Y/100:0.;},[this,Id](double V){if(SelectedId==Id&&Selected())SetSelectedSize(Selected()->Size.X/100,V);},.2,200);
                    auto Roles=SNew(SWrapBox).UseAllottedSize(true);for(const TCHAR* RoleId:{TEXT("work"),TEXT("supply"),TEXT("buffer"),TEXT("walkway"),TEXT("rework"),TEXT("reserve")}){const FString R=RoleId;Roles->AddSlot().Padding(0,3,4,0)[Button(FactoryRoleLabel(R),[this,R](){SetSelectedRole(R);},Editing)];}Rows->AddSlot().AutoHeight()[Roles];
                }
                Label(TEXT("표시 이름"),true);auto Draft=MakeShared<FFactoryNameDraft>();Draft->ItemId=Id;Draft->Korean=Item->NameKo;Draft->English=Item->NameEn;
                Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(SEditableTextBox).Font(Font).IsEnabled(Editing).Text(FText::FromString(Draft->Korean)).HintText(FText::FromString(TEXT("한국어 이름"))).OnTextChanged_Lambda([Draft](const FText& V){Draft->Korean=V.ToString();})];
                Rows->AddSlot().AutoHeight().Padding(0,3)[SNew(SEditableTextBox).Font(Font).IsEnabled(Editing).Text(FText::FromString(Draft->English)).HintText(FText::FromString(TEXT("영어 이름"))).OnTextChanged_Lambda([Draft](const FText& V){Draft->English=V.ToString();})];
                Pair(Button(TEXT("이름 적용"),[this,Draft,Generation](){if(Generation==PanelGeneration&&SelectedId==Draft->ItemId)SetSelectedName(Draft->Korean.TrimStartAndEnd(),Draft->English.TrimStartAndEnd());},Editing),Button(TEXT("오브젝트 삭제"),[this](){DeleteSelected();},Editing));
                if(!bUseAssemblyIntegration)Rows->AddSlot().AutoHeight()[Button(TEXT("작업 연결"),[this](){BindSelected();},Editing)];
            }
            else
            {
                Dynamic([this](){const FBox B=ObjectBounds(CurrentObjectId());return B.IsValid?FString::Printf(TEXT("중심 위치\nX %.3fm   Y %.3fm   Z %.3fm"),B.GetCenter().X/100,B.GetCenter().Y/100,B.GetCenter().Z/100):TEXT("오브젝트를 찾을 수 없습니다.");});
                Label(CurrentObjectId().StartsWith(TEXT("part_"))?TEXT("편집 모드에서 직접 드래그하여 공급 위치를 옮길 수 있습니다."):TEXT("현재 이 오브젝트는 조회만 가능합니다."));
            }
        }
    }
    else if(Model.Tab==EFactoryDetailTab::Problems)
    {
        Label(TEXT("배치·실행 문제"),true);
        if(Model.Issues.IsEmpty())Label(TEXT("현재 감지된 문제 없음\n실행 시 도달·센서 조건을 추가로 확인합니다."));
        for(const auto& I:Model.Issues)
        {
            Label((I.Level==EFactoryIssueLevel::Error?TEXT("! "):TEXT("△ "))+I.Summary,true);
            Label(TEXT("로봇: ")+(I.RobotId.IsEmpty()?TEXT("미할당 또는 해당 없음"):ObjectName(I.RobotId))+TEXT("\n부품: ")+(I.PartId.IsEmpty()?TEXT("해당 없음"):ObjectName(I.PartId))+TEXT("\n공정: ")+(FactoryTasks::Find(I.TaskId)?FactoryTasks::Find(I.TaskId)->Label:TEXT("공통 라인")));
            Label(I.Detail);Label(I.Remedy);const FString Id=I.Id;Rows->AddSlot().AutoHeight().Padding(0,3,0,12)[Button(TEXT("문제 위치 보기"),[this,Id](){FocusIssue(Id);})];
        }

    }
    else if(Model.Tab==EFactoryDetailTab::Process){Label(TEXT("공정 진행"),true);Dynamic([this](){return ProcessDetails();});}
    else if(Model.Tab==EFactoryDetailTab::Results){Label(TEXT("작업 결과"),true);Dynamic([this](){return ResultDetails();});}
    else if(Model.Tab==EFactoryDetailTab::Sensors)
    {
        Label(TEXT("센서 모니터"),true);
        Pair(Button(TEXT("모든 센서"),[this](){Inspection().ShowAllSensors=true;bNeedsPanel=true;}),Button(TEXT("선택 대상만"),[this](){Inspection().ShowAllSensors=false;bNeedsPanel=true;}));
        auto ValueText=[this](const FString& Id){const auto* C=Inspection().Sensors.Find(Id);if(!C)return FString(TEXT("연결 정보 없음"));const auto& V=C->Latest;
            const TCHAR* State=V.State()==EFactorySampleState::Disconnected?TEXT("미연결"):V.State()==EFactorySampleState::Invalid?TEXT("측정 무효"):V.State()==EFactorySampleState::Stale?TEXT("오래된 측정"):TEXT("정상");
            return V.Label+TEXT(" · ")+State+(V.State()==EFactorySampleState::Valid?FString::Printf(TEXT("  %.3f %s"),V.Value,*V.Unit):TEXT("  —"));};
        Dynamic([this,ValueText](){const auto* C=Inspection().Sensors.Find(Inspection().SelectedChannel);return ValueText(Inspection().SelectedChannel)+(C?TEXT("\n출처: ")+C->Latest.Source+TEXT("\n대상: ")+ObjectName(C->Latest.ObjectId):FString());});
        Rows->AddSlot().AutoHeight().Padding(0,5)[SNew(SFactorySensorChart).Model(InspectionState)];
                Dynamic([this](){const auto* C=Inspection().Sensors.Find(Inspection().SelectedChannel);if(!C||C->History.IsEmpty())return FString(TEXT("측정 이력 없음"));double Min=DBL_MAX,Max=-DBL_MAX;for(const auto& P:C->History)if(P.Valid){Min=FMath::Min(Min,P.Value);Max=FMath::Max(Max,P.Value);}return Min>Max?FString(TEXT("유효 측정 이력 없음")):FString::Printf(TEXT("%.2f–%.2f초 · %.3f–%.3f %s\n세로축 자동 범위 · 최대 180회\n무효 측정은 선을 연결하지 않음"),C->History[0].Time,C->History.Last().Time,Min,Max,*C->Latest.Unit);});
        TArray<FString> Channels;Model.Sensors.GetKeys(Channels);Channels.Sort();int Shown=0;
        for(const FString& Id:Channels){const auto& C=Model.Sensors[Id];const auto* Task=Layout.Tasks.FindByPredicate([&](const auto& T){return T.Id==C.Latest.TaskId;});const bool PartChannel=Task&&(Task->PartId.IsEmpty()?TEXT("part_")+Task->Id:Task->PartId)==CurrentObjectId();if(!Model.ShowAllSensors&&C.Latest.ObjectId!=CurrentObjectId()&&!PartChannel&&!Id.StartsWith(TEXT("extension.")))continue;++Shown;
            Rows->AddSlot().AutoHeight().Padding(0,4)[SNew(SButton).IsFocusable(false).ContentPadding(7).OnClicked_Lambda([this,Id](){Inspection().SelectedChannel=Id;return FReply::Handled();})[
                SNew(STextBlock).Font(Font).AutoWrapText(true).Text_Lambda([this,ValueText,Id](){const auto* Channel=Inspection().Sensors.Find(Id);return FText::FromString(ValueText(Id)+(Channel&&!Channel->Latest.ObjectId.IsEmpty()?TEXT("\n")+ObjectName(Channel->Latest.ObjectId):FString()));})]];}
        if(!Shown)Label(TEXT("이 오브젝트에 연결된 센서가 없습니다."));
    }
    else
    {
        Label(TEXT("공장·보기"),true);
        Pair(Button(TEXT("전체 보기"),[this](){FitView();}),Button(TEXT("작업 구역 보기"),[this](){FocusWorkcell();}));
        Pair(Button(TEXT("평면 / 입체"),[this](){ToggleView();}),Button(TEXT("공장 테두리 선택"),[this](){SelectHall();FitView();},Editing));
        Number(TEXT("공장 길이 (m)"),[this](){return Layout.LengthCm/100;},[this](double V){SetHallSize(V,Layout.WidthCm/100);},12,200);
        Number(TEXT("공장 너비 (m)"),[this](){return Layout.WidthCm/100;},[this](double V){SetHallSize(Layout.LengthCm/100,V);},12,120);
        Rows->AddSlot().AutoHeight().Padding(0,5)[SNew(SCheckBox).IsChecked_Lambda([this](){return bRouteVisible?ECheckBoxState::Checked:ECheckBoxState::Unchecked;}).OnCheckStateChanged_Lambda([this](ECheckBoxState V){SetRouteVisible(V==ECheckBoxState::Checked);})[SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("공정 흐름 표시")))]];
        Label(TEXT("배치 저장"),true);Label(TEXT("A/B는 서로 다른 배치 저장 슬롯입니다. 같은 슬롯 저장 시 덮어씁니다."));
        Pair(Button(TEXT("A 저장"),[this](){SaveLayout(TEXT("A"));},Editing),Button(TEXT("A 불러오기"),[this](){LoadLayout(TEXT("A"));},Editing));
        Pair(Button(TEXT("B 저장"),[this](){SaveLayout(TEXT("B"));},Editing),Button(TEXT("B 불러오기"),[this](){LoadLayout(TEXT("B"));},Editing));
        Rows->AddSlot().AutoHeight()[Button(TEXT("기본 배치로 복원"),[this](){RestoreDefault();},Editing)];
        Label(TEXT("공정 흐름 편집"),true);Dynamic([this](){return RouteText();});
        Pair(Button(TEXT("선택 구역 추가"),[this](){RouteAppendSelected();},Editing),Button(TEXT("순서에서 제외"),[this](){RouteRemoveSelected();},Editing));
        Pair(Button(TEXT("앞 순서로"),[this](){RouteMoveSelected(-1);},Editing),Button(TEXT("뒤 순서로"),[this](){RouteMoveSelected(1);},Editing));
        Number(TEXT("곡선 굽힘 (m)"),[this](){return Layout.RouteBendCm/100;},[this](double V){SetRouteBend(V*100);},-20,20);
        Label(TEXT("흐름선은 순서 표시이며 실제 운반 경로가 아닙니다."));
    }

    auto Palette=SNew(SVerticalBox);auto PaletteTop=SNew(SHorizontalBox);
    PaletteTop->AddSlot().AutoWidth()[Button(Model.PaletteOpen?TEXT("추가 도구 접기"):TEXT("설비·구역 추가"),[this](){Inspection().PaletteOpen=!Inspection().PaletteOpen;bNeedsPanel=true;})];
    PaletteTop->AddSlot().AutoWidth().Padding(8,0)[SNew(SCheckBox).IsEnabled(Editing).IsChecked_Lambda([this](){return bSnap?ECheckBoxState::Checked:ECheckBoxState::Unchecked;}).OnCheckStateChanged_Lambda([this](ECheckBoxState V){bSnap=V==ECheckBoxState::Checked;})[SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("25cm 격자 맞춤")))]];
    PaletteTop->AddSlot().FillWidth(1).VAlign(VAlign_Center).Padding(12,0)[SNew(STextBlock).Font(Small).ColorAndOpacity(Muted).AutoWrapText(true).Text_Lambda([this](){return FText::FromString(PendingTypeId.IsEmpty()?TEXT("선택·이동: 좌클릭   카메라: 휠클릭 / 우클릭   초점: F"):TEXT("바닥을 클릭해 배치 · 구역은 드래그 · Esc 취소"));})];
    Palette->AddSlot().AutoHeight()[PaletteTop];
    Palette->AddSlot().AutoHeight().Padding(0,5)[SNew(STextBlock).Font(Small).ColorAndOpacity(Muted).OverflowPolicy(ETextOverflowPolicy::Ellipsis).Text_Lambda([this](){return FText::FromString(Status);})];
    if(Model.PaletteOpen)
    {auto Cards=SNew(SHorizontalBox);EnsureRegistry();for(const auto& D:Registry->All())if(IsPaletteTypeAllowed(D->TypeId.ToString())&&D->TypeId!=TEXT("assembly_wheel_right"))
        {const FString Id=D->TypeId.ToString();Cards->AddSlot().AutoWidth().Padding(0,6,8,0)[Button(Id==TEXT("assembly_wheel_left")?TEXT("＋ 휠 작업 로봇"):TEXT("＋ ")+D->NameKo,[this,Id](){BeginPlacement(Id);},Editing)];}
        Palette->AddSlot().AutoHeight()[SNew(SScrollBox).Orientation(Orient_Horizontal)+SScrollBox::Slot()[Cards]];}
    auto Chip=SNew(SVerticalBox);
    Chip->AddSlot().AutoHeight()[SNew(STextBlock).Font(Font).ColorAndOpacity(Blue).AutoWrapText(true).Text_Lambda([this](){return FText::FromString(CurrentObjectId().IsEmpty()?TEXT("오브젝트를 선택하세요"):TEXT("선택됨 · ")+ObjectName(CurrentObjectId()));})];
    Chip->AddSlot().AutoHeight().Padding(0,5)[SNew(STextBlock).Font(Small).ColorAndOpacity(Muted).Text(FText::FromString(TEXT("파랑: 선택  /  주황: 문제  /  빨강: 확인 중")))];
    Chip->AddSlot().AutoHeight()[SNew(SButton).IsFocusable(false).OnClicked_Lambda([this](){OpenDetails(EFactoryDetailTab::Problems);return FReply::Handled();})[
        SNew(STextBlock).Font(Font).ColorAndOpacity(Orange).Text_Lambda([this](){return FText::FromString(FString::Printf(TEXT("문제 %d건 · 상세 보기"),Inspection().Issues.Num()));})]];
    if(!ViewportSurface)ViewportSurface=SNew(SFactoryViewport).Runtime(this);const bool NewPanel=!Panel;
    if(NewPanel)Panel=SNew(SOverlay).Visibility(EVisibility::SelfHitTestInvisible)+SOverlay::Slot()[ViewportSurface.ToSharedRef()];
    auto Overlay=StaticCastSharedRef<SOverlay>(Panel.ToSharedRef());while(Overlay->GetNumWidgets()>1)Overlay->RemoveSlot(Overlay->GetChildren()->GetChildAt(1));
    Overlay->AddSlot().HAlign(HAlign_Fill).VAlign(VAlign_Top).Padding(10)[Card(Top)];
    Overlay->AddSlot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(10,76,0,0)[SNew(SBox).WidthOverride(260)[Card(Chip)]];
    Overlay->AddSlot().HAlign(HAlign_Fill).VAlign(VAlign_Bottom).Padding(10)[Card(Palette)];
    if(Model.PanelOpen)
    {
        auto PanelBody=SNew(SVerticalBox);auto Tabs=SNew(SWrapBox).UseAllottedSize(true);
        const TCHAR* Names[]={TEXT("선택"),TEXT("문제"),TEXT("공정"),TEXT("센서"),TEXT("결과")};
        for(int I=0;I<5;++I){const auto Tab=EFactoryDetailTab(I);Tabs->AddSlot().Padding(0,0,4,5)[Button((Model.Tab==Tab?TEXT("● "):TEXT(""))+FString(Names[I]),[this,Tab](){OpenDetails(Tab);})];}
        Tabs->AddSlot().Padding(0,0,0,5)[Button(TEXT("닫기"),[this](){Inspection().PanelOpen=false;bNeedsPanel=true;})];
        PanelBody->AddSlot().AutoHeight()[Tabs];PanelBody->AddSlot().FillHeight(1)[SAssignNew(PanelScroll,SScrollBox).ScrollBarAlwaysVisible(true)+SScrollBox::Slot()[Rows]];
        Overlay->AddSlot().HAlign(HAlign_Right).VAlign(VAlign_Fill).Padding(0,76,10,Model.PaletteOpen?140:85)[SNew(SBox).WidthOverride(348)[Card(PanelBody)]];
    }
    if(NewPanel)GEngine->GameViewport->AddViewportWidgetContent(Panel.ToSharedRef(),30);
    if(RestoreFocus||NewPanel)FSlateApplication::Get().SetKeyboardFocus(ViewportSurface.ToSharedRef(),EFocusCause::SetDirectly);
    if(NewPanel)FSlateApplication::Get().RegisterInputPreProcessor(MakeShared<FFactoryShortcutInput>(this,Panel.ToSharedRef(),ViewportSurface.ToSharedRef(),true),0);
    bNeedsPanel=false;
}

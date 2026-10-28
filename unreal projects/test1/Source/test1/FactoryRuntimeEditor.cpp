#include "FactoryRuntime.h"
#include "FactoryEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "Components/SceneComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "DrawDebugHelpers.h"

namespace
{
const FVector2D Handles[]={{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};
FVector Bezier(FVector A,FVector B,FVector C,FVector D,double T)
{const double U=1-T;return U*U*U*A+3*U*U*T*B+3*U*T*T*C+T*T*T*D;}
FString ViewSettingsPath(){return FPaths::ProjectSavedDir()/TEXT("FactoryEditorView.json");}
}
void AFactoryRuntime::EnsureRegistry()
{
    if(Registry)return;Registry=NewObject<UFactoryEquipmentRegistry>(this);Registry->Initialize();
    for(const auto& D:ExtraEquipment){FString Error;if(!Registry->Register(D,Error))Status=Error;}
}
void AFactoryRuntime::BeginEdit(){History.Begin(Layout,SelectedId);}
void AFactoryRuntime::EndEdit(){History.Commit(Layout);RefreshDirty();}
void AFactoryRuntime::FinishDiscreteEdit(){if(!bDragging)EndEdit();else RefreshDirty();}
void AFactoryRuntime::RefreshDirty(){bDirty=Layout.ToJson()!=SavedSnapshot;}
void AFactoryRuntime::UndoEdit()
{
    if(!CanEdit())return;CancelEdit();
    if(History.Undo(Layout,SelectedId)){Rebuild();RefreshDirty();Status=TEXT("실행 취소 · Ctrl+Y로 다시 실행할 수 있습니다.");}
}
void AFactoryRuntime::RedoEdit()
{
    if(!CanEdit())return;CancelEdit();
    if(History.Redo(Layout,SelectedId)){Rebuild();RefreshDirty();Status=TEXT("다시 실행했습니다.");}
}
void AFactoryRuntime::CancelEdit()
{
    bDragging=bCreateZone=bPanning=bOrbiting=false;ResizeHandle=-1;PendingTypeId.Empty();
    if(History.Cancel(Layout,SelectedId)){Rebuild();RefreshDirty();}
}
void AFactoryRuntime::SelectHall(){if(!CanEdit())return;CancelEdit();SelectedId.Empty();bHallSelected=true;RefreshHighlights();bNeedsPanel=true;}
void AFactoryRuntime::BeginPlacement(const FString& Id)
{
    if(!CanEdit())return;CancelEdit();EnsureRegistry();const auto* D=Registry->Find(Id);if(!D)return;
    PendingTypeId=Id;bCreateZone=D->Kind==TEXT("zone");bHallSelected=false;
    Status=bCreateZone?TEXT("바닥에서 드래그하여 구역 만들기 · Esc 취소"):D->NameKo+TEXT(": 바닥을 클릭해 배치 · Esc 취소");
}
void AFactoryRuntime::AddRegisteredItem(const FString& Id,const FVector2D& Point)
{
    if(!CanEdit()||Layout.Items.Num()>=128)return;EnsureRegistry();const auto* D=Registry->Find(Id);if(!D)return;
    BeginEdit();FFactoryLayoutItem Item;Item.Id=TEXT("item_")+FGuid::NewGuid().ToString(EGuidFormats::Digits);
    Item.TypeId=Id;Item.NameKo=D->NameKo;Item.NameEn=D->NameEn;Item.Size=D->FootprintCm;
    FFactoryLayout::ParseKind(D->Kind,Item.Kind);Item.Position=Point;
    if(bSnap){Item.Position.X=FMath::GridSnap(Item.Position.X,SnapCm);Item.Position.Y=FMath::GridSnap(Item.Position.Y,SnapCm);}
    Layout.Items.Add(Item);SelectedId=Item.Id;bHallSelected=false;Rebuild();EndEdit();
    Status=D->NameKo+TEXT(" 추가 완료 · 필요하면 왼쪽 패널에서 휠 작업에 연결하세요.");
}
void AFactoryRuntime::SetZoneLabels(bool V)
{bZoneLabels=V;for(const auto& I:Layout.Items)if(I.Kind==EFactoryItemKind::Zone)if(auto* A=Equipment.FindRef(I.Id).Get())A->SetLabelVisible(V);SaveViewSettings();}
void AFactoryRuntime::SetEquipmentLabels(bool V)
{bEquipmentLabels=V;for(const auto& I:Layout.Items)if(I.Kind!=EFactoryItemKind::Zone)if(auto* A=Equipment.FindRef(I.Id).Get())A->SetLabelVisible(V);SaveViewSettings();}
void AFactoryRuntime::SetRouteVisible(bool V){bRouteVisible=V;SaveViewSettings();}
void AFactoryRuntime::SaveViewSettings()
{
    if(FParse::Param(FCommandLine::Get(),TEXT("FactoryRuntimeAutoTest"))||bCapturing)return;
    FFileHelper::SaveStringToFile(FString::Printf(TEXT("{\"zone_names\":%s,\"equipment_names\":%s,\"flow\":%s}"),
        bZoneLabels?TEXT("true"):TEXT("false"),bEquipmentLabels?TEXT("true"):TEXT("false"),bRouteVisible?TEXT("true"):TEXT("false")),*ViewSettingsPath());
}
void AFactoryRuntime::LoadViewSettings()
{
    if(FParse::Param(FCommandLine::Get(),TEXT("FactoryRuntimeAutoTest"))||FParse::Param(FCommandLine::Get(),TEXT("FactoryRuntimeCapture")))return;
    FString Json;TSharedPtr<FJsonObject> O;
    if(FFileHelper::LoadFileToString(Json,*ViewSettingsPath())&&FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json),O)&&O.IsValid())
    {O->TryGetBoolField(TEXT("zone_names"),bZoneLabels);O->TryGetBoolField(TEXT("equipment_names"),bEquipmentLabels);O->TryGetBoolField(TEXT("flow"),bRouteVisible);}
}
void AFactoryRuntime::RouteAppendSelected()
{
    const auto* I=Selected();if(!CanEdit()||!I||(I->Kind!=EFactoryItemKind::Zone&&I->Kind!=EFactoryItemKind::VehicleStation)||Layout.ProcessOrder.Contains(I->Id))return;
    BeginEdit();Layout.ProcessOrder.Add(I->Id);EndEdit();bNeedsPanel=true;
}
void AFactoryRuntime::RouteRemoveSelected(){if(!CanEdit())return;BeginEdit();Layout.ProcessOrder.Remove(SelectedId);EndEdit();bNeedsPanel=true;}
void AFactoryRuntime::RouteMoveSelected(int32 Delta)
{
    if(!CanEdit())return;const int32 I=Layout.ProcessOrder.IndexOfByKey(SelectedId),J=I+Delta;
    if(I==INDEX_NONE||!Layout.ProcessOrder.IsValidIndex(J))return;BeginEdit();Layout.ProcessOrder.Swap(I,J);EndEdit();bNeedsPanel=true;
}
void AFactoryRuntime::SetRouteBend(double Cm){if(!CanEdit()||!FMath::IsFinite(Cm)||FMath::Abs(Cm)>2000)return;BeginEdit();Layout.RouteBendCm=Cm;EndEdit();}
FString AFactoryRuntime::RouteText() const
{
    FString S;
    for(int32 I=0;I<Layout.ProcessOrder.Num();++I)if(const auto* Item=Layout.Find(Layout.ProcessOrder[I]))
        S+=FString::Printf(TEXT("%d. %s%s"),I+1,*Item->NameKo,I+1<Layout.ProcessOrder.Num()?TEXT("\n"):TEXT(""));
    return S.IsEmpty()?TEXT("공정 흐름이 없습니다. 구역을 선택해 순서에 추가하세요."):S;
}
FFactoryLayoutItem AFactoryRuntime::ResizeBox() const
{
    FFactoryLayoutItem B;if(bHallSelected){B.Position=Layout.HallOrigin+FVector2D(Layout.LengthCm*.5,0);B.Size={Layout.LengthCm,Layout.WidthCm};}
    else if(Selected())B=*Selected();return B;
}
int32 AFactoryRuntime::HitResizeHandle(const FVector2D& Pixel) const
{
    if(!bHallSelected&&(!Selected()||Selected()->Kind!=EFactoryItemKind::Zone))return -1;
    const auto B=ResizeBox();auto* PC=UGameplayStatics::GetPlayerController(this,0);if(!PC)return -1;
    for(int32 I=0;I<8;++I){FVector2D Screen;const FVector P=B.Transform(8).TransformPosition(FVector(Handles[I]*B.Size*.5,0));
        if(PC->ProjectWorldLocationToScreen(P,Screen)&&FVector2D::Distance(Screen,Pixel)<16)return I;}
    return -1;
}
void AFactoryRuntime::PreviewResize(const FVector& Point)
{
    if(ResizeHandle<0)return;const FVector2D H=Handles[ResizeHandle];
    FVector P=ResizeStartBox.Transform().InverseTransformPosition(Point);
    const FVector StartP=ResizeStartBox.Transform().InverseTransformPosition(DragStart);
    const FVector2D Original=H*ResizeStartBox.Size*.5;
    FVector2D Dragged=Original+FVector2D(P.X-StartP.X,P.Y-StartP.Y);
    if(bSnap){Dragged.X=FMath::GridSnap(Dragged.X,SnapCm);Dragged.Y=FMath::GridSnap(Dragged.Y,SnapCm);}
    FVector2D Lo=-ResizeStartBox.Size*.5,Hi=ResizeStartBox.Size*.5;
    const FVector2D Min=bHallSelected?FVector2D(1200,1200):FVector2D(20,20);
    const FVector2D Max=bHallSelected?FVector2D(20000,12000):FVector2D(20000,20000);
    for(int32 Axis=0;Axis<2;++Axis){if(H[Axis]<0)Lo[Axis]=FMath::Clamp(Dragged[Axis],Hi[Axis]-Max[Axis],Hi[Axis]-Min[Axis]);
        if(H[Axis]>0)Hi[Axis]=FMath::Clamp(Dragged[Axis],Lo[Axis]+Min[Axis],Lo[Axis]+Max[Axis]);}
    const FVector2D Size=Hi-Lo;const FVector Centre=ResizeStartBox.Transform().TransformPosition(FVector((Hi+Lo)*.5,0));
    if(bHallSelected)
    {
        Layout.LengthCm=Size.X;Layout.WidthCm=Size.Y;Layout.HallOrigin={Centre.X-Size.X*.5,Centre.Y};
        // Transform the shell only during preview. Equipment world coordinates stay fixed.
        if(HallRoot){const FVector Scale(Size.X/ResizeStartBox.Size.X,Size.Y/ResizeStartBox.Size.Y,1);
            HallRoot->SetRelativeScale3D(Scale);HallRoot->SetRelativeLocation(Centre-FVector(ResizeStartBox.Position,0)*Scale);}
    }
    else if(auto* Item=Selected())
    {
        Item->Position={Centre.X,Centre.Y};Item->Size=Size;
        if(auto* A=Equipment.FindRef(Item->Id).Get()){A->UpdatePlacement(*Item);A->VisualRoot->SetRelativeScale3D(FVector(Size.X/ResizeStartBox.Size.X,Size.Y/ResizeStartBox.Size.Y,1));A->SetLabelVisible(false);}
    }
    RefreshHighlights();RefreshDirty();
}
void AFactoryRuntime::ViewportPointer(FVector2D Pixel,FKey Button,bool Pressed)
{
    PointerPixel=LastPointerPixel=Pixel;bHasViewportPointer=true;
    if(Button==EKeys::MiddleMouseButton){bPanning=Pressed;return;}
    if(Button==EKeys::RightMouseButton){bOrbiting=Pressed;return;}
    if(Button!=EKeys::LeftMouseButton||!CanEdit())return;
    if(!Pressed){OnPointerUp();return;}
    FVector P;if(!PointerOnFloor(P))return;DragStart=P;
    if(!PendingTypeId.IsEmpty()&&!bCreateZone){const FString Id=PendingTypeId;PendingTypeId.Empty();AddRegisteredItem(Id,{P.X,P.Y});return;}
    ResizeHandle=PendingTypeId.IsEmpty()?HitResizeHandle(Pixel):-1;
    if(ResizeHandle>=0){ResizeStartBox=ResizeBox();BeginEdit();bDragging=true;return;}
    if(bCreateZone){BeginEdit();bDragging=true;return;}
    const FFactoryLayoutItem* Hit=nullptr;
    for(const auto& Item:Layout.Items){const FVector Local=Item.Transform().InverseTransformPosition(P);
        if(FMath::Abs(Local.X)<=Item.Size.X*.5&&FMath::Abs(Local.Y)<=Item.Size.Y*.5)
            if(!Hit||Item.Kind!=EFactoryItemKind::Zone)Hit=&Item;}
    SelectItem(Hit?Hit->Id:FString());
    if(Hit){BeginEdit();bDragging=true;DragOffset=Hit->Position-FVector2D(P.X,P.Y);}
}
void AFactoryRuntime::ViewportMove(FVector2D Pixel)
{
    const FVector2D Delta=Pixel-LastPointerPixel;PointerPixel=LastPointerPixel=Pixel;bHasViewportPointer=true;
    if(bPanning)PanCamera(Delta);else if(bOrbiting)OrbitCamera(Delta);
    if(bDragging&&CanEdit())
    {
        FVector P;if(!PointerOnFloor(P))return;
        if(ResizeHandle>=0)PreviewResize(P);
        else if(!bCreateZone&&Selected())SetSelectedPosition((P.X+DragOffset.X)/100.,(P.Y+DragOffset.Y)/100.);
    }
}
void AFactoryRuntime::OnPointerUp()
{
    if(!bDragging)return;const bool WasResize=ResizeHandle>=0;bDragging=false;ResizeHandle=-1;
    if(bCreateZone&&CanEdit())
    {
        FVector P;if(!PointerOnFloor(P)){CancelEdit();return;}
        if(bSnap){P.X=FMath::GridSnap(P.X,SnapCm);P.Y=FMath::GridSnap(P.Y,SnapCm);DragStart.X=FMath::GridSnap(DragStart.X,SnapCm);DragStart.Y=FMath::GridSnap(DragStart.Y,SnapCm);}
        const FVector2D Size(FMath::Abs(P.X-DragStart.X),FMath::Abs(P.Y-DragStart.Y));
        if(Size.X<20||Size.Y<20||Size.X>20000||Size.Y>20000||Layout.Items.Num()>=128){CancelEdit();Status=TEXT("구역은 각 변 0.2~200m로 그려주세요. 최대 128개입니다.");return;}
        EnsureRegistry();const auto* D=Registry->Find(PendingTypeId);if(!D){CancelEdit();return;}
        FFactoryLayoutItem I;I.Id=TEXT("zone_")+FGuid::NewGuid().ToString(EGuidFormats::Digits);I.TypeId=PendingTypeId;
        I.NameKo=D->NameKo;I.NameEn=D->NameEn;I.Position=FVector2D(P.X+DragStart.X,P.Y+DragStart.Y)*.5;I.Size=Size;
        Layout.Items.Add(I);SelectedId=I.Id;bCreateZone=false;PendingTypeId.Empty();Rebuild();
    }
    else if(WasResize)Rebuild();
    EndEdit();bNeedsPanel=true;
}
void AFactoryRuntime::PanCamera(FVector2D D)
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);int32 W=1600,H=900;if(PC)PC->GetViewportSize(W,H);
    const double Scale=CameraWidth/FMath::Max(W,1);
    const FVector Forward=FRotator(0,CameraYaw,0).Vector(),Right=FRotator(0,CameraYaw,0).RotateVector(FVector::RightVector);
    CameraCentre+=FVector2D(Forward.X,Forward.Y)*D.Y*Scale-FVector2D(Right.X,Right.Y)*D.X*Scale;UpdateCamera();
}
void AFactoryRuntime::OrbitCamera(FVector2D D)
{if(bTopView)return;CameraYaw=FRotator::NormalizeAxis(CameraYaw+D.X*.25);CameraPitch=FMath::Clamp(CameraPitch+D.Y*.2,-85.,-10.);UpdateCamera();}
void AFactoryRuntime::FocusSelected()
{if(const auto* I=Selected()){CameraCentre=I->Position;CameraWidth=FMath::Clamp(I->Size.GetMax()*2.5,700.,10000.);UpdateCamera();}else if(bHallSelected)FitView();else FocusWorkcell();}
void AFactoryRuntime::ViewportKey(FKey Key,bool Ctrl,bool Shift)
{
    if(Key==EKeys::Escape){CancelEdit();return;}
    if(Ctrl&&Key==EKeys::Z){if(Shift)RedoEdit();else UndoEdit();return;}
    if(Ctrl&&Key==EKeys::Y){RedoEdit();return;}
    if(Key==EKeys::F)FocusSelected();
    if(Key==EKeys::Delete)DeleteSelected();
}
void AFactoryRuntime::EditorTick(){DrawEditorGuides();}
void AFactoryRuntime::DrawEditorGuides()
{
    if(bRouteVisible)
    {
        for(int32 I=1;I<Layout.ProcessOrder.Num();++I)
        {
            const auto* From=Layout.Find(Layout.ProcessOrder[I-1]);const auto* To=Layout.Find(Layout.ProcessOrder[I]);if(!From||!To)continue;
            const FVector A=From->Transform(12).GetLocation();
            const FVector D=To->Transform(12).GetLocation();
            const double Dist=FMath::Clamp(FVector::Distance(A,D)*.4,50.,600.);
            const FVector B=A+From->Transform().TransformVector(FVector(Dist,Layout.RouteBendCm,0));
            const FVector C=D+To->Transform().TransformVector(FVector(-Dist,Layout.RouteBendCm,0));
            FVector Previous=A;for(int32 J=1;J<=24;++J){const FVector P=Bezier(A,B,C,D,J/24.);DrawDebugLine(GetWorld(),Previous,P,FColor(70,215,245),false,0,0,4);Previous=P;}
            DrawDebugDirectionalArrow(GetWorld(),Bezier(A,B,C,D,.7),Bezier(A,B,C,D,.78),35,FColor(70,215,245),false,0,0,5);
        }
    }
    if(!CanEdit())return;
    if((bHallSelected|| (Selected()&&Selected()->Kind==EFactoryItemKind::Zone))&&PendingTypeId.IsEmpty())
    {
        const auto B=ResizeBox();const FTransform T=B.Transform(10);
        DrawDebugBox(GetWorld(),T.GetLocation(),FVector(B.Size*.5,2),T.GetRotation(),FColor::Cyan,false,0,0,3);
        const double Radius=FMath::Clamp(CameraWidth*.003,6.,45.);
        for(const auto& H:Handles)DrawDebugBox(GetWorld(),T.TransformPosition(FVector(H*B.Size*.5,0)),FVector(Radius,Radius,3),T.GetRotation(),FColor::Yellow,false,0,0,3);
    }
    if(bHasViewportPointer&&!PendingTypeId.IsEmpty())
    {
        FVector P;if(!PointerOnFloor(P))return;
        if(bCreateZone&&bDragging)DrawDebugBox(GetWorld(),(DragStart+P)*.5+FVector(0,0,8),FVector(FMath::Abs(P.X-DragStart.X)*.5,FMath::Abs(P.Y-DragStart.Y)*.5,2),FColor::Green,false,0,0,3);
        else if(!bCreateZone){const auto* D=Registry->Find(PendingTypeId);if(D){if(bSnap){P.X=FMath::GridSnap(P.X,SnapCm);P.Y=FMath::GridSnap(P.Y,SnapCm);}DrawDebugBox(GetWorld(),P+FVector(0,0,10),FVector(D->FootprintCm*.5,10),FColor::Green,false,0,0,3);}}
    }
}

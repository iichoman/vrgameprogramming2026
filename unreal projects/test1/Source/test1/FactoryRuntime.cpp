#include "FactoryRuntime.h"
#include "FactoryEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "FactoryViewport.h"
#include "FactoryProcessRunner.h"
#include "FactoryMotionController.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InputComponent.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Engine/GameViewportClient.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Blueprint/WidgetLayoutLibrary.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Widgets/SWidget.h"
#include "DrawDebugHelpers.h"

namespace
{
bool Active(EFactoryRunState S) { return S>=EFactoryRunState::Approach && S<=EFactoryRunState::Verify; }
bool Finite(double V) { return FMath::IsFinite(V); }
constexpr double FixedStep=1.0/120.0;
const FVector VehicleOriginOffset(240,0,25);
FLinearColor ShellColor(.34,.40,.47);
void Color(UStaticMeshComponent* C,FLinearColor Value)
{
    auto* Base=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/DigitalTwin/Factory/Runtime/M_FactoryTint"));
    if(!Base)return;
    auto* M=UMaterialInstanceDynamic::Create(Base,C);M->SetVectorParameterValue(TEXT("Tint"),Value);C->SetMaterial(0,M);
}
}

AFactoryRuntime::AFactoryRuntime()
{
    PrimaryActorTick.bCanEverTick=true;
    WheelProcess=CreateDefaultSubobject<UFactoryWheelProcessRunner>(TEXT("WheelProcess"));
    RootComponent=CreateDefaultSubobject<USceneComponent>(TEXT("FactoryOrigin"));
    RootComponent->SetMobility(EComponentMobility::Movable);
}

UStaticMeshComponent* AFactoryRuntime::MakeMesh(USceneComponent* Parent,UStaticMesh* Mesh,FTransform Transform,bool Visible)
{
    auto* C=NewObject<UStaticMeshComponent>(this,NAME_None,RF_Transactional);
    AddInstanceComponent(C);C->SetupAttachment(Parent);C->SetMobility(EComponentMobility::Movable);
    C->SetStaticMesh(Mesh);C->SetRelativeTransform(Transform);C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->SetCastShadow(false);C->SetVisibleInRayTracing(false);C->SetAffectDistanceFieldLighting(false);
    C->SetVisibility(Visible);C->SetHiddenInGame(!Visible);C->RegisterComponent();GeneratedComponents.Add(C);
    return C;
}
void AFactoryRuntime::ClearGenerated()
{
    TArray<AFactoryEquipment*> Old;
    for(TActorIterator<AFactoryEquipment> It(GetWorld());It;++It)if(It->GetOwner()==this)Old.Add(*It);
    for(auto* A:Old)A->Destroy();
    Equipment.Empty();
    for(int32 I=GeneratedComponents.Num()-1;I>=0;--I)
        if(IsValid(GeneratedComponents[I])){RemoveInstanceComponent(GeneratedComponents[I]);GeneratedComponents[I]->DestroyComponent();}
    GeneratedComponents.Empty();VehicleRoot=WheelRoot=nullptr;
}
void AFactoryRuntime::BuildShell()
{
    HallRoot=NewObject<USceneComponent>(this);AddInstanceComponent(HallRoot);HallRoot->SetupAttachment(RootComponent);HallRoot->SetMobility(EComponentMobility::Movable);HallRoot->RegisterComponent();GeneratedComponents.Add(HallRoot);
    auto* Cube=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube"));
    auto Box=[&](FVector P,FVector S,FLinearColor Tint,bool Solid=false){
        auto* C=MakeMesh(HallRoot,Cube,FTransform(FQuat::Identity,P+FVector(Layout.HallOrigin,0),S/100.));Color(C,Tint);
        if(Solid){C->SetCollisionProfileName(TEXT("BlockAll"));C->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);}return C;};
    Box(FVector(Layout.LengthCm/2,0,-15),FVector(Layout.LengthCm,Layout.WidthCm,30),FLinearColor(.18,.21,.25),true);
    for(double Sign:{-1.,1.})
    {
        Box(FVector(Layout.LengthCm/2,Sign*Layout.WidthCm/2,60),FVector(Layout.LengthCm,24,120),ShellColor,true);
        for(double X=0;X<=Layout.LengthCm;X+=1200)
            Box(FVector(X,Sign*(Layout.WidthCm/2-15),350),FVector(30,30,700),ShellColor,true);
        double EndLength=(Layout.WidthCm-800)*.5;
        for(double X:{0.,Layout.LengthCm})Box(FVector(X,Sign*(400+EndLength/2),60),FVector(24,EndLength,120),ShellColor,true);
    }
}
void AFactoryRuntime::BuildVehicle()
{
    auto AddRoot=[&](USceneComponent* Parent){auto* C=NewObject<USceneComponent>(this,NAME_None,RF_Transactional);
        AddInstanceComponent(C);C->SetupAttachment(Parent);C->SetMobility(EComponentMobility::Movable);C->RegisterComponent();GeneratedComponents.Add(C);return C;};
    if(auto* Station=Equipment.FindRef(Layout.StationId).Get())
    {
        VehicleRoot=AddRoot(Station->GetRootComponent());VehicleRoot->SetRelativeLocation(VehicleOriginOffset);
        for(const auto& Part:BodyMeshes)if(Part.Mesh)MakeMesh(VehicleRoot,Part.Mesh,FTransform::Identity,Part.bPreviewVisible);
    }
    WheelRoot=AddRoot(RootComponent);
    for(const auto& Part:WheelMeshes)if(Part.Mesh)
        MakeMesh(WheelRoot,Part.Mesh,FTransform(FQuat::Identity,-WheelPivotCm),Part.bPreviewVisible);
}
void AFactoryRuntime::Rebuild(bool bRefreshPanel)
{
    EnsureRegistry();ClearGenerated();BuildShell();
    for(const auto& Item:Layout.Items)
    {
        FActorSpawnParameters P;P.Owner=this;
        const auto* D=Registry->Find(Item.EffectiveTypeId());
        if(!D)continue;
        auto* A=GetWorld()->SpawnActor<AFactoryEquipment>(D->ActorClass,Item.Transform(),P);
        if(!A)continue;
        UStaticMesh* Override=D->VisualMesh;
        for(const auto& V:VisualOverrides)if(V.EquipmentKind.ToString()==FFactoryLayout::KindName(Item.Kind))Override=V.Mesh;
        A->Configure(Item,Override);A->SetLabelVisible(Item.Kind==EFactoryItemKind::Zone?bZoneLabels:bEquipmentLabels);Equipment.Add(Item.Id,A);
    }
    BuildVehicle();ResetCycle();RefreshHighlights();UpdateCamera();bNeedsPanel|=bRefreshPanel;
}
void AFactoryRuntime::BuildPreview()
{
    Registry=nullptr;
    FString Error;
    if(InitialLayoutJson.IsEmpty())Layout=FFactoryLayout::Default();
    else if(!FFactoryLayout::FromJson(InitialLayoutJson,Layout,Error)){Status=Error;return;}
    EnsureRegistry();if(!Registry->Validate(Layout,Error)){Status=Error;return;}
    InitialLayoutJson=Layout.ToJson();Rebuild();
}
bool AFactoryRuntime::VerifySetup() const
{
    FFactoryLayout Check;FString Error;
    return FFactoryLayout::FromJson(InitialLayoutJson,Check,Error) && BodyMeshes.Num()>0 && WheelMeshes.Num()==2
        && WheelMeshes[0].Mesh && WheelMeshes[1].Mesh && !WheelPivotCm.ContainsNaN();
}

void AFactoryRuntime::BeginPlay()
{
    Super::BeginPlay();LoadViewSettings();BuildPreview();
    const bool bTest=FParse::Param(FCommandLine::Get(),TEXT("FactoryRuntimeAutoTest"));
    bCapturing=FParse::Param(FCommandLine::Get(),TEXT("FactoryRuntimeCapture"));
    if(bTest){RunChecks();return;}
    Status=TEXT("편집 모드: 설비를 드래그하거나 속성 값을 변경하세요. 배치는 A/B 슬롯에 따로 저장됩니다.");
    if(!bCapturing && (IFileManager::Get().FileExists(*SlotPath(TEXT("A")))
        || IFileManager::Get().FileExists(*(SlotPath(TEXT("A"))+TEXT(".bak")))))LoadLayout(TEXT("A"));
    History.Clear();SavedSnapshot=Layout.ToJson();RefreshDirty();
    auto* PC=UGameplayStatics::GetPlayerController(this,0);
    if(PC)
    {
        bInputConfigured=true;PC->bShowMouseCursor=true;PC->SetIgnoreMoveInput(true);PC->SetIgnoreLookInput(true);
        FInputModeGameAndUI Mode;Mode.SetHideCursorDuringCapture(false);Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);PC->SetInputMode(Mode);
        ViewCamera=GetWorld()->SpawnActor<ACameraActor>();PC->SetViewTarget(ViewCamera);
        FocusWorkcell();CreatePanel();bNeedsPanel=false;
    }
}
void AFactoryRuntime::EndPlay(const EEndPlayReason::Type Reason)
{
    if(GEngine && GEngine->GameViewport && Panel.IsValid())GEngine->GameViewport->RemoveViewportWidgetContent(Panel.ToSharedRef());
    Panel.Reset();PanelScroll.Reset();
    if(bInputConfigured)if(auto* PC=UGameplayStatics::GetPlayerController(this,0))
    {PC->SetIgnoreMoveInput(false);PC->SetIgnoreLookInput(false);PC->bShowMouseCursor=false;PC->SetInputMode(FInputModeGameOnly());}
    Super::EndPlay(Reason);
}
void AFactoryRuntime::Tick(float Dt)
{
    Super::Tick(Dt);
    EditorTick();
    /* Legacy input replaced by the Slate viewport surface.
    if(bDragging && CanEdit())
    {
        FVector Point;if(PointerOnFloor(Point) && !bCreateZone && Selected())
            SetSelectedPosition((Point.X+DragOffset.X)/100.,(Point.Y+DragOffset.Y)/100.);
        if(bCreateZone && PointerOnFloor(Point))
            DrawDebugBox(GetWorld(),(DragStart+Point)*.5+FVector(0,0,5),FVector(FMath::Abs(Point.X-DragStart.X)*.5,FMath::Abs(Point.Y-DragStart.Y)*.5,2),FColor::Green,false,0,0,3);
        if(auto* PC=UGameplayStatics::GetPlayerController(this,0))if(!PC->IsInputKeyDown(EKeys::LeftMouseButton))OnPointerUp();
    }
    */
    if(Active(RunState))
    {
        SimRemainder+=Dt;
        int32 Budget=120;
        while(SimRemainder>=FixedStep && Budget-->0 && Active(RunState))
        {SimRemainder-=FixedStep;StepSimulation(FixedStep);}
    }
    if(bNeedsPanel && bInputConfigured && !bDragging){CreatePanel();bNeedsPanel=false;}
    if(bCapturing)CaptureRuntime(Dt);
}
void AFactoryRuntime::SelectItem(const FString& Id){if(!CanEdit())return;bHallSelected=false;SelectedId=Layout.Find(Id)?Id:FString();RefreshHighlights();bNeedsPanel=true;}
void AFactoryRuntime::RefreshHighlights()
{
    for(const auto& Item:Layout.Items)if(auto* A=Equipment.FindRef(Item.Id).Get())A->SetSelected(Item.Id==SelectedId,!Layout.Contains(Item));
}
void AFactoryRuntime::ApplyEditedItem()
{
    if(auto* Item=Selected())if(auto* A=Equipment.FindRef(Item->Id).Get())A->UpdatePlacement(*Item);
    RefreshPreviewWheel();RefreshHighlights();UpdateCamera();
}
void AFactoryRuntime::SetHallSize(double L,double W)
{
    if(!CanEdit() || !Finite(L) || !Finite(W))return;
    if(L<12||L>200||W<12||W>120){Status=TEXT("공장 길이는 12~200m, 폭은 12~120m로 설정하세요.");return;}
    BeginEdit();Layout.LengthCm=L*100;Layout.WidthCm=W*100;Rebuild(false);FinishDiscreteEdit();FitView();
    Status=TEXT("공장 크기를 변경했습니다. 설비 크기와 좌표는 유지되며 바깥 항목은 빨간색으로 표시됩니다.");
}
void AFactoryRuntime::SetSelectedPosition(double X,double Y)
{
    auto* Item=Selected();if(!CanEdit()||!Item||!Finite(X)||!Finite(Y)||FMath::Abs(X)>1000||FMath::Abs(Y)>1000)return;
    BeginEdit();Item->Position=FVector2D(X*100,Y*100);
    if(bSnap){Item->Position.X=FMath::GridSnap(Item->Position.X,SnapCm);Item->Position.Y=FMath::GridSnap(Item->Position.Y,SnapCm);}
    ApplyEditedItem();FinishDiscreteEdit();
}
void AFactoryRuntime::SetSelectedYaw(double Degrees)
{
    if(!CanEdit()||!Selected()||!Finite(Degrees))return;
    BeginEdit();Selected()->Yaw=FRotator::NormalizeAxis(Degrees);ApplyEditedItem();FinishDiscreteEdit();
}
void AFactoryRuntime::SetSelectedSize(double X,double Y)
{
    if(!CanEdit()||!Selected()||Selected()->Kind!=EFactoryItemKind::Zone||!Finite(X)||!Finite(Y)||X<.2||Y<.2||X>200||Y>200)return;
    BeginEdit();Selected()->Size=FVector2D(X*100,Y*100);Rebuild(false);FinishDiscreteEdit();
}
void AFactoryRuntime::SetSelectedRole(const FString& NewRole)
{
    if(!CanEdit()||!Selected()||Selected()->Kind!=EFactoryItemKind::Zone)return;
    BeginEdit();const FString Old=Selected()->Role;Selected()->Role=NewRole;FString Error;
    if(!Layout.Validate(Error)){Selected()->Role=Old;Status=Error;FinishDiscreteEdit();return;}
    Rebuild();FinishDiscreteEdit();
}
void AFactoryRuntime::SetSelectedName(const FString& Ko,const FString& En)
{
    if(!CanEdit()||!Selected())return;
    BeginEdit();const FString OldKo=Selected()->NameKo,OldEn=Selected()->NameEn;
    Selected()->NameKo=Ko.TrimStartAndEnd();Selected()->NameEn=En.TrimStartAndEnd();FString Error;
    if(!Layout.Validate(Error)){Selected()->NameKo=OldKo;Selected()->NameEn=OldEn;Status=Error;FinishDiscreteEdit();return;}
    Rebuild();FinishDiscreteEdit();
}
void AFactoryRuntime::AddItem(EFactoryItemKind Kind)
{ AddRegisteredItem(FFactoryLayout::KindName(Kind),CameraCentre); }
void AFactoryRuntime::DeleteSelected()
{
    if(!CanEdit()||SelectedId.IsEmpty())return;
    BeginEdit();Layout.ProcessOrder.Remove(SelectedId);
    if(Layout.StationId==SelectedId)Layout.StationId.Empty();if(Layout.SupplyId==SelectedId)Layout.SupplyId.Empty();if(Layout.HandlerId==SelectedId)Layout.HandlerId.Empty();
    Layout.Items.RemoveAll([&](const auto& I){return I.Id==SelectedId;});SelectedId.Empty();Rebuild();FinishDiscreteEdit();
}
void AFactoryRuntime::BindSelected()
{
    if(!CanEdit()||!Selected())return;
    BeginEdit();switch(Selected()->Kind)
    {
    case EFactoryItemKind::VehicleStation:Layout.StationId=SelectedId;break;
    case EFactoryItemKind::WheelSupply:Layout.SupplyId=SelectedId;break;
    case EFactoryItemKind::ArticulatedRobot:
    case EFactoryItemKind::CartesianHandler:Layout.HandlerId=SelectedId;break;
    default:Status=TEXT("차량 정지대·휠 공급대·로봇을 선택해 연결하세요.");FinishDiscreteEdit();return;
    }
    Rebuild();FinishDiscreteEdit();Status=TEXT("선택한 설비를 휠 작업에 연결했습니다.");
}
FString AFactoryRuntime::SlotPath(const FString& Slot) const
{
    if(Slot!=TEXT("A") && Slot!=TEXT("B"))return FString();
    FString Dir=FPaths::ProjectSavedDir()/TEXT("FactoryLayouts");
    if(FParse::Param(FCommandLine::Get(),TEXT("FactoryRuntimeAutoTest")))Dir/=TEXT("AutomatedTests");
    return Dir/(TEXT("layout_")+Slot+TEXT(".json"));
}
bool AFactoryRuntime::SaveLayout(const FString& Slot)
{
    if(!CanEdit())return false;
    FString Error;if(!Layout.Validate(Error)){Status=Error;return false;}
    const FString Path=SlotPath(Slot);if(Path.IsEmpty())return false;
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path),true);
    const FString Temp=Path+TEXT(".tmp");
    if(!FFileHelper::SaveStringToFile(Layout.ToJson(),*Temp,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
    {Status=TEXT("임시 배치를 저장하지 못했습니다. 기존 저장은 유지됩니다.");return false;}
    const FString Backup=Path+TEXT(".bak");
    const bool HadPrevious=IFileManager::Get().FileExists(*Path);
    if(HadPrevious && IFileManager::Get().Copy(*Backup,*Path,true,true)!=COPY_OK)
    {Status=TEXT("기존 배치 백업에 실패하여 저장을 중단했습니다.");return false;}
    if(!IFileManager::Get().Move(*Path,*Temp,true,true))
    {
        if(HadPrevious)IFileManager::Get().Copy(*Path,*Backup,true,true);
        Status=TEXT("배치 교체에 실패했습니다. 이전 배치는 .bak 파일에 보존됩니다.");return false;
    }
    SavedSnapshot=Layout.ToJson();RefreshDirty();Status=TEXT("배치 ")+Slot+TEXT(" 저장 완료 · Saved/FactoryLayouts/layout_")+Slot+TEXT(".json");return true;
}
bool AFactoryRuntime::LoadLayout(const FString& Slot)
{
    if(!CanEdit())return false;
    FString Json,Error;FFactoryLayout Candidate;const FString Path=SlotPath(Slot);
    if(Path.IsEmpty()){Status=TEXT("저장 슬롯이 올바르지 않습니다.");return false;}
    const bool FromBackup=!IFileManager::Get().FileExists(*Path);
    if(!FFileHelper::LoadFileToString(Json,*(FromBackup?Path+TEXT(".bak"):Path))){Status=TEXT("저장된 배치 파일을 읽을 수 없습니다.");return false;}
    if(!FFactoryLayout::FromJson(Json,Candidate,Error)){Status=TEXT("불러오기 거부: ")+Error;return false;}
    EnsureRegistry();if(!Registry->Validate(Candidate,Error)){Status=Error;return false;}
    BeginEdit();Layout=MoveTemp(Candidate);SelectedId.Empty();Rebuild();EndEdit();SavedSnapshot=FromBackup?FString():Layout.ToJson();RefreshDirty();FitView();Status=TEXT("배치 ")+Slot+(FromBackup?TEXT(" 백업 복구 완료 · 다시 저장하세요."):TEXT(" 불러오기 완료"));return true;
}
void AFactoryRuntime::RestoreDefault()
{
    if(!CanEdit())return;BeginEdit();Layout=FFactoryLayout::Default();SelectedId.Empty();Rebuild();FinishDiscreteEdit();FitView();Status=TEXT("기본 배치로 복원했습니다. 저장 파일은 유지됩니다.");
}
bool AFactoryRuntime::PointerOverPanel() const { return false; } // Slate hit testing owns UI isolation.
bool AFactoryRuntime::PointerOnFloor(FVector& Point) const
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);FVector O,D;
    if(!PC||!PC->DeprojectScreenPositionToWorld(PointerPixel.X,PointerPixel.Y,O,D)||FMath::Abs(D.Z)<.00001)return false;
    const double T=-O.Z/D.Z;if(T<0)return false;Point=O+D*T;return !Point.ContainsNaN();
}
void AFactoryRuntime::UpdateCamera()
{
    if(!ViewCamera)return;auto* C=ViewCamera->GetCameraComponent();C->SetFieldOfView(60);
    const FVector Target(CameraCentre.X,CameraCentre.Y,50);
    C->ProjectionMode=bTopView?ECameraProjectionMode::Orthographic:ECameraProjectionMode::Perspective;
    C->OrthoWidth=CameraWidth;
    const FRotator ViewRotation(bTopView?-90:CameraPitch,CameraYaw,0);
    const FVector P=Target-ViewRotation.Vector()*(bTopView?10000:CameraWidth*.9);
    ViewCamera->SetActorLocation(P);ViewCamera->SetActorRotation(ViewRotation);
    for(const auto& Entry:Equipment)if(Entry.Value)Entry.Value->OrientLabel(ViewCamera->GetActorRotation().Yaw);
}
void AFactoryRuntime::FitView(){CameraWidth=FMath::Max(Layout.LengthCm*1.5,Layout.WidthCm*2.4);CameraCentre=Layout.HallOrigin+FVector2D(Layout.LengthCm*.5-CameraWidth*.1,0);CameraYaw=-90;bTopView=true;UpdateCamera();}
void AFactoryRuntime::FocusWorkcell(){auto* S=Layout.Find(Layout.StationId);CameraCentre=S?S->Position+FVector2D(100,-180):FVector2D(Layout.LengthCm*.5,0);CameraWidth=1800;CameraYaw=90;CameraPitch=-46;bTopView=false;UpdateCamera();}
void AFactoryRuntime::ToggleView(){bTopView=!bTopView;UpdateCamera();}
void AFactoryRuntime::ZoomIn(){CameraWidth=FMath::Max(250.,CameraWidth*.85);UpdateCamera();}
void AFactoryRuntime::ZoomOut(){CameraWidth=FMath::Min(30000.,CameraWidth/ .85);UpdateCamera();}
void AFactoryRuntime::OnWheelZoomIn(){if(!PointerOverPanel())ZoomIn();}
void AFactoryRuntime::OnWheelZoomOut(){if(!PointerOverPanel())ZoomOut();}

FVector AFactoryRuntime::HubWorld() const {auto* S=Layout.Find(Layout.StationId);return S?S->Transform().TransformPosition(VehicleOriginOffset+WheelPivotCm):FVector::ZeroVector;}
double AFactoryRuntime::VehicleYaw() const {auto* S=Layout.Find(Layout.StationId);return S?S->Yaw:0;}
FVector AFactoryRuntime::SupplyWorld() const {auto* A=Equipment.FindRef(Layout.SupplyId).Get();return A?A->Anchor():FVector::ZeroVector;}
double AFactoryRuntime::SupplyYaw() const {auto* S=Layout.Find(Layout.SupplyId);return S?S->Yaw:0;}
void AFactoryRuntime::RefreshPreviewWheel()
{
    if(WheelRoot && CanEdit()){WheelRoot->SetWorldLocation(SupplyWorld());WheelRoot->SetWorldRotation(FRotator(0,SupplyYaw(),0));}
    if(auto* H=Equipment.FindRef(Layout.HandlerId).Get())if(auto* C=H->Controller())H->UpdateTool(C->Pose().GetLocation(),C->Pose().Rotator().Yaw);
}
bool AFactoryRuntime::Obstructed(const FVector& A,const FVector& B,FString& Obstacle) const
{
    for(const auto& I:Layout.Items)
    {
        if(I.Kind==EFactoryItemKind::Zone||I.Id==Layout.HandlerId||I.Id==Layout.StationId||I.Id==Layout.SupplyId)continue;
        double Height=I.Kind==EFactoryItemKind::Conveyor?60:(I.Kind==EFactoryItemKind::WheelSupply?130:350);
        if(FMath::Min(A.Z,B.Z)-36>Height)continue;
        if(FFactoryLayout::SegmentHits(A,B,I,36)){Obstacle=I.DisplayName();return true;}
    }
    return false;
}
bool AFactoryRuntime::ValidateRun(FString& Error) const
{
    if(!Layout.Validate(Error))return false;
    if(!Equipment.FindRef(Layout.StationId).Get()||!Equipment.FindRef(Layout.SupplyId).Get()||!Equipment.FindRef(Layout.HandlerId).Get())
    {Error=TEXT("차량 정지대·휠 공급대·직교 이송장치를 각각 작업에 연결하세요.");return false;}
    for(const auto& I:Layout.Items)if(I.Kind!=EFactoryItemKind::Zone && !Layout.Contains(I))
    {Error=I.DisplayName()+TEXT(": 공장 외곽을 벗어났습니다.");return false;}
    for(int32 A=0;A<Layout.Items.Num();++A)for(int32 B=A+1;B<Layout.Items.Num();++B)
        if(Layout.Items[A].Kind!=EFactoryItemKind::Zone&&Layout.Items[B].Kind!=EFactoryItemKind::Zone
            &&FFactoryLayout::Overlaps(Layout.Items[A],Layout.Items[B]))
        {Error=TEXT("설비 바닥 영역 겹침: ")+Layout.Items[A].DisplayName()+TEXT(" / ")+Layout.Items[B].DisplayName();return false;}
    return WheelProcess&&WheelProcess->Validate(Error);
}
bool AFactoryRuntime::StartCycle(){return WheelProcess&&WheelProcess->Start();}
void AFactoryRuntime::StopCycle(){if(WheelProcess)WheelProcess->Stop();}
void AFactoryRuntime::ResetCycle(){if(WheelProcess)WheelProcess->Reset();}
void AFactoryRuntime::StepSimulation(double Dt){if(WheelProcess)WheelProcess->Step(Dt);}
FString AFactoryRuntime::StateText() const
{
    switch(RunState){case EFactoryRunState::Editing:return TEXT("편집");case EFactoryRunState::Approach:return TEXT("공급점 접근");case EFactoryRunState::Pickup:return TEXT("휠 인수");case EFactoryRunState::Lift:return TEXT("휠 상승");case EFactoryRunState::Transfer:return TEXT("운반");case EFactoryRunState::Align:return TEXT("허브 접근");case EFactoryRunState::Verify:return TEXT("정렬·검사");case EFactoryRunState::Complete:return TEXT("정렬 완료");case EFactoryRunState::Stopped:return TEXT("정지");default:return TEXT("이상 중단");}
}
FString AFactoryRuntime::BindingText() const
{
    auto Name=[&](const FString& Id){auto* I=Layout.Find(Id);return I?I->NameKo:TEXT("미지정");};
    return TEXT("차량: ")+Name(Layout.StationId)+TEXT("\n공급: ")+Name(Layout.SupplyId)+TEXT("\n이송: ")+Name(Layout.HandlerId);
}
FString AFactoryRuntime::ValidationText() const
{
    const FString Key=Layout.ToJson();if(Key==ValidationKey&&!ValidationMessage.IsEmpty())return ValidationMessage;
    ValidationKey=Key;FString Error;
    if(!ValidateRun(Error))ValidationMessage=TEXT("실행 조건: ")+Error;
    else {int32 Outside=0;for(const auto& I:Layout.Items)if(!Layout.Contains(I))++Outside;
        ValidationMessage=Outside?FString::Printf(TEXT("작업 실행 가능 · 공장 밖 구역 %d개를 확인하세요."),Outside):TEXT("작업 실행 가능 · 구역은 용도 표시이며 인증된 안전구역이 아닙니다.");}
    return ValidationMessage;
}

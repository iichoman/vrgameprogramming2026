#include "FactoryEquipment.h"
#include "FactoryMotionController.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/WidgetComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/StaticMesh.h"
#include "Misc/Paths.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Styling/CoreStyle.h"

namespace
{
void Tint(UStaticMeshComponent* C, FLinearColor Color)
{
    if(auto* Existing=Cast<UMaterialInstanceDynamic>(C->GetMaterial(0)))
    { Existing->SetVectorParameterValue(TEXT("Tint"), Color); return; }
    UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/DigitalTwin/Factory/Runtime/M_FactoryTint"));
    if (!Base) Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
    UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, C);
    M->SetVectorParameterValue(TEXT("Tint"), Color);
    C->SetMaterial(0, M);
}
}

AFactoryEquipment::AFactoryEquipment()
{
    PrimaryActorTick.bCanEverTick = false;
    FunctionalRoot = CreateDefaultSubobject<USceneComponent>(TEXT("FunctionalRoot"));
    RootComponent = FunctionalRoot;
    FunctionalRoot->SetMobility(EComponentMobility::Movable);
    VisualRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ReplaceableVisualRoot"));
    VisualRoot->SetupAttachment(FunctionalRoot);
    VisualRoot->SetMobility(EComponentMobility::Movable);
}

UStaticMeshComponent* AFactoryEquipment::Box(FName Name, FVector Pos, FVector Size, FLinearColor Color, bool bSelectable)
{
    auto* C = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transactional);
    AddInstanceComponent(C);
    C->SetupAttachment(VisualRoot);
    C->SetMobility(EComponentMobility::Movable);
    C->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube")));
    C->SetRelativeLocation(Pos);
    C->SetRelativeScale3D(Size / 100.0);
    C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->SetCastShadow(false);
    C->SetVisibleInRayTracing(false);
    C->SetAffectDistanceFieldLighting(false);
    Tint(C, Color);
    C->ComponentTags.Add(Name);
    C->RegisterComponent();
    Parts.Add(C);
    return C;
}

void AFactoryEquipment::Configure(const FFactoryLayoutItem& Item, UStaticMesh* Replacement)
{
    if(MotionController){RemoveInstanceComponent(MotionController);MotionController->DestroyComponent();MotionController=nullptr;}
    if(Item.Kind==EFactoryItemKind::CartesianHandler||Item.Kind==EFactoryItemKind::ArticulatedRobot)
    {MotionController=CreateController();AddInstanceComponent(MotionController);MotionController->RegisterComponent();MotionController->Reset(0);}
    Definition = Item;
    ItemId = Item.Id;
    SetActorTransform(Item.Transform());
    for (UStaticMeshComponent* C : Parts) if (IsValid(C)) { RemoveInstanceComponent(C); C->DestroyComponent(); }
    Parts.Empty();
    if (IsValid(Label)) { RemoveInstanceComponent(Label); Label->DestroyComponent(); }
    Tool = BeamX = BeamY = Mast = nullptr;
    Selection = Box(TEXT("SelectionFootprint"), FVector(0,0,0.6), FVector(Item.Size.X+18,Item.Size.Y+18,1), FLinearColor(1,.7,.06));
    Selection->SetVisibility(false);
    FLinearColor Color(.10,.20,.28);
    if(Item.Role==TEXT("supply"))Color=FLinearColor(.37,.25,.10);
    if(Item.Role==TEXT("reserve"))Color=FLinearColor(.22,.22,.31);
    if(Item.Role==TEXT("rework"))Color=FLinearColor(.40,.19,.10);
    if(Item.Role==TEXT("buffer"))Color=FLinearColor(.15,.29,.25);
    if(Item.Role==TEXT("walkway"))Color=FLinearColor(.08,.30,.16);
    if(Item.Kind==EFactoryItemKind::Zone)
        Box(TEXT("ZoneArea"), FVector(0,0,1.2), FVector(Item.Size.X,Item.Size.Y,1), Color);
    else
    {
        Box(TEXT("EquipmentFootprint"), FVector(0,0,1.3), FVector(Item.Size.X,Item.Size.Y,1), FLinearColor(.12,.14,.17));
        const FLinearColor Steel(.28,.34,.40);
        switch(Item.Kind)
        {
        case EFactoryItemKind::VehicleStation:
            Box(TEXT("VehicleSupport"), FVector(0,0,12), FVector(Item.Size.X,Item.Size.Y,24), Steel);
            break;
        case EFactoryItemKind::WheelSupply:
            Box(TEXT("SupplyTop"), FVector(0,0,52), FVector(Item.Size.X,Item.Size.Y,10), FLinearColor(.42,.29,.10));
            for(double X : {-50.,50.}) for(double Y : {-50.,50.})
                Box(TEXT("SupplyLeg"), FVector(X,Y,24), FVector(10,10,48), Steel);
            break;
        case EFactoryItemKind::CartesianHandler:
            Box(TEXT("HandlerBase"), FVector(0,0,10), FVector(90,90,20), FLinearColor(.65,.36,.05));
            Box(TEXT("HandlerColumn"), FVector(0,0,160), FVector(22,22,300), Steel);
            BeamX=Box(TEXT("AxisX"), FVector(0,0,310), FVector(20,16,16), FLinearColor(.7,.4,.08));
            BeamY=Box(TEXT("AxisY"), FVector(0,0,295), FVector(16,20,16), FLinearColor(.12,.44,.61));
            Mast=Box(TEXT("AxisZ"), FVector(0,0,240), FVector(12,12,110), Steel);
            Tool=Box(TEXT("ToolProxy"), FVector(0,0,250), FVector(32,30,18), FLinearColor(.8,.55,.12));
            break;
        case EFactoryItemKind::Conveyor:
            Box(TEXT("ConveyorDeck"), FVector(0,0,40), FVector(Item.Size.X,Item.Size.Y,12), Steel);
            for(int32 I=0;I<12;++I)
                Box(TEXT("RollerProxy"), FVector(-Item.Size.X*.46+I*Item.Size.X*.083,0,49), FVector(18,Item.Size.Y-20,8), FLinearColor(.17,.20,.23));
            for(double X : {-1.,1.}) for(double Y : {-1.,1.})
                Box(TEXT("ConveyorLeg"), FVector(X*Item.Size.X*.42,Y*Item.Size.Y*.36,20), FVector(12,12,40), Steel);
            break;
        default: break;
        }
        if(Replacement)
        {
            // Asset must use centimetres and a floor-centred origin. Process anchors remain independent.
            for(int32 I=2;I<Parts.Num();++I) if(Parts[I]!=Tool && Parts[I]!=BeamX && Parts[I]!=BeamY && Parts[I]!=Mast)
                Parts[I]->SetVisibility(false);
            auto* C=Box(TEXT("ReplacementVisual"), FVector::ZeroVector, FVector(100), Steel);
            C->SetStaticMesh(Replacement);
            for(int32 Slot=0;Slot<Replacement->GetStaticMaterials().Num();++Slot) C->SetMaterial(Slot, Replacement->GetMaterial(Slot));
        }
    }
    Label=NewObject<UWidgetComponent>(this, NAME_None, RF_Transactional);
    AddInstanceComponent(Label);
    Label->SetupAttachment(FunctionalRoot);
    Label->SetMobility(EComponentMobility::Movable);
    Label->SetWidgetSpace(EWidgetSpace::World);
    Label->SetDrawSize(FVector2D(1100,400));
    Label->SetPivot(FVector2D(.5,.5));
    Label->SetTwoSided(true);
    Label->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Label->SetCastShadow(false);
    Label->SetRelativeRotation(FRotator(90,90,0));
    const bool bZone=Item.Kind==EFactoryItemKind::Zone;
    Label->SetRelativeLocation(FVector(0,bZone?Item.Size.Y*.24:Item.Size.Y*.5+38,bZone?5:6));
    Label->SetRelativeScale3D(FVector(bZone?FMath::Min(Item.Size.X/1200.,1.0):.18));
    Label->RegisterComponent();
    const FString FontPath=FPaths::ProjectContentDir()/TEXT("DigitalTwin/Fonts/NanumGothic.ttf");
    const FSlateFontInfo KoreanFont(FontPath,bZone?84.f:58.f), EnglishFont(FontPath,bZone?46.f:34.f);
    Label->SetSlateWidget(SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(.015f,.025f,.04f,.95f)).Padding(12)
        .HAlign(HAlign_Center).VAlign(VAlign_Center)
        [SNew(SVerticalBox)
            +SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Font(KoreanFont).ColorAndOpacity(FLinearColor::White)
                .Justification(ETextJustify::Center).Text(FText::FromString(Item.NameKo)).WrapTextAt(1060)]
            +SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Font(EnglishFont).ColorAndOpacity(FLinearColor(.7f,.85f,1.f))
                .Justification(ETextJustify::Center).Text(FText::FromString(TEXT("(")+Item.NameEn+TEXT(")"))).WrapTextAt(1060)]]);
    Label->SetRedrawTime(.2f);
#if WITH_EDITOR
    SetActorLabel(Item.DisplayName());
    SetFolderPath(Item.Kind==EFactoryItemKind::Zone?TEXT("FactoryRuntime/구역 (Zones)"):TEXT("FactoryRuntime/설비 (Equipment)"));
#endif
}

void AFactoryEquipment::OrientLabel(double CameraYaw)
{
    if(Label)Label->SetWorldRotation(FRotator(90,CameraYaw+180,0));
}

void AFactoryEquipment::SetSelected(bool bSelected, bool bInvalid)
{
    if(!Selection)return;
    Selection->SetVisibility(bSelected||bInvalid);
    Tint(Selection,bInvalid?FLinearColor(.9,.05,.03):FLinearColor(1,.75,.04));
}

FVector AFactoryEquipment::Anchor() const
{
    const FVector Offset=Definition.Kind==EFactoryItemKind::WheelSupply?FVector(0,0,90):FVector(0,0,25);
    return GetActorTransform().TransformPosition(Offset);
}
bool AFactoryEquipment::SolveCartesian(const FVector& WorldPoint, FVector& JointCm) const
{
    JointCm=GetActorTransform().InverseTransformPosition(WorldPoint);
    return !JointCm.ContainsNaN() && FMath::Abs(JointCm.X)<=650 && FMath::Abs(JointCm.Y)<=650 && JointCm.Z>=30 && JointCm.Z<=350;
}
FVector AFactoryEquipment::ForwardCartesian(const FVector& JointCm) const { return GetActorTransform().TransformPosition(JointCm); }
void AFactoryEquipment::UpdateTool(const FVector& WorldPoint, double WorldYaw)
{
    if(!Tool)return;
    FVector Q=GetActorTransform().InverseTransformPosition(WorldPoint);
    Tool->SetWorldLocation(WorldPoint+FVector(0,0,28));
    Tool->SetWorldRotation(FRotator(0,WorldYaw,0));
    BeamX->SetRelativeLocation(FVector(Q.X*.5,0,310));
    BeamX->SetRelativeScale3D(FVector(FMath::Max(20.,FMath::Abs(Q.X))/100.,.16,.16));
    BeamY->SetRelativeLocation(FVector(Q.X,Q.Y*.5,295));
    BeamY->SetRelativeScale3D(FVector(.16,FMath::Max(20.,FMath::Abs(Q.Y))/100.,.16));
    Mast->SetRelativeLocation(FVector(Q.X,Q.Y,(295+Q.Z+28)*.5));
    Mast->SetRelativeScale3D(FVector(.12,.12,FMath::Max(10.,FMath::Abs(295-Q.Z-28))/100.));
}

void AFactoryEquipment::SetLabelVisible(bool Visible) { if(Label){Label->SetVisibility(Visible);Label->SetHiddenInGame(!Visible);} }

UFactoryMotionController* AFactoryEquipment::CreateController(){return NewObject<UFactoryMotionController>(this);}

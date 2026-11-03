#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "RobotArm.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

namespace
{
const FString Types[]={TEXT("assembly_wheel_left"),TEXT("assembly_wheel_right"),TEXT("assembly_door")};
}
const ARobotArm* AFactoryRuntime::AssemblyRobotTemplate(const FString& TypeId) const
{
    for(int32 I=0;I<3;++I)if(Types[I]==TypeId&&AssemblyTemplates.IsValidIndex(I))return AssemblyTemplates[I];
    return nullptr;
}
bool AFactoryRuntime::IsPaletteTypeAllowed(const FString& Id) const
{
    return bUseAssemblyIntegration?(Id==TEXT("zone")||AFactoryAssemblyEquipment::IsAssemblyType(Id)):!AFactoryAssemblyEquipment::IsAssemblyType(Id);
}
void AFactoryRuntime::InitializeAssemblyCatalog()
{
    auto* L=TestAssemblyLine.Get();if(!L||!L->WheelCell||!L->DoorCell)return;
    if(AssemblyTemplates.Num()!=3)
        AssemblyTemplates={L->WheelCell->LeftRobot,L->WheelCell->RightRobot,L->DoorCell->Robot};
    if(CalibratedLeftRear.Num()!=6)CalibratedLeftRear=L->WheelCell->LeftRearApproachAnglesDegrees;
    if(CalibratedRightRear.Num()!=6)CalibratedRightRear=L->WheelCell->RightRearApproachAnglesDegrees;
    // Migrate only the old fixed-cell map. A v3 layout with deleted robots stays deleted.
    TSharedPtr<FJsonObject> Source;double Version=0;
    if(FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(InitialLayoutJson),Source)&&Source)
        Source->TryGetNumberField(TEXT("version"),Version);
    for(int32 I=0;I<3;++I)
    {
        auto* T=AssemblyTemplates[I].Get();if(!IsValid(T))continue;
        if(Version<3)
        {
            const auto* D=Registry->Find(Types[I]);if(!D)continue;
            FFactoryLayoutItem Item;Item.Id=TEXT("default_")+Types[I];Item.TypeId=Types[I];
            Item.NameKo=D->NameKo;Item.NameEn=D->NameEn;Item.Size=D->FootprintCm;
            Item.Kind=EFactoryItemKind::ArticulatedRobot;Item.Position=FVector2D(T->GetActorLocation());Item.Yaw=T->GetActorRotation().Yaw;
            if(!Layout.Find(Item.Id))Layout.Items.Add(Item);
            Layout.AssemblyBindings.Add(Types[I],Item.Id);
        }
        T->SetActorHiddenInGame(true);T->SetActorEnableCollision(false);T->SetActorTickEnabled(false);
#if WITH_EDITOR
        T->SetIsTemporarilyHiddenInEditor(true);
#endif
    }
}
void AFactoryRuntime::BindAssemblyRobots()
{
    auto* L=TestAssemblyLine.Get();if(!L||!L->WheelCell||!L->DoorCell)return;
    auto Find=[&](int32 I)->ARobotArm*
    {
        auto* A=Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(Layout.AssemblyBindings.FindRef(Types[I])).Get());
        return A&&A->GetDefinition().EffectiveTypeId()==Types[I]?A->GetArm():nullptr;
    };
    L->WheelCell->LeftRobot=Find(0);L->WheelCell->RightRobot=Find(1);L->DoorCell->Robot=Find(2);
    // Keep the calibrated world transit pose, solve new joint targets for a moved base.
    for(int32 I=0;I<2;++I)
    {
        const auto* Arm=Find(I);auto* T=const_cast<ARobotArm*>(AssemblyRobotTemplate(Types[I]));
        const auto& Original=I==0?CalibratedLeftRear:CalibratedRightRear;
        auto& Angles=I==0?L->WheelCell->LeftRearApproachAnglesDegrees:L->WheelCell->RightRearApproachAnglesDegrees;
        Angles=Original;
        if(Arm&&T&&!Arm->GetActorTransform().Equals(T->GetActorTransform(),.001))
        {
            T->InitializeArm();TArray<double> Radians;for(double D:Original)Radians.Add(FMath::DegreesToRadians(D));FTransform Local;
            Angles.Reset();
            if(FRobotKinematics::ForwardKinematics(T->GetDefinitions(),Radians,T->ToolInLastJointCm,Local))
            {
                const auto Goal=(Local*T->GetActorTransform()).GetRelativeTransform(Arm->GetActorTransform());
                const auto Solution=FRobotKinematics::SolveInverseKinematics(Arm->GetDefinitions(),Arm->ToolInLastJointCm,Goal,Radians);
                if(Solution.IsSuccess())for(double R:Solution.JointRadians)Angles.Add(FMath::RadiansToDegrees(R));
            }
        }
    }
}
bool AFactoryRuntime::ValidateAssemblyBindings(FString& Error) const
{
    for(const auto& Type:Types)
    {
        const auto* I=Layout.Find(Layout.AssemblyBindings.FindRef(Type));
        const auto* D=Registry?Registry->Find(Type):nullptr;
        const FString Name=D?D->NameKo:Type;
        const auto* A=I?Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(I->Id).Get()):nullptr;
        if(!I||I->EffectiveTypeId()!=Type||!IsValid(A)||!IsValid(A->GetArm()))
        {Error=Name+TEXT(" 미연결 · 하단에서 추가 후 왼쪽 작업 연결을 누르세요.");return false;}
        if(!Layout.Contains(*I)){Error=Name+TEXT("이 공장 경계 밖에 있습니다.");return false;}
    }
    return true;
}

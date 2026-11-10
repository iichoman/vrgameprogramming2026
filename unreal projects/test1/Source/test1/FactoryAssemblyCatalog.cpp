#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "RobotArm.h"
#include "CamryAssembly.h"
#include "Components/SceneComponent.h"
#include "DoorPoseComponent.h"
#include "DoorInspectionComponent.h"
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
    FactoryTasks::MigrateLegacy(Layout);
    EnsureRightDoor();
    if(Layout.LayoutVersion<5)
    {
        const auto* Spec=FactoryTasks::Find(TEXT("door_fr"));
        if(!Spec){Status=TEXT("오른쪽 도어 작업 정의가 로드되지 않았습니다. 프로젝트 모듈을 다시 불러오세요.");UE_LOG(LogTemp,Error,TEXT("%s"),*Status);return;}
        // Upgrade once; a user-deleted v5 robot is never respawned by this migration.
        const auto* D=Registry->Find(TEXT("assembly_door"));const auto* T=AssemblyRobotTemplate(TEXT("assembly_door"));
        auto* Car=L->WheelCell->Assembly.Get();
        if(D&&T&&Car)
        {
            const auto Left=Car->GetDoorPoseController(ECamryDoorSide::Left)->GetReferencePoseInBody();
            const auto Right=Car->GetDoorPoseController(ECamryDoorSide::Right)->GetReferencePoseInBody();
            const FQuat Q(FVector::UpVector,PI);const FTransform M(Q,Right.GetLocation()-Q.RotateVector(Left.GetLocation()));
            auto Station=L->VehicleAtWheelStation;Station.AddToTranslation(L->DoorStationOffsetCm);
            const auto Body=Car->GetBodyFrame()->GetComponentTransform().GetRelativeTransform(Car->GetActorTransform())*Station;
            const auto P=(T->GetActorTransform().GetRelativeTransform(Body)*M)*Body;
            FFactoryLayoutItem I;I.Id=TEXT("default_door_second");I.TypeId=TEXT("assembly_door");I.NameKo=TEXT("도어 로봇 2");I.NameEn=TEXT("Door Robot 2");
            I.Kind=EFactoryItemKind::ArticulatedRobot;I.Position=FVector2D(P.GetLocation());I.Yaw=P.Rotator().Yaw;I.Size=D->FootprintCm;
            if(!Layout.Find(I.Id))Layout.Items.Add(I);
            auto* Task=Layout.Tasks.FindByPredicate([](const auto& V){return V.Id==TEXT("door_fr");});
            if(!Task){FFactoryTask New;New.Id=Spec->Id;New.Operation=Spec->Operation;New.ProductId=TEXT("vehicle_1");New.TargetPort=Spec->TargetPort;New.PartType=Spec->PartType;New.SourcePort=Spec->SourcePort;Layout.Tasks.Add(New);Task=&Layout.Tasks.Last();}
            Task->RobotId=I.Id;Task->PartId=TEXT("part_door_fr");
        }
        for(auto& Tsk:Layout.Tasks)if(Tsk.PartId.IsEmpty())Tsk.PartId=TEXT("part_")+Tsk.Id;
        Layout.LayoutVersion=5;
    }
}
void AFactoryRuntime::BindAssemblyRobots()
{
    auto* L=TestAssemblyLine.Get();if(!L||!L->WheelCell||!L->DoorCell)return;
    EnsureRightDoor();ConfigurePartPlacement();
    auto Find=[&](const FString& Id)->ARobotArm*
    {
        const auto* Task=Layout.Tasks.FindByPredicate([&](const auto& T){return T.Id==Id;});
        auto* A=Task?Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(Task->RobotId).Get()):nullptr;
        return A?A->GetArm():nullptr;
    };
    auto* W=L->WheelCell.Get();
    W->TaskRobots={Find(TEXT("wheel_fl")),Find(TEXT("wheel_rl")),Find(TEXT("wheel_fr")),Find(TEXT("wheel_rr"))};
    W->LeftRobot=W->TaskRobots[0];W->RightRobot=W->TaskRobots[2];L->DoorCell->Robot=Find(TEXT("door_fl"));
    W->TaskPartIndices.Reset();
    for(const TCHAR* Id:{TEXT("wheel_fl"),TEXT("wheel_rl"),TEXT("wheel_fr"),TEXT("wheel_rr")})
    {const auto* Task=Layout.Tasks.FindByPredicate([&](const auto& V){return V.Id==Id;});
        const auto* P=Task&&!Task->PartId.IsEmpty()?FactoryTasks::Find(Task->PartId.Mid(5)):FactoryTasks::Find(Id);W->TaskPartIndices.Add(P?P->WheelIndex:W->TaskPartIndices.Num());}
    if(RightDoorCell)RightDoorCell->Robot=Find(TEXT("door_fr"));
    W->PrepareTaskLanes();
    for(int32 Lane=0;Lane<4;++Lane)
    {
        const int32 Index=Lane,Side=Index/2;
        const auto* Arm=W->TaskRobots[Index].Get();auto* T=const_cast<ARobotArm*>(AssemblyRobotTemplate(Types[Side]));
        const auto& Original=Side==0?CalibratedLeftRear:CalibratedRightRear;
        auto& Angles=W->RearTransit[Lane];Angles=Original;
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
    W->LeftRearApproachAnglesDegrees=W->RearTransit[1];W->RightRearApproachAnglesDegrees=W->RearTransit[3];
}

bool AFactoryRuntime::ValidateAssemblyBindings(FString& Error) const
{
    if(!FactoryTasks::Validate(Layout,Error,true))return false;
    for(const auto& Task:Layout.Tasks)
    {
        const auto* I=Layout.Find(Task.RobotId);
        const auto* A=I?Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(I->Id).Get()):nullptr;
        if(!I||!IsValid(A)||!IsValid(A->GetArm())){Error=TEXT("작업 로봇 인스턴스가 없습니다.");return false;}
        if(!Layout.Contains(*I)){Error=I->NameKo+TEXT("이 공장 경계 밖에 있습니다.");return false;}
    }
    const auto* LeftFront=Layout.Tasks.FindByPredicate([](const auto& T){return T.Id==TEXT("door_fl");});
    const auto* RightFront=Layout.Tasks.FindByPredicate([](const auto& T){return T.Id==TEXT("door_fr");});
    if(LeftFront && RightFront && LeftFront->RobotId==RightFront->RobotId)
    {Error=TEXT("현재 두 도어는 병렬 작업이므로 서로 다른 로봇에 할당하세요.");return false;}
    return true;
}

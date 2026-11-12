#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "DoorPoseComponent.h"
#include "DoorInspectionComponent.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "VehicleProcessTest.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"

void AFactoryRuntime::EnsureRightDoor()
{
    auto* L=TestAssemblyLine.Get();if(!L||!L->DoorCell||!L->WheelCell||!L->WheelCell->Assembly)return;
    auto* D=L->DoorCell.Get();auto* Car=D->Assembly.Get();if(!Car)return;
    if(!bDoorPickupCaptured){DefaultDoorPickup=D->PickupOffsetBodyCm;bDoorPickupCaptured=true;}
    if(!RightDoorCell)
    {
        FActorSpawnParameters P;P.Owner=this;P.Template=D;
        RightDoorCell=GetWorld()->SpawnActor<ADoorAssemblyCell>(ADoorAssemblyCell::StaticClass(),FTransform::Identity,P);
        if(!RightDoorCell)return;
        auto* R=RightDoorCell.Get();R->bRightDoor=true;R->bExternallyDriven=true;R->bAutoStart=false;R->SetActorTickEnabled(false);R->Assembly=Car;
        const auto Left=Car->GetDoorPoseController(ECamryDoorSide::Left)->GetReferencePoseInBody();
        const auto Right=Car->GetDoorPoseController(ECamryDoorSide::Right)->GetReferencePoseInBody();
        // The integrated map attaches Door_FL to its pose frame, but leaves Door_FR on the body.
        // Reuse the actual process module and its captured mounted pose (do not duplicate geometry).
        const auto RightInCar=Right*Car->GetBodyFrame()->GetComponentTransform().GetRelativeTransform(Car->GetActorTransform());
        for(TActorIterator<AVehicleProcessModule> It(GetWorld());It;++It)
        {
            if(It->VehicleFrame!=Car||It->ModuleId!=FName(TEXT("Door_FR"))||!It->bHasReferencePose)continue;
            if(It->AttachToComponent(Car->GetRightDoor(),FAttachmentTransformRules::KeepWorldTransform))
                It->GetRootComponent()->SetRelativeTransform(It->ReferencePoseInVehicle.GetRelativeTransform(RightInCar));
            break;
        }
        const FQuat Q(FVector::UpVector,PI);const FTransform M(Q,Right.GetLocation()-Q.RotateVector(Left.GetLocation()));
        R->TCPInDoorCm=((D->TCPInDoorCm*Left)*M).GetRelativeTransform(Right);
        R->ApproachOffsetBodyCm=Q.RotateVector(D->ApproachOffsetBodyCm);
        R->LiftOffsetBodyCm=Q.RotateVector(D->LiftOffsetBodyCm);R->TransferOffsetBodyCm=Q.RotateVector(D->TransferOffsetBodyCm);
        // Mirrored virtual reference gauges, not OEM panel gap geometry.
        auto* Gauge=NewObject<UDoorInspectionComponent>(R);R->AddInstanceComponent(Gauge);
        Gauge->Configure(Car->GetDoorPoseController(ECamryDoorSide::Right),Car->GetBodyFrame(),{});
        if(auto* Original=Car->FindComponentByClass<UDoorInspectionComponent>())for(auto Point:Original->Points)
        {
            auto Mirror=[](FVector V){V.Y=-V.Y;return V;};
            Point.DoorPointLocalCm=Right.InverseTransformPosition(Mirror(Left.TransformPosition(Point.DoorPointLocalCm)));
            Point.BodyPointLocalCm=Mirror(Point.BodyPointLocalCm);Point.ReferenceDoorPointBodyCm=Mirror(Point.ReferenceDoorPointBodyCm);
            Point.GapDirectionBody=Mirror(Point.GapDirectionBody);Point.FlushDirectionBody=Mirror(Point.FlushDirectionBody);
            Gauge->Points.Add(Point);
        }
        Gauge->bShowInspection=false;Gauge->RegisterComponent();R->InspectionOverride=Gauge;
    }
    L->SecondDoorCell=RightDoorCell;
}
void AFactoryRuntime::ConfigurePartPlacement()
{
    auto* L=TestAssemblyLine.Get();if(!L||!L->WheelCell||!L->DoorCell)return;
    auto* W=L->WheelCell.Get();W->SupplyOffsetsCm.Reset();
    for(const auto& S:FactoryTasks::Catalog())if(S.WheelIndex!=INDEX_NONE)W->SupplyOffsetsCm.Add(S.WheelIndex,Layout.PartOffsetsCm.FindRef(TEXT("part_")+S.Id));
    if(L->DoorCell->Assembly)
    {
        const auto Body=L->DoorCell->Assembly->GetBodyFrame()->GetComponentTransform();
        L->DoorCell->PickupOffsetBodyCm=DefaultDoorPickup+Body.InverseTransformVectorNoScale(Layout.PartOffsetsCm.FindRef(TEXT("part_door_fl")));
        if(RightDoorCell)RightDoorCell->PickupOffsetBodyCm=FQuat(FVector::UpVector,PI).RotateVector(DefaultDoorPickup)
            +Body.InverseTransformVectorNoScale(Layout.PartOffsetsCm.FindRef(TEXT("part_door_fr")));
    }
}
bool AFactoryRuntime::PartWorld(const FString& Id,FTransform& Pose) const
{
    const auto* S=Id.StartsWith(TEXT("part_"))?FactoryTasks::Find(Id.Mid(5)):nullptr;
    auto* L=TestAssemblyLine.Get();if(!S||!L||!L->WheelCell||!L->WheelCell->Assembly)return false;
    if(S->WheelIndex!=INDEX_NONE){Pose=L->WheelCell->GetPhysicalPartWorld(S->WheelIndex);return true;}
    auto* Door=Id==TEXT("part_door_fr")?L->WheelCell->Assembly->GetRightDoor():L->WheelCell->Assembly->GetLeftDoor();
    if(!Door)return false;Pose=Door->GetComponentTransform();return true;
}
bool AFactoryRuntime::SetPartWorld(const FString& Id,const FTransform& Pose)
{
    const auto* S=Id.StartsWith(TEXT("part_"))?FactoryTasks::Find(Id.Mid(5)):nullptr;
    auto* L=TestAssemblyLine.Get();if(!S||!L||!L->WheelCell||!L->WheelCell->Assembly||!Pose.IsValid())return false;
    if(S->WheelIndex!=INDEX_NONE)return L->WheelCell->SetPhysicalPartWorld(S->WheelIndex,Pose);
    return L->WheelCell->Assembly->GetDoorPoseController(Id==TEXT("part_door_fr")?ECamryDoorSide::Right:ECamryDoorSide::Left)->SetDoorWorldPose(Pose);
}
FBox AFactoryRuntime::PartBounds(const FString& Id) const
{
    FTransform P;if(!PartWorld(Id,P))return FBox(ForceInit);
    if(!Id.StartsWith(TEXT("part_door")))return FBox(P.GetLocation()-FVector(38),P.GetLocation()+FVector(38));
    auto* Car=TestAssemblyLine->WheelCell->Assembly.Get();
    auto* Door=Id==TEXT("part_door_fr")?Car->GetRightDoor():Car->GetLeftDoor();
    TArray<USceneComponent*> DoorComponents;Door->GetChildrenComponents(true,DoorComponents);
    FBox Bounds(ForceInit);
    for(auto* C:DoorComponents)if(auto* Mesh=Cast<UStaticMeshComponent>(C))if(Mesh->GetStaticMesh()&&Mesh->IsVisible()&&!Mesh->bHiddenInGame)
        Bounds+=Mesh->CalcBounds(Mesh->GetComponentTransform()).GetBox();
    return Bounds;
}
namespace
{
double RayBox(FVector O,FVector D,FBox B)
{
    double Min=0,Max=100000;
    for(int I=0;I<3;++I){if(FMath::Abs(D[I])<1e-8){if(O[I]<B.Min[I]||O[I]>B.Max[I])return -1;continue;}
        double A=(B.Min[I]-O[I])/D[I],C=(B.Max[I]-O[I])/D[I];if(A>C)Swap(A,C);Min=FMath::Max(Min,A);Max=FMath::Min(Max,C);if(Max<Min)return -1;}
    return Min;
}
}
FString AFactoryRuntime::HitPart(FVector2D Pixel) const
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);FVector O,D;if(!PC||!PC->DeprojectScreenPositionToWorld(Pixel.X,Pixel.Y,O,D))return {};
    FString Best;double Dist=100000;
    for(const auto& S:FactoryTasks::Catalog())
    {
        const FString Id=TEXT("part_")+S.Id;
        const FBox Bounds=PartBounds(Id);
        if(!Bounds.IsValid)continue;
        const double T=RayBox(O,D,Bounds);if(T>=0&&T<Dist){Dist=T;Best=Id;}
    }
    return Best;
}
FString AFactoryRuntime::HitRobot(FVector2D Pixel) const
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);FVector O,D;if(!PC||!PC->DeprojectScreenPositionToWorld(Pixel.X,Pixel.Y,O,D))return {};
    FString Best;double Dist=100000;
    for(const auto& Pair:Equipment)if(auto* A=Cast<AFactoryAssemblyEquipment>(Pair.Value.Get()))if(A->GetArm())
    {
        const double T=RayBox(O,D,A->GetArm()->GetComponentsBoundingBox(true));if(T>=0&&T<Dist){Dist=T;Best=Pair.Key;}
    }
    return Best;
}
bool AFactoryRuntime::DragPlane(FVector2D Pixel,double Z,FVector& Out) const
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);FVector O,D;if(!PC||!PC->DeprojectScreenPositionToWorld(Pixel.X,Pixel.Y,O,D)||FMath::Abs(D.Z)<1.e-6)return false;
    const double T=(Z-O.Z)/D.Z;if(T<0)return false;Out=O+D*T;return true;
}
void AFactoryRuntime::OpenRobotPopup(const FString& Id,FVector2D Pixel)
{
    OpenDetails(EFactoryDetailTab::Selection);
    SelectItem(Id);SelectedPartId.Empty();PendingTaskId.Empty();bRobotPopup=bAssigningTask=true;
    RobotPopupPixel=Pixel+FVector2D(25,20);bNeedsPanel=true;
    Status=TEXT("① 차체의 작업 지점 클릭 → ② 운반할 부품 클릭 · Esc 닫기");
}
void AFactoryRuntime::CloseRobotPopup(){bRobotPopup=bAssigningTask=false;PendingTaskId.Empty();Inspection().InteractionIssue.Reset();bNeedsPanel=true;}
bool AFactoryRuntime::ChoosePart(const FString& Id)
{
    if(!CanEdit()||!Selected()||PendingTaskId.IsEmpty())return false;
    auto* Task=Layout.Tasks.FindByPredicate([&](const auto& T){return T.Id==PendingTaskId;});
    const auto* Part=Id.StartsWith(TEXT("part_"))?FactoryTasks::Find(Id.Mid(5)):nullptr;
    Inspection().InteractionIssue.Reset();
    auto Reject=[&](const FString& Detail){Status=Detail;FFactoryIssue I;I.Id=TEXT("assignment_rejected");I.Summary=TEXT("작업 연결 실패");I.Detail=Detail;I.Remedy=TEXT("현재 작업 지점과 호환되는 미사용 부품을 선택하세요.");I.RobotId=SelectedId;I.PartId=Id;I.TaskId=PendingTaskId;I.Objects={SelectedId,Id};Inspection().InteractionIssue=I;RefreshInspection();bNeedsPanel=true;return false;};
    if(!Task||!Part||Part->PartType!=Task->PartType||(Part->PartType==TEXT("door")&&Id!=TEXT("part_")+Task->Id))
    {return Reject(TEXT("선택한 장착 지점과 부품이 호환되지 않습니다."));}
    auto Candidate=Layout;auto* T=Candidate.Tasks.FindByPredicate([&](const auto& V){return V.Id==PendingTaskId;});T->PartId=Id;T->SourcePort=Part->SourcePort;T->RobotId=SelectedId;
    FString Error;if(!Candidate.Validate(Error)){return Reject(Error+TEXT(" 기존 연결을 해제한 뒤 선택하세요."));}
    BeginEdit();*Task=*T;PendingTaskId.Empty();BindAssemblyRobots();ResetCycle();EndEdit();bNeedsPanel=true;
    Status=TEXT("연결 완료. 다른 작업 지점을 클릭하면 이 로봇에 작업을 추가할 수 있습니다.");return true;
}

#include "FactoryRuntime.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "DrawDebugHelpers.h"

void AFactoryRuntime::BeginTaskAssignment()
{
    if(!CanEdit()||!Selected())return;
    bool Supported=false;for(const auto& S:FactoryTasks::Catalog())Supported|=FactoryTasks::Supports(Selected()->EffectiveTypeId(),S);
    if(!Supported){Status=TEXT("작업 가능한 로봇을 선택하세요.");return;}
    CancelEdit();bAssigningTask=true;
    Status=TEXT("초록색 작업 지점을 클릭하세요. 같은 로봇에 여러 작업 할당 가능 · Esc 취소");bNeedsPanel=true;
}
bool AFactoryRuntime::AssignTask(const FString& Id)
{
    if(!CanEdit()||!Selected())return false;
    auto* T=Layout.Tasks.FindByPredicate([&](const auto& V){return V.Id==Id;});const auto* S=FactoryTasks::Find(Id);
    if(!T||!S||!FactoryTasks::Supports(Selected()->EffectiveTypeId(),*S))
    {Status=TEXT("이 로봇의 툴·보정으로 수행할 수 없는 작업입니다.");return false;}
    if(T->RobotId==SelectedId){Status=TEXT("이미 이 로봇에 할당된 작업입니다.");return true;}
    BeginEdit();T->RobotId=SelectedId;if(T->PartId.IsEmpty())T->PartId=TEXT("part_")+T->Id;BindAssemblyRobots();ResetCycle();FinishDiscreteEdit();
    Status=S->Label+TEXT(" 작업을 선택한 로봇에 할당했습니다. 기존 담당 로봇의 해당 작업은 해제됩니다.");bNeedsPanel=true;return true;
}
void AFactoryRuntime::UnassignTask(const FString& Id)
{
    if(!CanEdit())return;auto* T=Layout.Tasks.FindByPredicate([&](const auto& V){return V.Id==Id;});
    if(!T||T->RobotId!=SelectedId)return;
    BeginEdit();T->RobotId.Empty();T->PartId.Empty();BindAssemblyRobots();ResetCycle();FinishDiscreteEdit();bNeedsPanel=true;
}
void AFactoryRuntime::SetTaskPartMode(const FString& Id,bool Explicit)
{
    if(!CanEdit())return;auto* T=Layout.Tasks.FindByPredicate([&](const auto& V){return V.Id==Id;});if(!T)return;
    BeginEdit();T->PartId=Explicit?TEXT("part_")+T->Id:FString();FinishDiscreteEdit();bNeedsPanel=true;
    Status=Explicit?TEXT("해당 공급 포트의 부품 ID를 지정했습니다."):TEXT("해당 공급 포트의 호환 부품을 자동 선택합니다. 현재 포트당 부품은 1개입니다.");
}
bool AFactoryRuntime::TaskTargetWorld(const FString& Id,FVector& Out) const
{
    const auto* S=FactoryTasks::Find(Id);const auto* L=TestAssemblyLine.Get();if(!S||!L)return false;
    if(S->WheelIndex!=INDEX_NONE&&L->WheelCell){Out=L->WheelCell->GetMountWorld(S->WheelIndex).GetLocation();return true;}
    if(S->Operation==TEXT("door_install_demo"))
    {
        auto* D=Id==TEXT("door_fr")?RightDoorCell.Get():L->DoorCell.Get();FTransform Part;
        const auto Bounds=PartBounds(TEXT("part_")+Id);
        if(D&&Bounds.IsValid&&PartWorld(TEXT("part_")+Id,Part))
        {Out=D->GetMountDoorWorld().TransformPosition(Part.InverseTransformPosition(Bounds.GetCenter()));return true;}
    }
    return false;
}
bool AFactoryRuntime::HitTaskTarget(FVector2D Pixel)
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);if(!PC||!Selected())return false;
    FString Best;double Distance=36;
    for(const auto& S:FactoryTasks::Catalog())
    {
        if(!FactoryTasks::Supports(Selected()->EffectiveTypeId(),S))continue;
        FVector World;FVector2D Screen;
        if(TaskTargetWorld(S.Id,World)&&PC->ProjectWorldLocationToScreen(World,Screen))
        {const double D=(Screen-Pixel).Size();if(D<Distance){Distance=D;Best=S.Id;}}
    }
    if(!Best.IsEmpty()){PendingTaskId=Best;Status=FactoryTasks::Find(Best)->Label+TEXT(" 선택 · 이제 실제 부품을 클릭하세요.");bNeedsPanel=true;return true;}
    Status=TEXT("차체의 초록색 작업 지점을 클릭하세요. Esc로 닫을 수 있습니다.");return false;
}
void AFactoryRuntime::DrawTaskTargets()
{
    if(!bRobotPopup||!CanEdit()||!Selected())return;
    for(const auto& S:FactoryTasks::Catalog())if(FactoryTasks::Supports(Selected()->EffectiveTypeId(),S))
    {
        FVector P;if(!TaskTargetWorld(S.Id,P))continue;
        DrawDebugSphere(GetWorld(),P,16,12,FColor::Green,false,0,0,3);
        DrawDebugString(GetWorld(),P+FVector(0,0,22),S.Label,nullptr,FColor::White,0,true);
    }
}

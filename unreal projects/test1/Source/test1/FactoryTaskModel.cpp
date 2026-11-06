#include "FactoryTaskModel.h"
#include "FactoryLayoutModel.h"

namespace
{
// Live Coding preserves previously constructed function-local statics. Give each
// incompatible catalog revision its own storage (including its initialization guard).
const TArray<FFactoryTaskSpec>& DirectEditCatalogV5()
{
    // Adapter registration boundary. Add calibrated targets here; do not infer from mesh names.
    static const TArray<FFactoryTaskSpec> Specs={
        {TEXT("wheel_fl"),TEXT("왼쪽 앞 휠"),TEXT("wheel_install"),TEXT("hub_fl"),TEXT("wheel"),TEXT("rack_fl"),TEXT("wheel_left"),0},
        {TEXT("wheel_rl"),TEXT("왼쪽 뒤 휠"),TEXT("wheel_install"),TEXT("hub_rl"),TEXT("wheel"),TEXT("rack_rl"),TEXT("wheel_left"),1},
        {TEXT("wheel_fr"),TEXT("오른쪽 앞 휠"),TEXT("wheel_install"),TEXT("hub_fr"),TEXT("wheel"),TEXT("rack_fr"),TEXT("wheel_right"),2},
        {TEXT("wheel_rr"),TEXT("오른쪽 뒤 휠"),TEXT("wheel_install"),TEXT("hub_rr"),TEXT("wheel"),TEXT("rack_rr"),TEXT("wheel_right"),3},
        {TEXT("door_fl"),TEXT("왼쪽 앞문"),TEXT("door_install_demo"),TEXT("door_fl"),TEXT("door"),TEXT("door_rack_fl"),TEXT("door"),INDEX_NONE},
        {TEXT("door_fr"),TEXT("오른쪽 앞문"),TEXT("door_install_demo"),TEXT("door_fr"),TEXT("door"),TEXT("door_rack_fr"),TEXT("door"),INDEX_NONE}};
    return Specs;
}
}
const TArray<FFactoryTaskSpec>& FactoryTasks::Catalog(){return DirectEditCatalogV5();}
const FFactoryTaskSpec* FactoryTasks::Find(const FString& Id)
{ return Catalog().FindByPredicate([&](const auto& S){return S.Id==Id;}); }
bool FactoryTasks::Supports(const FString& Type,const FFactoryTaskSpec& S)
{
    return (Type.StartsWith(TEXT("assembly_wheel"))&&S.PartType==TEXT("wheel"))
        ||(Type==TEXT("assembly_door")&&S.PartType==TEXT("door"));
}
void FactoryTasks::MigrateLegacy(FFactoryLayout& L)
{
    if(L.bTasksInitialized)return;
    if(L.AssemblyBindings.IsEmpty())return;
    L.bTasksInitialized=true;
    for(const auto& S:Catalog())
    {
        FFactoryTask T;T.Id=S.Id;T.Operation=S.Operation;T.ProductId=TEXT("vehicle_1");
        T.TargetPort=S.TargetPort;T.PartType=S.PartType;T.SourcePort=S.SourcePort;
        T.PartId=TEXT("part_")+T.Id;
        T.RobotId=L.AssemblyBindings.FindRef(TEXT("assembly_")+S.Capability);
        L.Tasks.Add(T);
    }
}
bool FactoryTasks::Validate(const FFactoryLayout& L,FString& Error,bool RequireAssigned)
{
    if(L.Tasks.Num()>128){Error=TEXT("작업은 최대 128개입니다.");return false;}
    TSet<FString> Ids,Targets,Parts;
    for(const auto& T:L.Tasks)
    {
        const auto* S=Find(T.Id);
        if(!S||Ids.Contains(T.Id)||T.Operation!=S->Operation||T.TargetPort!=S->TargetPort
            ||T.PartType!=S->PartType||T.ProductId!=TEXT("vehicle_1"))
        {Error=TEXT("등록되지 않은 작업·포트 또는 중복 작업입니다.");return false;}
        Ids.Add(T.Id);
        const FString Target=T.ProductId+TEXT("/")+T.TargetPort+TEXT("/")+T.Operation;
        if(Targets.Contains(Target)){Error=TEXT("같은 대상 작업이 중복되었습니다.");return false;}Targets.Add(Target);
        if(!T.PartId.IsEmpty())
        {
            const auto* Part=T.PartId.StartsWith(TEXT("part_"))?Find(T.PartId.Mid(5)):nullptr;
            if(!Part||Part->PartType!=S->PartType||T.SourcePort!=Part->SourcePort||(S->PartType==TEXT("door")&&T.PartId!=TEXT("part_")+T.Id)||Parts.Contains(T.PartId))
            {Error=TEXT("부품 ID가 공급처와 맞지 않거나 중복 예약되었습니다.");return false;}
            Parts.Add(T.PartId);
        }
        if(T.RobotId.IsEmpty()){if(!RequireAssigned)continue;Error=S->Label+TEXT(" 작업이 미할당입니다.");return false;}
        const auto* I=L.Find(T.RobotId);
        if(!I||!Supports(I->EffectiveTypeId(),*S)){Error=S->Label+TEXT(" 작업과 로봇 툴 보정이 호환되지 않습니다.");return false;}
    }
    if(RequireAssigned&&Ids.Num()!=Catalog().Num()){Error=TEXT("현재 조립 레시피의 필수 작업이 누락되었습니다.");return false;}
    return true;
}

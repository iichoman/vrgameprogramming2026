#include "FactoryEquipmentRegistry.h"
#include "FactoryEquipment.h"
#include "FactoryRobotEquipment.h"

void UFactoryEquipmentRegistry::Initialize()
{
    Definitions.Reset();
    auto Add=[this](const TCHAR* Id,const TCHAR* Ko,const TCHAR* En,const TCHAR* Category,EFactoryItemKind Kind,FVector2D Size,UClass* Class,const TCHAR* Description)
    {
        auto* D=NewObject<UFactoryEquipmentDefinition>(this);
        D->TypeId=FName(Id);D->NameKo=Ko;D->NameEn=En;D->Category=Category;
        D->Kind=FFactoryLayout::KindName(Kind);D->FootprintCm=Size;D->ActorClass=Class;D->Description=Description;
        FString Error;Register(D,Error);
    };
    Add(TEXT("zone"),TEXT("작업 구역"),TEXT("Work Zone"),TEXT("구역"),EFactoryItemKind::Zone,{600,400},AFactoryEquipment::StaticClass(),TEXT("바닥을 드래그해 구역을 그립니다."));
    Add(TEXT("vehicle_station"),TEXT("차량 정지대"),TEXT("Vehicle Station"),TEXT("설비"),EFactoryItemKind::VehicleStation,{520,220},AFactoryEquipment::StaticClass(),TEXT("차량과 허브 기준 위치. 휠 작업에 연결해서 사용합니다."));
    Add(TEXT("wheel_supply"),TEXT("휠 공급대"),TEXT("Wheel Supply"),TEXT("설비"),EFactoryItemKind::WheelSupply,{150,150},AFactoryEquipment::StaticClass(),TEXT("휠 인수 기준점이 있는 공급대."));
    Add(TEXT("kuka_robot"),TEXT("KUKA 로봇"),TEXT("KUKA Robot"),TEXT("로봇"),EFactoryItemKind::ArticulatedRobot,{90,90},AFactoryRobotEquipment::StaticClass(),TEXT("기존 KUKA 링크 모델 · IK + 관절 PID · 단순 휠 그리퍼."));
    Add(TEXT("cartesian_handler"),TEXT("직교 이송장치"),TEXT("Cartesian Handler"),TEXT("로봇"),EFactoryItemKind::CartesianHandler,{90,90},AFactoryEquipment::StaticClass(),TEXT("기존 저장 배치와 비교 실험을 위한 직교 PID 장치."));
    Add(TEXT("conveyor"),TEXT("컨베이어 모형"),TEXT("Conveyor Proxy"),TEXT("설비"),EFactoryItemKind::Conveyor,{600,240},AFactoryEquipment::StaticClass(),TEXT("배치·간섭 확인용 모형. 벨트 구동 기능은 없습니다."));
}
bool UFactoryEquipmentRegistry::Register(UFactoryEquipmentDefinition* D,FString& Error)
{
    EFactoryItemKind Kind;
    if(!D||D->TypeId.IsNone()||Find(D->TypeId.ToString())||!D->ActorClass||D->ActorClass->HasAnyClassFlags(CLASS_Abstract|CLASS_CompiledFromBlueprint)
        ||!FFactoryLayout::ParseKind(D->Kind,Kind)||D->NameKo.IsEmpty()||D->NameEn.IsEmpty())
    {Error=TEXT("설비 등록 실패: 고유 TypeId, 한국어·영어 이름, 네이티브 Actor 클래스와 종류를 확인하세요.");return false;}
    FFactoryLayout Test;Test.StationId=Test.SupplyId=Test.HandlerId=TEXT("");FFactoryLayoutItem Item;
    Item.Id=TEXT("validation");Item.TypeId=D->TypeId.ToString();Item.NameKo=D->NameKo;Item.NameEn=D->NameEn;Item.Kind=Kind;Item.Size=D->FootprintCm;Test.Items.Add(Item);
    if(!Test.Validate(Error))return false;
    Definitions.Add(D);return true;
}
const UFactoryEquipmentDefinition* UFactoryEquipmentRegistry::Find(const FString& Id) const
{for(const auto& D:Definitions)if(D&&D->TypeId.ToString()==Id)return D;return nullptr;}
bool UFactoryEquipmentRegistry::Validate(const FFactoryLayout& Layout,FString& Error) const
{
    for(const auto& Item:Layout.Items)
    {
        const auto* D=Find(Item.EffectiveTypeId());
        if(!D||D->Kind!=FFactoryLayout::KindName(Item.Kind))
        {Error=TEXT("등록되지 않았거나 종류가 맞지 않는 설비: ")+Item.EffectiveTypeId();return false;}
    }
    return true;
}

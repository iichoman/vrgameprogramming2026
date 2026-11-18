#pragma once
#include "CoreMinimal.h"

struct FAssemblySituation
{
    int32 Preset;
    const TCHAR* Label;
    FName Process;
};

namespace AssemblySituations
{
inline const TArray<FAssemblySituation>& Catalog()
{
    static const TArray<FAssemblySituation> Items={
        {0,TEXT("정상"),NAME_None},
        {1,TEXT("부품 누락"),TEXT("wheels")}, {2,TEXT("센서 끊김"),TEXT("wheels")},
        {3,TEXT("파지 실패"),TEXT("wheels")}, {4,TEXT("휠 위치 변화"),TEXT("wheels")},
        {7,TEXT("저마찰"),TEXT("wheels")}, {8,TEXT("과중량"),TEXT("wheels")},
        {14,TEXT("운반 중 마찰 저하"),TEXT("wheels")}, {15,TEXT("레이저 재스캔"),TEXT("wheels")},
        {16,TEXT("과대 휠 교체"),TEXT("wheels")}, {17,TEXT("과소 휠 교체"),TEXT("wheels")},
        {11,TEXT("과대 문 교체"),TEXT("doors")}, {12,TEXT("과소 문 교체"),TEXT("doors")},
        {13,TEXT("도킹 초과"),TEXT("wheels")}, {19,TEXT("차체 위치 변화"),TEXT("wheels")},
        {20,TEXT("지그 이물질"),TEXT("doors")}, {21,TEXT("강제 체결 손상"),TEXT("doors")}
    };
    return Items;
}
inline FString Label(int32 Preset)
{
    for(const auto& Item:Catalog())if(Item.Preset==Preset)return Item.Label;
    return Preset==5||Preset==18?TEXT("무작위 복구"):TEXT("정상");
}
// No sensor/control code reads this selection. It only configures plant inputs.
inline int32 Choose(int32 Seed,FName Process,double NormalProbability)
{
    FRandomStream Random(Seed);
    if(Random.FRand()<FMath::Clamp(NormalProbability,0.,1.))return 0;
    TArray<int32> Candidates;
    for(const auto& Item:Catalog())if(Item.Preset!=0&&(Process.IsNone()||Item.Process==Process))Candidates.Add(Item.Preset);
    return Candidates.IsEmpty()?0:Candidates[Random.RandRange(0,Candidates.Num()-1)];
}
}

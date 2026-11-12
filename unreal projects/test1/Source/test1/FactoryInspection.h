#pragma once
#include "CoreMinimal.h"
#include "UObject/StrongObjectPtr.h"
#include "Materials/MaterialInterface.h"
#include "Components/StaticMeshComponent.h"

enum class EFactoryDetailTab : uint8 { Selection, Problems, Process, Sensors, Results, Layout };
enum class EFactoryIssueLevel : uint8 { Warning, Error };
enum class EFactorySampleState : uint8 { Disconnected, Valid, Invalid, Stale };

/** Presentation records contain stable identities, never control commands. */
struct FFactoryIssue
{
    FString Id, Summary, Detail, Remedy, RobotId, PartId, TaskId;
    TArray<FString> Objects;
    EFactoryIssueLevel Level=EFactoryIssueLevel::Error;
};
struct FFactoryTrendPoint { double Time=0, Value=0; bool Valid=false; };
struct FFactorySensorValue
{
    FString Id, Label, ObjectId, TaskId, Unit, Source;
    double Value=0, SampleTime=-1, ClockTime=0, MaxAge=.25;
    bool Connected=false, Valid=false;
    EFactorySampleState State() const
    {
        if(!Connected)return EFactorySampleState::Disconnected;
        if(!Valid||!FMath::IsFinite(Value)||!FMath::IsFinite(SampleTime)||!FMath::IsFinite(ClockTime)||!FMath::IsFinite(MaxAge)||MaxAge<0||SampleTime<0)return EFactorySampleState::Invalid;
        return ClockTime<SampleTime||ClockTime-SampleTime>MaxAge?EFactorySampleState::Stale:EFactorySampleState::Valid;
    }
};
struct FFactorySensorChannel
{
    FFactorySensorValue Latest;
    TArray<FFactoryTrendPoint> History;
};
struct FFactoryInspectionModel
{
    EFactoryDetailTab Tab=EFactoryDetailTab::Selection;
    bool PanelOpen=false, PaletteOpen=true, ShowAllSensors=true;
    FString LooseSelection, FocusedIssue, SelectedChannel=TEXT("arrival.speed");
    TArray<FFactoryIssue> Issues;
    TOptional<FFactoryIssue> InteractionIssue;
    FString StartError;
    TMap<FString,FFactorySensorChannel> Sensors;
    double NextRefresh=0;
    TStrongObjectPtr<UMaterialInterface> SelectedMaterial, IssueMaterial;
    TMap<TWeakObjectPtr<UStaticMeshComponent>,TStrongObjectPtr<UMaterialInterface>> OriginalOverlays;
    TMap<TWeakObjectPtr<UStaticMeshComponent>,uint8> OverlayKinds;

    // Call on the game thread. A new sensor needs only an adapter publishing this contract.
    // Source must identify simulated / physical / vision data; absent data never becomes zero.
    void Publish(const FFactorySensorValue& Value)
    {
        if(Value.Id.IsEmpty()||!FMath::IsFinite(Value.ClockTime))return;
        auto& C=Sensors.FindOrAdd(Value.Id);
        if(Value.Connected&&Value.SampleTime>=0&&Value.SampleTime<C.Latest.SampleTime)C.History.Reset(); // New simulation epoch.
        if(Value.Connected&&FMath::IsFinite(Value.SampleTime)&&Value.SampleTime>=0&&(C.History.IsEmpty()||C.Latest.SampleTime!=Value.SampleTime))
        {
            if(C.History.Num()>=180)C.History.RemoveAt(0,C.History.Num()-179,EAllowShrinking::No);
            C.History.Add({Value.SampleTime,Value.Value,Value.State()==EFactorySampleState::Valid});
        }
        else if(Value.State()!=EFactorySampleState::Valid&&!C.History.IsEmpty()&&C.History.Last().Valid)
        {
            if(C.History.Num()>=180)C.History.RemoveAt(0,1,EAllowShrinking::No);
            C.History.Add({FMath::Max(Value.ClockTime,C.History.Last().Time),0,false});
        }
        C.Latest=Value;
    }
    void RestoreOverlays()
    {
        for(auto& P:OriginalOverlays)if(P.Key.IsValid())P.Key->SetOverlayMaterial(P.Value.Get());
        OriginalOverlays.Empty();OverlayKinds.Empty();
    }
};

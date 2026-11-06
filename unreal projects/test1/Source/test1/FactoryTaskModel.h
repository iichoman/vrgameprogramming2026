#pragma once
#include "CoreMinimal.h"

// Persistent intent, not an Actor/mesh owner. IDs survive save/load and Undo.
struct FFactoryTask
{
    FString Id, Operation, ProductId, TargetPort, PartType, SourcePort;
    FString PartId; // Empty = adapter selects a compatible available part at SourcePort.
    FString RobotId; // Empty = unassigned; deleting a robot does not delete the task.
};

struct FFactoryTaskSpec
{
    FString Id, Label, Operation, TargetPort, PartType, SourcePort, Capability;
    int32 WheelIndex = INDEX_NONE;
};

struct FFactoryLayout;
namespace FactoryTasks
{
    const TArray<FFactoryTaskSpec>& Catalog();
    const FFactoryTaskSpec* Find(const FString& Id);
    bool Supports(const FString& EquipmentType, const FFactoryTaskSpec& Spec);
    void MigrateLegacy(FFactoryLayout& Layout);
    bool Validate(const FFactoryLayout& Layout, FString& Error, bool RequireAssigned=false);
}

// Sensor observations are distinct from simulation truth. A future camera provider
// must publish calibrated world poses and simulation timestamps, never perfect poses
// as a fallback on dropout. Units: cm, seconds; physical models use explicit SI fields.
struct FFactoryPartObservation
{
    FString PartId, PartType, SourcePort;
    FTransform MeasuredWorld;
    double SampleTimeSeconds=-1, Confidence=0;
    bool bValid=false, bDetected=false;
    bool Fresh(double Now, double MaxAge) const
    { return bValid && SampleTimeSeconds>=0 && Now>=SampleTimeSeconds && Now-SampleTimeSeconds<=MaxAge; }
};
class IFactoryPartObservationProvider
{
public:
    virtual ~IFactoryPartObservationProvider()=default;
    virtual FFactoryPartObservation ObservePart(const FString& PartId, double SimulationSeconds) const=0;
};

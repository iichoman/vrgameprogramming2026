#pragma once
#include "CoreMinimal.h"

/** Virtual observations only. The consumer never reads the test fault switches. */
struct FFactoryTransferReading
{
    bool Present=false, Gripped=false, Valid=false;
    FVector Position=FVector::ZeroVector;
    double SampledAt=-1, PoseAt=-1;
    bool Fresh(double Now) const
    {return Valid&&SampledAt>=0&&PoseAt>=0&&Now>=SampledAt&&Now>=PoseAt&&Now-SampledAt<=.25&&Now-PoseAt<=.25;}
};

struct FFactoryTransferSensors
{
    FFactoryTransferReading Reading;
    bool PublishPose=true, GripFault=false, GripClosed=false;
    double LastSample=-1;
    void Reset() {*this=FFactoryTransferSensors();}
    void Sample(double Now,bool Present,const FVector& ActualPart,const FVector& ActualTool)
    {
        if(!FMath::IsFinite(Now)||Now<0||ActualPart.ContainsNaN()||ActualTool.ContainsNaN())
        {Reading.Valid=false;return;}
        if(LastSample>=0&&Now-LastSample<.05-1.e-9)return;
        LastSample=Now;Reading.SampledAt=Now;Reading.Valid=true;Reading.Present=Present;
        Reading.Gripped=Present&&GripClosed&&!GripFault&&FVector::Distance(ActualPart,ActualTool)<=.2;
        if(PublishPose){Reading.Position=ActualPart;Reading.PoseAt=Now;}
    }
};

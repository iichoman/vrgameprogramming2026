#pragma once
#include "FactoryMotionController.h"

enum class EAssemblyQuality : uint8 { NotEvaluated, Pass, Fail };
struct FAssemblyOperationRequest
{
    FFactoryRequestId Id;
    FString ProductId, PartId, EquipmentId, TaskId, SourcePort, TargetPort;
};
struct FAssemblyOperationResult
{
    FAssemblyOperationRequest Request;
    EFactoryCommandStatus Control=EFactoryCommandStatus::Idle;
    EAssemblyQuality SupportQuality=EAssemblyQuality::NotEvaluated;
    EAssemblyQuality FasteningQuality=EAssemblyQuality::NotEvaluated;
    bool bModelValid=true;
    FString Reason;
};
// Per equipment, not a global scheduler. Reuses the existing motion request identity.
struct FAssemblyOperationLedger
{
    uint64 Epoch=1, LastSequence=0;
    bool bActive=false;
    FAssemblyOperationRequest Active;
    TArray<FAssemblyOperationResult> Results;
    bool Accept(const FAssemblyOperationRequest& R)
    {
        if(bActive||R.Id.Epoch!=Epoch||R.Id.Sequence<=LastSequence||R.ProductId.IsEmpty()
            ||R.PartId.IsEmpty()||R.EquipmentId.IsEmpty()||R.TaskId.IsEmpty()||R.SourcePort.IsEmpty()||R.TargetPort.IsEmpty())return false;
        Active=R;LastSequence=R.Id.Sequence;bActive=true;return true;
    }
    bool Finish(FFactoryRequestId Id,EFactoryCommandStatus Control,EAssemblyQuality Support,bool ModelValid,const FString& Reason)
    {
        if(!bActive||!(Active.Id==Id))return false;
        Results.Add({Active,Control,Support,EAssemblyQuality::NotEvaluated,ModelValid,Reason});bActive=false;return true;
    }
    void Reset()
    {
        if(bActive)Finish(Active.Id,EFactoryCommandStatus::Cancelled,EAssemblyQuality::NotEvaluated,true,TEXT("Reset"));
        ++Epoch;LastSequence=0; // Preserve prior results; late responses cannot finish a new request.
    }
};

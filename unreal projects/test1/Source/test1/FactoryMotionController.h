#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FactoryMotionController.generated.h"

struct FFactoryRequestId
{
    uint64 Epoch=0, Sequence=0;
    bool operator==(const FFactoryRequestId& Other) const {return Epoch==Other.Epoch&&Sequence==Other.Sequence;}
};
enum class EFactoryCommandStatus:uint8 {Idle,Running,Completed,Cancelled,Failed};
DECLARE_MULTICAST_DELEGATE_TwoParams(FFactoryMotionTerminal,FFactoryRequestId,EFactoryCommandStatus);

/** Device adapter. Step is called ONLY by the process fixed clock, never component Tick. */
UCLASS(NotBlueprintable)
class TEST1_API UFactoryMotionController : public UActorComponent
{
    GENERATED_BODY()
public:
    UFactoryMotionController();
    virtual bool Solve(const FTransform& WorldGoal,TArray<double>& Solution,const TArray<double>& Seed) const;
    virtual FTransform Forward(const TArray<double>& Angles) const;
    virtual TArray<double> Home() const;
    virtual void Limits(int32 Axis,double& Min,double& Max,double& Speed,double& Acceleration) const;
    bool RequestMove(FFactoryRequestId Id,const FTransform& WorldGoal,FString& Error);
    void Step(double Dt);
    void Cancel();
    void Reset(uint64 Epoch);
    FTransform Pose() const {return Forward(Q);}
    const TArray<double>& Joints() const {return Q;}
    EFactoryCommandStatus Status() const {return CommandStatus;}
    FFactoryRequestId RequestId() const {return ActiveId;}
    FFactoryMotionTerminal OnTerminal;
protected:
    TArray<double> Q,Velocity,Integral,Target;
    EFactoryCommandStatus CommandStatus=EFactoryCommandStatus::Idle;
    FFactoryRequestId ActiveId;
    uint64 CurrentEpoch=0,LastSequence=0;
    FTransform Goal;
    double Elapsed=0;
    void Terminal(EFactoryCommandStatus State);
};

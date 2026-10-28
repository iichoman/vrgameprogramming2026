#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FactoryRuntime.h"
#include "FactoryMotionController.h"
#include "FactoryProcessRunner.generated.h"
class UFactoryMotionController;

/** Extension point for native operations. Equipment adapters never decide production order. */
UCLASS(Abstract,NotBlueprintable)
class TEST1_API UFactoryProcessRunner : public UActorComponent
{
    GENERATED_BODY()
public:
    UFactoryProcessRunner(){PrimaryComponentTick.bCanEverTick=false;}
    virtual FName OperationType() const PURE_VIRTUAL(UFactoryProcessRunner::OperationType,return NAME_None;);
    virtual bool Validate(FString& Error) const PURE_VIRTUAL(UFactoryProcessRunner::Validate,return false;);
    virtual bool Start() PURE_VIRTUAL(UFactoryProcessRunner::Start,return false;);
    virtual void Stop() PURE_VIRTUAL(UFactoryProcessRunner::Stop,);
    virtual void Reset() PURE_VIRTUAL(UFactoryProcessRunner::Reset,);
    virtual void Step(double Dt) PURE_VIRTUAL(UFactoryProcessRunner::Step,);
};

UCLASS(NotBlueprintable)
class TEST1_API UFactoryWheelProcessRunner : public UFactoryProcessRunner
{
    GENERATED_BODY()
public:
    virtual FName OperationType() const override {return TEXT("wheel_transfer");}
    virtual bool Validate(FString& Error) const override;
    virtual bool Start() override;
    virtual void Stop() override;
    virtual void Reset() override;
    virtual void Step(double Dt) override;
private:
    AFactoryRuntime* Runtime() const;
    UFactoryMotionController* Controller() const;
    bool Move(const FVector& Position,double Yaw);
    void Change(EFactoryRunState State);
    FFactoryRequestId Pending;
    bool bHolding=false;
    double StateSeconds=0;
};

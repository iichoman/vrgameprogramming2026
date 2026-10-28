#pragma once
#include "CoreMinimal.h"
#include "FactoryEquipment.h"
#include "FactoryMotionController.h"
#include "FactoryRobotEquipment.generated.h"

/** Mesh-derived frame model. These are visual asset axes, not a calibrated industrial robot. */
class FFactoryRobotKinematics
{
public:
    static FTransform Rest(int32 Axis);
    static FTransform Tool();
    static FTransform Forward(const TArray<double>& Degrees);
    static bool Solve(const FTransform& Goal,TArray<double>& Out,const TArray<double>& Seed);
};
UCLASS(NotBlueprintable)
class TEST1_API UFactoryRobotController : public UFactoryMotionController
{
    GENERATED_BODY()
public:
    virtual bool Solve(const FTransform&,TArray<double>&,const TArray<double>&) const override;
    virtual FTransform Forward(const TArray<double>&) const override;
    virtual TArray<double> Home() const override;
    virtual void Limits(int32,double&,double&,double&,double&) const override;
};

UCLASS(NotBlueprintable)
class TEST1_API AFactoryRobotEquipment : public AFactoryEquipment
{
    GENERATED_BODY()
public:
    virtual void Configure(const FFactoryLayoutItem&,UStaticMesh* = nullptr) override;
    virtual void UpdateTool(const FVector&,double) override;
protected:
    virtual UFactoryMotionController* CreateController() override;
private:
    UPROPERTY() TArray<TObjectPtr<USceneComponent>> RobotNodes;
    UPROPERTY() TArray<TObjectPtr<USceneComponent>> JointNodes;
};

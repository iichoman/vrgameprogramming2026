#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FactoryRuntime.h"
#include "FactoryMotionController.h"
#include "FactoryTransferSensors.h"
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
    // Explicit, transient feature checks; no random scenario or layout schema change.
    bool StartFeatureCheck(int32 Kind);
    void RestoreFeatureInputs();
    void InterruptFeaturePose() {if(bSensorFeatures)Sensors.PublishPose=false;}
    void InterruptHubSensor() {if(bSensorFeatures)bPublishHub=false;}
    FString FeatureSummary() const;
    void DrawFeatureVisualization() const;
    FString VisualizationSummary() const;
    bool bShowSensors=true, bShowRoute=true;
    bool FeaturePassed() const;
    bool FeaturesEnabled() const {return bSensorFeatures;}
    int32 Replans=0, GripRetries=0, Recoveries=0;
private:
    AFactoryRuntime* Runtime() const;
    UFactoryMotionController* Controller() const;
    bool Move(const FVector& Position,double Yaw);
    bool Move(const FVector& Position,double Yaw,const TArray<double>& Seed);
    void Change(EFactoryRunState State);
    FFactoryRequestId Pending;
    bool bHolding=false;
    double StateSeconds=0;
    FFactoryTransferSensors Sensors;
    FVector ObservedPickup=FVector::ZeroVector;
    FVector ActiveGoal=FVector::ZeroVector;
    bool bSensorFeatures=false, bWaiting=false, bObservedFault=false;
    double SupplyDeliveryAt=-1;
    bool bReserveAvailable=true;
    friend class AFactoryRuntime; // Runtime checks can simulate an empty reserve.
    int32 FeatureKind=0, GripAttempts=0;
    FVector WheelVelocityMps = FVector::ZeroVector;
    double GripDemandN = 0;
    double GripCapacityN = 0;
    double SlipMm = 0;
    bool UpdatePhysicalGrip(FTransform& WheelPose, double Dt);
    double WaitSeconds=0, StableSeconds=0, GripSeconds=0;
    FString FeatureStatus=TEXT("미실행");
    bool CheckObservations(double Dt);
    bool AssessMount(double Dt);
    bool PlanApproach();
    TArray<TArray<double>> ArmPlan;
    bool bPublishHub=true;
    double HubSampleAt=-1, SeatSince=-1, LastDepth=0, ProgressAt=-1;
    FTransform MeasuredHub, MeasuredWheel;
    int32 Realignments=0;
    FString MountDecision;
    void FeatureFault(const FString& Reason);
};

/** Fixed-layout team cell adapter. Owns scheduling, not robot/sensor algorithms. */
UCLASS(NotBlueprintable)
class TEST1_API UFactoryAssemblyProcessRunner : public UFactoryProcessRunner
{
    GENERATED_BODY()
public:
    virtual FName OperationType() const override { return TEXT("assembly_line_test"); }
    virtual bool Validate(FString& Error) const override;
    virtual bool Start() override;
    virtual void Stop() override;
    virtual void Reset() override;
    virtual void Step(double Dt) override;
};

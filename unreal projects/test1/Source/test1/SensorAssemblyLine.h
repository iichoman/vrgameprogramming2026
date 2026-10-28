#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "IntegratedAssemblyLine.h"
#include "SensorAssemblyLine.generated.h"

/** Value is in cm or cm/s; booleans are encoded as X=0/1. */
USTRUCT()
struct FLineSensorReading
{
    GENERATED_BODY()
    UPROPERTY(VisibleAnywhere) FName SensorId;
    UPROPERTY(VisibleAnywhere) FVector Value = FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere) double MeasuredAt = -1;
    UPROPERTY(VisibleAnywhere) bool bValid = false;
    bool Fresh(double Now, double MaxAge) const { return bValid && MeasuredAt >= 0 && Now-MeasuredAt <= MaxAge; }
};

/** Only this component reads world truth to produce observations. No control commands. */
UCLASS()
class TEST1_API UArrivalSensorBank : public UActorComponent
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere) double SamplePeriod = .05;
    UPROPERTY(EditAnywhere) double MaxAge = .25;
    UPROPERTY(EditAnywhere) FVector ArrivalHalfExtentCm = FVector(.2,2,2);
    UPROPERTY(EditAnywhere) FVector ObservationHalfExtentCm = FVector(900,300,200);
    UPROPERTY(VisibleAnywhere) FLineSensorReading Occupancy;
    UPROPERTY(VisibleAnywhere) FLineSensorReading Arrival;
    UPROPERTY(VisibleAnywhere) FLineSensorReading Position;
    UPROPERTY(VisibleAnywhere) FLineSensorReading Speed;
    UPROPERTY() TObjectPtr<AActor> Vehicle;
    FTransform Station;
    bool bPublishPosition = true;
    void Reset(AActor* InVehicle, const FTransform& InStation);
    void Sample(double Now);
    bool Healthy(double Now) const;
private:
    double LastSample = -1;
    FVector PreviousPosition;
};

/** Kinematic conveyor plant. Faults change this plant, not the controller's decision. */
UCLASS()
class TEST1_API UConveyorPlant : public UActorComponent
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere) double CruiseSpeedCmS = 90;
    UPROPERTY(EditAnywhere) double AccelerationCmS2 = 180;
    UPROPERTY(VisibleAnywhere) FVector Velocity;
    UPROPERTY() TObjectPtr<AActor> Vehicle;
    UPROPERTY() TObjectPtr<AActor> Carriage;
    FVector Target;
    bool bJammed = false;
    void Reset(AActor* InVehicle, AActor* InCarriage, FVector InTarget);
    void CommandRun(bool Run) { bRun=Run; }
    void Step(double Seconds);
private:
    bool bRun = false;
};

UENUM(BlueprintType)
enum class EArrivalPhase : uint8 { Ready, Moving, Docking, Released, Stopped, Fault };
UENUM(BlueprintType)
enum class EArrivalDemo : uint8 { Normal, Jam, MisDock, PositionDropout, WheelMissing, WheelDisplaced, GripFailure, WheelPoseDropout };

UCLASS(NotBlueprintable)
class TEST1_API ASensorAssemblyLine : public AIntegratedAssemblyLine
{
    GENERATED_BODY()
public:
    ASensorAssemblyLine();
    UPROPERTY(VisibleAnywhere) TObjectPtr<UArrivalSensorBank> Sensors;
    UPROPERTY(VisibleAnywhere) TObjectPtr<UConveyorPlant> Conveyor;
    UPROPERTY(EditAnywhere) EArrivalDemo Demo = EArrivalDemo::Normal;
    UPROPERTY(VisibleAnywhere, Transient) EArrivalPhase ArrivalPhase = EArrivalPhase::Ready;
    UPROPERTY(EditAnywhere) double DockToleranceCm = .5;
    UPROPERTY(EditAnywhere) double StopSpeedCmS = .1;
    UPROPERTY(EditAnywhere) double StableSeconds = .5;
    UPROPERTY(EditAnywhere) double ArrivalTimeoutSeconds = 15;
    UPROPERTY(EditAnywhere) bool bShowSensorVisualization = true;
    UPROPERTY(VisibleAnywhere, Transient) double SensorTime = 0;
    UPROPERTY(VisibleAnywhere, Transient) FString EventLog;
    UFUNCTION() virtual bool SelectDemo(EArrivalDemo Selection);
    virtual bool ResetLine() override;
    virtual bool StartLine() override;
    virtual void StopLine() override;
    virtual bool AdvanceLine(double Seconds) override;
    virtual FString GetSummary() const override;
    virtual void Tick(float Seconds) override;
protected:
	virtual void BeforeSensorStep(double Dt) {}
	void Record(const FString& Kind, const FString& Message);
private:
    double Remainder=0, RunTime=0, StableTime=0, NoProgressTime=0;
    bool bInjected=false, bStopRecorded=false, bLoggedComplete=false;
	FString LastLeftRobotState;
    void SensorFault(const FString& Reason);
    void FixedStep(double Dt);
};

#pragma once

#include "SensorAssemblyLine.h"
#include "AutonomousAssemblyLine.generated.h"

UENUM(BlueprintType)
enum class ERandomWheelEvent : uint8 { SupplyMissing, PickupPoseDropout, GripFailure, WheelDisplaced };

/** Separate experiment map: seeded plant faults, sensor-driven left-arm recovery. */
UCLASS(NotBlueprintable)
class TEST1_API AAutonomousAssemblyLine : public ASensorAssemblyLine
{
	GENERATED_BODY()
public:
	UPROPERTY(EditAnywhere, Category="Autonomous Demo") int32 BaseSeed = 20261027;
	UPROPERTY(EditAnywhere, Category="Autonomous Demo", meta=(Units="s")) double MinRecoveryDelaySeconds = .7;
	UPROPERTY(EditAnywhere, Category="Autonomous Demo", meta=(Units="s")) double MaxRecoveryDelaySeconds = 1.1;
	UPROPERTY(VisibleAnywhere, Transient, Category="Autonomous Demo") int32 CurrentSeed = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Autonomous Demo") int32 RunNumber = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Autonomous Demo") int32 InjectedEvents = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Autonomous Demo") int32 RecoveredEvents = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Autonomous Demo") FString ActiveEvent;
	virtual bool SelectDemo(EArrivalDemo Selection) override;
	virtual bool ResetLine() override;
	virtual FString GetSummary() const override;
protected:
	virtual void BeforeSensorStep(double Dt) override;
private:
	FRandomStream Random;
	ERandomWheelEvent Planned[2];
	double FirstSeenAt[2] = {-1,-1};
	double TriggerAfter[2] = {0,0};
	double RestoreAt[2] = {-1,-1};
	bool bInjected[2] = {false,false};
	bool bRestored[2] = {false,false};
	void InjectEvent(int32 Index);
	void RestoreEvent(int32 Index);
};

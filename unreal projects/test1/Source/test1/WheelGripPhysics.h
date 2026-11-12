#pragma once
#include "CoreMinimal.h"

inline FVector StepWheelGrip(const FVector& Previous, const FVector& Target, double Dt, bool LowFriction, double MassKg,
    FVector& WheelVelocityMps, double& GripDemandN, double& GripCapacityN, double& SlipMm,
    double& AppliedGripForceN)
{
    constexpr double TotalPadForceN = 1200.0;
    const double Friction = LowFriction ? 0.08 : 0.65;
    GripCapacityN = TotalPadForceN * Friction;
    const FVector Gravity(0, 0, -9.81);
    const FVector TargetVelocity = (Target - Previous) / (100.0 * Dt);
    const FVector Required = MassKg * ((TargetVelocity - WheelVelocityMps) / Dt - Gravity);
    GripDemandN = Required.Size();
    const FVector AppliedForce = Required.GetClampedToMaxSize(GripCapacityN);
    AppliedGripForceN = AppliedForce.Size();
    WheelVelocityMps += (Gravity + AppliedForce / MassKg) * Dt;
    const FVector Actual = Previous + WheelVelocityMps * (100.0 * Dt);
    SlipMm = FVector::Distance(Target, Actual) * 10.0;
    return Actual;
}

#pragma once
#include "CoreMinimal.h"

/** Demo pad model. Values are teaching assumptions, not measured gripper data. */
namespace WheelGrip
{
    constexpr double TotalPadForceN = 1200.0;
    constexpr double NormalFriction = 0.65;
    constexpr double LowFriction = 0.08;
    /** Linear loss of pad friction from Normal to Low over DecaySeconds (assumed pad contamination). */
    inline double DegradedFriction(double SecondsSinceOnset, double DecaySeconds)
    {
        if (!FMath::IsFinite(SecondsSinceOnset) || SecondsSinceOnset <= 0.0) return NormalFriction;
        const double Alpha = DecaySeconds > 1.e-6 ? FMath::Clamp(SecondsSinceOnset / DecaySeconds, 0.0, 1.0) : 1.0;
        return FMath::Lerp(NormalFriction, LowFriction, Alpha);
    }
}

/** Friction is the pad friction coefficient for this step; callers choose the scenario. */
inline FVector StepWheelGrip(const FVector& Previous, const FVector& Target, double Dt, double Friction, double MassKg,
    FVector& WheelVelocityMps, double& GripDemandN, double& GripCapacityN, double& SlipMm,
    double& AppliedGripForceN)
{
    GripCapacityN = WheelGrip::TotalPadForceN * FMath::Max(0.0, Friction);
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

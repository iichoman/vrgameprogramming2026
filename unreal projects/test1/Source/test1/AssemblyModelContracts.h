#pragma once
#include "CoreMinimal.h"

// Independent recipe/inspection boundary. No scene or fault-injection access.
struct FAssemblyDimensionCriterion
{
    double NominalCm = 0;
    double ToleranceCm = 1;
    bool IsValid() const
    { return FMath::IsFinite(NominalCm) && NominalCm > 0 && FMath::IsFinite(ToleranceCm) && ToleranceCm >= 0; }
    bool Accepts(double MeasuredCm, bool bMeasurementValid) const
    { return IsValid() && bMeasurementValid && FMath::IsFinite(MeasuredCm) && MeasuredCm > 0 && FMath::Abs(MeasuredCm-NominalCm) <= ToleranceCm; }
};

namespace AssemblyRecipes
{
// Authored reference geometry, NOT measured production parts or OEM tolerances.
// process_vehicle_manifest.json: visible mesh bounds, local X, meters -> cm.
inline double CamryDoorWidthCm(const FString& TaskId)
{
    if(TaskId==TEXT("door_fl")) return 126.6232;
    if(TaskId==TEXT("door_fr")) return 126.5966;
    if(TaskId==TEXT("door_rl")) return 117.7864;
    if(TaskId==TEXT("door_rr")) return 117.7863;
    return 0; // Missing recipe must not silently calibrate from the inspected part.
}
}

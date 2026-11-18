#pragma once
#include "CoreMinimal.h"
#include "WheelContactSettings.generated.h"

// Part recipe and tool recipe are independent of robot placement and task order.
USTRUCT(BlueprintType)
struct FWheelContactProperties
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere) double MassKg=18;
    UPROPERTY(EditAnywhere) double OuterRadiusM=.33;
    UPROPERTY(EditAnywhere) double InnerRadiusM=.18;
    UPROPERTY(EditAnywhere) double WidthM=.20;
    UPROPERTY(EditAnywhere) double Friction=.65;
};
USTRUCT(BlueprintType)
struct FWheelJawSettings
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere) double HalfOpeningCm=10;
    UPROPERTY(EditAnywhere) double HalfPadSpacingCm=12;
    UPROPERTY(EditAnywhere) double PadRadiusM=.025;
    UPROPERTY(EditAnywhere) double TotalForceN=1200;
    UPROPERTY(EditAnywhere) double CloseSeconds=.08;
    UPROPERTY(EditAnywhere) double StiffnessNm=200000;
    UPROPERTY(EditAnywhere) double DampingNsM=450;
    UPROPERTY(EditAnywhere) double MaxStepSeconds=.0005;
};

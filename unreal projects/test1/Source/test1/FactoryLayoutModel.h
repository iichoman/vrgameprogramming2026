#pragma once
#include "CoreMinimal.h"

// Plain layout data, independent of rendered meshes and actors. All lengths are cm.
enum class EFactoryItemKind : uint8 { Zone, VehicleStation, WheelSupply, CartesianHandler, Conveyor, ArticulatedRobot };

struct FFactoryLayoutItem
{
    FString Id;
    FString TypeId; // Registry ID; empty legacy values resolve from Kind.
    FString NameKo;
    FString NameEn;
    FString Role = TEXT("work");
    EFactoryItemKind Kind = EFactoryItemKind::Zone;
    FVector2D Position = FVector2D::ZeroVector;
    FVector2D Size = FVector2D(600, 400);
    double Yaw = 0;
    FString EffectiveTypeId() const;
    FString DisplayName() const { return NameKo + TEXT(" (") + NameEn + TEXT(")"); }
    FTransform Transform(double Z = 0) const { return FTransform(FRotator(0, Yaw, 0), FVector(Position.X, Position.Y, Z)); }
};

struct FFactoryLayout
{
    FVector2D HallOrigin = FVector2D::ZeroVector; // left edge X, centre Y
    TArray<FString> ProcessOrder; // Explicit IDs, never inferred from positions
    double RouteBendCm = 120;
    double LengthCm = 7200;
    double WidthCm = 4000;
    TArray<FFactoryLayoutItem> Items;
    FString StationId = TEXT("station_1");
    FString SupplyId = TEXT("supply_1");
    FString HandlerId = TEXT("handler_1");
    static FFactoryLayout Default();
    static FFactoryLayout LegacyDefault();
    static FString KindName(EFactoryItemKind Kind);
    static bool ParseKind(const FString& Name, EFactoryItemKind& Out);
    bool Validate(FString& Error) const;
    FString ToJson() const;
    static bool FromJson(const FString& Json, FFactoryLayout& Out, FString& Error);
    FFactoryLayoutItem* Find(const FString& Id);
    const FFactoryLayoutItem* Find(const FString& Id) const;
    bool Contains(const FFactoryLayoutItem& Item) const;
    static bool Overlaps(const FFactoryLayoutItem& A, const FFactoryLayoutItem& B, double MarginCm = 0);
    static bool SegmentHits(const FVector& A, const FVector& B, const FFactoryLayoutItem& Obstacle, double InflateCm);
};

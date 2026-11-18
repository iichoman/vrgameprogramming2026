#pragma once
#include "CoreMinimal.h"
#include "AssemblyContactBody.h"
#include "WheelContactSettings.h"

// Two opposed finite rim pads. Closure generates normal force; friction and
// rigid-body inertia determine slip. No displacement threshold releases a pad.
struct FWheelContactGrip
{
    FAssemblyContactBody Body;
    FTransform Previous=FTransform::Identity;
    bool bInitialized=false;
    double Closure=0;
    void Reset(){*this=FWheelContactGrip();}
    bool Step(const FTransform& Goal,double Mass,double Mu,double Dt,FTransform& Actual)
    {
        FWheelContactProperties Part;Part.MassKg=Mass;Part.Friction=Mu;
        return Step(Goal,Part,FWheelJawSettings(),Dt,Actual);
    }
    bool Step(const FTransform& Goal,const FWheelContactProperties& Part,const FWheelJawSettings& Jaw,double Dt,FTransform& Actual)
    {
        const double Mass=Part.MassKg, Mu=Part.Friction;
        if(!FMath::IsFinite(Part.OuterRadiusM)||!FMath::IsFinite(Part.InnerRadiusM)||!FMath::IsFinite(Part.WidthM)
            ||Part.InnerRadiusM<0||Part.OuterRadiusM<=Part.InnerRadiusM||Part.WidthM<=0
            ||!FMath::IsFinite(Jaw.CloseSeconds)||Jaw.CloseSeconds<=0||!FMath::IsFinite(Jaw.TotalForceN)||Jaw.TotalForceN<=0
            ||!FMath::IsFinite(Jaw.HalfOpeningCm)||Jaw.HalfOpeningCm<=0||!FMath::IsFinite(Jaw.HalfPadSpacingCm)||Jaw.HalfPadSpacingCm<=0)return false;
        if(!bInitialized){Body.Pose=Goal;Previous=Goal;bInitialized=true;}
        Body.Mass=Mass;Body.MaxStep=Jaw.MaxStepSeconds;
        // Annular wheel approximation, outer radius .33 m, inner .18 m, width .20 m.
        const double Axial=.5*Mass*(FMath::Square(Part.OuterRadiusM)+FMath::Square(Part.InnerRadiusM));
        Body.Inertia=FVector(Axial/2+Mass*FMath::Square(Part.WidthM)/12,Axial,Axial/2+Mass*FMath::Square(Part.WidthM)/12);
        Closure=FMath::Min(1.,Closure+Dt/Jaw.CloseSeconds);
        TArray<FAssemblyContact> Pads;
        for(double Side:{-1.,1.})for(double Height:{-Jaw.HalfPadSpacingCm,Jaw.HalfPadSpacingCm})
        {
            FAssemblyContact C;C.bGrip=true;C.Region=TEXT("림 파지면");C.PointCm=FVector(0,Side*Jaw.HalfOpeningCm,Height);
            C.Normal=FVector(0,-Side,0);C.PreloadN=Jaw.TotalForceN/4*Closure;C.Stiffness=Jaw.StiffnessNm;C.Damping=Jaw.DampingNsM;C.Friction=Mu;C.RadiusM=Jaw.PadRadiusM;
            C.MaxNormalForceN=C.PreloadN;
            Pads.Add(C);
        }
        if(!Body.Step(Pads,Previous,Goal,Goal,Dt))return false;
        Previous=Goal;Actual=Body.Pose;return true;
    }
};

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

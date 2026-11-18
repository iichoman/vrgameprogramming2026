#pragma once
#include "CoreMinimal.h"

// SI forces/velocities; Unreal cm poses. Finite compliant contact patches, not FEM.
// A patch disappears geometrically at its edge: slip alarms never release it.
struct FAssemblyContact
{
    FString Region;
    FString CounterpartId;
    FVector PointCm=FVector::ZeroVector, Normal=FVector::UpVector;
    double RadiusM=.04, Stiffness=100000, Damping=500, Friction=.65, PreloadN=0;
    double YieldN=6000, DebrisM=0;
    double CounterpartYieldN=6000;
    // Zero means a passive, unlimited spring. Force-driven jaws provide a finite cap.
    double MaxNormalForceN=0;
    bool bBilateral=false, bGrip=false, bSeat=false, bAdhesive=false;
};
struct FAssemblyDamage
{
    FString PartId, Region;
    // Contact location in the damaged part, support, or gripper's own frame.
    FVector LocalPointCm=FVector::ZeroVector;
    FVector WorldPointCm=FVector::ZeroVector;
    bool bCounterpart=false;
    bool bGripCounterpart=false;
    double ElasticDeflectionMm=0;
    double PeakForceN=0, LimitN=0;
    FVector MarkerWorldPosition(const FTransform& Part,const FTransform& Support,const FTransform& Grip) const
    {return (bCounterpart?(bGripCounterpart?Grip:Support):Part).TransformPosition(LocalPointCm);}
};
struct FAssemblyContactBody
{
    FString PartId;
    FTransform Pose=FTransform::Identity;
    FVector Velocity=FVector::ZeroVector, Omega=FVector::ZeroVector;
    FVector Inertia=FVector(4), Force=FVector::ZeroVector, Torque=FVector::ZeroVector;
    FVector GripForce=FVector::ZeroVector, GripTorque=FVector::ZeroVector, SupportForce=FVector::ZeroVector;
    double Mass=25, Time=0, LastContactForceN=0, MaxStep=.0005;
    double GripFrictionCapacityN=0;
    double SeatInterferenceMm=0;
    int GripContacts=0, SupportContacts=0, SeatContacts=0;
    TArray<FAssemblyDamage> Damage;
    FVector PointVelocity(const FVector& Rm) const {return Velocity+FVector::CrossProduct(Omega,Rm);}
    FVector InverseInertia(const FVector& World) const
    {return Pose.GetRotation().RotateVector(Pose.GetRotation().UnrotateVector(World)/Inertia);}
    void ResolveImpact(const FVector& Normal,const FVector& R,double Friction=.5,double Restitution=.15)
    {
        const FVector N=Normal.GetSafeNormal();const double Vn=FVector::DotProduct(PointVelocity(R),N);
        if(Vn>=0)return;
        auto MassInverse=[&](const FVector& D){return 1/Mass+FVector::DotProduct(D,FVector::CrossProduct(InverseInertia(FVector::CrossProduct(R,D)),R));};
        auto Apply=[&](const FVector& J){Velocity+=J/Mass;Omega+=InverseInertia(FVector::CrossProduct(R,J));};
        const double Jn=-(1+Restitution)*Vn/MassInverse(N);Apply(N*Jn);
        const auto V=PointVelocity(R);const auto Tangent=V-N*FVector::DotProduct(V,N);
        if(!Tangent.IsNearlyZero())
        {const auto T=Tangent.GetSafeNormal();Apply(-T*FMath::Min(Friction*Jn,Tangent.Size()/MassInverse(T)));}
    }
    void RecordDamage(const FAssemblyContact& C,double Magnitude,bool Other=false,const FVector& At=FVector::ZeroVector)
    {
        const double Limit=Other?C.CounterpartYieldN:C.YieldN;
        const FString& Victim=Other?C.CounterpartId:PartId;
        if(Magnitude<=Limit||Victim.IsEmpty())return;
        auto* D=Damage.FindByPredicate([&](const auto& X){return X.Region==C.Region&&X.PartId==Victim;});
        if(!D){FAssemblyDamage New;New.PartId=Victim;New.Region=C.Region;New.LocalPointCm=C.PointCm;New.LimitN=Limit;New.bCounterpart=Other;Damage.Add(New);D=&Damage.Last();}
        if(Magnitude>D->PeakForceN)
        {
            D->PeakForceN=Magnitude;
            D->LocalPointCm=C.PointCm+(Other?C.Normal.GetSafeNormal()*(C.DebrisM*100):FVector::ZeroVector);
            D->bGripCounterpart=C.bGrip;D->WorldPointCm=At;
            D->ElasticDeflectionMm=1000*Magnitude/C.Stiffness;
        }
    }
    bool Step(const TArray<FAssemblyContact>& Contacts,const FTransform& PreviousGrip,const FTransform& Grip,
        const FTransform& Support,double Dt)
    {
        if(!FMath::IsFinite(Dt)||Dt<=0||Dt>1||!FMath::IsFinite(Mass)||Mass<=0||Inertia.ContainsNaN()||Inertia.GetMin()<=0
            ||!Pose.IsValid()||!Grip.IsValid()||!Support.IsValid()||!PreviousGrip.IsValid()||!FMath::IsFinite(MaxStep)||MaxStep<.0001||MaxStep>.002)return false;
        for(const auto& C:Contacts)
            if(C.PointCm.ContainsNaN()||C.Normal.ContainsNaN()||C.Normal.IsNearlyZero()
                ||!FMath::IsFinite(C.Stiffness)||C.Stiffness<=0||!FMath::IsFinite(C.Damping)||C.Damping<0
                ||!FMath::IsFinite(C.Friction)||C.Friction<0||!FMath::IsFinite(C.PreloadN)||C.PreloadN<0
                ||!FMath::IsFinite(C.RadiusM)||C.RadiusM<=0||!FMath::IsFinite(C.DebrisM)||!FMath::IsFinite(C.YieldN)||C.YieldN<=0
                ||!FMath::IsFinite(C.MaxNormalForceN)||C.MaxNormalForceN<0
                ||!FMath::IsFinite(C.CounterpartYieldN)||C.CounterpartYieldN<=0)return false;
        const int Steps=FMath::CeilToInt(Dt/MaxStep);const double H=Dt/Steps;
        for(int Step=0;Step<Steps;++Step)
        {
            const double Alpha=double(Step+1)/Steps;
            FTransform G(FQuat::Slerp(PreviousGrip.GetRotation(),Grip.GetRotation(),Alpha),FMath::Lerp(PreviousGrip.GetLocation(),Grip.GetLocation(),Alpha));
            Force=FVector(0,0,-9.81*Mass);Torque=GripForce=GripTorque=SupportForce=FVector::ZeroVector;
            GripContacts=SupportContacts=SeatContacts=0;LastContactForceN=GripFrictionCapacityN=SeatInterferenceMm=0;
            for(const auto& C:Contacts)
            {
                const auto& Frame=C.bGrip?G:Support;
                const FVector N=Frame.TransformVectorNoScale(C.Normal).GetSafeNormal();
                const FVector R=Pose.TransformVectorNoScale(C.PointCm)/100;
                const FVector Point=Pose.GetLocation()/100+R;
                const FVector Anchor=Frame.TransformPosition(C.PointCm)/100;
                const FVector Error=Point-Anchor;
                const double Gap=FVector::DotProduct(Error,N)-C.DebrisM;
                const FVector Tangent=Error-N*FVector::DotProduct(Error,N);
                if(Tangent.Size()>C.RadiusM || (C.bAdhesive?FMath::Abs(Gap)>.005:(!C.bBilateral&&Gap>C.PreloadN/C.Stiffness+.001)))continue;
                const FVector AnchorV=C.bGrip?(Grip.TransformPosition(C.PointCm)-PreviousGrip.TransformPosition(C.PointCm))/(100*Dt):FVector::ZeroVector;
                const FVector RelV=PointVelocity(R)-AnchorV;
                const double Vn=FVector::DotProduct(RelV,N);
                const double TrialNormal=-C.Stiffness*Gap-C.Damping*Vn;
                double NormalForce=C.bAdhesive?FMath::Clamp(TrialNormal,-C.PreloadN,C.PreloadN):C.bBilateral?TrialNormal:FMath::Max(0.,C.PreloadN+TrialNormal);
                if(C.MaxNormalForceN>0)NormalForce=FMath::Clamp(NormalForce,-C.MaxNormalForceN,C.MaxNormalForceN);
                if(!C.bBilateral&&!C.bAdhesive&&NormalForce<=0)continue;
                if(C.bSeat)
                {
                    ++SeatContacts;
                    // Ideal force + displacement probe: infer unmodeled thickness
                    // from spring compression and measured surface gap, not a fault flag.
                    const double Compression=(NormalForce+C.Damping*Vn)/C.Stiffness;
                    SeatInterferenceMm=FMath::Max(SeatInterferenceMm,FMath::Abs(Compression+FVector::DotProduct(Error,N))*1000);
                }
                const double NormalCapacity=C.bAdhesive?C.PreloadN:FMath::Abs(NormalForce)+(C.bBilateral?C.PreloadN:0);
                const FVector Tangential=(-C.Stiffness*Tangent-C.Damping*(RelV-N*Vn)).GetClampedToMaxSize(C.Friction*NormalCapacity);
                const FVector F=N*NormalForce+Tangential;
                const FVector M=FVector::CrossProduct(R,F);
                Force+=F;Torque+=M;LastContactForceN=FMath::Max(LastContactForceN,F.Size());RecordDamage(C,F.Size());
                if(!C.CounterpartId.IsEmpty())RecordDamage(C,F.Size(),true,Frame.TransformPosition(C.PointCm)+N*(C.DebrisM*100));
                if(C.bGrip){++GripContacts;GripForce+=F;GripTorque+=M;GripFrictionCapacityN+=C.Friction*NormalCapacity;}else{++SupportContacts;SupportForce+=F;}
            }
            Velocity+=Force*(H/Mass);
            const FVector LocalW=Pose.GetRotation().UnrotateVector(Omega);
            const FVector AngularMomentum=Pose.GetRotation().RotateVector(Inertia*LocalW);
            Omega+=InverseInertia(Torque-FVector::CrossProduct(Omega,AngularMomentum))*H;
            Pose.AddToTranslation(Velocity*(100*H));
            const double Speed=Omega.Size();if(Speed>1.e-10)Pose.SetRotation((FQuat(Omega/Speed,Speed*H)*Pose.GetRotation()).GetNormalized());
            Time+=H;
            if(!Pose.IsValid()||Velocity.ContainsNaN()||Omega.ContainsNaN())return false;
        }
        return true;
    }
};

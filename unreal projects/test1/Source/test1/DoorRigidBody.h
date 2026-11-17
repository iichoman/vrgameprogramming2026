#pragma once
#include "CoreMinimal.h"
#include "RobotControlMath.h"
#include "DoorRigidBody.generated.h"

// Reduced 6-DOF rigid body. SI, isotropic inertia approximation. Forces act at COM;
// clamp moment is bounded separately. No claim of mesh contact, plate FEM or bolts.
USTRUCT()
struct FDoorRigidBodySettings
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere, Category="가정 물성", meta=(Units="kg")) double MassKg=25;
    UPROPERTY(EditAnywhere, Category="가정 물성") double InertiaKgM2=4;
    UPROPERTY(EditAnywhere, Category="그리퍼") double GripNormalN=2400;
    UPROPERTY(EditAnywhere, Category="그리퍼") double Friction=0.65;
    UPROPERTY(EditAnywhere, Category="그리퍼") double GripMomentNm=100;
    UPROPERTY(EditAnywhere, Category="그리퍼") double GripStiffnessNm=80000;
    UPROPERTY(EditAnywhere, Category="그리퍼") double GripAngularStiffness=3000;
    UPROPERTY(EditAnywhere, Category="받침") double SupportStiffnessNm=300000;
    UPROPERTY(EditAnywhere, Category="받침") double SupportMomentNm=300;
    bool Valid() const
    {
        for(double V:{MassKg,InertiaKgM2,GripNormalN,Friction,GripMomentNm,GripStiffnessNm,GripAngularStiffness,SupportStiffnessNm,SupportMomentNm})
            if(!FMath::IsFinite(V)||V<=0)return false;
        return true;
    }
};
struct FDoorRigidBody
{
    FTransform Pose=FTransform::Identity;
    FVector VelocityMps=FVector::ZeroVector, AngularVelocity=FVector::ZeroVector;
    FVector GripForceN=FVector::ZeroVector, GripTorqueNm=FVector::ZeroVector, SupportForceN=FVector::ZeroVector;
    bool bActive=false, bSupportContact=false, bValid=true;
    double SlipMm=0, SupportDeflectionMm=0;
    bool Step(const FDoorRigidBodySettings& S,const FTransform& GripGoal,const FTransform& SupportGoal,
        bool GripClosed,bool SupportClosed,double Dt)
    {
        if(!S.Valid()||!FMath::IsFinite(Dt)||Dt<=0||Dt>0.1||!Pose.IsValid()||!GripGoal.IsValid()||!SupportGoal.IsValid())return bValid=false;
        // Substeps are part of the plant, independent of render frames.
        const int N=FMath::CeilToInt(Dt/0.001);const double H=Dt/N;
        for(int I=0;I<N;++I)
        {
            GripForceN=GripTorqueNm=SupportForceN=FVector::ZeroVector;
            FVector Torque=FVector::ZeroVector;
            const FVector GripError=(GripGoal.GetLocation()-Pose.GetLocation())/100;
            if(GripClosed)
            {
                GripForceN=(S.GripStiffnessNm*GripError-2*FMath::Sqrt(S.MassKg*S.GripStiffnessNm)*VelocityMps).GetClampedToMaxSize(S.GripNormalN*S.Friction);
                GripTorqueNm=(S.GripAngularStiffness*FRobotKinematics::RotationError(GripGoal.GetRotation(),Pose.GetRotation())
                    -2*FMath::Sqrt(S.InertiaKgM2*S.GripAngularStiffness)*AngularVelocity).GetClampedToMaxSize(S.GripMomentNm);
                Torque+=GripTorqueNm;
            }
            const FVector SupportError=(SupportGoal.GetLocation()-Pose.GetLocation())/100;
            // A closed support can engage only inside its 10 mm capture envelope.
            bSupportContact=SupportClosed&&(bSupportContact||SupportError.Size()<=0.01);
            if(bSupportContact)
            {
                SupportForceN=S.SupportStiffnessNm*SupportError-2*FMath::Sqrt(S.MassKg*S.SupportStiffnessNm)*VelocityMps;
                Torque+=(S.GripAngularStiffness*FRobotKinematics::RotationError(SupportGoal.GetRotation(),Pose.GetRotation())
                    -2*FMath::Sqrt(S.InertiaKgM2*S.GripAngularStiffness)*AngularVelocity).GetClampedToMaxSize(S.SupportMomentNm);
            }
            VelocityMps+=(FVector(0,0,-9.81)+(GripForceN+SupportForceN)/S.MassKg)*H;
            AngularVelocity+=Torque*(H/S.InertiaKgM2);
            Pose.AddToTranslation(VelocityMps*(100*H));
            const double Speed=AngularVelocity.Size();
            if(Speed>1.e-12)Pose.SetRotation((FQuat(AngularVelocity/Speed,Speed*H)*Pose.GetRotation()).GetNormalized());
        }
        SlipMm=FVector::Distance(Pose.GetLocation(),GripGoal.GetLocation())*10;
        SupportDeflectionMm=FVector::Distance(Pose.GetLocation(),SupportGoal.GetLocation())*10;
        return bValid=Pose.IsValid()&&!VelocityMps.ContainsNaN()&&!AngularVelocity.ContainsNaN();
    }
};

#pragma once
#include "RobotControlMath.h"

// Diagonal joint inertia approximation. Link gravity/coupling are not modeled.
struct FRobotTorquePlant
{
    static bool Step(const FRobotJointDefinition& J,double Goal,double GoalRate,double Inertia,
        double LimitNm,double ExternalNm,double Dt,FRobotJointState& Q,double& DriveNm)
    {
        if(!FMath::IsFinite(Inertia)||Inertia<=0||!FMath::IsFinite(LimitNm)||LimitNm<=0
            ||!FMath::IsFinite(ExternalNm)||!FMath::IsFinite(Goal)||!FMath::IsFinite(GoalRate)
            ||!FMath::IsFinite(Dt)||Dt<=0||Dt>0.02)return false;
        // Ideal measured-load compensation; bounded drive can still saturate.
        const double K=Inertia*100, D=Inertia*20;
        const double Error=Goal-Q.AngleRadians;
        const double Candidate=FMath::Clamp(Q.IntegralErrorSeconds+Error*Dt,-0.4,0.4);
        const double Raw=K*Error+D*(GoalRate-Q.VelocityRadians)+Inertia*5*Candidate-ExternalNm;
        if(FMath::Abs(Raw)<=LimitNm||Raw*Error<=0)Q.IntegralErrorSeconds=Candidate;
        DriveNm=FMath::Clamp(K*Error+D*(GoalRate-Q.VelocityRadians)+Inertia*5*Q.IntegralErrorSeconds-ExternalNm,-LimitNm,LimitNm);
        Q.VelocityRadians+=(DriveNm+ExternalNm)/Inertia*Dt;
        Q.AngleRadians+=Q.VelocityRadians*Dt;
        if(Q.AngleRadians<J.MinRadians){Q.AngleRadians=J.MinRadians;Q.VelocityRadians=FMath::Max(0.0,Q.VelocityRadians);}
        if(Q.AngleRadians>J.MaxRadians){Q.AngleRadians=J.MaxRadians;Q.VelocityRadians=FMath::Min(0.0,Q.VelocityRadians);}
        return FMath::IsFinite(Q.AngleRadians)&&FMath::IsFinite(Q.VelocityRadians);
    }
};

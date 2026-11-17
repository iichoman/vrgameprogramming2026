#pragma once
#include "RobotMotionPlanner.h"

/** A continuous reference sampled by the PID; knots are not stop commands. */
struct FRobotMotionTrajectory
{
    TArray<TArray<double>> Samples;
    FTransform StartTCP,GoalTCP;
    bool bLinear=false;
    double Duration=0;

    void SetDuration(const TArray<FRobotJointDefinition>& Joints)
    {
        Duration=.25;
        const int N=Samples.Num()-1;
        for(int K=0;K<N;++K)for(int J=0;J<Joints.Num();++J)
        {
            const double D=FMath::Abs(Samples[K+1][J]-Samples[K][J])*N;
            Duration=FMath::Max(Duration,1.875*D/Joints[J].MaxVelocityRadians);
            Duration=FMath::Max(Duration,FMath::Sqrt(6*D/Joints[J].MaxAccelerationRadians));
            if(K>0)
            {
                const double Bend=FMath::Abs(Samples[K+1][J]-2*Samples[K][J]+Samples[K-1][J])*N*N;
                Duration=FMath::Max(Duration,FMath::Sqrt(3.516*Bend/Joints[J].MaxAccelerationRadians));
            }
        }
        if(bLinear)Duration=FMath::Max(Duration,1.875*FVector::Distance(StartTCP.GetLocation(),GoalTCP.GetLocation())/10.);
    }
    void Reference(double Time,TArray<double>& Q,TArray<double>& Velocity) const
    {
        const double S=FMath::Clamp(Time/Duration,0.,1.);
        const double Blend=S*S*S*(10+S*(-15+6*S));
        const double Rate=S<1?30*S*S*(1-S)*(1-S)/Duration:0;
        const int N=Samples.Num()-1,K=FMath::Min(FMath::FloorToInt(Blend*N),N-1);
        const double Alpha=Blend*N-K;
        Q.SetNum(Samples[0].Num());Velocity.SetNum(Q.Num());
        for(int J=0;J<Q.Num();++J)
        {
            Q[J]=FMath::Lerp(Samples[K][J],Samples[K+1][J],Alpha);
            Velocity[J]=(Samples[K+1][J]-Samples[K][J])*N*Rate;
        }
    }
    bool InCorridor(const FTransform& Pose,double PositionCm=.2,double AngleDeg=.5) const
    {
        if(!bLinear)return true;
        const FVector A=StartTCP.GetLocation(),D=GoalTCP.GetLocation()-A;
        const double T=D.SizeSquared()>1.e-10?FMath::Clamp(FVector::DotProduct(Pose.GetLocation()-A,D)/D.SizeSquared(),0.,1.):0;
        return FVector::Distance(Pose.GetLocation(),A+D*T)<=PositionCm
            &&FMath::RadiansToDegrees(Pose.GetRotation().AngularDistance(GoalTCP.GetRotation()))<=AngleDeg;
    }
};

/** Per-arm planning state. Advanced on the game thread in small work slices. */
struct FRobotMotionRequest
{
    enum class EPhase { Seeds, Build, Check, Preview };
    EPhase Phase=EPhase::Seeds;
    FTransform Base,From,Goal;
    TArray<double> Start;
    TArray<TArray<double>> Seeds,Goals;
    TArray<FTransform> Stops;
    FRobotMotionTrajectory Trial;
    int SeedIndex=0,GoalIndex=0,Route=-1,Stop=0,Sample=1,Steps=0,Edge=0,EdgeSample=0,EdgeSteps=0,Scale=0;
    bool bLinear=false,bPreferStraight=false,bJointOnly=false;
    double PreviewTime=0,NominalDuration=0,ElapsedPlanning=0;
    TArray<FRobotJointState> PreviewState;
    FString Failure;
};

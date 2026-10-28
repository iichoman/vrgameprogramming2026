#include "FactoryMotionController.h"
#include "GameFramework/Actor.h"

UFactoryMotionController::UFactoryMotionController(){PrimaryComponentTick.bCanEverTick=false;}
TArray<double> UFactoryMotionController::Home() const{return {0,0,250,0};}
bool UFactoryMotionController::Solve(const FTransform& World,TArray<double>& Out,const TArray<double>&) const
{
    if(!GetOwner()||World.ContainsNaN())return false;
    const auto Local=World.GetRelativeTransform(GetOwner()->GetActorTransform());const FVector P=Local.GetLocation();
    if(FMath::Abs(P.X)>650||FMath::Abs(P.Y)>650||P.Z<30||P.Z>350)return false;
    Out={P.X,P.Y,P.Z,Local.Rotator().Yaw};return true;
}
FTransform UFactoryMotionController::Forward(const TArray<double>& A) const
{return A.Num()==4&&GetOwner()?FTransform(FRotator(0,A[3],0),FVector(A[0],A[1],A[2]))*GetOwner()->GetActorTransform():FTransform::Identity;}
void UFactoryMotionController::Limits(int32 Axis,double& Min,double& Max,double& Speed,double& Acc) const
{Min=Axis<2?-650:(Axis==2?30:-720);Max=Axis<2?650:(Axis==2?350:720);Speed=Axis==3?90:120;Acc=Axis==3?180:240;}
bool UFactoryMotionController::RequestMove(FFactoryRequestId Id,const FTransform& World,FString& Error)
{
    if(CommandStatus==EFactoryCommandStatus::Running){Error=TEXT("설비가 이전 명령을 실행 중입니다.");return false;}
    if(Id.Epoch!=CurrentEpoch||Id.Sequence<=LastSequence){Error=TEXT("오래되거나 중복된 명령입니다.");return false;}
    TArray<double> Solution;if(!Solve(World,Solution,Q)){Error=TEXT("목표 자세에 도달할 수 없습니다. 로봇·공급대 위치를 조정하세요.");return false;}
    if(Q.Num()!=Solution.Num()){Error=TEXT("관절 구성이 일치하지 않습니다.");return false;}
    ActiveId=Id;LastSequence=Id.Sequence;Target=MoveTemp(Solution);Goal=World;Elapsed=0;Integral.Init(0,Q.Num());CommandStatus=EFactoryCommandStatus::Running;return true;
}
void UFactoryMotionController::Terminal(EFactoryCommandStatus State)
{if(CommandStatus!=EFactoryCommandStatus::Running)return;CommandStatus=State;OnTerminal.Broadcast(ActiveId,State);}
void UFactoryMotionController::Cancel(){Terminal(EFactoryCommandStatus::Cancelled);Velocity.Init(0,Q.Num());Integral.Init(0,Q.Num());}
void UFactoryMotionController::Reset(uint64 Epoch)
{Cancel();CurrentEpoch=Epoch;LastSequence=0;Q=Home();Target=Q;Velocity.Init(0,Q.Num());Integral.Init(0,Q.Num());CommandStatus=EFactoryCommandStatus::Idle;}
void UFactoryMotionController::Step(double Dt)
{
    if(CommandStatus!=EFactoryCommandStatus::Running||!FMath::IsFinite(Dt)||Dt<=0||Dt>.05)return;
    Elapsed+=Dt;bool Settled=true;
    for(int32 Axis=0;Axis<Q.Num();++Axis)
    {
        double Lo,Hi,Speed,Acc;Limits(Axis,Lo,Hi,Speed,Acc);
        const double E=Target[Axis]-Q[Axis];
        // This ideal acceleration plant has no static load. Bleed accumulated integral
        // near the target instead of retaining a false load compensation after saturation.
        if(FMath::Abs(E)<.1)Integral[Axis]*=FMath::Exp(-4*Dt);
        const double Candidate=FMath::Clamp(Integral[Axis]+E*Dt,-5.,5.);
        const double Raw=36*E+.1*Candidate-12*Velocity[Axis];
        if(FMath::Abs(Raw)<Acc||E*Integral[Axis]<0)Integral[Axis]=Candidate;
        Velocity[Axis]=FMath::Clamp(Velocity[Axis]+FMath::Clamp(36*E+.1*Integral[Axis]-12*Velocity[Axis],-Acc,Acc)*Dt,-Speed,Speed);
        const double Proposed=Q[Axis]+Velocity[Axis]*Dt;Q[Axis]=FMath::Clamp(Proposed,Lo,Hi);
        if(Q[Axis]!=Proposed)Velocity[Axis]=0;
        Settled&=FMath::Abs(Target[Axis]-Q[Axis])<.002&&FMath::Abs(Velocity[Axis])<.01;
    }
    if(Settled)
    {
        const FTransform Actual=Pose();
        if(FVector::Distance(Actual.GetLocation(),Goal.GetLocation())<.05&&FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Goal.GetRotation()))<.25)
            Terminal(EFactoryCommandStatus::Completed);
    }
    if(Elapsed>40)Terminal(EFactoryCommandStatus::Failed);
}

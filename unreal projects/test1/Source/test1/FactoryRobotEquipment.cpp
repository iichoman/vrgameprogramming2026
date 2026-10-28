#include "FactoryRobotEquipment.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

FTransform FFactoryRobotKinematics::Rest(int32 I)
{
    // Extracted read-only from BP_KUKA_Robot_InputFixed, 2026-10-27. Joint axes are local Z.
    const FTransform R[]={
        FTransform(FQuat(0,0,1,0),FVector(0,0,86.7)),
        FTransform(FQuat(-.7065533996,.0279708405,-.0279708039,.7065532958),FVector(-74.7652893,5.9288611,38)),
        FTransform(FQuat::Identity,FVector(0,-125,0)),
        FTransform(FQuat(0,-.7071067812,0,.7071067812),FVector(-135.8000336,5.5000172,0)),
        FTransform(FQuat(0,.7071067812,0,.7071067812),FVector(0,0,54.2000275)),
        FTransform(FQuat(.7065532804,-.0279708114,.0279707771,.7065534172),FVector(0,22.9999905,0))};
    return R[FMath::Clamp(I,0,5)];
}
FTransform FFactoryRobotKinematics::Tool()
{
    // Wheel centre is held 25cm from flange, with wheel local Y along gripper approach.
    return FTransform(FRotator(90,0,0),FVector(0,0,-25));
}
FTransform FFactoryRobotKinematics::Forward(const TArray<double>& Q)
{
    if(Q.Num()!=6)return FTransform::Identity;FTransform T=FTransform::Identity;
    for(int32 I=0;I<6;++I)T=FTransform(FRotator(0,Q[I],0))*Rest(I)*T;
    return Tool()*T;
}
namespace
{
void PoseError(const FTransform& Goal,const FTransform& Current,double* E)
{
    const FVector P=Goal.GetLocation()-Current.GetLocation();E[0]=P.X;E[1]=P.Y;E[2]=P.Z;
    FQuat D=Goal.GetRotation()*Current.GetRotation().Inverse();D.Normalize();if(D.W<0)D=D*-1.;
    E[3]=D.X*200;E[4]=D.Y*200;E[5]=D.Z*200;
}
bool LinearSolve(double A[6][7],double* X)
{
    for(int32 I=0;I<6;++I)
    {
        int32 Pivot=I;for(int32 R=I+1;R<6;++R)if(FMath::Abs(A[R][I])>FMath::Abs(A[Pivot][I]))Pivot=R;
        if(FMath::Abs(A[Pivot][I])<1e-12)return false;
        for(int32 C=I;C<7;++C)Swap(A[I][C],A[Pivot][C]);
        const double D=A[I][I];for(int32 C=I;C<7;++C)A[I][C]/=D;
        for(int32 R=0;R<6;++R)if(R!=I){const double F=A[R][I];for(int32 C=I;C<7;++C)A[R][C]-=F*A[I][C];}
    }
    for(int32 I=0;I<6;++I)X[I]=A[I][6];return true;
}
}
bool FFactoryRobotKinematics::Solve(const FTransform& Goal,TArray<double>& Out,const TArray<double>& Seed)
{
    if(Goal.ContainsNaN()||Goal.GetLocation().Size()>550)return false;
    TArray<TArray<double>> Seeds;if(Seed.Num()==6)Seeds.Add(Seed);
    for(double A:{0.,90.,-90.,170.,-170.})for(double B:{-45.,45.})Seeds.Add({A,B,-B,0,40,0});
    double Best=1e30;
    for(TArray<double> Q:Seeds)
    {
        for(int32 Iter=0;Iter<240;++Iter)
        {
            const FTransform Current=Forward(Q);double E[6];PoseError(Goal,Current,E);
            double Norm=0;for(double V:E)Norm+=V*V;
            if(Norm<Best){Best=Norm;Out=Q;}
            if(FVector::Distance(Goal.GetLocation(),Current.GetLocation())<.015&&Goal.GetRotation().AngularDistance(Current.GetRotation())<.0005)return true;
            double J[6][6];
            for(int32 Axis=0;Axis<6;++Axis)
            {
                auto Perturbed=Q;Perturbed[Axis]+=.1;double D[6];PoseError(Forward(Perturbed),Current,D);
                for(int32 Row=0;Row<6;++Row)J[Row][Axis]=D[Row]/.1;
            }
            double A[6][7]={};
            for(int32 R=0;R<6;++R){for(int32 C=0;C<6;++C){for(int32 K=0;K<6;++K)A[R][C]+=J[K][R]*J[K][C];if(R==C)A[R][C]+=.08;}
                for(int32 K=0;K<6;++K)A[R][6]+=J[K][R]*E[K];}
            double Step[6];if(!LinearSolve(A,Step))break;
            for(int32 I=0;I<6;++I)Q[I]=FMath::Clamp(Q[I]+FMath::Clamp(Step[I],-7.,7.),I==5?-350.:-175.,I==5?350.:175.);
        }
    }
    return false;
}
bool UFactoryRobotController::Solve(const FTransform& WorldGoal,TArray<double>& Out,const TArray<double>& Seed) const
{return GetOwner()&&FFactoryRobotKinematics::Solve(WorldGoal.GetRelativeTransform(GetOwner()->GetActorTransform()),Out,Seed);}
FTransform UFactoryRobotController::Forward(const TArray<double>& Angles) const
{return GetOwner()?FFactoryRobotKinematics::Forward(Angles)*GetOwner()->GetActorTransform():FTransform::Identity;}
TArray<double> UFactoryRobotController::Home() const{return {0,-35,65,0,-30,0};}
void UFactoryRobotController::Limits(int32 Axis,double& Min,double& Max,double& Speed,double& Acc) const
{Min=Axis==5?-350:-175;Max=-Min;Speed=45;Acc=90;}
UFactoryMotionController* AFactoryRobotEquipment::CreateController(){return NewObject<UFactoryRobotController>(this);}

void AFactoryRobotEquipment::Configure(const FFactoryLayoutItem& Item,UStaticMesh* Replacement)
{
    for(int32 I=RobotNodes.Num()-1;I>=0;--I)if(IsValid(RobotNodes[I])){RemoveInstanceComponent(RobotNodes[I]);RobotNodes[I]->DestroyComponent();}
    RobotNodes.Reset();JointNodes.Reset();Super::Configure(Item,nullptr);
    auto Node=[&](USceneComponent* Parent,FTransform T){auto* C=NewObject<USceneComponent>(this);AddInstanceComponent(C);C->SetupAttachment(Parent);
        C->SetMobility(EComponentMobility::Movable);C->SetRelativeTransform(T);C->RegisterComponent();RobotNodes.Add(C);return C;};
    auto Mesh=[&](USceneComponent* Parent,const FString& Name)
    {
        auto* M=LoadObject<UStaticMesh>(nullptr,*(TEXT("/Game/Robot/Meshes/")+Name));if(!M)return;
        auto* C=NewObject<UStaticMeshComponent>(this);AddInstanceComponent(C);C->SetupAttachment(Parent);C->SetMobility(EComponentMobility::Movable);
        C->SetStaticMesh(M);C->SetCollisionEnabled(ECollisionEnabled::NoCollision);C->SetCastShadow(false);C->SetVisibleInRayTracing(false);C->SetAffectDistanceFieldLighting(false);
        C->RegisterComponent();RobotNodes.Add(C);
    };
    auto* Base=Node(VisualRoot,FTransform(FVector(0,0,59.2)));Mesh(Base,TEXT("SM_Pedestal"));Mesh(Base,TEXT("SM_Base"));
    USceneComponent* Parent=VisualRoot;
    for(int32 I=0;I<6;++I){auto* RestNode=Node(Parent,FFactoryRobotKinematics::Rest(I));auto* Joint=Node(RestNode,FTransform::Identity);
        JointNodes.Add(Joint);Mesh(Joint,FString::Printf(TEXT("SM_A%d"),I+1));Parent=Joint;}
    const TCHAR* Extra[]={TEXT("SM_REVIEW_A1_Body9"),TEXT("SM_REVIEW_A1_Body13"),TEXT("SM_REVIEW_A2_Body1"),TEXT("SM_REVIEW_A3_Body8"),TEXT("SM_REVIEW_A3_Body11"),TEXT("SM_REVIEW_A3_Body14")};
    for(int32 I=0;I<6;++I)Mesh(JointNodes[I<2?0:(I==2?1:2)],Extra[I]);
    auto* Grip=Node(Parent,FFactoryRobotKinematics::Tool());
    // Replaceable simple tyre support; no automatic force or fastening is implied.
    for(double X:{-26.,26.})
    {auto* Finger=Box(TEXT("WheelGripper"),FVector::ZeroVector,FVector(8,18,68),FLinearColor(.14,.18,.23));
        Finger->AttachToComponent(Grip,FAttachmentTransformRules::KeepRelativeTransform);Finger->SetRelativeLocation(FVector(X,0,0));}
    UpdateTool(FVector::ZeroVector,0);
}
void AFactoryRobotEquipment::UpdateTool(const FVector&,double)
{
    if(!MotionController)return;const auto& Q=MotionController->Joints();
    for(int32 I=0;I<JointNodes.Num()&&I<Q.Num();++I)JointNodes[I]->SetRelativeRotation(FRotator(0,Q[I],0));
}

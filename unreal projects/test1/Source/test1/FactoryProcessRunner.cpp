#include "FactoryProcessRunner.h"
#include "FactoryEquipment.h"
#include "Components/SceneComponent.h"

AFactoryRuntime* UFactoryWheelProcessRunner::Runtime() const{return Cast<AFactoryRuntime>(GetOwner());}
UFactoryMotionController* UFactoryWheelProcessRunner::Controller() const
{auto* R=Runtime();auto* H=R?R->Equipment.FindRef(R->Layout.HandlerId).Get():nullptr;return H?H->Controller():nullptr;}
bool UFactoryWheelProcessRunner::Validate(FString& Error) const
{
    auto* R=Runtime();auto* C=Controller();if(!R||!C){Error=TEXT("휠 작업을 지원하는 로봇·이송장치를 연결하세요.");return false;}
    const FVector Supply=R->SupplyWorld(),Hub=R->HubWorld();const FVector Outside=Hub+FRotator(0,R->VehicleYaw(),0).RotateVector(FVector(0,-100,0));
    const TArray<FVector> P={C->Forward(C->Home()).GetLocation(),Supply+FVector(0,0,80),Supply,FVector(Supply.X,Supply.Y,250),FVector(Outside.X,Outside.Y,250),Outside,Hub};
    TArray<double> Seed=C->Home(),Solution;
    for(int32 I=0;I<P.Num();++I)
    {
        if(I>0&&!C->Solve(FTransform(FRotator(0,I<4?R->SupplyYaw():R->VehicleYaw(),0),P[I]),Solution,Seed))
        {Error=FString::Printf(TEXT("로봇이 %d번 작업 자세에 도달할 수 없습니다. 공급대·로봇 위치를 조정하세요."),I);return false;}
        if(I>0)Seed=Solution;
        if(P[I].X<R->Layout.HallOrigin.X+36||P[I].X>R->Layout.HallOrigin.X+R->Layout.LengthCm-36||FMath::Abs(P[I].Y-R->Layout.HallOrigin.Y)>R->Layout.WidthCm*.5-36)
        {Error=TEXT("휠 이동 경로가 공장 경계를 벗어납니다.");return false;}
        FString Obstacle;if(I>0&&R->Obstructed(P[I-1],P[I],Obstacle)){Error=TEXT("휠 이동 경로 간섭: ")+Obstacle;return false;}
    }
    return true;
}
void UFactoryWheelProcessRunner::Change(EFactoryRunState State){Runtime()->RunState=State;StateSeconds=0;}
bool UFactoryWheelProcessRunner::Move(const FVector& Position,double Yaw)
{
    auto* R=Runtime();auto* C=Controller();FString Error;
    Pending={R->RunEpoch,++R->NextRequestSequence};
    if(!C||!C->RequestMove(Pending,FTransform(FRotator(0,Yaw,0),Position),Error))
    {R->RunState=EFactoryRunState::Fault;R->Status=Error;return false;}
    return true;
}
bool UFactoryWheelProcessRunner::Start()
{
    auto* R=Runtime();if(!R||!R->CanEdit())return false;R->CancelEdit();FString Error;
    if(!R->ValidateRun(Error)){R->Status=Error;return false;}
    R->SimulationSeconds=R->TravelCm=R->AlignmentMm=R->SimRemainder=0;bHolding=false;
    ++R->RunEpoch;Controller()->Reset(R->RunEpoch);Change(EFactoryRunState::Approach);
    R->Status=TEXT("실행 중: 현재 배치의 공급점과 허브를 기준으로 휠을 운반합니다.");
    return Move(R->SupplyWorld()+FVector(0,0,80),R->SupplyYaw());
}
void UFactoryWheelProcessRunner::Stop()
{
    auto* R=Runtime();if(!R||R->RunState<EFactoryRunState::Approach||R->RunState>EFactoryRunState::Verify)return;
    if(auto* C=Controller())C->Cancel();R->RunState=EFactoryRunState::Stopped;R->SimRemainder=0;
    R->Status=TEXT("정지했습니다. 로봇과 휠의 현재 자세를 유지합니다. 편집 복귀로 초기화하세요.");
}
void UFactoryWheelProcessRunner::Reset()
{
    auto* R=Runtime();if(!R)return;++R->RunEpoch;
    if(auto* C=Controller())C->Reset(R->RunEpoch);
    bHolding=false;StateSeconds=0;R->RunState=EFactoryRunState::Editing;
    R->SimulationSeconds=R->TravelCm=R->AlignmentMm=R->SimRemainder=0;
    R->RefreshPreviewWheel();R->Status=TEXT("편집 준비: 휠은 현재 배치의 공급 위치로 돌아갔습니다.");
}
void UFactoryWheelProcessRunner::Step(double Dt)
{
    auto* R=Runtime();auto* C=Controller();if(!R||!C||R->RunState<EFactoryRunState::Approach||R->RunState>EFactoryRunState::Verify)return;
    R->SimulationSeconds+=Dt;StateSeconds+=Dt;const FTransform Old=C->Pose();C->Step(Dt);const FTransform Pose=C->Pose();
    if(auto* H=R->Equipment.FindRef(R->Layout.HandlerId).Get())H->UpdateTool(Pose.GetLocation(),Pose.Rotator().Yaw);
    if(bHolding&&R->WheelRoot)
    {
        FString Obstacle;if(R->Obstructed(Old.GetLocation(),Pose.GetLocation(),Obstacle))
        {C->Cancel();R->RunState=EFactoryRunState::Fault;R->Status=TEXT("운반 중 간섭 감지: ")+Obstacle;return;}
        R->TravelCm+=FVector::Distance(Old.GetLocation(),Pose.GetLocation());R->WheelRoot->SetWorldTransform(Pose);
    }
    if(C->Status()==EFactoryCommandStatus::Failed||StateSeconds>40)
    {C->Cancel();R->RunState=EFactoryRunState::Fault;R->Status=TEXT("목표 도달 시간 초과");return;}
    if(!(C->RequestId()==Pending)||C->Status()!=EFactoryCommandStatus::Completed)return;
    const FVector Supply=R->SupplyWorld(),Hub=R->HubWorld();const FVector Outside=Hub+FRotator(0,R->VehicleYaw(),0).RotateVector(FVector(0,-100,0));
    switch(R->RunState)
    {
    case EFactoryRunState::Approach:Change(EFactoryRunState::Pickup);Move(Supply,R->SupplyYaw());break;
    case EFactoryRunState::Pickup: bHolding=true;Change(EFactoryRunState::Lift);Move(FVector(Supply.X,Supply.Y,250),R->SupplyYaw());break;
    case EFactoryRunState::Lift:Change(EFactoryRunState::Transfer);Move(FVector(Outside.X,Outside.Y,250),R->VehicleYaw());break;
    case EFactoryRunState::Transfer:Change(EFactoryRunState::Align);Move(Outside,R->VehicleYaw());break;
    case EFactoryRunState::Align:Change(EFactoryRunState::Verify);Move(Hub,R->VehicleYaw());break;
    case EFactoryRunState::Verify:
        R->AlignmentMm=R->WheelRoot?FVector::Distance(R->WheelRoot->GetComponentLocation(),Hub)*10:999;
        if(R->AlignmentMm<=.5){bHolding=false;R->LastCycleSeconds=R->SimulationSeconds;Change(EFactoryRunState::Complete);
            R->Status=TEXT("허브 정렬 완료 · 좌표 기반 검사 / 볼트 체결·접촉 물리는 미포함");}break;
    default:break;
    }
}

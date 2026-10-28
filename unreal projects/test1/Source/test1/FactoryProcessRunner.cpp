#include "FactoryProcessRunner.h"
#include "FactoryEquipment.h"
#include "Components/SceneComponent.h"
#include "DrawDebugHelpers.h"

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
    ActiveGoal=Position;
    return true;
}
bool UFactoryWheelProcessRunner::Start()
{
    auto* R=Runtime();if(!R||!R->CanEdit())return false;R->CancelEdit();FString Error;
    if(!R->ValidateRun(Error)){R->Status=Error;return false;}
    R->SimulationSeconds=R->TravelCm=R->AlignmentMm=R->SimRemainder=0;bHolding=false;
    bSensorFeatures=false;bWaiting=false;Sensors.Reset();Replans=GripRetries=Recoveries=0;
    GripAttempts=0;WaitSeconds=StableSeconds=GripSeconds=0;bObservedFault=false;
    ObservedPickup=R->SupplyWorld();FeatureStatus=TEXT("미실행");
    ++R->RunEpoch;Controller()->Reset(R->RunEpoch);Change(EFactoryRunState::Approach);
    R->Status=TEXT("실행 중: 현재 배치의 공급점과 허브를 기준으로 휠을 운반합니다.");
    return Move(R->SupplyWorld()+FVector(0,0,80),R->SupplyYaw());
}
void UFactoryWheelProcessRunner::Stop()
{
    auto* R=Runtime();if(!R||R->RunState<EFactoryRunState::Approach||R->RunState>EFactoryRunState::Verify)return;
    if(auto* C=Controller())C->Cancel();R->RunState=EFactoryRunState::Stopped;R->SimRemainder=0;
    if(bSensorFeatures)FeatureStatus=TEXT("사용자 정지 · 검증 미완료");
    R->Status=TEXT("정지했습니다. 로봇과 휠의 현재 자세를 유지합니다. 편집 복귀로 초기화하세요.");
}
void UFactoryWheelProcessRunner::Reset()
{
    auto* R=Runtime();if(!R)return;++R->RunEpoch;
    if(auto* C=Controller())C->Reset(R->RunEpoch);
    bHolding=false;StateSeconds=0;R->RunState=EFactoryRunState::Editing;
    bSensorFeatures=false;bWaiting=false;Sensors.Reset();FeatureStatus=TEXT("미실행");
    if(R->WheelRoot)R->WheelRoot->SetVisibility(true,true);
    R->SimulationSeconds=R->TravelCm=R->AlignmentMm=R->SimRemainder=0;
    R->RefreshPreviewWheel();R->Status=TEXT("편집 준비: 휠은 현재 배치의 공급 위치로 돌아갔습니다.");
}
void UFactoryWheelProcessRunner::Step(double Dt)
{
    auto* R=Runtime();auto* C=Controller();if(!R||!C||R->RunState<EFactoryRunState::Approach||R->RunState>EFactoryRunState::Verify)return;
    R->SimulationSeconds+=Dt;
    if(bSensorFeatures&&!CheckObservations(Dt))return;
    StateSeconds+=Dt;const FTransform Old=C->Pose();C->Step(Dt);const FTransform Pose=C->Pose();
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
    const FVector Supply=bSensorFeatures?ObservedPickup:R->SupplyWorld(),Hub=R->HubWorld();const FVector Outside=Hub+FRotator(0,R->VehicleYaw(),0).RotateVector(FVector(0,-100,0));
    switch(R->RunState)
    {
    case EFactoryRunState::Approach:Change(EFactoryRunState::Pickup);Move(Supply,R->SupplyYaw());break;
    case EFactoryRunState::Pickup:
        if(bSensorFeatures)
        {
            if(!Sensors.GripClosed){Sensors.GripClosed=true;GripAttempts=1;GripSeconds=0;}
            GripSeconds+=Dt;
            if(!Sensors.Reading.Gripped)
            {
                if(GripSeconds>=.25)
                {
                    bObservedFault=true;
                    if(GripAttempts>=8){FeatureFault(TEXT("파지 확인 실패 · 재시도 한도 초과"));return;}
                    ++GripAttempts;++GripRetries;GripSeconds=0;
                    FeatureStatus=FString::Printf(TEXT("파지 확인 대기 · 재시도 %d/7"),GripRetries);
                }
                return;
            }
            FeatureStatus=TEXT("파지 센서 확인 · 운반 진행");
        }
        bHolding=true;Change(EFactoryRunState::Lift);Move(FVector(Supply.X,Supply.Y,250),R->SupplyYaw());break;
    case EFactoryRunState::Lift:Change(EFactoryRunState::Transfer);Move(FVector(Outside.X,Outside.Y,250),R->VehicleYaw());break;
    case EFactoryRunState::Transfer:Change(EFactoryRunState::Align);Move(Outside,R->VehicleYaw());break;
    case EFactoryRunState::Align:Change(EFactoryRunState::Verify);Move(Hub,R->VehicleYaw());break;
    case EFactoryRunState::Verify:
        R->AlignmentMm=R->WheelRoot?FVector::Distance(R->WheelRoot->GetComponentLocation(),Hub)*10:999;
        if(R->AlignmentMm<=.5){bHolding=false;R->LastCycleSeconds=R->SimulationSeconds;Change(EFactoryRunState::Complete);
            Sensors.GripClosed=false;
            if(bSensorFeatures)FeatureStatus=FeaturePassed()?TEXT("통과 · 센서 확인 및 목표 동작 관측 완료"):TEXT("미확인 · 운반 완료, 해당 검증 조건은 관측되지 않음");
            R->Status=TEXT("허브 정렬 완료 · 좌표 기반 검사 / 볼트 체결·접촉 물리는 미포함");}break;
    default:break;
    }
}

bool UFactoryWheelProcessRunner::StartFeatureCheck(int32 Kind)
{
    if(Kind<0||Kind>4)return false;
    if(!Start()){FeatureStatus=TEXT("시작 불가 · 배치/작업 연결을 확인하세요.");return false;}
    auto* R=Runtime();FeatureKind=Kind;bSensorFeatures=true;
    Sensors.Sample(0,true,R->WheelRoot->GetComponentLocation(),Controller()->Pose().GetLocation());
    if(Kind==1)R->WheelRoot->SetVisibility(false,true);
    if(Kind==2)Sensors.PublishPose=false;
    if(Kind==3)Sensors.GripFault=true;
    if(Kind==4)
    {
        const auto* Supply=R->Layout.Find(R->Layout.SupplyId);
        R->WheelRoot->AddWorldOffset(Supply->Transform().TransformVectorNoScale(FVector(0,-4,0)));
    }
    FeatureStatus=TEXT("실행 중 · 관측값 확인 대기");return true;
}

void UFactoryWheelProcessRunner::RestoreFeatureInputs()
{
    auto* R=Runtime();if(!bSensorFeatures||!R||R->RunState==EFactoryRunState::Fault||R->RunState==EFactoryRunState::Stopped||R->RunState==EFactoryRunState::Complete)return;
    if(R->WheelRoot)R->WheelRoot->SetVisibility(true,true);
    Sensors.PublishPose=true;Sensors.GripFault=false;
    // Deliberately do not command the robot here. Fresh observations authorize recovery.
}

void UFactoryWheelProcessRunner::FeatureFault(const FString& Reason)
{
    if(auto* C=Controller())C->Cancel();
    auto* R=Runtime();R->RunState=EFactoryRunState::Fault;R->Status=Reason;
    FeatureStatus=TEXT("중단 · ")+Reason;
}

bool UFactoryWheelProcessRunner::CheckObservations(double Dt)
{
    auto* R=Runtime();auto* C=Controller();
    if(!R->WheelRoot){FeatureFault(TEXT("부품 참조 상실"));return false;}
    Sensors.Sample(R->SimulationSeconds,R->WheelRoot->IsVisible(),R->WheelRoot->GetComponentLocation(),C->Pose().GetLocation());
    const auto& S=Sensors.Reading;
    if(!S.Present||!S.Fresh(R->SimulationSeconds))
    {
        bObservedFault=true;
        if(bHolding){FeatureFault(TEXT("부품 보유 중 센서 상실 · 자동 재개 금지"));return false;}
        if(!bWaiting){C->Cancel();bWaiting=true;WaitSeconds=StableSeconds=0;Sensors.GripClosed=false;}
        WaitSeconds+=Dt;StableSeconds=0;
        FeatureStatus=!S.Present?TEXT("부품 없음 · 공급 복원 대기"):TEXT("위치 측정 지연 · 새 측정값 대기");
        if(WaitSeconds>5)FeatureFault(TEXT("공급/위치 센서 복구 제한 5초 초과"));
        return false;
    }
    if(bWaiting)
    {
        WaitSeconds+=Dt;StableSeconds+=Dt;
        if(WaitSeconds>5){FeatureFault(TEXT("복구 안정화 시간 초과"));return false;}
        if(StableSeconds<.15)return false;
        ++Recoveries;bWaiting=false;ObservedPickup=S.Position;GripAttempts=0;GripSeconds=0;
        Change(EFactoryRunState::Approach);Move(ObservedPickup+FVector(0,0,80),R->SupplyYaw());
        FeatureStatus=TEXT("새 측정 안정 확인 · 접근 재개");return false;
    }
    if(!bHolding&&FVector::Distance(S.Position,ObservedPickup)>.1)
    {
        if(FVector::Distance(S.Position,R->SupplyWorld())>50)
        {FeatureFault(TEXT("부품 위치 변화가 회복 범위를 초과했습니다."));return false;}
        C->Cancel();ObservedPickup=S.Position;++Replans;Sensors.GripClosed=false;
        Change(EFactoryRunState::Approach);Move(ObservedPickup+FVector(0,0,80),R->SupplyYaw());
        FeatureStatus=TEXT("위치 변화 측정 · 접근 목표 재계산");return false;
    }
    return true;
}

bool UFactoryWheelProcessRunner::FeaturePassed() const
{
    auto* R=Runtime();if(!bSensorFeatures||!R||R->RunState!=EFactoryRunState::Complete)return false;
    if(FeatureKind==1||FeatureKind==2)return bObservedFault&&Recoveries>0;
    if(FeatureKind==3)return bObservedFault&&GripRetries>0;
    if(FeatureKind==4)return Replans>0;
    return true;
}

FString UFactoryWheelProcessRunner::FeatureSummary() const
{
    if(!bSensorFeatures)return FeatureStatus;
    const auto& S=Sensors.Reading;auto* R=Runtime();
    return FString::Printf(TEXT("%s\n부품 %s · 위치 %s · 파지 %s\n재계산 %d · 재시도 %d · 대기 복구 %d"),
        *FeatureStatus,S.Present?TEXT("있음"):TEXT("없음"),S.Fresh(R->SimulationSeconds)?TEXT("유효"):TEXT("지연/무효"),
        S.Gripped?TEXT("확인"):TEXT("미확인"),Replans,GripRetries,Recoveries);
}

void UFactoryWheelProcessRunner::DrawFeatureVisualization() const
{
    const auto* R=Runtime();const auto* C=Controller();
    if(!R||!C||!GetWorld()||!R->WheelRoot||!R->Layout.Find(R->Layout.SupplyId)||!R->Layout.Find(R->Layout.StationId))return;
    const FVector Supply=R->SupplyWorld(),Hub=R->HubWorld(),TCP=C->Pose().GetLocation();
    // Draw for this frame only: view toggles never leave persistent stale geometry.
    auto Sphere=[&](FVector P,double Radius,FColor Color){DrawDebugSphere(GetWorld(),P,Radius,16,Color,false,-1,0,2);};
    const auto& S=Sensors.Reading;
    if(bShowSensors)
    {
        // A symbolic virtual probe at the supply reference, not a physical sensor or field of view.
        const FVector Probe=Supply+FVector(0,0,55);
        DrawDebugBox(GetWorld(),Probe,FVector(6),FColor::Cyan,false,-1,0,2);
        DrawDebugLine(GetWorld(),Probe,Supply,FColor::Cyan,false,-1,0,1);
        if(R->WheelRoot->IsVisible())Sphere(R->WheelRoot->GetComponentLocation(),5,FColor::White);
        Sphere(Hub,12,FColor::Green);
        DrawDebugCoordinateSystem(GetWorld(),Hub,FRotator(0,R->VehicleYaw(),0),25,false,-1,0,1);
        const FColor ToolColor=!bSensorFeatures?FColor::Cyan:!S.Fresh(R->SimulationSeconds)||!S.Present?FColor::Red:S.Gripped?FColor::Green:FColor::Yellow;
        Sphere(TCP,8,ToolColor);
        if(bSensorFeatures&&S.PoseAt>=0)
        {
            const FColor MeasuredColor=S.Fresh(R->SimulationSeconds)?FColor::Yellow:FColor::Red;
            Sphere(S.Position,10,MeasuredColor);
            DrawDebugLine(GetWorld(),Probe,S.Position,MeasuredColor,false,-1,0,1);
        }
    }
    if(bShowRoute)
    {
        const FVector Pickup=bSensorFeatures&&!R->CanEdit()?ObservedPickup:Supply;
        const FVector Outside=Hub+FRotator(0,R->VehicleYaw(),0).RotateVector(FVector(0,-100,0));
        const TArray<FVector> Route={Pickup+FVector(0,0,80),Pickup,FVector(Pickup.X,Pickup.Y,250),FVector(Outside.X,Outside.Y,250),Outside,Hub};
        for(int32 I=1;I<Route.Num();++I)
            DrawDebugDirectionalArrow(GetWorld(),Route[I-1],Route[I],12,FColor(90,150,255),false,-1,0,1.5);
        if(C->Status()==EFactoryCommandStatus::Running)
        {
            Sphere(ActiveGoal,7,FColor::Magenta);
            DrawDebugDirectionalArrow(GetWorld(),TCP,ActiveGoal,14,FColor::Magenta,false,-1,0,2);
        }
    }
}

FString UFactoryWheelProcessRunner::VisualizationSummary() const
{
    const auto* R=Runtime();const auto& S=Sensors.Reading;
    if(!bSensorFeatures||!R)return TEXT("센서 측정: 검증 항목 실행 후 표시");
    if(S.PoseAt<0)return TEXT("센서 측정: 아직 수신하지 않음");
    return FString::Printf(TEXT("위치 측정 나이 %.2f초 / 허용 0.25초%s%s"),
        R->SimulationSeconds-S.PoseAt,S.Fresh(R->SimulationSeconds)?TEXT(""):TEXT(" · 갱신 중단"),
        bWaiting?*FString::Printf(TEXT("\n복구 대기: 중단까지 %.1f초"),FMath::Max(0.,5-WaitSeconds)):TEXT(""));
}

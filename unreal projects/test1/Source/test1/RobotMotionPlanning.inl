// Included once by RobotArm.cpp. Planning owns no UObject and runs only on the
// game thread; scene/physics queries are never sent to an unsafe worker thread.
bool ARobotArm::QueueTCPMotion(const FTransform& World,bool bLinear)
{
    if(!bInitialized||!World.IsValid()||!World.GetScale3D().Equals(FVector::OneVector,1.e-6))
    {LastStatus=TEXT("유효하지 않은 TCP 이동 요청");return false;}
    const FTransform From=GetTCPWorld();
    if(bLinear&&FMath::RadiansToDegrees(From.GetRotation().AngularDistance(World.GetRotation()))>.5)
    {LastStatus=TEXT("직선 삽입 차단 · 장착 바깥에서 먼저 각도를 맞추세요.");return false;}
    PendingMotion=MakeShared<FRobotMotionRequest>();auto& R=*PendingMotion;
    R.Base=GetActorTransform();R.From=From;R.Goal=World;R.Start=Angles;R.bLinear=bLinear;
    // Preserve the TCP attitude/IK branch on transfers as well as short moves.
    // The wheelbase exceeds the old 220 cm cutoff: a joint chord could flip
    // the wrist even though the two rack poses had equal attitude.
    R.bPreferStraight=FMath::RadiansToDegrees(From.GetRotation().AngularDistance(World.GetRotation()))<5;
    R.Seeds.Add(Angles);
    if(PreferredIKSeedDegrees.Num()==6)
    {auto Seed=PreferredIKSeedDegrees;for(auto& Q:Seed)Q=FMath::DegreesToRadians(Q);R.Seeds.Add(Seed);}
    for(double Offset:{-PI/2,PI/2})
    {auto Seed=Angles;Seed[0]=FMath::Clamp(Seed[0]+Offset,Definitions[0].MinRadians,Definitions[0].MaxRadians);R.Seeds.Add(Seed);}
    if(bLinear)R.Phase=FRobotMotionRequest::EPhase::Build;
    MotionPath.Reset();MotionDurations.Reset();ActiveTrajectory=FRobotMotionTrajectory();
    Targets=SegmentStart=Angles;SegmentDuration=SegmentElapsed=0;bMotionBlocked=false;
    PlannedWaypoints=0;PlannedMotionSeconds=0;LastIKIterations=0;
    MotionScene.Capture(*this,MotionPart.Get(),bMotionHolding);SceneAge=0;
    UE_LOG(LogTemp,Verbose,TEXT("MOTION_REQUEST %s base=%s from=%s goal=%s angle=%.2f joints=%s"),*GetName(),*GetActorLocation().ToString(),*From.GetLocation().ToString(),*World.GetLocation().ToString(),FMath::RadiansToDegrees(From.GetRotation().AngularDistance(World.GetRotation())),*FString::JoinBy(Angles,TEXT(","),[](double Q){return FString::Printf(TEXT("%.2f"),FMath::RadiansToDegrees(Q));}));
    LastStatus=bLinear?TEXT("자세 유지 직선 경로 계산 중"):TEXT("현재 자세 기준 이동 경로 계산 중");
    return true; // Request accepted; IsAtJointTarget stays false until execution settles.
}
bool ARobotArm::CommandTCPWorld(const FTransform& World){return QueueTCPMotion(World,false);}
bool ARobotArm::CommandLinearTCPWorld(const FTransform& World){return QueueTCPMotion(World,true);}
bool ARobotArm::PlanJointMove(const TArray<double>& Goal)
{
    FTransform TCP;
    if(!FRobotKinematics::ForwardKinematics(Definitions,Goal,ToolInLastJointCm,TCP)||!QueueTCPMotion(TCP*GetActorTransform(),false))return false;
    auto& R=*PendingMotion;R.bJointOnly=true;R.bPreferStraight=false;R.Goals={Goal};R.Phase=FRobotMotionRequest::EPhase::Build;
    return true;
}

bool ARobotArm::AdvanceMotionPlanning()
{
    if(!PendingMotion)return true;
    // Fixed-step catch-up must not multiply planning cost in a slow render
    // frame. Share a 4 ms soft budget across arms, with 1.5 ms per work slice.
    // Commandlet/manual pre-BeginPlay tests have no rendering frame to yield to.
    static uint64 BudgetFrame=MAX_uint64;static double FrameUsed=0;
    const bool LimitFrame=FApp::CanEverRender()&&GetWorld()&&GetWorld()->HasBegunPlay()&&!IsRunningCommandlet();
    if(LimitFrame&&BudgetFrame!=GFrameCounter){BudgetFrame=GFrameCounter;FrameUsed=0;}
    if(LimitFrame&&FrameUsed>=.004)return true;
    const double Begin=FPlatformTime::Seconds(),BudgetEnd=Begin+(LimitFrame?FMath::Min(.0015,.004-FrameUsed):.0015);
    ON_SCOPE_EXIT{if(LimitFrame)FrameUsed+=FPlatformTime::Seconds()-Begin;};
    auto& R=*PendingMotion;
    auto Reject=[&](const FString& Detail)
    {
        LastStatus=(R.bLinear?TEXT("직선 삽입 차단 · 회전 우회 없음 · "):TEXT("이동 경로 차단 · "))+Detail;
        UE_LOG(LogTemp,Warning,TEXT("MOTION_REJECT %s phase=%d route=%d cpu=%.3f %s"),*GetName(),int(R.Phase),R.Route,R.ElapsedPlanning,*LastStatus);
        PendingMotion.Reset();bMotionBlocked=true;return false;
    };
    if(!R.Base.Equals(GetActorTransform(),1.e-6))return Reject(TEXT("계산 중 로봇 기준 좌표 변경"));
    if(R.ElapsedPlanning>2.)return Reject(TEXT("계산 예산 초과 · 배치/간섭 위치를 확인하세요. ")+R.Failure);
    FRobotIKSettings IK;IK.MaxRestarts=0;IK.MaxIterations=100;IK.PositionToleranceCm=.005;IK.OrientationToleranceRadians=.0001;
    auto Normalize=[&](TArray<double>& Q,const TArray<double>& Near)
    {
        for(int J=0;J<Q.Num();++J)
        {
            FRobotMotionPlanner::ClosestEquivalent(Definitions[J],Near[J],Q[J]);
        }
    };
    auto NextCandidate=[&](){UE_LOG(LogTemp,Verbose,TEXT("MOTION_CANDIDATE %s route=%d sample=%d/%d %s"),*GetName(),R.Route,R.Sample,R.Steps,*R.Failure);R.Trial=FRobotMotionTrajectory();R.Phase=FRobotMotionRequest::EPhase::Build;};
    auto StartPreview=[&]()
    {
        R.PreviewState=Controllers;R.PreviewTime=0;
        if(!bTorqueDynamics)for(auto& C:R.PreviewState)C.IntegralErrorSeconds=0;
        R.Trial.Duration=R.NominalDuration*FMath::Pow(1.5,R.Scale);
        R.Phase=FRobotMotionRequest::EPhase::Preview;
    };
    // Each iteration performs one IK solve, collision sample or servo step.
    // A single geometry query can exceed the soft slice; no multi-second nested
    // search or complete 120 Hz simulation runs inside an EnterMove call.
    do
    {
        if(R.Phase==FRobotMotionRequest::EPhase::Seeds)
        {
            if(R.SeedIndex<R.Seeds.Num())
            {
                auto Sol=FRobotKinematics::SolveInverseKinematics(Definitions,ToolInLastJointCm,R.Goal.GetRelativeTransform(R.Base),R.Seeds[R.SeedIndex++],IK);
                LastIKIterations+=Sol.Iterations;
                if(Sol.IsSuccess())
                {
                    Normalize(Sol.JointRadians,R.Start);
                    const bool Duplicate=R.Goals.ContainsByPredicate([&](const auto& Q)
                    {for(int J=0;J<6;++J)if(FMath::Abs(Q[J]-Sol.JointRadians[J])>.001)return false;return true;});
                    if(!Duplicate)R.Goals.Add(MoveTemp(Sol.JointRadians));
                }
                continue;
            }
            // Rank by configured travel time, with a small cost for needless
            // base/wrist winding. Current-branch Cartesian travel gets first
            // refusal for nearby empty-gripper pickups with the same attitude.
            auto Cost=[&](const TArray<double>& Q)
            {return FRobotMotionPlanner::Duration(Definitions,R.Start,Q)+.15*(FMath::Abs(Q[0]-R.Start[0])+FMath::Abs(Q[5]-R.Start[5]));};
            R.Goals.Sort([&](const auto& A,const auto& B){return Cost(A)<Cost(B);});
            R.Phase=FRobotMotionRequest::EPhase::Build;
        }
        if(R.Phase==FRobotMotionRequest::EPhase::Build)
        {
            if(R.Trial.Samples.IsEmpty())
            {
                ++R.Route;R.Trial.StartTCP=R.From;R.Trial.GoalTCP=R.Goal;R.Trial.bLinear=R.bLinear;
                R.Trial.Samples.Add(R.Start);R.Stops={R.From};R.Stop=1;R.Sample=1;R.Steps=0;
                int Direct=INDEX_NONE,Detour=INDEX_NONE;
                if(R.bLinear)
                {if(R.Route>0)return Reject(R.Failure);R.Stops.Add(R.Goal);}
                else if(R.bJointOnly)
                {if(R.Route>0)return Reject(R.Failure);Direct=0;}
                else
                {
                    const int Straight=R.bPreferStraight?0:R.Goals.Num();
                    if(R.Route==Straight)R.Stops.Add(R.Goal);
                    else if(R.Route<=R.Goals.Num())Direct=R.Route-(R.bPreferStraight?1:0);
                    else Detour=R.Route-R.Goals.Num()-1;
                    if(Detour>=12)return Reject(R.Failure.IsEmpty()?TEXT("도달 가능한 충돌 없는 경로 없음"):R.Failure);
                }
                if(Direct!=INDEX_NONE)
                {
                    R.Trial.Samples.Add(R.Goals[Direct]);R.Phase=FRobotMotionRequest::EPhase::Check;
                    R.Edge=R.EdgeSample=R.EdgeSteps=0;continue;
                }
                if(Detour!=INDEX_NONE)
                {
                    FTransform A=R.From,B=R.Goal;
                    if(Detour<4)
                    {
                        const double Lifts[]={15.,30.,60.,90.};
                        const double Height=FMath::Max(A.GetLocation().Z,B.GetLocation().Z)+Lifts[Detour];
                        auto P=A.GetLocation();P.Z=Height;A.SetLocation(P);P=B.GetLocation();P.Z=Height;B.SetLocation(P);
                    }
                    else
                    {
                        const FVector Side=FVector::CrossProduct((B.GetLocation()-A.GetLocation()).GetSafeNormal2D(),FVector::UpVector);
                        const double Offsets[]={30.,-30.,60.,-60.,100.,-100.,150.,-150.};
                        const FVector Offset=Side*Offsets[Detour-4]+FVector(0,0,20);
                        A.AddToTranslation(Offset);B.AddToTranslation(Offset);
                    }
                    R.Stops.Append({A,B,R.Goal});
                }
            }
            if(R.Stop>=R.Stops.Num())
            {R.Edge=R.EdgeSample=R.EdgeSteps=0;R.Phase=FRobotMotionRequest::EPhase::Check;continue;}
            const FTransform& A=R.Stops[R.Stop-1];const FTransform& B=R.Stops[R.Stop];
            if(!R.Steps)R.Steps=FMath::Max(1,FMath::CeilToInt(FMath::Max(
                FVector::Distance(A.GetLocation(),B.GetLocation())/(R.bLinear?1.:15.),
                FMath::RadiansToDegrees(A.GetRotation().AngularDistance(B.GetRotation()))/5.)));
            const double T=double(R.Sample)/R.Steps;
            const FTransform Pose(R.bLinear?R.Goal.GetRotation():FQuat::Slerp(A.GetRotation(),B.GetRotation(),T),FMath::Lerp(A.GetLocation(),B.GetLocation(),T));
            auto Sol=FRobotKinematics::SolveInverseKinematics(Definitions,ToolInLastJointCm,Pose.GetRelativeTransform(R.Base),R.Trial.Samples.Last(),IK);
            LastIKIterations+=Sol.Iterations;
            if(!Sol.IsSuccess()){R.Failure=FString::Printf(TEXT("연속 자세 IK: %s · 위치 오차 %.3fcm / 각도 오차 %.3f도 · 관절 %s"),FRobotKinematics::StatusText(Sol.Status),Sol.PositionErrorCm,FMath::RadiansToDegrees(Sol.OrientationErrorRadians),*FString::JoinBy(Sol.JointRadians,TEXT(","),[](double Q){return FString::Printf(TEXT("%.1f"),FMath::RadiansToDegrees(Q));}));NextCandidate();continue;}
            Normalize(Sol.JointRadians,R.Trial.Samples.Last());
            R.Trial.Samples.Add(MoveTemp(Sol.JointRadians));
            if(++R.Sample>R.Steps){++R.Stop;R.Sample=1;R.Steps=0;}
            continue;
        }
        if(R.Phase==FRobotMotionRequest::EPhase::Check)
        {
            if(R.Edge+1>=R.Trial.Samples.Num())
            {R.Trial.SetDuration(Definitions);R.NominalDuration=R.Trial.Duration;R.Scale=0;StartPreview();continue;}
            const auto& A=R.Trial.Samples[R.Edge];const auto& B=R.Trial.Samples[R.Edge+1];
            if(!R.EdgeSteps)
            {double Travel=0;for(int J=0;J<6;++J)Travel+=FMath::Abs(B[J]-A[J]);R.EdgeSteps=FMath::Max(1,FMath::CeilToInt(Travel/FMath::DegreesToRadians(.25)));}
            auto Q=A;for(int J=0;J<6;++J)Q[J]=FMath::Lerp(A[J],B[J],double(R.EdgeSample)/R.EdgeSteps);
            FTransform TCP;FRobotKinematics::ForwardKinematics(Definitions,Q,ToolInLastJointCm,TCP);
            if(!R.Trial.InCorridor(TCP*R.Base,.05,.1))
            {R.Failure=TEXT("직선/자세 유지 오차 초과 · 바깥에서 재정렬 필요");NextCandidate();continue;}
            if(!MotionScene.Clear(*this,Q,R.Failure)){NextCandidate();continue;}
            if(++R.EdgeSample>R.EdgeSteps){++R.Edge;R.EdgeSample=R.EdgeSteps=0;}
            continue;
        }
        if(R.Phase==FRobotMotionRequest::EPhase::Preview)
        {
            constexpr double Dt=1./120.;R.PreviewTime+=Dt;
            TArray<double> Ref,Feed,Q;R.Trial.Reference(R.PreviewTime,Ref,Feed);Q.SetNum(6);
            FRobotJointControllerSettings Settings;Settings.Kp=Kp;Settings.Ki=Ki;Settings.Kd=Kd;Settings.VelocityResponseSeconds=MotorResponseSeconds;
            bool Safe=true,Settled=R.PreviewTime>=R.Trial.Duration+.2;
            TArray<FTransform> Frames;FTransform TCP;
            TArray<double> Before;for(const auto& C:R.PreviewState)Before.Add(C.AngleRadians);
            if(bTorqueDynamics)FRobotKinematics::ForwardKinematics(Definitions,Before,ToolInLastJointCm,TCP,&Frames);
            for(int J=0;J<6;++J)
            {
                double Torque=0,Load=0;
                if(bTorqueDynamics)
                {
                    const auto Frame=Frames[J]*R.Base;const FVector Axis=Frame.TransformVectorNoScale(Definitions[J].AxisLocal);
                    Load=FVector::DotProduct(Axis,FVector::CrossProduct((ForcePointWorldCm-Frame.GetLocation())/100,ToolReactionForceN)+ToolReactionTorqueNm);
                }
                Safe&=bTorqueDynamics?FRobotTorquePlant::Step(Definitions[J],Ref[J],Feed[J],J<3?40:4,(J<3?4000:400)*TorqueLimitScale,Load,Dt,R.PreviewState[J],Torque)
                    :FRobotJointController::Step(Definitions[J],Settings,Ref[J],Feed[J],Dt,R.PreviewState[J]);
                Q[J]=R.PreviewState[J].AngleRadians;
                Settled&=FMath::Abs(Q[J]-R.Trial.Samples.Last()[J])<FMath::DegreesToRadians(.05)
                    &&FMath::Abs(R.PreviewState[J].VelocityRadians)<FMath::DegreesToRadians(.1);
            }
            FRobotKinematics::ForwardKinematics(Definitions,Q,ToolInLastJointCm,TCP);
            if(!R.Trial.InCorridor(TCP*R.Base)){Safe=false;R.Failure=TEXT("PID 예측에서 삽입 각도/직선 허용 오차 초과");}
            if(Safe)Safe=MotionScene.Clear(*this,Q,R.Failure);
            if(!Safe||(!Settled&&R.PreviewTime>R.Trial.Duration+3))
            {if(++R.Scale<4)StartPreview();else NextCandidate();continue;}
            if(Settled)
            {
                // Keep the full continuous reference. No waypoint stop/start
                // cycle and no geometry shortcut that can cut an insertion arc.
                ActiveTrajectory=MoveTemp(R.Trial);MotionPath=ActiveTrajectory.Samples;
                Targets=MotionPath.Last();SegmentStart=Angles;SegmentElapsed=0;SegmentDuration=ActiveTrajectory.Duration;
                PlannedMotionSeconds=SegmentDuration;PlannedWaypoints=MotionPath.Num()-1;MotionIndex=MotionPath.Num()-1;
                UE_LOG(LogTemp,Display,TEXT("MOTION_READY %s linear=%d route=%d cpu=%.3f duration=%.3f wrist_delta=%.2f"),*GetName(),R.bLinear,R.Route,R.ElapsedPlanning+FPlatformTime::Seconds()-Begin,SegmentDuration,FMath::RadiansToDegrees(Targets[5]-R.Start[5]));
                if(!bTorqueDynamics)for(auto& C:Controllers)C.IntegralErrorSeconds=0;
                PendingMotion.Reset();bMotionBlocked=false;
                LastStatus=FString::Printf(TEXT("%s · PID 추종 %.2f초"),ActiveTrajectory.bLinear?TEXT("자세 유지 직선 이동"):TEXT("연속 경로 이동"),SegmentDuration);
                return true;
            }
        }
    }while(FPlatformTime::Seconds()<BudgetEnd);
    R.ElapsedPlanning+=FPlatformTime::Seconds()-Begin;
    return true;
}

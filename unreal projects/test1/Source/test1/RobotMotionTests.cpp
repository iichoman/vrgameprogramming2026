#include "RobotArm.h"
#include "RobotMotionPlanner.h"
#include "Engine/World.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/BoxComponent.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotWristRangeTests,"DigitalTwin.Assembly.CommercialWristRange",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRobotWristRangeTests::RunTest(const FString&)
{
    FRobotJointDefinition Wrist;Wrist.MinRadians=FMath::DegreesToRadians(-350.);Wrist.MaxRadians=-Wrist.MinRadians;
    double Q=FMath::DegreesToRadians(-170.);
    TestTrue(TEXT("equivalent wrist pose found across +180"),FRobotMotionPlanner::ClosestEquivalent(Wrist,FMath::DegreesToRadians(185.),Q));
    TestTrue(TEXT("wrist continues to +190 instead of reversing to -170"),FMath::IsNearlyEqual(Q,FMath::DegreesToRadians(190.),1.e-9));
    Q=FMath::DegreesToRadians(170.);
    TestTrue(TEXT("equivalent wrist pose found across -180"),FRobotMotionPlanner::ClosestEquivalent(Wrist,FMath::DegreesToRadians(-185.),Q));
    TestTrue(TEXT("negative turn stays continuous"),FMath::IsNearlyEqual(Q,FMath::DegreesToRadians(-190.),1.e-9));
    Q=FMath::DegreesToRadians(-349.);
    TestTrue(TEXT("finite cable interval still permits a longer legal equivalent"),FRobotMotionPlanner::ClosestEquivalent(Wrist,FMath::DegreesToRadians(349.),Q));
    TestTrue(TEXT("never crosses +350 stop using a short quaternion arc"),FMath::IsNearlyEqual(Q,FMath::DegreesToRadians(11.),1.e-9));
    FRobotJointDefinition Shoulder;Shoulder.MinRadians=FMath::DegreesToRadians(-130.);Shoulder.MaxRadians=FMath::DegreesToRadians(17.5);
    Q=FMath::DegreesToRadians(90.);
    TestFalse(TEXT("asymmetric shoulder range is not an unlimited revolute joint"),FRobotMotionPlanner::ClosestEquivalent(Shoulder,0,Q));
    FRobotJointControllerSettings PID;FRobotJointState State;State.AngleRadians=FMath::DegreesToRadians(179.);
    TestTrue(TEXT("PID accepts a continuous reference above 180"),FRobotJointController::Step(Wrist,PID,FMath::DegreesToRadians(181.),0,1./120,State));
    TestTrue(TEXT("PID turns forward through 180, without an angle wrap"),State.AngleRadians>FMath::DegreesToRadians(179.));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotSceneTests,"DigitalTwin.Assembly.MotionScene",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRobotSceneTests::RunTest(const FString&)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Game,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    auto* Cube=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube"));
    auto Box=[&](FVector P,FVector Scale)
    {
        auto* A=World->SpawnActor<AStaticMeshActor>();auto* C=A->GetStaticMeshComponent();
        C->SetMobility(EComponentMobility::Movable);C->SetStaticMesh(Cube);C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        A->SetActorLocation(P);A->SetActorScale3D(Scale);return A;
    };
    auto* Arm=World->SpawnActor<ARobotArm>();Arm->Joints.SetNum(6);Arm->InitialAnglesDegrees={0,0,0,0,0,0};
    FRobotMeshBinding Link;Link.Mesh=Cube;Link.JointIndex=0;
    Link.RelativePoseCm=FTransform(FQuat::Identity,FVector(100,0,50),FVector(.4,.2,.2));Arm->MeshBindings.Add(Link);
    Arm->ToolInLastJointCm=FTransform(FVector(100,0,50));
    TestTrue(TEXT("synthetic arm initializes"),Arm->InitializeArm());
    auto* Floor=Box(FVector(0,0,-5),FVector(10,10,.1));
    auto* Obstacle=Box(FVector(70,70,50),FVector(.15,.15,1));
    FRobotMotionScene Scene;Scene.Capture(*Arm,nullptr,false);FString Reason;
    const auto Start=Arm->GetAnglesRadians();auto Goal=Start;Goal[0]=PI/2;
    auto Clear=[&](const auto& Q){return Scene.Clear(*Arm,Q,Reason);};
    TestTrue(TEXT("floor below arm permits start"),Clear(Start));
    TestTrue(TEXT("endpoint beyond obstacle is free"),Clear(Goal));
    TestFalse(TEXT("scene mesh blocks intermediate link even with UE collision disabled"),FRobotMotionPlanner::Edge(Start,Goal,Clear));
    Obstacle->SetActorLocation(FVector(500,500,50));Scene.RefreshTransforms();
    TestTrue(TEXT("relocated obstacle changes path result"),FRobotMotionPlanner::Edge(Start,Goal,Clear));
    Floor->SetActorLocation(FVector(0,0,50));Scene.RefreshTransforms();
    TestFalse(TEXT("raised terrain blocks link"),Clear(Start));
    Floor->SetActorLocation(FVector(0,0,-5));
    auto* Part=Box(FVector(100,0,50),FVector(.2,.6,.2));
    Obstacle->SetActorLocation(FVector(100,28,50));
    Scene.Capture(*Arm,Part->GetRootComponent(),true);
    TestFalse(TEXT("carried part collides when link alone is clear"),Clear(Start));
    Part->SetActorHiddenInGame(true);Scene.Capture(*Arm,nullptr,false);
    TestTrue(TEXT("same pose without payload clears obstacle"),Clear(Start));
    Obstacle->SetActorLocation(FVector(100,0,50));Obstacle->SetActorScale3D(FVector(2));Scene.RefreshTransforms();
    TestFalse(TEXT("fully enclosed link is blocked"),Clear(Start));
    const auto Before=Arm->CurrentAnglesDegrees;
    TestTrue(TEXT("command queues without synchronous search"),Arm->CommandJointDegrees({20,0,0,0,0,0}));
    TestTrue(TEXT("pending plan cannot report arrival"),Arm->IsMotionPlanning()&&!Arm->IsAtJointTarget());
    TestFalse(TEXT("blocked start rejected by planning/execution guard"),Arm->StepControl(1./120));
    TestTrue(TEXT("rejection never teleports the arm"),Before==Arm->CurrentAnglesDegrees);
    Obstacle->SetActorLocation(FVector(500,500,50));Obstacle->SetActorScale3D(FVector(.15));
    TestTrue(TEXT("new clear request can be planned"),Arm->CommandJointDegrees({20,0,0,0,0,0}));
    for(int I=0;I<5000&&Arm->IsMotionPlanning()&&!Arm->bMotionBlocked;++I)Arm->StepControl(1./120);
    TestTrue(TEXT("sliced plan becomes executable"),!Arm->IsMotionPlanning()&&!Arm->bMotionBlocked);
    const auto BeforeObstacle=Arm->CurrentAnglesDegrees;
    Obstacle->SetActorLocation(FVector(100,0,50));
    TestFalse(TEXT("new obstacle stops PID before accepting penetrating state"),Arm->StepControl(.1));
    TestTrue(TEXT("runtime collision retains previous joint pose"),BeforeObstacle==Arm->CurrentAnglesDegrees);
    Obstacle->SetActorLocation(FVector(500,500,50));
    TestTrue(TEXT("next request accepted before stop"),Arm->CommandJointDegrees({0,0,0,0,0,0}));
    Arm->Halt();TestFalse(TEXT("stop cancels pending plan"),Arm->IsMotionPlanning());
    FTransform Tilted=Arm->GetTCPWorld();Tilted.SetRotation(FQuat(FVector::UpVector,PI/6)*Tilted.GetRotation());
    TestFalse(TEXT("linear insertion cannot rotate to reach its target"),Arm->CommandLinearTCPWorld(Tilted));
    TestFalse(TEXT("rejected tilt never launches detour planning"),Arm->IsMotionPlanning());
    Arm->SetActorRotation(FRotator(0,90,0));Scene.Capture(*Arm,nullptr,false);
    TestTrue(TEXT("base rotation changes scene collision geometry"),Clear(Start));
    Arm->SetActorRotation(FRotator::ZeroRotator);Obstacle->SetActorLocation(FVector(500,500,50));
    auto* Volume=World->SpawnActor<AActor>();auto* VolumeBox=NewObject<UBoxComponent>(Volume);
    Volume->AddInstanceComponent(VolumeBox);Volume->SetRootComponent(VolumeBox);VolumeBox->SetBoxExtent(FVector(10));
    VolumeBox->SetCollisionEnabled(ECollisionEnabled::QueryOnly);VolumeBox->SetCollisionResponseToAllChannels(ECR_Block);
    VolumeBox->RegisterComponent();Volume->SetActorLocation(FVector(100,0,50));Scene.Capture(*Arm,nullptr,false);
    TestFalse(TEXT("non-mesh collision volumes block motion"),Clear(Start));
    Volume->SetActorHiddenInGame(true);Scene.Capture(*Arm,nullptr,false);
    TestFalse(TEXT("hidden blocking volumes still block motion"),Clear(Start));
    VolumeBox->SetCollisionResponseToAllChannels(ECR_Ignore);VolumeBox->SetCollisionResponseToChannel(ECC_Visibility,ECR_Block);
    Scene.Capture(*Arm,nullptr,false);TestTrue(TEXT("visibility-only sensor proxy is not an obstacle"),Clear(Start));
    Arm->MeshBindings[0].Mesh=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cylinder"));
    Arm->MeshBindings[0].RelativePoseCm=FTransform(FRotator(0,0,90),FVector(100,0,50),FVector(.66,.66,.16));
    Arm->InitializeArm();Obstacle->SetActorLocation(FVector(130,0,80));Obstacle->SetActorScale3D(FVector(.04));
    Scene.Capture(*Arm,nullptr,false);
    TestTrue(TEXT("wheel bounding-box corner does not falsely collide outside tire surface"),Clear(Start));
    Obstacle->SetActorLocation(FVector(500,500,50));Floor->SetActorScale3D(FVector(40,20,.3));Floor->SetActorLocation(FVector(0,0,-15));
    Arm->MeshBindings[0].RelativePoseCm.SetLocation(FVector(100,0,33.7));Arm->InitializeArm();Scene.Capture(*Arm,nullptr,false);
    TestTrue(TEXT("nonuniform floor and rotated wheel preserve real clearance"),Clear(Start));
    Arm->MeshBindings[0].RelativePoseCm.SetLocation(FVector(100,0,30));Arm->InitializeArm();Scene.Capture(*Arm,nullptr,false);
    TestFalse(TEXT("same wheel lowered into floor is blocked"),Clear(Start));
    Arm->MeshBindings[0].RelativePoseCm.SetLocation(FVector(100,0,50));Arm->InitializeArm();
    auto* OtherArm=World->SpawnActor<ARobotArm>();OtherArm->Joints=Arm->Joints;
    OtherArm->InitialAnglesDegrees=Arm->InitialAnglesDegrees;OtherArm->MeshBindings=Arm->MeshBindings;
    OtherArm->InitializeArm();Scene.Capture(*Arm,nullptr,false);
    TestFalse(TEXT("overlapping robot instances are physical obstacles"),Clear(Start));
    OtherArm->SetActorLocation(FVector(400,0,0));Scene.RefreshTransforms();
    TestTrue(TEXT("separated robot instances release the same pose"),Clear(Start));
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotInsertionReferenceTests,"DigitalTwin.Assembly.InsertionReference",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRobotInsertionReferenceTests::RunTest(const FString&)
{
    FRobotMotionTrajectory T;T.bLinear=true;T.Duration=4;
    T.StartTCP=FTransform(FQuat(FVector::UpVector,.7),FVector(80,120,50));
    T.GoalTCP=T.StartTCP;T.GoalTCP.AddToTranslation(FVector(12,-16,0));
    T.Samples={{0.,0.,0.,0.,0.,0.},{.2,0.,0.,0.,0.,0.},{.4,0.,0.,0.,0.,0.}};
    TArray<double> Q,V;T.Reference(2,Q,V);
    TestTrue(TEXT("intermediate knot is not a stop"),FMath::IsNearlyEqual(Q[0],.2,1.e-9)&&V[0]>0);
    T.Reference(4,Q,V);TestTrue(TEXT("end reference settles without snap"),FMath::IsNearlyEqual(Q[0],.4,1.e-9)&&V[0]==0);
    auto Middle=T.StartTCP;Middle.AddToTranslation(FVector(6,-8,0));
    TestTrue(TEXT("translated/rotated assembly accepts axial progress"),T.InCorridor(Middle));
    auto Bad=Middle;Bad.AddToTranslation(FVector(0,0,1));
    TestFalse(TEXT("lateral insertion deviation blocked"),T.InCorridor(Bad));
    Bad=Middle;Bad.SetRotation(FQuat(FVector::RightVector,FMath::DegreesToRadians(2.))*Bad.GetRotation());
    TestFalse(TEXT("mid-insertion tilt blocked"),T.InCorridor(Bad));
    TestTrue(TEXT("checking a path does not change the requested mounting pose"),T.GoalTCP.GetLocation().Equals(FVector(92,104,50)));
    return true;
}
#endif

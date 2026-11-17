#include "AssemblyModelContracts.h"
#include "ReducedPartPhysics.h"
#include "DoorRigidBody.h"
#include "AssemblyOperation.h"
#include "RobotTorquePlant.h"
#include "RobotMotionPlanner.h"
#include "Misc/AutomationTest.h"
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAssemblyBoundaryTests,"DigitalTwin.Assembly.ModelBoundaries",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAssemblyBoundaryTests::RunTest(const FString&)
{
    const FAssemblyDimensionCriterion Recipe{120,1};
    TestTrue(TEXT("nominal part"),Recipe.Accepts(120,true));
    TestFalse(TEXT("wrong part without a fault flag"),Recipe.Accepts(130,true));
    TestFalse(TEXT("missing observation"),Recipe.Accepts(120,false));
    TestFalse(TEXT("unknown recipe cannot self-calibrate"),FAssemblyDimensionCriterion{}.Accepts(120,true));
    TestFalse(TEXT("NaN measurement"),Recipe.Accepts(std::numeric_limits<double>::quiet_NaN(),true));
    // Independent analytic solution: z(1)=-g/2, vz(1)=-g, regardless of h.
    for(int Steps:{60,120,240})
    {
        FVector V(2,0,0),P(0,0,0);
        for(int I=0;I<Steps;++I)P+=ReducedPartPhysics::FreeFlightDisplacement(V,1.0/Steps);
        TestTrue(TEXT("analytic gravity and inherited release velocity"),P.Equals(FVector(2,0,-4.905),1.e-9));
        TestTrue(TEXT("analytic velocity"),V.Equals(FVector(2,0,-9.81),1.e-9));
    }
    const FVector Incoming(1,0,-2);
    const auto Out=ReducedPartPhysics::ContactVelocity(Incoming,FVector::UpVector);
    TestTrue(TEXT("passive collision does not add kinetic energy"),Out.SizeSquared()<=Incoming.SizeSquared());
    TestTrue(TEXT("restitution normal speed"),FMath::IsNearlyEqual(Out.Z,0.3,1.e-9));
    TestTrue(TEXT("unilateral contact preserves separating velocity"),ReducedPartPhysics::ContactVelocity(FVector(0,0,1),FVector::UpVector)==FVector(0,0,1));
    FAssemblyOperationLedger Ledger;FAssemblyOperationRequest Request;
    Request.Id={Ledger.Epoch,1};Request.ProductId=TEXT("product-A");Request.PartId=TEXT("door-A");
    Request.EquipmentId=TEXT("arm-A");Request.TaskId=TEXT("handoff");Request.SourcePort=TEXT("rack");Request.TargetPort=TEXT("fixture");
    TestTrue(TEXT("accept explicit request"),Ledger.Accept(Request));
    TestFalse(TEXT("duplicate active rejected"),Ledger.Accept(Request));
    TestTrue(TEXT("one terminal result"),Ledger.Finish(Request.Id,EFactoryCommandStatus::Completed,EAssemblyQuality::Pass,true,TEXT("supported")));
    TestFalse(TEXT("terminal cannot repeat"),Ledger.Finish(Request.Id,EFactoryCommandStatus::Completed,EAssemblyQuality::Pass,true,TEXT("duplicate")));
    TestTrue(TEXT("bolts unevaluated"),Ledger.Results[0].FasteningQuality==EAssemblyQuality::NotEvaluated);
    Ledger.Reset();TestFalse(TEXT("stale epoch rejected"),Ledger.Accept(Request));
    Request.Id={Ledger.Epoch,1};Request.PartId=TEXT("door-B");TestTrue(TEXT("next part accepted"),Ledger.Accept(Request));
    FDoorRigidBodySettings S;FDoorRigidBody Body;Body.bActive=true;
    for(int I=0;I<600;++I)Body.Step(S,FTransform::Identity,FTransform::Identity,false,true,1./120);
    TestTrue(TEXT("support equilibrium mg/k"),FMath::IsNearlyEqual(Body.Pose.GetLocation().Z/100,-S.MassKg*9.81/S.SupportStiffnessNm,1.e-5));
    TestTrue(TEXT("support reaction mg"),FMath::IsNearlyEqual(Body.SupportForceN.Z,S.MassKg*9.81,0.1));
    FDoorRigidBody Weak;S.GripNormalN=10;
    for(int I=0;I<120;++I)Weak.Step(S,FTransform::Identity,FTransform::Identity,true,false,1./120);
    TestTrue(TEXT("insufficient grip causes physical fall"),Weak.Pose.GetLocation().Z<-400);
    FRobotJointDefinition J;J.MinRadians=-100;J.MaxRadians=100;FRobotJointState Q;double Drive=0;
    for(int I=0;I<120;++I)FRobotTorquePlant::Step(J,0,0,4,10,-20,1./120,Q,Drive);
    TestTrue(TEXT("overload defeats torque hold"),Q.AngleRadians<-0.1&&FMath::Abs(Drive)<=10);
    TArray<FRobotJointDefinition> Chain;Chain.SetNum(6);
    for(auto& Joint:Chain){Joint.MinRadians=-1;Joint.MaxRadians=1;}
    const TArray<double> From={-.8,0,0,0,0,0},To={.8,0,0,0,0,0};
    auto Free=[](const TArray<double>& Angles){return FMath::Square(Angles[0])+FMath::Square(Angles[1])>.09;};
    TestFalse(TEXT("valid endpoints do not authorize crossing an obstacle"),FRobotMotionPlanner::Edge(From,To,Free));
    TArray<TArray<double>> Path,Repeat;
    TestTrue(TEXT("bounded planner finds a detour"),FRobotMotionPlanner::Plan(Chain,From,To,Free,Path));
    auto Prev=From;for(const auto& Point:Path){TestTrue(TEXT("every returned edge checked"),FRobotMotionPlanner::Edge(Prev,Point,Free));Prev=Point;}
    TestTrue(TEXT("detour ends at requested joints"),Prev==To);
    FRobotMotionPlanner::Plan(Chain,From,To,Free,Repeat);
    TestTrue(TEXT("planner repeatability"),Path==Repeat);
    TestFalse(TEXT("blocked goal rejects without unsafe fallback"),FRobotMotionPlanner::Plan(Chain,From,{0,0,0,0,0,0},Free,Repeat));
    TestTrue(TEXT("rejected request has no path"),Repeat.IsEmpty());
    TestFalse(TEXT("disconnected free endpoints do not produce an unsafe fallback"),FRobotMotionPlanner::Plan(Chain,From,To,
        [](const auto& Angles){return FMath::Abs(Angles[0])>.1;},Repeat));
    return true;
}
#endif

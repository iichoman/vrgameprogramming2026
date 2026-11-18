#include "AssemblyContactBody.h"
#include "WheelGripPhysics.h"
#include "Misc/AutomationTest.h"
#include <limits>
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAssemblyContactTests,"DigitalTwin.Assembly.ContactPhysics",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAssemblyContactTests::RunTest(const FString&)
{
    FWheelContactGrip Normal,Weak,Heavy;FTransform A,B,H;
    double PeakCapacity=0;
    for(int I=0;I<600;++I){Normal.Step(FTransform::Identity,18,.65,1./120,A);Weak.Step(FTransform::Identity,18,.08,1./120,B);
        Heavy.Step(FTransform::Identity,90,.65,1./120,H);PeakCapacity=FMath::Max(PeakCapacity,Heavy.Body.GripFrictionCapacityN);}
    TestTrue(TEXT("finite pads support normal wheel"),Normal.Body.GripContacts>0&&FMath::Abs(A.GetLocation().Z)<2);
    TestTrue(TEXT("low friction leaves finite pad and falls without release command"),Weak.Body.GripContacts==0&&B.GetLocation().Z<-100);
    TestTrue(TEXT("wheel has positive angular inertia"),Normal.Body.Inertia.GetMin()>0);
    TestTrue(TEXT("force controlled jaws cannot exceed actuator friction capacity"),PeakCapacity<=1200*.65+.001);
    TestTrue(TEXT("overweight wheel naturally leaves finite pads"),Heavy.Body.GripContacts==0&&H.GetLocation().Z<-100);
    FWheelContactGrip Placed;FWheelContactProperties Recipe;FWheelJawSettings Tool;
    const FTransform Station(FRotator(0,180,0),FVector(600,700,100));FTransform PlacedPose;
    for(int I=0;I<600;++I)Placed.Step(Station,Recipe,Tool,1./120,PlacedPose);
    TestTrue(TEXT("same tool and part recipe is invariant under station yaw and translation"),PlacedPose.GetRelativeTransform(Station).Equals(A,.001));
    Recipe.MassKg=27;Recipe.OuterRadiusM=.4;Recipe.InnerRadiusM=.2;Recipe.WidthM=.3;
    FWheelContactGrip Sized;Sized.Step(Station,Recipe,Tool,1./120,PlacedPose);
    TestTrue(TEXT("bound part dimensions determine annular inertia"),FMath::IsNearlyEqual(Sized.Body.Inertia.Y,.5*27*(.4*.4+.2*.2),1.e-8));
    Tool.MaxStepSeconds=std::numeric_limits<double>::quiet_NaN();
    TestFalse(TEXT("invalid physics step rejected"),Sized.Step(Station,Recipe,Tool,1./120,PlacedPose));
    for(double Step:{.001,.0005})
    {
        FAssemblyContactBody Part;Part.MaxStep=Step;FAssemblyContact Pad;Pad.Region=TEXT("support");Pad.RadiusM=.1;
        Pad.Stiffness=300000;Pad.Damping=2*FMath::Sqrt(Part.Mass*Pad.Stiffness);
        for(int I=0;I<240;++I)Part.Step({Pad},FTransform::Identity,FTransform::Identity,FTransform::Identity,1./120);
        TestTrue(TEXT("unilateral support balances gravity"),FMath::IsNearlyEqual(Part.SupportForce.Z,Part.Mass*9.81,.5));
        TestTrue(TEXT("support penetration matches mg/k"),FMath::IsNearlyEqual(Part.Pose.GetLocation().Z/100,-Part.Mass*9.81/Pad.Stiffness,.00001));
        const double Before=Part.Pose.GetLocation().Z;
        for(int I=0;I<120;++I)Part.Step({},FTransform::Identity,FTransform::Identity,FTransform::Identity,1./120);
        TestTrue(TEXT("opening fixture removes support without pose reset"),Part.Pose.GetLocation().Z<Before-400);
    }
    FAssemblyContactBody Bad;Bad.PartId=TEXT("door-A");FAssemblyContact Bolt;Bolt.Region=TEXT("upper hinge / body");
    Bolt.bBilateral=true;Bolt.Stiffness=100000;Bolt.YieldN=1000;Bolt.RadiusM=.1;Bolt.DebrisM=.02;
    Bad.Step({Bolt},FTransform::Identity,FTransform::Identity,FTransform::Identity,.001);
    TestTrue(TEXT("obstructed fastening records localized overload"),Bad.Damage.Num()==1&&Bad.Damage[0].PartId==TEXT("door-A")&&Bad.Damage[0].Region==Bolt.Region);
    Bolt.CounterpartId=TEXT("body-A");Bolt.CounterpartYieldN=1500;
    Bad.Step({Bolt},FTransform::Identity,FTransform::Identity,FTransform::Identity,.001);
    TestTrue(TEXT("body and part damage identified independently"),Bad.Damage.ContainsByPredicate([](const auto& D){return D.PartId==TEXT("body-A")&&D.bCounterpart;})
        &&Bad.Damage.ContainsByPredicate([](const auto& D){return D.PartId==TEXT("door-A")&&!D.bCounterpart;}));
    const FTransform MovedPart(FRotator(0,90,0),FVector(200,300,100));
    const FTransform MovedSupport(FRotator(90,0,0),FVector(-100,50,80));
    for(const auto& D:Bad.Damage)
    {
        const FVector Expected=D.bCounterpart?MovedSupport.TransformPosition(FVector(0,0,2)):MovedPart.GetLocation();
        TestTrue(TEXT("damage marker follows its own part or support, including obstruction height"),
            D.MarkerWorldPosition(MovedPart,MovedSupport,FTransform::Identity).Equals(Expected,.0001));
    }
    FAssemblyContactBody ToolDamage;ToolDamage.PartId=TEXT("door-B");
    FAssemblyContact Jaw;Jaw.Region=TEXT("jaw pad");Jaw.CounterpartId=TEXT("gripper-B");
    Jaw.PointCm=FVector(5,10,0);Jaw.bGrip=true;Jaw.CounterpartYieldN=100;
    ToolDamage.RecordDamage(Jaw,200,true);
    TestTrue(TEXT("gripper damage follows the tool instead of the fixture"),ToolDamage.Damage.Num()==1&&
        ToolDamage.Damage[0].MarkerWorldPosition(MovedPart,FTransform::Identity,MovedSupport)
            .Equals(MovedSupport.TransformPosition(Jaw.PointCm),.0001));
    FAssemblyContactBody Vacuum;TArray<FAssemblyContact> Cups;
    for(double X:{-16.,16.})for(double Z:{-16.,16.})
    {FAssemblyContact C;C.PointCm=FVector(X,26,Z-19);C.Normal=FVector::RightVector;C.bGrip=C.bAdhesive=true;
     C.Stiffness=100000;C.Damping=1581;C.PreloadN=600;C.RadiusM=.03;Cups.Add(C);}
    for(int I=0;I<240;++I)Vacuum.Step(Cups,FTransform::Identity,FTransform::Identity,FTransform::Identity,1./120);
    TestTrue(TEXT("eccentric door held by four finite suction pads"),Vacuum.GripContacts==4&&FMath::Abs(Vacuum.GripForce.Z-25*9.81)<1);
    for(auto& C:Cups)C.PreloadN=2.5;
    for(int I=0;I<120;++I)Vacuum.Step(Cups,FTransform::Identity,FTransform::Identity,FTransform::Identity,1./120);
    TestTrue(TEXT("loss of vacuum capacity produces natural separation"),Vacuum.GripContacts==0&&Vacuum.Pose.GetLocation().Z<-100);
    FAssemblyContactBody Off;Off.Pose.SetLocation(FVector(20,0,0));Bolt.bBilateral=false;Bolt.DebrisM=0;
    Off.Step({Bolt},FTransform::Identity,FTransform::Identity,FTransform::Identity,.01);
    TestTrue(TEXT("misplaced panel cannot be attracted to fixture"),Off.SupportContacts==0&&Off.Velocity.Z<0);
    FAssemblyContactBody Spin;Spin.Omega=FVector(0,0,1);Spin.Step({},FTransform::Identity,FTransform::Identity,FTransform::Identity,.01);
    TestTrue(TEXT("angular state is integrated"),Spin.Pose.GetRotation().AngularDistance(FQuat::Identity)>.009);
    FAssemblyContactBody Impact;Impact.Velocity=FVector(2,0,-3);Impact.Omega=FVector(0,2,0);
    const double BeforeEnergy=.5*Impact.Mass*Impact.Velocity.SizeSquared()+.5*FVector::DotProduct(Impact.Inertia*Impact.Omega,Impact.Omega);
    Impact.ResolveImpact(FVector::UpVector,FVector(.1,0,-.3));
    const double AfterEnergy=.5*Impact.Mass*Impact.Velocity.SizeSquared()+.5*FVector::DotProduct(Impact.Inertia*Impact.Omega,Impact.Omega);
    TestTrue(TEXT("contact impulse with angular inertia is passive"),AfterEnergy<=BeforeEnergy);
    return true;
}
#endif

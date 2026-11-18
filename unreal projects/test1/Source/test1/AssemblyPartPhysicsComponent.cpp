#include "AssemblyPartPhysicsComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "ReducedPartPhysics.h"

UAssemblyPartPhysicsComponent::UAssemblyPartPhysicsComponent(){PrimaryComponentTick.bCanEverTick=false;}
void UAssemblyPartPhysicsComponent::Initialize(USceneComponent* Root,const FString& Id,const FTransform& Offset,double Mass,const FVector& Inertia)
{
    RenderRoot=Root;BodyInRender=Offset;Body=FAssemblyContactBody();Body.PartId=Id;PartIdentity=Id;Body.Mass=Mass;Body.Inertia=Inertia;
    Body.Pose=Offset*Root->GetComponentTransform();Grip=PreviousGrip=Support=Body.Pose;bActive=true;
    TArray<USceneComponent*> Children;Root->GetChildrenComponents(true,Children);Children.Add(Root);FBox Bounds(ForceInit);
    for(auto* Child:Children)if(auto* Mesh=Cast<UStaticMeshComponent>(Child))if(Mesh->GetStaticMesh()&&Mesh->IsVisible()&&!Mesh->bHiddenInGame)
        Bounds+=Mesh->GetStaticMesh()->GetBoundingBox().TransformBy(Mesh->GetComponentTransform().ToMatrixWithScale()*Body.Pose.ToInverseMatrixWithScale());
    if(Bounds.IsValid)
    {
        CollisionHalfSizeCm=Bounds.GetExtent().ComponentMax(FVector(1));
        const auto E=CollisionHalfSizeCm/100;
        Body.Inertia=Mass/3*FVector(E.Y*E.Y+E.Z*E.Z,E.X*E.X+E.Z*E.Z,E.X*E.X+E.Y*E.Y);
    }
}
bool UAssemblyPartPhysicsComponent::Advance(double Dt)
{
    if(!bActive||!RenderRoot.IsValid())return true;
    if(Dt==0)return true;
    const FVector Before=Body.Pose.GetLocation();
    TArray<FAssemblyContact> Enabled;
    for(const auto& C:Contacts)if(C.bGrip||(C.bBilateral?bFastenersClosed:bFixtureClosed))Enabled.Add(C);
    if(!Body.Step(Enabled,PreviousGrip,Grip,Support,Dt))return false;
    PreviousGrip=Grip;
    // Transport collisions remain protected by the full CAD motion planner.
    // Physical free/contact motion uses a swept proxy and inherits angular speed.
    if(GetWorld())
    {
        FCollisionQueryParams P(SCENE_QUERY_STAT(AssemblyPartContact),false,RenderRoot->GetOwner());
        if(Vehicle.IsValid())P.AddIgnoredActor(Vehicle.Get());if(Robot.IsValid())P.AddIgnoredActor(Robot.Get());
        FHitResult Hit;
        if(GetWorld()->SweepSingleByChannel(Hit,Before,Body.Pose.GetLocation(),Body.Pose.GetRotation(),ECC_PhysicsBody,
            FCollisionShape::MakeBox(CollisionHalfSizeCm),P))
        {
            const FVector N=(Hit.bStartPenetrating?Hit.Normal:Hit.ImpactNormal).GetSafeNormal();
            const double Closing=FMath::Max(0.,-FVector::DotProduct(Body.Velocity,N));
            FAssemblyContact C;C.Region=TEXT("충돌 접촉부");C.PointCm=Body.Pose.InverseTransformPosition(Hit.ImpactPoint);C.YieldN=5000;
            C.Stiffness=300000; // Equivalent local compliance: peak from mv²/2=kx²/2, independent of render/outer step.
            Body.RecordDamage(C,Closing*FMath::Sqrt(Body.Mass*C.Stiffness));
            Body.Pose.SetLocation(Hit.bStartPenetrating?Before+N*(Hit.PenetrationDepth+.01):Hit.Location+N*.01);
            Body.ResolveImpact(N,(Hit.ImpactPoint-Body.Pose.GetLocation())/100);
        }
        if(bShowContacts&&LastDrawFrame!=GFrameCounter)for(const auto& C:Enabled)
        {
            const auto& Frame=C.bGrip?Grip:Support;
            const FVector At=Frame.TransformPosition(C.PointCm);
            const auto Q=FRotationMatrix::MakeFromZ(Frame.TransformVectorNoScale(C.Normal)).ToQuat();
            DrawDebugBox(GetWorld(),At,FVector(C.RadiusM*100,C.RadiusM*100,.2),Q,C.bGrip?FColor::Cyan:FColor::Green,false,.04f,0,.3f);
        }
        if(bShowDamage&&LastDrawFrame!=GFrameCounter)for(const auto& D:Body.Damage)
        {const auto At=D.bCounterpart?D.WorldPointCm:Body.Pose.TransformPosition(D.LocalPointCm);DrawDebugSphere(GetWorld(),At,5,12,FColor::Red,false,.08f);
         DrawDebugString(GetWorld(),At+FVector(0,0,8),D.PartId+TEXT(" · ")+D.Region+TEXT(" 손상 추정"),nullptr,FColor::Red,.08f);}
        LastDrawFrame=GFrameCounter;
    }
    ObservedPose=Body.Pose;ObservedVelocityMps=Body.Velocity;ObservedAngularVelocity=Body.Omega;
    ObservedMassKg=Body.Mass;ObservedInertiaKgM2=Body.Inertia;ObservedSupportContacts=Body.SupportContacts;DamageReport=DamageSummary();
    ObservedSeatInterferenceMm=Body.SeatInterferenceMm;ObservationTimeSeconds=Body.Time;
    RenderRoot->SetWorldTransform(BodyInRender.Inverse()*Body.Pose);
    return true;
}
FString UAssemblyPartPhysicsComponent::DamageSummary() const
{
    FString Result;for(const auto& D:Body.Damage)Result+=FString::Printf(TEXT("%s · %s: %.0fN / 허용 %.0fN · 탄성 변위 추정 %.2fmm\n"),*D.PartId,*D.Region,D.PeakForceN,D.LimitN,D.ElasticDeflectionMm);
    return Result;
}

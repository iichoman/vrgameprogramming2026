#pragma once
#include "Components/ActorComponent.h"
#include "AssemblyContactBody.h"
#include "AssemblyPartPhysicsComponent.generated.h"

// Part-owned state. Cells write commands and read observations; a single line
// clock advances every part, including completed/released/failed operations.
UCLASS()
class TEST1_API UAssemblyPartPhysicsComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UAssemblyPartPhysicsComponent();
    FAssemblyContactBody Body;
    TArray<FAssemblyContact> Contacts;
    FTransform BodyInRender, Grip, PreviousGrip, Support;
    FVector CollisionHalfSizeCm=FVector(40,6,35);
    TWeakObjectPtr<USceneComponent> RenderRoot;
    TWeakObjectPtr<AActor> Vehicle, Robot;
    bool bActive=false, bShowDamage=true, bShowContacts=true;
    UPROPERTY(EditAnywhere, Category="Physics|Commands") bool bFixtureClosed=true;
    UPROPERTY(EditAnywhere, Category="Physics|Commands") bool bFastenersClosed=true;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") FString PartIdentity;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") FTransform ObservedPose;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") FVector ObservedVelocityMps;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") FVector ObservedAngularVelocity;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") double ObservedMassKg=0;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") FVector ObservedInertiaKgM2;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") int ObservedSupportContacts=0;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") double ObservedSeatInterferenceMm=0;
    UPROPERTY(VisibleAnywhere, Category="Physics|Part") double ObservationTimeSeconds=0;
    UPROPERTY(VisibleAnywhere, Category="Physics|Damage") FString DamageReport;
    uint64 LastDrawFrame=MAX_uint64;
    void Initialize(USceneComponent* Root,const FString& Id,const FTransform& Offset,double Mass,const FVector& Inertia);
    bool Advance(double Dt);
    FString DamageSummary() const;
};

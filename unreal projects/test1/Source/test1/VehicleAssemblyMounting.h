#pragma once
#include "CamryAssembly.h"
#include "VehicleProcessTest.h"
#include "Components/SceneComponent.h"
#include "EngineUtils.h"

/** Lift-supported assembly pose, separate from sensor readings and criteria.
 * The imported running gear is a merged rigid module. This is a 30 mm fixture
 * pose approximation, NOT a spring/force model or a measured Camry droop limit.
 * Keep the hub target, visible running gear and fastened wheel in one frame. */
namespace VehicleAssemblyMounting
{
inline constexpr double RearExtensionCm=3.;
inline USceneComponent* RearFrame(const ACamryAssembly& Car)
{
    return FindObjectFast<USceneComponent>(const_cast<ACamryAssembly*>(&Car),TEXT("AssemblyRearHubFrame"));
}
inline FVector HubOffset(const ACamryAssembly& Car,int WheelIndex)
{
    const auto* Frame=RearFrame(Car);
    return Frame&&WheelIndex>=0&&WheelIndex<4&&WheelIndex%2==1?Frame->GetRelativeLocation():FVector::ZeroVector;
}
inline bool SetRearExtension(ACamryAssembly& Car,double ExtensionCm)
{
    if(!FMath::IsFinite(ExtensionCm)||ExtensionCm<0||ExtensionCm>RearExtensionCm||!Car.GetBodyFrame())return false;
    AVehicleProcessModule* Gear=nullptr;
    for(TActorIterator<AVehicleProcessModule> It(Car.GetWorld());It;++It)
        if(It->VehicleFrame==&Car&&It->ModuleId==TEXT("RunningGear_Rear")&&It->bHasReferencePose){Gear=*It;break;}
    // Legacy placeholder maps have no merged running gear and keep their pose.
    if(!Gear){if(auto* Frame=RearFrame(Car))Frame->SetRelativeLocation(FVector::ZeroVector);return true;}
    auto* Frame=RearFrame(Car);
    if(!Frame)
    {
        Frame=NewObject<USceneComponent>(&Car,TEXT("AssemblyRearHubFrame"),RF_Transient);
        Car.AddInstanceComponent(Frame);Frame->SetupAttachment(Car.GetBodyFrame());
        Frame->SetMobility(EComponentMobility::Movable);Frame->RegisterComponent();
    }
    const FVector Offset(0,0,-ExtensionCm);Frame->SetRelativeLocation(Offset);
    auto Pose=Gear->ReferencePoseInVehicle*Car.GetActorTransform();
    Pose.AddToTranslation(Car.GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(Offset));
    return Gear->SetActorTransform(Pose);
}
}

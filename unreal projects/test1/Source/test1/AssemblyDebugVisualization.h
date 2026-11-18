#pragma once
#include "AssemblyContactBody.h"
#include "DrawDebugHelpers.h"

// Rendering policy only. Visibility must never change sensing or physics.
namespace AssemblyDebugVisualization
{
inline void SensorSphere(const UWorld* World,const FVector& At,double Radius,int32 Segments,const FColor& Color)
{
    DrawDebugSphere(World,At,Radius,Segments,Color,false,0.f,SDPG_World,.35f);
}
inline void Damage(const UWorld* World,const FAssemblyContactBody& Body,
    const FTransform& Support=FTransform::Identity,const FTransform& Grip=FTransform::Identity)
{
    for(const auto& D:Body.Damage)
    {
        const FVector At=D.MarkerWorldPosition(Body.Pose,Support,Grip);
        // A fixed pixel-size dot identifies the recorded contact point even
        // when the panel/body hides it. No large sphere obscures the part.
        DrawDebugPoint(World,At,10.f,FColor::Red,false,0.f,SDPG_Foreground);
        const FVector Label=At+FVector(0,0,12);
        DrawDebugLine(World,At,Label,FColor::Red,false,0.f,SDPG_Foreground,.5f);
        DrawDebugString(World,Label,FString::Printf(TEXT("%s · %s\n%.0fN / 한계 %.0fN"),
            *D.PartId,*D.Region,D.PeakForceN,D.LimitN),nullptr,FColor::Red,0.f,true,.85f);
    }
}
}

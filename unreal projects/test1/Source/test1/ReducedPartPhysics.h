#pragma once
#include "CoreMinimal.h"

// SI translation-only model, ideal gravity and a sphere collision adapter.
// No angular inertia, rolling resistance, robot reaction load or deformation.
namespace ReducedPartPhysics
{
inline FVector FreeFlightDisplacement(FVector& VelocityMps, double Dt)
{
    const FVector Gravity(0,0,-9.81);
    const FVector Displacement = VelocityMps*Dt + Gravity*(0.5*Dt*Dt);
    VelocityMps += Gravity*Dt;
    return Displacement;
}
inline FVector ContactVelocity(FVector VelocityMps, const FVector& Normal,
    double Restitution=0.15, double Friction=0.5)
{
    const double NormalSpeed=FVector::DotProduct(VelocityMps,Normal);
    if(NormalSpeed>=0) return VelocityMps;
    const FVector Tangential=VelocityMps-Normal*NormalSpeed;
    const double Speed=Tangential.Size();
    const double Reduction=FMath::Min(Speed,Friction*(1+Restitution)*(-NormalSpeed));
    return Tangential*(Speed>1.e-12 ? (Speed-Reduction)/Speed : 0)-Normal*NormalSpeed*Restitution;
}
}

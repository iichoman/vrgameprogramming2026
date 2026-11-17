#pragma once
#include "RobotArm.h"
#include "Engine/StaticMesh.h"

/** Explicit wheel tool adapter. The imported square door tool extends below
 * the floor at the wheel TCP. This replaceable, uncalibrated rim gripper keeps
 * the existing flange/TCP frames and is included in every motion query.
 */
namespace WheelGripperGeometry
{
inline void Configure(ARobotArm& Arm)
{
    const int Index=Arm.MeshBindings.IndexOfByPredicate([](const auto& B)
        {return B.Mesh&&B.Mesh->GetPathName()==TEXT("/Game/Robot/Meshes/SM_Tool.SM_Tool");});
    if(Index==INDEX_NONE||Arm.MeshBindings.Num()+4>24)return;
    auto* Cylinder=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cylinder"));if(!Cylinder)return;
    auto Binding=[&](FVector P,FVector Size)
    {FRobotMeshBinding B;B.Mesh=Cylinder;B.JointIndex=5;B.RelativePoseCm=FTransform(FQuat::Identity,P,Size/100.)*Arm.ToolInLastJointCm;return B;};
    Arm.MeshBindings[Index]=Binding(FVector(0,0,5),FVector(44,44,4));
    Arm.MeshBindings.Add(Binding(FVector(0,0,16),FVector(10,10,18)));
    for(int I=0;I<3;++I)
    {const double A=I*2*PI/3;Arm.MeshBindings.Add(Binding(FVector(20*FMath::Cos(A),20*FMath::Sin(A),0),FVector(5,5,8)));}
    Arm.CalibrationSource+=TEXT(" Wheel tool: 44 cm plate, three rim pads, existing flange/TCP. Demonstration geometry, not OEM-calibrated.");
}
}

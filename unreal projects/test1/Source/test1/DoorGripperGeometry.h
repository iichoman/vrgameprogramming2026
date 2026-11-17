#pragma once
#include "RobotArm.h"
#include "Engine/StaticMesh.h"

// Demonstration vacuum gripper. Its plate/stem stay behind the existing TCP;
// the imported generic tool protrudes through the rear door into the body.
// Geometry and motion collision share these bindings. No OEM calibration claim.
namespace DoorGripperGeometry
{
inline void Configure(ARobotArm& Arm)
{
    const int Index=Arm.MeshBindings.IndexOfByPredicate([](const auto& B)
        {return B.Mesh&&B.Mesh->GetPathName()==TEXT("/Game/Robot/Meshes/SM_Tool.SM_Tool");});
    if(Index==INDEX_NONE||Arm.MeshBindings.Num()+5>24)return;
    auto* Cube=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube"));
    auto* Cylinder=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cylinder"));if(!Cube||!Cylinder)return;
    auto Binding=[&](UStaticMesh* Mesh,FVector P,FVector Size)
    {FRobotMeshBinding B;B.Mesh=Mesh;B.JointIndex=5;B.RelativePoseCm=FTransform(FQuat::Identity,P,Size/100.)*Arm.ToolInLastJointCm;return B;};
    Arm.MeshBindings[Index]=Binding(Cube,FVector(0,0,5),FVector(44,44,4));
    Arm.MeshBindings.Add(Binding(Cylinder,FVector(0,0,16),FVector(10,10,18)));
    for(double X:{-16.,16.})for(double Y:{-16.,16.})Arm.MeshBindings.Add(Binding(Cylinder,FVector(X,Y,0),FVector(6,6,8)));
    Arm.CalibrationSource+=TEXT(" Door vacuum tool: 44 cm plate, four pads, existing flange/TCP; demonstration geometry.");
}
}

#include "FactoryAssemblyEquipment.h"
#include "FactoryRuntime.h"
#include "RobotArm.h"
#include "Engine/World.h"

bool AFactoryAssemblyEquipment::IsAssemblyType(const FString& Id)
{ return Id==TEXT("assembly_wheel_left")||Id==TEXT("assembly_wheel_right")||Id==TEXT("assembly_door"); }

void AFactoryAssemblyEquipment::Configure(const FFactoryLayoutItem& Item,UStaticMesh*)
{
    Super::Configure(Item,nullptr);
    if(IsValid(Arm))Arm->Destroy();
    auto* Runtime=Cast<AFactoryRuntime>(GetOwner());
    const auto* Source=Runtime?Runtime->AssemblyRobotTemplate(Item.EffectiveTypeId()):nullptr;
    if(!IsValid(Source))return;
    FActorSpawnParameters P;P.Owner=this;P.SpawnCollisionHandlingOverride=ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Arm=GetWorld()->SpawnActor<ARobotArm>(ARobotArm::StaticClass(),Item.Transform(),P);
    if(!Arm)return;
    // Copy calibration data, never mutable controller/sensor state or object references.
    Arm->Joints=Source->Joints;Arm->MeshBindings=Source->MeshBindings;
    Arm->ToolInLastJointCm=Source->ToolInLastJointCm;Arm->InitialAnglesDegrees=Source->InitialAnglesDegrees;
    Arm->Kp=Source->Kp;Arm->Ki=Source->Ki;Arm->Kd=Source->Kd;Arm->MotorResponseSeconds=Source->MotorResponseSeconds;
    Arm->CalibrationSource=Source->CalibrationSource;
    Arm->AttachToActor(this,FAttachmentTransformRules::KeepWorldTransform);
    Arm->SetActorTickEnabled(false);Arm->InitializeArm();
}
void AFactoryAssemblyEquipment::Destroyed()
{
    if(IsValid(Arm)){Arm->Halt();Arm->Destroy();}Arm=nullptr;
    Super::Destroyed();
}

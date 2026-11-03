#pragma once
#include "CoreMinimal.h"
#include "FactoryEquipment.h"
#include "FactoryAssemblyEquipment.generated.h"
class ARobotArm;

/** An independently owned team robot. The line coordinates explicitly bound instances. */
UCLASS(NotBlueprintable)
class TEST1_API AFactoryAssemblyEquipment : public AFactoryEquipment
{
    GENERATED_BODY()
public:
    virtual void Configure(const FFactoryLayoutItem& Item, UStaticMesh* Replacement=nullptr) override;
    virtual void Destroyed() override;
    ARobotArm* GetArm() const { return Arm; }
    static bool IsAssemblyType(const FString& Id);
protected:
    virtual UFactoryMotionController* CreateController() override { return nullptr; }
    UPROPERTY() TObjectPtr<ARobotArm> Arm;
};

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FactoryLayoutModel.h"
#include "FactoryEquipment.generated.h"
class UStaticMeshComponent;
class UStaticMesh;
class UMaterialInterface;
class USceneComponent;
class UWidgetComponent;
class UFactoryMotionController;

/** Replaceable visual representation; geometry never defines the process recipe. */
UCLASS(NotBlueprintable)
class TEST1_API AFactoryEquipment : public AActor
{
    GENERATED_BODY()
public:
    AFactoryEquipment();
    virtual void Configure(const FFactoryLayoutItem& Item, UStaticMesh* Replacement = nullptr);
    void UpdatePlacement(const FFactoryLayoutItem& Item) { Definition=Item; SetActorTransform(Item.Transform()); }
    void SetSelected(bool bSelected, bool bInvalid = false);
    virtual void UpdateTool(const FVector& WorldPoint, double WorldYaw);
    void OrientLabel(double CameraYaw);
    virtual FVector Anchor() const;
    void SetLabelVisible(bool Visible);
    bool SolveCartesian(const FVector& WorldPoint, FVector& JointCm) const;
    FVector ForwardCartesian(const FVector& JointCm) const;
    const FFactoryLayoutItem& GetDefinition() const { return Definition; }
    UFactoryMotionController* Controller() const { return MotionController; }
    UPROPERTY(VisibleAnywhere, Category="Factory") FString ItemId;
    UPROPERTY(VisibleAnywhere, Category="Factory") TObjectPtr<USceneComponent> FunctionalRoot;
    UPROPERTY(VisibleAnywhere, Category="Factory") TObjectPtr<USceneComponent> VisualRoot;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Parts;
    UPROPERTY() TObjectPtr<UWidgetComponent> Label;
protected:
    virtual UFactoryMotionController* CreateController();
    UPROPERTY() TObjectPtr<UFactoryMotionController> MotionController;
    UStaticMeshComponent* Box(FName Name, FVector Pos, FVector Size, FLinearColor Color, bool bSelectable = true);
    FFactoryLayoutItem Definition;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Tool;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> BeamX;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> BeamY;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Mast;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Selection;
};

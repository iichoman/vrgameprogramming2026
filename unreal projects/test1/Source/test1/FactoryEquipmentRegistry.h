#pragma once
#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FactoryLayoutModel.h"
#include "FactoryEquipmentRegistry.generated.h"
class AFactoryEquipment;

/** Native data asset, no Blueprint implementation required. Stable IDs are persisted in JSON. */
UCLASS(NotBlueprintable)
class TEST1_API UFactoryEquipmentDefinition : public UDataAsset
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere) FName TypeId;
    UPROPERTY(EditAnywhere) FString NameKo;
    UPROPERTY(EditAnywhere) FString NameEn;
    UPROPERTY(EditAnywhere) FString Category = TEXT("설비");
    UPROPERTY(EditAnywhere) FString Kind = TEXT("conveyor");
    UPROPERTY(EditAnywhere) FVector2D FootprintCm = FVector2D(200,200);
    UPROPERTY(EditAnywhere) TSubclassOf<AFactoryEquipment> ActorClass;
    UPROPERTY(EditAnywhere) TObjectPtr<UStaticMesh> VisualMesh;
    UPROPERTY(EditAnywhere) FString Description;
};

UCLASS()
class TEST1_API UFactoryEquipmentRegistry : public UObject
{
    GENERATED_BODY()
public:
    void Initialize();
    bool Register(UFactoryEquipmentDefinition* Definition, FString& Error);
    const UFactoryEquipmentDefinition* Find(const FString& TypeId) const;
    const TArray<TObjectPtr<UFactoryEquipmentDefinition>>& All() const { return Definitions; }
    bool Validate(const FFactoryLayout& Layout, FString& Error) const;
private:
    UPROPERTY() TArray<TObjectPtr<UFactoryEquipmentDefinition>> Definitions;
};

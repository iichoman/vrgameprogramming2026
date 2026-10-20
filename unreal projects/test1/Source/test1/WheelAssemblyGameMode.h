#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "WheelAssemblyGameMode.generated.h"

class AWheelAssemblyCell;
class ACameraActor;
class SWidget;

UCLASS(NotBlueprintable)
class TEST1_API AWheelAssemblyPlayerController : public APlayerController
{
	GENERATED_BODY()
public:
	AWheelAssemblyPlayerController();
	AWheelAssemblyCell* GetCell() const { return Cell.Get(); }
	void StartAssembly();
	void StopAssembly();
	void ResetAssembly();
	void ResetCamera();
	void ZoomIn();
	void ZoomOut();
protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void SetupInputComponent() override;
	virtual void PlayerTick(float DeltaSeconds) override;
private:
	void UpdateCamera();
	UPROPERTY(Transient) TObjectPtr<AWheelAssemblyCell> Cell;
	UPROPERTY(Transient) TObjectPtr<ACameraActor> AssemblyCamera;
	TSharedPtr<SWidget> Panel;
	FVector Focus = FVector(0, 0, 70);
	FRotator Orbit = FRotator::ZeroRotator;
	double Distance = 750;
	FVector2D LastMouse = FVector2D::ZeroVector;
	bool bDragging = false;
};

UCLASS(NotBlueprintable)
class TEST1_API AWheelAssemblyGameMode : public AGameModeBase
{
	GENERATED_BODY()
public:
	AWheelAssemblyGameMode();
};

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "DoorAssemblyGameMode.generated.h"

class ACameraActor;
class ADoorAssemblyCell;
class SWidget;

/** Native operator controls and an orbit camera for the door assembly cell. */
UCLASS(NotBlueprintable)
class TEST1_API ADoorAssemblyPlayerController : public APlayerController
{

	GENERATED_BODY()

public:
	ADoorAssemblyPlayerController();
	ADoorAssemblyCell* GetAssemblyCell() const { return AssemblyCell.Get(); }
	void StartAssembly();
	void StopAssembly();
	void ResetAssembly();
	void ResetCamera();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void SetupInputComponent() override;
	virtual void PlayerTick(float DeltaSeconds) override;

private:
	void FindAssemblyCell();
	void UpdateCamera();
	void ZoomIn();
	void ZoomOut();

	UPROPERTY(Transient)
	TObjectPtr<ADoorAssemblyCell> AssemblyCell;

	UPROPERTY(Transient)
	TObjectPtr<ACameraActor> AssemblyCamera;

	TSharedPtr<SWidget> AssemblyPanel;
	FVector CameraFocus = FVector(40.0, -130.0, 100.0);
	FRotator OrbitRotation = FRotator::ZeroRotator;
	double OrbitDistance = 600.0;
	FVector2D LastMousePosition = FVector2D::ZeroVector;
	bool bOrbitDragging = false;
};

/** Map-specific native GameMode; simulation lives in ADoorAssemblyCell. */
UCLASS(NotBlueprintable)
class TEST1_API ADoorAssemblyGameMode : public AGameModeBase
{

	GENERATED_BODY()

public:
	ADoorAssemblyGameMode();
};

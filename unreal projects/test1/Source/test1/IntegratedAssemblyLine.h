#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "IntegratedAssemblyLine.generated.h"

class AWheelAssemblyCell;
class ADoorAssemblyCell;
class ACameraActor;
class ARobotArm;
class SWidget;

UENUM(BlueprintType)
enum class EAssemblyLineStage : uint8 { Idle, Wheels, Transfer, Door, Complete, Stopped, Fault };

/** One vehicle, two sequential stations, with the existing measured joint controllers. */
UCLASS(NotBlueprintable)
class TEST1_API AIntegratedAssemblyLine : public AActor
{
    GENERATED_BODY()
public:
    AIntegratedAssemblyLine();
    UPROPERTY(EditInstanceOnly) TObjectPtr<AWheelAssemblyCell> WheelCell;
    UPROPERTY(EditInstanceOnly) TObjectPtr<ADoorAssemblyCell> DoorCell;
    UPROPERTY(EditAnywhere) FTransform VehicleAtWheelStation;
    UPROPERTY(EditAnywhere) FTransform CarriageAtWheelStation;
    UPROPERTY(EditAnywhere) FVector DoorStationOffsetCm = FVector(1000,0,0);
    UPROPERTY(EditAnywhere) double TransferSeconds = 10;
    UPROPERTY(EditAnywhere) double DockSeconds = 1;
    UPROPERTY(VisibleAnywhere, Transient) EAssemblyLineStage Stage = EAssemblyLineStage::Idle;
    UPROPERTY(VisibleAnywhere, Transient) FString Status;
    UPROPERTY(VisibleAnywhere, Transient) double ElapsedSeconds = 0;
    UPROPERTY(VisibleAnywhere, Transient) double TransferProgress = 0;
    UFUNCTION() virtual bool ResetLine();
    UFUNCTION() virtual bool StartLine();
    UFUNCTION() virtual void StopLine();
    UFUNCTION() virtual bool AdvanceLine(double Seconds);
    UFUNCTION() virtual FString GetSummary() const;
    virtual void BeginPlay() override;
    virtual void Tick(float Seconds) override;
protected:
    FTransform DoorSupplyWorld;
    FTransform ExpectedVehicle, ExpectedCarriage;
    double TransferTime = 0;
    bool bReady = false;
    void Fail(const FString& Reason);
};

UCLASS(NotBlueprintable)
class TEST1_API AIntegratedAssemblyController : public APlayerController
{
    GENERATED_BODY()
public:
    AIntegratedAssemblyController();
    void StartAssembly();
    void StopAssembly();
    void ResetAssembly();
    void ResetCamera();
    void ZoomIn();
    void ZoomOut();
    void DemoNormal();
    void DemoJam();
    void DemoMisDock();
    void DemoDropout();
    void DemoWheelMissing();
    void DemoWheelDisplaced();
    void DemoGripFailure();
    void DemoWheelPoseDropout();
    void ArrivalCamera();
    void LeftRobotCamera();
    void RightRobotCamera();
    void DoorRobotCamera();
    FString GetCameraLabel() const;
protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void SetupInputComponent() override;
    virtual void PlayerTick(float Seconds) override;
private:
    enum class ECameraMode : uint8 { Free, Arrival, LeftRobot, RightRobot, DoorRobot };
    ECameraMode CameraMode = ECameraMode::Free;
    UPROPERTY() TObjectPtr<AIntegratedAssemblyLine> Line;
    UPROPERTY() TObjectPtr<ACameraActor> AssemblyCamera;
    TSharedPtr<SWidget> Panel;
    FVector Focus = FVector(200,0,70);
    FRotator Orbit;
    double Distance = 1900;
    FVector2D LastMouse;
    bool bDragging = false;
    void UpdateCamera();
    void FollowRobot(ARobotArm* Robot, double Side);
};

UCLASS(NotBlueprintable)
class TEST1_API AIntegratedAssemblyGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    AIntegratedAssemblyGameMode();
};

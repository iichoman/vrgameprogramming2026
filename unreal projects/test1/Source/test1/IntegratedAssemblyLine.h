#pragma once
#include "CoreMinimal.h"
#include "AssemblyOperation.h"
#include "GameFramework/Actor.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "IntegratedAssemblyLine.generated.h"

class AWheelAssemblyCell;
class ADoorAssemblyCell;
class ACameraActor;
class ARobotArm;
class AVehicleProcessModule;
class SWidget;

struct FDoorOperationSnapshot
{
    FString TaskId;
    double AlignmentMm=0, WidthCm=0, SampleTime=-1, WidthSampleTime=-1, SimulationTime=0;
    bool bAlignmentValid=false, bComplete=false;
    FString QualitySummary;
};

UENUM(BlueprintType)
enum class EAssemblyLineStage : uint8 { Idle, Wheels, Transfer, Door, Complete, Stopped, Fault, Extension };

enum class EAssemblyProcessProgress : uint8 { Running, Complete, Fault };
/** Line-owned route entry. Validate is read-only; Enter prepares a supported vehicle,
 * clears this operation's stale commands and starts its real sensor/control gates.
 * Added glass/seat operations implement this contract, never the UI button. */
struct FAssemblyProcessEntry
{
    FName Id;
    FString Label;
    TFunction<bool(FString&)> Validate;
    TFunction<bool()> Enter;
    TFunction<EAssemblyProcessProgress(double)> Advance;
    TFunction<void()> Stop;
};

/** One vehicle, two sequential stations, with the existing measured joint controllers. */
UCLASS(NotBlueprintable)
class TEST1_API AIntegratedAssemblyLine : public AActor
{
    GENERATED_BODY()
public:
    AIntegratedAssemblyLine();
    UPROPERTY(EditInstanceOnly) TObjectPtr<AWheelAssemblyCell> WheelCell;
    UPROPERTY(EditInstanceOnly) TObjectPtr<ADoorAssemblyCell> DoorCell;
    UPROPERTY(EditInstanceOnly) TObjectPtr<ARobotArm> RightDoorRobot;
    UPROPERTY(VisibleAnywhere, Transient) int32 MountedDoorCount = 0;
    UPROPERTY(VisibleAnywhere, Transient) bool bFourDoorMode = false;
    UPROPERTY(Transient) TObjectPtr<ADoorAssemblyCell> SecondDoorCell;
    // Resolved by the factory adapter. Standalone legacy maps may leave this empty.
    UPROPERTY(Transient) TArray<TObjectPtr<ARobotArm>> DoorTaskRobots;
    UPROPERTY(Transient) TArray<TObjectPtr<AVehicleProcessModule>> DoorModules;
    TMap<FString,FDoorOperationSnapshot> CompletedDoorOperations;
    TMap<int32,FVector> DoorSupplyOffsetsWorldCm;
    ADoorAssemblyCell* ActiveDoorCell(const FString& TaskId) const;
    TMap<FString,FAssemblyOperationRequest> DoorRequests;
    bool StartDoorOperation(ADoorAssemblyCell* Cell);
    void ConfigureFrontDoorSize(double Scale);
    bool DoorSnapshot(const FString& TaskId, FDoorOperationSnapshot& Out) const;
    void CaptureDoorResult(ADoorAssemblyCell* Cell);
    UPROPERTY(EditAnywhere) FTransform VehicleAtWheelStation;
    UPROPERTY(EditAnywhere) FTransform CarriageAtWheelStation;
    UPROPERTY(EditAnywhere, Category="Assembly|Placement", meta=(ClampMin="0",ClampMax="80",Units="cm")) double WheelStationLiftCm=20;
    FTransform WheelStationPose() const {auto P=VehicleAtWheelStation;P.AddToTranslation(FVector(0,0,WheelStationLiftCm));return P;}
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
    bool RegisterFollowingProcess(FAssemblyProcessEntry Entry);
    bool CanSkipToNextProcess() const;
    FString NextProcessLabel() const;
    virtual bool SkipToNextProcess();
    FString SkippedProcessSummary() const;
    bool WasProcessSkipped(FName Id) const { return SkippedProcesses.Contains(Id); }
    virtual void BeginPlay() override;
    virtual void Tick(float Seconds) override;
protected:
    UPROPERTY() TArray<TObjectPtr<class UStaticMeshComponent>> LiftSupports;
    void UpdateLiftSupports();
    FTransform DoorSupplyWorld;
    UPROPERTY(Transient) TObjectPtr<ARobotArm> LeftDoorRobot;
    FTransform BaselineTCPInLeftDoor;
    FTransform BaselineTCPInRightDoor;
    FVector FrontDoorPickupOffsets[2];
    double FrontDoorSizeScales[2]={1,1};
    FTransform SecondDoorSupplyWorld;
    FTransform ExpectedVehicle, ExpectedCarriage;
    FTransform TransferVehicleStart, TransferCarriageStart;
    double TransferTime = 0;
    bool bReady = false;
    TArray<FAssemblyProcessEntry> ProcessRoute;
    TArray<FName> SkippedProcesses;
    int32 ActiveProcessIndex = 0;
    bool EnterFollowingProcess(bool Skipping);
    bool PrepareDoorStation();
    virtual void BeforeProcessEntry(FName Id) {}
    virtual void AfterProcessEntry() {}
    void Fail(const FString& Reason);
    bool PrepareDoor(ADoorAssemblyCell* Cell, int32 Index);
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

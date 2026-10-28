#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VehicleProcessTest.h"
#include "FactoryLayoutModel.h"
#include "FactoryEditHistory.h"
#include "FactoryRuntime.generated.h"
class AFactoryEquipment;
class UFactoryWheelProcessRunner;
class UFactoryEquipmentRegistry;
class UFactoryEquipmentDefinition;
class SFactoryViewport;
class ACameraActor;
class SWidget;
class SScrollBox;
class USceneComponent;
class UStaticMeshComponent;

USTRUCT()
struct FFactoryVisualOverride
{
    GENERATED_BODY()
    // vehicle_station, wheel_supply, cartesian_handler or conveyor.
    UPROPERTY(EditAnywhere, Category="Visual") FName EquipmentKind;
    UPROPERTY(EditAnywhere, Category="Visual") TObjectPtr<UStaticMesh> Mesh;
};

enum class EFactoryRunState : uint8 { Editing, Approach, Pickup, Lift, Transfer, Align, Verify, Complete, Stopped, Fault };

/** Play-mode layout editing and one wheel transfer. Native C++ only. */
UCLASS(NotBlueprintable)
class TEST1_API AFactoryRuntime : public AActor
{
    GENERATED_BODY()
public:
    AFactoryRuntime();
    UPROPERTY(EditAnywhere, Category="Factory|Layout") FString InitialLayoutJson;
    UPROPERTY(EditAnywhere, Category="Factory|Vehicle") TArray<FVehicleModuleMesh> BodyMeshes;
    UPROPERTY(EditAnywhere, Category="Factory|Vehicle") TArray<FVehicleModuleMesh> WheelMeshes;
    UPROPERTY(EditAnywhere, Category="Factory|Vehicle") FVector WheelPivotCm = FVector(-94.52773, -80.64976, 33.70584);
    UPROPERTY(EditAnywhere, Category="Factory|Visuals") TArray<FFactoryVisualOverride> VisualOverrides;
    UPROPERTY(EditAnywhere, Category="Factory|Catalog") TArray<TObjectPtr<UFactoryEquipmentDefinition>> ExtraEquipment;
    UFUNCTION(CallInEditor, Category="Factory") void BuildPreview();
    UFUNCTION() bool VerifySetup() const;
    virtual void Tick(float DeltaSeconds) override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;

    // UI interface; edits are rejected outside Editing, including after Stop.
    bool CanEdit() const { return RunState == EFactoryRunState::Editing; }
    void SelectItem(const FString& Id);
    void SetHallSize(double LengthM, double WidthM);
    void SetSelectedPosition(double XMetres, double YMetres);
    void SetSelectedYaw(double Degrees);
    void SetSelectedSize(double XMetres, double YMetres);
    void SetSelectedRole(const FString& Role);
    void SetSelectedName(const FString& NameKo, const FString& NameEn);
    void AddItem(EFactoryItemKind Kind);
    void DeleteSelected();
    void BindSelected();
    bool SaveLayout(const FString& Slot = TEXT("A"));
    bool LoadLayout(const FString& Slot = TEXT("A"));
    void RestoreDefault();
    bool StartCycle();
    void StopCycle();
    void ResetCycle();
    void FitView();
    void FocusWorkcell();
    void ToggleView();
    void ZoomIn();
    void ZoomOut();
    FString StateText() const;
    FString BindingText() const;
    FString ValidationText() const;
    FFactoryLayoutItem* Selected() { return Layout.Find(SelectedId); }
    const FFactoryLayoutItem* Selected() const { return Layout.Find(SelectedId); }
    FFactoryLayout Layout;
    FString SelectedId;
    FString Status;
    bool bSnap = true;
    double SnapCm = 25;
    bool bCreateZone = false;
    bool bDirty = false;
    bool bTopView = true;
    EFactoryRunState RunState = EFactoryRunState::Editing;
    double SimulationSeconds = 0;
    double TravelCm = 0;
    double AlignmentMm = 0;
    double LastCycleSeconds = 0;
    bool bNeedsPanel = false;
    uint64 PanelGeneration = 0;
    void CreatePanel();
    UFactoryEquipmentRegistry* GetRegistry() const { return Registry; }
    void BeginPlacement(const FString& TypeId);
    void SelectHall();
    void UndoEdit();
    void RedoEdit();
    bool CanUndo() const { return CanEdit() && History.CanUndo(); }
    bool CanRedo() const { return CanEdit() && History.CanRedo(); }
    void CancelEdit();
    void SetZoneLabels(bool Visible);
    void SetEquipmentLabels(bool Visible);
    void SetRouteVisible(bool Visible);
    void RouteAppendSelected();
    void RouteRemoveSelected();
    void RouteMoveSelected(int32 Delta);
    void SetRouteBend(double Cm);
    FString RouteText() const;
    void ViewportPointer(FVector2D Pixel, FKey Button, bool Pressed);
    void ViewportMove(FVector2D Pixel);
    void ViewportCaptureLost(){bPanning=bOrbiting=false;if(bDragging)CancelEdit();}
    void ViewportKey(FKey Key, bool Control, bool Shift);
    void PanCamera(FVector2D Delta);
    void OrbitCamera(FVector2D Delta);
    void FocusSelected();
    bool bZoneLabels = false;
    bool bEquipmentLabels = true;
    bool bRouteVisible = true;
    bool bHallSelected = false;
    FString PendingTypeId;
    FString PaletteCategory = TEXT("전체");


private:
    friend class UFactoryWheelProcessRunner;
    UPROPERTY() TObjectPtr<UFactoryWheelProcessRunner> WheelProcess;
    uint64 RunEpoch=0, NextRequestSequence=0;
    void EnsureRegistry();
    void BeginEdit();
    void EndEdit();
    void FinishDiscreteEdit();
    void RefreshDirty();
    void LoadViewSettings();
    void SaveViewSettings();
    void EditorTick();
    FFactoryLayoutItem ResizeBox() const;
    int32 HitResizeHandle(const FVector2D& Pixel) const;
    void PreviewResize(const FVector& Point);
    void DrawEditorGuides();
    void AddRegisteredItem(const FString& TypeId, const FVector2D& Point);
    void Rebuild(bool bRefreshPanel = true);
    void BuildShell();
    void BuildVehicle();
    void ClearGenerated();
    void ApplyEditedItem();
    void RefreshHighlights();
    void RefreshPreviewWheel();
    UStaticMeshComponent* MakeMesh(USceneComponent* Parent, UStaticMesh* Mesh, FTransform Transform, bool Visible = true);
    void UpdateCamera();
    void OnPointerDown();
    void OnPointerUp();
    void OnWheelZoomIn();
    void OnWheelZoomOut();
    bool PointerOnFloor(FVector& Point) const;
    bool PointerOverPanel() const;
    void StepSimulation(double Seconds);
    void SetMotionTarget(const FVector& World, double Yaw);
    void UpdateMechanism();
    bool ValidateRun(FString& Error) const;
    FVector HubWorld() const;
    double VehicleYaw() const;
    FVector SupplyWorld() const;
    double SupplyYaw() const;
    void ChangeState(EFactoryRunState State);
    bool Obstructed(const FVector& Start, const FVector& End, FString& Obstacle) const;
    FString SlotPath(const FString& Slot) const;
    void RunChecks();
    bool RunUIChecks();
    void CaptureRuntime(float DeltaSeconds);
    UPROPERTY() TObjectPtr<UFactoryEquipmentRegistry> Registry;
    UPROPERTY() TObjectPtr<USceneComponent> HallRoot;
    UPROPERTY() TMap<FString, TObjectPtr<AFactoryEquipment>> Equipment;
    UPROPERTY() TArray<TObjectPtr<USceneComponent>> GeneratedComponents;
    UPROPERTY() TObjectPtr<USceneComponent> VehicleRoot;
    UPROPERTY() TObjectPtr<USceneComponent> WheelRoot;
    UPROPERTY() TObjectPtr<ACameraActor> ViewCamera;
    TSharedPtr<SWidget> Panel;
    TSharedPtr<SScrollBox> PanelScroll;
    TSharedPtr<SFactoryViewport> ViewportSurface;
    FFactoryEditHistory History;
    FString SavedSnapshot;
    mutable FString ValidationKey,ValidationMessage;
    FVector2D PointerPixel=FVector2D::ZeroVector;
    FVector2D LastPointerPixel=FVector2D::ZeroVector;
    bool bHasViewportPointer=false;
    bool bPanning=false, bOrbiting=false;
    int32 ResizeHandle=-1;
    FFactoryLayoutItem ResizeStartBox;
    double CameraYaw=-90, CameraPitch=-46;
    bool bInputConfigured = false;
    bool bDragging = false;
    FVector DragStart = FVector::ZeroVector;
    FVector2D DragOffset = FVector2D::ZeroVector;
    FVector2D CameraCentre = FVector2D(4200, -150);
    double CameraWidth = 2400;
    double SimRemainder = 0;
    double StateSeconds = 0;
    FVector JointPosition = FVector::ZeroVector;
    FVector JointVelocity = FVector::ZeroVector;
    FVector JointIntegral = FVector::ZeroVector;
    FVector JointTarget = FVector::ZeroVector;
    double ToolYaw = 0;
    double TargetYaw = 0;
    double YawVelocity = 0;
    double YawIntegral = 0;
    bool bHoldingWheel = false;
    bool bWheelInstalled = false;
    bool bCapturing = false;
    bool bUIProbeDone=false, bUIProbePassed=false;
    int32 CaptureStage = 0;
    double CaptureClock = 0;
};

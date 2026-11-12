#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "RobotArm.h"
#include "VehicleProcessTest.h"
#include "CamryAssembly.h"
#include "EngineUtils.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

bool AFactoryRuntime::RunModularEditorChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;
    auto Check=[&](const TCHAR* Name,bool OK){auto C=MakeShared<FJsonObject>();C->SetStringField(TEXT("check"),Name);C->SetBoolField(TEXT("passed"),OK);Checks.Add(MakeShared<FJsonValueObject>(C));Pass&=OK;UE_LOG(LogTemp,Display,TEXT("FACTORY_MODULAR %s %s: %s"),Name,OK?TEXT("PASS"):TEXT("FAIL"),*Status);};
    const auto Original=Layout;const auto Initial=Layout.ToJson();FString Error;FFactoryLayout Read;
    Check(TEXT("three_registered_bound_team_robots"),bUseAssemblyIntegration&&Layout.AssemblyBindings.Num()==3&&ValidateAssemblyBindings(Error));
    Check(TEXT("v3_roundtrip_preserves_explicit_bindings"),FFactoryLayout::FromJson(Initial,Read,Error)&&Read.ToJson()==Initial);
    auto Broken=Layout;Broken.AssemblyBindings.Add(TEXT("assembly_door"),TEXT("missing"));
    Check(TEXT("dangling_operation_binding_rejected"),!Broken.Validate(Error));
    bSnap=false;
    for(const FString Type:{TEXT("assembly_wheel_left"),TEXT("assembly_wheel_right"),TEXT("assembly_door")})
    {
        const FString Id=Layout.AssemblyBindings.FindRef(Type);SelectedId=Id;
        auto* Before=Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(Id).Get());TWeakObjectPtr<ARobotArm> OldArm=Before?Before->GetArm():nullptr;
        DeleteSelected();
        Check(*(Type+TEXT("_delete_unbinds_and_keeps_editable")),CanEdit()&&!Layout.Find(Id)&&!Layout.AssemblyBindings.Contains(Type)&&!OldArm.IsValid()&&!StartCycle());
        UndoEdit();Check(*(Type+TEXT("_undo_restores_id_binding_new_actor")),CanEdit()&&Layout.Find(Id)&&Layout.AssemblyBindings.FindRef(Type)==Id&&ValidateAssemblyBindings(Error));
        RedoEdit();UndoEdit();Check(*(Type+TEXT("_redo_undo_valid")),Layout.ToJson()==Initial);
    }
    const FString Left=Layout.AssemblyBindings.FindRef(TEXT("assembly_wheel_left"));
    AddRegisteredItem(TEXT("assembly_wheel_left"),FVector2D(1500,-700));const FString Spare=SelectedId;
    const auto* A=Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(Left).Get());
    const auto* B=Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(Spare).Get());
    Check(TEXT("same_type_instances_own_different_arms"),A&&B&&A->GetArm()!=B->GetArm()&&Layout.AssemblyBindings.FindRef(TEXT("assembly_wheel_left"))==Left);
    BindSelected();Check(TEXT("explicit_binding_replaces_previous_robot"),Layout.AssemblyBindings.FindRef(TEXT("assembly_wheel_left"))==Spare);
    UndoEdit();Check(TEXT("binding_undo_preserves_spare"),Layout.AssemblyBindings.FindRef(TEXT("assembly_wheel_left"))==Left&&Layout.Find(Spare));
    UndoEdit();Check(TEXT("spawn_undo_destroys_owned_arm"),!Layout.Find(Spare)&&Layout.ToJson()==Initial);
    SelectedId=Left;const auto Position=Selected()->Position;
    SetSelectedPosition((Position.X+10)/100,Position.Y/100);
    const auto* Moved=Cast<AFactoryAssemblyEquipment>(Equipment.FindRef(Left).Get());
    Check(TEXT("robot_base_follows_editor_movement"),Moved&&FMath::IsNearlyEqual(Moved->GetArm()->GetActorLocation().X,Position.X+10));
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    bool MovedStarted=StartCycle();
    for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("moved_robot_replanned_cycle_completes"),MovedStarted&&RunState==EFactoryRunState::Complete);
    ResetCycle();UndoEdit();SelectedId=Left;SetSelectedYaw(0);
    Check(TEXT("robot_base_follows_editor_rotation"),FMath::IsNearlyZero(TestAssemblyLine->WheelCell->LeftRobot->GetActorRotation().Yaw));UndoEdit();
    SelectedId=Left;SetSelectedPosition(60,10);Check(TEXT("unreachable_rear_target_blocks_start"),CanEdit()&&!StartCycle());UndoEdit();
    SelectedId.Empty();bHallSelected=true;ResizeHandle=7;ResizeStartBox=ResizeBox();DragStart=FVector(Layout.HallOrigin,0);
    BeginEdit();bDragging=true;PreviewResize(DragStart-FVector(500,0,0));bDragging=false;ResizeHandle=-1;EndEdit();Rebuild();
    Check(TEXT("hall_edge_drag_changes_size_not_robot_position"),FMath::IsNearlyEqual(Layout.LengthCm,Original.LengthCm+500)&&Layout.Find(Left)->Position==Position);
    UndoEdit();Check(TEXT("hall_drag_undo_restores_origin_and_size"),Layout.ToJson()==Initial);
    bHallSelected=false;AddRegisteredItem(TEXT("zone"),FVector2D(2500,0));Selected()->Size={400,400};Selected()->Yaw=37;Rebuild();
    ResizeStartBox=ResizeBox();ResizeHandle=3;DragStart=ResizeStartBox.Transform().TransformPosition(FVector(200,0,0));
    const auto FixedEdge=ResizeStartBox.Transform().TransformPosition(FVector(-200,0,0));
    BeginEdit();bDragging=true;PreviewResize(DragStart+ResizeStartBox.Transform().TransformVector(FVector(20000,0,0)));
    Check(TEXT("rotated_zone_drag_clamped_to_hall"),Layout.Contains(*Selected())&&Selected()->Size.X>400);
    Check(TEXT("clamped_resize_keeps_opposite_edge"),FixedEdge.Equals(Selected()->Transform().TransformPosition(FVector(-Selected()->Size.X*.5,0,0)),.001));
    CancelEdit();SetSelectedSize(200,200);Check(TEXT("numeric_zone_size_also_clamped"),Layout.Contains(*Selected()));
    Layout=Original;Rebuild();History.Clear();bSnap=true;SelectedId.Empty();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    bool Started=StartCycle();for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("recreated_robots_execute_full_team_cycle"),Started&&RunState==EFactoryRunState::Complete&&TestAssemblyLine->WheelCell->MountedCount==4&&TestAssemblyLine->DoorCell->bDoorFastened&&TestAssemblyLine->bFourDoorMode&&TestAssemblyLine->MountedDoorCount==4);
    bool FourMounted=true;int32 FoundDoors=0;
    for(TActorIterator<AVehicleProcessModule> It(GetWorld());It;++It)
    {
        const FString Id=It->ModuleId.ToString();
        if(Id!=TEXT("Door_FL")&&Id!=TEXT("Door_FR")&&Id!=TEXT("Door_RL")&&Id!=TEXT("Door_RR"))continue;
        ++FoundDoors;
        const FTransform Mount=It->ReferencePoseInVehicle*TestAssemblyLine->WheelCell->Assembly->GetActorTransform();
        FourMounted &= It->GetActorTransform().Equals(Mount,0.2);
    }
    Check(TEXT("all_four_distinct_door_meshes_at_authored_mounts"),FoundDoors==4&&FourMounted);
    TestAssemblyLine->WheelCell->bLaserScanFrontLeftHub=true;
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    const bool ScanStarted=StartCycle();
    for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    const auto& Scan=TestAssemblyLine->WheelCell->FrontLeftHubScan;
    Check(TEXT("laser_hub_hits_drive_complete_cycle"),ScanStarted&&RunState==EFactoryRunState::Complete
        &&Scan.bValid&&Scan.HitCount>=3&&Scan.MeasuredAt>=0&&Scan.ResidualMm<=1);
    const auto* LaserCell=TestAssemblyLine->WheelCell.Get();
    Check(TEXT("laser_three_measurement_stages"),LaserCell->FrontLeftDockScan.bValid
        &&LaserCell->FrontLeftPreMountScan.bValid&&LaserCell->FrontLeftFinalWheelScan.bValid
        &&LaserCell->FrontLeftDockScan.MeasuredAt<LaserCell->FrontLeftPreMountScan.MeasuredAt
        &&LaserCell->FrontLeftPreMountScan.MeasuredAt<LaserCell->FrontLeftFinalWheelScan.MeasuredAt
        &&FVector::Distance(LaserCell->FrontLeftPreMountScan.MeasuredWorld.GetLocation(),
            LaserCell->FrontLeftFinalWheelScan.MeasuredWorld.GetLocation())*10<=LaserCell->QualityToleranceMm);
    const FVector BaselineHubMeasurement=Scan.MeasuredWorld.GetLocation();
    TestAssemblyLine->WheelCell->FrontLeftHubOffsetCm=FVector(2,0,0);
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    StartCycle();
    for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    const auto& ShiftedScan=TestAssemblyLine->WheelCell->FrontLeftHubScan;
    Check(TEXT("laser_measured_hub_shift_changes_mount_target"),RunState==EFactoryRunState::Complete
        &&ShiftedScan.bValid&&FMath::IsNearlyEqual(
            ShiftedScan.MeasuredWorld.GetLocation().X-BaselineHubMeasurement.X,2.0,.1));
    TestAssemblyLine->WheelCell->FrontLeftHubOffsetCm=FVector::ZeroVector;
    TestAssemblyLine->WheelCell->LaserOutlierMm=10;
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    StartCycle();
    for(int32 I=0;I<120*20&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("laser_outlier_residual_blocks_pickup"),RunState==EFactoryRunState::Fault
        &&!TestAssemblyLine->WheelCell->FrontLeftHubScan.bValid
        &&TestAssemblyLine->WheelCell->FrontLeftHubScan.ResidualMm>1);
    TestAssemblyLine->WheelCell->LaserOutlierMm=0;
    TestAssemblyLine->WheelCell->bBlockFrontLeftLaser=true;
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    StartCycle();
    for(int32 I=0;I<120*20&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("blocked_laser_prevents_wheel_pickup"),RunState==EFactoryRunState::Fault
        &&!TestAssemblyLine->WheelCell->FrontLeftHubScan.bValid
        &&TestAssemblyLine->WheelCell->MountedCount==0);
    TestAssemblyLine->WheelCell->bBlockFrontLeftLaser=false;
    TestAssemblyLine->WheelCell->bLaserScanFrontLeftHub=false;
    for(int32 Kind : {0,1,2,3,4,6,7,8,9,10})
    {
        TestAssemblyLine->SetFactoryCheck(Kind);
        ResetCycle();
        for(int32 I=0;I<24;++I)StepSimulation(1./120);
        const bool Began=StartCycle();
        bool WaitObserved=false;
        bool NormalGripObserved=false;
        for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)
        {
            StepSimulation(1./120);
            const auto Stage=TestAssemblyLine->WheelCell->GetLeftStage();
            WaitObserved|=(Kind==1&&Stage==EWheelStage::WaitSupply)||(Kind==2&&Stage==EWheelStage::WaitPose);
            const auto& Grip=TestAssemblyLine->WheelCell->GetLeftGripState();
            NormalGripObserved|=Kind==6&&Grip.AppliedForceN>0&&Grip.AppliedForceN<=Grip.CapacityN+1.e-6&&Grip.SlipMm<3.0;
        }
        const auto* Line=TestAssemblyLine.Get();
        auto* Cell=Line->WheelCell.Get();
        if(Kind==9||Kind==10)
        {
            Check(Kind==9?TEXT("oversized_wheel_laser_rejects_before_pickup"):TEXT("undersized_wheel_laser_rejects_before_pickup"),
                Began&&RunState==EFactoryRunState::Fault&&!Cell->bFrontLeftWheelSizePassed
                &&(Kind==9?Cell->FrontLeftWheelMeasuredDiameterCm>67:Cell->FrontLeftWheelMeasuredDiameterCm<65)
                &&!Cell->GetLeftGripState().bHeld&&Cell->MountedCount==0);
            continue;
        }
        if(Kind==7||Kind==8)
        {
            Check(Kind==7?TEXT("integrated_low_friction_stops_left_wheel"):TEXT("overweight_wheel_exceeds_normal_friction_grip"),
                Began && Cell->Outcome==EWheelOutcome::Blocked && Cell->Status.Contains(TEXT("미끄러짐"))
                &&(Kind==7||(!Cell->bLowFrictionGrip&&Cell->FrontLeftWheelMassKg==90.0
                    &&Cell->GetLeftGripState().DemandN>Cell->GetLeftGripState().CapacityN)));
            const FVector BeforeFall=Cell->GetWheelWorld(0).GetLocation();
            Cell->Tick(0.25f);
            Check(Kind==7?TEXT("low_friction_releases_and_moves_wheel_after_stop"):TEXT("overweight_wheel_releases_and_moves_after_stop"),Cell->GetLeftGripState().bHeld==false
                &&Cell->LeftWheelSlipFallSeconds>0
                &&FVector::Distance(BeforeFall,Cell->GetWheelWorld(0).GetLocation())>1.0);
            continue;
        }
        const bool EventObserved=Kind==0 || Kind==6 ? Line->InjectedEvents==0 :
            Line->InjectedEvents==1 && Line->RecoveredEvents==1;
        const bool DecisionObserved=Kind==1||Kind==2 ? WaitObserved :
            Kind==3 ? Cell->LeftGripRetries>0 : Kind==4 ? Cell->LeftPickupReplans>0 : true;
        if(Kind==6)
            Check(TEXT("normal_grip_stays_within_force_limit"),NormalGripObserved);
        Check(*FString::Printf(TEXT("selected_sensor_case_%d"),Kind),
            Began&&RunState==EFactoryRunState::Complete&&Cell->MountedCount==4&&EventObserved&&DecisionObserved);
    }
    for(int32 Kind : {11,12})
    {
        TestAssemblyLine->SetFactoryCheck(Kind);
        ResetCycle();
        for(int32 I=0;I<24;++I)StepSimulation(1./120);
        const bool Began=StartCycle();
        for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
        const auto* Door=TestAssemblyLine->DoorCell.Get();
        Check(Kind==11?TEXT("oversized_door_laser_rejects_before_pickup"):TEXT("undersized_door_laser_rejects_before_pickup"),
            Began&&RunState==EFactoryRunState::Fault&&Door->MeasuredDoorWidthCm>0&&!Door->bDoorSizePassed
            &&(Kind==11?Door->MeasuredDoorWidthCm>Door->NominalDoorWidthCm+1:Door->MeasuredDoorWidthCm<Door->NominalDoorWidthCm-1)
            &&!Door->bDoorHeld);
    }
    TestAssemblyLine->SetFactoryCheck(13);
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    const bool DockBegan=StartCycle();
    for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("dock_overshoot_reverses_and_full_cycle_completes"),DockBegan&&RunState==EFactoryRunState::Complete
        &&TestAssemblyLine->bDockOvershootRecovered
        &&TestAssemblyLine->MaxDockOvershootCm>=34.0);
    TestAssemblyLine->SetFactoryCheck(5);
    ResetCycle();Layout=Original;Rebuild();History.Clear();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    return FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryModularReport.json")))&&Pass;
}

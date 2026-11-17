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
    // The factory UI sends one request to the line adapter; selecting a new case clears prior faults.
    const bool DemoAdapterValid=TestAssemblyLine && TestAssemblyLine->ConfigureIncidentDemo(14)
        &&TestAssemblyLine->WheelCell->bGripDegradesInTransit
        &&TestAssemblyLine->ConfigureIncidentDemo(15)
        &&!TestAssemblyLine->WheelCell->bGripDegradesInTransit
        &&TestAssemblyLine->WheelCell->LaserTransientBlockScans==2
        &&TestAssemblyLine->ConfigureIncidentDemo(16)
        &&TestAssemblyLine->WheelCell->bReplaceRejectedWheel
        &&TestAssemblyLine->WheelCell->LaserTransientBlockScans==0
        &&TestAssemblyLine->ConfigureIncidentDemo(18)
        &&TestAssemblyLine->bExtendedRandomEvents
        &&TestAssemblyLine->ConfigureIncidentDemo(5)
        &&!TestAssemblyLine->bExtendedRandomEvents
        &&!TestAssemblyLine->WheelCell->bReplaceRejectedWheel;
    Check(TEXT("incident_demo_adapter_resets_previous_fault_configuration"),DemoAdapterValid);
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
    // Transient occlusion: two blocked attempts, then the third measurement is accepted.
    TestAssemblyLine->WheelCell->LaserTransientBlockScans=2;
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    const bool RescanStarted=StartCycle();
    for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("laser_transient_block_rescans_then_completes"),RescanStarted&&RunState==EFactoryRunState::Complete
        &&TestAssemblyLine->WheelCell->FrontLeftDockScan.bValid
        &&TestAssemblyLine->WheelCell->FrontLeftDockScan.Attempts==3
        &&TestAssemblyLine->WheelCell->LaserScanRetries==2
        &&TestAssemblyLine->WheelCell->MountedCount==4);
    // Occlusion outlasting the attempt limit still blocks pickup.
    Check(TEXT("incident_log_records_laser_rescan_as_recovered"),
        TestAssemblyLine->WheelCell->CountIncidents(TEXT("laser_rescan"),TEXT("recovered"))==1
        &&TestAssemblyLine->WheelCell->CountIncidents(TEXT("safety_stop"),TEXT("stopped"))==0);
    TestAssemblyLine->WheelCell->LaserTransientBlockScans=3;
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    StartCycle();
    for(int32 I=0;I<120*20&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("laser_rescan_limit_blocks_pickup"),RunState==EFactoryRunState::Fault
        &&!TestAssemblyLine->WheelCell->FrontLeftHubScan.bValid
        &&TestAssemblyLine->WheelCell->FrontLeftHubScan.Attempts==3
        &&TestAssemblyLine->WheelCell->MountedCount==0);
    TestAssemblyLine->WheelCell->LaserTransientBlockScans=0;
    TestAssemblyLine->WheelCell->bLaserScanFrontLeftHub=false;
    for(int32 Kind : {0,1,2,3,4,6,7,8,9,10})
    {
        TestAssemblyLine->SetFactoryCheck(Kind);
        ResetCycle();
        for(int32 I=0;I<24;++I)StepSimulation(1./120);
        const bool Began=StartCycle();
        bool WaitObserved=false;
        bool NormalGripObserved=false;
        bool NormalWheelSizeObserved=false;
        bool NormalDoorSizeObserved=false;
        bool SupplyTransitObserved=false;
        TOptional<FVector> SupplyTransitFirstPosition;
        bool GripRetryMovementObserved=false;
        TOptional<FVector> GripRetryFirstTcp;
        bool StalePickupPoseObserved=false;
        bool FreshPickupPoseAfterRecovery=false;
        bool SlipFallObserved=false;
        bool SlipWorkerObserved=false;
        bool SlipResupplyObserved=false;
        for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)
        {
            StepSimulation(1./120);
            const auto Stage=TestAssemblyLine->WheelCell->GetLeftStage();
            WaitObserved|=(Kind==1&&Stage==EWheelStage::WaitSupply)||(Kind==2&&Stage==EWheelStage::WaitPose);
            if(Kind==2)
            {
                const auto* PoseSensor=TestAssemblyLine->WheelCell->LeftSensors.Get();
                const double Now=TestAssemblyLine->WheelCell->SimulationTimeSeconds;
                StalePickupPoseObserved|=Stage==EWheelStage::WaitPose
                    &&PoseSensor->PickupPose.MeasuredAt>=0
                    &&Now-PoseSensor->PickupPose.MeasuredAt>PoseSensor->MaxAge;
                FreshPickupPoseAfterRecovery|=TestAssemblyLine->RecoveredEvents>0
                    &&Stage!=EWheelStage::WaitPose&&PoseSensor->PickupPose.Fresh(Now,PoseSensor->MaxAge);
            }
            const auto& Grip=TestAssemblyLine->WheelCell->GetLeftGripState();
            if(Kind==7||Kind==8)
            {
                const auto* SlipCell=TestAssemblyLine->WheelCell.Get();
                SlipFallObserved|=SlipCell->SlipRecoveryPhase==1&&SlipCell->LeftWheelSlipFallSeconds>0.2;
                SlipWorkerObserved|=(SlipCell->SlipRecoveryPhase==2||SlipCell->SlipRecoveryPhase==3)
                    &&SlipCell->IsRecoveryWorkerVisible();
                SlipResupplyObserved|=SlipCell->SlipRecoveryPhase==4;
            }
            if(Kind==3&&TestAssemblyLine->WheelCell->LeftGripRetries>0)
            {
                const FVector CurrentTcp=TestAssemblyLine->WheelCell->LeftRobot->GetTCPWorld().GetLocation();
                if(!GripRetryFirstTcp.IsSet())GripRetryFirstTcp=CurrentTcp;
                GripRetryMovementObserved|=FVector::Distance(GripRetryFirstTcp.GetValue(),CurrentTcp)>5.0;
            }
            NormalGripObserved|=Kind==6&&Grip.AppliedForceN>0&&Grip.AppliedForceN<=Grip.CapacityN+1.e-6&&Grip.SlipMm<3.0;
            NormalWheelSizeObserved|=Kind==0&&TestAssemblyLine->WheelCell->bFrontLeftWheelSizePassed
                &&FMath::IsNearlyEqual(TestAssemblyLine->WheelCell->FrontLeftWheelMeasuredDiameterCm,66.0,1.0);
            NormalDoorSizeObserved|=Kind==0&&TestAssemblyLine->DoorCell->bDoorSizePassed
                &&TestAssemblyLine->DoorCell->MeasuredDoorWidthCm>0;
            const auto* SupplyCell=TestAssemblyLine->WheelCell.Get();
            if(Kind==1&&SupplyCell->IsWheelVisible(0)
                &&FVector::Distance(SupplyCell->GetWheelWorld(0).GetLocation(),SupplyCell->GetPickupWorld(0).GetLocation())>20.0
                &&Stage==EWheelStage::WaitSupply&&!SupplyCell->LeftSensors->Presence.bDetected)
            {
                const FVector CurrentSupply=SupplyCell->GetWheelWorld(0).GetLocation();
                if(!SupplyTransitFirstPosition.IsSet())SupplyTransitFirstPosition=CurrentSupply;
                SupplyTransitObserved|=FVector::Distance(SupplyTransitFirstPosition.GetValue(),CurrentSupply)>40.0;
            }
        }
        const auto* Line=TestAssemblyLine.Get();
        auto* Cell=Line->WheelCell.Get();
        if(Kind==1)
        {
            const FWheelIncidentRecord* Event=Cell->Incidents.FindByPredicate([](const FWheelIncidentRecord& R){return R.Kind==TEXT("supply_missing");});
            Check(TEXT("incident_identifies_demo_request_product_and_part"),Event
                &&Event->RunEpoch==Line->RunNumber-1
                &&Event->RequestId==Cell->RequestId
                &&Event->ProductInstanceId==Cell->ProductInstanceId
                &&Event->PartInstanceId==Event->ProductInstanceId+TEXT("/")+Event->PartId+TEXT("/initial")
                &&Event->PartId==TEXT("part_wheel_fl"));
        }
        if(Kind==0)
        {
            const auto Assessment=Cell->GetEvaluationSnapshot();
            Check(TEXT("demo_completion_does_not_claim_physical_quality"),
                RunState==EFactoryRunState::Complete
                &&Assessment.Control==EWheelControlAssessment::Completed
                &&Assessment.DemoGeometry==EWheelQualityAssessment::DemoGeometryPassed
                &&Assessment.PhysicalQuality==EWheelQualityAssessment::NotAssessed
                &&Assessment.Model==EWheelModelAssessment::DemoOnly);
        }
        if(Kind==9||Kind==10)
        {
            RefreshInspection();
            const auto* Diameter=Inspection().Sensors.Find(TEXT("wheel.fl.diameter"));
            Check(Kind==9?TEXT("oversized_wheel_measurement_reaches_sensor_panel"):TEXT("undersized_wheel_measurement_reaches_sensor_panel"),
                Diameter&&Diameter->Latest.Unit==TEXT("cm")&&Diameter->Latest.Value==Cell->FrontLeftWheelMeasuredDiameterCm
                &&Diameter->Latest.SampleTime>=0);
            Check(Kind==9?TEXT("oversized_wheel_laser_rejects_before_pickup"):TEXT("undersized_wheel_laser_rejects_before_pickup"),
                Began&&RunState==EFactoryRunState::Fault&&!Cell->bFrontLeftWheelSizePassed
                &&(Kind==9?Cell->FrontLeftWheelMeasuredDiameterCm>67:Cell->FrontLeftWheelMeasuredDiameterCm<65)
                &&!Cell->GetLeftGripState().bHeld&&Cell->MountedCount==0);
            for(int32 TickIndex=0;TickIndex<40;++TickIndex)Cell->Tick(0.05f);
            Check(Kind==9?TEXT("oversized_wheel_reaches_local_hold_point"):TEXT("undersized_wheel_reaches_local_hold_point"),
                Cell->bFrontLeftWheelRejected&&!Cell->bFrontLeftWheelRejecting
                &&FVector::Distance(Cell->GetWheelWorld(0).GetLocation(),Cell->WheelRejectTargetWorld)<0.5
                &&FVector::Distance(Cell->WheelRejectStartWorld,Cell->WheelRejectTargetWorld)>170.0);
            continue;
        }
        if(Kind==7||Kind==8)
        {
            Check(Kind==7?TEXT("low_friction_dropped_wheel_collides_and_recovers"):TEXT("overweight_dropped_wheel_collides_and_recovers"),
                Began&&SlipFallObserved&&SlipWorkerObserved&&SlipResupplyObserved
                &&Cell->bDroppedWheelHitSurface&&Cell->SlipRecoveryPhase==0
                &&Cell->SlipFailureMassKg==(Kind==7?18.0:90.0)
                &&Cell->bSlipFailureLowFriction==(Kind==7)
                &&RunState==EFactoryRunState::Complete&&Cell->MountedCount==4);
            continue;
        }
        const bool EventObserved=Kind==0 || Kind==6 ? Line->InjectedEvents==0 :
            Line->InjectedEvents==1 && Line->RecoveredEvents==1;
        const bool DecisionObserved=Kind==1||Kind==2 ? WaitObserved :
            Kind==3 ? Cell->LeftGripRetries>0 : Kind==4 ? Cell->LeftPickupReplans>0 : true;
        if(Kind==4)
            Check(TEXT("replanned_pickup_has_distinct_before_after_positions"),
                Cell->LeftPickupReplans>0&&FVector::Distance(Cell->PreviousPickupWorld,Cell->ReplannedPickupWorld)>=3.0);
        if(Kind==3)
            Check(TEXT("grip_retry_retracts_and_moves_tcp_before_regrip"),GripRetryMovementObserved);
        if(Kind==2)
            Check(TEXT("pickup_pose_link_stales_then_new_measurement_resumes"),
                StalePickupPoseObserved&&FreshPickupPoseAfterRecovery);
        if(Kind==6)
        {
            Check(TEXT("normal_grip_stays_within_force_limit"),NormalGripObserved);
            RefreshInspection();
            const auto* Grip=Inspection().Sensors.Find(TEXT("wheel.fl.grip.demand"));
            Check(TEXT("physical_grip_force_reaches_sensor_panel"),Grip&&Grip->Latest.Unit==TEXT("N")
                &&Grip->Latest.SampleTime>=0&&Grip->Latest.Value>0);
        }
        if(Kind==0)
            Check(TEXT("normal_wheel_and_door_use_size_scan"),NormalWheelSizeObserved&&NormalDoorSizeObserved);
        if(Kind==1)
            Check(TEXT("missing_wheel_visible_supply_transit_waits_for_arrival_sensor"),SupplyTransitObserved);
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
    for(int32 Kind : {1,2,3,4,9})
    {
        // Incident rows: each recoverable event is logged and closed as recovered with no stop row;
        // a size reject without replacement is closed as stopped together with one stop row.
        TestAssemblyLine->SetFactoryCheck(Kind);
        ResetCycle();
        for(int32 I=0;I<24;++I)StepSimulation(1./120);
        StartCycle();
        for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
        const auto* LogCell=TestAssemblyLine->WheelCell.Get();
        const TCHAR* Expected=Kind==1?TEXT("supply_missing"):Kind==2?TEXT("pickup_pose_stale")
            :Kind==3?TEXT("grip_unconfirmed"):Kind==4?TEXT("pickup_displaced"):TEXT("wheel_size_reject");
        const bool Stops=Kind==9;
        Check(*FString::Printf(TEXT("incident_log_case_%d_%s"),Kind,Expected),
            LogCell->CountIncidents(Expected,Stops?TEXT("stopped"):TEXT("recovered"))>=1
            &&LogCell->CountIncidents(Expected,TEXT("pending"))==0
            &&LogCell->CountIncidents(TEXT("safety_stop"),TEXT("stopped"))==(Stops?1:0)
            &&!LogCell->GetIncidentCsv().IsEmpty());
    }
    for(int32 Kind : {9,10})
    {
        // Size reject with replacement: hold-point move, spare delivery, re-scan, full cycle.
        TestAssemblyLine->SetFactoryCheck(Kind);
        TestAssemblyLine->WheelCell->bReplaceRejectedWheel=true;
        ResetCycle();
        for(int32 I=0;I<24;++I)StepSimulation(1./120);
        const bool ReplaceBegan=StartCycle();
        bool WaitedWhileReplacing=false;
        bool SpareTransitUnseen=false;
        for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)
        {
            StepSimulation(1./120);
            const auto* RepCell=TestAssemblyLine->WheelCell.Get();
            WaitedWhileReplacing|=RepCell->WheelReplacePhase>0&&RepCell->GetLeftStage()==EWheelStage::WaitSupply
                &&!RepCell->GetLeftGripState().bHeld;
            SpareTransitUnseen|=RepCell->WheelReplacePhase==2&&!RepCell->LeftSensors->Presence.bDetected;
        }
        const auto* RepCell=TestAssemblyLine->WheelCell.Get();
        Check(Kind==9?TEXT("oversized_wheel_replaced_rescanned_and_cycle_completes"):TEXT("undersized_wheel_replaced_rescanned_and_cycle_completes"),
            ReplaceBegan&&WaitedWhileReplacing&&SpareTransitUnseen
            &&RepCell->WheelReplacements==1&&RepCell->WheelReplacePhase==0&&RepCell->bFrontLeftWheelRejected
            &&(Kind==9?RepCell->RejectedWheelDiameterCm>67:(RepCell->RejectedWheelDiameterCm>0&&RepCell->RejectedWheelDiameterCm<65))
            &&RepCell->bFrontLeftWheelSizePassed
            &&FMath::IsNearlyEqual(RepCell->FrontLeftWheelMeasuredDiameterCm,66.0,1.0)
            &&FVector::Distance(RepCell->WheelRejectStartWorld,RepCell->WheelRejectTargetWorld)>170.0
            &&RunState==EFactoryRunState::Complete&&RepCell->MountedCount==4);
        if(Kind==9)
        {
            const FWheelIncidentRecord* Reject=RepCell->Incidents.FindByPredicate([](const FWheelIncidentRecord& R){return R.Kind==TEXT("wheel_size_reject");});
            const FWheelIncidentRecord* Spare=RepCell->Incidents.FindByPredicate([](const FWheelIncidentRecord& R){return R.Kind==TEXT("spare_supplied");});
            Check(TEXT("rejected_and_spare_wheels_have_distinct_demo_identities"),Reject
                &&Spare&&Spare->Outcome==TEXT("recovered")
                &&!RepCell->RejectedWheelInstanceId.IsEmpty()
                &&RepCell->RejectedWheelInstanceId==Reject->PartInstanceId
                &&RepCell->SpareWheelInstanceId==Spare->PartInstanceId
                &&RepCell->SpareWheelInstanceId==RepCell->FrontLeftActiveWheelInstanceId
                &&RepCell->SpareWheelInstanceId!=RepCell->RejectedWheelInstanceId
                &&RepCell->GetIncidentCsv().Contains(RepCell->RejectedWheelInstanceId));
            Check(TEXT("incident_log_closes_replaced_size_reject_as_recovered"),
                RepCell->CountIncidents(TEXT("wheel_size_reject"),TEXT("recovered"))==1
                &&RepCell->CountIncidents(TEXT("safety_stop"),TEXT("stopped"))==0);
        }
        TestAssemblyLine->WheelCell->bReplaceRejectedWheel=false;
    }
    {
        // Grip degrades in transit: the wheel is held through Lift, slips only after
        // Transfer begins, then the existing drop recovery finishes the cycle.
        TestAssemblyLine->SetFactoryCheck(6);
        TestAssemblyLine->WheelCell->bGripDegradesInTransit=true;
        ResetCycle();
        for(int32 I=0;I<24;++I)StepSimulation(1./120);
        const bool DegradeBegan=StartCycle();
        bool HeldInTransfer=false;
        for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)
        {
            StepSimulation(1./120);
            const auto* DegCell=TestAssemblyLine->WheelCell.Get();
            const auto& DegGrip=DegCell->GetLeftGripState();
            HeldInTransfer|=DegCell->GetLeftStage()==EWheelStage::Transfer&&DegGrip.WheelIndex==0&&DegGrip.bHeld&&DegCell->SlipRecoveryPhase==0;
        }
        const auto* DegCell=TestAssemblyLine->WheelCell.Get();
        Check(TEXT("grip_degrades_in_transit_slips_after_lift_and_recovers"),DegradeBegan&&HeldInTransfer
            &&DegCell->bSlipFailureGripDegraded&&!DegCell->bSlipFailureLowFriction&&DegCell->SlipFailureMassKg==18.0
            &&DegCell->SlipFailureStage!=EWheelStage::Grip&&DegCell->SlipFailureStage!=EWheelStage::Lift
            &&DegCell->SlipFailureTravelCm>20.0
            &&DegCell->bDroppedWheelHitSurface&&DegCell->SlipRecoveryPhase==0
            &&RunState==EFactoryRunState::Complete&&DegCell->MountedCount==4);
        Check(TEXT("incident_log_closes_transit_slip_as_recovered"),
            DegCell->CountIncidents(TEXT("grip_slip"),TEXT("recovered"))==1
            &&DegCell->CountIncidents(TEXT("grip_slip"),TEXT("pending"))==0);
        TestAssemblyLine->WheelCell->bGripDegradesInTransit=false;
    }
    TestAssemblyLine->SetFactoryCheck(13);
    ResetCycle();
    for(int32 I=0;I<24;++I)StepSimulation(1./120);
    const bool DockBegan=StartCycle();
    for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
    Check(TEXT("dock_overshoot_reverses_and_full_cycle_completes"),DockBegan&&RunState==EFactoryRunState::Complete
        &&TestAssemblyLine->bDockOvershootRecovered
        &&TestAssemblyLine->MaxDockOvershootCm>=34.0);
    {
        // Extended seeded pool: whatever line-level event the seed plans must actually
        // occur, be the only physical event, and end in a complete four-wheel cycle.
        TestAssemblyLine->SetFactoryCheck(5);
        TestAssemblyLine->bExtendedRandomEvents=true;
        bool ExtendedConsistent=true;
        for(int32 Run=0;Run<4;++Run)
        {
            ResetCycle();
            for(int32 I=0;I<24;++I)StepSimulation(1./120);
            const int32 Planned=TestAssemblyLine->ExtendedEvent;
            const bool ExtendedBegan=StartCycle();
            for(int32 I=0;I<120*300&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);
            const auto* ExtCell=TestAssemblyLine->WheelCell.Get();
            const bool SlipSeen=ExtCell->SlipFailureMassKg>0&&ExtCell->bDroppedWheelHitSurface;
            const bool PlanMatched=Planned==1 ? (SlipSeen&&ExtCell->SlipFailureMassKg==90.0&&!ExtCell->bSlipFailureGripDegraded)
                : Planned==2 ? (SlipSeen&&ExtCell->bSlipFailureGripDegraded) : !SlipSeen;
            ExtendedConsistent&=ExtendedBegan&&Planned>=0&&Planned<=3&&PlanMatched
                &&(Planned==3)==TestAssemblyLine->bDockOvershootRecovered
                &&RunState==EFactoryRunState::Complete&&ExtCell->MountedCount==4;
        }
        Check(TEXT("extended_random_line_events_match_plan_and_recover"),ExtendedConsistent);
        TestAssemblyLine->bExtendedRandomEvents=false;
    }
    TestAssemblyLine->SetFactoryCheck(5);
    ResetCycle();Layout=Original;Rebuild();History.Clear();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    return FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryModularReport.json")))&&Pass;
}

#include "FactoryRuntime.h"
#include "FactoryEquipment.h"
#include "FactoryMotionController.h"
#include "FactoryRobotEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "Components/WidgetComponent.h"
#include "Components/SceneComponent.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "UnrealClient.h"

void AFactoryRuntime::RunChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;
    auto Check=[&](const TCHAR* Name,bool Result){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("check"),Name);O->SetBoolField(TEXT("passed"),Result);Checks.Add(MakeShared<FJsonValueObject>(O));Pass&=Result;UE_LOG(LogTemp,Display,TEXT("FACTORY_TEST %s %s"),Name,Result?TEXT("PASS"):TEXT("FAIL"));};
    FString Error;FFactoryLayout Read;
    Check(TEXT("native_map_assets_and_default_schema"),VerifySetup()&&Layout.Validate(Error)&&Layout.Items.Num()==19);
    const FString Initial=Layout.ToJson();
    Check(TEXT("json_round_trip_preserves_korean_geometry_and_bindings"),FFactoryLayout::FromJson(Initial,Read,Error)&&Read.ToJson()==Initial);
    const FString Before=Read.ToJson();
    Check(TEXT("malformed_json_does_not_partially_replace_layout"),!FFactoryLayout::FromJson(TEXT("{\"version\":1}"),Read,Error)&&Read.ToJson()==Before);
    auto Invalid=Layout;const auto Duplicate=Invalid.Items[0];Invalid.Items.Add(Duplicate);Check(TEXT("duplicate_ids_rejected"),!Invalid.Validate(Error));
    Invalid=Layout;Invalid.HandlerId=Invalid.StationId;Check(TEXT("wrong_resource_kind_rejected"),!Invalid.Validate(Error));
    Invalid=Layout;Invalid.Items[0].Position=FVector2D(-200,0);Check(TEXT("outside_design_can_be_saved_but_is_flagged"),Invalid.Validate(Error)&&!Invalid.Contains(Invalid.Items[0]));
    FFactoryLayoutItem A,B;A.Position=FVector2D(100,100);A.Size=FVector2D(100,100);B=A;B.Position.X+=100;
    Check(TEXT("touching_footprints_not_overlapping"),!FFactoryLayout::Overlaps(A,B));B.Position.X-=1;
    Check(TEXT("intersecting_footprints_detected"),FFactoryLayout::Overlaps(A,B));
    A.Position=FVector2D(40,0);A.Yaw=45;Check(TEXT("rotated_bounds_use_corners"),!Layout.Contains(A));
    Check(TEXT("rotated_segment_obstacle"),FFactoryLayout::SegmentHits(FVector(-100,0,40),FVector(200,0,40),A,10));
    SelectItem(Layout.StationId);bSnap=false;SetSelectedPosition(43,1);
    Check(TEXT("vehicle_hub_follows_edited_station"),HubWorld().Equals(Layout.Find(Layout.StationId)->Transform().TransformPosition(FVector(240,0,25)+WheelPivotCm),.0001));
    Check(TEXT("save_layout_A"),SaveLayout(TEXT("A")));
    SetSelectedPosition(47,2);Check(TEXT("load_layout_A_restores_geometry"),LoadLayout(TEXT("A"))&&Layout.Find(Layout.StationId)->Position.Equals(FVector2D(4300,100),.001));
    const FString Good=Layout.ToJson();FFileHelper::SaveStringToFile(TEXT("broken"),*SlotPath(TEXT("B")));
    Check(TEXT("bad_saved_file_preserves_current_scene"),!LoadLayout(TEXT("B"))&&Layout.ToJson()==Good);
    Layout=FFactoryLayout::LegacyDefault();Rebuild();
    const auto StationSize=Layout.Find(Layout.StationId)->Size;const auto StationPos=Layout.Find(Layout.StationId)->Position;
    SetHallSize(80,45);Check(TEXT("hall_resize_does_not_scale_or_move_equipment"),Layout.Find(Layout.StationId)->Size==StationSize&&Layout.Find(Layout.StationId)->Position==StationPos);
    Layout=FFactoryLayout::LegacyDefault();Rebuild();
    Check(TEXT("default_cycle_starts"),StartCycle());
    const FString Locked=Layout.ToJson();SelectItem(Layout.StationId);SetHallSize(100,60);AddItem(EFactoryItemKind::Conveyor);DeleteSelected();
    Check(TEXT("running_blocks_layout_mutation_and_save"),Layout.ToJson()==Locked&&!SaveLayout(TEXT("A")));
    for(int32 I=0;I<120;++I)StepSimulation(1./120);
    StopCycle();const FTransform Stopped=Equipment.FindRef(Layout.HandlerId)->Controller()->Pose();const double StopTime=SimulationSeconds;
    for(int32 I=0;I<240;++I)StepSimulation(1./120);
    Check(TEXT("stop_freezes_motion_and_simulation_clock"),Equipment.FindRef(Layout.HandlerId)->Controller()->Pose().Equals(Stopped,.000001)&&SimulationSeconds==StopTime);
    ResetCycle();Check(TEXT("reset_returns_wheel_to_current_supply"),CanEdit()&&WheelRoot->GetComponentLocation().Equals(SupplyWorld(),.001));
    auto Finish=[&](){for(int32 I=0;I<120*90 && RunState!=EFactoryRunState::Complete&&RunState!=EFactoryRunState::Fault;++I)StepSimulation(1./120);return RunState==EFactoryRunState::Complete;};
    Check(TEXT("default_wheel_transfer_aligns_at_hub"),StartCycle()&&Finish()&&WheelRoot->GetComponentLocation().Equals(HubWorld(),.051));
    const double Baseline=LastCycleSeconds;const FVector OriginalHub=HubWorld();
    Check(TEXT("cycle_reports_time_distance_and_alignment"),Baseline>1&&TravelCm>100&&AlignmentMm<=.5);
    ResetCycle();SelectItem(Layout.SupplyId);SetSelectedPosition(43.45,-9);
    Check(TEXT("longer_supply_route_changes_cycle_time"),StartCycle()&&Finish()&&LastCycleSeconds>Baseline+.5);
    ResetCycle();Layout=FFactoryLayout::LegacyDefault();
    const FVector2D Pivot=Layout.Find(Layout.StationId)->Position;const FRotator Rotate(0,37,0);
    for(const FString& Id:{Layout.StationId,Layout.SupplyId,Layout.HandlerId})
    {auto* I=Layout.Find(Id);const FVector V=Rotate.RotateVector(FVector(I->Position.X-Pivot.X,I->Position.Y-Pivot.Y,0));I->Position=Pivot+FVector2D(V.X+800,V.Y+300);I->Yaw=37;}
    Rebuild();Check(TEXT("translated_rotated_cell_targets_new_vehicle_hub"),StartCycle()&&Finish()&&WheelRoot->GetComponentLocation().Equals(HubWorld(),.051)&&!HubWorld().Equals(OriginalHub,100));
    ResetCycle();SelectItem(Layout.HandlerId);SetSelectedPosition(10,-3);
    Check(TEXT("unreachable_handler_cannot_start"),!StartCycle()&&CanEdit());
    Layout=FFactoryLayout::LegacyDefault();Rebuild();
    FFactoryLayoutItem Obstacle;Obstacle.Id=TEXT("test_obstacle");Obstacle.Kind=EFactoryItemKind::Conveyor;Obstacle.NameKo=TEXT("시험 장애물");Obstacle.NameEn=TEXT("Test Obstacle");Obstacle.Size=FVector2D(60,60);
    const FVector Outside=HubWorld()+FVector(0,-100,0);Obstacle.Position=FVector2D(Outside.X,Outside.Y);Layout.Items.Add(Obstacle);Rebuild();
    Check(TEXT("blocked_wheel_route_cannot_start"),!StartCycle());
    Layout=FFactoryLayout::LegacyDefault();Rebuild();SetHallSize(40,40);
    Check(TEXT("outside_equipment_cannot_run"),!StartCycle());
    Layout=FFactoryLayout::LegacyDefault();Rebuild();
    SelectItem(Layout.SupplyId);DeleteSelected();Check(TEXT("deleting_bound_resource_clears_binding"),Layout.SupplyId.IsEmpty()&&!StartCycle());
    AddItem(EFactoryItemKind::WheelSupply);SetSelectedPosition(43.45,-5.5);BindSelected();
    Check(TEXT("newly_added_supply_can_be_bound_and_used"),!Layout.SupplyId.IsEmpty()&&StartCycle()&&Finish());
    ResetCycle();Layout=FFactoryLayout::LegacyDefault();Rebuild();
    History.Clear();SavedSnapshot=Layout.ToJson();SelectedId=Layout.StationId;
    const FString EditBefore=Layout.ToJson();
    SetSelectedPosition(44,1);UndoEdit();
    Check(TEXT("undo_restores_data_and_saved_dirty_state"),Layout.ToJson()==EditBefore&&!bDirty&&CanRedo());
    RedoEdit();Check(TEXT("redo_restores_edit"),Layout.Find(Layout.StationId)->Position.Equals({4400,100},.001)&&bDirty);
    UndoEdit();SetSelectedYaw(25);Check(TEXT("new_edit_discards_redo_branch"),!CanRedo());
    const auto GestureBefore=Layout.ToJson();BeginEdit();bDragging=true;
    for(int32 I=0;I<20;++I)SetSelectedPosition(44+I*.1,1);
    bDragging=false;EndEdit();UndoEdit();Check(TEXT("drag_is_one_undo_transaction"),Layout.ToJson()==GestureBefore);
    BeginEdit();bDragging=true;SetSelectedPosition(48,2);CancelEdit();Check(TEXT("escape_reverts_drag"),Layout.ToJson()==GestureBefore);
    SelectedId=Layout.SupplyId;const auto DeletedId=SelectedId;DeleteSelected();UndoEdit();
    Check(TEXT("undo_delete_restores_stable_id_and_job_binding"),Layout.Find(DeletedId)&&Layout.SupplyId==DeletedId);
    SelectedId=TEXT("P03");const auto Order=Layout.ProcessOrder;RouteMoveSelected(1);UndoEdit();Check(TEXT("flow_order_is_undoable"),Layout.ProcessOrder==Order);
    DeleteSelected();Check(TEXT("delete_route_node_removes_dangling_reference"),!Layout.ProcessOrder.Contains(TEXT("P03"))&&Layout.Validate(Error));UndoEdit();
    SetZoneLabels(false);bool AllHidden=true;for(const auto& I:Layout.Items)if(I.Kind==EFactoryItemKind::Zone)AllHidden&=!Equipment.FindRef(I.Id)->Label->IsVisible();
    Check(TEXT("zone_label_toggle_hides_all_zone_labels"),AllHidden);Rebuild();Check(TEXT("zone_label_setting_survives_rebuild"),!Equipment.FindRef(TEXT("P01"))->Label->IsVisible());
    SetZoneLabels(true);Check(TEXT("zone_label_toggle_restores_labels"),Equipment.FindRef(TEXT("P01"))->Label->IsVisible());SetZoneLabels(false);
    auto Unknown=Layout;Unknown.Items[0].TypeId=TEXT("missing_team_plugin");
    Check(TEXT("unknown_equipment_definition_rejected"),!Registry->Validate(Unknown,Error));
    Layout=FFactoryLayout::LegacyDefault();Rebuild();History.Clear();SelectedId.Empty();bHallSelected=true;
    const auto FixedStation=Layout.Find(Layout.StationId)->Position;BeginEdit();bDragging=true;ResizeHandle=7;ResizeStartBox=ResizeBox();DragStart=FVector(0,0,0);
    PreviewResize(FVector(-400,0,0));bDragging=false;EndEdit();Rebuild();
    Check(TEXT("hall_left_resize_moves_origin_not_equipment"),FMath::IsNearlyEqual(Layout.HallOrigin.X,-400.)&&FMath::IsNearlyEqual(Layout.LengthCm,7600.)&&Layout.Find(Layout.StationId)->Position==FixedStation);
    UndoEdit();Check(TEXT("undo_hall_resize_restores_origin"),Layout.HallOrigin.IsNearlyZero()&&FMath::IsNearlyEqual(Layout.LengthCm,7200.));
    SelectedId=TEXT("P01");bHallSelected=false;Selected()->Yaw=37;ResizeStartBox=ResizeBox();ResizeHandle=3;DragStart=ResizeStartBox.Transform().TransformPosition(FVector(ResizeStartBox.Size.X*.5,0,0));
    const FVector FixedEdge=ResizeStartBox.Transform().TransformPosition(FVector(-ResizeStartBox.Size.X*.5,0,0));
    BeginEdit();bDragging=true;PreviewResize(DragStart+ResizeStartBox.Transform().TransformVector(FVector(200,0,0)));
    const FVector NewFixedEdge=Selected()->Transform().TransformPosition(FVector(-Selected()->Size.X*.5,0,0));
    Check(TEXT("rotated_zone_resize_keeps_opposite_edge"),FixedEdge.Equals(NewFixedEdge,.001));CancelEdit();
    Layout=FFactoryLayout::Default();Rebuild();History.Clear();bHallSelected=false;SelectedId.Empty();
    auto* RC=Equipment.FindRef(Layout.HandlerId)->Controller();TArray<double> IK;const auto Known=RC->Forward({25,-30,55,20,40,-15});
    Check(TEXT("kuka_full_pose_ik_matches_forward_pose"),RC->Solve(Known,IK,RC->Home())&&RC->Forward(IK).Equals(Known,.03));
    int32 TerminalEvents=0;RC->OnTerminal.AddLambda([&TerminalEvents](FFactoryRequestId,EFactoryCommandStatus){++TerminalEvents;});
    RC->Reset(5000);const auto Goal=RC->Forward({20,-30,55,15,35,10});FString CommandError;
    Check(TEXT("controller_accepts_command_and_rejects_busy_duplicate"),RC->RequestMove({5000,1},Goal,CommandError)&&!RC->RequestMove({5000,2},Goal,CommandError));
    for(int32 I=0;I<120*40&&RC->Status()==EFactoryCommandStatus::Running;++I)RC->Step(1./120);
    RC->Step(1./120);Check(TEXT("pid_reaches_pose_and_emits_one_terminal"),RC->Status()==EFactoryCommandStatus::Completed&&TerminalEvents==1);
    RC->Reset(5001);Check(TEXT("old_epoch_command_rejected_after_reset"),!RC->RequestMove({5000,3},Goal,CommandError));RC->OnTerminal.Clear();
    ResetCycle();const bool RobotStarted=StartCycle();UE_LOG(LogTemp,Display,TEXT("ROBOT_START %s"),*Status);
    const bool RobotDone=RobotStarted&&Finish();UE_LOG(LogTemp,Display,TEXT("ROBOT_FINISH %s"),*Status);
    Check(TEXT("kuka_wheel_transfer_completes_with_position_and_orientation_pid"),RobotDone&&AlignmentMm<=.5);
    const double RobotCycle=LastCycleSeconds;
    ResetCycle();const auto RobotPivot=Layout.Find(Layout.StationId)->Position;
    for(const FString& Id:{Layout.StationId,Layout.SupplyId,Layout.HandlerId})
    {auto* I=Layout.Find(Id);const auto P=FRotator(0,90,0).RotateVector(FVector(I->Position-RobotPivot,0));I->Position=RobotPivot+FVector2D(P.X+500,P.Y+400);I->Yaw=90;}
    Rebuild();Check(TEXT("kuka_cell_rotated_90_degrees_and_translated_completes"),StartCycle()&&Finish());
    ResetCycle();Layout=FFactoryLayout::Default();Rebuild();
    auto* FirstRobot=Equipment.FindRef(Layout.HandlerId).Get();const auto FirstPose=FirstRobot->Controller()->Pose();
    AddItem(EFactoryItemKind::ArticulatedRobot);auto* SecondRobot=Equipment.FindRef(SelectedId).Get();
    Check(TEXT("two_robot_instances_have_independent_controller_state"),SecondRobot&&SecondRobot->Controller()!=Equipment.FindRef(Layout.HandlerId)->Controller());
    ResetCycle();Layout=FFactoryLayout::Default();Rebuild();
    FFileHelper::SaveStringToFile(Layout.ToJson(),*(FPaths::ProjectSavedDir()/TEXT("FactoryDefaultV2.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);Report->SetNumberField(TEXT("baseline_cycle_sim_seconds"),Baseline);
    Report->SetNumberField(TEXT("robot_cycle_seconds"),RobotCycle);
    Report->SetStringField(TEXT("scope"),TEXT("Native edit history, serialization, registry, kinematic six-axis IK and ideal joint acceleration PID. Not torque/contact/fastening certification. Mouse input needs separate visual/manual review."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    bool Saved=FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryRuntimeTestReport.json")));
    FPlatformMisc::RequestExitWithStatus(false,Pass&&Saved?0:1);
}

void AFactoryRuntime::CaptureRuntime(float Dt)
{
    CaptureClock+=Dt;
    const FString Dir=FPaths::ProjectSavedDir()/TEXT("FactoryRuntimePreview");
    IFileManager::Get().MakeDirectory(*Dir,true);
    if(!bUIProbeDone&&CaptureClock>2){bUIProbePassed=RunUIChecks();bUIProbeDone=true;}
    if(CaptureStage==0&&CaptureClock>3){if(bUseAssemblyIntegration)FocusWorkcell();else FitView();CaptureStage=1;}
    if(CaptureStage==1&&CaptureClock>6){FScreenshotRequest::RequestScreenshot(Dir/TEXT("Layout_Korean.png"),true,false);CaptureStage=2;}
    if(CaptureStage==2&&CaptureClock>9){if(!bUseAssemblyIntegration){Layout=FFactoryLayout::Default();Rebuild();}FocusWorkcell();if(StartCycle())CaptureStage=3;}
    if(CaptureStage==3&&RunState==EFactoryRunState::Complete){CaptureStage=4;CaptureClock=0;}
    if(CaptureStage==4&&CaptureClock>3){FScreenshotRequest::RequestScreenshot(Dir/TEXT("Wheel_Complete_Korean.png"),true,false);CaptureStage=5;}
    if(CaptureStage==5&&CaptureClock>6)
    {
        auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),bUIProbePassed&&RunState==EFactoryRunState::Complete&&IFileManager::Get().FileSize(*(Dir/TEXT("Layout_Korean.png")))>0&&IFileManager::Get().FileSize(*(Dir/TEXT("Wheel_Complete_Korean.png")))>0);
        Report->SetNumberField(TEXT("simulation_seconds"),SimulationSeconds);Report->SetNumberField(TEXT("alignment_mm"),AlignmentMm);
        FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));FFileHelper::SaveStringToFile(Json,*(Dir/TEXT("report.json")));
        FPlatformMisc::RequestExitWithStatus(false,Report->GetBoolField(TEXT("success"))?0:1);
    }
    if(CaptureClock>(bUseAssemblyIntegration?420:150))FPlatformMisc::RequestExitWithStatus(false,1);
}

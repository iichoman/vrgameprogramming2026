#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "FactoryEquipmentRegistry.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "RobotArm.h"
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
    Check(TEXT("recreated_robots_execute_full_team_cycle"),Started&&RunState==EFactoryRunState::Complete&&TestAssemblyLine->WheelCell->MountedCount==4&&TestAssemblyLine->DoorCell->bDoorFastened);
    ResetCycle();Layout=Original;Rebuild();History.Clear();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    return FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryModularReport.json")))&&Pass;
}

#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "RobotArm.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

bool AFactoryRuntime::RunDirectEditChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;FString Error;
    auto Check=[&](const TCHAR* N,bool OK){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("check"),N);O->SetBoolField(TEXT("passed"),OK);O->SetStringField(TEXT("status"),Status);Checks.Add(MakeShared<FJsonValueObject>(O));Pass&=OK;UE_LOG(LogTemp,Display,TEXT("DIRECT_TEST %s %d %s"),N,OK,*Status);};
    auto* PC=UGameplayStatics::GetPlayerController(this,0);
    auto Task=[&](const FString& Id)->FFactoryTask*{return Layout.Tasks.FindByPredicate([&](const auto& T){return T.Id==Id;});};
    const auto Original=Layout;const FString Initial=Layout.ToJson();
    int Wheels=0,Doors=0;for(const auto& I:Layout.Items){Wheels+=I.EffectiveTypeId().StartsWith(TEXT("assembly_wheel"))?1:0;Doors+=I.EffectiveTypeId()==TEXT("assembly_door")?1:0;}
    Check(TEXT("default_two_wheel_two_door_robots"),Wheels==2&&Doors==2&&Layout.Tasks.Num()==6&&RightDoorCell&&ValidateAssemblyBindings(Error));
    const FString Left=Task(TEXT("wheel_fl"))->RobotId;
    Check(TEXT("wheel_tool_not_restricted_to_side_name"),FactoryTasks::Supports(TEXT("assembly_wheel_left"),*FactoryTasks::Find(TEXT("wheel_rr"))));
    OpenRobotPopup(Left,FVector2D(700,200));Check(TEXT("robot_selection_opens_context"),bRobotPopup&&bAssigningTask&&SelectedId==Left);
    UnassignTask(TEXT("wheel_fl"));FVector Target=FVector::ZeroVector;FVector2D Pixel=FVector2D::ZeroVector;
    FocusWorkcell();if(PC&&PC->PlayerCameraManager)PC->PlayerCameraManager->UpdateCamera(.02f);
    bool Hit=PC&&TaskTargetWorld(TEXT("wheel_fl"),Target)&&PC->ProjectWorldLocationToScreen(Target,Pixel)&&HitTaskTarget(Pixel);
    Check(TEXT("target_click_waits_for_part_without_assigning"),Hit&&PendingTaskId==TEXT("wheel_fl")&&Task(TEXT("wheel_fl"))->RobotId.IsEmpty());
    const auto BeforeWrong=Layout.ToJson();Check(TEXT("wrong_part_click_preserves_pending_task"),!ChoosePart(TEXT("part_door_fl"))&&Layout.ToJson()==BeforeWrong&&!PendingTaskId.IsEmpty());
    Check(TEXT("second_click_commits_robot_target_part"),ChoosePart(TEXT("part_wheel_fl"))&&Task(TEXT("wheel_fl"))->RobotId==Left&&Task(TEXT("wheel_fl"))->PartId==TEXT("part_wheel_fl"));
    UndoEdit();Check(TEXT("connection_undo"),Task(TEXT("wheel_fl"))->RobotId.IsEmpty());RedoEdit();Check(TEXT("connection_redo"),Task(TEXT("wheel_fl"))->RobotId==Left);
    CloseRobotPopup();bSnap=false;
    auto AimAt=[&](FVector P,FVector2D& Out){CameraCentre=FVector2D(P);CameraWidth=1000;bTopView=false;UpdateCamera();PC->PlayerCameraManager->UpdateCamera(.02f);return PC->ProjectWorldLocationToScreen(P,Out);};
    const FString DoorRobot=Task(TEXT("door_fr"))->RobotId;OpenRobotPopup(DoorRobot,Pixel);UnassignTask(TEXT("door_fr"));
    const auto RightBounds=PartBounds(TEXT("part_door_fr"));
    Check(TEXT("right_door_visible_module_has_dimensions"),RightBounds.IsValid&&RightBounds.GetSize().GetMax()>50);
    if(TaskTargetWorld(TEXT("door_fr"),Target)&&AimAt(Target,Pixel))ViewportPointer(Pixel,EKeys::LeftMouseButton,true);
    Check(TEXT("door_mount_click_selects_target"),PendingTaskId==TEXT("door_fr")&&Task(TEXT("door_fr"))->RobotId.IsEmpty());
    if(RightBounds.IsValid&&AimAt(RightBounds.GetCenter(),Pixel))ViewportPointer(Pixel,EKeys::LeftMouseButton,true);
    Check(TEXT("right_door_mesh_click_connects_executor"),Task(TEXT("door_fr"))->RobotId==DoorRobot&&Task(TEXT("door_fr"))->PartId==TEXT("part_door_fr"));
    CloseRobotPopup();
    for(const TCHAR* Id:{TEXT("part_wheel_fl"),TEXT("part_door_fr")})
    {
        FTransform P;PartWorld(Id,P);const FVector PickPoint=PartBounds(Id).GetCenter();
        CameraCentre=FVector2D(PickPoint);CameraWidth=1200;bTopView=false;UpdateCamera();PC->PlayerCameraManager->UpdateCamera(.02f);
        PC->ProjectWorldLocationToScreen(PickPoint,Pixel);
        UE_LOG(LogTemp,Display,TEXT("PART_PICK %s pivot=%s bounds=%s hit=%s"),Id,*P.GetLocation().ToString(),*PartBounds(Id).ToString(),*HitPart(Pixel));
        const FString Before=Layout.ToJson();
        ViewportPointer(Pixel,EKeys::LeftMouseButton,true);
        Check(TEXT("physical_part_click_starts_drag"),bDraggingPart&&SelectedPartId==Id);
        ViewportMove(Pixel+FVector2D(15,0));ViewportPointer(Pixel+FVector2D(15,0),EKeys::LeftMouseButton,false);
        FTransform M;PartWorld(Id,M);
        Check(TEXT("drag_updates_pose_and_persistent_supply"),!Layout.PartOffsetsCm.FindRef(Id).IsNearlyZero()&&!M.GetLocation().Equals(P.GetLocation(),.01)&&FMath::IsNearlyEqual(M.GetLocation().Z,P.GetLocation().Z,.01)&&(PartBounds(Id).GetCenter()-PickPoint).Equals(M.GetLocation()-P.GetLocation(),.02));
        const FString After=Layout.ToJson();UndoEdit();Check(TEXT("part_drag_undo"),Layout.ToJson()==Before);RedoEdit();Check(TEXT("part_drag_redo"),Layout.ToJson()==After);UndoEdit();
    }
    FFactoryLayout Read;Check(TEXT("v5_roundtrip"),FFactoryLayout::FromJson(Layout.ToJson(),Read,Error)&&Read.ToJson()==Layout.ToJson());
    auto Run=[&](){ResetCycle();for(int I=0;I<24;++I)StepSimulation(1./120);bool OK=StartCycle();
        for(int I=0;I<36000&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);return OK&&RunState==EFactoryRunState::Complete;};
    Check(TEXT("two_wheels_two_doors_complete_with_independent_inspection"),Run()&&TestAssemblyLine->WheelCell->MountedCount==4&&TestAssemblyLine->DoorCell->bDoorFastened&&RightDoorCell->bDoorFastened);
    if(RightDoorCell)UE_LOG(LogTemp,Display,TEXT("RIGHT_DOOR_DETAIL %s"),*RightDoorCell->Status);
    ResetCycle();Layout.PartOffsetsCm.Add(TEXT("part_wheel_fl"),FVector(5,0,0));Layout.PartOffsetsCm.Add(TEXT("part_door_fr"),FVector(0,3,0));Rebuild();
    Check(TEXT("moved_parts_used_by_sequence"),Run());
    ResetCycle();Layout=Original;Rebuild();History.Clear();bSnap=true;CloseRobotPopup();FocusWorkcell();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    Report->SetStringField(TEXT("scope"),TEXT("Runtime picking/drag handlers with projected camera rays, independent two-door cycle. Not OS mouse injection or certified contact physics."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryDirectReport.json")));return Pass;
}

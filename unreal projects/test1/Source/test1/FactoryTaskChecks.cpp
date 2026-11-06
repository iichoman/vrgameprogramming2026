#include "FactoryRuntime.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "FactoryAssemblyEquipment.h"
#include "RobotArm.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
class FTestPartSensor final : public IFactoryPartObservationProvider
{
public:
    FFactoryPartObservation Value;
    virtual FFactoryPartObservation ObservePart(const FString&,double) const override { return Value; }
};
}

bool AFactoryRuntime::RunTaskAssignmentChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;FString Error;
    auto Check=[&](const TCHAR* N,bool Ok){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("check"),N);O->SetBoolField(TEXT("passed"),Ok);O->SetStringField(TEXT("status"),Status);Checks.Add(MakeShared<FJsonValueObject>(O));Pass&=Ok;UE_LOG(LogTemp,Display,TEXT("TASK_TEST %s %d %s"),N,Ok,*Status);};
    auto Task=[&](const FString& Id)->FFactoryTask*{return Layout.Tasks.FindByPredicate([&](const auto& T){return T.Id==Id;});};
    const auto Original=Layout;
    Check(TEXT("legacy_migrated_to_six_independent_tasks"),Layout.Tasks.Num()==6&&ValidateAssemblyBindings(Error));
    if(!Pass)return false;
    const FString Left=Task(TEXT("wheel_fl"))->RobotId,Door=Task(TEXT("door_fl"))->RobotId;
    SelectedId=Left;const FString Before=Layout.ToJson();
    Check(TEXT("wrong_tool_rejected_without_mutation"),!AssignTask(TEXT("door_fl"))&&Layout.ToJson()==Before);
    UnassignTask(TEXT("wheel_rl"));Check(TEXT("unassigned_task_blocks_start"),!StartCycle()&&Task(TEXT("wheel_rl"))->RobotId.IsEmpty());
    UndoEdit();Check(TEXT("undo_restores_task_assignment"),Layout.ToJson()==Before);
    SelectedId=Door;DeleteSelected();Check(TEXT("delete_preserves_unassigned_door_task"),Task(TEXT("door_fl"))&&Task(TEXT("door_fl"))->RobotId.IsEmpty());
    UndoEdit();Check(TEXT("delete_undo_restores_door_executor"),Task(TEXT("door_fl"))->RobotId==Door&&ValidateAssemblyBindings(Error));
    SelectedId=Door;SetTaskPartMode(TEXT("door_fl"),true);
    FFactoryLayout Loaded;Check(TEXT("v4_roundtrip_includes_part_and_task_binding"),FFactoryLayout::FromJson(Layout.ToJson(),Loaded,Error)&&Loaded.ToJson()==Layout.ToJson());UndoEdit();
    auto Bad=Layout;Bad.Tasks[1].Id=Bad.Tasks[0].Id;Check(TEXT("duplicate_task_rejected"),!Bad.Validate(Error));
    Bad=Layout;Bad.Tasks[0].PartId=TEXT("missing");Check(TEXT("invalid_part_rejected"),!Bad.Validate(Error));
    auto Advance=[&](int N){for(int I=0;I<N;++I)StepSimulation(1./120);};
    auto Run=[&](){ResetCycle();Advance(24);bool Ok=StartCycle();for(int I=0;I<36000&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)StepSimulation(1./120);return Ok&&RunState==EFactoryRunState::Complete;};
    Check(TEXT("two_robot_tasks_full_cycle"),Run());const double TwoSeconds=SimulationSeconds;ResetCycle();
    bSnap=false;
    for(const FString Id:{TEXT("wheel_rl"),TEXT("wheel_rr")})
    {
        const auto* Base=Layout.Find(Task(Id)->RobotId);const FString Type=Base->EffectiveTypeId();const auto P=Base->Position;
        AddRegisteredItem(Type,P+FVector2D(-10,0));Check(TEXT("rear_task_reassigned_to_new_executor"),AssignTask(Id));
    }
    auto* W=TestAssemblyLine->WheelCell.Get();
    Check(TEXT("four_distinct_executors_and_sensor_banks"),W->TaskRobots[0]!=W->TaskRobots[1]&&W->TaskRobots[2]!=W->TaskRobots[3]&&W->TaskSensors.Num()==4&&W->TaskSensors[0]!=W->TaskSensors[2]);
    Check(TEXT("four_robot_tasks_full_cycle"),Run());const double FourSeconds=SimulationSeconds;
    Check(TEXT("all_parts_installed_once"),W->MountedCount==4&&TestAssemblyLine->DoorCell->bDoorFastened);
    ResetCycle();Advance(24);StartCycle();Advance(240);StopCycle();Advance(240);
    Check(TEXT("stop_in_task_mode"),RunState==EFactoryRunState::Stopped);
    ResetCycle();Check(TEXT("reset_clears_task_execution"),CanEdit()&&W->MountedCount==0);
    auto Provider=MakeShared<FTestPartSensor>();Provider->Value.PartId=TEXT("part_wheel_fr");
    Provider->Value.PartType=TEXT("wheel");Provider->Value.bValid=Provider->Value.bDetected=true;
    Provider->Value.Confidence=1;Provider->Value.SampleTimeSeconds=10;
    Provider->Value.MeasuredWorld=W->GetWheelWorld(2);Provider->Value.MeasuredWorld.AddToTranslation(FVector(0,2,0));
    auto* Bank=W->TaskSensors[1].Get();Bank->ObservationProvider=Provider;Bank->Sample(10,W,2);
    Check(TEXT("perception_pose_used_instead_of_truth"),Bank->PickupPose.Pose.Equals(Provider->Value.MeasuredWorld)&&Bank->Presence.bDetected);
    Bank->Sample(11,W,2);Check(TEXT("stale_perception_never_falls_back_to_truth"),!Bank->Presence.bValid&&!Bank->PickupPose.bValid);
    Provider->Value.SampleTimeSeconds=12;Provider->Value.PartId=TEXT("wrong_part");Bank->Sample(12,W,2);
    Check(TEXT("wrong_part_identity_rejected"),!Bank->Presence.bValid);
    Bank->ObservationProvider.Reset();
    Layout=Original;Rebuild();History.Clear();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    Report->SetNumberField(TEXT("two_robot_cycle_s"),TwoSeconds);Report->SetNumberField(TEXT("four_robot_cycle_s"),FourSeconds);
    Report->SetStringField(TEXT("scope"),TEXT("Functional allocation regression. Synthetic scene sensors, timed fastening, no collision/force certification; timing includes seeded demo events."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryTaskReport.json")));return Pass;
}

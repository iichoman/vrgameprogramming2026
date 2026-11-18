#include "FactoryRuntime.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "FactoryAssemblyEquipment.h"
#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "VehicleAssemblyMounting.h"
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
// Exact wheel geometry, independent of the arm's IK branch. This verifies that
// the authored hub pose actually admits straight insertion, including the rim.
bool RearWheelInsertionClear(AWheelAssemblyCell& Cell,int Index,FString& Why)
{
    auto* Arm=Cell.RobotForWheel(Index);USceneComponent* Part=nullptr;
    TInlineComponentArray<USceneComponent*> Components(&Cell);
    for(auto* C:Components)if(C->GetName()==FString::Printf(TEXT("Wheel%d"),Cell.PartIndex(Index)))Part=C;
    if(!Arm||!Part){Why=TEXT("Missing wheel/robot");return false;}
    FRobotMotionScene Scene;Scene.Capture(*Arm,Part,true);
    Scene.Shapes.RemoveAll([](const auto& S){return !S.bPayload;});
    if(Scene.Shapes.IsEmpty()){Why=TEXT("Missing carried wheel geometry");return false;}
    Scene.PrepareGeometry(true);
    const auto OriginalShapes=Scene.Shapes;const auto TCP=Arm->GetTCPWorld();const auto PartPose=Part->GetComponentTransform();
    for(int Out=40;Out>=0;--Out)
    {
        auto Goal=Cell.GetMountWorld(Index);Goal.AddToTranslation(Goal.TransformVectorNoScale(FVector(0,Index==1?-Out:Out,0)));
        for(int J=0;J<Scene.Shapes.Num();++J)
            Scene.Shapes[J].Local=(OriginalShapes[J].Local*TCP).GetRelativeTransform(PartPose)*Goal.GetRelativeTransform(TCP);
        if(!Scene.Clear(*Arm,Arm->GetAnglesRadians(),Why))return false;
    }
    return true;
}

}

bool AFactoryRuntime::RunTaskAssignmentChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;FString Error;
    auto Check=[&](const TCHAR* N,bool Ok){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("check"),N);O->SetBoolField(TEXT("passed"),Ok);O->SetStringField(TEXT("status"),Status);Checks.Add(MakeShared<FJsonValueObject>(O));Pass&=Ok;UE_LOG(LogTemp,Display,TEXT("TASK_TEST %s %d %s"),N,Ok,*Status);};
    auto Task=[&](const FString& Id)->FFactoryTask*{return Layout.Tasks.FindByPredicate([&](const auto& T){return T.Id==Id;});};
    const auto Original=Layout;
    Check(TEXT("legacy_migrated_to_eight_independent_tasks"),Layout.Tasks.Num()==8&&ValidateAssemblyBindings(Error));
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
    double HeadSwing[2]={0,0};bool RearPickupObserved[2]={false,false};
    bool RearClearanceChecked=false,RearClearance=true;FString RearClearanceFailure;
    auto Run=[&](){ResetCycle();Advance(24);bool Ok=StartCycle();for(int I=0;I<36000&&RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;++I)
    {
        StepSimulation(1./120);auto* Cell=TestAssemblyLine->WheelCell.Get();
        if(!RearClearanceChecked&&TestAssemblyLine->Stage==EAssemblyLineStage::Wheels&&!Cell->bTransporting)
        {
            RearClearanceChecked=true;
            for(int Index:{1,3})RearClearance=RearWheelInsertionClear(*Cell,Index,RearClearanceFailure)&&RearClearance;
        }
        for(int Side=0;Side<2;++Side)if(const auto* Lane=Cell->LaneSnapshot(Side))
        {
            if(Lane->WheelIndex!=Side*2+1||Lane->bHeld||(Lane->Stage!=EWheelStage::ApproachPickup&&Lane->Stage!=EWheelStage::Grip))continue;
            const auto* Arm=Side==0?Cell->LeftRobot.Get():Cell->RightRobot.Get();
            const auto Reference=(Side==0?Cell->LeftTCPInWheel:Cell->RightTCPInWheel)*Lane->Pickup;
            HeadSwing[Side]=FMath::Max(HeadSwing[Side],FMath::RadiansToDegrees(Arm->GetTCPWorld().GetRotation().AngularDistance(Reference.GetRotation())));
            RearPickupObserved[Side]=true;
        }
    }return Ok&&RunState==EFactoryRunState::Complete;};
    Check(TEXT("two_robot_tasks_full_cycle"),Run());const double TwoSeconds=SimulationSeconds;
    Check(TEXT("lifted_rear_wheels_allow_straight_insertion"),RearClearanceChecked&&RearClearance);
    if(!RearClearance)UE_LOG(LogTemp,Warning,TEXT("REAR_INSERTION_CLEARANCE %s"),*RearClearanceFailure);
    auto* TwoWheelCell=TestAssemblyLine->WheelCell.Get();
    Check(TEXT("four_wheels_fastened_before_door_transfer"),TwoWheelCell->MountedCount==4);
    bool HubRestored=TwoWheelCell->MountedCount==4&&VehicleAssemblyMounting::HubOffset(*TwoWheelCell->Assembly,1).IsNearlyZero(.0001);
    for(int Index:{1,3})HubRestored&=TwoWheelCell->GetWheelWorld(Index).GetLocation().Equals(TwoWheelCell->GetMountWorld(Index).GetLocation(),.1);
    Check(TEXT("rear_wheels_follow_hubs_after_lift_lowering"),HubRestored);
    const double LeftPickupSeconds=TwoWheelCell->LaneSnapshot(0)->NextPickupSeconds;
    const double RightPickupSeconds=TwoWheelCell->LaneSnapshot(1)->NextPickupSeconds;
    const double LeftHeadSwing=HeadSwing[0],RightHeadSwing=HeadSwing[1];
    Check(TEXT("rear_pickup_without_head_flip"),RearPickupObserved[0]&&RearPickupObserved[1]&&LeftPickupSeconds>0&&RightPickupSeconds>0&&LeftHeadSwing<1&&RightHeadSwing<1);
    UE_LOG(LogTemp,Display,TEXT("MOTION_TEST stage %d wheel %d left_next %.3f right_next %.3f"),int32(TwoWheelCell->FailedStage),TwoWheelCell->FailedWheelIndex,LeftPickupSeconds,RightPickupSeconds);
    ResetCycle();
    bSnap=false;
    for(const FString Id:{TEXT("wheel_rl"),TEXT("wheel_rr")})
    {
        const auto* Base=Layout.Find(Task(Id)->RobotId);const FString Type=Base->EffectiveTypeId();const auto P=Base->Position;
        const int Index=Id==TEXT("wheel_rl")?1:3;
        const FVector StationDelta=TestAssemblyLine->WheelCell->GetMountWorld(Index).GetLocation()-TestAssemblyLine->WheelCell->GetMountWorld(Index-1).GetLocation();
        AddRegisteredItem(Type,P+FVector2D(StationDelta));Check(TEXT("rear_task_reassigned_to_new_executor"),AssignTask(Id));
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
    Report->SetNumberField(TEXT("left_second_pickup_s"),LeftPickupSeconds);Report->SetNumberField(TEXT("right_second_pickup_s"),RightPickupSeconds);
    Report->SetNumberField(TEXT("left_rear_pickup_tcp_rotation_deg"),LeftHeadSwing);Report->SetNumberField(TEXT("right_rear_pickup_tcp_rotation_deg"),RightHeadSwing);
    Report->SetStringField(TEXT("scope"),TEXT("Functional allocation regression. Synthetic scene sensors, timed fastening, no collision/force certification; timing includes seeded demo events."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryTaskReport.json")));return Pass;
}

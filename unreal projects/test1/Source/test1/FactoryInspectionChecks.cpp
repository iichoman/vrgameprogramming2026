#include "FactoryRuntime.h"
#include "FactoryEquipment.h"
#include "Components/WidgetComponent.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

bool AFactoryRuntime::RunInspectionChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;
    auto Check=[&](const TCHAR* Name,bool OK){auto C=MakeShared<FJsonObject>();C->SetStringField(TEXT("check"),Name);C->SetBoolField(TEXT("passed"),OK);Checks.Add(MakeShared<FJsonValueObject>(C));Pass&=OK;};
    Check(TEXT("slate_keyboard_camera_and_text_focus_regression"),RunUIChecks());
    FFactoryInspectionModel Probe;FFactorySensorValue V;V.Id=TEXT("probe");
    Probe.Publish(V);Check(TEXT("disconnected_is_not_zero_measurement"),Probe.Sensors[V.Id].Latest.State()==EFactorySampleState::Disconnected&&Probe.Sensors[V.Id].History.IsEmpty());
    V.Connected=true;V.Valid=true;V.SampleTime=1;V.ClockTime=1;V.Value=12.5;Probe.Publish(V);
    Check(TEXT("valid_value_and_units_are_preserved"),Probe.Sensors[V.Id].History.Num()==1&&Probe.Sensors[V.Id].Latest.Value==12.5);
    Probe.Publish(V);Check(TEXT("unchanged_timestamp_is_not_duplicate_sample"),Probe.Sensors[V.Id].History.Num()==1);
    V.ClockTime=2;Probe.Publish(V);Check(TEXT("old_sample_reported_stale"),Probe.Sensors[V.Id].Latest.State()==EFactorySampleState::Stale);
    Check(TEXT("stale_sample_breaks_trace_without_faking_measurement"),!Probe.Sensors[V.Id].History.Last().Valid);
    V.SampleTime=V.ClockTime=3;V.Valid=false;Probe.Publish(V);Check(TEXT("invalid_sample_creates_graph_gap"),!Probe.Sensors[V.Id].History.Last().Valid);
    V.Valid=true;for(int N=4;N<210;++N){V.SampleTime=V.ClockTime=N;Probe.Publish(V);}
    Check(TEXT("sensor_history_is_bounded"),Probe.Sensors[V.Id].History.Num()==180);
    V.SampleTime=V.ClockTime=0;Probe.Publish(V);Check(TEXT("clock_reset_starts_new_trace"),Probe.Sensors[V.Id].History.Num()==1);
    RefreshInspection();auto& M=Inspection();Check(TEXT("future_camera_has_no_fabricated_reading"),M.Sensors.Contains(TEXT("extension.camera"))&&M.Sensors[TEXT("extension.camera")].Latest.State()==EFactorySampleState::Disconnected);
    Check(TEXT("future_pressure_has_no_fabricated_reading"),M.Sensors.Contains(TEXT("extension.pressure"))&&M.Sensors[TEXT("extension.pressure")].Latest.State()==EFactorySampleState::Disconnected);
    const FFactoryLayout Original=Layout;const auto InitialState=RunState;
    bool NoLabels=true;for(const auto& P:Equipment)if(IsValid(P.Value.Get()))NoLabels&=!IsValid(P.Value->Label);
    Check(TEXT("no_rotating_world_name_widgets"),NoLabels);
    Check(TEXT("body_is_inspectable"),ObjectMeshes(TEXT("vehicle_1")).Num()>0);
    bool AllParts=true;for(const auto& S:FactoryTasks::Catalog())AllParts&=ObjectMeshes(TEXT("part_")+S.Id).Num()>0;
    Check(TEXT("all_six_process_parts_have_selection_geometry"),AllParts);
    auto Meshes=ObjectMeshes(TEXT("part_wheel_fl"));UMaterialInterface* Before=Meshes.IsEmpty()?nullptr:Meshes[0]->GetOverlayMaterial();
    M.Issues.Empty();SelectObject(TEXT("part_wheel_fl"));UpdateObjectHighlights();
    Check(TEXT("selected_part_changes_actual_mesh_overlay"),!Meshes.IsEmpty()&&Meshes[0]->GetOverlayMaterial()==M.SelectedMaterial.Get()&&M.SelectedMaterial.IsValid());
    SelectObject(TEXT("vehicle_1"));UpdateObjectHighlights();Check(TEXT("deselection_restores_original_overlay"),!Meshes.IsEmpty()&&Meshes[0]->GetOverlayMaterial()==Before);
    const FString Robot=Layout.Tasks[0].RobotId;SelectObject(Robot);UpdateObjectHighlights();
    Check(TEXT("robot_selection_uses_real_robot_mesh"),ObjectMeshes(Robot).Num()>0&&M.OverlayKinds.Num()>0);
    const FString Snapshot=Layout.ToJson();RunState=EFactoryRunState::Transfer;SelectObject(TEXT("part_door_fr"));
    Check(TEXT("inspection_allowed_during_run_without_edit"),CurrentObjectId()==TEXT("part_door_fr")&&Layout.ToJson()==Snapshot);RunState=InitialState;
    Layout.Tasks[0].RobotId.Empty();RefreshInspection();const FString IssueId=TEXT("unassigned:")+Layout.Tasks[0].Id;
    const auto* Found=M.Issues.FindByPredicate([&](const auto& I){return I.Id==IssueId;});
    Check(TEXT("unassigned_task_identifies_part_and_process"),Found&&!Found->PartId.IsEmpty()&&Found->TaskId==Layout.Tasks[0].Id&&Found->Objects.Contains(TEXT("vehicle_1")));
    FocusIssue(IssueId);Check(TEXT("issue_focus_opens_problem_panel_and_selects_model"),M.FocusedIssue==IssueId&&M.Tab==EFactoryDetailTab::Problems&&!CurrentObjectId().IsEmpty());
    Layout=Original;RefreshInspection();Check(TEXT("resolved_issue_is_removed"),!M.Issues.ContainsByPredicate([&](const auto& I){return I.Id==IssueId;}));
    OpenRobotPopup(Robot,FVector2D::ZeroVector);PendingTaskId=TEXT("wheel_fl");
    Check(TEXT("incompatible_part_assignment_is_rejected"),!ChoosePart(TEXT("part_door_fr")));
    Check(TEXT("assignment_error_has_robot_part_and_process_identity"),M.InteractionIssue.IsSet()&&M.InteractionIssue->RobotId==Robot&&M.InteractionIssue->PartId==TEXT("part_door_fr")&&M.InteractionIssue->TaskId==TEXT("wheel_fl"));
    UpdateObjectHighlights();const auto DoorMeshes=ObjectMeshes(TEXT("part_door_fr"));
    Check(TEXT("error_highlights_actual_part_mesh"),!DoorMeshes.IsEmpty()&&DoorMeshes[0]->GetOverlayMaterial()==M.IssueMaterial.Get());
    CloseRobotPopup();SelectObject(FString());RefreshInspection();UpdateObjectHighlights();

    Layout=Original;RunState=InitialState;SelectObject(Robot);RefreshInspection();FocusWorkcell();CreatePanel();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    Report->SetStringField(TEXT("scope"),TEXT("Native inspector/telemetry/real mesh overlay and synthetic Slate input checks; no physical sensor validation."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    return FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryInspectionReport.json")))&&Pass;
}

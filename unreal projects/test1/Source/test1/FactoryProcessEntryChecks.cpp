#include "FactoryRuntime.h"
#include "AutonomousAssemblyLine.h"
#include "AssemblySituationCatalog.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "CamryAssembly.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

bool AFactoryRuntime::RunProcessEntryChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;
    auto Check=[&](const TCHAR* Name,bool OK){auto C=MakeShared<FJsonObject>();C->SetStringField(TEXT("check"),Name);C->SetBoolField(TEXT("passed"),OK);
        Checks.Add(MakeShared<FJsonValueObject>(C));Pass&=OK;UE_LOG(LogTemp,Display,TEXT("PROCESS_ENTRY %s %s: %s"),Name,OK?TEXT("PASS"):TEXT("FAIL"),*Status);};
    auto* L=TestAssemblyLine.Get();if(!L||!L->WheelCell)return false;auto* W=L->WheelCell.Get();
    auto Running=[this](){return RunState>=EFactoryRunState::Approach&&RunState<=EFactoryRunState::Verify;};
    auto Finish=[&](){for(int I=0;I<120*180&&Running();++I)StepSimulation(1./120);};
    L->ConfigureIncidentDemo(0);ResetCycle();
    Check(TEXT("idle_skip_moves_vehicle_and_support_to_door_station"),SkipToNextProcess()
        &&W->Assembly->GetActorLocation().Equals(L->VehicleAtWheelStation.GetLocation()+L->DoorStationOffsetCm,.001)
        &&W->TransportCarriage->GetActorLocation().Equals(L->CarriageAtWheelStation.GetLocation()+L->DoorStationOffsetCm,.001)
        &&L->ArrivalPhase==EArrivalPhase::Released&&L->Conveyor->Velocity.IsNearlyZero());
    Check(TEXT("skipped_wheels_are_not_certified_or_attached"),W->MountedCount==0&&W->Outcome==EWheelOutcome::NotEvaluated&&L->WasProcessSkipped(TEXT("wheels")));
    Finish();Check(TEXT("direct_door_entry_completes_four_real_door_operations"),RunState==EFactoryRunState::Complete&&L->MountedDoorCount==4&&W->MountedCount==0);
    Check(TEXT("last_process_cannot_skip_and_result_retains_omission"),!L->CanSkipToNextProcess()&&!SkipToNextProcess()&&ResultDetails().Contains(TEXT("건너뜀 / 미검사")));
    ResetCycle();Check(TEXT("reset_restores_initial_route"),CanEdit()&&L->CanSkipToNextProcess()&&!L->WasProcessSkipped(TEXT("wheels"))&&L->ArrivalPhase==EArrivalPhase::Ready);
    for(int I=0;I<24;++I)StepSimulation(1./120);bool Began=StartCycle(),Held=false;
    for(int I=0;I<120*50&&Running();++I){StepSimulation(1./120);const auto* Lane=W->LaneSnapshot(0);if(Lane&&Lane->bHeld){Held=true;break;}}
    const bool Skipped=SkipToNextProcess();bool NoneHeld=true;
    for(int I=0;I<W->LaneCount();++I)if(const auto* Lane=W->LaneSnapshot(I))NoneHeld&=!Lane->bHeld;
    Check(TEXT("mid_grip_skip_cancels_wheel_work_and_returns_parts_to_supply"),Began&&Held&&Skipped&&NoneHeld&&W->MountedCount==0&&W->Outcome==EWheelOutcome::NotEvaluated);
    for(int Preset:{20,21})
    {
        L->ConfigureIncidentDemo(Preset);ResetCycle();const bool Entered=SkipToNextProcess();Finish();
        bool Debris=false,Damage=false;for(auto P:L->DoorCell->PartPlants)if(IsValid(P)){Debris|=P->ObservedSeatInterferenceMm>1;Damage|=!P->Body.Damage.IsEmpty();}
        Check(Preset==20?TEXT("skipped_entry_retains_debris_sensor_gate"):TEXT("skipped_entry_retains_force_damage_response"),
            Entered&&RunState==EFactoryRunState::Fault&&(Preset==20?Debris:Damage));
    }
    int Normals=0;bool ScopeOK=true,Replay=true;
    for(int Seed=1;Seed<=1000;++Seed)
    {
        const int Choice=AssemblySituations::Choose(Seed,TEXT("doors"),.35);Normals+=Choice==0;
        ScopeOK&=Choice==0||Choice==11||Choice==12||Choice==20||Choice==21;
        Replay&=Choice==AssemblySituations::Choose(Seed,TEXT("doors"),.35);
    }
    Check(TEXT("random_includes_normal_and_only_eligible_stage_incidents"),ScopeOK&&Normals>300&&Normals<400&&Replay
        &&AssemblySituations::Choose(1,NAME_None,1)==0&&AssemblySituations::Choose(1,NAME_None,0)!=0);
    L->SelectRandomSituation();ResetCycle();Check(TEXT("random_skip_redraws_for_target_process_and_keeps_selection"),SkipToNextProcess()&&L->bRandomSituation
        &&(L->SelectedIncidentPreset==0||L->SelectedIncidentPreset==11||L->SelectedIncidentPreset==12||L->SelectedIncidentPreset>=20));
    L->ConfigureIncidentDemo(0);ResetCycle();for(int I=0;I<24;++I)StepSimulation(1./120);
    const bool FullStart=StartCycle();
    for(int I=0;I<120*180&&Running()&&L->Stage!=EAssemblyLineStage::Transfer;++I)StepSimulation(1./120);
    const bool WheelsPassed=W->Outcome==EWheelOutcome::Pass;
    Check(TEXT("transfer_skip_keeps_completed_wheels_and_their_result"),FullStart&&WheelsPassed&&SkipToNextProcess()
        &&W->MountedCount==4&&W->Outcome==EWheelOutcome::Pass&&!L->WasProcessSkipped(TEXT("wheels")));
    // Two future process adapters prove the button has no door/last-stage switch.
    L->ConfigureIncidentDemo(0);ResetCycle();
    auto Counts=MakeShared<TArray<int32>>();Counts->Init(0,3);
    auto Register=[&](const TCHAR* Id,int Index){FAssemblyProcessEntry P;P.Id=Id;P.Label=Id;
        P.Validate=[](FString&){return true;};P.Enter=[L,Counts,Index](){++(*Counts)[Index];L->WheelCell->Assembly->AddActorWorldOffset(FVector(100,0,0));return true;};
        P.Advance=[](double){return EAssemblyProcessProgress::Complete;};P.Stop=[Counts](){++(*Counts)[2];};return L->RegisterFollowingProcess(MoveTemp(P));};
    const bool Registered=Register(TEXT("glass_test"),0)&&Register(TEXT("seat_test"),1);
    const bool EnteredDoors=SkipToNextProcess();StepSimulation(1./120);
    const bool EnteredGlass=SkipToNextProcess();StepSimulation(1./120); // normal completion advances to seat
    Check(TEXT("registered_glass_and_seat_use_same_skip_and_normal_progression"),Registered&&EnteredDoors&&EnteredGlass
        &&(*Counts)[0]==1&&(*Counts)[1]==1&&L->WasProcessSkipped(TEXT("doors")));
    Check(TEXT("registered_final_process_disables_skip"),!L->CanSkipToNextProcess());
    StepSimulation(1./120);Check(TEXT("registered_final_process_completes"),RunState==EFactoryRunState::Complete);
    ResetCycle();FAssemblyProcessEntry Invalid;Invalid.Id=TEXT("invalid_test");Invalid.Label=TEXT("invalid");
    Check(TEXT("incomplete_process_adapter_is_rejected"),!L->RegisterFollowingProcess(MoveTemp(Invalid)));
    FAssemblyProcessEntry Blocked;Blocked.Id=TEXT("blocked_test");Blocked.Label=TEXT("blocked");
    Blocked.Validate=[](FString& Error){Error=TEXT("시험: 다음 작업 위치 무효");return false;};
    Blocked.Enter=[](){return false;};Blocked.Advance=[](double){return EAssemblyProcessProgress::Fault;};Blocked.Stop=[](){};
    const bool Added=L->RegisterFollowingProcess(MoveTemp(Blocked));
    SkipToNextProcess();SkipToNextProcess();SkipToNextProcess();const FVector Before=W->Assembly->GetActorLocation();
    Check(TEXT("invalid_next_position_does_not_move_vehicle_or_mark_skipped"),Added&&!SkipToNextProcess()
        &&Before.Equals(W->Assembly->GetActorLocation(),.001)&&!L->WasProcessSkipped(TEXT("seat_test")));
    const int StopsBefore=(*Counts)[2];StepSimulation(1./120);Check(TEXT("automatic_entry_validation_failure_stops_the_route"),RunState==EFactoryRunState::Fault&&L->Stage==EAssemblyLineStage::Fault&&(*Counts)[2]==StopsBefore+1);
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    return FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryProcessEntryReport.json")))&&Pass;
}

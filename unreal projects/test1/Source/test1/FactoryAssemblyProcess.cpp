#include "FactoryProcessRunner.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "FactoryEquipment.h"
#include "EngineUtils.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
AFactoryRuntime* Runtime(const UActorComponent* C){return Cast<AFactoryRuntime>(C->GetOwner());}
bool Running(const AFactoryRuntime* R){return R->RunState>=EFactoryRunState::Approach&&R->RunState<=EFactoryRunState::Verify;}
}
bool UFactoryAssemblyProcessRunner::Validate(FString& Error) const
{
    const auto* R=Runtime(this);const auto* L=R?R->TestAssemblyLine.Get():nullptr;
    if(!IsValid(L)||!IsValid(L->WheelCell)||!IsValid(L->DoorCell)||!L->Sensors||!L->Conveyor
        ||!IsValid(L->WheelCell->Assembly)||L->DoorCell->Assembly!=L->WheelCell->Assembly
        ||!IsValid(L->WheelCell->LeftRobot)||!IsValid(L->WheelCell->RightRobot)||!IsValid(L->DoorCell->Robot)
        ||!IsValid(L->WheelCell->TransportCarriage))
    {Error=TEXT("통합 셀 참조 누락: 로봇·차체·운반대·센서 연결을 확인하세요.");return false;}
    // Conservative reserved envelope for this fixed-layout test, not a safety certificate.
    FFactoryLayoutItem Envelope;Envelope.Position=FVector2D(350,0);Envelope.Size=FVector2D(3000,1800);
    if(!R->Layout.Contains(Envelope)){Error=TEXT("공장 경계가 고정 통합 셀의 예약 영역을 벗어났습니다.");return false;}
    for(const auto& I:R->Layout.Items)if(I.Kind!=EFactoryItemKind::Zone)
    {Error=TEXT("통합 시험 배치는 구역만 편집합니다. 기본 배치로 복원하세요.");return false;}
    return true;
}
bool UFactoryAssemblyProcessRunner::Start()
{
    auto* R=Runtime(this);if(!R||!R->CanEdit())return false;
    R->CancelEdit();FString Error;
    if(!Validate(Error)){R->Status=Error;return false;}
    auto* L=R->TestAssemblyLine.Get();
    L->SetActorTickEnabled(false);
    if(!L->StartLine()){R->Status=L->Status;return false;}
    R->SimulationSeconds=R->TravelCm=R->AlignmentMm=R->SimRemainder=0;
    R->RunState=EFactoryRunState::Transfer;R->Status=L->Status;return true;
}
void UFactoryAssemblyProcessRunner::Stop()
{
    auto* R=Runtime(this);if(!R||!Running(R)||!IsValid(R->TestAssemblyLine))return;
    R->TestAssemblyLine->StopLine();R->RunState=EFactoryRunState::Stopped;R->Status=TEXT("정지 명령 · 반입 중에는 감속 후 정지 / 편집 복귀로 초기화");
}
void UFactoryAssemblyProcessRunner::Reset()
{
    auto* R=Runtime(this);if(!R)return;
    R->SimulationSeconds=R->TravelCm=R->AlignmentMm=R->SimRemainder=0;
    auto* L=R->TestAssemblyLine.Get();
    if(!IsValid(L)){R->RunState=EFactoryRunState::Fault;R->Status=TEXT("통합 테스트 라인이 연결되지 않았습니다.");return;}
    L->SetActorTickEnabled(false);
    if(!L->ResetLine()){R->RunState=EFactoryRunState::Fault;R->Status=L->Status;return;}
    R->RunState=EFactoryRunState::Editing;R->Status=TEXT("통합 셀 준비 · 센서 갱신 후 작업 실행 / 구역 편집은 설비 위치와 별개");
}
void UFactoryAssemblyProcessRunner::Step(double Dt)
{
    auto* R=Runtime(this);auto* L=R?R->TestAssemblyLine.Get():nullptr;
    if(!IsValid(L))return;
    // All phases, including sensor warm-up and braking, use the one factory clock.
    if(Running(R))R->SimulationSeconds+=Dt;
    const bool Ok=L->AdvanceLine(Dt);
    if(Running(R))
    {
        R->Status=L->Status;
        if(!Ok||L->Stage==EAssemblyLineStage::Fault||L->ArrivalPhase==EArrivalPhase::Fault)
            R->RunState=EFactoryRunState::Fault;
        else if(L->Stage==EAssemblyLineStage::Complete)
        {R->RunState=EFactoryRunState::Complete;R->LastCycleSeconds=R->SimulationSeconds;}
    }
}
FString AFactoryRuntime::AssemblyMetrics() const
{
    auto* L=TestAssemblyLine.Get();if(!IsValid(L))return TEXT("통합 라인 연결 없음");
    const auto* W=L->WheelCell.Get();const auto* D=L->DoorCell.Get();
    FString Sensor=W?W->GetLeftSensorSummary():TEXT("휠 센서 없음");
    return FString::Printf(TEXT("경과 %.2f초 · 휠 %d/4 · 왼쪽 앞문 %s\n고장 주입 %d / 복구 %d · 시드 %d\n%s\n%s"),
        SimulationSeconds,W?W->MountedCount:0,D&&D->bDoorFastened?TEXT("완료"):TEXT("대기"),
        L->InjectedEvents,L->RecoveredEvents,L->CurrentSeed,*L->ActiveEvent,*Sensor);
}

bool AFactoryRuntime::RunAssemblyIntegrationChecks()
{
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;
    auto Check=[&](const TCHAR* Name,bool Ok){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("check"),Name);O->SetBoolField(TEXT("passed"),Ok);Checks.Add(MakeShared<FJsonValueObject>(O));Pass&=Ok;UE_LOG(LogTemp,Display,TEXT("ASSEMBLY_INTEGRATION %s %s %s"),Name,Ok?TEXT("PASS"):TEXT("FAIL"),*Status);};
    FString Error;Check(TEXT("explicit_cell_references"),bUseAssemblyIntegration&&AssemblyProcess->Validate(Error));
    if(!Pass)return false;
    auto* L=TestAssemblyLine.Get();const int32 Seed=L->BaseSeed;
    int32 OldRobots=0;for(TActorIterator<AFactoryEquipment> It(GetWorld());It;++It)
        if(It->GetOwner()==this&&It->GetDefinition().Kind!=EFactoryItemKind::Zone)++OldRobots;
    Check(TEXT("old_factory_equipment_replaced"),OldRobots==0&&VehicleRoot==nullptr&&WheelRoot==nullptr);
    ResetCycle();Check(TEXT("only_factory_drives_line"),!L->IsActorTickEnabled()&&L->WheelCell->bExternallyDriven&&L->DoorCell->bExternallyDriven);
    Check(TEXT("start_waits_for_fresh_speed_sample"),!StartCycle()&&CanEdit());
    auto Advance=[&](int32 N){for(int32 I=0;I<N;++I)StepSimulation(1./120);};
    Advance(24);Check(TEXT("factory_ui_start_command"),StartCycle());Advance(240);StopCycle();Advance(240);
    const FVector Stopped=L->WheelCell->Assembly->GetActorLocation();Advance(60);
    Check(TEXT("stop_brakes_conveyor_and_preserves_stopped_state"),RunState==EFactoryRunState::Stopped&&L->Conveyor->Velocity.IsNearlyZero(.01)&&L->WheelCell->Assembly->GetActorLocation().Equals(Stopped,.001));
    for(int32 Run=0;Run<3;++Run)
    {
        L->BaseSeed=9300+Run*37;L->RunNumber=0;ResetCycle();Advance(24);
        bool Ok=StartCycle();
        for(int32 I=0;I<120*300&&Running(this);++I)StepSimulation(1./120);
        Check(*FString::Printf(TEXT("seed_%d_full_wheels_and_door_cycle"),L->CurrentSeed),Ok&&RunState==EFactoryRunState::Complete&&L->WheelCell->MountedCount==4&&L->DoorCell->bDoorFastened&&L->InjectedEvents==2&&L->RecoveredEvents==2);
    }
    ResetCycle();Advance(24);StartCycle();Advance(240);L->Sensors->bPublishPosition=false;Advance(120);
    Check(TEXT("sensor_dropout_stops_run"),RunState==EFactoryRunState::Fault);
    L->BaseSeed=Seed;L->RunNumber=0;ResetCycle();
    Check(TEXT("reset_returns_to_editing"),CanEdit()&&L->WheelCell->MountedCount==0);
    const auto Original=Layout;Layout.WidthCm=1200;Check(TEXT("hall_resize_cannot_authorize_outside_test_cell"),!AssemblyProcess->Validate(Error));Layout=Original;
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    Report->SetStringField(TEXT("scope"),TEXT("Fixed layout integration, not arbitrary equipment placement or force/contact simulation."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    return FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryAssemblyIntegrationReport.json")))&&Pass;
}

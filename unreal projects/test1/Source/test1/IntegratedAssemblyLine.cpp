#include "IntegratedAssemblyLine.h"
#include "SensorAssemblyLine.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "DoorPoseComponent.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "VehicleProcessTest.h"
#include "VehicleAssemblyMounting.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/InputComponent.h"
#include "Camera/CameraActor.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
    FString StageLabel(EAssemblyLineStage Stage)
    {
        switch (Stage)
        {
        case EAssemblyLineStage::Idle: return TEXT("대기");
        case EAssemblyLineStage::Wheels: return TEXT("차체 반입 · 타이어 조립");
        case EAssemblyLineStage::Transfer: return TEXT("문 조립 구역 이송");
        case EAssemblyLineStage::Door: return TEXT("문 4개 정렬 · 체결");
        case EAssemblyLineStage::Complete: return TEXT("완료 · 품질 합격");
        case EAssemblyLineStage::Stopped: return TEXT("운전자 정지");
        case EAssemblyLineStage::Fault: return TEXT("이상 감지 · 안전 정지");
        default: return TEXT("상태 확인 중");
        }
    }

    FString ProcessRail(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("○ 1 차체 반입  ─  ○ 2 타이어 조립  ─  ○ 3 구역 이송  ─  ○ 4 문 4개 조립  ─  ○ 품질 판정");
        int32 Current = 0;
        if (Line->Stage == EAssemblyLineStage::Wheels)
            Current = Line->WheelCell && Line->WheelCell->bTransporting ? 0 : 1;
        else if (Line->Stage == EAssemblyLineStage::Transfer) Current = 2;
        else if (Line->Stage == EAssemblyLineStage::Door) Current = 3;
        else if (Line->Stage == EAssemblyLineStage::Complete) Current = 4;
        const TCHAR* Names[] = {TEXT("차체 반입"), TEXT("타이어 조립"), TEXT("구역 이송"), TEXT("문 4개 조립"), TEXT("품질 판정")};
        FString Result;
        for (int32 Index=0; Index<5; ++Index)
        {
            if (Index) Result += TEXT("  ─  ");
            const bool Done = Line->Stage == EAssemblyLineStage::Complete || Index < Current;
            Result += FString::Printf(TEXT("%s %d %s"), Index == Current && Line->Stage != EAssemblyLineStage::Complete ? TEXT("●") : Done ? TEXT("✓") : TEXT("○"), Index+1, Names[Index]);
        }
        return Result;
    }

    FString CurrentOperation(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("조립 라인 액터를 찾는 중입니다.");
        switch (Line->Stage)
        {
        case EAssemblyLineStage::Idle: return TEXT("차체·로봇·센서의 초기 상태를 확인하고 시작 명령을 기다립니다.");
        case EAssemblyLineStage::Wheels:
            if (Line->WheelCell && Line->WheelCell->bTransporting) return TEXT("컨베이어가 차체를 작업 기준점으로 반입하고 도킹 여부를 확인합니다.");
            return FString::Printf(TEXT("좌·우 로봇이 타이어를 파지해 허브에 정렬·체결합니다. 현재 %d/4개 완료."), Line->WheelCell ? Line->WheelCell->MountedCount : 0);
        case EAssemblyLineStage::Transfer: return FString::Printf(TEXT("타이어 조립이 끝난 차체를 문 조립 구역으로 이송합니다. 진행률 %.0f%%."),Line->TransferProgress*100.0);
        case EAssemblyLineStage::Door:
            return FString::Printf(TEXT("문 로봇이 각 문을 파지·운반하고 센서 자세를 검증해 체결합니다. %d/4개 완료 · 현재: %s"),
                Line->MountedDoorCount, Line->DoorCell ? *Line->DoorCell->GetStateLabel() : TEXT("연결 확인 중"));
        case EAssemblyLineStage::Complete: return TEXT("타이어 4개와 문 4개 체결을 마치고 최종 자세 검증을 통과했습니다.");
        case EAssemblyLineStage::Stopped: return TEXT("운전자 정지 명령으로 모든 구동 목표를 유지합니다. 원인 확인 후 초기화가 필요합니다.");
        case EAssemblyLineStage::Fault: return TEXT("이상을 감지하여 후속 공정을 차단하고 로봇과 이송 장치를 안전 정지했습니다.");
        default: return Line->Status;
        }
    }

    FString IncidentSummary(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("상태 데이터 없음");
        if (const auto* Autonomous = Cast<AAutonomousAssemblyLine>(Line))
        {
            if (!Autonomous->ActiveEvent.IsEmpty())
                return FString::Printf(TEXT("감지 이벤트: %s\n발생 %d/2 · 자동 복구 %d/2"), *Autonomous->ActiveEvent, Autonomous->InjectedEvents, Autonomous->RecoveredEvents);
        }
        if (Line->Stage == EAssemblyLineStage::Fault) return TEXT("발생 문제: ") + Line->Status;
        if (Line->Stage == EAssemblyLineStage::Stopped) return TEXT("발생 상황: 운전자 정지 명령");
        if (const auto* Sensor = Cast<ASensorAssemblyLine>(Line))
        {
            const bool Healthy = Sensor->Sensors && Sensor->Sensors->Healthy(Sensor->SensorTime);
            return Healthy ? TEXT("현재 감지된 이상 없음 · 도착/점유/위치/속도 센서 정상") : TEXT("센서 갱신 지연 또는 무효값 감지");
        }
        return TEXT("현재 감지된 이상 없음");
    }

    FString ResponseSummary(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line)) return TEXT("대응 판단 대기");
        if (Line->Stage == EAssemblyLineStage::Fault) return TEXT("현재 대응: 구동 정지 → 영향 공정 차단 → 원인 확인 → R로 안전 초기화");
        if (Line->Stage == EAssemblyLineStage::Stopped) return TEXT("현재 대응: 자세 유지 → 작업자 확인 → R로 초기 조건 복구");
        if (const auto* Autonomous = Cast<AAutonomousAssemblyLine>(Line))
        {
            if (Autonomous->InjectedEvents > Autonomous->RecoveredEvents)
                return TEXT("현재 대응: 해당 로봇 대기 → 공급/센서/그리퍼 복구 → 새 측정값 검증 → 자동 재시도");
        }
        if (Line->Stage == EAssemblyLineStage::Door) return TEXT("판단 기준: TCP 수렴 → 정렬 센서 허용 범위 → 독립 품질 검증 → 체결 승인");
        if (Line->Stage == EAssemblyLineStage::Wheels) return TEXT("판단 기준: 부품 존재·자세 확인 → 파지 확인 → 정렬 오차 확인 → 체결 승인");
        return TEXT("현재 대응: 공정 조건을 계속 감시하며 다음 단계 진입 조건을 확인합니다.");
    }

    FLinearColor StateColor(const AIntegratedAssemblyLine* Line)
    {
        if (!IsValid(Line) || Line->Stage == EAssemblyLineStage::Fault) return FLinearColor(1.0f,.2f,.15f);
        if (Line->Stage == EAssemblyLineStage::Stopped) return FLinearColor(1.0f,.65f,.12f);
        if (Line->Stage == EAssemblyLineStage::Complete) return FLinearColor(.2f,1.0f,.45f);
        if (const auto* Autonomous = Cast<AAutonomousAssemblyLine>(Line); Autonomous && Autonomous->InjectedEvents > Autonomous->RecoveredEvents)
            return FLinearColor(1.0f,.65f,.12f);
        return FLinearColor(.18f,.75f,1.0f);
    }
}

AIntegratedAssemblyLine::AIntegratedAssemblyLine()
{
    PrimaryActorTick.bCanEverTick = true;
    FAssemblyProcessEntry Wheels; Wheels.Id=TEXT("wheels");Wheels.Label=TEXT("휠 장착");
    ProcessRoute.Add(MoveTemp(Wheels));
    FAssemblyProcessEntry Doors;Doors.Id=TEXT("doors");Doors.Label=TEXT("문 장착");
    Doors.Validate=[this](FString& Error){
        const bool Valid=IsValid(WheelCell)&&IsValid(DoorCell)&&IsValid(WheelCell->Assembly)
            &&IsValid(WheelCell->TransportCarriage)&&!DoorStationOffsetCm.ContainsNaN()
            &&VehicleAtWheelStation.IsValid()&&CarriageAtWheelStation.IsValid();
        if(!Valid)Error=TEXT("문 공정의 차체·운반대·준비 위치 연결을 확인하세요.");return Valid;};
    Doors.Enter=[this](){return PrepareDoorStation();};
    ProcessRoute.Add(MoveTemp(Doors));
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("LineRoot"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube"));
    for(int I=0;I<4;++I)
    {
        auto* Post=CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("VehicleLiftSupport%d"),I));
        Post->SetupAttachment(RootComponent);Post->SetMobility(EComponentMobility::Movable);
        if(Cube.Succeeded())Post->SetStaticMesh(Cube.Object);
        Post->SetCollisionProfileName(TEXT("BlockAll"));Post->SetVisibility(false);LiftSupports.Add(Post);
    }
}
bool AIntegratedAssemblyLine::RegisterFollowingProcess(FAssemblyProcessEntry Entry)
{
    if(Stage!=EAssemblyLineStage::Idle||Entry.Id.IsNone()||Entry.Label.IsEmpty()
        ||!Entry.Validate||!Entry.Enter||!Entry.Advance||!Entry.Stop
        ||ProcessRoute.ContainsByPredicate([&](const auto& P){return P.Id==Entry.Id;}))return false;
    ProcessRoute.Add(MoveTemp(Entry));return true;
}
bool AIntegratedAssemblyLine::CanSkipToNextProcess() const
{
    return Stage!=EAssemblyLineStage::Complete&&ProcessRoute.IsValidIndex(ActiveProcessIndex+1);
}
FString AIntegratedAssemblyLine::NextProcessLabel() const
{
    return CanSkipToNextProcess()?ProcessRoute[ActiveProcessIndex+1].Label:FString();
}
FString AIntegratedAssemblyLine::SkippedProcessSummary() const
{
    FString Out;
    for(const auto& P:ProcessRoute)if(SkippedProcesses.Contains(P.Id))
        Out+=P.Label+TEXT(" · 건너뜀 / 미검사\n");
    return Out;
}
bool AIntegratedAssemblyLine::SkipToNextProcess(){return EnterFollowingProcess(true);}
bool AIntegratedAssemblyLine::EnterFollowingProcess(bool Skipping)
{
    if(!ProcessRoute.IsValidIndex(ActiveProcessIndex+1)|| (Skipping&&!CanSkipToNextProcess()))return false;
    auto& Next=ProcessRoute[ActiveProcessIndex+1];FString Error;
    if(!Next.Validate(Error)){if(Skipping)Status=Error;else Fail(Error);return false;}
    // Validate before any mutation. Stop the old executor before moving the product.
    if(!Skipping&&ProcessRoute[ActiveProcessIndex].Stop)ProcessRoute[ActiveProcessIndex].Stop();
    if(Skipping)
    {
        const bool AlreadyDone=ActiveProcessIndex==0&&WheelCell&&WheelCell->Outcome==EWheelOutcome::Pass;
        if(!AlreadyDone)SkippedProcesses.AddUnique(ProcessRoute[ActiveProcessIndex].Id);
        StopLine();
    }
    BeforeProcessEntry(Next.Id);
    ++ActiveProcessIndex;
    if(!Next.Enter()){Fail(Status);return false;}
    bReady=true;
    if(ActiveProcessIndex>=2)Stage=EAssemblyLineStage::Extension;
    AfterProcessEntry();
    Status=Next.Label+TEXT(" · 준비 위치 이동 / 센서 검사 후 실행");return true;
}
bool AIntegratedAssemblyLine::PrepareDoorStation()
{
    auto* Car=WheelCell->Assembly.Get();auto* Carriage=WheelCell->TransportCarriage.Get();
    if(WheelCell->Outcome!=EWheelOutcome::Pass)
    {
        // A demo skip abandons the unfinished operation, including held/falling
        // wheels. Return them to supply; never fabricate a mounted/pass result.
        Car->SetActorTransform(WheelStationPose());Carriage->SetActorTransform(CarriageAtWheelStation);
        const bool Transport=WheelCell->bEnableTransport;WheelCell->bEnableTransport=false;
        const bool Reset=WheelCell->ResetAtStation(WheelStationPose(),CarriageAtWheelStation);
        WheelCell->bEnableTransport=Transport;
        if(!Reset){Status=WheelCell->Status;return false;}
    }
    ExpectedVehicle=VehicleAtWheelStation;ExpectedVehicle.AddToTranslation(DoorStationOffsetCm);
    ExpectedCarriage=CarriageAtWheelStation;ExpectedCarriage.AddToTranslation(DoorStationOffsetCm);
    Car->SetActorTransform(ExpectedVehicle);Carriage->SetActorTransform(ExpectedCarriage);
    if(!VehicleAssemblyMounting::SetRearExtension(*Car,0)){Status=TEXT("문 작업용 허브 자세 복원 실패");return false;}
    UpdateLiftSupports();WheelCell->UpdateFastenedWheels();
    if(!DoorCell->ResetCycle()){Status=DoorCell->Status;return false;}
    DoorSupplyWorld=Car->GetLeftDoor()->GetComponentTransform();
    if(SecondDoorCell)
    {
        if(!SecondDoorCell->ResetCycle()){Status=SecondDoorCell->Status;return false;}
        SecondDoorSupplyWorld=Car->GetRightDoor()->GetComponentTransform();
    }
    Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
    if(SecondDoorCell)Car->GetRightDoor()->SetWorldTransform(SecondDoorSupplyWorld);
    TransferVehicleStart=ExpectedVehicle;TransferCarriageStart=ExpectedCarriage;
    TransferTime=TransferSeconds+DockSeconds;TransferProgress=1;
    Stage=EAssemblyLineStage::Transfer;
    // The regular transfer exit performs supply inspection, replacement, IK and
    // alignment checks. Teleporting does not certify any of these conditions.
    return true;
}
void AIntegratedAssemblyLine::UpdateLiftSupports()
{
    if(!WheelCell||!WheelCell->Assembly||!WheelCell->TransportCarriage)return;
    FVector Center=FVector::ZeroVector;for(int I=0;I<4;++I)Center+=WheelCell->GetMountWorld(I).GetLocation()/4.;
    const double Bottom=WheelCell->TransportCarriage->GetActorLocation().Z+8,Top=Center.Z-15;
    const FQuat Rotation=WheelCell->Assembly->GetActorQuat();
    for(int I=0;I<LiftSupports.Num();++I)
    {
        auto* Post=LiftSupports[I].Get();const bool Visible=WheelStationLiftCm>0&&Top>Bottom;
        Post->SetVisibility(Visible);Post->SetCollisionEnabled(Visible?ECollisionEnabled::QueryAndPhysics:ECollisionEnabled::NoCollision);
        if(!Visible)continue;
        FVector P=Center+Rotation.RotateVector(FVector(I<2?-75:75,I%2?-45:45,0));P.Z=(Top+Bottom)/2;
        Post->SetWorldTransform(FTransform(Rotation,P,FVector(.16,.16,(Top-Bottom)/100.)));
    }
}
void AIntegratedAssemblyLine::Fail(const FString& Reason)
{
    const FString Saved = Reason;
    if(ProcessRoute.IsValidIndex(ActiveProcessIndex)&&ProcessRoute[ActiveProcessIndex].Stop)ProcessRoute[ActiveProcessIndex].Stop();
    if (WheelCell) WheelCell->StopCycle();
    if (DoorCell) DoorCell->StopCycle();
    if(SecondDoorCell)SecondDoorCell->StopCycle();
    Stage = EAssemblyLineStage::Fault;
    Status = Saved;
    bReady = false;
}
ADoorAssemblyCell* AIntegratedAssemblyLine::ActiveDoorCell(const FString& TaskId) const
{
    if(DoorCell && DoorCell->GetOperationTaskId()==TaskId) return DoorCell;
    if(SecondDoorCell && SecondDoorCell->GetOperationTaskId()==TaskId) return SecondDoorCell;
    return nullptr;
}
bool AIntegratedAssemblyLine::DoorSnapshot(const FString& TaskId, FDoorOperationSnapshot& Out) const
{
    if(const auto* Saved=CompletedDoorOperations.Find(TaskId)){Out=*Saved;return true;}
    const auto* D=ActiveDoorCell(TaskId);if(!D)return false;
    Out.TaskId=TaskId;Out.AlignmentMm=D->AlignmentPositionErrorMm;Out.WidthCm=D->MeasuredDoorWidthCm;
    Out.SampleTime=D->LastAlignmentSampleTime;Out.WidthSampleTime=D->DoorSizeMeasuredAt;Out.SimulationTime=D->SimulationTimeSeconds;
    Out.bAlignmentValid=D->bAlignmentSensorValid&&D->bLastAlignmentSampleValid;
    Out.bComplete=D->bDoorFastened;Out.QualitySummary=D->GetQualitySummary();return true;
}
void AIntegratedAssemblyLine::CaptureDoorResult(ADoorAssemblyCell* Cell)
{
    if(!Cell)return;
    FDoorOperationSnapshot Result;
    if(DoorSnapshot(Cell->GetOperationTaskId(),Result))CompletedDoorOperations.Add(Result.TaskId,Result);
}

bool AIntegratedAssemblyLine::StartDoorOperation(ADoorAssemblyCell* Cell)
{
    if(!Cell)return false;
    if(const auto* Binding=DoorRequests.Find(Cell->GetOperationTaskId()))Cell->RequestBinding=*Binding;
    else Cell->RequestBinding=FAssemblyOperationRequest();
    return Cell->StartCycle();
}
void AIntegratedAssemblyLine::ConfigureFrontDoorSize(double Scale)
{
    FrontDoorSizeScales[0]=Scale;
    if(DoorCell&&!DoorCell->TargetModule)DoorCell->DoorSizeScale=Scale;
}
bool AIntegratedAssemblyLine::PrepareDoor(ADoorAssemblyCell* Cell, int32 Index)
{
    if (!Cell || !bFourDoorMode || !DoorModules.IsValidIndex(Index) || !IsValid(DoorModules[Index])) return false;
    Cell->DoorSizeScale=1.0;
    const bool bRight = Index >= 2;
    Cell->TargetModule=(Index % 2) ? DoorModules[Index].Get() : nullptr;
    Cell->Robot=bRight ? RightDoorRobot.Get() : LeftDoorRobot.Get();
    const auto* Car=WheelCell->Assembly.Get();
    const FTransform BodyWorld=Car->GetBodyFrame()->GetComponentTransform();
    const FTransform OldMount=Car->GetDoorPoseController(ECamryDoorSide::Left)->GetReferencePoseInBody()*BodyWorld;
    const FTransform NewMount=DoorModules[Index]->ReferencePoseInVehicle*WheelCell->Assembly->GetActorTransform();
    FTransform GraspWorld=BaselineTCPInLeftDoor*OldMount;
    GraspWorld.AddToTranslation(NewMount.GetLocation()-OldMount.GetLocation());
    if (bRight && SecondDoorCell && SecondDoorCell->Robot)
    {
        // Transfer the front calibration in the nominal mounting frame. A
        // completed front door can sag on its support; that is not calibration.
        // Accumulate visible mesh bounds in module coordinates before taking
        // their center. Inverting a world AABB depends on the current panel yaw
        // and can include non-physical proxies/pivots in the grasp offset.
        auto MountedCenter=[&](int32 DoorIndex,FVector& Center)
        {
            auto* Module=DoorModules[DoorIndex].Get();
            if(!IsValid(Module)||!Module->GetRootComponent())return false;
            TArray<USceneComponent*> Parts;
            Module->GetRootComponent()->GetChildrenComponents(true,Parts);
            Parts.Add(Module->GetRootComponent());
            FBox LocalBounds(ForceInit);
            for(auto* Part:Parts)if(auto* Mesh=Cast<UStaticMeshComponent>(Part))
                if(Mesh->GetStaticMesh()&&Mesh->IsVisible()&&!Mesh->bHiddenInGame)
                    LocalBounds+=Mesh->GetStaticMesh()->GetBoundingBox().TransformBy(
                        Mesh->GetComponentTransform().ToMatrixWithScale()*Module->GetActorTransform().ToInverseMatrixWithScale());
            if(!LocalBounds.IsValid)return false;
            Center=(Module->ReferencePoseInVehicle*Car->GetActorTransform()).TransformPosition(LocalBounds.GetCenter());
            return true;
        };
        FVector RearCenter,FrontCenter;
        if(!MountedCenter(Index,RearCenter)||!MountedCenter(2,FrontCenter))
        {Cell->Status=Status=TEXT("도어 파지 보정 실패 · 기준 메시 형상 없음");return false;}
        const FTransform FrontMount=Car->GetDoorPoseController(ECamryDoorSide::Right)->GetReferencePoseInBody()*BodyWorld;
        GraspWorld=BaselineTCPInRightDoor*FrontMount;
        GraspWorld.AddToTranslation(RearCenter-FrontCenter);
        Cell->Robot=SecondDoorCell->Robot;
    }
    else if (bRight) GraspWorld.SetRotation((FQuat(FVector::UpVector,PI)*GraspWorld.GetRotation()).GetNormalized());
    if(DoorTaskRobots.Num()==4) Cell->Robot=DoorTaskRobots[Index];
    if(!IsValid(Cell->Robot)){Status=TEXT("도어 작업 로봇 미할당");return false;}
    Cell->OperationTaskId=DoorModules[Index]->ModuleId.ToString().ToLower();
    Cell->TCPInDoorCm=GraspWorld.GetRelativeTransform(NewMount);
    Cell->PickupOffsetBodyCm=FVector(0,bRight ? 100 : -100,20)
        +WheelCell->Assembly->GetBodyFrame()->GetComponentTransform().InverseTransformVectorNoScale(DoorSupplyOffsetsWorldCm.FindRef(Index));
    if(Index % 2 == 0)
    {
        Cell->TCPInDoorCm=bRight ? BaselineTCPInRightDoor : BaselineTCPInLeftDoor;
        Cell->PickupOffsetBodyCm=FrontDoorPickupOffsets[bRight?1:0];
        Cell->DoorSizeScale=FrontDoorSizeScales[bRight?1:0];
    }
    Cell->ApproachOffsetBodyCm=FVector(0,bRight ? 25 : -25,0);
    Cell->TransferOffsetBodyCm=FVector(0,bRight ? 50 : -50,25);
    if(!Cell->PrepareNextOperation()){Status=Cell->Status;return false;}
    return true;
}
bool AIntegratedAssemblyLine::ResetLine()
{
    bReady = false;
    if(ProcessRoute.IsValidIndex(ActiveProcessIndex)&&ProcessRoute[ActiveProcessIndex].Stop)ProcessRoute[ActiveProcessIndex].Stop();
    ActiveProcessIndex=0;SkippedProcesses.Reset();
    if (!IsValid(WheelCell) || !IsValid(DoorCell) || !IsValid(WheelCell->Assembly)
        || DoorCell->Assembly != WheelCell->Assembly || !IsValid(WheelCell->TransportCarriage)
        || !WheelCell->bEnableTransport || !FMath::IsFinite(TransferSeconds) || TransferSeconds <= 0
        || !FMath::IsFinite(DockSeconds) || DockSeconds < 0 || DoorStationOffsetCm.ContainsNaN()
        || !FMath::IsFinite(WheelStationLiftCm)||WheelStationLiftCm<0||WheelStationLiftCm>80)
    { Fail(TEXT("Invalid line references or transport configuration.")); return false; }
    WheelCell->bExternallyDriven = DoorCell->bExternallyDriven = true;
    WheelCell->StopCycle(); DoorCell->StopCycle(); if(SecondDoorCell)SecondDoorCell->StopCycle();
    MountedDoorCount=0;CompletedDoorOperations.Reset();
    DoorCell->OperationTaskId=TEXT("door_fl");
    if (!DoorCell->TargetModule || !IsValid(LeftDoorRobot)) LeftDoorRobot=DoorCell->Robot;
    if (!DoorCell->TargetModule)
    {BaselineTCPInLeftDoor=DoorCell->TCPInDoorCm;FrontDoorPickupOffsets[0]=DoorCell->PickupOffsetBodyCm;FrontDoorSizeScales[0]=DoorCell->DoorSizeScale;}
    if(SecondDoorCell && !SecondDoorCell->TargetModule)
    {BaselineTCPInRightDoor=SecondDoorCell->TCPInDoorCm;FrontDoorPickupOffsets[1]=SecondDoorCell->PickupOffsetBodyCm;FrontDoorSizeScales[1]=SecondDoorCell->DoorSizeScale;}
    DoorModules.Reset();
    const FName DoorIds[]={TEXT("Door_FL"),TEXT("Door_RL"),TEXT("Door_FR"),TEXT("Door_RR")};
    for (const FName Id:DoorIds)
    {
        AVehicleProcessModule* Found=nullptr;
        for (TActorIterator<AVehicleProcessModule> It(GetWorld());It;++It)
            if (It->ModuleId==Id && It->VehicleFrame==WheelCell->Assembly) { Found=*It; break; }
        DoorModules.Add(Found);
    }
    bFourDoorMode=IsValid(SecondDoorCell) && IsValid(RightDoorRobot) && IsValid(LeftDoorRobot);
    for (const auto& Module:DoorModules) bFourDoorMode &= IsValid(Module) && Module->bHasReferencePose;
    auto* Car = WheelCell->Assembly.Get();
    if(!VehicleAssemblyMounting::SetRearExtension(*Car,0)) {Fail(TEXT("후방 허브 기준 자세 복원 실패"));return false;}
    FTransform DoorStation = VehicleAtWheelStation;
    DoorStation.AddToTranslation(DoorStationOffsetCm);
    Car->SetActorTransform(DoorStation);
    if (bFourDoorMode)
    {
        if(DoorCell->TargetModule) DoorCell->TCPInDoorCm=BaselineTCPInLeftDoor;
        DoorCell->TargetModule=nullptr;
        DoorCell->PickupOffsetBodyCm=FrontDoorPickupOffsets[0];
        DoorCell->DoorSizeScale=FrontDoorSizeScales[0];
        SecondDoorCell->TargetModule=nullptr;
        SecondDoorCell->TCPInDoorCm=BaselineTCPInRightDoor;
        SecondDoorCell->PickupOffsetBodyCm=FrontDoorPickupOffsets[1];
        SecondDoorCell->DoorSizeScale=FrontDoorSizeScales[1];
        SecondDoorCell->OperationTaskId=TEXT("door_fr");
        SecondDoorCell->Robot=RightDoorRobot;
        DoorCell->Robot=LeftDoorRobot;
        DoorCell->ApproachOffsetBodyCm=FVector(0,-25,0);
        DoorCell->TransferOffsetBodyCm=FVector(0,-50,25);
        for(int32 Index : {1,3})
        {
            auto* Module=DoorModules[Index].Get();
            Module->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
            auto Supply=Module->ReferencePoseInVehicle*Car->GetActorTransform();
            Supply.AddToTranslation(Car->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(FVector(0,Index==3?100:-100,20)));
            Supply.AddToTranslation(DoorSupplyOffsetsWorldCm.FindRef(Index));
            Module->SetActorTransform(Supply);
        }
    }
    if (!DoorCell->ResetCycle()) { Fail(DoorCell->Status); return false; }
    DoorSupplyWorld = Car->GetLeftDoor()->GetComponentTransform();
    if(SecondDoorCell){SecondDoorCell->bExternallyDriven=true;
        if(!SecondDoorCell->ResetCycle()){Fail(SecondDoorCell->Status);return false;}
        SecondDoorSupplyWorld=Car->GetRightDoor()->GetComponentTransform();}
    Car->SetActorTransform(WheelStationPose());
    if(!VehicleAssemblyMounting::SetRearExtension(*Car,FMath::Min(WheelStationLiftCm,VehicleAssemblyMounting::RearExtensionCm)))
    {Fail(TEXT("리프트 작업용 후방 허브 자세 설정 실패"));return false;}
    WheelCell->TransportCarriage->SetActorTransform(CarriageAtWheelStation);
    if (!WheelCell->ResetAtStation(WheelStationPose(),CarriageAtWheelStation)) { Fail(WheelCell->Status); return false; }
    UpdateLiftSupports();
    Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
    if(SecondDoorCell)Car->GetRightDoor()->SetWorldTransform(SecondDoorSupplyWorld);
    ElapsedSeconds = TransferTime = TransferProgress = 0;
    Stage = EAssemblyLineStage::Idle;
    Status = bFourDoorMode ? TEXT("대기 · 차체 반입 → 타이어 4개 → 구역 이송 → 문 4개") : TEXT("대기 · 차체 반입 → 타이어 4개 → 다음 구역 이송 → 왼쪽 앞문");
    bReady = true;
    return true;
}
bool AIntegratedAssemblyLine::StartLine()
{
    if (!bReady || Stage != EAssemblyLineStage::Idle) return false;
    if (!WheelCell->StartCycle()) { Fail(WheelCell->Status); return false; }
    Stage = EAssemblyLineStage::Wheels;
    return true;
}
void AIntegratedAssemblyLine::StopLine()
{
    if (Stage == EAssemblyLineStage::Idle || Stage == EAssemblyLineStage::Complete || Stage == EAssemblyLineStage::Fault) return;
    if(ProcessRoute.IsValidIndex(ActiveProcessIndex)&&ProcessRoute[ActiveProcessIndex].Stop)ProcessRoute[ActiveProcessIndex].Stop();
    WheelCell->StopCycle(); DoorCell->StopCycle();if(SecondDoorCell)SecondDoorCell->StopCycle();
    Stage = EAssemblyLineStage::Stopped;
    Status = TEXT("정지 · 현재 자세 유지, R로 초기화 후 다시 시작");
    bReady = false;
}
bool AIntegratedAssemblyLine::AdvanceLine(double Seconds)
{
    if (!FMath::IsFinite(Seconds) || Seconds < 0 || Seconds > 1) { StopLine(); return false; }
    if(DoorCell)DoorCell->AdvanceRetiredParts(Seconds);if(SecondDoorCell)SecondDoorCell->AdvanceRetiredParts(Seconds);
    if(Stage==EAssemblyLineStage::Complete){if(DoorCell)DoorCell->AdvanceSimulation(Seconds);if(SecondDoorCell)SecondDoorCell->AdvanceSimulation(Seconds);}
    if((Stage==EAssemblyLineStage::Fault || Stage==EAssemblyLineStage::Stopped) && WheelCell)
        WheelCell->AdvanceSimulation(Seconds);
    if(Stage==EAssemblyLineStage::Fault||Stage==EAssemblyLineStage::Stopped)
    {if(DoorCell)DoorCell->AdvanceSimulation(Seconds);if(SecondDoorCell)SecondDoorCell->AdvanceSimulation(Seconds);}
    if (Stage == EAssemblyLineStage::Fault) return false;
    if (Stage == EAssemblyLineStage::Idle || Stage == EAssemblyLineStage::Stopped || Stage == EAssemblyLineStage::Complete) return true;
    if (!IsValid(WheelCell) || !IsValid(DoorCell) || !IsValid(WheelCell->Assembly) || !IsValid(WheelCell->TransportCarriage))
    { Fail(TEXT("Lost line reference.")); return false; }
    ElapsedSeconds += Seconds;
    auto* Car = WheelCell->Assembly.Get();
    auto* Carriage = WheelCell->TransportCarriage.Get();
    if(Stage==EAssemblyLineStage::Extension)
    {
        if(DoorCell)DoorCell->AdvanceSimulation(Seconds);if(SecondDoorCell)SecondDoorCell->AdvanceSimulation(Seconds);
        const auto Progress=ProcessRoute[ActiveProcessIndex].Advance(Seconds);
        if(Progress==EAssemblyProcessProgress::Fault){Fail(Status);return false;}
        if(Progress==EAssemblyProcessProgress::Complete)
        {
            if(ProcessRoute.IsValidIndex(ActiveProcessIndex+1))return EnterFollowingProcess(false);
            Stage=EAssemblyLineStage::Complete;Status=TEXT("등록 공정 완료");
        }
        return true;
    }
    if (Stage == EAssemblyLineStage::Wheels)
    {
        const bool Ok = WheelCell->AdvanceSimulation(Seconds);
        Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
    if(SecondDoorCell)Car->GetRightDoor()->SetWorldTransform(SecondDoorSupplyWorld);
        Status = WheelCell->bTransporting ? TEXT("1/4 차체 반입") : TEXT("2/4 타이어 장착 · ") + WheelCell->Status;
        if (!Ok || WheelCell->Outcome == EWheelOutcome::Blocked || WheelCell->Outcome == EWheelOutcome::QualityFail || WheelCell->Outcome == EWheelOutcome::Stopped)
        { Fail(WheelCell->Status); return false; }
        if (WheelCell->Outcome == EWheelOutcome::Pass)
        {
            ExpectedVehicle = Car->GetActorTransform();
            ExpectedCarriage = Carriage->GetActorTransform();
            TransferVehicleStart=ExpectedVehicle;TransferCarriageStart=ExpectedCarriage;
            Stage = EAssemblyLineStage::Transfer;
        }
    }
    else if (Stage == EAssemblyLineStage::Transfer)
    {
        if (!Car->GetActorTransform().Equals(ExpectedVehicle,1.e-3) || !Carriage->GetActorTransform().Equals(ExpectedCarriage,1.e-3))
        { Fail(TEXT("Vehicle or carriage moved externally between stations.")); return false; }
        TransferTime += Seconds;
        const double T = FMath::Clamp(TransferTime / TransferSeconds,0.,1.);
        TransferProgress = T*T*T*(10 + T*(-15 + 6*T));
        ExpectedVehicle = TransferVehicleStart;
        ExpectedVehicle.SetLocation(FMath::Lerp(TransferVehicleStart.GetLocation(),VehicleAtWheelStation.GetLocation()+DoorStationOffsetCm,TransferProgress));
        ExpectedCarriage = TransferCarriageStart;
        ExpectedCarriage.SetLocation(FMath::Lerp(TransferCarriageStart.GetLocation(),CarriageAtWheelStation.GetLocation()+DoorStationOffsetCm,TransferProgress));
        Car->SetActorTransform(ExpectedVehicle);
        Carriage->SetActorTransform(ExpectedCarriage);
        if(!VehicleAssemblyMounting::SetRearExtension(*Car,FMath::Min(WheelStationLiftCm,VehicleAssemblyMounting::RearExtensionCm)*(1-TransferProgress)))
        {Fail(TEXT("리프트 하강 중 후방 허브 자세 갱신 실패"));return false;}
        UpdateLiftSupports();
        WheelCell->UpdateFastenedWheels();
        Car->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
    if(SecondDoorCell)Car->GetRightDoor()->SetWorldTransform(SecondDoorSupplyWorld);
        Status = T < 1 ? TEXT("3/4 타이어 장착 차체를 문 작업 구역으로 이송") : TEXT("문 작업 구역 도킹 확인");
        if (TransferTime + 1.e-9 >= TransferSeconds + DockSeconds)
        {
            // Inspect supplied front panels before either arm enters the door
            // work area: an oversized front panel can obstruct a rear pickup.
            if(bFourDoorMode)
            {
                bool Pending=false;
                for(auto* Cell:{DoorCell.Get(),SecondDoorCell.Get()})
                {
                    if(Cell->State==EDoorAssemblyState::WaitReplacement)
                    {if(!Cell->AdvanceSimulation(Seconds)){Fail(Cell->Status);return false;}}
                    else if(!Cell->CheckDoorSupply(true)){Fail(Cell->Status);return false;}
                    Pending|=Cell->State==EDoorAssemblyState::WaitReplacement;
                }
                if(Pending)return true;
                FrontDoorSizeScales[0]=DoorCell->DoorSizeScale;
                FrontDoorSizeScales[1]=SecondDoorCell->DoorSizeScale;
            }
            // The rear panel lip passes behind the front panel. Fit both rears
            // first; then both fronts admit a fixed-attitude straight insertion.
            if(bFourDoorMode && (!PrepareDoor(DoorCell,1)||!PrepareDoor(SecondDoorCell,3)))
            {Fail(Status);return false;}
            if (!StartDoorOperation(DoorCell)) { Fail(DoorCell->Status); return false; }
            if(SecondDoorCell&&!StartDoorOperation(SecondDoorCell)){Fail(SecondDoorCell->Status);return false;}
            ActiveProcessIndex=1;Stage = EAssemblyLineStage::Door;
        }
    }
    else if (Stage == EAssemblyLineStage::Door)
    {
        if (!Carriage->GetActorTransform().Equals(ExpectedCarriage,1.e-3))
        { Fail(TEXT("Carriage moved during door assembly.")); return false; }
        if (!DoorCell->AdvanceSimulation(Seconds) || DoorCell->Outcome == EDoorCellOutcome::QualityFail || DoorCell->Outcome == EDoorCellOutcome::Stopped)
        { Fail(DoorCell->Status); return false; }
        if(SecondDoorCell&&(!SecondDoorCell->AdvanceSimulation(Seconds)||SecondDoorCell->Outcome==EDoorCellOutcome::QualityFail||SecondDoorCell->Outcome==EDoorCellOutcome::Stopped))
        {Fail(SecondDoorCell->Status);return false;}
        Status = (bFourDoorMode&&MountedDoorCount==0?TEXT("4/4 뒷문 장착 · "):TEXT("4/4 앞문 장착 · ")) + DoorCell->GetStateLabel();
        if (DoorCell->State == EDoorAssemblyState::Complete&&(!SecondDoorCell||SecondDoorCell->State==EDoorAssemblyState::Complete))
        {
            if (DoorCell->Outcome != EDoorCellOutcome::Pass||(SecondDoorCell&&SecondDoorCell->Outcome!=EDoorCellOutcome::Pass)) { Fail(DoorCell->Status); return false; }
            CaptureDoorResult(DoorCell);
            CaptureDoorResult(SecondDoorCell);
            if(bFourDoorMode)
            {
                MountedDoorCount+=2;
                if(MountedDoorCount<4)
                {
                    if(!PrepareDoor(DoorCell,0)||!PrepareDoor(SecondDoorCell,2))
                    {Fail(Status);return false;}
                    if(!StartDoorOperation(DoorCell)){Fail(DoorCell->Status);return false;}
                    if(!StartDoorOperation(SecondDoorCell)){Fail(SecondDoorCell->Status);return false;}
                    return true;
                }
            }
            if(ProcessRoute.IsValidIndex(ActiveProcessIndex+1))return EnterFollowingProcess(false);
            Stage = EAssemblyLineStage::Complete;
            Status = !SkippedProcesses.IsEmpty()?TEXT("선택 공정 완료 · 이전 공정 건너뜀 / 미검사"):bFourDoorMode?TEXT("완료 · 휠 4개와 문 4개 장착"):SecondDoorCell?TEXT("완료 · 휠 4개와 앞문 2개 시험 장착·검사 완료"):TEXT("완료 · 타이어 4개와 왼쪽 앞문 장착·검사 합격");
        }
    }
    return Stage != EAssemblyLineStage::Fault;
}
FString AIntegratedAssemblyLine::GetSummary() const
{
    return FString::Printf(TEXT("통합 자동차 조립\n%s\n경과 %.1f초 | 타이어 %d/4 | 문 %d/%d\n\nEnter 시작 / Space 정지 / R 초기화\n우클릭 회전 / 휠 확대·축소 / Home 전체 보기"),
        *Status, ElapsedSeconds, WheelCell ? WheelCell->MountedCount : 0,
        MountedDoorCount+(MountedDoorCount==0 && DoorCell && DoorCell->bDoorFastened ? 1 : 0), bFourDoorMode ? 4 : 1);
}
void AIntegratedAssemblyLine::BeginPlay() { Super::BeginPlay(); ResetLine(); }
void AIntegratedAssemblyLine::Tick(float Seconds) { Super::Tick(Seconds); AdvanceLine(Seconds); }

AIntegratedAssemblyController::AIntegratedAssemblyController() { bAutoManageActiveCameraTarget=false; bShowMouseCursor=true; }
void AIntegratedAssemblyController::BeginPlay()
{
    Super::BeginPlay();
    if (!IsLocalController()) return;
    for (TActorIterator<AIntegratedAssemblyLine> It(GetWorld()); It; ++It) { Line=*It; break; }
    AssemblyCamera=GetWorld()->SpawnActor<ACameraActor>();
    ResetCamera(); SetViewTarget(AssemblyCamera);
    Panel=SNew(SBox).HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(18).WidthOverride(650).MaxDesiredHeight(980)
    [SNew(SScrollBox)
        +SScrollBox::Slot()
        [SNew(SBorder).Padding(16).BorderBackgroundColor(FLinearColor(.012f,.022f,.035f,.96f))
        [SNew(SVerticalBox)
            +SVerticalBox::Slot().AutoHeight()
            [SNew(STextBlock).Text(FText::FromString(TEXT("DIGITAL TWIN · ASSEMBLY CONTROL")))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold",14)).ColorAndOpacity(FLinearColor(.35f,.72f,1.0f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,2,0,10)
            [SNew(STextBlock).Text(FText::FromString(TEXT("자동차 조립 공정 모니터")))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold",28)).ColorAndOpacity(FLinearColor::White)]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,10)
            [SNew(SBorder).Padding(10).BorderBackgroundColor_Lambda([this](){ return StateColor(Line)*FLinearColor(1,1,1,.22f); })
                [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(FString::Printf(TEXT("●  %s"),*StageLabel(Line ? Line->Stage : EAssemblyLineStage::Fault))); })
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold",22)).ColorAndOpacity_Lambda([this](){ return StateColor(Line); })]]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,12)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(ProcessRail(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Bold",15)).ColorAndOpacity(FLinearColor(.72f,.82f,.9f))]

            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,4)
            [SNew(STextBlock).Text(FText::FromString(TEXT("지금 하는 작업"))).Font(FCoreStyle::GetDefaultFontStyle("Bold",17)).ColorAndOpacity(FLinearColor(.4f,.82f,1.0f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,12)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(CurrentOperation(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",18)).ColorAndOpacity(FLinearColor::White)]

            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,4)
            [SNew(STextBlock).Text(FText::FromString(TEXT("이상 감지"))).Font(FCoreStyle::GetDefaultFontStyle("Bold",17)).ColorAndOpacity(FLinearColor(1.0f,.68f,.25f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,10)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(IncidentSummary(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",17)).ColorAndOpacity_Lambda([this](){ return StateColor(Line); })]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,4)
            [SNew(STextBlock).Text(FText::FromString(TEXT("시스템 판단 · 대응"))).Font(FCoreStyle::GetDefaultFontStyle("Bold",17)).ColorAndOpacity(FLinearColor(.45f,1.0f,.65f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,12)
            [SNew(STextBlock).Text_Lambda([this](){ return FText::FromString(ResponseSummary(Line)); })
                .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",17)).ColorAndOpacity(FLinearColor(.83f,.92f,.86f))]

            +SVerticalBox::Slot().AutoHeight().Padding(0,0,0,8)
            [SNew(SBorder).Padding(10).BorderBackgroundColor(FLinearColor(.04f,.065f,.085f,.95f))
                [SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(Line ? Line->GetSummary() : TEXT("조립 라인 연결 대기")); })
                    .AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular",16)).ColorAndOpacity(FLinearColor(.82f,.88f,.92f))]]
            +SVerticalBox::Slot().AutoHeight().Padding(0,8,0,0)[SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(GetCameraLabel()); }).AutoWrapText(true).ColorAndOpacity(FLinearColor(.68f,.72f,.76f))]
            +SVerticalBox::Slot().AutoHeight().Padding(0,4,0,0)[SNew(SHorizontalBox)
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("전체"))).OnClicked_Lambda([this](){ResetCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("도킹 센서"))).OnClicked_Lambda([this](){ArrivalCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("왼팔"))).OnClicked_Lambda([this](){LeftRobotCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("오른팔"))).OnClicked_Lambda([this](){RightRobotCamera();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("문 팔"))).OnClicked_Lambda([this](){DoorRobotCamera();return FReply::Handled();})]
            ]
            +SVerticalBox::Slot().AutoHeight().Padding(0,10,0,0)[SNew(SHorizontalBox).Visibility(Line && Line->GetClass()==ASensorAssemblyLine::StaticClass()?EVisibility::Visible:EVisibility::Collapsed)
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("정상"))).OnClicked_Lambda([this](){DemoNormal();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("이송 멈춤"))).OnClicked_Lambda([this](){DemoJam();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("위치 틀어짐"))).OnClicked_Lambda([this](){DemoMisDock();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("센서 끊김"))).OnClicked_Lambda([this](){DemoDropout();return FReply::Handled();})]
            ]
			+SVerticalBox::Slot().AutoHeight().Padding(0,4,0,0)[SNew(SHorizontalBox).Visibility(Line && Line->GetClass()==ASensorAssemblyLine::StaticClass()?EVisibility::Visible:EVisibility::Collapsed)
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("타이어 없음"))).OnClicked_Lambda([this](){DemoWheelMissing();return FReply::Handled();})]
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("타이어 위치"))).OnClicked_Lambda([this](){DemoWheelDisplaced();return FReply::Handled();})]
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("파지 실패"))).OnClicked_Lambda([this](){DemoGripFailure();return FReply::Handled();})]
				+SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("타이어 센서 끊김"))).OnClicked_Lambda([this](){DemoWheelPoseDropout();return FReply::Handled();})]
			]
            +SVerticalBox::Slot().AutoHeight().Padding(0,12,0,0)[SNew(SHorizontalBox)
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("시작"))).OnClicked_Lambda([this](){StartAssembly();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("정지"))).OnClicked_Lambda([this](){StopAssembly();return FReply::Handled();})]
                +SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false).Text(FText::FromString(TEXT("초기화"))).OnClicked_Lambda([this](){ResetAssembly();return FReply::Handled();})]
            ]
        ]]
    ];
    if (GEngine && GEngine->GameViewport) GEngine->GameViewport->AddViewportWidgetContent(Panel.ToSharedRef(),10);
    FInputModeGameAndUI Mode; Mode.SetHideCursorDuringCapture(false); SetInputMode(Mode);
}
void AIntegratedAssemblyController::EndPlay(const EEndPlayReason::Type Reason)
{
    if (Panel.IsValid() && GEngine && GEngine->GameViewport) GEngine->GameViewport->RemoveViewportWidgetContent(Panel.ToSharedRef());
    Panel.Reset(); if (AssemblyCamera) AssemblyCamera->Destroy(); Super::EndPlay(Reason);
}
void AIntegratedAssemblyController::SetupInputComponent()
{
    Super::SetupInputComponent();
    InputComponent->BindKey(EKeys::Enter,IE_Pressed,this,&AIntegratedAssemblyController::StartAssembly);
    InputComponent->BindKey(EKeys::One,IE_Pressed,this,&AIntegratedAssemblyController::DemoNormal);
    InputComponent->BindKey(EKeys::Two,IE_Pressed,this,&AIntegratedAssemblyController::DemoJam);
    InputComponent->BindKey(EKeys::Three,IE_Pressed,this,&AIntegratedAssemblyController::DemoMisDock);
    InputComponent->BindKey(EKeys::Four,IE_Pressed,this,&AIntegratedAssemblyController::DemoDropout);
	InputComponent->BindKey(EKeys::Five,IE_Pressed,this,&AIntegratedAssemblyController::DemoWheelMissing);
	InputComponent->BindKey(EKeys::Six,IE_Pressed,this,&AIntegratedAssemblyController::DemoWheelDisplaced);
	InputComponent->BindKey(EKeys::Seven,IE_Pressed,this,&AIntegratedAssemblyController::DemoGripFailure);
	InputComponent->BindKey(EKeys::Eight,IE_Pressed,this,&AIntegratedAssemblyController::DemoWheelPoseDropout);
    InputComponent->BindKey(EKeys::SpaceBar,IE_Pressed,this,&AIntegratedAssemblyController::StopAssembly);
    InputComponent->BindKey(EKeys::R,IE_Pressed,this,&AIntegratedAssemblyController::ResetAssembly);
    InputComponent->BindKey(EKeys::Home,IE_Pressed,this,&AIntegratedAssemblyController::ResetCamera);
    InputComponent->BindKey(EKeys::F1,IE_Pressed,this,&AIntegratedAssemblyController::ResetCamera);
    InputComponent->BindKey(EKeys::F2,IE_Pressed,this,&AIntegratedAssemblyController::ArrivalCamera);
    InputComponent->BindKey(EKeys::F3,IE_Pressed,this,&AIntegratedAssemblyController::LeftRobotCamera);
    InputComponent->BindKey(EKeys::F4,IE_Pressed,this,&AIntegratedAssemblyController::RightRobotCamera);
    InputComponent->BindKey(EKeys::F5,IE_Pressed,this,&AIntegratedAssemblyController::DoorRobotCamera);
    InputComponent->BindKey(EKeys::MouseScrollUp,IE_Pressed,this,&AIntegratedAssemblyController::ZoomIn);
    InputComponent->BindKey(EKeys::MouseScrollDown,IE_Pressed,this,&AIntegratedAssemblyController::ZoomOut);
}
void AIntegratedAssemblyController::PlayerTick(float Seconds)
{
    Super::PlayerTick(Seconds);
    if (!IsLocalController() || !AssemblyCamera) return;
    const double Forward=(IsInputKeyDown(EKeys::W)?1.:0.)-(IsInputKeyDown(EKeys::S)?1.:0.);
    const double Right=(IsInputKeyDown(EKeys::D)?1.:0.)-(IsInputKeyDown(EKeys::A)?1.:0.);
    const double Up=(IsInputKeyDown(EKeys::E)?1.:0.)-(IsInputKeyDown(EKeys::Q)?1.:0.);
    if (Forward!=0 || Right!=0 || Up!=0)
    {
        CameraMode=ECameraMode::Free;
        const FRotator View=(Focus-AssemblyCamera->GetActorLocation()).Rotation();
        FVector FlatForward=View.Vector(); FlatForward.Z=0; FlatForward.Normalize();
        FVector FlatRight=View.RotateVector(FVector::RightVector); FlatRight.Z=0; FlatRight.Normalize();
        const double Speed=(IsInputKeyDown(EKeys::LeftShift)||IsInputKeyDown(EKeys::RightShift)?900.:350.)*Seconds;
        Focus+=(FlatForward*Forward+FlatRight*Right+FVector::UpVector*Up).GetClampedToMaxSize(1.)*Speed;
        UpdateCamera();
    }
    float X=0,Y=0;
    if (IsInputKeyDown(EKeys::RightMouseButton) && GetMousePosition(X,Y))
    {
        const FVector2D P(X,Y);
        if (bDragging) { CameraMode=ECameraMode::Free; const FVector2D D=P-LastMouse; Orbit.Yaw-=D.X*.25; Orbit.Pitch=FMath::Clamp(Orbit.Pitch+D.Y*.25,4.,80.); UpdateCamera(); }
        LastMouse=P; bDragging=true;
    }
    else bDragging=false;
    if (CameraMode!=ECameraMode::Free) UpdateCamera();
}
void AIntegratedAssemblyController::StartAssembly() { if(Line) Line->StartLine(); }
void AIntegratedAssemblyController::DemoNormal() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::Normal); }
void AIntegratedAssemblyController::DemoJam() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::Jam); }
void AIntegratedAssemblyController::DemoMisDock() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::MisDock); }
void AIntegratedAssemblyController::DemoDropout() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::PositionDropout); }
void AIntegratedAssemblyController::DemoWheelMissing() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::WheelMissing); }
void AIntegratedAssemblyController::DemoWheelDisplaced() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::WheelDisplaced); }
void AIntegratedAssemblyController::DemoGripFailure() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::GripFailure); }
void AIntegratedAssemblyController::DemoWheelPoseDropout() { if(auto* S=Cast<ASensorAssemblyLine>(Line)) S->SelectDemo(EArrivalDemo::WheelPoseDropout); }
void AIntegratedAssemblyController::StopAssembly() { if(Line) Line->StopLine(); }
void AIntegratedAssemblyController::ResetAssembly() { if(Line) Line->ResetLine(); }
void AIntegratedAssemblyController::ResetCamera() { CameraMode=ECameraMode::Free; Focus=FVector(200,0,70); Orbit=FVector(550,-1600,1000).Rotation(); Distance=2000; UpdateCamera(); }
void AIntegratedAssemblyController::ArrivalCamera()
{
    if (!Line) return;
    CameraMode=ECameraMode::Arrival;
    Focus=Line->VehicleAtWheelStation.GetLocation()+FVector(-170,0,80);
    Orbit=FVector(-330,-480,250).Rotation(); Distance=640; UpdateCamera();
}
void AIntegratedAssemblyController::LeftRobotCamera() { CameraMode=ECameraMode::LeftRobot; UpdateCamera(); }
void AIntegratedAssemblyController::RightRobotCamera() { CameraMode=ECameraMode::RightRobot; UpdateCamera(); }
void AIntegratedAssemblyController::DoorRobotCamera() { CameraMode=ECameraMode::DoorRobot; UpdateCamera(); }
FString AIntegratedAssemblyController::GetCameraLabel() const
{
    const TCHAR* Names[]={TEXT("전체·자유 시점"),TEXT("도킹 센서 관찰"),TEXT("왼쪽 타이어 로봇 추적"),TEXT("오른쪽 타이어 로봇 추적"),TEXT("문 로봇 추적")};
    return FString::Printf(TEXT("시점: %s | F1~F5 전환 · WASD 이동 · Q/E 높이 · Shift 빠르게"),Names[int32(CameraMode)]);
}
void AIntegratedAssemblyController::FollowRobot(ARobotArm* Robot,double Side)
{
    if (!IsValid(Robot) || !AssemblyCamera) return;
    const FVector TCP=Robot->GetTCPWorld().GetLocation();
    Focus=TCP*.7+Robot->GetActorLocation()*.3+FVector(0,0,20);
    const FVector P=Focus+FVector(-220,Side*300,180);
    const FVector Offset=P-Focus;
    Orbit=Offset.Rotation(); Distance=Offset.Size();
    AssemblyCamera->SetActorLocationAndRotation(P,(Focus-P).Rotation());
}
void AIntegratedAssemblyController::UpdateCamera()
{
    if (!AssemblyCamera) return;
    if (Line && CameraMode==ECameraMode::LeftRobot && Line->WheelCell) { FollowRobot(Line->WheelCell->LeftRobot,-1); return; }
    if (Line && CameraMode==ECameraMode::RightRobot && Line->WheelCell) { FollowRobot(Line->WheelCell->RightRobot,1); return; }
    if (Line && CameraMode==ECameraMode::DoorRobot && Line->DoorCell) { FollowRobot(Line->DoorCell->Robot,-1); return; }
    const FVector P=Focus+Orbit.Vector()*Distance;
    AssemblyCamera->SetActorLocationAndRotation(P,(Focus-P).Rotation());
}
void AIntegratedAssemblyController::ZoomIn() { CameraMode=ECameraMode::Free; Distance=FMath::Max(150.,Distance*.9); UpdateCamera(); }
void AIntegratedAssemblyController::ZoomOut() { CameraMode=ECameraMode::Free; Distance=FMath::Min(5000.,Distance/.9); UpdateCamera(); }
AIntegratedAssemblyGameMode::AIntegratedAssemblyGameMode() { PlayerControllerClass=AIntegratedAssemblyController::StaticClass(); DefaultPawnClass=nullptr; HUDClass=nullptr; }

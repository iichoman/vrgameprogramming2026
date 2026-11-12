#include "FactoryInspection.h"
#include "FactoryRuntime.h"
#include "FactoryAssemblyEquipment.h"
#include "FactoryEquipment.h"
#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "DrawDebugHelpers.h"

namespace
{
FString ModuleLabel(FName Id)
{
    const TMap<FName,FString> Names={
        {TEXT("Seat_Front_L"),TEXT("왼쪽 앞좌석")},{TEXT("Seat_Front_R"),TEXT("오른쪽 앞좌석")},{TEXT("Seat_Rear"),TEXT("뒷좌석")},
        {TEXT("Door_RL"),TEXT("왼쪽 뒷문")},{TEXT("Door_RR"),TEXT("오른쪽 뒷문")},{TEXT("Engine"),TEXT("엔진")},
        {TEXT("SteeringWheel"),TEXT("핸들")},{TEXT("SteeringColumn"),TEXT("조향 컬럼")},{TEXT("CockpitFrame"),TEXT("콕핏 프레임")},
        {TEXT("Cockpit"),TEXT("콕핏")},{TEXT("Windshield"),TEXT("앞유리")},{TEXT("RearGlass"),TEXT("뒷유리")},
        {TEXT("Hood"),TEXT("보닛")},{TEXT("TrunkLid"),TEXT("트렁크")},{TEXT("Transmission"),TEXT("변속기")},
        {TEXT("RunningGear_Front"),TEXT("전륜 구동·현가 모듈")},{TEXT("RunningGear_Rear"),TEXT("후륜 현가 모듈")},
        {TEXT("Subframe_Front"),TEXT("전방 서브프레임")},{TEXT("Subframe_Rear"),TEXT("후방 서브프레임")},
        {TEXT("Bumper_Front"),TEXT("앞 범퍼")},{TEXT("Bumper_Rear"),TEXT("뒤 범퍼")},{TEXT("CenterConsole"),TEXT("센터 콘솔")},
        {TEXT("AuxBattery"),TEXT("보조 배터리")},{TEXT("CoolingModule"),TEXT("냉각 모듈")},{TEXT("BrakeBooster"),TEXT("브레이크 부스터")},
        {TEXT("ExhaustSystem"),TEXT("배기 계통")},{TEXT("FuelSystem"),TEXT("연료 계통")},{TEXT("PowertrainMounts_Review"),TEXT("구동계 마운트")},
        {TEXT("PedalBrake_Review"),TEXT("페달·브레이크 부품")},{TEXT("InteriorFixed_Review"),TEXT("고정 내장재")},
        {TEXT("Lamp_Front_L"),TEXT("왼쪽 전조등")},{TEXT("Lamp_Front_R"),TEXT("오른쪽 전조등")},
        {TEXT("Lamp_Rear_L"),TEXT("왼쪽 후미등")},{TEXT("Lamp_Rear_R"),TEXT("오른쪽 후미등")}};
    const auto* N=Names.Find(Id);return (N?*N:TEXT("차량 부품"))+TEXT(" (")+Id.ToString()+TEXT(")");
}
FString WheelStageLabel(EWheelStage S)
{
    switch(S){case EWheelStage::Idle:return TEXT("대기");case EWheelStage::ApproachPickup:return TEXT("공급 부품 접근");case EWheelStage::Grip:return TEXT("파지 확인");case EWheelStage::Lift:return TEXT("상승");case EWheelStage::Transfer:return TEXT("운반");case EWheelStage::ApproachMount:return TEXT("장착점 접근");case EWheelStage::Align:return TEXT("정렬");case EWheelStage::Fasten:return TEXT("체결");case EWheelStage::Release:return TEXT("놓기");case EWheelStage::Retract:return TEXT("복귀");case EWheelStage::Reposition:return TEXT("다음 작업 이동");case EWheelStage::WaitSupply:return TEXT("부품 공급 대기");case EWheelStage::WaitPose:return TEXT("자세 측정 대기");case EWheelStage::Complete:return TEXT("완료");case EWheelStage::Stopped:return TEXT("정지");default:return TEXT("이상 중단");}
}
bool VisibleMesh(const UStaticMeshComponent* M)
{return IsValid(M)&&M->GetStaticMesh()&&M->IsVisible()&&!M->bHiddenInGame&&!M->GetOwner()->IsHidden();}
bool IntersectsRay(FVector O,FVector D,const FBox& B,double& Distance)
{
    if(!B.IsValid)return false;double Lo=0,Hi=100000;
    for(int A=0;A<3;++A){if(FMath::Abs(D[A])<1e-8){if(O[A]<B.Min[A]||O[A]>B.Max[A])return false;continue;}
        double X=(B.Min[A]-O[A])/D[A],Y=(B.Max[A]-O[A])/D[A];if(X>Y)Swap(X,Y);Lo=FMath::Max(Lo,X);Hi=FMath::Min(Hi,Y);if(Lo>Hi)return false;}
    Distance=Lo;return true;
}
}
FFactoryInspectionModel& AFactoryRuntime::Inspection()
{if(!InspectionState)InspectionState=MakeShared<FFactoryInspectionModel>();return *InspectionState;}
FString AFactoryRuntime::CurrentObjectId() const
{return !SelectedId.IsEmpty()?SelectedId:!SelectedPartId.IsEmpty()?SelectedPartId:InspectionState?InspectionState->LooseSelection:FString();}
FString AFactoryRuntime::ObjectName(const FString& Id) const
{
    if(Id==TEXT("vehicle_1"))return TEXT("차체 (Vehicle Body)");
    if(Id==TEXT("line_conveyor"))return TEXT("차체 운반 대차 (Transfer Carriage)");
    if(Id==TEXT("hall"))return TEXT("공장 (Factory)");
    if(const auto* I=Layout.Find(Id))return I->DisplayName();
    if(Id.StartsWith(TEXT("part_")))if(const auto* T=FactoryTasks::Find(Id.Mid(5)))return T->Label+TEXT(" 부품");
    if(Id.StartsWith(TEXT("module:")))return ModuleLabel(FName(*Id.Mid(7)));
    return Id.IsEmpty()?TEXT("선택된 오브젝트 없음"):Id;
}
TArray<FString> AFactoryRuntime::InspectableObjects() const
{
    TArray<FString> Out;for(const auto& I:Layout.Items)Out.Add(I.Id);
    if(bUseAssemblyIntegration&&TestAssemblyLine)
    {
        Out.Add(TEXT("vehicle_1"));Out.Add(TEXT("line_conveyor"));
        for(const auto& S:FactoryTasks::Catalog())Out.Add(TEXT("part_")+S.Id);
        auto* Car=TestAssemblyLine->WheelCell?TestAssemblyLine->WheelCell->Assembly.Get():nullptr;
        for(TActorIterator<AVehicleProcessModule> It(GetWorld());It;++It)if(It->VehicleFrame==Car)
        {const FString Id=It->ModuleId.ToString();if(!Id.StartsWith(TEXT("Body_"))&&!Id.StartsWith(TEXT("Wheel_"))&&Id!=TEXT("Door_FL")&&Id!=TEXT("Door_FR"))Out.Add(TEXT("module:")+Id);}
    }
    return Out;
}
TArray<UStaticMeshComponent*> AFactoryRuntime::ObjectMeshes(const FString& Id) const
{
    TArray<UStaticMeshComponent*> Out;
    auto AddActor=[&](const AActor* A){if(!IsValid(A))return;TArray<UStaticMeshComponent*> Components;A->GetComponents(Components);for(auto* M:Components)if(VisibleMesh(M)&&!M->ComponentHasTag(TEXT("SelectionFootprint"))&&!M->ComponentHasTag(TEXT("EquipmentFootprint")))Out.AddUnique(M);};
    if(auto* A=Equipment.FindRef(Id).Get()){if(auto* R=Cast<AFactoryAssemblyEquipment>(A))AddActor(R->GetArm());else AddActor(A);return Out;}
    auto* L=TestAssemblyLine.Get();auto* Car=L&&L->WheelCell?L->WheelCell->Assembly.Get():nullptr;if(!Car)return Out;
    if(Id==TEXT("line_conveyor")){AddActor(L->WheelCell->TransportCarriage);return Out;}
    FString Module;
    if(Id.StartsWith(TEXT("module:")))Module=Id.Mid(7);
    const TMap<FString,FString> PartModules={{TEXT("part_wheel_fl"),TEXT("Wheel_FL")},{TEXT("part_wheel_rl"),TEXT("Wheel_RL")},{TEXT("part_wheel_fr"),TEXT("Wheel_FR")},{TEXT("part_wheel_rr"),TEXT("Wheel_RR")},{TEXT("part_door_fl"),TEXT("Door_FL")},{TEXT("part_door_fr"),TEXT("Door_FR")}};
    if(const auto* N=PartModules.Find(Id))Module=*N;
    for(TActorIterator<AVehicleProcessModule> It(GetWorld());It;++It)if(It->VehicleFrame==Car)
        if((!Module.IsEmpty()&&It->ModuleId==FName(*Module))||(Id==TEXT("vehicle_1")&&It->ModuleId.ToString().StartsWith(TEXT("Body_"))))AddActor(*It);
    if(Out.IsEmpty()&&Id==TEXT("vehicle_1"))AddActor(Car);
    return Out;
}
FBox AFactoryRuntime::ObjectBounds(const FString& Id) const
{
    FBox B(ForceInit);for(auto* M:ObjectMeshes(Id))B+=M->CalcBounds(M->GetComponentTransform()).GetBox();
    if(!B.IsValid)if(const auto* I=Layout.Find(Id))B=FBox(FVector(-I->Size*.5,0),FVector(I->Size*.5,40)).TransformBy(I->Transform());
    if(!B.IsValid&&Id.StartsWith(TEXT("part_")))B=PartBounds(Id);return B;
}
FString AFactoryRuntime::HitInspectable(FVector2D Pixel) const
{
    auto* PC=UGameplayStatics::GetPlayerController(this,0);FVector O,D;if(!PC||!PC->DeprojectScreenPositionToWorld(Pixel.X,Pixel.Y,O,D))return {};
    FString Best;double Nearest=100000;
    for(const auto& Id:InspectableObjects())
    {
        if(Layout.Find(Id)||Id.StartsWith(TEXT("part_")))continue;
        for(auto* M:ObjectMeshes(Id)){double T;if(IntersectsRay(O,D,M->CalcBounds(M->GetComponentTransform()).GetBox(),T)&&T<Nearest){Nearest=T;Best=Id;}}
    }
    return Best;
}
void AFactoryRuntime::SelectObject(const FString& Id)
{
    CloseRobotPopup();SelectedId.Empty();SelectedPartId.Empty();Inspection().LooseSelection.Empty();
    if(Layout.Find(Id))SelectedId=Id;else if(Id.StartsWith(TEXT("part_")))SelectedPartId=Id;else Inspection().LooseSelection=Id;
    Inspection().FocusedIssue.Empty();OpenDetails(EFactoryDetailTab::Selection);RefreshHighlights();
}
void AFactoryRuntime::OpenDetails(EFactoryDetailTab Tab)
{auto& M=Inspection();M.PanelOpen=true;M.Tab=Tab;bNeedsPanel=true;}
void AFactoryRuntime::FocusObject(const FString& Id)
{const FBox B=ObjectBounds(Id);if(B.IsValid){CameraCentre=FVector2D(B.GetCenter());CameraWidth=FMath::Clamp(B.GetSize().GetMax()*3.,650.,6000.);UpdateCamera();}}
void AFactoryRuntime::FocusIssue(const FString& Id)
{
    auto& M=Inspection();const auto* I=M.Issues.FindByPredicate([&](const auto& V){return V.Id==Id;});if(!I)return;
    const FString Target=!I->RobotId.IsEmpty()&&Layout.Find(I->RobotId)?I->RobotId:!I->PartId.IsEmpty()?I->PartId:I->Objects.IsEmpty()?TEXT("vehicle_1"):I->Objects[0];
    const auto SavedInteraction=M.InteractionIssue;SelectObject(Target);M.InteractionIssue=SavedInteraction;M.FocusedIssue=Id;FocusObject(Target);OpenDetails(EFactoryDetailTab::Problems);
}
void AFactoryRuntime::RefreshInspection()
{
    auto& M=Inspection();FString Previous;for(const auto& I:M.Issues)Previous+=I.Id;
    M.Issues.Reset();
    auto Issue=[&](FString Id,FString Summary,FString Detail,FString Remedy,FString Robot,FString Part,FString Task,TArray<FString> Objects,EFactoryIssueLevel Level=EFactoryIssueLevel::Error)
    {FFactoryIssue I;I.Id=Id;I.Summary=Summary;I.Detail=Detail;I.Remedy=Remedy;I.RobotId=Robot;I.PartId=Part;I.TaskId=Task;I.Objects=MoveTemp(Objects);I.Level=Level;
        if(!Robot.IsEmpty())I.Objects.AddUnique(Robot);if(!Part.IsEmpty())I.Objects.AddUnique(Part);M.Issues.Add(MoveTemp(I));};
    for(const auto& I:Layout.Items)if(!Layout.Contains(I))Issue(TEXT("outside:")+I.Id,TEXT("공장 경계 밖 배치"),I.DisplayName(),TEXT("강조된 오브젝트를 공장 안으로 이동하거나 공장 크기를 늘리세요."),I.Kind==EFactoryItemKind::ArticulatedRobot?I.Id:FString(),{},{},{I.Id});
    auto* L=TestAssemblyLine.Get();auto* W=L?L->WheelCell.Get():nullptr;
    if(bUseAssemblyIntegration)
    {
        TSet<FString> Parts;
        for(const auto& T:Layout.Tasks)
        {
            const auto* S=FactoryTasks::Find(T.Id);const auto* R=Layout.Find(T.RobotId);const FString Part=T.PartId.IsEmpty()?TEXT("part_")+T.Id:T.PartId;
            if(!R)Issue(TEXT("unassigned:")+T.Id,TEXT("작업 로봇 미할당"),S?S->Label:T.Id,TEXT("호환 로봇을 선택한 뒤 차체 작업 지점과 실제 부품을 차례로 클릭하세요."),{},Part,T.Id,{TEXT("vehicle_1")});
            else if(!S||!FactoryTasks::Supports(R->EffectiveTypeId(),*S))Issue(TEXT("tool:")+T.Id,TEXT("로봇 툴과 작업 불일치"),ObjectName(R->Id),TEXT("휠 작업은 휠 로봇, 도어 작업은 도어 로봇에 연결하세요."),R->Id,Part,T.Id,{});
            if(Parts.Contains(Part))Issue(TEXT("duplicate:")+T.Id,TEXT("동일 부품 중복 연결"),ObjectName(Part),TEXT("기존 연결을 해제하고 작업마다 서로 다른 부품을 지정하세요."),T.RobotId,Part,T.Id,{});Parts.Add(Part);
            if(W&&S&&S->WheelIndex>=0&&S->WheelIndex%2==1&&R&&W->RearTransit[S->WheelIndex].Num()!=6)
                Issue(TEXT("reach:")+T.Id,TEXT("후방 작업 자세에 도달할 수 없음"),S->Label,TEXT("강조된 로봇의 위치·방향을 조정하세요. 전체 경로의 충돌 검사를 대신하지 않습니다."),T.RobotId,Part,T.Id,{});
        }
        const auto* RightTask=Layout.Tasks.FindByPredicate([](const auto& T){return T.Id==TEXT("door_fr");});
        if(RightTask&&L&&L->DoorCell&&RightDoorCell&&RightDoorCell->Robot&&RightDoorCell->Robot==L->DoorCell->Robot)
            Issue(TEXT("door_resource"),TEXT("도어 병렬 작업 로봇 충돌"),TEXT("두 앞문에 동일 로봇이 지정되었습니다."),TEXT("현재 도어 실행은 서로 다른 로봇 2대가 필요합니다."),RightTask->RobotId,TEXT("part_door_fr"),TEXT("door_fr"),{TEXT("part_door_fl")});
        if(W)for(int Lane=0;Lane<W->LaneCount();++Lane)
        {
            const auto* State=W->LaneSnapshot(Lane);if(!State||!W->LaneEnabled(Lane)||State->WheelIndex<0)continue;
            const auto* T=Layout.Tasks.FindByPredicate([&](const auto& X){const auto* S=FactoryTasks::Find(X.Id);return S&&S->WheelIndex==State->WheelIndex;});if(!T)continue;
            if(State->Stage==EWheelStage::WaitSupply||State->Stage==EWheelStage::WaitPose)
                Issue(TEXT("wait:")+T->Id,WheelStageLabel(State->Stage),FactoryTasks::Find(T->Id)->Label,TEXT("공급 부품·센서 유효 상태를 확인하세요. 현재 복구를 기다리는 중입니다."),T->RobotId,T->PartId,T->Id,{},EFactoryIssueLevel::Warning);
        }
        if(W&&W->Outcome==EWheelOutcome::Blocked)
        {
            const auto* T=Layout.Tasks.FindByPredicate([&](const auto& X){const auto* S=FactoryTasks::Find(X.Id);return S&&S->WheelIndex==W->FailedWheelIndex;});
            Issue(TEXT("wheel_fault"),TEXT("휠 공정 중단 · ")+WheelStageLabel(W->FailedStage),W->Status,TEXT("강조된 로봇·부품을 확인한 뒤 편집 복귀로 초기화하세요."),T?T->RobotId:FString(),T?T->PartId:FString(),T?T->Id:FString(),T?TArray<FString>{}:TArray<FString>{TEXT("vehicle_1")});
        }
        for(const TCHAR* Id:{TEXT("door_fl"),TEXT("door_fr")})
        {
            auto* D=FString(Id)==TEXT("door_fr")?RightDoorCell.Get():L?L->DoorCell.Get():nullptr;
            const auto* T=Layout.Tasks.FindByPredicate([&](const auto& X){return X.Id==Id;});
            if(D&&T&&D->State==EDoorAssemblyState::Fault)Issue(FString(TEXT("door_fault:"))+Id,TEXT("도어 공정 중단"),D->Status,TEXT("로봇 도달 범위, 부품 자세와 정렬·검사 센서를 확인하세요."),T->RobotId,T->PartId,Id,{});
        }
        if(L&&L->ArrivalPhase==EArrivalPhase::Fault)Issue(TEXT("arrival_fault"),TEXT("차체 반입·도킹 중단"),L->Status,TEXT("레일 구동, 도착·위치·속도 센서와 차체 위치를 확인하세요."),{},{},{},{TEXT("vehicle_1"),TEXT("line_conveyor")});
    }
    if(RunState==EFactoryRunState::Fault&&M.Issues.IsEmpty())Issue(TEXT("runtime_fault"),TEXT("공정 중단 · 원인 대상 확인 필요"),Status,TEXT("공정 상세 로그를 확인하세요. 관련 오브젝트가 식별되지 않아 특정 로봇을 추정하지 않습니다."),{},{},{},{TEXT("vehicle_1")});
    if(M.InteractionIssue.IsSet())M.Issues.Add(M.InteractionIssue.GetValue());
    if(!M.StartError.IsEmpty()&&M.Issues.IsEmpty())Issue(TEXT("start_rejected"),TEXT("실행 조건을 확인하세요"),M.StartError,TEXT("공정의 시작 조건을 확인한 뒤 다시 실행하세요."),{},{},{},{});
    FString Current;for(const auto& I:M.Issues)Current+=I.Id;if(Current!=Previous)bNeedsPanel=true;
    const int ChannelsBefore=M.Sensors.Num();
    TSet<FString> NativeChannels;
    auto Publish=[&](FString Id,FString Label,FString Object,FString Unit,double Value,double Stamp,double Now,bool Valid,FString Source=TEXT("가상 센서"),FString Task=FString())
    {NativeChannels.Add(Id);FFactorySensorValue V;V.Id=Id;V.Label=Label;V.ObjectId=Object;V.Unit=Unit;V.Value=Value;V.SampleTime=Stamp;V.ClockTime=Now;V.Valid=Valid;V.Connected=true;V.Source=Source;V.TaskId=Task;M.Publish(V);};
    if(L&&L->Sensors)
    {
        const auto* S=L->Sensors.Get();Publish(TEXT("arrival.speed"),TEXT("차체 이동 속도"),TEXT("vehicle_1"),TEXT("m/s"),S->Speed.Value.Size()/100,S->Speed.MeasuredAt,L->SensorTime,S->Speed.bValid);
        Publish(TEXT("arrival.position"),TEXT("정지점 기준 차체 X"),TEXT("vehicle_1"),TEXT("m"),S->Position.Value.X/100,S->Position.MeasuredAt,L->SensorTime,S->Position.bValid);
        Publish(TEXT("arrival.detected"),TEXT("도착 감지"),TEXT("vehicle_1"),TEXT("0/1"),S->Arrival.Value.X,S->Arrival.MeasuredAt,L->SensorTime,S->Arrival.bValid);
    }
    if(W)for(int Lane=0;Lane<W->LaneCount();++Lane)
    {
        const auto* S=W->SensorsFor(Lane);const auto* State=W->LaneSnapshot(Lane);if(!S||!State||!W->LaneEnabled(Lane))continue;
        const int Index=State->WheelIndex>=0?State->WheelIndex:W->TaskQueue[Lane].IsEmpty()?-1:W->TaskQueue[Lane][0];
        const auto* T=Layout.Tasks.FindByPredicate([&](const auto& X){const auto* Spec=FactoryTasks::Find(X.Id);return Spec&&Spec->WheelIndex==Index;});if(!T)continue;
        const FString Prefix=TEXT("wheel.")+T->RobotId;const FString Source=S->ObservationProvider?TEXT("관측 Provider"):TEXT("가상 센서");
        Publish(Prefix+TEXT(".presence"),TEXT("부품 감지"),T->RobotId,TEXT("0/1"),S->Presence.bDetected?1:0,S->Presence.MeasuredAt,W->SimulationTimeSeconds,S->Presence.bValid,Source,T->Id);
        Publish(Prefix+TEXT(".grip"),TEXT("파지 감지"),T->RobotId,TEXT("0/1"),S->Gripped.bDetected?1:0,S->Gripped.MeasuredAt,W->SimulationTimeSeconds,S->Gripped.bValid,TEXT("가상 접촉 센서"),T->Id);
        Publish(Prefix+TEXT(".pickup_x"),TEXT("부품 측정 X"),T->RobotId,TEXT("m"),S->PickupPose.Pose.GetLocation().X/100,S->PickupPose.MeasuredAt,W->SimulationTimeSeconds,S->PickupPose.bValid,Source,T->Id);
    }
    for(const TCHAR* Id:{TEXT("door_fl"),TEXT("door_fr")})
    {
        const auto* D=FString(Id)==TEXT("door_fr")?RightDoorCell.Get():L?L->DoorCell.Get():nullptr;if(!D)continue;
        Publish(FString(Id)+TEXT(".alignment"),TEXT("도어 정렬 오차"),TEXT("part_")+FString(Id),TEXT("mm"),D->AlignmentPositionErrorMm,D->LastAlignmentSampleTime,D->SimulationTimeSeconds,D->bAlignmentSensorValid&&D->bLastAlignmentSampleValid,TEXT("가상 자세 센서"),Id);
    }
    for(const auto& Entry:TArray<TPair<FString,FString>>{{TEXT("extension.camera"),TEXT("카메라 센서")},{TEXT("extension.pressure"),TEXT("압력 센서")}})
        if(!M.Sensors.Contains(Entry.Key)){FFactorySensorValue V;V.Id=Entry.Key;V.Label=Entry.Value;V.Source=TEXT("센서 어댑터 미연결");M.Publish(V);}
    for(auto It=M.Sensors.CreateIterator();It;++It)
        if((It.Key().StartsWith(TEXT("arrival."))||It.Key().StartsWith(TEXT("wheel."))||It.Key().StartsWith(TEXT("door_fl."))||It.Key().StartsWith(TEXT("door_fr.")))&&!NativeChannels.Contains(It.Key()))It.RemoveCurrent();
    if(ChannelsBefore!=M.Sensors.Num())bNeedsPanel=true;
}
void AFactoryRuntime::UpdateObjectHighlights()
{
    auto& M=Inspection();if(!M.SelectedMaterial.IsValid())
    {
        auto* Base=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/DigitalTwin/Factory/Runtime/M_FactoryTint"));
        if(Base){auto* Blue=UMaterialInstanceDynamic::Create(Base,this);Blue->SetVectorParameterValue(TEXT("Tint"),FLinearColor(.06f,.55f,1.f));M.SelectedMaterial.Reset(Blue);
            auto* Orange=UMaterialInstanceDynamic::Create(Base,this);Orange->SetVectorParameterValue(TEXT("Tint"),FLinearColor(1.f,.25f,.025f));M.IssueMaterial.Reset(Orange);}
    }
    TMap<TWeakObjectPtr<UStaticMeshComponent>,uint8> Desired;const FString SelectedObject=CurrentObjectId();
    auto Mark=[&](const FString& Id,uint8 Kind){for(auto* C:ObjectMeshes(Id))Desired.Add(C,Kind);};
    if(!SelectedObject.IsEmpty())Mark(SelectedObject,1);
    for(const auto& I:M.Issues)for(const auto& Id:I.Objects)Mark(Id,2);
    for(auto It=M.OriginalOverlays.CreateIterator();It;++It)if(!Desired.Contains(It.Key())){if(It.Key().IsValid())It.Key()->SetOverlayMaterial(It.Value().Get());M.OverlayKinds.Remove(It.Key());It.RemoveCurrent();}
    for(const auto& P:Desired)if(P.Key.IsValid())
    {if(!M.OriginalOverlays.Contains(P.Key))M.OriginalOverlays.Add(P.Key,TStrongObjectPtr<UMaterialInterface>(P.Key->GetOverlayMaterial()));
        if(M.OverlayKinds.FindRef(P.Key)!=P.Value){P.Key->SetOverlayMaterial(P.Value==1?M.SelectedMaterial.Get():M.IssueMaterial.Get());M.OverlayKinds.Add(P.Key,P.Value);}}
    auto Frame=[&](const FString& Id,FColor Color,float Thick){const FBox B=ObjectBounds(Id);if(B.IsValid)DrawDebugBox(GetWorld(),B.GetCenter(),B.GetExtent()+FVector(4),Color,false,0,SDPG_Foreground,Thick);};
    if(!SelectedObject.IsEmpty())Frame(SelectedObject,FColor(50,190,255),3);
    for(const auto& I:M.Issues)
    {
        const FColor Color=I.Id==M.FocusedIssue?FColor::Red:FColor::Orange;
        for(const auto& Id:I.Objects)Frame(Id,Color,I.Id==M.FocusedIssue?5:2);
        FVector P;if(!I.TaskId.IsEmpty()&&TaskTargetWorld(I.TaskId,P))DrawDebugSphere(GetWorld(),P,20,12,Color,false,0,SDPG_Foreground,3);
    }
}
FString AFactoryRuntime::ProcessDetails() const
{
    FString Out=StateText()+TEXT("\n\n차체 반입 → 휠 작업 → 구역 이송 → 앞문 작업\n\n");
    const auto* W=TestAssemblyLine?TestAssemblyLine->WheelCell.Get():nullptr;
    for(const auto& T:Layout.Tasks)
    {
        const auto* S=FactoryTasks::Find(T.Id);FString State=TEXT("대기");
        if(T.RobotId.IsEmpty())State=TEXT("미할당");
        else if(S&&W&&S->WheelIndex>=0)
        {if(W->FastenedWheels.IsValidIndex(S->WheelIndex)&&W->FastenedWheels[S->WheelIndex])State=TEXT("장착 완료");
            else for(int I=0;I<W->LaneCount();++I)if(const auto* Lane=W->LaneSnapshot(I))if(Lane->WheelIndex==S->WheelIndex)State=WheelStageLabel(Lane->Stage);}
        else if(S){const auto* D=T.Id==TEXT("door_fr")?RightDoorCell.Get():TestAssemblyLine?TestAssemblyLine->DoorCell.Get():nullptr;if(D)State=D->bDoorFastened?TEXT("장착 완료"):D->State==EDoorAssemblyState::Fault?TEXT("이상 중단"):D->State==EDoorAssemblyState::Idle?TEXT("대기"):TEXT("진행 중");}
        Out+=(S?S->Label:T.Id)+TEXT(" · ")+State+TEXT("\n담당: ")+(T.RobotId.IsEmpty()?TEXT("없음"):ObjectName(T.RobotId))+TEXT("\n부품: ")+ObjectName(T.PartId)+TEXT("\n\n");
    }
    return Out+TEXT("상세 상태\n")+Status;
}
FString AFactoryRuntime::ResultDetails() const
{
    if(!bUseAssemblyIntegration)return FString::Printf(TEXT("%s\n경과 %.2f초\n이동 거리 %.2fm\n정렬 오차 %.2fmm"),*StateText(),SimulationSeconds,TravelCm/100,AlignmentMm);
    FString Out=FString::Printf(TEXT("%s · %.2f초\n"),*StateText(),SimulationSeconds);
    auto* W=TestAssemblyLine?TestAssemblyLine->WheelCell.Get():nullptr;
    if(W)for(const auto& S:FactoryTasks::Catalog())if(S.WheelIndex>=0)
    {const int I=S.WheelIndex;const bool Done=W->FastenedWheels.IsValidIndex(I)&&W->FastenedWheels[I];
        Out+=S.Label+(Done&&W->MountPositionErrorMm.IsValidIndex(I)?FString::Printf(TEXT(" · 장착 / %.2fmm\n"),W->MountPositionErrorMm[I]):TEXT(" · 아직 장착되지 않음\n"));}
    for(const TCHAR* Id:{TEXT("door_fl"),TEXT("door_fr")}){const auto* D=FString(Id)==TEXT("door_fr")?RightDoorCell.Get():TestAssemblyLine?TestAssemblyLine->DoorCell.Get():nullptr;
        if(D)Out+=FactoryTasks::Find(Id)->Label+TEXT(" · ")+(D->bDoorFastened?D->GetQualitySummary():TEXT("아직 검사 완료되지 않음"))+TEXT("\n");}
    return Out+TEXT("\n가상 공정 결과입니다. 현재 질량·접촉력·체결 토크 기반 검증은 포함하지 않습니다.");
}

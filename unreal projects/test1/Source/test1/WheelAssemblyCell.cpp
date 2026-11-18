#include "WheelAssemblyCell.h"
#include "ReducedPartPhysics.h"
#include "WheelGripPhysics.h"
#include "CamryAssembly.h"
#include "VehicleAssemblyMounting.h"
#include "RobotArm.h"
#include "WheelGripperGeometry.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SphereComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "VehicleProcessTest.h"
#include "EngineUtils.h"

namespace
{
// Resolve the visible part by stable module identity, never by the invisible proxy's pose alone.
bool CheckWheelVisuals(AWheelAssemblyCell* Cell,const TArray<TObjectPtr<USceneComponent>>& Roots,ACamryAssembly* Car,bool Repair,FString& Error)
{
    if(!Car)return false;
    const FName Ids[]={TEXT("Wheel_FL"),TEXT("Wheel_RL"),TEXT("Wheel_FR"),TEXT("Wheel_RR")};
    AVehicleProcessModule* Modules[4]={};int Found=0;
    for(TActorIterator<AVehicleProcessModule> It(Cell->GetWorld());It;++It)if(It->VehicleFrame==Car)
        for(int I=0;I<4;++I)if(It->ModuleId==Ids[I]){if(Modules[I]){Error=TEXT("중복 휠 모델: ")+Ids[I].ToString();return false;}Modules[I]=*It;++Found;}
    if(!Found){if(Cell->UsesTaskBindings()){Error=TEXT("작업에 연결할 실제 휠 모델이 없습니다.");return false;}return true;} // Standalone primitive demonstrations use the cell meshes.
    for(int I=0;I<4;++I)
    {
        auto* M=Modules[I];auto* Root=Roots.IsValidIndex(I)?Roots[I].Get():nullptr;
        if(!M||!Root||!M->bHasReferencePose){Error=TEXT("실제 휠 모델/기준 연결 누락: ")+Ids[I].ToString();return false;}
        // Calibration is relative to the nominal hub, independent of the
        // temporary lift/droop and scanned hub displacement during execution.
        auto NominalHub=Cell->GetMountWorld(I);
        FVector Offset=VehicleAssemblyMounting::HubOffset(*Car,I);
        if(I==0&&Cell->bLaserScanFrontLeftHub)Offset+=Cell->FrontLeftHubOffsetCm;
        NominalHub.AddToTranslation(-Car->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(Offset));
        const FTransform InWheel=(M->ReferencePoseInVehicle*Car->GetActorTransform()).GetRelativeTransform(NominalHub);
        if(Repair)
        {
            M->bWheelMotionEnabled=false;M->SetActorTickEnabled(false);
            M->GetRootComponent()->SetAbsolute(false,false,false);
            M->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform); // Rebuild both sides of the attachment after reinstancing.
            if(!M->AttachToComponent(Root,FAttachmentTransformRules::KeepWorldTransform)){Error=TEXT("휠 모델 연결 실패: ")+Ids[I].ToString();return false;}
            M->GetRootComponent()->SetRelativeTransform(InWheel);
        }
        if(M->GetRootComponent()->GetAttachParent()!=Root||M->bWheelMotionEnabled||!M->GetActorTransform().Equals(InWheel*Root->GetComponentTransform(),.01))
        {Error=TEXT("실제 휠 모델이 제어 위치를 따라오지 않습니다: ")+Ids[I].ToString();return false;}
        TArray<UStaticMeshComponent*> Meshes;M->GetComponents(Meshes);bool HasMesh=false;
        for(auto* Mesh:Meshes)if(Mesh->GetStaticMesh())
        {
            HasMesh=true;
            if(Mesh->GetAttachParent()!=M->GetRootComponent()||!Mesh->GetComponentTransform().Equals(Mesh->GetRelativeTransform()*M->GetActorTransform(),.01))
            {Error=TEXT("휠 렌더 메시 연결 오류: ")+Ids[I].ToString();return false;}
        }
        if(!HasMesh){Error=TEXT("휠 모델 메시가 없습니다: ")+Ids[I].ToString();return false;}
    }
    return true;
}
}

void UWheelRobotSensorBank::Reset()
{
	LastSample = -1; LastWheelIndex = INDEX_NONE; bGripClosed = false; bPublishPickupPose = true; bGripActuatorFault = false;
	Presence = PickupPose = Gripped = MountPose = FWheelSensorReading();
}

void UWheelRobotSensorBank::Sample(double Now, const AWheelAssemblyCell* Cell, int32 WheelIndex)
{
	if (LastWheelIndex == WheelIndex && LastSample >= 0 && Now - LastSample + 1.e-9 < SamplePeriod) return;
	LastSample = Now; LastWheelIndex = WheelIndex;
	const bool Ready = IsValid(Cell) && IsValid(Cell->RobotForWheel(WheelIndex)) && WheelIndex >= 0 && WheelIndex < 4;
	const bool Present = Ready && Cell->IsWheelVisible(WheelIndex);
	Presence = FWheelSensorReading(); Presence.bValid = Ready; Presence.bDetected = Present; Presence.MeasuredAt = Now;
	if (bPublishPickupPose)
	{
		PickupPose.bValid = Present; PickupPose.bDetected = Present;
		PickupPose.Pose = Present ? Cell->GetWheelWorld(WheelIndex) : FTransform::Identity;
		PickupPose.MeasuredAt = Now;
	}
	const FTransform Wheel = Present ? Cell->GetWheelWorld(WheelIndex) : FTransform::Identity;
	const FTransform TCP = Ready ? Cell->RobotForWheel(WheelIndex)->GetTCPWorld() : FTransform::Identity;
	const FTransform Contact = Ready ? Cell->TCPForWheel(WheelIndex) * Wheel : FTransform::Identity;
	const bool AtContact = Present && FVector::Distance(Contact.GetLocation(), TCP.GetLocation()) * 10 <= Cell->PositionToleranceMm
		&& FMath::RadiansToDegrees(Contact.GetRotation().AngularDistance(TCP.GetRotation())) <= Cell->OrientationToleranceDeg;
    const int Contacts=Ready?Cell->PhysicalGripContacts(WheelIndex):-1;
    Gripped.bValid = Ready; Gripped.bDetected = bGripClosed && !bGripActuatorFault && (Contacts>=0?Contacts>0:AtContact); Gripped.MeasuredAt = Now;
	MountPose.bValid = Present; MountPose.bDetected = Present; MountPose.Pose = Wheel; MountPose.MeasuredAt = Now;
    if(ObservationProvider)
    {
        // Camera adapter supplies observation only. Grip contact above remains plant truth.
        const TCHAR* PartIds[]={TEXT("part_wheel_fl"),TEXT("part_wheel_rl"),TEXT("part_wheel_fr"),TEXT("part_wheel_rr")};
        const auto M=ObservationProvider->ObservePart(WheelIndex>=0&&WheelIndex<4?PartIds[Cell->PartIndex(WheelIndex)]:TEXT(""),Now);
        const bool Valid=Ready&&M.PartId==PartIds[Cell->PartIndex(WheelIndex)]&&M.PartType==TEXT("wheel")
            &&M.Fresh(Now,MaxAge)&&M.MeasuredWorld.IsValid()&&FMath::IsFinite(M.Confidence)&&M.Confidence>0;
        Presence.bValid=Valid;Presence.bDetected=Valid&&M.bDetected;Presence.MeasuredAt=M.SampleTimeSeconds;
        if(bPublishPickupPose){PickupPose.bValid=Valid;PickupPose.bDetected=Presence.bDetected;
            PickupPose.Pose=M.MeasuredWorld;PickupPose.MeasuredAt=M.SampleTimeSeconds;}
        MountPose=PickupPose;
    }
	// Supply presence is a pickup-zone observation, not merely mesh visibility.
	// A wheel travelling from reserve remains unavailable until its measured pose
	// enters the station. The same tolerance still accepts the 3-5 cm replan demo.
	const auto* Lane=Cell->LaneForWheel(WheelIndex);
	if (Lane && !Lane->bHeld && !Lane->bFastened
		&& PickupPose.Fresh(Now,MaxAge))
		Presence.bDetected = Presence.bDetected
			&& FVector::Distance(PickupPose.Pose.GetLocation(),Cell->GetPickupWorld(WheelIndex).GetLocation())<=8.0;
}

AWheelAssemblyCell::AWheelAssemblyCell()
{
	PrimaryActorTick.bCanEverTick = true;
    WheelProperties.SetNum(4);
	CellRoot = CreateDefaultSubobject<USceneComponent>(TEXT("WheelCellRoot"));
	SetRootComponent(CellRoot);
	FrontLeftHubProxy = CreateDefaultSubobject<USphereComponent>(TEXT("FrontLeftHubScanProxy"));
	FrontLeftHubProxy->SetupAttachment(CellRoot);
	FrontLeftHubProxy->SetMobility(EComponentMobility::Movable);
	FrontLeftHubProxy->InitSphereRadius(8.0);
	FrontLeftHubProxy->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	FrontLeftHubProxy->SetGenerateOverlapEvents(false);
	FrontLeftHubProxy->SetCollisionResponseToAllChannels(ECR_Ignore);
	FrontLeftHubProxy->SetCollisionResponseToChannel(ECC_Visibility,ECR_Block);
	FrontLeftHubProxy->SetHiddenInGame(true);
	DroppedWheelCollision = CreateDefaultSubobject<USphereComponent>(TEXT("DroppedWheelCollision"));
	DroppedWheelCollision->SetupAttachment(CellRoot);
	DroppedWheelCollision->SetMobility(EComponentMobility::Movable);
	DroppedWheelCollision->InitSphereRadius(30.0);
	DroppedWheelCollision->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DroppedWheelCollision->SetCollisionResponseToAllChannels(ECR_Ignore);
	DroppedWheelCollision->SetCollisionResponseToChannel(ECC_PhysicsBody,ECR_Block);
	DroppedWheelCollision->SetGenerateOverlapEvents(false);
	LeftSensors = CreateDefaultSubobject<UWheelRobotSensorBank>(TEXT("LeftRobotSensors"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	DroppedTireVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroppedTireVisual"));
	DroppedTireVisual->SetupAttachment(DroppedWheelCollision);
	DroppedTireVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DroppedTireVisual->SetRelativeRotation(FRotator(0,0,90));
	DroppedTireVisual->SetRelativeScale3D(FVector(0.66,0.66,0.16));
	if(Cylinder.Succeeded())DroppedTireVisual->SetStaticMesh(Cylinder.Object);
	DroppedTireVisual->SetVisibility(false);
	DroppedHubVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroppedHubVisual"));
	DroppedHubVisual->SetupAttachment(DroppedWheelCollision);
	DroppedHubVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DroppedHubVisual->SetRelativeRotation(FRotator(0,0,90));
	DroppedHubVisual->SetRelativeLocation(FVector(0,-8.5,0));
	DroppedHubVisual->SetRelativeScale3D(FVector(0.37,0.37,0.025));
	if(Cylinder.Succeeded())DroppedHubVisual->SetStaticMesh(Cylinder.Object);
	DroppedHubVisual->SetVisibility(false);
	RecoveryWorkerVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RecoveryWorkerMarker"));
	RecoveryWorkerVisual->SetupAttachment(CellRoot);
	RecoveryWorkerVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	RecoveryWorkerVisual->SetRelativeScale3D(FVector(0.32,0.32,1.25));
	if(Cylinder.Succeeded())RecoveryWorkerVisual->SetStaticMesh(Cylinder.Object);
	RecoveryWorkerVisual->SetVisibility(false);
	RecoveryWorkerHeadVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RecoveryWorkerHead"));
	RecoveryWorkerHeadVisual->SetupAttachment(CellRoot);
	RecoveryWorkerHeadVisual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	RecoveryWorkerHeadVisual->SetRelativeScale3D(FVector(0.23));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if(Sphere.Succeeded())RecoveryWorkerHeadVisual->SetStaticMesh(Sphere.Object);
	RecoveryWorkerHeadVisual->SetVisibility(false);
	for (int32 Index = 0; Index < 4; ++Index)
	{
		USceneComponent* Wheel = CreateDefaultSubobject<USceneComponent>(FName(*FString::Printf(TEXT("Wheel%d"), Index)));
		Wheel->SetupAttachment(CellRoot);
		Wheel->SetMobility(EComponentMobility::Movable);
		WheelRoots.Add(Wheel);
		UStaticMeshComponent* Tire = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("Tire%d"), Index)));
		Tire->SetupAttachment(Wheel);
		Tire->SetMobility(EComponentMobility::Movable);
		Tire->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Tire->SetGenerateOverlapEvents(false);
		Tire->SetRelativeRotation(FRotator(0, 0, 90));
		Tire->SetRelativeScale3D(FVector(0.66, 0.66, 0.16));
		if (Cylinder.Succeeded()) Tire->SetStaticMesh(Cylinder.Object);
		TireVisuals.Add(Tire);
		UStaticMeshComponent* Hub = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("WheelHub%d"), Index)));
		Hub->SetupAttachment(Wheel);
		Hub->SetMobility(EComponentMobility::Movable);
		Hub->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Hub->SetGenerateOverlapEvents(false);
		Hub->SetRelativeRotation(FRotator(0, 0, 90));
		Hub->SetRelativeLocation(FVector(0, Index < 2 ? -8.5 : 8.5, 0));
		Hub->SetRelativeScale3D(FVector(0.37, 0.37, 0.025));
		if (Cylinder.Succeeded()) Hub->SetStaticMesh(Cylinder.Object);
		HubVisuals.Add(Hub);
	}
}

ARobotArm* AWheelAssemblyCell::RobotForWheel(int32 Index) const
{ return Index<0||Index>3?nullptr:(UsesTaskBindings()?TaskRobots[Index].Get():(Index<2?LeftRobot.Get():RightRobot.Get())); }
ARobotArm* AWheelAssemblyCell::RobotFor(int32 Side) const
{ return UsesTaskBindings()?(TaskQueue[Side].IsEmpty()?nullptr:RobotForWheel(TaskQueue[Side][0])):(Side==0?LeftRobot.Get():RightRobot.Get()); }
UWheelRobotSensorBank* AWheelAssemblyCell::SensorsFor(int32 Side) const
{ return UsesTaskBindings()?(TaskSensors.IsValidIndex(Side)?TaskSensors[Side].Get():nullptr):LeftSensors.Get(); }
bool AWheelAssemblyCell::SensorDrivenLane(int32 Side) const
{ return UsesTaskBindings()||(Side==0&&bSensorDrivenLeftRobot); }
bool AWheelAssemblyCell::PrepareTaskLanes()
{
    for(auto& Q:TaskQueue)Q.Reset();
    if(!UsesTaskBindings())return true;
    for(int32 I=0;I<4;++I)if(!IsValid(TaskRobots[I])){Status=TEXT("휠 작업의 로봇이 미할당입니다.");return false;}
    TArray<ARobotArm*> Owners;
    for(int32 Index:{0,2,1,3})
    {
        auto* Arm=TaskRobots[Index].Get();int32 Lane=Owners.IndexOfByKey(Arm);
        if(Lane==INDEX_NONE){Lane=Owners.Add(Arm);}TaskQueue[Lane].Add(Index);
    }
    if(TaskSensors.Num()!=4){TaskSensors.SetNum(4);TaskSensors[0]=LeftSensors;
        for(int32 I=1;I<4;++I){TaskSensors[I]=NewObject<UWheelRobotSensorBank>(this);TaskSensors[I]->RegisterComponent();}}
    for(auto S:TaskSensors)if(S)S->Reset();
    return true;
}
FTransform AWheelAssemblyCell::TCPFor(int32 Side) const
{
    if(const auto* TCP=GraspTCPByPart.Find(PartIndex(Lanes[Side].WheelIndex)))return *TCP;
    return GeometricSide(Side)==0?LeftTCPInWheel:RightTCPInWheel;
}

FTransform AWheelAssemblyCell::GetMountWorld(int32 Index) const
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || Index < 0 || Index > 3) return FTransform::Identity;
	const double X = Index % 2 == 0 ? FrontXcm : RearXcm;
	const double Y = Index < 2 ? -HubHalfTrackCm : HubHalfTrackCm;
    const FVector Nominal=MountCentersCm.Num()==4?MountCentersCm[Index]:FVector(X,Y,HubHeightCm);
	FTransform Mount = FTransform(FRotator::ZeroRotator,Nominal+VehicleAssemblyMounting::HubOffset(*Assembly,Index)) * Assembly->GetBodyFrame()->GetComponentTransform();
	if (Index == 0 && bLaserScanFrontLeftHub)
		Mount.AddToTranslation(Assembly->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(FrontLeftHubOffsetCm));
	return Mount;
}

FTransform AWheelAssemblyCell::GetMountTargetWorld(int32 Index) const
{
	return Index == 0 && bLaserScanFrontLeftHub && FrontLeftHubScan.bValid
		? FrontLeftHubScan.MeasuredWorld : GetMountWorld(Index);
}

FString AWheelAssemblyCell::GetFrontLeftPartId() const
{
    return TaskReferences.IsValidIndex(0) && !TaskReferences[0].PartId.IsEmpty()
        ? TaskReferences[0].PartId : TEXT("part_wheel_fl");
}

void AWheelAssemblyCell::SetRunIdentity(const FString& InRequestId, const FString& InProductInstanceId, int32 InRunEpoch)
{
    RequestId=InRequestId; ProductInstanceId=InProductInstanceId; RunEpoch=InRunEpoch;
    FrontLeftActiveWheelInstanceId=ProductInstanceId+TEXT("/")+GetFrontLeftPartId()+TEXT("/initial");
    RejectedWheelInstanceId.Reset(); SpareWheelInstanceId.Reset();
}

void AWheelAssemblyCell::OpenIncident(const TCHAR* Kind, int32 WheelIndex, EWheelStage Stage, const TCHAR* Quantity,
	const TCHAR* Unit, double Measured, double Limit, const FString& Response, const TCHAR* IncidentOutcome)
{
	if (Incidents.Num() >= 64) return;
	static const TCHAR* PartIds[]={TEXT("part_wheel_fl"),TEXT("part_wheel_rl"),TEXT("part_wheel_fr"),TEXT("part_wheel_rr")};
	const int32 Part = WheelIndex>=0 && WheelIndex<4 ? PartIndex(WheelIndex) : INDEX_NONE;
	FWheelIncidentRecord Row;
	Row.Kind = Kind;
	Row.RequestId=RequestId; Row.ProductInstanceId=ProductInstanceId; Row.RunEpoch=RunEpoch;
	Row.PartId = Part>=0 && Part<4 ? PartIds[Part] : TEXT("");
	if (TaskReferences.IsValidIndex(WheelIndex))
	{
		const FWheelTaskReference& Task=TaskReferences[WheelIndex];
		Row.PartId=Task.PartId; Row.TaskId=Task.TaskId;
		Row.TargetPort=Task.TargetPort; Row.SourcePort=Task.SourcePort;
	}
	Row.PartInstanceId=!Row.PartId.IsEmpty()&&!ProductInstanceId.IsEmpty()?ProductInstanceId+TEXT("/")+Row.PartId:TEXT("");
	if (WheelIndex==0 && !FrontLeftActiveWheelInstanceId.IsEmpty()) Row.PartInstanceId=FrontLeftActiveWheelInstanceId;
	Row.Stage = StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Stage));
	Row.DetectedAt = SimulationTimeSeconds;
	Row.Quantity = Quantity; Row.Unit = Unit; Row.Measured = Measured; Row.Limit = Limit;
	Row.Response = Response; Row.Outcome = IncidentOutcome;
	if (Row.Outcome != TEXT("pending")) Row.ResolvedAt = SimulationTimeSeconds;
	Incidents.Add(MoveTemp(Row));
}

void AWheelAssemblyCell::CloseIncidents(const TCHAR* Kind, const TCHAR* IncidentOutcome)
{
	for (FWheelIncidentRecord& Row : Incidents)
		if (Row.Outcome == TEXT("pending") && (!Kind || Row.Kind == Kind))
		{ Row.Outcome = IncidentOutcome; Row.ResolvedAt = SimulationTimeSeconds; }
}

int32 AWheelAssemblyCell::CountIncidents(const TCHAR* Kind, const TCHAR* IncidentOutcome) const
{
	int32 Count = 0;
	for (const FWheelIncidentRecord& Row : Incidents)
		if (Row.Kind == Kind && Row.Outcome == IncidentOutcome) ++Count;
	return Count;
}

FWheelEvaluationSnapshot AWheelAssemblyCell::GetEvaluationSnapshot() const
{
    FWheelEvaluationSnapshot Snapshot;
    Snapshot.Model = IsValid(Assembly) && IsValid(LeftRobot) && IsValid(RightRobot)
        ? EWheelModelAssessment::DemoOnly : EWheelModelAssessment::InvalidSetup;
    if (bRunning) Snapshot.Control = EWheelControlAssessment::Running;
    else if (Outcome == EWheelOutcome::Pass) Snapshot.Control = EWheelControlAssessment::Completed;
    else if (Outcome == EWheelOutcome::Blocked || Outcome == EWheelOutcome::Stopped || Outcome == EWheelOutcome::QualityFail)
        Snapshot.Control = EWheelControlAssessment::Failed;
    if (Outcome == EWheelOutcome::Pass) Snapshot.DemoGeometry = EWheelQualityAssessment::DemoGeometryPassed;
    else if (Outcome == EWheelOutcome::QualityFail) Snapshot.DemoGeometry = EWheelQualityAssessment::DemoGeometryFailed;
    // The current Transform-driven mount and time-based fastening cannot establish physical quality.
    Snapshot.PhysicalQuality = EWheelQualityAssessment::NotAssessed;
    return Snapshot;
}

FString AWheelAssemblyCell::GetIncidentSummary(int32 MaxRows) const
{
	if (Incidents.IsEmpty()) return TEXT("기록된 사건 없음");
	FString Text;
	for (int32 Index = FMath::Max(0, Incidents.Num()-FMath::Max(1,MaxRows)); Index < Incidents.Num(); ++Index)
	{
		const FWheelIncidentRecord& Row = Incidents[Index];
		const TCHAR* Result = Row.Outcome == TEXT("recovered") ? TEXT("복구 완료")
			: Row.Outcome == TEXT("stopped") ? TEXT("안전 중단")
			: Row.Outcome == TEXT("observed") ? TEXT("확인") : TEXT("진행 중");
		Text += FString::Printf(TEXT("%.2fs %s"), Row.DetectedAt, *Row.Kind);
		if (!Row.Quantity.IsEmpty())
			Text += FString::Printf(TEXT(" · %s %.2f%s / 기준 %.2f%s"), *Row.Quantity, Row.Measured, *Row.Unit, Row.Limit, *Row.Unit);
		Text += FString::Printf(TEXT(" · %s → %s\n"), *Row.Response, Result);
	}
	return Text;
}

FString AWheelAssemblyCell::GetIncidentCsv() const
{
	FString Csv = TEXT("request_id,product_instance_id,part_instance_id,run_epoch,detected_s,kind,part_id,task_id,target_port,source_port,stage,quantity,unit,measured,limit,response,outcome,resolved_s\n");
	for (const FWheelIncidentRecord& Row : Incidents)
		Csv += FString::Printf(TEXT("%s,%s,%s,%d,%.4f,%s,%s,%s,%s,%s,%s,%s,%s,%.4f,%.4f,%s,%s,%.4f\n"), *Row.RequestId, *Row.ProductInstanceId, *Row.PartInstanceId, Row.RunEpoch, Row.DetectedAt, *Row.Kind, *Row.PartId, *Row.TaskId, *Row.TargetPort, *Row.SourcePort, *Row.Stage,
			*Row.Quantity.Replace(TEXT(","),TEXT(";")), *Row.Unit, Row.Measured, Row.Limit,
			*Row.Response.Replace(TEXT(","),TEXT(";")), *Row.Outcome, Row.ResolvedAt);
	return Csv;
}

const TCHAR* AWheelAssemblyCell::GetSlipFailureLabel() const
{
	return bSlipFailureGripDegraded ? TEXT("GRIP DEGRADED IN TRANSIT")
		: bSlipFailureLowFriction ? TEXT("LOW FRICTION") : TEXT("OVERWEIGHT");
}

// Bounded re-scan. Each attempt is a fresh measurement; a transient occlusion can
// clear, while a permanent fault (blocked beam, outlier, missing proxy) still fails.
// Attempts run within one fixed step: scanner cycle time is not modelled.
bool AWheelAssemblyCell::ScanFrontLeftHub(bool bScanWheel)
{
	const int32 MaxAttempts=FMath::Clamp(MaxLaserScanAttempts,1,5);
	FWheelHubScanReading& Reading=bScanWheel?FrontLeftFinalWheelScan:FrontLeftHubScan;
	for (int32 Attempt=1; Attempt<=MaxAttempts; ++Attempt)
	{
		const bool bAccepted=ScanFrontLeftHubOnce(bScanWheel);
		Reading.Attempts=Attempt;
		if (bAccepted)
		{
			if (Attempt>1) OpenIncident(TEXT("laser_rescan"),0,Lanes[0].Stage,TEXT("레이저 측정 시도"),TEXT("회"),
				Attempt,MaxAttempts,TEXT("재스캔한 측정값 채택"),TEXT("recovered"));
			return true;
		}
		if (Attempt<MaxAttempts) ++LaserScanRetries;
	}
	if (MaxAttempts>1) Reading.FailureReason+=FString::Printf(TEXT(" (%d회 시도)"),MaxAttempts);
	OpenIncident(TEXT("laser_scan_failed"),0,Lanes[0].Stage,TEXT("레이저 측정 시도"),TEXT("회"),
		MaxAttempts,MaxAttempts,TEXT("측정 무효 · 작업 차단: ")+Reading.FailureReason);
	return false;
}

bool AWheelAssemblyCell::ScanFrontLeftHubOnce(bool bScanWheel)
{
	FWheelHubScanReading& Reading=bScanWheel?FrontLeftFinalWheelScan:FrontLeftHubScan;
	Reading = FWheelHubScanReading();
	Reading.TargetPort=bScanWheel?TEXT("wheel_fl_seated"):TEXT("hub_fl");
	if (bBlockFrontLeftLaser) { Reading.FailureReason=TEXT("광선 차단"); return false; }
	if (LaserTransientBlocksRemaining>0)
	{ --LaserTransientBlocksRemaining; Reading.FailureReason=TEXT("광선 일시 가림"); return false; }
	if (!IsValid(Assembly) || !IsValid(RobotForWheel(0)) || !IsValid(FrontLeftHubProxy))
	{ Reading.FailureReason=TEXT("스캐너 또는 허브 연결 누락"); return false; }
	if (!FMath::IsFinite(LaserOutlierMm) || FrontLeftHubOffsetCm.ContainsNaN())
	{ Reading.FailureReason=TEXT("스캔 설정값 오류"); return false; }
	// The proxy follows the plant hub. The scanner aims using the docked nominal
	// pose and reconstructs the center only from collision hits and normals.
	constexpr double RadiusCm = 8.0;
	constexpr double RangeCm = 1500.0;
	constexpr double MaxResidualMm = 1.0;
	FrontLeftHubProxy->SetSphereRadius(RadiusCm);
	FrontLeftHubProxy->SetWorldLocation(bScanWheel?GetWheelWorld(0).GetLocation():GetMountWorld(0).GetLocation());
	const FVector NominalCenter = bScanWheel?FrontLeftHubScan.MeasuredWorld.GetLocation():(FTransform(FQuat::Identity,
		MountCentersCm.Num() == 4 ? MountCentersCm[0] : FVector(FrontXcm,-HubHalfTrackCm,HubHeightCm)) * DockBodyWorld).GetLocation();
	const FVector Origin = bScanWheel ? NominalCenter+BodyAtStart.TransformVectorNoScale(FVector(0,-60,0))
		: RobotForWheel(0)->GetTCPWorld().GetLocation();
	Reading.ScanOriginWorld = Origin;
	FVector CenterSum = FVector::ZeroVector;
	TArray<FVector> CenterSamples;
	const FVector2D Offsets[] = {{0,0},{3,0},{-3,0},{0,3},{0,-3}};
	for (int32 RayIndex=0; RayIndex<UE_ARRAY_COUNT(Offsets); ++RayIndex)
	{
		const FVector2D Offset = Offsets[RayIndex];
		const FVector Direction = (NominalCenter + FVector(Offset.X,0,Offset.Y) - Origin).GetSafeNormal();
		const FVector End = Origin + Direction*RangeCm;
		Reading.RayEndsWorld.Add(End);
		FHitResult Hit;
		if (!FrontLeftHubProxy->LineTraceComponent(Hit,Origin,End,FCollisionQueryParams(SCENE_QUERY_STAT(HubLaserScan),false))) continue;
		const FVector MeasuredHit = Hit.ImpactPoint + Direction*(RayIndex==2 ? LaserOutlierMm/10.0 : 0.0);
		Reading.RayEndsWorld.Last() = MeasuredHit;
		Reading.HitPointsWorld.Add(MeasuredHit);
		const FVector Candidate = MeasuredHit - Hit.ImpactNormal*RadiusCm;
		CenterSamples.Add(Candidate);
		CenterSum += Candidate;
	}
	Reading.HitCount = Reading.HitPointsWorld.Num();
	Reading.MeasuredAt = SimulationTimeSeconds;
	Reading.Confidence = double(Reading.HitCount)/UE_ARRAY_COUNT(Offsets);
	if (Reading.HitCount < 3)
	{ Reading.FailureReason=TEXT("광선 명중 3개 미만"); return false; }
	const FVector Estimated = CenterSum/Reading.HitCount;
	double SquaredError = 0;
	for (const FVector& Candidate : CenterSamples)
		SquaredError += FVector::DistSquared(Candidate,Estimated);
	Reading.ResidualMm = 10.0*FMath::Sqrt(SquaredError/Reading.HitCount);
	if (Reading.ResidualMm > MaxResidualMm)
	{ Reading.FailureReason=TEXT("측정 중심 잔차 1mm 초과"); return false; }
	Reading.MeasuredWorld = FTransform(BodyAtStart.GetRotation(),Estimated);
	Reading.bValid = true;
	return true;
}

bool AWheelAssemblyCell::ScanFrontLeftWheelSize()
{
	if (!IsValid(FrontLeftHubProxy)) return false;
	constexpr double NominalRadiusCm=33.0;
	FrontLeftHubProxy->SetSphereRadius(NominalRadiusCm*FrontLeftWheelSizeScale);
	const FVector Center=GetWheelWorld(0).GetLocation();
	FrontLeftHubProxy->SetWorldLocation(Center);
	const FVector Axis=BodyAtStart.TransformVectorNoScale(FVector(1,0,0)).GetSafeNormal();
	FHitResult NearHit,FarHit;
	const bool Near=FrontLeftHubProxy->LineTraceComponent(NearHit,Center-Axis*100,Center+Axis*100,FCollisionQueryParams(SCENE_QUERY_STAT(WheelSizeLaser),false));
	const bool Far=FrontLeftHubProxy->LineTraceComponent(FarHit,Center+Axis*100,Center-Axis*100,FCollisionQueryParams(SCENE_QUERY_STAT(WheelSizeLaser),false));
	WheelSizeNearHitWorld=Near?NearHit.ImpactPoint:FVector::ZeroVector;
	WheelSizeFarHitWorld=Far?FarHit.ImpactPoint:FVector::ZeroVector;
	FrontLeftWheelMeasuredDiameterCm=Near&&Far?FVector::Distance(NearHit.ImpactPoint,FarHit.ImpactPoint):0;
	FrontLeftWheelSizeMeasuredAt=SimulationTimeSeconds;
	bFrontLeftWheelSizePassed=Near&&Far&&FMath::Abs(FrontLeftWheelMeasuredDiameterCm-2*NominalRadiusCm)<=1.0;
	return bFrontLeftWheelSizePassed;
}

FTransform AWheelAssemblyCell::GetPickupWorld(int32 Index) const
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || Index < 0 || Index > 3) return FTransform::Identity;
	Index=PartIndex(Index);
	const double X = MountCentersCm.Num() == 4 ? MountCentersCm[Index].X : (Index % 2 == 0 ? FrontXcm : RearXcm);
	const double Y = Index < 2 ? -PickupHalfTrackCm : PickupHalfTrackCm;
	// Wheel racks stay at the station while the vehicle travels toward them.
	FTransform P=FTransform(FRotator::ZeroRotator, FVector(X, Y, MountCentersCm.Num() == 4 ? MountCentersCm[Index].Z : HubHeightCm)) * (bDockReferenceReady ? DockBodyWorld : Assembly->GetBodyFrame()->GetComponentTransform());
    P.AddToTranslation(SupplyOffsetsCm.FindRef(Index));return P;
}

FTransform AWheelAssemblyCell::GetWheelWorld(int32 Index) const
{
	return GetPhysicalPartWorld(PartIndex(Index));
}

FVector AWheelAssemblyCell::GetDroppedWheelLocation() const
{return DroppedWheelCollision?DroppedWheelCollision->GetComponentLocation():FVector::ZeroVector;}

bool AWheelAssemblyCell::IsRecoveryWorkerVisible() const
{return RecoveryWorkerVisual&&RecoveryWorkerHeadVisual&&RecoveryWorkerVisual->GetStaticMesh()
	&&RecoveryWorkerHeadVisual->GetStaticMesh()&&RecoveryWorkerVisual->IsVisible()&&RecoveryWorkerHeadVisual->IsVisible();}

FTransform AWheelAssemblyCell::GetPhysicalPartWorld(int32 Index) const
{return WheelRoots.IsValidIndex(Index)&&WheelRoots[Index]?WheelRoots[Index]->GetComponentTransform():FTransform::Identity;}
bool AWheelAssemblyCell::SetPhysicalPartWorld(int32 Index,const FTransform& Pose)
{if(!WheelRoots.IsValidIndex(Index)||!WheelRoots[Index]||!Pose.IsValid())return false;WheelRoots[Index]->SetWorldTransform(Pose);return true;}
bool AWheelAssemblyCell::IsWheelVisible(int32 Index) const
{
	Index=PartIndex(Index);
	return TireVisuals.IsValidIndex(Index) && IsValid(TireVisuals[Index]) && TireVisuals[Index]->IsVisible();
}

void AWheelAssemblyCell::InjectLeftWheelMissing()
{
	SetLeftWheelSupplyVisible(0, false);
}

void AWheelAssemblyCell::InjectLeftWheelDisplacement(FVector OffsetCm)
{
	DisplaceLeftWheel(0, OffsetCm);
}

void AWheelAssemblyCell::SetLeftWheelSupplyVisible(int32 Index, bool bVisible)
{
	if (Index < 0 || Index > 1 || (FastenedWheels.IsValidIndex(Index) && FastenedWheels[Index])) return;
	Index=PartIndex(Index);
	if (TireVisuals.IsValidIndex(Index)) TireVisuals[Index]->SetVisibility(bVisible);
	if (HubVisuals.IsValidIndex(Index)) HubVisuals[Index]->SetVisibility(bVisible);
}

void AWheelAssemblyCell::DisplaceLeftWheel(int32 Index, FVector OffsetCm)
{
	if (Index >= 0 && Index <= 1 && WheelRoots.IsValidIndex(Index) && WheelRoots[Index]
		&& !OffsetCm.ContainsNaN() && !(FastenedWheels.IsValidIndex(Index) && FastenedWheels[Index]))
		WheelRoots[PartIndex(Index)]->AddWorldOffset(OffsetCm);
}

FString AWheelAssemblyCell::GetLeftSensorSummary() const
{
	if (!bSensorDrivenLeftRobot || !LeftSensors) return TEXT("왼팔 센서: 비활성");
	const double Now = SimulationTimeSeconds;
	const int32 Index = Lanes[0].WheelIndex;
	const bool PoseFresh = LeftSensors->PickupPose.Fresh(Now, LeftSensors->MaxAge);
	const bool GripFresh = LeftSensors->Gripped.Fresh(Now, LeftSensors->MaxAge);
	return FString::Printf(TEXT("왼팔 %s · 타이어 %d · 공급위치 %s · 위치 %s · 파지 %s · 장착 측정 %s · 재시도 %d"),
		*GetLaneStage(true), Index + 1,
		LeftSensors->Presence.Fresh(Now, LeftSensors->MaxAge) ? (LeftSensors->Presence.bDetected ? TEXT("있음") : TEXT("없음")) : TEXT("신호 지연"),
		PoseFresh ? TEXT("유효") : TEXT("무효/지연"), GripFresh ? (LeftSensors->Gripped.bDetected ? TEXT("확인") : TEXT("미확인")) : TEXT("지연"),
		LeftSensors->MountPose.Fresh(Now, LeftSensors->MaxAge) ? TEXT("유효") : TEXT("무효/지연"), Lanes[0].GripAttempts)
		+ FString::Printf(TEXT(" · 경로 재계산 %d · 파지 재시도 %d"), LeftPickupReplans, LeftGripRetries)
        + FString::Printf(TEXT("\n네 휠 공통 접촉 물리 %s · 파지 사고 대상 %d"),bUseContactPhysics?TEXT("켜짐"):TEXT("꺼짐"),GripIncidentWheelIndex+1)
		+ (bLaserScanFrontLeftHub ? FString::Printf(TEXT("\n레이저 도착 %s · 장착 직전 %s · 장착 후 %s\n광선 %d/5 · 잔차 %.2fmm · 실패 %s"),
			FrontLeftDockScan.bValid ? TEXT("유효") : TEXT("대기/실패"),
			FrontLeftPreMountScan.bValid ? TEXT("유효") : TEXT("대기/실패"),
			FrontLeftFinalWheelScan.bValid ? TEXT("유효") : TEXT("대기/실패"),
			FrontLeftHubScan.HitCount,FrontLeftHubScan.ResidualMm,*FrontLeftHubScan.FailureReason) : FString());
}

FString AWheelAssemblyCell::GetLaserInspectionSummary() const
{
	if (!bLaserScanFrontLeftHub) return TEXT("레이저 검사 꺼짐 · 왼쪽 앞 휠만 지원");
	const FWheelHubScanReading* Readings[]={&FrontLeftDockScan,&FrontLeftPreMountScan,&FrontLeftFinalWheelScan};
	const TCHAR* Names[]={TEXT("① 차량 도착/허브"),TEXT("② 부착 직전/허브"),TEXT("③ 부착 후/휠")};
	FString Result=TEXT("왼쪽 앞 휠 · 광선 5개 중 3개 이상, 잔차 1mm 이하\n");
	for (int32 I=0;I<3;++I)
	{
		const auto& R=*Readings[I];
		Result+=FString::Printf(TEXT("%s: %s | %.2fs | %d/5 | 잔차 %.2fmm"),Names[I],
			R.MeasuredAt<0?TEXT("대기"):(R.bValid?TEXT("통과"):TEXT("실패")),R.MeasuredAt,R.HitCount,R.ResidualMm);
		if (!R.FailureReason.IsEmpty()) Result+=TEXT(" | ")+R.FailureReason;
		Result+=TEXT("\n");
	}
	if (FrontLeftDockScan.bValid&&FrontLeftPreMountScan.bValid)
		Result+=FString::Printf(TEXT("허브 이동 %.2f/5.00mm\n"),10*FVector::Distance(FrontLeftDockScan.MeasuredWorld.GetLocation(),FrontLeftPreMountScan.MeasuredWorld.GetLocation()));
	if (FrontLeftPreMountScan.bValid&&FrontLeftFinalWheelScan.bValid)
		Result+=FString::Printf(TEXT("최종 휠 중심 오차 %.2f/%.2fmm"),10*FVector::Distance(FrontLeftPreMountScan.MeasuredWorld.GetLocation(),FrontLeftFinalWheelScan.MeasuredWorld.GetLocation()),QualityToleranceMm);
	return Result;
}

FTransform AWheelAssemblyCell::OffsetOutward(const FTransform& Pose, int32 Side, double Centimeters) const
{
	FTransform Result = Pose;
	Result.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, GeometricSide(Side) == 0 ? -Centimeters : Centimeters, 0)));
	return Result;
}

bool AWheelAssemblyCell::ValidateSetup()
{
	if(!PrepareTaskLanes())return false;
	if (MountCentersCm.Num() != 0 && MountCentersCm.Num() != 4)
	{ Status = TEXT("Provide all four measured wheel centers or leave the array empty."); return false; }
	for (const FVector& Center : MountCentersCm) if (Center.ContainsNaN())
	{ Status = TEXT("Wheel centers must be finite."); return false; }
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || !IsValid(LeftRobot) || !IsValid(RightRobot) || (!UsesTaskBindings()&&LeftRobot == RightRobot))
	{ Status = TEXT("Assign one vehicle and two distinct calibrated robots."); return false; }
	const double Positive[] = {FrontXcm, HubHalfTrackCm, HubHeightCm, PickupHalfTrackCm, PositionToleranceMm,
		OrientationToleranceDeg, QualityToleranceMm, QualityOrientationDeg, SettleSeconds, FasteningSeconds,
		StageTimeoutSeconds, FixedStepSeconds, MaxPickupDisplacementMm, MaxPickupRotationDeg,
		LeftRecoveryTimeoutSeconds, LeftRecoveryStableSeconds};
	for (double Value : Positive) if (!FMath::IsFinite(Value) || Value <= 0)
	{ Status = TEXT("Wheel geometry, control times and tolerances must be positive finite values."); return false; }
	if (!FMath::IsFinite(RearXcm) || FixedStepSeconds > 0.02 || FaultWheelIndex < 0 || FaultWheelIndex > 3
		|| MaxAutoGripAttempts < 2
		|| DemoChassisShiftCm.ContainsNaN() || DemoChassisShiftCm.Size() > 5.0
		|| SensorBiasMm.ContainsNaN() || InitialOffsetMm.ContainsNaN() || InitialRotationDeg.ContainsNaN()
		|| !LeftTCPInWheel.IsValid() || !RightTCPInWheel.IsValid()
		|| !LeftTCPInWheel.GetScale3D().Equals(FVector::OneVector, 1.e-6)
		|| !RightTCPInWheel.GetScale3D().Equals(FVector::OneVector, 1.e-6))
	{ Status = TEXT("Invalid wheel experiment or grasp calibration."); return false; }
	BodyAtStart = Assembly->GetBodyFrame()->GetComponentTransform();
	if (!BodyAtStart.IsValid() || !BodyAtStart.GetScale3D().Equals(FVector::OneVector, 1.e-6))
	{ Status = TEXT("Vehicle body must have a valid rigid transform."); return false; }
	for (int32 Side = 0; Side < LaneCount(); ++Side)
	{
		if(!LaneEnabled(Side))continue;
		RobotAtStart[Side] = RobotFor(Side)->GetActorTransform();
		if (!RobotAtStart[Side].IsValid() || !RobotAtStart[Side].GetScale3D().Equals(FVector::OneVector, 1.e-6))
		{ Status = TEXT("Robot base must have a valid rigid transform."); return false; }
	}
	return true;
}

bool AWheelAssemblyCell::ResetCycle()
{
    for(auto Arm:{LeftRobot.Get(),RightRobot.Get()})if(IsValid(Arm)){Arm->ToolReactionForceN=Arm->ToolReactionTorqueNm=FVector::ZeroVector;Arm->bTorqueDynamics=false;}
    for(auto Arm:TaskRobots)if(IsValid(Arm)){Arm->ToolReactionForceN=Arm->ToolReactionTorqueNm=FVector::ZeroVector;Arm->bTorqueDynamics=false;}
    if(LeftRobot)WheelGripperGeometry::Configure(*LeftRobot);
    if(RightRobot)WheelGripperGeometry::Configure(*RightRobot);
    for(auto R:TaskRobots)if(IsValid(R))WheelGripperGeometry::Configure(*R);
	if (LeftRobot) LeftRobot->Halt();
	if (RightRobot) RightRobot->Halt();
    for(auto R:TaskRobots)if(IsValid(R))R->Halt();
	bRunning = false;
	bTransporting = false;
	if (bStationCaptured && bEnableTransport && IsValid(Assembly) && IsValid(TransportCarriage))
	{
		Assembly->SetActorTransform(VehicleAtStation);
		TransportCarriage->SetActorTransform(CarriageAtStation);
	}
	if (!ValidateSetup() || !LeftRobot->InitializeArm() || !RightRobot->InitializeArm())
	{ Outcome = EWheelOutcome::Blocked; Status += TEXT(" Robot calibration failed or references missing."); return false; }
    DockBodyWorld=BodyAtStart;bDockReferenceReady=true;
	for(int32 S=0;S<LaneCount();++S)if(LaneEnabled(S)&&!RobotFor(S)->InitializeArm())return false;

	if (bEnableTransport)
	{
		if (!IsValid(TransportCarriage) || TransportCarriage == Assembly || TransportCarriage == LeftRobot || TransportCarriage == RightRobot
			|| TransportStartOffsetCm.ContainsNaN() || TransportStartOffsetCm.IsNearlyZero()
			|| !FMath::IsFinite(TransportSeconds) || TransportSeconds <= 0
			|| !FMath::IsFinite(DockSettleSeconds) || DockSettleSeconds < 0)
		{ Outcome = EWheelOutcome::Blocked; Status = TEXT("Invalid transport carriage, travel or docking settings."); return false; }
		VehicleAtStation = Assembly->GetActorTransform();
		CarriageAtStation = TransportCarriage->GetActorTransform();
		bStationCaptured = true;
	}
    if(!CheckWheelVisuals(this,WheelRoots,Assembly,true,Status)){Outcome=EWheelOutcome::Blocked;return false;}
	MountedCount = 0; SimulationTimeSeconds = 0; Accumulator = 0;
	bLeftWheelSlipFall = false; LeftWheelSlipFallSeconds = 0; LeftWheelSlipVelocityCmps = FVector::ZeroVector;
	SlipRecoveryPhase=0;SlipRecoveryPhaseSeconds=0;SlipFailureMassKg=0;bSlipFailureLowFriction=false;
	bDroppedWheelHitSurface=false;SlipRecoveryWorkerLocation=FVector::ZeroVector;
	bSlipFailureGripDegraded=false;SlipFailureTravelCm=0;SlipFailureStage=EWheelStage::Idle;
	GripDegradeStartedAt=-1;bGripDegradeSpent=false;LeftGripFriction=WheelGrip::NormalFriction;
	LaserScanRetries=0;LaserTransientBlocksRemaining=FMath::Max(0,LaserTransientBlockScans);
	DroppedWheelCollision->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	DroppedTireVisual->SetVisibility(false);DroppedHubVisual->SetVisibility(false);
	RecoveryWorkerVisual->SetVisibility(false);
	RecoveryWorkerHeadVisual->SetVisibility(false);
	for(int32 Side=0;Side<LaneCount();++Side)if(IsValid(RobotFor(Side)))RobotFor(Side)->SetDroppedWheelCollision(false);
	FrontLeftHubScan = FWheelHubScanReading();
	FrontLeftDockScan = FrontLeftPreMountScan = FrontLeftFinalWheelScan = FWheelHubScanReading();
	bDemoChassisShiftApplied=false; HubRecheckCount=0; DockToPreMountShiftMm=0;
    bHubRecheckPending=false;PendingChassisShift=FVector::ZeroVector;
	CarriageAfterDemoShift=CarriageAtStation;
	FrontLeftWheelMeasuredDiameterCm = 0; bFrontLeftWheelSizePassed = false;
	FrontLeftWheelSizeMeasuredAt=-1;LeftGripMeasuredAt=-1;
    for(int I=0;I<4;++I){PartPhysics[I]=FAssemblyContactBody();GripObservations[I]=FWheelGripObservation();SecondaryFalling[I]=false;}
    DroppedWheelIndex=RecoveryLane=0;
	WheelSizeNearHitWorld = WheelSizeFarHitWorld = FVector::ZeroVector;
	bFrontLeftWheelRejecting = bFrontLeftWheelRejected = false;
	WheelRejectElapsedSeconds = 0;
	Incidents.Reset();
	WheelReplacePhase = 0; WheelReplacements = 0; WheelReplaceSeconds = 0; RejectedWheelDiameterCm = 0;
	RejectedWheelInstanceId.Reset(); SpareWheelInstanceId.Reset();
	FrontLeftActiveWheelInstanceId=ProductInstanceId.IsEmpty()?TEXT(""):ProductInstanceId+TEXT("/")+GetFrontLeftPartId()+TEXT("/initial");
	WheelSpareStartWorld = FVector::ZeroVector;
	DroppedTireVisual->SetRelativeScale3D(FVector(0.66,0.66,0.16));
	DroppedHubVisual->SetRelativeScale3D(FVector(0.37,0.37,0.025));
	WheelRejectStartWorld = WheelRejectTargetWorld = FVector::ZeroVector;
	PreviousPickupWorld = ReplannedPickupWorld = FVector::ZeroVector;
	LeftPickupReplans = LeftGripRetries = 0;
	Outcome = EWheelOutcome::NotEvaluated;
	FailedWheelIndex=INDEX_NONE;FailedStage=EWheelStage::Idle;
	MountPositionErrorMm.Init(0, 4); MountOrientationErrorDeg.Init(0, 4); FastenedWheels.Init(false, 4);
	for (int32 Index = 0; Index < 4; ++Index)
	{
		FTransform Initial = GetPickupWorld(Index);
		if (Index == FaultWheelIndex)
		{
			Initial.AddToTranslation(BodyAtStart.TransformVectorNoScale(InitialOffsetMm / 10.0));
			Initial.SetRotation((BodyAtStart.GetRotation() * InitialRotationDeg.Quaternion()
				* BodyAtStart.GetRotation().Inverse() * Initial.GetRotation()).GetNormalized());
		}
		WheelRoots[PartIndex(Index)]->SetWorldTransform(Initial);
		TireVisuals[PartIndex(Index)]->SetRelativeScale3D(FVector(0.66,0.66,0.16)*(Index==0?FrontLeftWheelSizeScale:1.0));
		HubVisuals[PartIndex(Index)]->SetRelativeScale3D(FVector(0.37,0.37,0.025)*(Index==0?FrontLeftWheelSizeScale:1.0));
		const bool Visible = Index != FaultWheelIndex || bWheelPresent;
		TireVisuals[PartIndex(Index)]->SetVisibility(Visible); HubVisuals[PartIndex(Index)]->SetVisibility(Visible);
	}
	for (int32 Side = 0; Side < LaneCount(); ++Side) Lanes[Side] = FWheelLane();
	if (LeftSensors) LeftSensors->Reset();
	RefreshQuality();
	TransportTime = 0; TransportProgress = 0;
	if (bEnableTransport) SetTransportProgress(0);
	Status = TEXT("Ready: Enter starts both arms; Space stops; R resets.");
	return true;
}

bool AWheelAssemblyCell::StartCycle()
{
	if (bRunning || Outcome != EWheelOutcome::NotEvaluated) { Status = TEXT("Reset before starting a new cycle."); return false; }
    if(!CheckWheelVisuals(this,WheelRoots,Assembly,false,Status)){Fail(Status);return false;}
	if (!bEnableTransport && !ValidateSetup()) { Fail(Status); return false; }
	if (bEnableTransport && (!bStationCaptured || !IsValid(Assembly) || !IsValid(TransportCarriage)
		|| !Assembly->GetActorTransform().Equals(ExpectedVehicle, 1.e-3)
		|| !TransportCarriage->GetActorTransform().Equals(ExpectedCarriage, 1.e-3)))
	{ Fail(TEXT("Transport setup changed; reset before starting.")); return false; }
	if (!bAlignmentSensorValid) { Fail(TEXT("Alignment sensor invalid; both robots remain stopped.")); return false; }
	for (int32 Index = 0; Index < 4; ++Index)
	{
		if (!(bSensorDrivenLeftRobot && Index < 2) && Index == FaultWheelIndex && !bWheelPresent) { Fail(TEXT("Pickup blocked: wheel missing.")); return false; }
		if (bSensorDrivenLeftRobot && Index < 2) continue;
		const FTransform Actual = GetWheelWorld(Index), Expected = GetPickupWorld(Index);
		if (!Actual.IsValid() || !Actual.GetScale3D().Equals(FVector::OneVector, 1.e-6)
			|| FVector::Distance(Actual.GetLocation(), Expected.GetLocation()) * 10 > MaxPickupDisplacementMm)
		{ Fail(TEXT("Pickup blocked: wheel outside recovery window.")); return false; }
		if (FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Expected.GetRotation())) > MaxPickupRotationDeg)
		{ Fail(TEXT("Pickup blocked: reversed wheel or invalid grasp orientation.")); return false; }
	}
	SimulationTimeSeconds = 0; Accumulator = 0; bRunning = true;
	if (bEnableTransport)
	{
		bTransporting = true; TransportTime = 0;
		Status = TEXT("차체 운반 중 — 도킹 완료 후 타이어 조립을 시작합니다.");
		return true;
	}
	for (int32 Side = 0; Side < LaneCount(); ++Side) if (!BeginWheel(Side)) return false;
	Status = TEXT("Both arms installing front wheels.");
	return true;
}

bool AWheelAssemblyCell::BeginWheel(int32 Side)
{
    TGuardValue<int32> DiagnosticLane(EvaluatingLane,Side);
	FWheelLane& Lane = Lanes[Side];
    if(!LaneEnabled(Side)){Lane.Stage=EWheelStage::Complete;return true;}
    Lane.WheelIndex = UsesTaskBindings()?TaskQueue[Side][Lane.Completed]:Side * 2 + Lane.Completed;
    if (Lane.WheelIndex == 0 && bLaserScanFrontLeftHub && !ScanFrontLeftHub())
    { Fail(TEXT("왼쪽 앞 허브 레이저 스캔 실패 · ")+FrontLeftHubScan.FailureReason); return false; }
    if (Lane.WheelIndex == 0 && bLaserScanFrontLeftHub) FrontLeftDockScan=FrontLeftHubScan;
    auto* Sensors=SensorsFor(Side);
    // Rear calibration is an IK seed, not a mandatory physical detour.
    RobotFor(Side)->PreferredIKSeedDegrees=Lane.WheelIndex%2==1
        ?(UsesTaskBindings()?RearTransit[Lane.WheelIndex]:(Side==0?LeftRearApproachAnglesDegrees:RightRearApproachAnglesDegrees)):TArray<double>();
	Lane.bHeld = false; Lane.bFastened = false; Lane.Corrections = 0;Lane.ContactGrip.Reset();
	Lane.GripAttempts = 0; Lane.bGripCommanded = false; Lane.GripCommandTime = 0;
	if (SensorDrivenLane(Side))
	{
		if (!Sensors) { Fail(TEXT("Left robot sensor bank missing.")); return false; }
		Sensors->CommandGrip(false);
		Sensors->Sample(SimulationTimeSeconds, this, Lane.WheelIndex);
		if (!Sensors->Presence.Fresh(SimulationTimeSeconds, Sensors->MaxAge) || !Sensors->Presence.bDetected)
		{
			if (bAutoRecoverLeftRobot) { WaitForLeftSensor(Side,EWheelStage::WaitSupply, TEXT("Left robot: waiting for wheel supply")); return true; }
			Fail(TEXT("Left robot: pickup presence sensor reports no wheel.")); return false;
		}
		if (!Sensors->PickupPose.Fresh(SimulationTimeSeconds, Sensors->MaxAge))
		{
			if (bAutoRecoverLeftRobot) { WaitForLeftSensor(Side,EWheelStage::WaitPose, TEXT("Left robot: waiting for pickup measurement")); return true; }
			Fail(TEXT("Left robot: pickup pose measurement unavailable.")); return false;
		}
		Lane.Pickup = Sensors->PickupPose.Pose;
	}
	else Lane.Pickup = GetWheelWorld(Lane.WheelIndex);
	// Apply the same supply inspection to normal and defective front-left wheels.
	// Presence and pickup pose must be observed first so a missing wheel cannot
	// pass inspection through the independent collision proxy.
	if (Lane.WheelIndex == 0 && !ScanFrontLeftWheelSize())
	{
		if (FrontLeftWheelMeasuredDiameterCm>0)
		{
			WheelRejectStartWorld=GetWheelWorld(0).GetLocation();
			WheelRejectTargetWorld=WheelRejectStartWorld
				+BodyAtStart.TransformVectorNoScale(FVector(-80,-160,0));
			if (bReplaceRejectedWheel && bAutoRecoverLeftRobot && SensorDrivenLane(Side) && WheelReplacements==0)
			{
				// One replacement per cycle. The lane waits in WaitSupply; it is released only by
				// the supply sensor and a passing size scan of the spare, not by this sequence.
				RejectedWheelDiameterCm=FrontLeftWheelMeasuredDiameterCm;
				OpenIncident(TEXT("wheel_size_reject"),0,Lane.Stage,TEXT("휠 지름"),TEXT("cm"),
					FrontLeftWheelMeasuredDiameterCm,66.0,TEXT("파지 금지 · 격리 후 대체 휠 공급"));
				bFrontLeftWheelRejecting=true; bFrontLeftWheelRejected=false;
				WheelReplacePhase=1; WheelReplaceSeconds=0;
				WaitForLeftSensor(Side,EWheelStage::WaitSupply,
					FString::Printf(TEXT("휠 규격 불합격 %.1fcm / 기준 66.0cm ± 1.0cm · 격리 후 대체품 대기"),RejectedWheelDiameterCm));
				return true;
			}
			bFrontLeftWheelRejecting=true;
			bFrontLeftWheelRejected=false;
			WheelRejectElapsedSeconds=0;
		}
		OpenIncident(TEXT("wheel_size_reject"),0,Lane.Stage,TEXT("휠 지름"),TEXT("cm"),
			FrontLeftWheelMeasuredDiameterCm,66.0,TEXT("파지 금지 · 격리 · 라인 중단"));
		Fail(FString::Printf(TEXT("휠 규격 검사 실패 · 레이저 측정 지름 %.1fcm / 기준 66.0cm ± 1.0cm · 불합격품 격리"),FrontLeftWheelMeasuredDiameterCm));
		return false;
	}
	return MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Lane.Pickup, Side, 25));
}

void AWheelAssemblyCell::SetStage(int32 Side, EWheelStage Next, const FString& Message)
{
	FWheelLane& Lane = Lanes[Side];
	Lane.Stage = Next; Lane.StageTime = 0; Lane.StableTime = 0;
	Status = FString::Printf(TEXT("%s arm: %s"), Side == 0 ? TEXT("Left") : TEXT("Right"), *Message);
}

bool AWheelAssemblyCell::MoveTo(int32 Side, EWheelStage Next, const FTransform& WheelTarget)
{
    TGuardValue<int32> DiagnosticLane(EvaluatingLane,Side);
	FWheelLane& Lane = Lanes[Side];
	Lane.TargetWheel = WheelTarget;
	Lane.TargetTCP = (Lane.bHeld ? Lane.WheelInTCP.Inverse() : TCPFor(Side)) * WheelTarget;
    RobotFor(Side)->SetMotionPart(WheelRoots[PartIndex(Lane.WheelIndex)],Lane.bHeld);
    const bool Linear=Next==EWheelStage::Align||Next==EWheelStage::Retract
        ||(Next==EWheelStage::ApproachMount&&Lane.Stage==EWheelStage::Align);
    if(Next==EWheelStage::Retract)Lane.TargetTCP.SetRotation(RobotFor(Side)->GetTCPWorld().GetRotation());
	if (!(Linear?RobotFor(Side)->CommandLinearTCPWorld(Lane.TargetTCP):RobotFor(Side)->CommandTCPWorld(Lane.TargetTCP)))
	{
		Fail(FString::Printf(TEXT("%s arm IK: %s"), Side == 0 ? TEXT("Left") : TEXT("Right"), *RobotFor(Side)->LastStatus));
		return false;
	}
	SetStage(Side, Next, StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Next)));
	return true;
}

void AWheelAssemblyCell::WaitForLeftSensor(int32 Side,EWheelStage WaitStage, const FString& Message)
{
	RobotFor(Side)->Halt();
	SensorsFor(Side)->CommandGrip(false);
	Lanes[Side].bGripCommanded = false;
	if (WheelReplacePhase == 0)
	{
		// The size-reject path records its own row before waiting.
		if (WaitStage == EWheelStage::WaitSupply)
			OpenIncident(TEXT("supply_missing"),Lanes[Side].WheelIndex,Lanes[Side].Stage,TEXT("공급 위치 휠 검출"),TEXT(""),
				0,1,TEXT("로봇 대기 · 보충 후 도착 센서 확인"));
		else
		{
			const auto* Bank = SensorsFor(Side);
			const double Age = Bank->PickupPose.MeasuredAt >= 0 ? SimulationTimeSeconds - Bank->PickupPose.MeasuredAt : 0;
			OpenIncident(TEXT("pickup_pose_stale"),Lanes[Side].WheelIndex,Lanes[Side].Stage,TEXT("위치 측정값 나이"),TEXT("s"),
				Age,Bank->MaxAge,TEXT("로봇 대기 · 새 측정값 수신 후 재개"));
		}
	}
	SetStage(Side, WaitStage, Message);
}

void AWheelAssemblyCell::Fail(const FString& Reason)
{
    FailedWheelIndex=EvaluatingLane>=0?Lanes[EvaluatingLane].WheelIndex:INDEX_NONE;
    FailedStage=EvaluatingLane>=0?Lanes[EvaluatingLane].Stage:EWheelStage::Idle;
	const FString Saved = Reason+FString::Printf(TEXT(" [wheel=%d stage=%d mass=%.0f contact=%d slipMass=%.0f recovery=%d]"),FailedWheelIndex,int(FailedStage),GripIncidentMassKg,PhysicalGripContacts(FailedWheelIndex),SlipFailureMassKg,SlipRecoveryPhase);
	if(EvaluatingLane<0)for(const FWheelLane& Lane:Lanes)if(Lane.Stage!=EWheelStage::Idle&&Lane.Stage!=EWheelStage::Complete&&Lane.Stage!=EWheelStage::Fault)
	{FailedWheelIndex=Lane.WheelIndex;FailedStage=Lane.Stage;break;}
	bRunning = false; bTransporting = false; Outcome = EWheelOutcome::Blocked;
	if (LeftRobot) LeftRobot->Halt();
	if (RightRobot) RightRobot->Halt();
    for(auto R:TaskRobots)if(IsValid(R))R->Halt();
	for (FWheelLane& Lane : Lanes) if (Lane.Stage != EWheelStage::Complete) Lane.Stage = EWheelStage::Fault;
	CloseIncidents(nullptr,TEXT("stopped"));
	OpenIncident(TEXT("safety_stop"),INDEX_NONE,EWheelStage::Fault,TEXT(""),TEXT(""),0,0,Saved,TEXT("stopped"));
	Status = Saved;
}

void AWheelAssemblyCell::StopCycle()
{
	if (!bRunning) return;
	bRunning = false; bTransporting = false; Outcome = EWheelOutcome::Stopped;
	LeftRobot->Halt(); RightRobot->Halt();
    for(auto R:TaskRobots)if(IsValid(R))R->Halt();
	for (FWheelLane& Lane : Lanes) if (Lane.Stage != EWheelStage::Complete) Lane.Stage = EWheelStage::Stopped;
	Status = TEXT("Stopped. Reset to restart; mounted wheels stay in place until reset.");
}

void AWheelAssemblyCell::RefreshQuality()
{
	if (MountPositionErrorMm.Num() != 4 || MountOrientationErrorDeg.Num() != 4) return;
	for (int32 Index = 0; Index < 4; ++Index)
	{
		const FTransform Mount = GetMountWorld(Index), Actual = GetWheelWorld(Index);
		MountPositionErrorMm[Index] = FVector::Distance(Actual.GetLocation(), Mount.GetLocation()) * 10;
		MountOrientationErrorDeg[Index] = FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Mount.GetRotation()));
	}
}

bool AWheelAssemblyCell::ResumeChassisLanes(int32 Side)
{
        for(int S=0;S<LaneCount();++S)if(S!=Side&&LaneEnabled(S))
        {
            auto& Other=Lanes[S];
            if(Other.Stage==EWheelStage::Idle||Other.Stage==EWheelStage::Complete
                ||Other.Stage==EWheelStage::WaitSupply||Other.Stage==EWheelStage::WaitPose)continue;
            auto Target=Other.TargetWheel;
            if(Other.Stage==EWheelStage::Transfer||Other.Stage==EWheelStage::ApproachMount)Target.AddToTranslation(PendingChassisShift);
            if(!MoveTo(S,Other.Stage,Target))return false;
        }
        bHubRecheckPending=false;
    return true;
}

bool AWheelAssemblyCell::AdvanceCarriedWheel(int32 Side,double Dt)
{
    auto& Lane=Lanes[Side];auto* Robot=RobotFor(Side);auto* Sensors=SensorsFor(Side);
    if(!IsValid(Robot))return false;
	if (Lane.bHeld && !Lane.bFastened)
    {
        FTransform Target = Lane.WheelInTCP * Robot->GetTCPWorld();
        const bool Physical = bUseContactPhysics;
        const bool Incident = bGripIncidentOverride && Lane.WheelIndex==GripIncidentWheelIndex;
        auto Properties=ContactPropertiesFor(Lane.WheelIndex);
        if (Physical)
        {

            double Friction = Incident && bLowFrictionGrip ? WheelGrip::LowFriction : Properties.Friction;
            const bool Degrading = Incident && bGripDegradesInTransit && !bGripDegradeSpent && !bLowFrictionGrip;
            if (Degrading)
            {
                // Onset is the Transfer stage, so the wheel is already lifted and moving between stations.
                if (GripDegradeStartedAt < 0 && Lane.Stage == EWheelStage::Transfer) GripDegradeStartedAt = SimulationTimeSeconds;
                if (GripDegradeStartedAt >= 0)
                    Friction = WheelGrip::DegradedFriction(SimulationTimeSeconds - GripDegradeStartedAt, GripDegradeSeconds);
            }
            if(Incident)LeftGripFriction = Friction;
            const FVector TargetVelocity=(Target.GetLocation()-Lane.LastCarried.GetLocation())/(100*Dt);
            Lane.DemandN=(Properties.MassKg*((TargetVelocity-Lane.GripVelocity)/Dt-FVector(0,0,-9.81))).Size();
            const FTransform Command=Target;
            Properties.Friction=Friction;
            if(!Lane.ContactGrip.Step(Command,Properties,Robot->WheelJaw,Dt,Target)){Fail(TEXT("휠 접촉 물리 무효"));return false;}
            // The floor must also support a wheel while jaws are still touching it.
            // Otherwise an overweight pickup can sink through the floor before detaching.
            if(GetWorld())
            {
                FCollisionQueryParams Params(SCENE_QUERY_STAT(HeldWheelContact),false,this);
                Params.AddIgnoredActor(Robot);if(Assembly)Params.AddIgnoredActor(Assembly);
                FHitResult Hit;
                if(GetWorld()->SweepSingleByChannel(Hit,Lane.LastCarried.GetLocation(),Target.GetLocation(),FQuat::Identity,ECC_PhysicsBody,
                    FCollisionShape::MakeSphere(ContactRadiusCm(Lane.WheelIndex)),Params))
                {
                    auto& Body=Lane.ContactGrip.Body;
                    const FVector N=(Hit.bStartPenetrating?Hit.Normal:Hit.ImpactNormal).GetSafeNormal();
                    Body.Pose.SetLocation(Hit.bStartPenetrating?Lane.LastCarried.GetLocation()+N*(Hit.PenetrationDepth+.001):Hit.Location+N*.001);
                    Body.ResolveImpact(N,-N*(ContactRadiusCm(Lane.WheelIndex)/100));Target=Body.Pose;
                }
            }
            Lane.GripVelocity=Lane.ContactGrip.Body.Velocity;
            Lane.CapacityN=Lane.ContactGrip.Body.GripFrictionCapacityN;
            Lane.AppliedForceN=Lane.ContactGrip.Body.GripForce.Size();
            Lane.SlipMm=FVector::Distance(Target.GetLocation(),Command.GetLocation())*10;
            Robot->bTorqueDynamics=true;Robot->ToolReactionForceN=-Lane.ContactGrip.Body.GripForce;
            Robot->ToolReactionTorqueNm=-Lane.ContactGrip.Body.GripTorque;Robot->ForcePointWorldCm=Target.GetLocation();
            if(Lane.WheelIndex==GripIncidentWheelIndex)LeftGripMeasuredAt=SimulationTimeSeconds;
            Lane.ContactGrip.Body.PartId=TaskReferences.IsValidIndex(Lane.WheelIndex)?TaskReferences[Lane.WheelIndex].PartId:FString::Printf(TEXT("wheel_%d"),PartIndex(Lane.WheelIndex));
            PartPhysics[Lane.WheelIndex]=Lane.ContactGrip.Body;
            GripObservations[Lane.WheelIndex]={SimulationTimeSeconds,Lane.DemandN,Lane.CapacityN,Lane.SlipMm};
        }
        WheelRoots[PartIndex(Lane.WheelIndex)]->SetWorldTransform(Target);
        if (Physical && Lane.ContactGrip.Closure>=1 && Lane.ContactGrip.Body.GripContacts==0)
        {
            if(SlipRecoveryPhase>0)
            {
                SecondaryFalling[Lane.WheelIndex]=true;Lane.bHeld=false;bDemoRecoveryEnabled=false;
                Robot->ToolReactionForceN=Robot->ToolReactionTorqueNm=FVector::ZeroVector;
                Robot->SetMotionPart(WheelRoots[PartIndex(Lane.WheelIndex)],false);if(Sensors)Sensors->CommandGrip(false);
                OpenIncident(TEXT("grip_slip"),Lane.WheelIndex,Lane.Stage,TEXT("접촉 상실"),TEXT("개"),0,1,TEXT("동시 낙하 · 각 부품 물리 유지 · 작업자 개입 필요"));
                Fail(TEXT("복수 부품 파지 상실 · 자동 복구 차단"));return false;
            }
            DroppedWheelIndex=Lane.WheelIndex;RecoveryLane=Side;
            bLeftWheelSlipFall = true;
            LeftWheelSlipFallSeconds = 0;
            LeftWheelSlipVelocityCmps = Lane.GripVelocity * 100.0;
            DroppedWheelOmega=Lane.ContactGrip.Body.Omega;DroppedWheelInertia=Lane.ContactGrip.Body.Inertia;
            DroppedContactStableSeconds=0;
            Lane.bHeld = false;
            Robot->ToolReactionForceN=Robot->ToolReactionTorqueNm=FVector::ZeroVector;
            Robot->SetMotionPart(WheelRoots[PartIndex(Lane.WheelIndex)],false);
            if (Sensors) Sensors->CommandGrip(false);
			SlipFailureMassKg=Properties.MassKg;bSlipFailureLowFriction=Incident&&bLowFrictionGrip;
			bSlipFailureGripDegraded=Incident&&bGripDegradesInTransit&&!bGripDegradeSpent&&!bLowFrictionGrip&&GripDegradeStartedAt>=0;
			SlipFailureStage=Lane.Stage;
			SlipFailureTravelCm=FVector::Distance(Target.GetLocation(),Lane.Pickup.GetLocation());
			SlipRecoveryPhase=1;SlipRecoveryPhaseSeconds=0;
			OpenIncident(TEXT("grip_slip"),Lane.WheelIndex,Lane.Stage,TEXT("파지 미끄러짐"),TEXT("mm"),
				Lane.SlipMm,25.0,TEXT("유한 접촉면 이탈 · 로봇 정지 · 작업자 회수 · 대체 휠 공급"));
			DroppedWheelCollision->SetSphereRadius(ContactRadiusCm(DroppedWheelIndex));
			DroppedWheelCollision->SetWorldTransform(Target);
			DroppedWheelCollision->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
			DroppedTireVisual->SetVisibility(true);DroppedHubVisual->SetVisibility(true);
			TireVisuals[PartIndex(DroppedWheelIndex)]->SetVisibility(false);HubVisuals[PartIndex(DroppedWheelIndex)]->SetVisibility(false);
			SlipRecoveryWorkerLocation=Target.GetLocation()+BodyAtStart.TransformVectorNoScale(FVector(-250,GeometricSide(RecoveryLane)==0?-200:200,0));
			SlipRecoveryWorkerLocation.Z=90;
			RecoveryWorkerVisual->SetWorldLocation(SlipRecoveryWorkerLocation);
			RecoveryWorkerHeadVisual->SetWorldLocation(SlipRecoveryWorkerLocation+FVector(0,0,85));
			RecoveryWorkerVisual->SetVisibility(true);RecoveryWorkerHeadVisual->SetVisibility(true);
			for(int32 RecoverySide=0;RecoverySide<LaneCount();++RecoverySide)
				if(IsValid(RobotFor(RecoverySide)))
				{RobotFor(RecoverySide)->Halt();RobotFor(RecoverySide)->SetDroppedWheelCollision(true);}
			Status=FString::Printf(TEXT("%s · %.0fkg · 필요 %.0fN / 한계 %.0fN · 휠 낙하, 로봇 안전정지, 작업자 호출"),
				bSlipFailureGripDegraded?TEXT("운반 중 마찰 저하"):bLowFrictionGrip?TEXT("저마찰"):TEXT("과중량"),Properties.MassKg,Lane.DemandN,Lane.CapacityN);
            return false;
        }
    }
    Lane.LastCarried=GetWheelWorld(Lane.WheelIndex);
    return true;
}

void AWheelAssemblyCell::StepLane(int32 Side, double Dt)
{
    TGuardValue<int32> DiagnosticLane(EvaluatingLane,Side);
	FWheelLane& Lane = Lanes[Side];
	if (!LaneEnabled(Side)||Lane.Stage == EWheelStage::Complete) return;
	ARobotArm* Robot = RobotFor(Side);
	const bool SensorDriven = SensorDrivenLane(Side);
    auto* Sensors=SensorsFor(Side);
	if (SensorDriven && (Lane.Stage == EWheelStage::WaitSupply || Lane.Stage == EWheelStage::WaitPose))
	{
		// While the cell's own reject/replace sequence runs, the rejected wheel may still be inside
		// the pickup zone. Hold the lane; the recovery timeout starts once the spare is delivered.
		if (WheelReplacePhase > 0 && Lane.WheelIndex == 0) { Lane.StageTime = 0; Lane.StableTime = 0; return; }
		Lane.StageTime += Dt;
		if (Lane.StageTime > LeftRecoveryTimeoutSeconds)
		{ Fail(TEXT("Left robot: supply or sensor recovery timed out.")); return; }
		const bool Ready = Sensors && Sensors->Presence.Fresh(SimulationTimeSeconds, Sensors->MaxAge)
			&& Sensors->Presence.bDetected && Sensors->PickupPose.Fresh(SimulationTimeSeconds, Sensors->MaxAge);
		Lane.StableTime = Ready ? Lane.StableTime + Dt : 0;
		if (Lane.StableTime < LeftRecoveryStableSeconds) return;
		const FTransform Measured = Sensors->PickupPose.Pose;
		const FTransform Expected = GetPickupWorld(Lane.WheelIndex);
		if (FVector::Distance(Measured.GetLocation(), Expected.GetLocation()) * 10 > MaxPickupDisplacementMm
			|| FMath::RadiansToDegrees(Measured.GetRotation().AngularDistance(Expected.GetRotation())) > MaxPickupRotationDeg)
		{ Fail(TEXT("Left robot: restored supply is outside pickup range.")); return; }
		Lane.Pickup = Measured;
		if (Lane.Stage==EWheelStage::WaitSupply && Lane.WheelIndex==0 && !ScanFrontLeftWheelSize())
		{ Fail(FString::Printf(TEXT("보충 휠 규격 검사 실패 · 측정 지름 %.1fcm / 기준 66.0cm ± 1.0cm"),FrontLeftWheelMeasuredDiameterCm)); return; }
		if (Lane.Stage==EWheelStage::WaitSupply && CountIncidents(TEXT("supply_missing"),TEXT("pending"))>0)
		{
			if (Lane.WheelIndex==0)
				OpenIncident(TEXT("supply_verified"),Lane.WheelIndex,Lane.Stage,TEXT("재검사 휠 지름"),TEXT("cm"),
					FrontLeftWheelMeasuredDiameterCm,66.0,TEXT("허용 65–67 cm · 공급 위치 센서 확인 후 작업 재개"),TEXT("observed"));
			else
				OpenIncident(TEXT("supply_verified"),Lane.WheelIndex,Lane.Stage,TEXT("공급 휠 검출"),TEXT(""),
					1,1,TEXT("공급 위치·자세 센서 확인 후 작업 재개"),TEXT("observed"));
		}
		CloseIncidents(TEXT("supply_missing"),TEXT("recovered"));
		CloseIncidents(TEXT("pickup_pose_stale"),TEXT("recovered"));
		CloseIncidents(TEXT("wheel_size_reject"),TEXT("recovered"));
		CloseIncidents(TEXT("spare_supplied"),TEXT("recovered"));
		MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Measured, Side, 25));
		return;
	}
	if (SensorDriven)
	{
		if (!Sensors || !Sensors->Presence.Fresh(SimulationTimeSeconds, Sensors->MaxAge) || !Sensors->Presence.bDetected)
		{
			if (bAutoRecoverLeftRobot && !Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip))
			{ WaitForLeftSensor(Side,EWheelStage::WaitSupply, TEXT("Left robot: supply missing; request replacement")); return; }
			Fail(TEXT("Left robot: wheel missing or presence reading stale.")); return;
		}
		if (!Sensors->PickupPose.Fresh(SimulationTimeSeconds, Sensors->MaxAge))
		{
			if (bAutoRecoverLeftRobot && !Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip))
			{ WaitForLeftSensor(Side,EWheelStage::WaitPose, TEXT("Left robot: sensor link stale; waiting for new measurement")); return; }
			Fail(TEXT("Left robot: pickup pose reading stale.")); return;
		}
		if (Lane.bHeld && !Lane.bFastened && (!Sensors->Gripped.Fresh(SimulationTimeSeconds, Sensors->MaxAge) || !Sensors->Gripped.bDetected))
		{ Fail(TEXT("Left robot: grip lost, both arms stopped.")); return; }
		if (!Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip)
			&& !Sensors->PickupPose.Pose.Equals(Lane.Pickup, 1.e-5))
		{
			const FTransform Measured = Sensors->PickupPose.Pose;
			const FTransform Expected = GetPickupWorld(Lane.WheelIndex);
			const double OffsetMm = FVector::Distance(Measured.GetLocation(), Expected.GetLocation()) * 10;
			const double AngleDeg = FMath::RadiansToDegrees(Measured.GetRotation().AngularDistance(Expected.GetRotation()));
			if (OffsetMm > MaxPickupDisplacementMm || AngleDeg > MaxPickupRotationDeg)
			{ Fail(TEXT("Left robot: measured wheel outside pickup recovery window.")); return; }
			PreviousPickupWorld = Lane.Pickup.GetLocation();
			ReplannedPickupWorld = Measured.GetLocation();
			Lane.Pickup = Measured; Lane.bGripCommanded = false; Sensors->CommandGrip(false); ++LeftPickupReplans;
			OpenIncident(TEXT("pickup_displaced"),Lane.WheelIndex,Lane.Stage,TEXT("공급 위치 변화"),TEXT("mm"),
				OffsetMm,MaxPickupDisplacementMm,TEXT("측정 위치로 접근 경로 재계산"),TEXT("recovered"));
			MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Measured, Side, 25));
			return;
		}
	}
	if(!Robot->IsMotionPlanning())Lane.StageTime += Dt;
	if (Lane.StageTime > StageTimeoutSeconds)
	{
		const FTransform Actual = Robot->GetTCPWorld();
		Fail(FString::Printf(TEXT("%s %s timed out: TCP %.2f mm / %.2f deg; joints settled %d."),
			Side == 0 ? TEXT("Left") : TEXT("Right"),
			*StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Lane.Stage)),
			FVector::Distance(Actual.GetLocation(), Lane.TargetTCP.GetLocation()) * 10,
			FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Lane.TargetTCP.GetRotation())),
			Robot->IsAtJointTarget() ? 1 : 0));
		return;
	}
	if (Lane.bHeld && !Lane.bFastened && !GetWheelWorld(Lane.WheelIndex).Equals(Lane.LastCarried, 1.e-5))
	{ Fail(TEXT("Held wheel moved externally.")); return; }
	if (!SensorDriven && !Lane.bHeld && (Lane.Stage == EWheelStage::ApproachPickup || Lane.Stage == EWheelStage::Grip)
		&& !GetWheelWorld(Lane.WheelIndex).Equals(Lane.Pickup, 1.e-5))
	{ Fail(TEXT("Wheel moved during pickup.")); return; }
	if (!Robot->StepControl(Dt)) { Fail(Robot->LastStatus); return; }
    if(!AdvanceCarriedWheel(Side,Dt))return;
	Lane.LastCarried = GetWheelWorld(Lane.WheelIndex);
    if(Lane.Stage==EWheelStage::Align&&Lane.bHeld
        &&FMath::RadiansToDegrees(Lane.LastCarried.GetRotation().AngularDistance(Lane.TargetWheel.GetRotation()))>OrientationToleranceDeg)
    {Fail(TEXT("삽입 중 휠 각도 이탈 · 체결 차단"));return;}
    if(Robot->IsMotionPlanning())
    {Lane.StableTime=0;Status=StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Lane.Stage))+TEXT(" · ")+Robot->LastStatus;return;}
	const FTransform CurrentTCP = Robot->GetTCPWorld();
	const double TCPErrorMm = FVector::Distance(CurrentTCP.GetLocation(), Lane.TargetTCP.GetLocation()) * 10;
	const double TCPErrorDeg = FMath::RadiansToDegrees(CurrentTCP.GetRotation().AngularDistance(Lane.TargetTCP.GetRotation()));
	// A large station-to-station joint transit may retain a tiny PID oscillation.
	// The following IK command settles the exact pickup TCP before contact.
	const bool Settled = Lane.Stage == EWheelStage::Reposition
		? TCPErrorMm <= 5 && TCPErrorDeg <= 0.5
		: TCPErrorMm <= PositionToleranceMm && TCPErrorDeg <= OrientationToleranceDeg && Robot->IsAtJointTarget();
	Lane.StableTime = Settled ? Lane.StableTime + Dt : 0;
	if (Lane.Stage == EWheelStage::Fasten)
	{
		if (!Settled) { Fail(TEXT("Wheel alignment lost during fastening.")); return; }
		if (Lane.StageTime >= FasteningSeconds)
		{
			Lane.bFastened = true; FastenedWheels[Lane.WheelIndex] = true; ++MountedCount;
            if(Lane.WheelIndex==0&&bDemoChassisShiftApplied)CloseIncidents(TEXT("chassis_shift"),TEXT("recovered"));
            if(Lane.Completed+1<(UsesTaskBindings()?TaskQueue[Side].Num():2))Lane.NextPickupStartedAt=SimulationTimeSeconds;
            // Preserve the measured seating residual in the moving hub frame.
            // During lift unloading the wheel must follow its hub, not stay at
            // a frozen body-relative Z and detach from the running gear.
			FastenedInBody[Lane.WheelIndex] = GetWheelWorld(Lane.WheelIndex).GetRelativeTransform(GetMountWorld(Lane.WheelIndex));
			SetStage(Side, EWheelStage::Release, TEXT("virtual fastening complete"));
		}
		return;
	}
	if (Lane.Stage == EWheelStage::Release)
	{
		if (SensorDriven) Sensors->CommandGrip(false);
		Lane.bHeld = false;
        Robot->ToolReactionForceN=Robot->ToolReactionTorqueNm=FVector::ZeroVector;
		if (Lane.WheelIndex==0 && bLaserScanFrontLeftHub)
		{
			if (!ScanFrontLeftHub(true)) { Fail(TEXT("장착 후 휠 레이저 검증 실패 · ")+FrontLeftFinalWheelScan.FailureReason); return; }
			const double ErrorMm=FVector::Distance(FrontLeftFinalWheelScan.MeasuredWorld.GetLocation(),
				FrontLeftPreMountScan.MeasuredWorld.GetLocation())*10;
			if (ErrorMm>QualityToleranceMm)
			{ Fail(FString::Printf(TEXT("장착 후 휠 중심 오차 %.2fmm · 허용 %.2fmm 초과"),ErrorMm,QualityToleranceMm)); return; }
		}
		MoveTo(Side, EWheelStage::Retract, OffsetOutward(GetWheelWorld(Lane.WheelIndex), Side, 25));
		return;
	}
	if (Lane.StableTime < SettleSeconds) return;
	switch (Lane.Stage)
	{
	case EWheelStage::ApproachPickup:
		MoveTo(Side, EWheelStage::Grip, Lane.Pickup); break;
	case EWheelStage::Grip:
	{
		if (SensorDriven)
		{
			if (!Lane.bGripCommanded)
			{
				Sensors->CommandGrip(true); Lane.bGripCommanded = true;
				Lane.GripCommandTime = SimulationTimeSeconds; ++Lane.GripAttempts;
				Status = TEXT("Left arm: gripper closed; waiting for grip sensor");
				break;
			}
			if (!Sensors->Gripped.Fresh(SimulationTimeSeconds, Sensors->MaxAge))
			{ Fail(TEXT("Left robot: grip reading stale.")); break; }
			if (!Sensors->Gripped.bDetected)
			{
				if (SimulationTimeSeconds - Lane.GripCommandTime >= .3)
				{
					if (Lane.GripAttempts >= (bAutoRecoverLeftRobot ? MaxAutoGripAttempts : 2))
						Fail(TEXT("Left robot: grip recovery attempts exhausted."));
					else
					{
						Sensors->CommandGrip(false);
						Lane.bGripCommanded = false;
						++LeftGripRetries;
						OpenIncident(TEXT("grip_unconfirmed"),Lane.WheelIndex,Lane.Stage,TEXT("파지 시도"),TEXT("회"),
							Lane.GripAttempts,bAutoRecoverLeftRobot ? MaxAutoGripAttempts : 2,TEXT("그리퍼 열고 후퇴 후 재접근"));
						if (MoveTo(Side,EWheelStage::ApproachPickup,OffsetOutward(Lane.Pickup,Side,25)))
							Status=TEXT("파지 미확인 · 그리퍼 열고 후퇴 후 재접근");
					}
				}
				break;
			}
		}
		const FTransform Contact = TCPFor(Side) * Lane.Pickup;
		if (FVector::Distance(Contact.GetLocation(), CurrentTCP.GetLocation()) * 10 > PositionToleranceMm
			|| FMath::RadiansToDegrees(Contact.GetRotation().AngularDistance(CurrentTCP.GetRotation())) > OrientationToleranceDeg)
		{ Fail(TEXT("Gripper contact mismatch.")); break; }
		Lane.WheelInTCP = Lane.Pickup.GetRelativeTransform(CurrentTCP);
        Lane.ContactGrip.Reset();
        if(Lane.NextPickupStartedAt>=0)Lane.NextPickupSeconds=SimulationTimeSeconds-Lane.NextPickupStartedAt;
        Lane.bHeld = true; Lane.LastCarried = Lane.Pickup;
		if (SensorDriven) CloseIncidents(TEXT("grip_unconfirmed"),TEXT("recovered"));
		FTransform Lift = Lane.Pickup;
		Lift.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, 0, 25)));
		MoveTo(Side, EWheelStage::Lift, Lift);
		break;
	}
	case EWheelStage::Lift:
	{
		FTransform Transfer = OffsetOutward(GetMountTargetWorld(Lane.WheelIndex), Side, 55);
		Transfer.AddToTranslation(BodyAtStart.TransformVectorNoScale(FVector(0, 0, 25)));
		MoveTo(Side, EWheelStage::Transfer, Transfer); break;
	}
	case EWheelStage::Transfer:
		if (Lane.WheelIndex==0 && bLaserScanFrontLeftHub)
		{
			if (bDemoChassisShiftBeforeMount && !bDemoChassisShiftApplied)
			{
                // Do not inject a displacement while another tool is inserted.
                // Let that lane finish/retract; the left lane holds outside.
                for(int S=0;S<LaneCount();++S)if(S!=Side&&LaneEnabled(S))
                    if(Lanes[S].Stage==EWheelStage::Align||Lanes[S].Stage==EWheelStage::Fasten
                        ||Lanes[S].Stage==EWheelStage::Release||Lanes[S].Stage==EWheelStage::Retract)return;
				const FVector Offset=BodyAtStart.TransformVectorNoScale(DemoChassisShiftCm);
                PendingChassisShift=Offset;bHubRecheckPending=true;
                for(int S=0;S<LaneCount();++S)if(S!=Side&&LaneEnabled(S))RobotFor(S)->Halt();
				Assembly->AddActorWorldOffset(Offset);
				if(bEnableTransport && IsValid(TransportCarriage)) TransportCarriage->AddActorWorldOffset(Offset);
				BodyAtStart=Assembly->GetBodyFrame()->GetComponentTransform();
				if(IsValid(TransportCarriage))CarriageAfterDemoShift=TransportCarriage->GetActorTransform();
				bDemoChassisShiftApplied=true;
			}
			const FVector DockCenter=FrontLeftDockScan.MeasuredWorld.GetLocation();
			if (!ScanFrontLeftHub()) { Fail(TEXT("장착 직전 허브 재스캔 실패 · ")+FrontLeftHubScan.FailureReason); break; }
			DockToPreMountShiftMm=FVector::Distance(DockCenter,FrontLeftHubScan.MeasuredWorld.GetLocation())*10;
            PendingChassisShift=FrontLeftHubScan.MeasuredWorld.GetLocation()-DockCenter;
			if(DockToPreMountShiftMm>5)
			{
				if(!bDemoChassisShiftApplied || HubRecheckCount>0)
				{ Fail(FString::Printf(TEXT("도킹 후 허브 위치 변화 %.2fmm · 재도킹 필요"),DockToPreMountShiftMm)); break; }
				++HubRecheckCount;
				OpenIncident(TEXT("chassis_shift"),0,Lane.Stage,TEXT("도킹 후 허브 이동"),TEXT("mm"),
					DockToPreMountShiftMm,5,TEXT("기존 목표 폐기 · 팔 후퇴 · 허브 재스캔 후 재접근"));
				MoveTo(Side,EWheelStage::RecheckHub,OffsetOutward(GetMountTargetWorld(0),Side,65));
				break;
			}
			FrontLeftPreMountScan=FrontLeftHubScan;
            if(bHubRecheckPending&&!ResumeChassisLanes(Side))return;
		}
		MoveTo(Side, EWheelStage::ApproachMount, OffsetOutward(GetMountTargetWorld(Lane.WheelIndex), Side, 25)); break;
	case EWheelStage::RecheckHub:
		if(!ScanFrontLeftHub()) { Fail(TEXT("차체 이동 후 허브 재스캔 실패 · ")+FrontLeftHubScan.FailureReason); break; }
		FrontLeftPreMountScan=FrontLeftHubScan;
        PendingChassisShift=FrontLeftPreMountScan.MeasuredWorld.GetLocation()-FrontLeftDockScan.MeasuredWorld.GetLocation();
        if(!ResumeChassisLanes(Side))return;
		MoveTo(Side,EWheelStage::ApproachMount,OffsetOutward(GetMountTargetWorld(0),Side,25));
		break;
	case EWheelStage::ApproachMount:
    {
        if(SensorDriven&&!Sensors->MountPose.Fresh(SimulationTimeSeconds,Sensors->MaxAge))
        {Fail(TEXT("삽입 전 휠 자세 측정 만료"));break;}
        FTransform Measured=SensorDriven?Sensors->MountPose.Pose:GetWheelWorld(Lane.WheelIndex);
        Measured.AddToTranslation(BodyAtStart.TransformVectorNoScale(SensorBiasMm/10.));
        const FTransform Outside=OffsetOutward(GetMountTargetWorld(Lane.WheelIndex),Side,25);
        const FVector Residual=Outside.GetLocation()-Measured.GetLocation();
        const double Angle=FMath::RadiansToDegrees(Outside.GetRotation().AngularDistance(Measured.GetRotation()));
        if(Residual.Size()*10>PositionToleranceMm||Angle>OrientationToleranceDeg)
        {
            if(++Lane.Corrections>8){Fail(TEXT("장착 바깥 휠 사전 정렬 실패"));break;}
            auto Corrected=Lane.TargetWheel;Corrected.AddToTranslation(Residual);
            Corrected.SetRotation((Outside.GetRotation()*Measured.GetRotation().Inverse()*Corrected.GetRotation()).GetNormalized());
            MoveTo(Side,EWheelStage::ApproachMount,Corrected);break;
        }
        auto Insert=Lane.TargetWheel;
        Insert.AddToTranslation(GetMountTargetWorld(Lane.WheelIndex).GetLocation()-Outside.GetLocation());
        MoveTo(Side,EWheelStage::Align,Insert);break;
    }
	case EWheelStage::Align:
	{
		if (SensorDriven && !Sensors->MountPose.Fresh(SimulationTimeSeconds, Sensors->MaxAge))
		{ Fail(TEXT("Left robot: mount pose reading stale.")); break; }
		const FVector Measured = (SensorDriven ? Sensors->MountPose.Pose : GetWheelWorld(Lane.WheelIndex)).GetLocation()
			+ BodyAtStart.TransformVectorNoScale(SensorBiasMm / 10.0);
		const FVector Residual = GetMountTargetWorld(Lane.WheelIndex).GetLocation() - Measured;
		if (Residual.Length() * 10 > PositionToleranceMm)
		{
			if (++Lane.Corrections > 8) { Fail(TEXT("Sensor alignment did not converge.")); break; }
            MoveTo(Side,EWheelStage::ApproachMount,OffsetOutward(Lane.TargetWheel,Side,25));break;
		}
		if (ControlMode == EWheelControlMode::IndependentVerification)
		{
			if (!bVerificationSensorValid) { Fail(TEXT("Independent verification sensor invalid; fastening blocked.")); break; }
			RefreshQuality();
			const FTransform Verified = GetWheelWorld(Lane.WheelIndex); // Ideal independent evaluator, not a second hardware sensor.
			const FTransform Mount = GetMountWorld(Lane.WheelIndex);
			const double VerifiedMm = FVector::Distance(Verified.GetLocation(), Mount.GetLocation()) * 10;
			const double VerifiedDeg = FMath::RadiansToDegrees(Verified.GetRotation().AngularDistance(Mount.GetRotation()));
			if (VerifiedMm > QualityToleranceMm || VerifiedDeg > QualityOrientationDeg)
			{ Fail(TEXT("Independent wheel alignment rejected; fastening blocked.")); break; }
		}
		SetStage(Side, EWheelStage::Fasten, TEXT("wheel aligned; virtual fastening")); break;
	}
	case EWheelStage::Retract:
		++Lane.Completed;
        if (Lane.Completed < (UsesTaskBindings()?TaskQueue[Side].Num():2))
        {
            if(Lane.NextPickupStartedAt<0)Lane.NextPickupStartedAt=SimulationTimeSeconds;
            BeginWheel(Side);
        }
		else SetStage(Side, EWheelStage::Complete, TEXT("two wheels installed"));
		break;
	case EWheelStage::Reposition:
		BeginWheel(Side); break;
	default: break;
	}
}

bool AWheelAssemblyCell::AdvanceSimulation(double DeltaSeconds)
{
	if (!FMath::IsFinite(DeltaSeconds) || DeltaSeconds < 0 || DeltaSeconds > 1 || FixedStepSeconds <= 0 || FixedStepSeconds > 0.02) return false;
	UpdateFastenedWheels();
	if (!bRunning && !HasPassiveWork()) return true;
    if(bRunning&&!CheckWheelVisuals(this,WheelRoots,Assembly,false,Status)){Fail(Status);return false;}
    Accumulator += DeltaSeconds;
    while (Accumulator + 1.e-10 >= FixedStepSeconds)
	{
		Accumulator -= FixedStepSeconds; SimulationTimeSeconds += FixedStepSeconds;
        AdvanceFrontLeftWheelReject(FixedStepSeconds);
        AdvanceSecondaryFalls(FixedStepSeconds);
        if(!bRunning)
        {
            for(int Side=0;Side<LaneCount();++Side)if(bUseContactPhysics&&Lanes[Side].bHeld&&!Lanes[Side].bFastened)
                AdvanceCarriedWheel(Side,FixedStepSeconds);
            AdvanceLeftWheelSlipFall(FixedStepSeconds);
            for(int32 I=0;I<LaneCount();++I) if(auto* S=SensorsFor(I)) S->Sample(SimulationTimeSeconds,this,Lanes[I].WheelIndex);
            RefreshQuality();
            continue;
        }
		if(SlipRecoveryPhase>0)
		{
            for(int Side=0;Side<LaneCount();++Side)if(Lanes[Side].bHeld&&!Lanes[Side].bFastened)AdvanceCarriedWheel(Side,FixedStepSeconds);
			AdvanceSlipRecovery(FixedStepSeconds);
			RefreshQuality();
			continue;
		}
		if (bSensorDrivenLeftRobot && LeftSensors && !bTransporting && Lanes[0].WheelIndex >= 0)
			LeftSensors->Sample(SimulationTimeSeconds, this, Lanes[0].WheelIndex);
		if (bTransporting)
		{
			if (!IsValid(Assembly) || !IsValid(TransportCarriage) || !IsValid(LeftRobot) || !IsValid(RightRobot)
				// Scene components may elide sub-micron moves at the ends of the
				// smooth trajectory. Allow 0.01 mm, below assembly tolerances.
				|| !Assembly->GetActorTransform().Equals(ExpectedVehicle, 1.e-3)
				|| !TransportCarriage->GetActorTransform().Equals(ExpectedCarriage, 1.e-3)
				|| (!UsesTaskBindings()&&!LeftRobot->GetActorTransform().Equals(RobotAtStart[0], 1.e-6))
				|| (!UsesTaskBindings()&&!RightRobot->GetActorTransform().Equals(RobotAtStart[1], 1.e-6)))
			{ Fail(TEXT("Vehicle, carriage or robot moved externally during transport.")); break; }
			TransportTime += FixedStepSeconds;
			const double T = FMath::Clamp(TransportTime / TransportSeconds, 0.0, 1.0);
			SetTransportProgress(T*T*T*(10 + T*(-15 + 6*T)));
			Status = T < 1 ? TEXT("차체 운반 중") : TEXT("차체 도킹 · 위치 안정화 중");
			RefreshQuality();
			if (TransportTime + 1.e-9 >= TransportSeconds + DockSettleSeconds)
			{
				bTransporting = false;
				for (int32 Side = 0; Side < LaneCount(); ++Side) if (!BeginWheel(Side)) break;
			}
			continue;
		}
		if (!IsValid(Assembly) || !Assembly->GetBodyFrame()
			|| !IsValid(LeftRobot) || !IsValid(RightRobot)
			|| (bEnableTransport && (!IsValid(TransportCarriage) || !TransportCarriage->GetActorTransform().Equals(
				bDemoChassisShiftApplied ? CarriageAfterDemoShift : CarriageAtStation, 1.e-3)))
			|| !Assembly->GetBodyFrame()->GetComponentTransform().Equals(BodyAtStart, bEnableTransport ? 1.e-3 : 1.e-6)
			|| (!UsesTaskBindings()&&!LeftRobot->GetActorTransform().Equals(RobotAtStart[0], 1.e-6))
			|| (!UsesTaskBindings()&&!RightRobot->GetActorTransform().Equals(RobotAtStart[1], 1.e-6)))
		{ Fail(TEXT("Vehicle or robot base moved during assembly.")); break; }
        if(WheelReplacePhase>0)AdvanceWheelReplacement(FixedStepSeconds);
        for(int32 S=0;S<LaneCount()&&bRunning&&SlipRecoveryPhase==0;++S)if(LaneEnabled(S))
        {
            if(!IsValid(RobotFor(S))||!RobotFor(S)->GetActorTransform().Equals(RobotAtStart[S],1.e-6))
            {Fail(TEXT("작업 로봇이 삭제되거나 실행 중 이동했습니다."));break;}
            if(UsesTaskBindings()&&Lanes[S].WheelIndex>=0)SensorsFor(S)->Sample(SimulationTimeSeconds,this,Lanes[S].WheelIndex);
            if(!bHubRecheckPending||Lanes[S].WheelIndex==0)StepLane(S,FixedStepSeconds);
        }
		RefreshQuality();
		if (bRunning && MountedCount==4 && Lanes[0].Stage == EWheelStage::Complete && Lanes[1].Stage == EWheelStage::Complete && (!UsesTaskBindings()||(Lanes[2].Stage==EWheelStage::Complete&&Lanes[3].Stage==EWheelStage::Complete)))
		{
			bRunning = false;
			bool Passed = MountedCount == 4;
			for (int32 Index = 0; Index < 4; ++Index)
				Passed &= FastenedWheels[Index] && MountPositionErrorMm[Index] <= QualityToleranceMm
					&& MountOrientationErrorDeg[Index] <= QualityOrientationDeg;
			Outcome = Passed ? EWheelOutcome::Pass : EWheelOutcome::QualityFail;
			Status = Passed ? TEXT("PASS: four wheels installed and checked.") : TEXT("QUALITY FAIL: check individual wheel mount errors.");
		}
	}
	return Outcome != EWheelOutcome::Blocked;
}

bool AWheelAssemblyCell::SelectScenario(EWheelScenario Preset, EWheelControlMode Mode)
{
	if (bRunning) return false;
	Scenario = Preset; ControlMode = Mode;
	SensorBiasMm = FVector::ZeroVector; InitialOffsetMm = FVector::ZeroVector;
	InitialRotationDeg = FRotator::ZeroRotator; FaultWheelIndex = 0;
	bWheelPresent = true; bAlignmentSensorValid = true; bVerificationSensorValid = true;
	switch (Preset)
	{
	case EWheelScenario::SensorBias: SensorBiasMm = FVector(0, 5, 0); break;
	case EWheelScenario::DisplacedWheel: InitialOffsetMm = FVector(20, -30, 10); InitialRotationDeg = FRotator(0, 2, 0); break;
	case EWheelScenario::UnreachableWheel: InitialOffsetMm = FVector(0, -100000, 0); break;
	case EWheelScenario::ReversedWheel: InitialRotationDeg = FRotator(0, 0, 180); break;
	case EWheelScenario::MissingWheel: bWheelPresent = false; break;
	default: break;
	}
	return ResetCycle();
}

FString AWheelAssemblyCell::GetLaneStage(bool bLeft) const
{
	if (bTransporting) return TEXT("차체 도착 대기");
	return StaticEnum<EWheelStage>()->GetNameStringByValue(int64(Lanes[bLeft ? 0 : 1].Stage));
}

void AWheelAssemblyCell::SetTransportProgress(double Progress)
{
	TransportProgress = Progress;
	const FVector Offset = BodyAtStart.TransformVectorNoScale(TransportStartOffsetCm) * (1 - Progress);
	ExpectedVehicle = VehicleAtStation; ExpectedVehicle.AddToTranslation(Offset);
	ExpectedCarriage = CarriageAtStation; ExpectedCarriage.AddToTranslation(Offset);
	Assembly->SetActorTransform(ExpectedVehicle);
	TransportCarriage->SetActorTransform(ExpectedCarriage);
}

void AWheelAssemblyCell::UpdateFastenedWheels()
{
	if (!Assembly || !Assembly->GetBodyFrame()) return;
	for (int32 I=0; I<4; ++I)
		if (FastenedWheels.IsValidIndex(I) && FastenedWheels[I])
			WheelRoots[PartIndex(I)]->SetWorldTransform(FastenedInBody[I] * GetMountWorld(I));
}
void AWheelAssemblyCell::AdvanceLeftWheelSlipFall(double DeltaSeconds)
{
	if(!bLeftWheelSlipFall||!DroppedWheelCollision||!GetWorld())return;
	const double Dt=DeltaSeconds;
    if(!FMath::IsFinite(Dt)||Dt<=0||Dt>0.02)return;
	LeftWheelSlipFallSeconds+=Dt;
	const FVector Start=GetWheelWorld(DroppedWheelIndex).GetLocation();
    FVector VelocityMps=LeftWheelSlipVelocityCmps/100.0;
    const FVector End=Start+100.0*ReducedPartPhysics::FreeFlightDisplacement(VelocityMps,Dt);
    LeftWheelSlipVelocityCmps=VelocityMps*100.0;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(DroppedWheelSweep),false,this);
	if(LeftWheelSlipFallSeconds<0.12)Params.AddIgnoredActor(RobotForWheel(DroppedWheelIndex));
	FHitResult Hit;
	if(GetWorld()->SweepSingleByChannel(Hit,Start,End,FQuat::Identity,ECC_PhysicsBody,
		FCollisionShape::MakeSphere(ContactRadiusCm(DroppedWheelIndex)),Params))
	{
		const FVector Normal=(Hit.bStartPenetrating?Hit.Normal:Hit.ImpactNormal).GetSafeNormal();
        DroppedWheelCollision->SetWorldLocation(Hit.bStartPenetrating
            ? Start+Normal*(Hit.PenetrationDepth+0.001) : Hit.Location+Normal*0.001);
        FAssemblyContactBody Impact;Impact.Pose=GetWheelWorld(DroppedWheelIndex);Impact.Mass=SlipFailureMassKg;
        Impact.Inertia=DroppedWheelInertia;Impact.Velocity=LeftWheelSlipVelocityCmps/100;Impact.Omega=DroppedWheelOmega;
        Impact.ResolveImpact(Normal,-Normal*(ContactRadiusCm(DroppedWheelIndex)/100));
        const double AngularDrop=.02*SlipFailureMassKg*9.81*(ContactRadiusCm(DroppedWheelIndex)/100)*Dt/DroppedWheelInertia.GetMin();
        DroppedWheelOmega=Impact.Omega*FMath::Max(0.,1-AngularDrop/FMath::Max(Impact.Omega.Size(),1.e-9));
        LeftWheelSlipVelocityCmps=100*Impact.Velocity;
        const bool Supported=Normal.Z>0.5 && LeftWheelSlipVelocityCmps.Size()<2.0;
        DroppedContactStableSeconds=Supported ? DroppedContactStableSeconds+Dt : 0;
		if(Normal.Z>0.5 && Hit.GetActor()!=RobotForWheel(DroppedWheelIndex))bDroppedWheelHitSurface=true;
	}
	else {DroppedWheelCollision->SetWorldLocation(End);DroppedContactStableSeconds=0;}
    // One authoritative part pose; collision proxy and render geometry follow it.
    WheelRoots[PartIndex(DroppedWheelIndex)]->SetWorldLocation(DroppedWheelCollision->GetComponentLocation());
    const double AngularSpeed=DroppedWheelOmega.Size();
    if(AngularSpeed>1.e-9)WheelRoots[PartIndex(DroppedWheelIndex)]->AddWorldRotation(FQuat(DroppedWheelOmega/AngularSpeed,AngularSpeed*Dt));
    DroppedWheelCollision->SetWorldRotation(WheelRoots[PartIndex(DroppedWheelIndex)]->GetComponentQuat());
    if(DroppedContactStableSeconds>=0.25&&AngularSpeed<.1)
    {bLeftWheelSlipFall=false;LeftWheelSlipVelocityCmps=FVector::ZeroVector;}
    auto& Observed=PartPhysics[DroppedWheelIndex];Observed.Pose=GetWheelWorld(DroppedWheelIndex);
    Observed.Velocity=LeftWheelSlipVelocityCmps/100;Observed.Omega=DroppedWheelOmega;Observed.Time+=Dt;Observed.GripContacts=0;
}
void AWheelAssemblyCell::AdvanceSlipRecovery(double DeltaSeconds)
{
	SlipRecoveryPhaseSeconds+=DeltaSeconds;
	if(SlipRecoveryPhase==1)
	{
		AdvanceLeftWheelSlipFall(DeltaSeconds);
		if(bLeftWheelSlipFall)
        {
            if(LeftWheelSlipFallSeconds>10.0) Fail(FString::Printf(TEXT("낙하 휠 미안정 · 물리 갱신 지속, 작업자 확인 필요 · 속도 %s / 위치 %s / 안정 %.3fs"),*LeftWheelSlipVelocityCmps.ToString(),*GetWheelWorld(DroppedWheelIndex).GetLocation().ToString(),DroppedContactStableSeconds));
            return;
        }
        if(!bRunning)return;
        if(!bDemoRecoveryEnabled){Fail(TEXT("낙하 휠 안정 · 검사/부품 교체 요청 필요 (자동 수리 금지)"));return;}
		SlipRecoveryPhase=2;SlipRecoveryPhaseSeconds=0;
		SlipRecoveryTarget=DroppedWheelCollision->GetComponentLocation();
		SlipWorkerStart=SlipRecoveryTarget+BodyAtStart.TransformVectorNoScale(FVector(-250,GeometricSide(RecoveryLane)==0?-200:200,0));
		SlipWorkerStart.Z=SlipRecoveryTarget.Z+90;
		SlipRecoveryWorkerLocation=SlipWorkerStart;
		RecoveryWorkerVisual->SetWorldLocation(SlipWorkerStart);
		RecoveryWorkerVisual->SetVisibility(true);
		RecoveryWorkerHeadVisual->SetWorldLocation(SlipWorkerStart+FVector(0,0,85));
		RecoveryWorkerHeadVisual->SetVisibility(true);
		Status=TEXT("작업자 호출 완료 · 낙하 휠 회수 접근 · 로봇 대기");
		return;
	}
	if(SlipRecoveryPhase==2)
	{
		const double Alpha=FMath::Clamp(SlipRecoveryPhaseSeconds/1.5,0.0,1.0);
		SlipRecoveryWorkerLocation=FMath::Lerp(SlipWorkerStart,SlipRecoveryTarget+FVector(0,0,90),Alpha);
		RecoveryWorkerVisual->SetWorldLocation(SlipRecoveryWorkerLocation);
		RecoveryWorkerHeadVisual->SetWorldLocation(SlipRecoveryWorkerLocation+FVector(0,0,85));
		if(Alpha<1)return;
		SlipRecoveryPhase=3;SlipRecoveryPhaseSeconds=0;
		SlipRecoveryStart=DroppedWheelCollision->GetComponentLocation();
		// A station-relative quarantine port outside BOTH the pickup and mount corridors.
        // Offsetting from the random drop point can leave the rejected tire in the next pickup path.
        SlipRecoveryTarget=BodyAtStart.TransformPosition(FVector(QuarantineOffsetBodyCm.X,
            (GeometricSide(RecoveryLane)==0?-1:1)*FMath::Abs(QuarantineOffsetBodyCm.Y),0));
        SlipRecoveryTarget.Z=SlipRecoveryStart.Z;
		DroppedWheelCollision->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Status=TEXT("작업자가 낙하 휠을 격리 장소로 운반 · 로봇 대기");
		return;
	}
	if(SlipRecoveryPhase==3)
	{
		const double Alpha=FMath::Clamp(SlipRecoveryPhaseSeconds/1.6,0.0,1.0);
		const FVector Position=FMath::Lerp(SlipRecoveryStart,SlipRecoveryTarget,Alpha);
		DroppedWheelCollision->SetWorldLocation(Position);
        WheelRoots[PartIndex(DroppedWheelIndex)]->SetWorldLocation(Position);
		SlipRecoveryWorkerLocation=Position+FVector(0,0,90);
		RecoveryWorkerVisual->SetWorldLocation(SlipRecoveryWorkerLocation);
		RecoveryWorkerHeadVisual->SetWorldLocation(SlipRecoveryWorkerLocation+FVector(0,0,85));
		if(Alpha<1)return;
		SlipRecoveryPhase=4;SlipRecoveryPhaseSeconds=0;
		SlipRecoveryTarget=GetPickupWorld(DroppedWheelIndex).GetLocation();
		SlipRecoveryStart=SlipRecoveryTarget+BodyAtStart.TransformVectorNoScale(FVector(-200,GeometricSide(RecoveryLane)==0?-160:160,0));
		FTransform ReplacementPose=GetPickupWorld(DroppedWheelIndex);ReplacementPose.SetLocation(SlipRecoveryStart);
        WheelRoots[PartIndex(DroppedWheelIndex)]->SetWorldTransform(ReplacementPose);
		TireVisuals[PartIndex(DroppedWheelIndex)]->SetVisibility(true);HubVisuals[PartIndex(DroppedWheelIndex)]->SetVisibility(true);
		Status=TEXT("낙하 휠 격리 완료 · 규격품 재공급 이동 · 로봇 대기");
		return;
	}
	if(SlipRecoveryPhase==4)
	{
		const double Alpha=FMath::Clamp(SlipRecoveryPhaseSeconds/1.8,0.0,1.0);
		WheelRoots[PartIndex(DroppedWheelIndex)]->SetWorldLocation(FMath::Lerp(SlipRecoveryStart,SlipRecoveryTarget,Alpha));
		if(Alpha<1)return;
        if(!WheelProperties.IsValidIndex(PartIndex(DroppedWheelIndex)))WheelProperties.SetNum(4);
        WheelProperties[PartIndex(DroppedWheelIndex)].MassKg=18;
        WheelProperties[PartIndex(DroppedWheelIndex)].Friction=WheelGrip::NormalFriction;
		SlipRecoveryPhase=0;SlipRecoveryPhaseSeconds=0;
		RecoveryWorkerVisual->SetVisibility(false);
		RecoveryWorkerHeadVisual->SetVisibility(false);
		for(int32 Side=0;Side<LaneCount();++Side)if(IsValid(RobotFor(Side)))RobotFor(Side)->SetDroppedWheelCollision(false);
		bLowFrictionGrip=false;GripIncidentMassKg=18.0;
		// The replacement wheel is gripped with a clean pad for the rest of this cycle.
		bGripDegradeSpent=true;GripDegradeStartedAt=-1;LeftGripFriction=WheelGrip::NormalFriction;
		if(SensorsFor(RecoveryLane))SensorsFor(RecoveryLane)->Reset();
		CloseIncidents(TEXT("grip_slip"),TEXT("recovered"));
		Status=TEXT("대체 휠 공급 완료 · 센서 재검사 · 조립 재시도");
		if(!BeginWheel(RecoveryLane))return;
		for(int32 Side=0;Side<LaneCount();++Side)
		{
			if(Side==RecoveryLane||!LaneEnabled(Side)||Lanes[Side].Stage==EWheelStage::Complete)continue;
            // Contact compliance may settle while the other lane is recovering.
            // Re-enter the measured alignment gate before resuming an interrupted fastening.
            if(Lanes[Side].Stage==EWheelStage::Fasten)
            {if(!MoveTo(Side,EWheelStage::Align,Lanes[Side].TargetWheel))return;continue;}
			const bool Commanded=Lanes[Side].Stage==EWheelStage::Reposition
				? RobotFor(Side)->CommandJointDegrees(RearTransit[Lanes[Side].WheelIndex])
				: (Lanes[Side].Stage==EWheelStage::Align||Lanes[Side].Stage==EWheelStage::Retract
                    ? RobotFor(Side)->CommandLinearTCPWorld(Lanes[Side].TargetTCP):RobotFor(Side)->CommandTCPWorld(Lanes[Side].TargetTCP));
			if(!Commanded){Fail(TEXT("회수 후 로봇 작업 재개 실패 · ")+RobotFor(Side)->LastStatus);return;}
		}
	}
}
void AWheelAssemblyCell::AdvanceFrontLeftWheelReject(double DeltaSeconds)
{
	// The replacement sequence moves the wheel on the simulation clock instead.
	if (WheelReplacePhase > 0) return;
	if (!bFrontLeftWheelRejecting || !WheelRoots.IsValidIndex(PartIndex(0)) || !WheelRoots[PartIndex(0)]) return;
	const double Dt=DeltaSeconds;
	if (Dt<=0) return;
	WheelRejectElapsedSeconds+=Dt;
	const double Alpha=FMath::Clamp(WheelRejectElapsedSeconds/1.8,0.0,1.0);
	const double Smooth=Alpha*Alpha*(3.0-2.0*Alpha);
	FTransform Pose=GetWheelWorld(0);
	Pose.SetLocation(FMath::Lerp(WheelRejectStartWorld,WheelRejectTargetWorld,Smooth));
	WheelRoots[PartIndex(0)]->SetWorldTransform(Pose);
	if (Alpha>=1.0)
	{
		bFrontLeftWheelRejecting=false;
		bFrontLeftWheelRejected=true;
	}
}
void AWheelAssemblyCell::AdvanceWheelReplacement(double DeltaSeconds)
{
	if (WheelReplacePhase <= 0 || !WheelRoots.IsValidIndex(PartIndex(0)) || !WheelRoots[PartIndex(0)]) return;
	constexpr double MoveSeconds = 1.8;
	WheelReplaceSeconds += DeltaSeconds;
	const double Alpha = FMath::Clamp(WheelReplaceSeconds / MoveSeconds, 0.0, 1.0);
	const double Smooth = Alpha*Alpha*(3.0-2.0*Alpha);
	FTransform Pose = GetWheelWorld(0);
	if (WheelReplacePhase == 1)
	{
		Pose.SetLocation(FMath::Lerp(WheelRejectStartWorld, WheelRejectTargetWorld, Smooth));
		WheelRoots[PartIndex(0)]->SetWorldTransform(Pose);
		if (Alpha < 1.0) return;
		// Leave a distinct visual proxy for the rejected part at its rejected size.
		DroppedWheelCollision->SetWorldTransform(Pose);
		DroppedTireVisual->SetRelativeScale3D(FVector(0.66,0.66,0.16)*FrontLeftWheelSizeScale);
		DroppedHubVisual->SetRelativeScale3D(FVector(0.37,0.37,0.025)*FrontLeftWheelSizeScale);
		DroppedTireVisual->SetVisibility(true); DroppedHubVisual->SetVisibility(true);
		bFrontLeftWheelRejecting = false; bFrontLeftWheelRejected = true;
		RejectedWheelInstanceId=FrontLeftActiveWheelInstanceId;
		SpareWheelInstanceId=ProductInstanceId+TEXT("/")+GetFrontLeftPartId()+TEXT("/spare_1");
		FrontLeftActiveWheelInstanceId=SpareWheelInstanceId;
		// The spare is a nominal wheel. Until the next scenario selection this cell supplies nominal wheels.
		FrontLeftWheelSizeScale = 1.0;
		TireVisuals[PartIndex(0)]->SetRelativeScale3D(FVector(0.66,0.66,0.16));
		HubVisuals[PartIndex(0)]->SetRelativeScale3D(FVector(0.37,0.37,0.025));
		WheelSpareStartWorld = GetPickupWorld(0).GetLocation() + BodyAtStart.TransformVectorNoScale(FVector(0,-160,0));
		Pose = GetPickupWorld(0); Pose.SetLocation(WheelSpareStartWorld);
		WheelRoots[PartIndex(0)]->SetWorldTransform(Pose);
		WheelReplacePhase = 2; WheelReplaceSeconds = 0;
		Status = TEXT("불합격 휠 격리 완료 · 대체 휠 공급 이송 · 로봇 대기");
		return;
	}
	Pose = GetPickupWorld(0);
	Pose.SetLocation(FMath::Lerp(WheelSpareStartWorld, GetPickupWorld(0).GetLocation(), Smooth));
	WheelRoots[PartIndex(0)]->SetWorldTransform(Pose);
	if (Alpha < 1.0) return;
	WheelReplacePhase = 0; WheelReplaceSeconds = 0; ++WheelReplacements;
	OpenIncident(TEXT("spare_supplied"),0,EWheelStage::WaitSupply,TEXT("대체 휠 도착"),TEXT("개"),
		1,1,TEXT("공급 센서와 레이저 규격 재검사 대기"));
	Status = TEXT("대체 휠 도착 · 공급 센서 확인 후 규격 재검사");
}
void AWheelAssemblyCell::BeginPlay() { Super::BeginPlay(); if (!bExternallyDriven) ResetCycle(); }
void AWheelAssemblyCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if(bShowSensorVisualization&&GetWorld()&&(GetWorld()->WorldType==EWorldType::PIE||GetWorld()->WorldType==EWorldType::Game))
	{
		if(bDemoChassisShiftApplied && FrontLeftDockScan.bValid && FrontLeftHubScan.bValid)
		{
			const FVector Before=FrontLeftDockScan.MeasuredWorld.GetLocation();
			const FVector After=FrontLeftHubScan.MeasuredWorld.GetLocation();
			DrawDebugLine(GetWorld(),Before,After,FColor::Yellow,false,0.08f,0,5.f);
			DrawDebugSphere(GetWorld(),Before,9,12,FColor::Cyan,false,0.08f,0,2.f);
			DrawDebugSphere(GetWorld(),After,9,12,FColor::Yellow,false,0.08f,0,2.f);
			DrawDebugString(GetWorld(),After+FVector(0,0,125),
				FString::Printf(TEXT("HUB SHIFT %.1f mm / LIMIT 5.0 mm\n%s"),DockToPreMountShiftMm,
					FrontLeftPreMountScan.bValid?TEXT("RESCAN OK - NEW TARGET"):TEXT("RETRACT - RESCAN")),
				nullptr,FColor::Yellow,0.08f,false,1.f);
		}
		if (bFrontLeftWheelRejected && !RejectedWheelInstanceId.IsEmpty())
		{
			DrawDebugString(GetWorld(),DroppedWheelCollision->GetComponentLocation()+FVector(0,0,95),
				FString::Printf(TEXT("REJECTED WHEEL / %.1f cm\n%s"),RejectedWheelDiameterCm,*RejectedWheelInstanceId),
				nullptr,FColor::Red,0.08f,false,1.f);
			if (WheelReplacePhase==2)
				DrawDebugString(GetWorld(),GetWheelWorld(0).GetLocation()+FVector(0,0,95),
					FString::Printf(TEXT("SPARE WHEEL / %s"),*SpareWheelInstanceId),
					nullptr,FColor::Green,0.08f,false,1.f);
		}
		if(SlipRecoveryPhase>0)
		{
			const FVector Fallen=GetDroppedWheelLocation();
			DrawDebugSphere(GetWorld(),Fallen,35,16,FColor::Red,false,0.08f,0,3.f);
			DrawDebugString(GetWorld(),Fallen+FVector(0,0,115),
				FString::Printf(TEXT("%s / %s / %.0f kg / operator recovery %d/4"),
					GetSlipFailureLabel(),
					bDroppedWheelHitSurface?TEXT("SURFACE CONTACT"):TEXT("FALLING"),
					SlipFailureMassKg,SlipRecoveryPhase),nullptr,FColor::Yellow,0.08f,false,1.f);
			{
				DrawDebugCapsule(GetWorld(),SlipRecoveryWorkerLocation+FVector(0,0,35),100,28,FQuat::Identity,
					FColor::Yellow,false,0.08f,0,3.f);
				DrawDebugString(GetWorld(),SlipRecoveryWorkerLocation+FVector(0,0,150),TEXT("OPERATOR: COLLECT / RESUPPLY"),
					nullptr,FColor::Yellow,0.08f,false,1.f);
			}
		}
		if(FrontLeftWheelMeasuredDiameterCm>0&&!bFrontLeftWheelSizePassed)
		{
			const FVector Center=GetWheelWorld(0).GetLocation();
			DrawDebugSphere(GetWorld(),Center,33,24,FColor::Green,false,0.08f,0,1.5f);
			DrawDebugSphere(GetWorld(),Center,33*FrontLeftWheelSizeScale,24,FColor::Red,false,0.08f,0,3.f);
			DrawDebugString(GetWorld(),Center+FVector(0,0,120),
				FString::Printf(TEXT("SIZE REJECT: %.1f cm / 66.0 +/- 1.0 cm"),FrontLeftWheelMeasuredDiameterCm),
				nullptr,FColor::Red,0.08f,false,1.f);
		}
	}
	if (bExternallyDriven) return;
	if (DeltaSeconds > 1 && bRunning) { StopCycle(); Status = TEXT("Large frame stall; reset required."); return; }
	AdvanceSimulation(DeltaSeconds);
}

int32 AWheelAssemblyCell::PhysicalGripContacts(int32 Index) const
{
    if(!bUseContactPhysics||Index<0||Index>=4)return -1;
    for(const auto& Lane:Lanes)if(Lane.WheelIndex==Index&&Lane.ContactGrip.bInitialized)return Lane.ContactGrip.Body.GripContacts;
    return -1;
}

FWheelContactProperties AWheelAssemblyCell::ContactPropertiesFor(int32 Index) const
{
    auto P=WheelProperties.IsValidIndex(PartIndex(Index))?WheelProperties[PartIndex(Index)]:FWheelContactProperties();
    if(bGripIncidentOverride&&Index==GripIncidentWheelIndex)P.MassKg=GripIncidentMassKg;
    return P;
}
const FWheelLane* AWheelAssemblyCell::LaneForWheel(int32 Index) const
{for(const auto& Lane:Lanes)if(Lane.WheelIndex==Index)return &Lane;return nullptr;}
FTransform AWheelAssemblyCell::TCPForWheel(int32 Index) const
{if(const auto* TCP=GraspTCPByPart.Find(PartIndex(Index)))return *TCP;
 for(int I=0;I<LaneCount();++I)if(Lanes[I].WheelIndex==Index)return TCPFor(I);return Index<2?LeftTCPInWheel:RightTCPInWheel;}
void AWheelAssemblyCell::AdvanceSecondaryFalls(double Dt)
{
    for(int I=0;I<4;++I)if(SecondaryFalling[I])
    {
        auto& B=PartPhysics[I];const auto Before=B.Pose.GetLocation();
        if(!B.Step({},B.Pose,B.Pose,B.Pose,Dt)){Fail(TEXT("낙하 부품 물리 무효"));continue;}
        FHitResult Hit;FCollisionQueryParams Params(SCENE_QUERY_STAT(SecondaryWheelContact),false,this);
        if(GetWorld()->SweepSingleByChannel(Hit,Before,B.Pose.GetLocation(),FQuat::Identity,ECC_PhysicsBody,FCollisionShape::MakeSphere(ContactRadiusCm(I)),Params))
        {const auto N=(Hit.bStartPenetrating?Hit.Normal:Hit.ImpactNormal).GetSafeNormal();
         B.Pose.SetLocation(Hit.bStartPenetrating?Before+N*(Hit.PenetrationDepth+.001):Hit.Location+N*.001);B.ResolveImpact(N,-N*(ContactRadiusCm(I)/100));}
        WheelRoots[PartIndex(I)]->SetWorldTransform(B.Pose);
    }
}

double AWheelAssemblyCell::ContactRadiusCm(int32 Index) const
{const auto P=ContactPropertiesFor(Index);return 100*FMath::Sqrt(P.OuterRadiusM*P.OuterRadiusM+P.WidthM*P.WidthM/4);}

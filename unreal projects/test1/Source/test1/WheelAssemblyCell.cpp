#include "WheelAssemblyCell.h"
#include "WheelGripPhysics.h"
#include "CamryAssembly.h"
#include "RobotArm.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SphereComponent.h"
#include "UObject/ConstructorHelpers.h"

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
	const FTransform Contact = Ready ? (WheelIndex<2?Cell->LeftTCPInWheel:Cell->RightTCPInWheel) * Wheel : FTransform::Identity;
	const bool AtContact = Present && FVector::Distance(Contact.GetLocation(), TCP.GetLocation()) * 10 <= Cell->PositionToleranceMm
		&& FMath::RadiansToDegrees(Contact.GetRotation().AngularDistance(TCP.GetRotation())) <= Cell->OrientationToleranceDeg;
	Gripped.bValid = Ready; Gripped.bDetected = bGripClosed && !bGripActuatorFault && AtContact; Gripped.MeasuredAt = Now;
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
}

AWheelAssemblyCell::AWheelAssemblyCell()
{
	PrimaryActorTick.bCanEverTick = true;
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
	LeftSensors = CreateDefaultSubobject<UWheelRobotSensorBank>(TEXT("LeftRobotSensors"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
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
FTransform AWheelAssemblyCell::TCPFor(int32 Side) const { return GeometricSide(Side) == 0 ? LeftTCPInWheel : RightTCPInWheel; }

FTransform AWheelAssemblyCell::GetMountWorld(int32 Index) const
{
	if (!IsValid(Assembly) || !Assembly->GetBodyFrame() || Index < 0 || Index > 3) return FTransform::Identity;
	const double X = Index % 2 == 0 ? FrontXcm : RearXcm;
	const double Y = Index < 2 ? -HubHalfTrackCm : HubHalfTrackCm;
	FTransform Mount = FTransform(FRotator::ZeroRotator, MountCentersCm.Num() == 4 ? MountCentersCm[Index] : FVector(X, Y, HubHeightCm)) * Assembly->GetBodyFrame()->GetComponentTransform();
	if (Index == 0 && bLaserScanFrontLeftHub)
		Mount.AddToTranslation(Assembly->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(FrontLeftHubOffsetCm));
	return Mount;
}

FTransform AWheelAssemblyCell::GetMountTargetWorld(int32 Index) const
{
	return Index == 0 && bLaserScanFrontLeftHub && FrontLeftHubScan.bValid
		? FrontLeftHubScan.MeasuredWorld : GetMountWorld(Index);
}

bool AWheelAssemblyCell::ScanFrontLeftHub(bool bScanWheel)
{
	FWheelHubScanReading& Reading=bScanWheel?FrontLeftFinalWheelScan:FrontLeftHubScan;
	Reading = FWheelHubScanReading();
	Reading.TargetPort=bScanWheel?TEXT("wheel_fl_seated"):TEXT("hub_fl");
	if (bBlockFrontLeftLaser) { Reading.FailureReason=TEXT("광선 차단"); return false; }
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
		MountCentersCm.Num() == 4 ? MountCentersCm[0] : FVector(FrontXcm,-HubHalfTrackCm,HubHeightCm)) * BodyAtStart).GetLocation();
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
	FTransform P=FTransform(FRotator::ZeroRotator, FVector(X, Y, MountCentersCm.Num() == 4 ? MountCentersCm[Index].Z : HubHeightCm)) * (bEnableTransport && bStationCaptured ? BodyAtStart : Assembly->GetBodyFrame()->GetComponentTransform());
    P.AddToTranslation(SupplyOffsetsCm.FindRef(Index));return P;
}

FTransform AWheelAssemblyCell::GetWheelWorld(int32 Index) const
{
	return GetPhysicalPartWorld(PartIndex(Index));
}

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
	return FString::Printf(TEXT("왼팔 %s · 타이어 %d · 부품 %s · 위치 %s · 파지 %s · 장착 측정 %s · 재시도 %d"),
		*GetLaneStage(true), Index + 1,
		LeftSensors->Presence.Fresh(Now, LeftSensors->MaxAge) ? (LeftSensors->Presence.bDetected ? TEXT("있음") : TEXT("없음")) : TEXT("신호 지연"),
		PoseFresh ? TEXT("유효") : TEXT("무효/지연"), GripFresh ? (LeftSensors->Gripped.bDetected ? TEXT("확인") : TEXT("미확인")) : TEXT("지연"),
		LeftSensors->MountPose.Fresh(Now, LeftSensors->MaxAge) ? TEXT("유효") : TEXT("무효/지연"), Lanes[0].GripAttempts)
		+ FString::Printf(TEXT(" · 경로 재계산 %d · 파지 재시도 %d"), LeftPickupReplans, LeftGripRetries)
		+ (bPhysicalLeftGrip ? FString::Printf(TEXT("\n앞 휠 질량 %.0f kg · 필요 힘 %.0f N · 전달 힘 %.0f N · 마찰 한계 %.0f N · 미끄러짐 %.1f mm"), FrontLeftWheelMassKg, Lanes[0].DemandN, Lanes[0].AppliedForceN, Lanes[0].CapacityN, Lanes[0].SlipMm) : FString())
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
		if ((UsesTaskBindings()?RearTransit[TaskQueue[Side].Last()]:(Side == 0 ? LeftRearApproachAnglesDegrees : RightRearApproachAnglesDegrees)).Num() != 6)
		{ Status = TEXT("Both rear station joint targets require six angles."); return false; }
		RobotAtStart[Side] = RobotFor(Side)->GetActorTransform();
		if (!RobotAtStart[Side].IsValid() || !RobotAtStart[Side].GetScale3D().Equals(FVector::OneVector, 1.e-6))
		{ Status = TEXT("Robot base must have a valid rigid transform."); return false; }
	}
	return true;
}

bool AWheelAssemblyCell::ResetCycle()
{
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
	MountedCount = 0; SimulationTimeSeconds = 0; Accumulator = 0;
	bLeftWheelSlipFall = false; LeftWheelSlipFallSeconds = 0; LeftWheelSlipVelocityCmps = FVector::ZeroVector;
	FrontLeftHubScan = FWheelHubScanReading();
	FrontLeftDockScan = FrontLeftPreMountScan = FrontLeftFinalWheelScan = FWheelHubScanReading();
	FrontLeftWheelMeasuredDiameterCm = 0; bFrontLeftWheelSizePassed = false;
	WheelSizeNearHitWorld = WheelSizeFarHitWorld = FVector::ZeroVector;
	LeftPickupReplans = LeftGripRetries = 0;
	Outcome = EWheelOutcome::NotEvaluated;
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
	FWheelLane& Lane = Lanes[Side];
    if(!LaneEnabled(Side)){Lane.Stage=EWheelStage::Complete;return true;}
    Lane.WheelIndex = UsesTaskBindings()?TaskQueue[Side][Lane.Completed]:Side * 2 + Lane.Completed;
    if (Lane.WheelIndex == 0 && bLaserScanFrontLeftHub && !ScanFrontLeftHub())
    { Fail(TEXT("왼쪽 앞 허브 레이저 스캔 실패 · ")+FrontLeftHubScan.FailureReason); return false; }
    if (Lane.WheelIndex == 0 && bLaserScanFrontLeftHub) FrontLeftDockScan=FrontLeftHubScan;
    if (Lane.WheelIndex == 0 && FrontLeftWheelSizeScale != 1.0 && !ScanFrontLeftWheelSize())
    { Fail(FString::Printf(TEXT("휠 규격 불량 · 레이저 측정 지름 %.1fcm / 기준 66.0cm ± 1.0cm"),FrontLeftWheelMeasuredDiameterCm)); return false; }
    auto* Sensors=SensorsFor(Side);
    // Rear-only robots first move through the calibrated, base-adjusted transit pose.
    if(UsesTaskBindings()&&Lane.Completed==0&&Lane.Stage==EWheelStage::Idle&&Lane.WheelIndex%2==1)
    {
        if(!RobotFor(Side)->CommandJointDegrees(RearTransit[Lane.WheelIndex])){Fail(RobotFor(Side)->LastStatus);return false;}
        Lane.TargetTCP=RobotFor(Side)->GetTargetTCPWorld();SetStage(Side,EWheelStage::Reposition,TEXT("후방 작업 접근"));return true;
    }
	Lane.bHeld = false; Lane.bFastened = false; Lane.Corrections = 0;
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
	FWheelLane& Lane = Lanes[Side];
	Lane.TargetWheel = WheelTarget;
	Lane.TargetTCP = (Lane.bHeld ? Lane.WheelInTCP.Inverse() : TCPFor(Side)) * WheelTarget;
	if (!RobotFor(Side)->CommandTCPWorld(Lane.TargetTCP))
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
	SetStage(Side, WaitStage, Message);
}

void AWheelAssemblyCell::Fail(const FString& Reason)
{
	const FString Saved = Reason;
	bRunning = false; bTransporting = false; Outcome = EWheelOutcome::Blocked;
	if (LeftRobot) LeftRobot->Halt();
	if (RightRobot) RightRobot->Halt();
    for(auto R:TaskRobots)if(IsValid(R))R->Halt();
	for (FWheelLane& Lane : Lanes) if (Lane.Stage != EWheelStage::Complete) Lane.Stage = EWheelStage::Fault;
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

void AWheelAssemblyCell::StepLane(int32 Side, double Dt)
{
	FWheelLane& Lane = Lanes[Side];
	if (!LaneEnabled(Side)||Lane.Stage == EWheelStage::Complete) return;
	ARobotArm* Robot = RobotFor(Side);
	const bool SensorDriven = SensorDrivenLane(Side);
    auto* Sensors=SensorsFor(Side);
	if (SensorDriven && (Lane.Stage == EWheelStage::WaitSupply || Lane.Stage == EWheelStage::WaitPose))
	{
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
			Lane.Pickup = Measured; Lane.bGripCommanded = false; Sensors->CommandGrip(false); ++LeftPickupReplans;
			MoveTo(Side, EWheelStage::ApproachPickup, OffsetOutward(Measured, Side, 25));
			return;
		}
	}
	Lane.StageTime += Dt;
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
	if (!Robot->StepControl(Dt)) { Fail(TEXT("Robot servo failed.")); return; }
	if (Lane.bHeld && !Lane.bFastened)
    {
        FTransform Target = Lane.WheelInTCP * Robot->GetTCPWorld();
        const bool Physical = bPhysicalLeftGrip && Lane.WheelIndex == 0;
        if (Physical)
            Target.SetLocation(StepWheelGrip(Lane.LastCarried.GetLocation(), Target.GetLocation(), Dt, bLowFrictionGrip, FrontLeftWheelMassKg,
                Lane.GripVelocity, Lane.DemandN, Lane.CapacityN, Lane.SlipMm, Lane.AppliedForceN));
        WheelRoots[PartIndex(Lane.WheelIndex)]->SetWorldTransform(Target);
        if (Physical && Lane.SlipMm >= 3.0)
        {
            bLeftWheelSlipFall = true;
            LeftWheelSlipFallSeconds = 0;
            LeftWheelSlipVelocityCmps = BodyAtStart.TransformVectorNoScale(FVector(0,-100,0));
            Lane.bHeld = false;
            if (Sensors) Sensors->CommandGrip(false);
            Fail(FString::Printf(TEXT("%s · 휠 질량 %.0fkg · 미끄러짐 %.1fmm · 필요 %.0fN > 마찰 한계 %.0fN · 파지 해제/낙하"),
                bLowFrictionGrip?TEXT("저마찰"):TEXT("과중량"),FrontLeftWheelMassKg,Lane.SlipMm,Lane.DemandN,Lane.CapacityN));
            return;
        }
    }
	Lane.LastCarried = GetWheelWorld(Lane.WheelIndex);
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
			FastenedInBody[Lane.WheelIndex] = GetWheelWorld(Lane.WheelIndex).GetRelativeTransform(Assembly->GetBodyFrame()->GetComponentTransform());
			SetStage(Side, EWheelStage::Release, TEXT("virtual fastening complete"));
		}
		return;
	}
	if (Lane.Stage == EWheelStage::Release)
	{
		if (SensorDriven) Sensors->CommandGrip(false);
		Lane.bHeld = false;
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
					else { Sensors->CommandGrip(false); Lane.bGripCommanded = false; ++LeftGripRetries; Status = TEXT("Left arm: grip not confirmed; retrying"); }
				}
				break;
			}
		}
		const FTransform Contact = TCPFor(Side) * Lane.Pickup;
		if (FVector::Distance(Contact.GetLocation(), CurrentTCP.GetLocation()) * 10 > PositionToleranceMm
			|| FMath::RadiansToDegrees(Contact.GetRotation().AngularDistance(CurrentTCP.GetRotation())) > OrientationToleranceDeg)
		{ Fail(TEXT("Gripper contact mismatch.")); break; }
		Lane.WheelInTCP = Lane.Pickup.GetRelativeTransform(CurrentTCP);
		Lane.bHeld = true; Lane.LastCarried = Lane.Pickup;
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
			const FVector DockCenter=FrontLeftDockScan.MeasuredWorld.GetLocation();
			if (!ScanFrontLeftHub()) { Fail(TEXT("장착 직전 허브 재스캔 실패 · ")+FrontLeftHubScan.FailureReason); break; }
			FrontLeftPreMountScan=FrontLeftHubScan;
			const double ShiftMm=FVector::Distance(DockCenter,FrontLeftPreMountScan.MeasuredWorld.GetLocation())*10;
			if (ShiftMm>5) { Fail(FString::Printf(TEXT("도킹 후 허브 위치 변화 %.2fmm · 재도킹 필요"),ShiftMm)); break; }
		}
		MoveTo(Side, EWheelStage::ApproachMount, OffsetOutward(GetMountTargetWorld(Lane.WheelIndex), Side, 25)); break;
	case EWheelStage::ApproachMount:
		MoveTo(Side, EWheelStage::Align, GetMountTargetWorld(Lane.WheelIndex)); break;
	case EWheelStage::Align:
	{
		if (SensorDriven && !Sensors->MountPose.Fresh(SimulationTimeSeconds, Sensors->MaxAge))
		{ Fail(TEXT("Left robot: mount pose reading stale.")); break; }
		const FVector Measured = (SensorDriven ? Sensors->MountPose.Pose : GetWheelWorld(Lane.WheelIndex)).GetLocation()
			+ BodyAtStart.TransformVectorNoScale(SensorBiasMm / 10.0);
		const FVector Residual = GetMountWorld(Lane.WheelIndex).GetLocation() - Measured;
		if (Residual.Length() * 10 > PositionToleranceMm)
		{
			if (++Lane.Corrections > 8) { Fail(TEXT("Sensor alignment did not converge.")); break; }
			FTransform Corrected = Lane.TargetWheel;
			Corrected.AddToTranslation(Residual);
			MoveTo(Side, EWheelStage::Align, Corrected); break;
		}
		if (ControlMode == EWheelControlMode::IndependentVerification)
		{
			if (!bVerificationSensorValid) { Fail(TEXT("Independent verification sensor invalid; fastening blocked.")); break; }
			RefreshQuality();
			const FTransform Verified = SensorDriven ? Sensors->MountPose.Pose : GetWheelWorld(Lane.WheelIndex);
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
			if(UsesTaskBindings()&&TaskQueue[Side][Lane.Completed]%2==0){BeginWheel(Side);break;}
            const TArray<double>& RearAngles = UsesTaskBindings()?RearTransit[TaskQueue[Side][Lane.Completed]]:(Side == 0 ? LeftRearApproachAnglesDegrees : RightRearApproachAnglesDegrees);
			if (!Robot->CommandJointDegrees(RearAngles)) { Fail(Robot->LastStatus); break; }
			Lane.TargetTCP = Robot->GetTargetTCPWorld();
			SetStage(Side, EWheelStage::Reposition, TEXT("joint-space transit to rear station"));
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
	if (!bRunning) return true;
	Accumulator += DeltaSeconds;
	while (bRunning && Accumulator + 1.e-10 >= FixedStepSeconds)
	{
		Accumulator -= FixedStepSeconds; SimulationTimeSeconds += FixedStepSeconds;
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
			|| (bEnableTransport && (!IsValid(TransportCarriage) || !TransportCarriage->GetActorTransform().Equals(CarriageAtStation, 1.e-3)))
			|| !Assembly->GetBodyFrame()->GetComponentTransform().Equals(BodyAtStart, bEnableTransport ? 1.e-3 : 1.e-6)
			|| (!UsesTaskBindings()&&!LeftRobot->GetActorTransform().Equals(RobotAtStart[0], 1.e-6))
			|| (!UsesTaskBindings()&&!RightRobot->GetActorTransform().Equals(RobotAtStart[1], 1.e-6)))
		{ Fail(TEXT("Vehicle or robot base moved during assembly.")); break; }
        for(int32 S=0;S<LaneCount()&&bRunning;++S)if(LaneEnabled(S))
        {
            if(!IsValid(RobotFor(S))||!RobotFor(S)->GetActorTransform().Equals(RobotAtStart[S],1.e-6))
            {Fail(TEXT("작업 로봇이 삭제되거나 실행 중 이동했습니다."));break;}
            if(UsesTaskBindings()&&Lanes[S].WheelIndex>=0)SensorsFor(S)->Sample(SimulationTimeSeconds,this,Lanes[S].WheelIndex);
            StepLane(S,FixedStepSeconds);
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
			WheelRoots[PartIndex(I)]->SetWorldTransform(FastenedInBody[I] * Assembly->GetBodyFrame()->GetComponentTransform());
}
void AWheelAssemblyCell::AdvanceLeftWheelSlipFall(double DeltaSeconds)
{
	if (!bLeftWheelSlipFall || !WheelRoots.IsValidIndex(PartIndex(0)) || !WheelRoots[PartIndex(0)]) return;
	const double Dt=FMath::Clamp(DeltaSeconds,0.0,0.05);
	if (Dt<=0) return;
	LeftWheelSlipFallSeconds+=Dt;
	const double GroundCenterZ=FMath::Min(GetPickupWorld(0).GetLocation().Z,GetMountWorld(0).GetLocation().Z)-25.0;
	FTransform Pose=GetWheelWorld(0);
	FVector Position=Pose.GetLocation();
	LeftWheelSlipVelocityCmps.Z-=981.0*Dt;
	Position+=LeftWheelSlipVelocityCmps*Dt;
	if (Position.Z<=GroundCenterZ)
	{
		Position.Z=GroundCenterZ;
		LeftWheelSlipVelocityCmps.Z=0;
		LeftWheelSlipVelocityCmps.X*=FMath::Exp(-2.0*Dt);
		LeftWheelSlipVelocityCmps.Y*=FMath::Exp(-2.0*Dt);
	}
	Pose.SetLocation(Position);
	WheelRoots[PartIndex(0)]->SetWorldTransform(Pose);
	if (LeftWheelSlipFallSeconds>=2.0) bLeftWheelSlipFall=false;
}
void AWheelAssemblyCell::BeginPlay() { Super::BeginPlay(); if (!bExternallyDriven) ResetCycle(); }
void AWheelAssemblyCell::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	AdvanceLeftWheelSlipFall(DeltaSeconds);
	if (bExternallyDriven) return;
	if (DeltaSeconds > 1 && bRunning) { StopCycle(); Status = TEXT("Large frame stall; reset required."); return; }
	AdvanceSimulation(DeltaSeconds);
}

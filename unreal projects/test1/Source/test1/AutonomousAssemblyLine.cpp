#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "DoorAssemblyCell.h"
#include "CamryAssembly.h"
#include "DrawDebugHelpers.h"

bool AAutonomousAssemblyLine::SelectDemo(EArrivalDemo Selection)
{
	// This map always uses the normal conveyor; only the independent plant varies.
	return false;
}

bool AAutonomousAssemblyLine::ResetLine()
{
	Demo = EArrivalDemo::Normal;
	if (WheelCell) { WheelCell->bSensorDrivenLeftRobot = true; WheelCell->bAutoRecoverLeftRobot = true; }
	if (!Super::ResetLine()) return false;
	const int32 ThisRun=RunNumber++;
	CurrentSeed=BaseSeed+ThisRun;
	WheelCell->SetRunIdentity(FString::Printf(TEXT("demo_request_%d"),ThisRun),FString::Printf(TEXT("demo_vehicle_%d"),ThisRun),ThisRun);
	Random.Initialize(CurrentSeed);
	InjectedEvents = RecoveredEvents = 0;
	ActiveEvent = TEXT("첫 타이어 공급 측정 대기");
	for (int32 Index=0; Index<2; ++Index)
	{
		Planned[Index] = ERandomWheelEvent(Random.RandRange(0,3));
		FirstSeenAt[Index] = RestoreAt[Index] = SupplyTravelStartedAt[Index] = -1;
		SupplyTravelFrom[Index] = SupplyTravelTo[Index] = FVector::ZeroVector;
		TriggerAfter[Index] = Random.FRandRange(.1,.4);
		bInjected[Index] = bRestored[Index] = bSupplyArrivalRecorded[Index] = false;
	}
	if (FactoryCheck != 5)
	{
		bInjected[1] = bRestored[1] = true;
		if (FactoryCheck == 0 || FactoryCheck >= 6) bInjected[0] = bRestored[0] = true;
		else Planned[0] = static_cast<ERandomWheelEvent>(FactoryCheck-1);
	}
	// Drawn after the per-wheel plan so existing seeds keep their original wheel events.
	ExtendedEvent = 0;
	if (FactoryCheck == 5 && WheelCell)
	{
		WheelCell->bPhysicalLeftGrip = false;
		WheelCell->bLowFrictionGrip = false;
		WheelCell->bGripDegradesInTransit = false;
		WheelCell->FrontLeftWheelMassKg = 18.0;
		bDemoDockOvershoot = false;
		if (bExtendedRandomEvents)
		{
			ExtendedEvent = Random.RandRange(0,3);
			if (ExtendedEvent == 1 || ExtendedEvent == 2)
			{
				// One event per wheel: the front-left wheel carries the physical event,
				// the rear-left wheel keeps its seeded supply/sensor/grip event.
				WheelCell->bPhysicalLeftGrip = true;
                WheelCell->bDemoRecoveryEnabled = true;
				WheelCell->FrontLeftWheelMassKg = ExtendedEvent == 1 ? 90.0 : 18.0;
				WheelCell->bGripDegradesInTransit = ExtendedEvent == 2;
				bInjected[0] = bRestored[0] = true;
			}
			bDemoDockOvershoot = ExtendedEvent == 3;
		}
	}
	Record(TEXT("RANDOM_SEED"),FString::FromInt(CurrentSeed));
	if (bExtendedRandomEvents && FactoryCheck == 5) Record(TEXT("RANDOM_LINE_EVENT"),ExtendedEventName(ExtendedEvent));
	return true;
}

const TCHAR* AAutonomousAssemblyLine::ExtendedEventName(int32 Kind)
{
	switch (Kind)
	{
	case 1: return TEXT("overweight front-left wheel");
	case 2: return TEXT("grip degrades in transit");
	case 3: return TEXT("dock overshoot");
	default: return TEXT("none");
	}
}

void AAutonomousAssemblyLine::SetFactoryCheck(int32 Kind)
{
	FactoryCheck = FMath::Clamp(Kind, 0, 13);
    if (WheelCell)
    {
        WheelCell->bPhysicalLeftGrip = FactoryCheck >= 6 && FactoryCheck <= 8;
        WheelCell->bDemoRecoveryEnabled = FactoryCheck == 7 || FactoryCheck == 8; // Explicit legacy recovery presentation only.
        WheelCell->bLowFrictionGrip = FactoryCheck == 7;
        WheelCell->bGripDegradesInTransit = false;
        WheelCell->FrontLeftWheelMassKg = FactoryCheck == 8 ? 90.0 : 18.0;
        WheelCell->FrontLeftWheelSizeScale = FactoryCheck == 9 ? 1.12 : FactoryCheck == 10 ? 0.88 : 1.0;
    }
    bDemoDockOvershoot=FactoryCheck==13;
    if(DoorCell)
    {
        ConfigureFrontDoorSize(FactoryCheck==11?1.12:FactoryCheck==12?0.88:1.0);
        DoorCell->bReplaceRejectedDoor=FactoryCheck==11||FactoryCheck==12;
    }
}

bool AAutonomousAssemblyLine::ConfigureIncidentDemo(int32 Preset)
{
    if (!IsValid(WheelCell) || Preset < 0 || Preset > 19) return false;
    const int32 BaseCheck = Preset <= 13 ? Preset : Preset == 14 ? 6 : Preset == 15 ? 0
        : Preset == 16 ? 9 : Preset == 17 ? 10 : Preset == 19 ? 0 : 5;
    // The line adapter configures the cell before ResetLine; the UI never writes equipment state.
    bExtendedRandomEvents = false;
    WheelCell->bReplaceRejectedWheel = false;
    WheelCell->LaserTransientBlockScans = 0;
    WheelCell->bDemoChassisShiftBeforeMount = false;
    SetFactoryCheck(BaseCheck);
    switch (Preset)
    {
    case 14: WheelCell->bGripDegradesInTransit = true; WheelCell->bDemoRecoveryEnabled=true; break;
    case 15: WheelCell->bLaserScanFrontLeftHub = true; WheelCell->LaserTransientBlockScans = 2; break;
    case 16: case 17: WheelCell->bReplaceRejectedWheel = true; break;
    case 18: bExtendedRandomEvents = true; break;
    case 19: WheelCell->bLaserScanFrontLeftHub = true; WheelCell->bDemoChassisShiftBeforeMount = true; break;
    default: break;
    }
    return true;
}

void AAutonomousAssemblyLine::InjectEvent(int32 Index)
{
	if (!WheelCell || !WheelCell->LeftSensors) return;
	bInjected[Index] = true;
	++InjectedEvents;
	const double RecoveryDelay = Random.FRandRange(MinRecoveryDelaySeconds,MaxRecoveryDelaySeconds);
	switch (Planned[Index])
	{
	case ERandomWheelEvent::SupplyMissing:
		WheelCell->SetLeftWheelSupplyVisible(Index,false);
		SupplyTravelTo[Index] = WheelCell->GetWheelWorld(Index).GetLocation();
		SupplyTravelFrom[Index] = SupplyTravelTo[Index]
			+ WheelCell->Assembly->GetBodyFrame()->GetComponentTransform().TransformVectorNoScale(FVector(0,-160,0));
		WheelCell->DisplaceLeftWheel(Index,SupplyTravelFrom[Index]-SupplyTravelTo[Index]);
		RestoreAt[Index] = SensorTime + RecoveryDelay;
		ActiveEvent = FString::Printf(TEXT("타이어 %d 누락 → 자동 공급 요청"),Index+1);
		break;
	case ERandomWheelEvent::PickupPoseDropout:
		WheelCell->LeftSensors->bPublishPickupPose = false;
		RestoreAt[Index] = SensorTime + RecoveryDelay;
		ActiveEvent = FString::Printf(TEXT("타이어 %d 위치 센서 통신 끊김 → 재연결 대기"),Index+1);
		break;
	case ERandomWheelEvent::GripFailure:
		WheelCell->LeftSensors->bGripActuatorFault = true;
		RestoreAt[Index] = SensorTime + RecoveryDelay;
		ActiveEvent = FString::Printf(TEXT("타이어 %d 파지 실패 → 그리퍼 복구·재시도"),Index+1);
		break;
	case ERandomWheelEvent::WheelDisplaced:
		WheelCell->DisplaceLeftWheel(Index,FVector(0,-Random.FRandRange(3.,5.),0));
		bRestored[Index] = true; ++RecoveredEvents;
		ActiveEvent = FString::Printf(TEXT("타이어 %d 위치 변화 → 측정 경로 재계산"),Index+1);
		break;
	}
	Record(TEXT("RANDOM_EVENT"),ActiveEvent);
}

void AAutonomousAssemblyLine::AdvanceSupplyDelivery(int32 Index)
{
	if (SupplyTravelStartedAt[Index]<0)
	{
		SupplyTravelStartedAt[Index]=SensorTime;
		WheelCell->SetLeftWheelSupplyVisible(Index,true);
		WheelCell->OpenIncident(TEXT("spare_dispatched"),Index,WheelCell->GetLeftStage(),TEXT("공급 이송 예정 거리"),TEXT("cm"),
			FVector::Distance(SupplyTravelFrom[Index],SupplyTravelTo[Index]),
			FVector::Distance(SupplyTravelFrom[Index],SupplyTravelTo[Index]),
			TEXT("예비 위치에서 공급 위치로 이송 시작"),TEXT("observed"));
		Record(TEXT("SPARE_DISPATCHED"),FString::Printf(TEXT("wheel %d moving from reserve to pickup"),Index+1));
	}
	const double Progress=FMath::Clamp((SensorTime-SupplyTravelStartedAt[Index])/SupplyTravelSeconds,0.0,1.0);
	const FVector Desired=FMath::Lerp(SupplyTravelFrom[Index],SupplyTravelTo[Index],Progress);
	WheelCell->DisplaceLeftWheel(Index,Desired-WheelCell->GetWheelWorld(Index).GetLocation());
	ActiveEvent=FString::Printf(TEXT("예비 타이어 %d 공급 이송 %.0f%% → 도착 센서 확인"),Index+1,Progress*100.0);
	// The wheel sensor samples after this plant step. Only its next fresh pickup-zone
	// observation can release the robot from WaitSupply.
	if (Progress>=1.0 && WheelCell->LeftSensors->Presence.Fresh(WheelCell->SimulationTimeSeconds,WheelCell->LeftSensors->MaxAge)
		&& WheelCell->LeftSensors->Presence.bDetected)
		RestoreEvent(Index);
}

void AAutonomousAssemblyLine::RestoreEvent(int32 Index)
{
	if (!WheelCell || !WheelCell->LeftSensors) return;
	switch (Planned[Index])
	{
	case ERandomWheelEvent::SupplyMissing:
		if (!bSupplyArrivalRecorded[Index])
		{
			WheelCell->OpenIncident(TEXT("spare_arrived"),Index,WheelCell->GetLeftStage(),TEXT("공급 위치 오차"),TEXT("cm"),
				FVector::Distance(WheelCell->GetWheelWorld(Index).GetLocation(),WheelCell->GetPickupWorld(Index).GetLocation()),
				8.0,TEXT("공급 위치 센서 검출 · 휠 규격/자세 재검사 대기"),TEXT("observed"));
			bSupplyArrivalRecorded[Index]=true;
			ActiveEvent=FString::Printf(TEXT("타이어 %d 공급 위치 도착 → 센서·규격 재검사 대기"),Index+1);
			return;
		}
		// Arrival alone is not a recovered assembly. The cell must close the missing-supply
		// incident and leave WaitSupply after its own fresh sensor and size checks.
		if (WheelCell->GetLeftStage()==EWheelStage::WaitSupply
			|| WheelCell->CountIncidents(TEXT("supply_missing"),TEXT("pending"))>0) return;
		ActiveEvent=FString::Printf(TEXT("타이어 %d 센서·규격 재검사 완료 → 로봇 재개"),Index+1);
		break;
	case ERandomWheelEvent::PickupPoseDropout:
		WheelCell->LeftSensors->bPublishPickupPose = true;
		ActiveEvent = FString::Printf(TEXT("타이어 %d 위치 센서 재연결 → 새 값 대기"),Index+1);
		break;
	case ERandomWheelEvent::GripFailure:
		WheelCell->LeftSensors->bGripActuatorFault = false;
		ActiveEvent = FString::Printf(TEXT("타이어 %d 그리퍼 복구 → 재시도"),Index+1);
		break;
	default: return;
	}
	bRestored[Index] = true; ++RecoveredEvents;
	Record(TEXT("PLANT_RECOVERED"),ActiveEvent);
}

void AAutonomousAssemblyLine::BeforeSensorStep(double Dt)
{
	if (!WheelCell || !WheelCell->LeftSensors || ArrivalPhase!=EArrivalPhase::Released || Stage!=EAssemblyLineStage::Wheels) return;
	for (int32 Index=0; Index<2; ++Index)
		if (bInjected[Index] && !bRestored[Index] && RestoreAt[Index]>=0 && SensorTime>=RestoreAt[Index])
		{
			if (Planned[Index]==ERandomWheelEvent::SupplyMissing) AdvanceSupplyDelivery(Index);
			else RestoreEvent(Index);
		}
	const int32 Index = WheelCell->GetActiveLeftWheelIndex();
	if (Index<0 || Index>1 || bInjected[Index]) return;
	const EWheelStage ArmStage = WheelCell->GetLeftStage();
	if (ArmStage!=EWheelStage::ApproachPickup && ArmStage!=EWheelStage::Grip) return;
	if (FirstSeenAt[Index]<0) FirstSeenAt[Index]=SensorTime;
	if (SensorTime-FirstSeenAt[Index]<TriggerAfter[Index]) return;
	if (Planned[Index]==ERandomWheelEvent::GripFailure
		&& (ArmStage!=EWheelStage::Grip || !WheelCell->LeftSensors->IsGripCommandedClosed())) return;
	InjectEvent(Index);
}

FString AAutonomousAssemblyLine::GetSummary() const
{
	return FString::Printf(TEXT("자율 복구 조립 실험 · 시드 %d · 실행 %d\n%s\n\n발생 %d/2 | 복구 %d/2\n%s\n%s\n\nEnter 시작 / Space 정지 / R 새 난수 실행\nF2 도킹 센서 / F3 왼팔 추적"),
		CurrentSeed,RunNumber,*Status,InjectedEvents,RecoveredEvents,*ActiveEvent,
		WheelCell ? *WheelCell->GetLeftSensorSummary() : TEXT("로봇 센서 없음"));
}

void AAutonomousAssemblyLine::Tick(float Seconds)
{
	Super::Tick(Seconds);
	if (!bShowSensorVisualization || !GetWorld() ||
		(GetWorld()->WorldType!=EWorldType::PIE && GetWorld()->WorldType!=EWorldType::Game)) return;
	for (int32 Index=0;Index<2;++Index)
	{
		if (!bInjected[Index] || bRestored[Index] || Planned[Index]!=ERandomWheelEvent::SupplyMissing) continue;
		const FVector Start=SupplyTravelFrom[Index], End=SupplyTravelTo[Index];
		DrawDebugLine(GetWorld(),Start,End,FColor::Cyan,false,.08f,0,3.f);
		DrawDebugSphere(GetWorld(),Start,12,12,FColor::Cyan,false,.08f,0,2.f);
		DrawDebugSphere(GetWorld(),End,12,12,FColor::Green,false,.08f,0,2.f);
		DrawDebugString(GetWorld(),End+FVector(0,0,60),
			FString::Printf(TEXT("SPARE WHEEL %d: %s"),Index+1,
				SupplyTravelStartedAt[Index]<0?TEXT("DISPATCH WAIT"):TEXT("MOVING TO SUPPLY SENSOR")),
			nullptr,FColor::Cyan,.08f,false,1.f);
	}
}

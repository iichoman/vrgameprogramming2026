#include "AutonomousAssemblyLine.h"
#include "WheelAssemblyCell.h"

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
	CurrentSeed = BaseSeed + RunNumber++;
	Random.Initialize(CurrentSeed);
	InjectedEvents = RecoveredEvents = 0;
	ActiveEvent = TEXT("첫 타이어 공급 측정 대기");
	for (int32 Index=0; Index<2; ++Index)
	{
		Planned[Index] = ERandomWheelEvent(Random.RandRange(0,3));
		FirstSeenAt[Index] = RestoreAt[Index] = -1;
		TriggerAfter[Index] = Random.FRandRange(.1,.4);
		bInjected[Index] = bRestored[Index] = false;
	}
	Record(TEXT("RANDOM_SEED"),FString::FromInt(CurrentSeed));
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

void AAutonomousAssemblyLine::RestoreEvent(int32 Index)
{
	if (!WheelCell || !WheelCell->LeftSensors) return;
	switch (Planned[Index])
	{
	case ERandomWheelEvent::SupplyMissing:
		WheelCell->SetLeftWheelSupplyVisible(Index,true);
		ActiveEvent = FString::Printf(TEXT("타이어 %d 자동 보충 완료 → 센서 재확인"),Index+1);
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
			RestoreEvent(Index);
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

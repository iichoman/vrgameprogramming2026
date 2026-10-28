#include "SensorAssemblyLine.h"
#include "WheelAssemblyCell.h"
#include "RobotArm.h"
#include "CamryAssembly.h"
#include "Components/SceneComponent.h"
#include "EngineUtils.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "DrawDebugHelpers.h"

namespace
{
bool Within(const FVector& P,const FVector& Extent) { return FMath::Abs(P.X)<=Extent.X && FMath::Abs(P.Y)<=Extent.Y && FMath::Abs(P.Z)<=Extent.Z; }
void Publish(FLineSensorReading& R, FName Id, FVector Value, double Now, bool Valid=true)
{ R.SensorId=Id; R.Value=Value; R.MeasuredAt=Now; R.bValid=Valid; }
}
void UArrivalSensorBank::Reset(AActor* InVehicle,const FTransform& InStation)
{
    Vehicle=InVehicle; Station=InStation; LastSample=-1; bPublishPosition=true;
    Occupancy=Arrival=Position=Speed=FLineSensorReading();
    Sample(0);
}
bool UArrivalSensorBank::Healthy(double Now) const
{ return Occupancy.Fresh(Now,MaxAge) && Arrival.Fresh(Now,MaxAge) && Position.Fresh(Now,MaxAge) && Speed.Fresh(Now,MaxAge); }
void UArrivalSensorBank::Sample(double Now)
{
    if (LastSample>=0 && Now-LastSample+1.e-9<SamplePeriod) return;
    if (!IsValid(Vehicle)) { Occupancy.bValid=Arrival.bValid=Position.bValid=Speed.bValid=false; return; }
    const FVector Local=Station.InverseTransformPosition(Vehicle->GetActorLocation());
    const bool Observed=Within(Local,ObservationHalfExtentCm);
    const FVector MeasuredSpeed=LastSample>=0 ? (Local-PreviousPosition)/(Now-LastSample) : FVector::ZeroVector;
    bool Occupied=Within(Local,FVector(250,150,150));
    for (TActorIterator<AActor> It(GetWorld());It;++It)
        if (*It!=Vehicle && It->ActorHasTag(TEXT("SensorWorkpiece")) && Within(Station.InverseTransformPosition(It->GetActorLocation()),FVector(250,150,150))) Occupied=true;
    Publish(Occupancy,TEXT("ZONE_OCCUPIED"),FVector(Occupied?1:0,0,0),Now);
    Publish(Arrival,TEXT("ARRIVAL_SWITCH"),FVector(Within(Local,ArrivalHalfExtentCm)?1:0,0,0),Now);
    if (bPublishPosition) Publish(Position,TEXT("BODY_POSITION"),Local,Now,Observed);
    Publish(Speed,TEXT("BODY_SPEED"),MeasuredSpeed,Now,Observed && LastSample>=0);
    PreviousPosition=Local; LastSample=Now;
}
void UConveyorPlant::Reset(AActor* InVehicle,AActor* InCarriage,FVector InTarget)
{ Vehicle=InVehicle; Carriage=InCarriage; Target=InTarget; Velocity=FVector::ZeroVector; bRun=false; bJammed=false; }
void UConveyorPlant::Step(double Dt)
{
    if (!Vehicle || !Carriage) return;
    if (bJammed) { Velocity=FVector::ZeroVector; return; }
    const FVector Error=Target-Vehicle->GetActorLocation();
    const double Remaining=Error.Size();
    const double DesiredSpeed=bRun ? FMath::Min(CruiseSpeedCmS,FMath::Sqrt(2*AccelerationCmS2*Remaining)) : 0;
    const FVector Desired=Error.GetSafeNormal()*DesiredSpeed;
    Velocity+=(Desired-Velocity).GetClampedToMaxSize(AccelerationCmS2*Dt);
    FVector Delta=Velocity*Dt;
    if (bRun && Delta.Size()>=Remaining) { Delta=Error; Velocity=FVector::ZeroVector; }
    Vehicle->AddActorWorldOffset(Delta,false);
    Carriage->AddActorWorldOffset(Delta,false);
}
ASensorAssemblyLine::ASensorAssemblyLine()
{
    Sensors=CreateDefaultSubobject<UArrivalSensorBank>(TEXT("ArrivalSensors"));
    Conveyor=CreateDefaultSubobject<UConveyorPlant>(TEXT("ConveyorPlant"));
}
void ASensorAssemblyLine::Record(const FString& Kind,const FString& Message)
{
    const FString Row=FString::Printf(TEXT("%.4f,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%s\n"),SensorTime,*Kind,*Message,
        Sensors->Position.Value.X,Sensors->Position.Value.Y,Sensors->Position.Value.Z,Sensors->Speed.Value.Size(),
        SensorTime-Sensors->Position.MeasuredAt,Sensors->Healthy(SensorTime)?1:0,
        Sensors->Arrival.Value.X>.5?1:0,Sensors->Occupancy.Value.X>.5?1:0,int32(ArrivalPhase),
		WheelCell ? *WheelCell->GetLeftSensorSummary() : TEXT("no wheel cell"));
    if (!EventLog.IsEmpty()) FFileHelper::SaveStringToFile(Row,*EventLog,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);
}
bool ASensorAssemblyLine::ResetLine()
{
    if (Conveyor) Conveyor->CommandRun(false);
    if (WheelCell) WheelCell->bEnableTransport=true;
	if (WheelCell) WheelCell->bSensorDrivenLeftRobot=true;
    if (!Super::ResetLine()) return false;
    WheelCell->bEnableTransport=false;
    SensorTime=Remainder=RunTime=StableTime=NoProgressTime=0;
    bInjected=bStopRecorded=bLoggedComplete=false; LastLeftRobotState.Reset();
    ArrivalPhase=EArrivalPhase::Ready;
    Sensors->Reset(WheelCell->Assembly,VehicleAtWheelStation);
    Conveyor->Reset(WheelCell->Assembly,WheelCell->TransportCarriage,VehicleAtWheelStation.GetLocation());
    // Initial velocity requires two samples; Start waits for valid measurements.
    EventLog=FPaths::Combine(FPaths::ProjectSavedDir(),TEXT("SensorLineRuns"),FGuid::NewGuid().ToString(EGuidFormats::Digits)+TEXT(".csv"));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(EventLog),true);
    FFileHelper::SaveStringToFile(TEXT("time_s,event,reason,x_cm,y_cm,z_cm,speed_cm_s,position_age_s,sensors_healthy,arrival,occupied,phase,left_robot_sensor_state\n"),*EventLog);
    Status=TEXT("센서 갱신 대기 · 측정값 확인 후 시작");
    Record(TEXT("RESET"),TEXT("reset to upstream"));
    return true;
}
bool ASensorAssemblyLine::SelectDemo(EArrivalDemo Selection)
{
    if (ArrivalPhase==EArrivalPhase::Moving || ArrivalPhase==EArrivalPhase::Docking || (ArrivalPhase==EArrivalPhase::Released && Stage!=EAssemblyLineStage::Complete && Stage!=EAssemblyLineStage::Fault)) return false;
    Demo=Selection; return ResetLine();
}
bool ASensorAssemblyLine::StartLine()
{
    if (ArrivalPhase!=EArrivalPhase::Ready || !bReady) return false;
    if (!Sensors->Healthy(SensorTime)) { Status=TEXT("시작 대기: 센서 무효 또는 갱신 지연"); return false; }
    if (Sensors->Occupancy.Value.X>.5) { Status=TEXT("시작 차단: 작업 구역 점유"); Record(TEXT("START_DENIED"),TEXT("zone occupied")); return false; }
    if (Sensors->Speed.Value.Size()>StopSpeedCmS) { Status=TEXT("시작 대기: 차체가 움직이고 있음"); return false; }
    ArrivalPhase=EArrivalPhase::Moving;
    Conveyor->CommandRun(true);
    Status=TEXT("반입 중 · 도착 센서 대기");
    Record(TEXT("RUN_COMMAND"),TEXT("zone clear and sensors fresh"));
    return true;
}
void ASensorAssemblyLine::StopLine()
{
    if (ArrivalPhase==EArrivalPhase::Released) { Super::StopLine(); Record(TEXT("STOP_COMMAND"),TEXT("operator during assembly")); return; }
    if (ArrivalPhase!=EArrivalPhase::Moving && ArrivalPhase!=EArrivalPhase::Docking) return;
    Conveyor->CommandRun(false);
    ArrivalPhase=EArrivalPhase::Stopped; bReady=false;
    Status=TEXT("운전자 정지 명령 · 감속 후 정지, R 초기화 필요");
    Record(TEXT("STOP_COMMAND"),TEXT("operator"));
}
void ASensorAssemblyLine::SensorFault(const FString& Reason)
{
    Conveyor->CommandRun(false);
    ArrivalPhase=EArrivalPhase::Fault;
    Fail(Reason);
    Record(TEXT("FAULT_STOP_COMMAND"),Reason);
}
void ASensorAssemblyLine::FixedStep(double Dt)
{
    SensorTime+=Dt;
	BeforeSensorStep(Dt);
    // Injection only changes the plant or sensor publishing. Decision code below
    // never reads the demo selection or injection flag.
    if (ArrivalPhase==EArrivalPhase::Moving && !bInjected && RunTime>=2)
    {
        bInjected=true;
        switch(Demo)
        {
        case EArrivalDemo::Jam: Conveyor->bJammed=true; Record(TEXT("INJECT"),TEXT("drive jam")); break;
        case EArrivalDemo::MisDock: Conveyor->Target+=FVector(0,1,0); Record(TEXT("INJECT"),TEXT("drive target lateral offset 1 cm")); break;
        case EArrivalDemo::PositionDropout: Sensors->bPublishPosition=false; Record(TEXT("INJECT"),TEXT("position packets dropped")); break;
        default: break;
        }
    }
    if (ArrivalPhase!=EArrivalPhase::Released)
    {
        Conveyor->Step(Dt);
        if (WheelCell && WheelCell->Assembly) WheelCell->Assembly->GetLeftDoor()->SetWorldTransform(DoorSupplyWorld);
    }
    const double PreviousSample=Sensors->Arrival.MeasuredAt;
    Sensors->Sample(SensorTime);
    if (Sensors->Arrival.MeasuredAt!=PreviousSample && ArrivalPhase!=EArrivalPhase::Released) Record(TEXT("SAMPLE"),TEXT("sensor update"));
    if (ArrivalPhase==EArrivalPhase::Stopped || ArrivalPhase==EArrivalPhase::Fault)
    {
        if (!bStopRecorded && Sensors->Speed.Fresh(SensorTime,Sensors->MaxAge) && Sensors->Speed.Value.Size()<=StopSpeedCmS)
        { bStopRecorded=true; Record(TEXT("STOP_CONFIRMED"),TEXT("measured speed settled")); }
        return;
    }
    if (ArrivalPhase==EArrivalPhase::Ready) { Status=Sensors->Healthy(SensorTime)?TEXT("준비 · 센서 정상, 시작 가능"):TEXT("센서 갱신 대기"); return; }
    if (ArrivalPhase==EArrivalPhase::Released)
    {
        Super::AdvanceLine(Dt);
		if (WheelCell && Stage==EAssemblyLineStage::Wheels)
		{
			const FString State=WheelCell->GetLaneStage(true);
			if (State!=LastLeftRobotState) { LastLeftRobotState=State; Record(TEXT("ROBOT_DECISION"),State); }
		}
        if ((Stage==EAssemblyLineStage::Complete || Stage==EAssemblyLineStage::Fault) && !bLoggedComplete)
        { bLoggedComplete=true; Record(TEXT("ASSEMBLY_RESULT"),Status); }
        return;
    }
    RunTime+=Dt;
    if (!Sensors->Healthy(SensorTime)) { SensorFault(TEXT("센서 신호 무효 또는 0.25초 이상 갱신 끊김")); return; }
    const bool Arrived=Sensors->Arrival.Value.X>.5;
    const double Speed=Sensors->Speed.Value.Size();
    if (ArrivalPhase==EArrivalPhase::Moving)
    {
        if (Arrived)
        {
            Conveyor->CommandRun(false); ArrivalPhase=EArrivalPhase::Docking;
            Status=TEXT("도착 감지 · 정지 명령, 위치와 속도 확인 중");
            Record(TEXT("ARRIVAL_STOP_COMMAND"),TEXT("arrival switch detected"));
        }
        else
        {
            NoProgressTime=Speed<=StopSpeedCmS ? NoProgressTime+Dt : 0;
            if (RunTime>1 && NoProgressTime>1.5) { SensorFault(TEXT("이송 정체: 이동 명령 중 측정 속도가 1.5초간 정지")); return; }
            if (RunTime>ArrivalTimeoutSeconds) { SensorFault(TEXT("도착 제한시간 초과")); return; }
        }
    }
    else if (ArrivalPhase==EArrivalPhase::Docking)
    {
        // The arrival switch is a momentary trigger. The position and speed
        // sensors confirm docking after the conveyor finishes braking.
        const bool Aligned=Sensors->Position.Value.Size()<=DockToleranceCm && Speed<=StopSpeedCmS;
        StableTime=Aligned ? StableTime+Dt : 0;
        Status=FString::Printf(TEXT("도킹 확인 · 위치 %.1f mm / 속도 %.1f mm/s · 안정 %.2f/%.2f초"),Sensors->Position.Value.Size()*10,Speed*10,StableTime,StableSeconds);
        if (StableTime+1.e-9>=StableSeconds)
        {
            Record(TEXT("DOCK_ACCEPTED"),TEXT("arrival position speed stable"));
            if (!Super::StartLine()) { SensorFault(Status); return; }
            ArrivalPhase=EArrivalPhase::Released;
            Record(TEXT("WHEEL_START_COMMAND"),TEXT("sensor gate released"));
			switch(Demo)
			{
			case EArrivalDemo::WheelMissing: WheelCell->InjectLeftWheelMissing(); Record(TEXT("INJECT"),TEXT("left wheel removed from rack")); break;
			case EArrivalDemo::WheelDisplaced: WheelCell->InjectLeftWheelDisplacement(FVector(0,-4,0)); Record(TEXT("INJECT"),TEXT("left wheel shifted 4 cm")); break;
			case EArrivalDemo::GripFailure: WheelCell->LeftSensors->bGripActuatorFault=true; Record(TEXT("INJECT"),TEXT("left gripper cannot latch")); break;
			case EArrivalDemo::WheelPoseDropout: WheelCell->LeftSensors->bPublishPickupPose=false; Record(TEXT("INJECT"),TEXT("left wheel pose packets dropped")); break;
			default: break;
			}
        }
        else if (RunTime>ArrivalTimeoutSeconds) SensorFault(TEXT("도킹 불합격: 위치 또는 정지 조건 미충족"));
    }
}
bool ASensorAssemblyLine::AdvanceLine(double Seconds)
{
    if (!FMath::IsFinite(Seconds) || Seconds<0 || Seconds>1) { StopLine(); return false; }
    Remainder+=Seconds;
    while (Remainder+1.e-9>=1./120.) { Remainder-=1./120.; FixedStep(1./120.); }
    return ArrivalPhase!=EArrivalPhase::Fault && Stage!=EAssemblyLineStage::Fault;
}
void ASensorAssemblyLine::Tick(float Seconds)
{
    Super::Tick(Seconds);
    if (!bShowSensorVisualization || !Sensors || !GetWorld() || GetWorld()->WorldType!=EWorldType::PIE) return;
    const FVector Station=Sensors->Station.GetLocation();
    const bool Fresh=Sensors->Healthy(SensorTime);
    const FColor Color=Fresh ? (Sensors->Arrival.Value.X>.5 ? FColor::Green : FColor::Cyan) : FColor::Red;
    DrawDebugBox(GetWorld(),Station,FVector(250,150,150),FColor::Cyan,false,.08f,0,1.5f);
    DrawDebugLine(GetWorld(),Station,Station+FVector(0,0,180),Color,false,.08f,0,2.f);
    DrawDebugSphere(GetWorld(),Station+FVector(0,0,180),12,12,Color,false,.08f,0,2.f);
    DrawDebugString(GetWorld(),Station+FVector(0,0,205),Fresh ? TEXT("DOCK SENSOR: LIVE") : TEXT("DOCK SENSOR: STALE"),nullptr,Color,.08f,false,1.f);
    if (Sensors->Position.Fresh(SensorTime,Sensors->MaxAge))
        DrawDebugSphere(GetWorld(),Sensors->Station.TransformPosition(Sensors->Position.Value),9,10,FColor::Yellow,false,.08f,0,2.f);
	if (ArrivalPhase==EArrivalPhase::Released && WheelCell && WheelCell->bSensorDrivenLeftRobot
		&& WheelCell->LeftSensors && IsValid(WheelCell->LeftRobot))
	{
		const UWheelRobotSensorBank* ArmSensors=WheelCell->LeftSensors;
		const double Now=WheelCell->SimulationTimeSeconds;
		const FVector TCP=WheelCell->LeftRobot->GetTCPWorld().GetLocation();
		const bool Present=ArmSensors->Presence.Fresh(Now,ArmSensors->MaxAge) && ArmSensors->Presence.bDetected;
		const bool Gripped=ArmSensors->Gripped.Fresh(Now,ArmSensors->MaxAge) && ArmSensors->Gripped.bDetected;
		const FColor ArmColor=!Present ? FColor::Red : Gripped ? FColor::Green : FColor::Yellow;
		DrawDebugSphere(GetWorld(),TCP,9,10,ArmColor,false,.08f,0,2.f);
		DrawDebugString(GetWorld(),TCP+FVector(0,0,45),
			FString::Printf(TEXT("LEFT SENSOR: wheel %s | grip %s"),Present?TEXT("YES"):TEXT("NO"),Gripped?TEXT("YES"):TEXT("NO")),
			nullptr,ArmColor,.08f,false,1.f);
		if (ArmSensors->PickupPose.Fresh(Now,ArmSensors->MaxAge))
		{
			const FVector Measured=ArmSensors->PickupPose.Pose.GetLocation();
			DrawDebugSphere(GetWorld(),Measured,10,10,FColor::Yellow,false,.08f,0,2.f);
			DrawDebugLine(GetWorld(),TCP,Measured,ArmColor,false,.08f,0,1.f);
		}
	}
}
FString ASensorAssemblyLine::GetSummary() const
{
    const TCHAR* Names[]={TEXT("정상"),TEXT("이송 멈춤"),TEXT("도킹 위치 틀어짐"),TEXT("위치 센서 끊김"),TEXT("타이어 없음"),TEXT("타이어 위치 틀어짐"),TEXT("파지 실패"),TEXT("타이어 위치 센서 끊김")};
    return FString::Printf(TEXT("센서 기반 차체 반입 · %s\n%s\n\n도착 %s | 구역 점유 %s | 측정 %s\n위치 오차 X %.1f / Y %.1f / Z %.1f mm\n속도 %.1f mm/s | 위치값 나이 %.2f초\n타이어 %d/4 | 반입 경과 %.1f초\n%s\n\n1~8 상황 선택 / Enter 시작 / Space 정지 / R 초기화"),Names[int32(Demo)],*Status,
        Sensors->Arrival.Value.X>.5?TEXT("감지"):TEXT("미감지"),Sensors->Occupancy.Value.X>.5?TEXT("있음"):TEXT("없음"),
        Sensors->Healthy(SensorTime)?TEXT("정상"):TEXT("무효·오래됨"),Sensors->Position.Value.X*10,Sensors->Position.Value.Y*10,Sensors->Position.Value.Z*10,
        Sensors->Speed.Value.Size()*10,SensorTime-Sensors->Position.MeasuredAt,WheelCell?WheelCell->MountedCount:0,RunTime,
		WheelCell ? *WheelCell->GetLeftSensorSummary() : TEXT("왼팔 센서 없음"));
}

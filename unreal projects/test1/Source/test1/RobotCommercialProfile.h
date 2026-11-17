#pragma once
#include "RobotArm.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

/** Data-backed reference envelope, deliberately separate from imported FK/TCP.
 * The numeric source is commercial; the mapping to this unidentified mesh is
 * provisional. No physical payload/accuracy capability is inferred from it. */
namespace RobotCommercialProfile
{
inline bool Apply(ARobotArm& Arm,FString& Error)
{
    // Only the imported demo calibration opts in. Synthetic tests and other
    // equipment retain their explicitly supplied joint definitions.
    if(!Arm.CalibrationSource.StartsWith(TEXT("robot_calibration.json:")))return true;
    FString Text;TSharedPtr<FJsonObject> Root;
    const FString Path=FPaths::ProjectConfigDir()/TEXT("RobotMotionReference.json");
    if(!FFileHelper::LoadFileToString(Text,*Path)||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Root)||!Root)
    {Error=TEXT("로봇 참조 설정을 읽을 수 없습니다: Config/RobotMotionReference.json");return false;}
    bool Enabled=false;if(!Root->TryGetBoolField(TEXT("enabled"),Enabled)){Error=TEXT("로봇 참조 설정 enabled 값 오류");return false;}if(!Enabled)return true;
    const TArray<TSharedPtr<FJsonValue>>* Axes=nullptr;double Scale=0;FString Model,Prefix;
    if(!Root->TryGetArrayField(TEXT("axes"),Axes)||Axes->Num()!=6||Arm.Joints.Num()!=6
        ||!Root->TryGetNumberField(TEXT("speed_override"),Scale)||!FMath::IsFinite(Scale)||Scale<=0||Scale>1
        ||!Root->TryGetStringField(TEXT("reference_model"),Model)
        ||!Root->TryGetStringField(TEXT("apply_to_calibration_source_prefix"),Prefix)||Prefix.IsEmpty())
    {Error=TEXT("로봇 참조 설정의 축/속도 값 오류");return false;}
    if(!Arm.CalibrationSource.StartsWith(Prefix))return true;
    auto Updated=Arm.Joints;
    for(int I=0;I<6;++I)
    {
        const auto A=(*Axes)[I]->AsObject();double Lo=0,Hi=0,Speed=0,Sign=0,Zero=0;FString Name;bool ApplyRange=false;
        if(!A||!A->TryGetStringField(TEXT("name"),Name)||Name!=FString::Printf(TEXT("A%d"),I+1)
            ||!A->TryGetNumberField(TEXT("minimum_deg"),Lo)||!A->TryGetNumberField(TEXT("maximum_deg"),Hi)
            ||!A->TryGetNumberField(TEXT("rated_speed_deg_s"),Speed)||!A->TryGetNumberField(TEXT("native_sign"),Sign)
            ||!A->TryGetNumberField(TEXT("reference_zero_deg"),Zero)||!A->TryGetBoolField(TEXT("apply_position_range"),ApplyRange)||!FMath::IsFinite(Lo)||!FMath::IsFinite(Hi)
            ||!FMath::IsFinite(Speed)||!FMath::IsFinite(Zero)||Lo>=Hi||Speed<=0||(Sign!=1&&Sign!=-1))
        {Error=FString::Printf(TEXT("로봇 참조 설정 A%d 오류"),I+1);return false;}
        const double A0=(Lo-Zero)/Sign,A1=(Hi-Zero)/Sign;
        if(ApplyRange){Updated[I].MinimumDegrees=FMath::Min(A0,A1);Updated[I].MaximumDegrees=FMath::Max(A0,A1);}
        Updated[I].MaxVelocityDegrees=Speed*Scale;
        // Acceleration remains the explicit existing simulation assumption;
        // the published axis-speed table does not specify acceleration/jerk.
    }
    Arm.Joints=MoveTemp(Updated);
    const FString Marker=TEXT(" [Motion reference:");int At=Arm.CalibrationSource.Find(Marker);
    if(At!=INDEX_NONE)Arm.CalibrationSource.LeftInline(At);
    Arm.CalibrationSource+=FString::Printf(TEXT(" [Motion reference: %s, speed %.0f%%; A2/A3 range pending mastering; mesh identity/TCP unverified.]"),*Model,Scale*100);
    return true;
}
}

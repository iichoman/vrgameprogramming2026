#include "FactoryLayoutModel.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
bool Finite2(const FVector2D& V)
{
    return FMath::IsFinite(V.X) && FMath::IsFinite(V.Y);
}

bool ValidRole(const FString& Role)
{
    return Role == TEXT("work") || Role == TEXT("supply") || Role == TEXT("buffer")
        || Role == TEXT("walkway") || Role == TEXT("rework") || Role == TEXT("reserve");
}

bool ValidId(const FString& Id)
{
    if (Id.IsEmpty() || Id.Len() > 64) return false;
    for (const TCHAR C : Id)
    {
        if (!((C >= TEXT('A') && C <= TEXT('Z')) || (C >= TEXT('a') && C <= TEXT('z'))
            || (C >= TEXT('0') && C <= TEXT('9')) || C == TEXT('_') || C == TEXT('-'))) return false;
    }
    return true;
}

bool ValidLabel(const FString& Label)
{
    if (Label.TrimStartAndEnd().IsEmpty() || Label.Len() > 96) return false;
    for (const TCHAR C : Label) if (C < 32 || C == 127) return false;
    return true;
}

bool GeometryValid(const FFactoryLayoutItem& Item)
{
    return Finite2(Item.Position) && Finite2(Item.Size) && FMath::IsFinite(Item.Yaw)
        && Item.Size.X > 0 && Item.Size.Y > 0;
}

void Axes(const FFactoryLayoutItem& Item, FVector2D& X, FVector2D& Y)
{
    const double Angle = FMath::DegreesToRadians(FMath::UnwindDegrees(Item.Yaw));
    X = FVector2D(FMath::Cos(Angle), FMath::Sin(Angle));
    Y = FVector2D(-X.Y, X.X);
}

const TSharedPtr<FJsonValue>* Field(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, EJson Type)
{
    if (!Object.IsValid()) return nullptr;
    const TSharedPtr<FJsonValue>* Value = Object->Values.Find(Name);
    return Value && Value->IsValid() && (*Value)->Type == Type ? Value : nullptr;
}

bool ReadNumber(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, double& Out)
{
    const TSharedPtr<FJsonValue>* Value = Field(Object, Name, EJson::Number);
    if (!Value) return false;
    const double Number = (*Value)->AsNumber();
    if (!FMath::IsFinite(Number)) return false;
    Out = Number;
    return true;
}

bool ReadString(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FString& Out)
{
    const TSharedPtr<FJsonValue>* Value = Field(Object, Name, EJson::String);
    if (!Value) return false;
    Out = (*Value)->AsString();
    return true;
}

bool ReadVector(const TSharedPtr<FJsonObject>& Object, const TCHAR* Name, FVector2D& Out)
{
    const TSharedPtr<FJsonValue>* Value = Field(Object, Name, EJson::Array);
    if (!Value) return false;
    const TArray<TSharedPtr<FJsonValue>>& Array = (*Value)->AsArray();
    if (Array.Num() != 2) return false;
    for (const TSharedPtr<FJsonValue>& Element : Array)
        if (!Element.IsValid() || Element->Type != EJson::Number || !FMath::IsFinite(Element->AsNumber())) return false;
    Out = FVector2D(Array[0]->AsNumber(), Array[1]->AsNumber());
    return true;
}

TArray<TSharedPtr<FJsonValue>> JsonVector(const FVector2D& Vector)
{
    TArray<TSharedPtr<FJsonValue>> Result;
    Result.Add(MakeShared<FJsonValueNumber>(Vector.X));
    Result.Add(MakeShared<FJsonValueNumber>(Vector.Y));
    return Result;
}
}

FString FFactoryLayoutItem::EffectiveTypeId() const { return TypeId.IsEmpty() ? FFactoryLayout::KindName(Kind) : TypeId; }

FFactoryLayout FFactoryLayout::LegacyDefault()
{
    FFactoryLayout Layout;
    auto Add = [&Layout](const TCHAR* Id, const TCHAR* Ko, const TCHAR* En, const TCHAR* Role,
        EFactoryItemKind Kind, FVector2D Position, FVector2D Size)
    {
        FFactoryLayoutItem Item;
        Item.Id = Id; Item.NameKo = Ko; Item.NameEn = En; Item.Role = Role;
        Item.Kind = Kind; Item.Position = Position; Item.Size = Size;
        Layout.Items.Add(MoveTemp(Item));
    };
    const TCHAR* ProcessKo[] = { TEXT("차체 입고·준비"), TEXT("내장 조립"), TEXT("하부 결합"),
        TEXT("휠 작업"), TEXT("마무리 조립"), TEXT("최종 검사·출구") };
    const TCHAR* ProcessEn[] = { TEXT("Inbound / Preparation"), TEXT("Trim Assembly"), TEXT("Underbody Marriage"),
        TEXT("Wheel Assembly"), TEXT("Finish Assembly"), TEXT("Inspection / Outbound") };
    for (int32 Index = 0; Index < 6; ++Index)
    {
        const FString Id = FString::Printf(TEXT("P%02d"), Index + 1);
        Layout.ProcessOrder.Add(Id);
        Add(*Id, ProcessKo[Index], ProcessEn[Index], TEXT("work"), EFactoryItemKind::Zone,
            FVector2D(600 + Index * 1200, 0), FVector2D(1160, 1040));
    }
    const TCHAR* SupplyKo[] = { TEXT("부품 입고·준비"), TEXT("콕핏 사전 조립·공급"), TEXT("구동계 모듈 공급"),
        TEXT("휠 공급"), TEXT("마무리 부품 공급"), TEXT("완성차 대기") };
    const TCHAR* SupplyEn[] = { TEXT("Receiving / Kitting"), TEXT("Cockpit Preparation / Supply"), TEXT("Powertrain Supply"),
        TEXT("Wheel Supply"), TEXT("Finish Supply"), TEXT("Outbound Buffer") };
    for (int32 Index = 0; Index < 6; ++Index)
    {
        const FString Id = FString::Printf(TEXT("S%02d"), Index + 1);
        Add(*Id, SupplyKo[Index], SupplyEn[Index], Index == 5 ? TEXT("buffer") : TEXT("supply"),
            EFactoryItemKind::Zone, FVector2D(600 + Index * 1200, 1300), FVector2D(1120, 800));
    }
    Add(TEXT("R01"), TEXT("도어·모듈 사전 조립 예정"), TEXT("Door / Module Preparation"), TEXT("reserve"),
        EFactoryItemKind::Zone, FVector2D(1200, -1300), FVector2D(2320, 800));
    Add(TEXT("R02"), TEXT("확장 예정"), TEXT("Expansion Reserve"), TEXT("reserve"),
        EFactoryItemKind::Zone, FVector2D(3600, -1300), FVector2D(2320, 800));
    Add(TEXT("R03"), TEXT("재작업·품질 확인"), TEXT("Rework / Quality"), TEXT("rework"),
        EFactoryItemKind::Zone, FVector2D(6000, -1300), FVector2D(2320, 800));
    Add(TEXT("station_1"), TEXT("차량 작업 위치"), TEXT("Vehicle Station"), TEXT("work"),
        EFactoryItemKind::VehicleStation, FVector2D(4200, 0), FVector2D(520, 220));
    Add(TEXT("supply_1"), TEXT("휠 공급대"), TEXT("Wheel Supply Rack"), TEXT("work"),
        EFactoryItemKind::WheelSupply, FVector2D(4345, -550), FVector2D(150, 150));
    Add(TEXT("handler_1"), TEXT("직교 이송 장치"), TEXT("Cartesian Handler"), TEXT("work"),
        EFactoryItemKind::CartesianHandler, FVector2D(4200, -330), FVector2D(90, 90));
    // Reserved conveyor footprint only. This layout does not imply a running belt or material transport.
    Add(TEXT("conveyor_1"), TEXT("컨베이어 배치 모형"), TEXT("Conveyor Placeholder"), TEXT("work"),
        EFactoryItemKind::Conveyor, FVector2D(3550, 0), FVector2D(600, 240));
    return Layout;
}

FString FFactoryLayout::KindName(EFactoryItemKind Kind)
{
    switch (Kind)
    {
    case EFactoryItemKind::Zone: return TEXT("zone");
    case EFactoryItemKind::VehicleStation: return TEXT("vehicle_station");
    case EFactoryItemKind::WheelSupply: return TEXT("wheel_supply");
    case EFactoryItemKind::CartesianHandler: return TEXT("cartesian_handler");
    case EFactoryItemKind::ArticulatedRobot: return TEXT("kuka_robot");
    case EFactoryItemKind::Conveyor: return TEXT("conveyor");
    default: return FString();
    }
}

bool FFactoryLayout::ParseKind(const FString& Name, EFactoryItemKind& Out)
{
    const EFactoryItemKind Kinds[] = { EFactoryItemKind::Zone, EFactoryItemKind::VehicleStation,
        EFactoryItemKind::WheelSupply, EFactoryItemKind::CartesianHandler, EFactoryItemKind::Conveyor, EFactoryItemKind::ArticulatedRobot };
    for (EFactoryItemKind Kind : Kinds) if (Name == KindName(Kind)) { Out = Kind; return true; }
    return false;
}

FFactoryLayoutItem* FFactoryLayout::Find(const FString& Id)
{
    return Items.FindByPredicate([&Id](const FFactoryLayoutItem& Item) { return Item.Id == Id; });
}

const FFactoryLayoutItem* FFactoryLayout::Find(const FString& Id) const
{
    return Items.FindByPredicate([&Id](const FFactoryLayoutItem& Item) { return Item.Id == Id; });
}

bool FFactoryLayout::Validate(FString& Error) const
{
    Error.Reset();
    if (!Finite2(HallOrigin) || FMath::Abs(HallOrigin.X)>100000 || FMath::Abs(HallOrigin.Y)>100000 || !FMath::IsFinite(RouteBendCm) || FMath::Abs(RouteBendCm)>2000)
    { Error=TEXT("공장 원점 또는 흐름 곡선 값이 유효하지 않습니다."); return false; }
    if (!FMath::IsFinite(LengthCm) || !FMath::IsFinite(WidthCm)
        || LengthCm < 1200 || LengthCm > 20000 || WidthCm < 1200 || WidthCm > 12000)
    {
        Error = TEXT("공장 크기는 길이 12~200m, 너비 12~120m의 유효한 수치여야 합니다."); return false;
    }
    if (Items.Num() > 128) { Error = TEXT("구역과 설비는 합계 128개까지 저장할 수 있습니다."); return false; }
    TSet<FString> Ids;
    for (const FFactoryLayoutItem& Item : Items)
    {
        if (!ValidId(Item.Id) || Ids.Contains(Item.Id))
        { Error = TEXT("항목 ID는 64자 이내 영문·숫자·밑줄·하이픈으로 작성하고 중복되지 않아야 합니다."); return false; }
        Ids.Add(Item.Id);
        if (!ValidLabel(Item.NameKo) || !ValidLabel(Item.NameEn))
        { Error = FString::Printf(TEXT("%s: 한국어 이름과 영어 이름을 각각 1~96자로 입력해 주세요."), *Item.Id); return false; }
        if ((!Item.TypeId.IsEmpty() && !ValidId(Item.TypeId)) || KindName(Item.Kind).IsEmpty() || !ValidRole(Item.Role)
            || (Item.Kind != EFactoryItemKind::Zone && Item.Role != TEXT("work")))
        { Error = FString::Printf(TEXT("%s: 지원하지 않는 항목 종류 또는 구역 용도입니다."), *Item.Id); return false; }
        const double MaxSize = Item.Kind == EFactoryItemKind::Zone ? 20000.0 : 3000.0;
        if (!GeometryValid(Item) || FMath::Abs(Item.Position.X) > 100000 || FMath::Abs(Item.Position.Y) > 100000
            || Item.Size.X < 20 || Item.Size.Y < 20 || Item.Size.X > MaxSize || Item.Size.Y > MaxSize
            || FMath::Abs(Item.Yaw) > 360000)
        { Error = FString::Printf(TEXT("%s: 위치·크기·회전 값이 유효한 범위를 벗어났습니다."), *Item.Id); return false; }
    }
    auto CheckLink = [this, &Error](const FString& Id, EFactoryItemKind Kind, const TCHAR* Label)
    {
        if (Id.IsEmpty()) return true; // An unassigned job is an editable, saveable layout.
        const FFactoryLayoutItem* Item = Find(Id);
        if (!Item || (Item->Kind != Kind && !(Kind == EFactoryItemKind::CartesianHandler && Item->Kind == EFactoryItemKind::ArticulatedRobot)))
        { Error = FString::Printf(TEXT("%s 연결 대상이 없거나 설비 종류가 맞지 않습니다."), Label); return false; }
        return true;
    };
    TSet<FString> RouteIds;
    for (const FString& Id : ProcessOrder)
    {
        const auto* Item=Find(Id);
        if (!Item || (Item->Kind!=EFactoryItemKind::Zone && Item->Kind!=EFactoryItemKind::VehicleStation) || RouteIds.Contains(Id))
        { Error=TEXT("공정 흐름에 없는 대상, 중복 대상 또는 작업장이 아닌 항목이 있습니다."); return false; }
        RouteIds.Add(Id);
    }
    return CheckLink(StationId, EFactoryItemKind::VehicleStation, TEXT("차량 작업 위치"))
        && CheckLink(SupplyId, EFactoryItemKind::WheelSupply, TEXT("휠 공급대"))
        && CheckLink(HandlerId, EFactoryItemKind::CartesianHandler, TEXT("이송 장치"));
}

FString FFactoryLayout::ToJson() const
{
    FString Error;
    if (!Validate(Error)) return FString();
    const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetNumberField(TEXT("version"), 2);
    Root->SetStringField(TEXT("units"), TEXT("cm"));
    const TSharedRef<FJsonObject> Hall = MakeShared<FJsonObject>();
    Hall->SetNumberField(TEXT("length_cm"), LengthCm); Hall->SetNumberField(TEXT("width_cm"), WidthCm);
    Hall->SetArrayField(TEXT("origin_cm"), JsonVector(HallOrigin));
    Root->SetObjectField(TEXT("hall"), Hall);
    TArray<TSharedPtr<FJsonValue>> Route;
    for(const FString& Id:ProcessOrder) Route.Add(MakeShared<FJsonValueString>(Id));
    Root->SetArrayField(TEXT("process_order"),Route);
    Root->SetNumberField(TEXT("route_bend_cm"),RouteBendCm);
    TArray<TSharedPtr<FJsonValue>> Array;
    for (const FFactoryLayoutItem& Item : Items)
    {
        const TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
        Object->SetStringField(TEXT("id"), Item.Id); Object->SetStringField(TEXT("name_ko"), Item.NameKo);
        Object->SetStringField(TEXT("name_en"), Item.NameEn); Object->SetStringField(TEXT("kind"), KindName(Item.Kind));
        Object->SetStringField(TEXT("type_id"), Item.EffectiveTypeId());
        Object->SetStringField(TEXT("role"), Item.Role); Object->SetArrayField(TEXT("position_cm"), JsonVector(Item.Position));
        Object->SetArrayField(TEXT("size_cm"), JsonVector(Item.Size)); Object->SetNumberField(TEXT("yaw_deg"), Item.Yaw);
        Array.Add(MakeShared<FJsonValueObject>(Object));
    }
    Root->SetArrayField(TEXT("items"), Array);
    const TSharedRef<FJsonObject> Job = MakeShared<FJsonObject>();
    Job->SetStringField(TEXT("station_id"), StationId); Job->SetStringField(TEXT("supply_id"), SupplyId);
    Job->SetStringField(TEXT("handler_id"), HandlerId); Root->SetObjectField(TEXT("job"), Job);
    FString Json;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Json);
    return FJsonSerializer::Serialize(Root, Writer) ? Json : FString();
}

bool FFactoryLayout::FromJson(const FString& Json, FFactoryLayout& Out, FString& Error)
{
    Error.Reset();
    if (Json.Len() > 2 * 1024 * 1024) { Error = TEXT("배치 파일이 너무 큽니다. 최대 2MB의 JSON만 읽을 수 있습니다."); return false; }
    TSharedPtr<FJsonObject> Root;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Root) || !Root.IsValid())
    { Error = TEXT("배치 파일을 읽을 수 없습니다. JSON 형식을 확인해 주세요."); return false; }
    double Version = 0;
    FString Units;
    if (!ReadNumber(Root, TEXT("version"), Version) || (Version != 1.0 && Version != 2.0)
        || !ReadString(Root, TEXT("units"), Units) || Units != TEXT("cm"))
    { Error = TEXT("지원하는 배치 형식은 version 1 또는 2, 단위 cm입니다."); return false; }
    const TSharedPtr<FJsonValue>* HallValue = Field(Root, TEXT("hall"), EJson::Object);
    const TSharedPtr<FJsonValue>* ItemValue = Field(Root, TEXT("items"), EJson::Array);
    const TSharedPtr<FJsonValue>* JobValue = Field(Root, TEXT("job"), EJson::Object);
    if (!HallValue || !ItemValue || !JobValue)
    { Error = TEXT("배치 파일에 공장 크기, 항목 목록 또는 작업 연결 정보가 없습니다."); return false; }
    FFactoryLayout Candidate;
    if (!ReadNumber((*HallValue)->AsObject(), TEXT("length_cm"), Candidate.LengthCm)
        || !ReadNumber((*HallValue)->AsObject(), TEXT("width_cm"), Candidate.WidthCm))
    { Error = TEXT("공장 크기는 유효한 숫자 두 개로 지정해야 합니다."); return false; }
    if (Version == 2.0)
    {
        const auto* Route=Field(Root,TEXT("process_order"),EJson::Array);
        if (!ReadVector((*HallValue)->AsObject(),TEXT("origin_cm"),Candidate.HallOrigin) || !ReadNumber(Root,TEXT("route_bend_cm"),Candidate.RouteBendCm) || !Route || (*Route)->AsArray().Num()>128)
        { Error=TEXT("v2 공장 원점·공정 흐름 정보가 올바르지 않습니다."); return false; }
        for (const auto& Value:(*Route)->AsArray())
        { if (!Value.IsValid() || Value->Type!=EJson::String) { Error=TEXT("공정 순서는 ID 문자열 목록이어야 합니다."); return false; } Candidate.ProcessOrder.Add(Value->AsString()); }
    }
    const TArray<TSharedPtr<FJsonValue>>& Array = (*ItemValue)->AsArray();
    if (Array.Num() > 128) { Error = TEXT("구역과 설비는 합계 128개까지 읽을 수 있습니다."); return false; }
    for (int32 Index = 0; Index < Array.Num(); ++Index)
    {
        if (!Array[Index].IsValid() || Array[Index]->Type != EJson::Object)
        { Error = FString::Printf(TEXT("항목 %d의 형식이 올바르지 않습니다."), Index + 1); return false; }
        const TSharedPtr<FJsonObject> Object = Array[Index]->AsObject();
        FFactoryLayoutItem Item;
        FString Kind;
        if (!ReadString(Object, TEXT("id"), Item.Id) || !ReadString(Object, TEXT("name_ko"), Item.NameKo)
            || !ReadString(Object, TEXT("name_en"), Item.NameEn) || !ReadString(Object, TEXT("kind"), Kind)
            || !ParseKind(Kind, Item.Kind) || !ReadString(Object, TEXT("role"), Item.Role)
            || !ValidRole(Item.Role) || !ReadVector(Object, TEXT("position_cm"), Item.Position)
            || !ReadVector(Object, TEXT("size_cm"), Item.Size) || !ReadNumber(Object, TEXT("yaw_deg"), Item.Yaw))
        { Error = FString::Printf(TEXT("항목 %d의 필수 값, 종류 또는 용도가 올바르지 않습니다."), Index + 1); return false; }
        if (Version==2 && !ReadString(Object,TEXT("type_id"),Item.TypeId))
        { Error=TEXT("설비 등록 ID(type_id)가 없습니다."); return false; }
        Candidate.Items.Add(MoveTemp(Item));
    }
    const TSharedPtr<FJsonObject> Job = (*JobValue)->AsObject();
    if (!ReadString(Job, TEXT("station_id"), Candidate.StationId) || !ReadString(Job, TEXT("supply_id"), Candidate.SupplyId)
        || !ReadString(Job, TEXT("handler_id"), Candidate.HandlerId))
    { Error = TEXT("작업 연결 ID는 문자열이어야 합니다. 미지정은 빈 문자열로 표시해 주세요."); return false; }
    if (Version==1)
        for (const TCHAR* Id:{TEXT("P01"),TEXT("P02"),TEXT("P03"),TEXT("P04"),TEXT("P05"),TEXT("P06")})
            if (Candidate.Find(Id)) Candidate.ProcessOrder.Add(Id);
    if (!Candidate.Validate(Error)) return false;
    Out = MoveTemp(Candidate); // A rejected file never mutates the currently edited layout.
    return true;
}

bool FFactoryLayout::Contains(const FFactoryLayoutItem& Item) const
{
    if (!GeometryValid(Item) || !FMath::IsFinite(LengthCm) || !FMath::IsFinite(WidthCm)
        || LengthCm <= 0 || WidthCm <= 0) return false;
    FVector2D X, Y;
    Axes(Item, X, Y);
    const FVector2D Half = Item.Size * 0.5;
    constexpr double Tolerance = 1.e-6;
    for (int32 SX : { -1, 1 }) for (int32 SY : { -1, 1 })
    {
        const FVector2D Corner = Item.Position + X * (Half.X * SX) + Y * (Half.Y * SY);
        if (Corner.X < HallOrigin.X-Tolerance || Corner.X > HallOrigin.X+LengthCm + Tolerance
            || Corner.Y < HallOrigin.Y-WidthCm * 0.5 - Tolerance || Corner.Y > HallOrigin.Y+WidthCm * 0.5 + Tolerance) return false;
    }
    return true;
}

bool FFactoryLayout::Overlaps(const FFactoryLayoutItem& A, const FFactoryLayoutItem& B, double MarginCm)
{
    if (!GeometryValid(A) || !GeometryValid(B) || !FMath::IsFinite(MarginCm)) return true;
    FVector2D AX, AY, BX, BY;
    Axes(A, AX, AY); Axes(B, BX, BY);
    const FVector2D Delta = B.Position - A.Position;
    const FVector2D TestAxes[] = { AX, AY, BX, BY };
    // Margin is total separation clearance, not a validated safety distance.
    const double Margin = FMath::Max(0.0, MarginCm);
    for (const FVector2D& Axis : TestAxes)
    {
        const double ExtentA = 0.5 * (A.Size.X * FMath::Abs(FVector2D::DotProduct(AX, Axis))
            + A.Size.Y * FMath::Abs(FVector2D::DotProduct(AY, Axis)));
        const double ExtentB = 0.5 * (B.Size.X * FMath::Abs(FVector2D::DotProduct(BX, Axis))
            + B.Size.Y * FMath::Abs(FVector2D::DotProduct(BY, Axis)));
        if (FMath::Abs(FVector2D::DotProduct(Delta, Axis)) >= ExtentA + ExtentB + Margin - 1.e-6) return false;
    }
    return true;
}

bool FFactoryLayout::SegmentHits(const FVector& A, const FVector& B, const FFactoryLayoutItem& Obstacle, double InflateCm)
{
    if (A.ContainsNaN() || B.ContainsNaN() || !GeometryValid(Obstacle) || !FMath::IsFinite(InflateCm)) return true;
    FVector2D X, Y;
    Axes(Obstacle, X, Y);
    const FVector2D DA(A.X - Obstacle.Position.X, A.Y - Obstacle.Position.Y);
    const FVector2D DB(B.X - Obstacle.Position.X, B.Y - Obstacle.Position.Y);
    const FVector2D Start(FVector2D::DotProduct(DA, X), FVector2D::DotProduct(DA, Y));
    const FVector2D End(FVector2D::DotProduct(DB, X), FVector2D::DotProduct(DB, Y));
    const FVector2D Delta = End - Start;
    const FVector2D Half = Obstacle.Size * 0.5 + FVector2D(FMath::Max(0.0, InflateCm));
    double Enter = 0, Leave = 1;
    // Conservative 2D broad-phase: elevation is intentionally ignored.
    for (int32 Axis = 0; Axis < 2; ++Axis)
    {
        if (FMath::Abs(Delta[Axis]) < 1.e-10)
        {
            if (Start[Axis] < -Half[Axis] || Start[Axis] > Half[Axis]) return false;
            continue;
        }
        double First = (-Half[Axis] - Start[Axis]) / Delta[Axis];
        double Last = (Half[Axis] - Start[Axis]) / Delta[Axis];
        if (First > Last) Swap(First, Last);
        Enter = FMath::Max(Enter, First); Leave = FMath::Min(Leave, Last);
        if (Enter > Leave) return false;
    }
    return true;
}

FFactoryLayout FFactoryLayout::Default()
{
    auto L=LegacyDefault();auto* H=L.Find(L.HandlerId);
    H->Kind=EFactoryItemKind::ArticulatedRobot;H->TypeId=TEXT("kuka_robot");H->NameKo=TEXT("KUKA 휠 이송 로봇");H->NameEn=TEXT("KUKA Wheel Robot");H->Position={4350,-330};
    L.Find(L.SupplyId)->Position={4575,-300};return L;
}

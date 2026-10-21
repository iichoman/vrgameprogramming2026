#include "VehicleProcessTest.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InputComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Engine/GameViewportClient.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"
#include "Styling/CoreStyle.h"
#include "Kismet/GameplayStatics.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "HighResScreenshot.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

namespace
{
bool Finite(const FVector& V)
{
    return FMath::IsFinite(V.X) && FMath::IsFinite(V.Y) && FMath::IsFinite(V.Z);
}
bool Rigid(const FTransform& T)
{
    return !T.ContainsNaN() && T.GetRotation().IsNormalized()
        && T.GetScale3D().Equals(FVector::OneVector, 1.e-6);
}
constexpr double Step = 1.0 / 120.0;
const TCHAR* WheelNames[] = { TEXT("Wheel_FL"), TEXT("Wheel_FR"), TEXT("Wheel_RL"), TEXT("Wheel_RR") };
FVector Outward(int32 Index) { return FVector(0, Index % 2 == 0 ? -1200 : 1200, 0); }
}

AVehicleProcessModule::AVehicleProcessModule()
{
    PrimaryActorTick.bCanEverTick = true;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("ModuleRoot"));
    RootComponent->SetMobility(EComponentMobility::Movable);
    SetActorEnableCollision(false);
}

void AVehicleProcessModule::RebuildVisuals()
{
    for (UStaticMeshComponent* C : Visuals) if (IsValid(C)) { RemoveInstanceComponent(C); C->DestroyComponent(); }
    Visuals.Empty();
    for (const FVehicleModuleMesh& Part : MeshParts)
    {
        if (!Part.Mesh) continue;
        UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transactional);
        AddInstanceComponent(C);
        C->SetupAttachment(RootComponent);
        C->SetMobility(EComponentMobility::Movable);
        C->SetStaticMesh(Part.Mesh);
        C->SetRelativeLocation(MeshOffsetCm);
        C->SetCollisionProfileName(TEXT("NoCollision"));
        C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        C->SetCastShadow(false);
        C->SetVisibleInRayTracing(false);
        C->SetAffectDistanceFieldLighting(false);
        C->SetVisibility(Part.bPreviewVisible);
        C->SetHiddenInGame(!Part.bPreviewVisible);
        C->RegisterComponent();
        Visuals.Add(C);
    }
    SetActorEnableCollision(false);
}

bool AVehicleProcessModule::ValidFrame() const
{
    return IsValid(VehicleFrame) && VehicleFrame != this && Rigid(VehicleFrame->GetActorTransform())
        && Rigid(GetActorTransform());
}

bool AVehicleProcessModule::CaptureReferencePose()
{
    if (!ValidFrame()) return false;
    ReferencePoseInVehicle = GetActorTransform().GetRelativeTransform(VehicleFrame->GetActorTransform());
    bHasReferencePose = true;
    TargetOffsetCm = VelocityCm = IntegralCm = FVector::ZeroVector;
    Remainder = 0;
    return true;
}

bool AVehicleProcessModule::CommandOffsetMm(FVector OffsetMm)
{
    if (!bWheelMotionEnabled || !bHasReferencePose || !ValidFrame() || !Finite(OffsetMm)
        || OffsetMm.GetAbsMax() > 3000.0) return false;
    const FVector NewTarget = OffsetMm * .1;
    if (!TargetOffsetCm.Equals(NewTarget, 1.e-9)) IntegralCm = FVector::ZeroVector;
    TargetOffsetCm = NewTarget;
    return true;
}

bool AVehicleProcessModule::RestoreReference() { return CommandOffsetMm(FVector::ZeroVector); }

bool AVehicleProcessModule::ApplyOffsetCm(const FVector& OffsetCm)
{
    if (!bHasReferencePose || !ValidFrame() || !Finite(OffsetCm)) return false;
    FTransform Pose = ReferencePoseInVehicle;
    Pose.AddToTranslation(OffsetCm);
    return SetActorTransform(Pose * VehicleFrame->GetActorTransform(), false, nullptr, ETeleportType::TeleportPhysics);
}

bool AVehicleProcessModule::ResetToReference()
{
    if (!bWheelMotionEnabled || !ApplyOffsetCm(FVector::ZeroVector)) return false;
    TargetOffsetCm = VelocityCm = IntegralCm = FVector::ZeroVector;
    Remainder = 0;
    return true;
}

FVector AVehicleProcessModule::GetCurrentOffsetMm() const
{
    if (!bHasReferencePose || !ValidFrame()) return FVector::ZeroVector;
    return (GetActorTransform().GetRelativeTransform(VehicleFrame->GetActorTransform()).GetTranslation()
        - ReferencePoseInVehicle.GetTranslation()) * 10.0;
}

bool AVehicleProcessModule::IsSettled() const
{
    return bHasReferencePose && ValidFrame() && (GetCurrentOffsetMm()*.1 - TargetOffsetCm).Size() <= .05
        && VelocityCm.Size() <= .1;
}

void AVehicleProcessModule::AdvanceMotion(double Seconds)
{
    if (!bWheelMotionEnabled || !bHasReferencePose || !ValidFrame() || !FMath::IsFinite(Seconds) || Seconds <= 0) return;
    // Bounded fixed-step acceleration plant. Ideal pose feedback; no mass, torque or contact model.
    Remainder += FMath::Min(Seconds, .25);
    while (Remainder + 1.e-12 >= Step)
    {
        Remainder = FMath::Max(0.0, Remainder - Step);
        FVector Position = GetCurrentOffsetMm() * .1;
        const FVector Error = TargetOffsetCm - Position;
        const FVector CandidateI = (IntegralCm + Error * Step).GetClampedToMaxSize(5.0);
        const FVector UnclampedA = 36.0 * Error + .1 * CandidateI - 12.0 * VelocityCm;
        // Conditional integration prevents windup while acceleration saturates.
        if (UnclampedA.Size() <= 240.0 || FVector::DotProduct(Error, IntegralCm) < 0) IntegralCm = CandidateI;
        const FVector Acceleration = (36.0 * Error + .1 * IntegralCm - 12.0 * VelocityCm).GetClampedToMaxSize(240.0);
        VelocityCm = (VelocityCm + Acceleration * Step).GetClampedToMaxSize(80.0);
        Position += VelocityCm * Step;
        if ((TargetOffsetCm-Position).Size() < .05 && VelocityCm.Size() < .1)
        {
            Position = TargetOffsetCm;
            VelocityCm = IntegralCm = FVector::ZeroVector;
        }
        ApplyOffsetCm(Position);
    }
}

void AVehicleProcessModule::BeginPlay()
{
    Super::BeginPlay();
    if (Visuals.IsEmpty()) RebuildVisuals();
    // Never recapture: a saved displaced pose must not silently become the assembly datum.
    SetActorTickEnabled(bWheelMotionEnabled && bHasReferencePose);
}
void AVehicleProcessModule::Tick(float Seconds) { Super::Tick(Seconds); AdvanceMotion(Seconds); }

AVehicleProcessTestDriver::AVehicleProcessTestDriver()
{
    PrimaryActorTick.bCanEverTick = true;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("VehicleFrame"));
    RootComponent->SetMobility(EComponentMobility::Movable);
}

void AVehicleProcessTestDriver::BeginPlay()
{
    Super::BeginPlay();
    if (FParse::Param(FCommandLine::Get(), TEXT("VehicleModulesAutoTest"))) { RunRuntimeChecks(); return; }
    APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
    if (PC)
    {
        EnableInput(PC);
        bInputConfigured = true;
        if (PC->PlayerCameraManager) PC->PlayerCameraManager->SetFOV(70.0f);
        PC->bShowMouseCursor = true;
        PC->SetIgnoreLookInput(true);
        FInputModeGameAndUI Mode;
        Mode.SetHideCursorDuringCapture(false);
        Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        PC->SetInputMode(Mode);
        InputComponent->BindKey(EKeys::RightMouseButton, IE_Pressed, this, &AVehicleProcessTestDriver::EnableLook);
        InputComponent->BindKey(EKeys::RightMouseButton, IE_Released, this, &AVehicleProcessTestDriver::DisableLook);
    }
    CreatePanel();
    bCaptureRequested = FParse::Param(FCommandLine::Get(), TEXT("VehicleModulesCapture"));
}
void AVehicleProcessTestDriver::EnableLook()
{
    if (bLooking) return;
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
    { bLooking=true; PC->SetIgnoreLookInput(false); PC->bShowMouseCursor=false; }
}
void AVehicleProcessTestDriver::DisableLook()
{
    if (!bLooking) return;
    if (APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
    { bLooking=false; PC->SetIgnoreLookInput(true); PC->bShowMouseCursor=true; }
}

void AVehicleProcessTestDriver::CreatePanel()
{
    if (!GEngine || !GEngine->GameViewport) return;
    TSharedRef<SVerticalBox> Rows = SNew(SVerticalBox);
    const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular",16);
    auto Label = [&Rows,Font](const FString& S)
    {
        Rows->AddSlot().AutoHeight().Padding(3)[SNew(STextBlock).Font(Font).Text(FText::FromString(S))];
    };
    Label(TEXT("VEHICLE MODULE TEST | 4 WHEELS"));
    Label(TEXT("OUT: 1200 mm from saved assembly pose"));
    Label(TEXT("Vehicle axes | pose PID test | no fastening"));
    for (int32 Index=0; Index<Wheels.Num(); ++Index)
    {
        TWeakObjectPtr<AVehicleProcessModule> W = Wheels[Index];
        if (!W.IsValid()) continue;
        Rows->AddSlot().AutoHeight().Padding(3)[
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1)[SNew(STextBlock).Font(Font).Text_Lambda([W]() {
                if (!W.IsValid()) return FText::FromString(TEXT("Missing wheel"));
                return FText::FromString(FString::Printf(TEXT("%s   Y %+.1f mm  %s"), *W->ModuleId.ToString(),
                    W->GetCurrentOffsetMm().Y, W->IsSettled()?TEXT("HOLD"):TEXT("MOVING")));
            })]
            + SHorizontalBox::Slot().AutoWidth().Padding(4,0)[SNew(SButton).IsFocusable(false)
                .OnClicked_Lambda([W,Index]() { if(W.IsValid()) W->CommandOffsetMm(Outward(Index)); return FReply::Handled(); })
                [SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("OUT")))]]
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).IsFocusable(false)
                .OnClicked_Lambda([W]() { if(W.IsValid()) W->RestoreReference(); return FReply::Handled(); })
                [SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("RESTORE")))]]
        ];
    }
    Rows->AddSlot().AutoHeight().Padding(3)[SNew(SButton).IsFocusable(false)
        .OnClicked_Lambda([this]() { for(auto W:Wheels) if(IsValid(W)) W->RestoreReference(); return FReply::Handled(); })
        [SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("RESTORE ALL (PID)")))]];
    Rows->AddSlot().AutoHeight().Padding(3)[SNew(SButton).IsFocusable(false)
        .OnClicked_Lambda([this]() { for(auto W:Wheels) if(IsValid(W)) W->ResetToReference(); return FReply::Handled(); })
        [SNew(STextBlock).Font(Font).Text(FText::FromString(TEXT("RESET TEST (instant)")))]];
    Label(TEXT("RMB hold: look | WASD: fly | E/Q: up/down"));
    Label(TEXT("FL/FR = front left/right; RL/RR = rear"));
    Panel = SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left).Padding(14)[
            SNew(SBox).WidthOverride(640)[SNew(SBorder).Padding(8).BorderBackgroundColor(FLinearColor(.04f,.05f,.07f,.95f))[Rows]]];
    GEngine->GameViewport->AddViewportWidgetContent(Panel.ToSharedRef(), 20);
}

void AVehicleProcessTestDriver::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (!bCaptureRequested) return;
    CaptureClock += DeltaSeconds;
    if (CaptureClock >= 5.0 && !bCaptureDeployed)
    {
        bCaptureDeployed=true;
        if (Wheels.Num()==4) for(int32 I=0;I<4;++I) if(IsValid(Wheels[I])) Wheels[I]->CommandOffsetMm(Outward(I));
    }
    if (CaptureClock >= 12.0 && !bCaptureTaken)
    {
        bCaptureTaken=true;
        FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir()/TEXT("CAR_MODULE_TEST_Runtime.png"), true, false);
    }
    if (CaptureClock >= 18.0)
        FPlatformMisc::RequestExitWithStatus(false,IFileManager::Get().FileSize(*(FPaths::ProjectSavedDir()/TEXT("CAR_MODULE_TEST_Runtime.png")))>0?0:1);
}

void AVehicleProcessTestDriver::EndPlay(const EEndPlayReason::Type Reason)
{
    if (GEngine && GEngine->GameViewport && Panel.IsValid()) GEngine->GameViewport->RemoveViewportWidgetContent(Panel.ToSharedRef());
    Panel.Reset();
    if (APlayerController* PC = bInputConfigured ? UGameplayStatics::GetPlayerController(this,0) : nullptr)
    {
        if (!bLooking) PC->SetIgnoreLookInput(false);
        PC->bShowMouseCursor=false;
        PC->SetInputMode(FInputModeGameOnly());
    }
    Super::EndPlay(Reason);
}

void AVehicleProcessTestDriver::RunRuntimeChecks()
{
    TArray<TSharedPtr<FJsonValue>> Checks;
    bool bPassed = true;
    auto Check = [&](const TCHAR* Name, bool Result) {
        auto O=MakeShared<FJsonObject>(); O->SetStringField(TEXT("check"),Name); O->SetBoolField(TEXT("passed"),Result);
        Checks.Add(MakeShared<FJsonValueObject>(O)); bPassed &= Result;
        UE_LOG(LogTemp, Display, TEXT("MODULE_TEST %s: %s"),Name,Result?TEXT("PASS"):TEXT("FAIL"));
    };
    TArray<AVehicleProcessModule*> Modules;
    TMap<AVehicleProcessModule*,FTransform> Initial;
    for(TActorIterator<AVehicleProcessModule> It(GetWorld());It;++It) {Modules.Add(*It);Initial.Add(*It,It->GetActorTransform());}
    bool bReady=Wheels.Num()==4;
    TSet<AVehicleProcessModule*> Unique;
    for(int32 I=0;I<Wheels.Num();++I)
    {
        auto W=Wheels[I];
        bReady &= IsValid(W) && I<4 && W->ModuleId==FName(WheelNames[FMath::Min(I,3)]) && W->bHasReferencePose && W->bWheelMotionEnabled;
        Unique.Add(W.Get());
    }
    bReady &= Unique.Num()==4;
    Check(TEXT("four_unique_saved_wheels"),bReady);
    Check(TEXT("42_native_modules"),Modules.Num()==42);
    bool bAttached=true;
    for(auto M:Modules)bAttached &= M->GetAttachParentActor()==this && M->VehicleFrame==this;
    Check(TEXT("all_modules_attached_to_vehicle_frame"),bAttached);
    auto Advance=[&](double Seconds,double Dt){for(int32 N=0;N<FMath::RoundToInt(Seconds/Dt);++N)for(auto W:Wheels)W->AdvanceMotion(Dt);};
    if(bReady)
    {
        bool bIndependent=true,bRestore=true,bAbsolute=true,bReference=true;
        for(int32 I=0;I<4;++I)
        {
            auto W=Wheels[I]; const FTransform Ref=W->ReferencePoseInVehicle;
            W->CommandOffsetMm(Outward(I)); Advance(8,1.0/60);
            bAbsolute &= W->GetCurrentOffsetMm().Equals(Outward(I),.01) && W->IsSettled();
            W->CommandOffsetMm(Outward(I)); Advance(3,1.0/60);
            bAbsolute &= W->GetCurrentOffsetMm().Equals(Outward(I),.01);
            for(auto M:Modules) if(M!=W) bIndependent &= M->GetActorTransform().Equals(Initial[M],.001);
            W->RestoreReference(); Advance(8,1.0/60);
            bRestore &= W->GetActorTransform().Equals(Initial[W.Get()],.001) && W->IsSettled();
            bReference &= W->ReferencePoseInVehicle.Equals(Ref,.000001);
        }
        Check(TEXT("individual_wheel_does_not_move_other_modules"),bIndependent);
        Check(TEXT("absolute_commands_are_not_cumulative"),bAbsolute);
        Check(TEXT("all_four_restore_original_poses"),bRestore);
        Check(TEXT("reference_never_recaptured"),bReference);
        for(int32 I=0;I<4;++I)Wheels[I]->CommandOffsetMm(Outward(I)*(.25*(I+1)));
        Advance(8,1.0/60);
        bool bSimultaneous=true;
        for(int32 I=0;I<4;++I){bSimultaneous &= Wheels[I]->GetCurrentOffsetMm().Equals(Outward(I)*(.25*(I+1)),.01);Wheels[I]->ResetToReference();}
        Check(TEXT("simultaneous_different_targets"),bSimultaneous);
        auto W=Wheels[0];
        W->CommandOffsetMm(Outward(0)); Advance(1.7,1.0/30); FVector At30=W->GetCurrentOffsetMm();
        W->ResetToReference(); W->CommandOffsetMm(Outward(0)); Advance(1.7,1.0/120);
        Check(TEXT("30_vs_120_fps_fixed_step_trajectory"),W->GetCurrentOffsetMm().Equals(At30,.001));
        W->ResetToReference();
        const FTransform Frame=GetActorTransform();
        SetActorTransform(FTransform(FRotator(0,37,0),Frame.GetTranslation()+FVector(123,45,67)));
        bool bFollow=true;
        for(auto M:Modules)bFollow &= M->GetActorTransform().Equals(M->ReferencePoseInVehicle*GetActorTransform(),.001);
        Check(TEXT("all_modules_follow_vehicle_frame"),bFollow);
        W->CommandOffsetMm(FVector(20,-300,40));Advance(8,1.0/60);
        const FVector Expected=GetActorTransform().TransformPosition(W->ReferencePoseInVehicle.GetTranslation()+FVector(2,-30,4));
        Check(TEXT("commands_follow_translated_rotated_vehicle_frame"),W->GetActorLocation().Equals(Expected,.001));
        W->RestoreReference();Advance(8,1.0/60);
        Check(TEXT("restore_in_moved_vehicle_frame"),W->GetCurrentOffsetMm().IsNearlyZero(.01));
        SetActorTransform(Frame);
        Check(TEXT("reject_out_of_range"),!W->CommandOffsetMm(FVector(0,3001,0)));
        bool bFixed=true;for(auto M:Modules)if(!M->bWheelMotionEnabled)bFixed &= !M->CommandOffsetMm(FVector(0,100,0));
        Check(TEXT("nonwheel_motion_rejected"),bFixed);
        for(auto Wheel:Wheels)Wheel->ResetToReference();
        bool bFinal=true;for(auto M:Modules)bFinal &= M->GetActorTransform().Equals(Initial[M],.001);
        Check(TEXT("final_state_restored"),bFinal);
    }
    auto Report=MakeShared<FJsonObject>(); Report->SetBoolField(TEXT("success"),bPassed);Report->SetArrayField(TEXT("checks"),Checks);
    Report->SetStringField(TEXT("scope"),TEXT("Native BeginPlay; ideal-feedback translation PID with bounded acceleration; not robot/contact/fastening validation."));
    FString Json;auto Writer=TJsonWriterFactory<>::Create(&Json);FJsonSerializer::Serialize(Report,Writer);
    const bool bSaved=FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("VehicleModulesRuntimeReport.json")));
    FPlatformMisc::RequestExitWithStatus(false,(bPassed && bSaved)?0:1);
}

#include "FactoryRuntime.h"
#include "FactoryViewport.h"
#include "Framework/Application/SlateApplication.h"
#include "Layout/WidgetPath.h"
#include "Widgets/SWindow.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

bool AFactoryRuntime::RunUIChecks()
{
    if(!ViewportSurface.IsValid())return false;
    auto& App=FSlateApplication::Get();const auto Window=App.FindWidgetWindow(ViewportSurface.ToSharedRef());if(!Window)return false;
    bool Pass=true;TArray<TSharedPtr<FJsonValue>> Checks;
    auto Check=[&](const TCHAR* Name,bool OK){auto C=MakeShared<FJsonObject>();C->SetStringField(TEXT("check"),Name);C->SetBoolField(TEXT("passed"),OK);Checks.Add(MakeShared<FJsonValueObject>(C));Pass&=OK;};
    const auto Geometry=ViewportSurface->GetCachedGeometry();const FVector2D Size(Geometry.GetLocalSize());
    auto Absolute=[&](FVector2D Local){return FVector2D(Geometry.LocalToAbsolute(Local));};
    auto IsSurface=[&](FVector2D Local){return App.LocateWindowUnderMouse(Absolute(Local),{Window.ToSharedRef()}).ContainsWidget(ViewportSurface.Get());};
    Check(TEXT("viewport_surface_receives_empty_canvas_hits"),IsSurface({Size.X*.65,Size.Y*.4}));
    Check(TEXT("inspector_blocks_background_hit"),!IsSurface({50,50}));
    Check(TEXT("bottom_palette_blocks_background_hit"),!IsSurface({Size.X*.6,Size.Y-40}));
    auto Mouse=[&](FVector2D From,FVector2D To,FKey Button)
    {
        const FVector2D A=Absolute(From),B=Absolute(To);const TSet<FKey> Held={Button};
        const auto PathA=App.LocateWindowUnderMouse(A,{Window.ToSharedRef()});
        const auto PathB=App.LocateWindowUnderMouse(B,{Window.ToSharedRef()});
        App.RoutePointerDownEvent(PathA,FPointerEvent(0,A,A,Held,Button,0,FModifierKeysState()));
        App.RoutePointerMoveEvent(PathB,FPointerEvent(0,B,A,Held,EKeys::Invalid,0,FModifierKeysState()),false);
        App.RoutePointerUpEvent(PathB,FPointerEvent(0,B,B,TSet<FKey>(),Button,0,FModifierKeysState()));
    };
    const auto Centre=CameraCentre;const auto OldYaw=CameraYaw;const auto OldPitch=CameraPitch;
    const FVector2D Canvas(Size.X*.65,Size.Y*.4);
    Mouse(Canvas,Canvas+FVector2D(60,35),EKeys::MiddleMouseButton);
    Check(TEXT("middle_mouse_drag_pans_camera"),!CameraCentre.Equals(Centre,.001));
    Mouse(Canvas,Canvas+FVector2D(40,20),EKeys::RightMouseButton);
    Check(TEXT("right_mouse_drag_orbits_camera"),!FMath::IsNearlyEqual(CameraYaw,OldYaw)||!FMath::IsNearlyEqual(CameraPitch,OldPitch));
    CameraCentre=Centre;CameraYaw=OldYaw;CameraPitch=OldPitch;UpdateCamera();
    const FString Initial=Layout.ToJson();SelectedId=bUseAssemblyIntegration?Layout.Items[0].Id:Layout.SupplyId;bSnap=false;
    SetSelectedPosition(Selected()->Position.X/100+.5,Selected()->Position.Y/100);
    App.SetKeyboardFocus(ViewportSurface.ToSharedRef(),EFocusCause::SetDirectly);
    const FModifierKeysState Ctrl(false,false,true,false,false,false,false,false,false);
    App.ProcessKeyDownEvent(FKeyEvent(EKeys::Z,Ctrl,0,false,0,0));
    Check(TEXT("ctrl_z_routes_to_canvas_history"),Layout.ToJson()==Initial);
    App.ProcessKeyDownEvent(FKeyEvent(EKeys::Y,Ctrl,0,false,0,0));
    Check(TEXT("ctrl_y_routes_to_canvas_history"),Layout.ToJson()!=Initial);
    // A text input owns Ctrl+Z while focused; it must not bubble into layout history.
    TSharedPtr<SWidget> TextInput;TFunction<void(TSharedRef<SWidget>)> FindInput;
    FindInput=[&](TSharedRef<SWidget> W){if(TextInput)return;if(W->GetTypeAsString()==TEXT("SEditableText")){TextInput=W;return;}
        if(auto* Children=W->GetChildren())for(int32 I=0;I<Children->Num();++I)FindInput(Children->GetChildAt(I));};
    FindInput(Panel.ToSharedRef());const FString BeforeText=Layout.ToJson();
    if(TextInput){App.SetKeyboardFocus(TextInput.ToSharedRef(),EFocusCause::SetDirectly);App.ProcessKeyDownEvent(FKeyEvent(EKeys::Z,Ctrl,0,false,0,0));}
    Check(TEXT("text_input_ctrl_z_does_not_undo_layout"),TextInput.IsValid()&&Layout.ToJson()==BeforeText);
    UndoEdit();bSnap=true;SelectedId.Empty();History.Clear();SavedSnapshot=Layout.ToJson();RefreshDirty();
    auto Report=MakeShared<FJsonObject>();Report->SetBoolField(TEXT("success"),Pass);Report->SetArrayField(TEXT("checks"),Checks);
    Report->SetStringField(TEXT("scope"),TEXT("Synthetic Slate events and real rendered widget hit paths. No external mouse/OS input injection."));
    FString Json;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Json));
    FFileHelper::SaveStringToFile(Json,*(FPaths::ProjectSavedDir()/TEXT("FactoryEditorUIReport.json")));return Pass;
}

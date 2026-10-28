#include "FactoryViewport.h"
#include "FactoryRuntime.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"

FVector2D SFactoryViewport::Pixel(const FGeometry& G,const FPointerEvent& E) const
{
    int32 W=1,H=1;if(Runtime.IsValid())if(auto* PC=UGameplayStatics::GetPlayerController(Runtime.Get(),0))PC->GetViewportSize(W,H);
    return FVector2D(G.AbsoluteToLocal(E.GetScreenSpacePosition()))*FVector2D(W,H)/FVector2D(G.GetLocalSize()).ComponentMax(FVector2D(1,1));
}
FReply SFactoryViewport::OnMouseButtonDown(const FGeometry& G,const FPointerEvent& E)
{if(!Runtime.IsValid())return FReply::Unhandled();Runtime->ViewportPointer(Pixel(G,E),E.GetEffectingButton(),true);return FReply::Handled().SetUserFocus(SharedThis(this)).CaptureMouse(SharedThis(this));}
FReply SFactoryViewport::OnMouseButtonUp(const FGeometry& G,const FPointerEvent& E)
{if(Runtime.IsValid())Runtime->ViewportPointer(Pixel(G,E),E.GetEffectingButton(),false);return FReply::Handled().ReleaseMouseCapture();}
FReply SFactoryViewport::OnMouseMove(const FGeometry& G,const FPointerEvent& E)
{if(Runtime.IsValid())Runtime->ViewportMove(Pixel(G,E));return FReply::Handled();}
FReply SFactoryViewport::OnMouseWheel(const FGeometry&,const FPointerEvent& E)
{if(Runtime.IsValid()){if(E.GetWheelDelta()>0)Runtime->ZoomIn();else Runtime->ZoomOut();}return FReply::Handled();}
FReply SFactoryViewport::OnKeyDown(const FGeometry&,const FKeyEvent& E)
{
    if(!Runtime.IsValid())return FReply::Unhandled();
    const FKey K=E.GetKey();
    if(K==EKeys::W||K==EKeys::A||K==EKeys::S||K==EKeys::D||K==EKeys::Up||K==EKeys::Down||K==EKeys::Left||K==EKeys::Right)
    {if(!E.IsControlDown())Held.Add(K);return FReply::Handled();}
    if(K==EKeys::Escape||K==EKeys::F||K==EKeys::Delete||(E.IsControlDown()&&(K==EKeys::Z||K==EKeys::Y)))
    {if(!E.IsRepeat())Runtime->ViewportKey(K,E.IsControlDown(),E.IsShiftDown());return FReply::Handled();}
    return FReply::Unhandled();
}
FReply SFactoryViewport::OnKeyUp(const FGeometry&,const FKeyEvent& E){Held.Remove(E.GetKey());return FReply::Handled();}
void SFactoryViewport::OnFocusLost(const FFocusEvent&){Held.Reset();}
void SFactoryViewport::OnMouseCaptureLost(const FCaptureLostEvent&)
{if(Runtime.IsValid())Runtime->ViewportCaptureLost();}
void SFactoryViewport::Tick(const FGeometry&,double,float Dt)
{
    if(!Runtime.IsValid()||!HasKeyboardFocus())return;FVector2D D=FVector2D::ZeroVector;
    if(Held.Contains(EKeys::W)||Held.Contains(EKeys::Up))D.Y-=1;if(Held.Contains(EKeys::S)||Held.Contains(EKeys::Down))D.Y+=1;
    if(Held.Contains(EKeys::A)||Held.Contains(EKeys::Left))D.X+=1;if(Held.Contains(EKeys::D)||Held.Contains(EKeys::Right))D.X-=1;
    if(!D.IsNearlyZero())Runtime->PanCamera(D.GetSafeNormal()*700*FMath::Min(Dt,.05f));
}

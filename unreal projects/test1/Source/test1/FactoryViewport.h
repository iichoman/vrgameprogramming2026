#pragma once
#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"
class AFactoryRuntime;
/** A focusable viewport surface. Slate consumes UI hits before this surface sees them. */
class SFactoryViewport : public SLeafWidget
{
public:
    SLATE_BEGIN_ARGS(SFactoryViewport) {} SLATE_ARGUMENT(AFactoryRuntime*, Runtime) SLATE_END_ARGS()
    void Construct(const FArguments& Args){Runtime=Args._Runtime;}
    virtual bool SupportsKeyboardFocus() const override{return true;}
    virtual FVector2D ComputeDesiredSize(float) const override{return FVector2D(1,1);}
    virtual int32 OnPaint(const FPaintArgs&,const FGeometry&,const FSlateRect&,FSlateWindowElementList&,int32 Layer,const FWidgetStyle&,bool) const override{return Layer;}
    virtual FReply OnMouseButtonDown(const FGeometry&,const FPointerEvent&) override;
    virtual FReply OnMouseButtonUp(const FGeometry&,const FPointerEvent&) override;
    virtual FReply OnMouseMove(const FGeometry&,const FPointerEvent&) override;
    virtual FReply OnMouseWheel(const FGeometry&,const FPointerEvent&) override;
    virtual FReply OnKeyDown(const FGeometry&,const FKeyEvent&) override;
    virtual FReply OnKeyUp(const FGeometry&,const FKeyEvent&) override;
    virtual void OnFocusLost(const FFocusEvent&) override;
    virtual void OnMouseCaptureLost(const FCaptureLostEvent&) override;
    virtual void Tick(const FGeometry&,double,float) override;
private:
    FVector2D Pixel(const FGeometry&,const FPointerEvent&) const;
    TWeakObjectPtr<AFactoryRuntime> Runtime;
    TSet<FKey> Held;
};

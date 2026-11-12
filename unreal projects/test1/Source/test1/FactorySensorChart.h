#pragma once
#include "Widgets/SLeafWidget.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"
#include "FactoryInspection.h"

class SFactorySensorChart : public SLeafWidget
{
public:
    SLATE_BEGIN_ARGS(SFactorySensorChart) {} SLATE_ARGUMENT(TWeakPtr<FFactoryInspectionModel>, Model) SLATE_END_ARGS()
    void Construct(const FArguments& A){Model=A._Model;}
    virtual FVector2D ComputeDesiredSize(float) const override{return FVector2D(300,90);}
    virtual int32 OnPaint(const FPaintArgs&,const FGeometry& G,const FSlateRect&,FSlateWindowElementList& Out,int32 Layer,const FWidgetStyle&,bool) const override
    {
        FSlateDrawElement::MakeBox(Out,Layer,G.ToPaintGeometry(),FCoreStyle::Get().GetBrush("WhiteBrush"),ESlateDrawEffect::None,FLinearColor(.015f,.024f,.04f));
        const auto M=Model.Pin();const auto* C=M?M->Sensors.Find(M->SelectedChannel):nullptr;if(!C||C->History.Num()<2)return Layer;
        double Min=DBL_MAX,Max=-DBL_MAX;for(const auto& P:C->History)if(P.Valid&&FMath::IsFinite(P.Value)){Min=FMath::Min(Min,P.Value);Max=FMath::Max(Max,P.Value);}
        if(Min>Max)return Layer;if(Max-Min<.001){Min-=.5;Max+=.5;}
        const FVector2D Size(G.GetLocalSize());const double Start=C->History[0].Time,Span=FMath::Max(.001,C->History.Last().Time-Start);
        TArray<FVector2D> Segment;
        auto Flush=[&](){if(Segment.Num()>1)FSlateDrawElement::MakeLines(Out,Layer+1,G.ToPaintGeometry(),Segment,ESlateDrawEffect::None,FLinearColor(.2f,.8f,1.f),true,2.f);Segment.Reset();};
        for(const auto& P:C->History){if(!P.Valid||!FMath::IsFinite(P.Value)){Flush();continue;}Segment.Add(FVector2D(7+(P.Time-Start)/Span*(Size.X-14),Size.Y-7-(P.Value-Min)/(Max-Min)*(Size.Y-14)));}Flush();return Layer+1;
    }
private:TWeakPtr<FFactoryInspectionModel> Model;
};

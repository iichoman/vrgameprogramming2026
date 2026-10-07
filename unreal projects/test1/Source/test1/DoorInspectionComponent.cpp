#include "DoorInspectionComponent.h"
#include "DoorPoseComponent.h"
#include "Components/TextRenderComponent.h"
#include "PrimitiveSceneProxy.h"
#include "SceneManagement.h"

class FDoorInspectionProxy final : public FPrimitiveSceneProxy
{
	TArray<FDoorInspectionResult> Results;
public:
	explicit FDoorInspectionProxy(const UDoorInspectionComponent* Component)
		: FPrimitiveSceneProxy(Component), Results(Component->Measurements) {}
	virtual SIZE_T GetTypeHash() const override { static size_t Unique; return reinterpret_cast<SIZE_T>(&Unique); }
	virtual uint32 GetMemoryFootprint() const override { return sizeof(*this)+GetAllocatedSize()+Results.GetAllocatedSize(); }
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
	{
		FPrimitiveViewRelevance Relevance;
		Relevance.bDrawRelevance = IsShown(View);
		Relevance.bDynamicRelevance = true;
		return Relevance;
	}
	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily,
		uint32 VisibilityMap, FMeshElementCollector& Collector) const override
	{
		for (int32 ViewIndex=0; ViewIndex<Views.Num(); ++ViewIndex)
		{
			if (!(VisibilityMap & (1u<<ViewIndex))) continue;
			FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);
			for (const auto& R : Results)
			{
				PDI->DrawPoint(R.BodyWorld, FLinearColor::Green, 8, SDPG_Foreground);
				PDI->DrawPoint(R.DoorWorld, FLinearColor::Yellow, 6, SDPG_Foreground);
				PDI->DrawLine(R.BodyWorld, R.DoorWorld, FLinearColor::White, SDPG_Foreground);
				PDI->DrawLine(R.BodyWorld, R.BodyWorld+R.GapWorld*4, FLinearColor::Yellow, SDPG_Foreground);
				PDI->DrawLine(R.BodyWorld, R.BodyWorld+R.FlushWorld*4, FLinearColor(0,1,1), SDPG_Foreground);
			}
		}
	}
};

UDoorInspectionComponent::UDoorInspectionComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	bTickInEditor = true;
	SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SetGenerateOverlapEvents(false);
	SetMobility(EComponentMobility::Movable);
	CastShadow = false;
}

void UDoorInspectionComponent::Configure(UDoorPoseComponent* InPose, USceneComponent* InBody,
	const TArray<UTextRenderComponent*>& InLabels)
{
	Pose = InPose;
	BodyFrame = InBody;
	Labels.Reset();
	for (auto* Label : InLabels) Labels.Add(Label);
}

void UDoorInspectionComponent::OnRegister()
{
	Super::OnRegister();
	if (Pose) Pose->OnDoorPoseChanged.AddUObject(this, &UDoorInspectionComponent::HandlePoseChanged);
	RefreshMeasurements();
}

void UDoorInspectionComponent::OnUnregister()
{
	if (Pose) Pose->OnDoorPoseChanged.RemoveAll(this);
	Super::OnUnregister();
}

void UDoorInspectionComponent::HandlePoseChanged(UDoorPoseComponent*) { RefreshMeasurements(); }
void UDoorInspectionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	RefreshMeasurements(); // Also follows transforms applied directly by carriers/editor manipulation.
}

bool UDoorInspectionComponent::RefreshMeasurements()
{
	TArray<FDoorInspectionResult> Next;
	bool Valid = Pose && BodyFrame && Pose->GetControlledDoor() && Points.Num()>0;
	FTransform BodyWorld, DoorWorld;
	if (Valid)
	{
		BodyWorld = BodyFrame->GetComponentTransform();
		DoorWorld = Pose->GetControlledDoor()->GetComponentTransform();
		Valid = BodyWorld.IsValid() && DoorWorld.IsValid()
			&& BodyWorld.GetScale3D().Equals(FVector::OneVector,1.e-6)
			&& DoorWorld.GetScale3D().Equals(FVector::OneVector,1.e-6);
	}
	if (Valid) for (const auto& P : Points)
	{
		if (P.DoorPointLocalCm.ContainsNaN() || P.BodyPointLocalCm.ContainsNaN() || P.ReferenceDoorPointBodyCm.ContainsNaN()
			|| P.GapDirectionBody.ContainsNaN() || P.FlushDirectionBody.ContainsNaN()
			|| P.GapDirectionBody.IsNearlyZero() || P.FlushDirectionBody.IsNearlyZero()
			|| FMath::Abs(FVector::DotProduct(P.GapDirectionBody.GetSafeNormal(),P.FlushDirectionBody.GetSafeNormal()))>1.e-4)
		{ Valid=false; break; }
		FDoorInspectionResult R;
		R.Name=P.Name;
		R.DoorWorld=DoorWorld.TransformPosition(P.DoorPointLocalCm);
		R.BodyWorld=BodyWorld.TransformPosition(P.BodyPointLocalCm);
		const FVector CurrentDoorBody=BodyWorld.InverseTransformPosition(R.DoorWorld);
		const FVector Gap=P.GapDirectionBody.GetSafeNormal(), Flush=P.FlushDirectionBody.GetSafeNormal();
		const double CurrentGap=FVector::DotProduct(P.BodyPointLocalCm-CurrentDoorBody,Gap);
		const double BaselineGap=FVector::DotProduct(P.BodyPointLocalCm-P.ReferenceDoorPointBodyCm,Gap);
		const double CurrentFlush=FVector::DotProduct(CurrentDoorBody-P.BodyPointLocalCm,Flush);
		const double BaselineFlush=FVector::DotProduct(P.ReferenceDoorPointBodyCm-P.BodyPointLocalCm,Flush);
		R.GapChangeMm=(CurrentGap-BaselineGap)*10;
		R.FlushChangeMm=(CurrentFlush-BaselineFlush)*10;
		R.GapWorld=BodyWorld.TransformVectorNoScale(Gap);
		R.FlushWorld=BodyWorld.TransformVectorNoScale(Flush);
		Next.Add(R);
	}
	if (!Valid) Next.Reset();
	bool Changed = bMeasurementsValid!=Valid || Measurements.Num()!=Next.Num();
	if (!Changed) for (int32 i=0;i<Next.Num();++i)
	{
		const auto& A=Measurements[i]; const auto& B=Next[i];
		if (A.Name!=B.Name || !A.DoorWorld.Equals(B.DoorWorld,1.e-7) || !A.BodyWorld.Equals(B.BodyWorld,1.e-7)
			|| !A.GapWorld.Equals(B.GapWorld,1.e-7) || !A.FlushWorld.Equals(B.FlushWorld,1.e-7)
			|| !FMath::IsNearlyEqual(A.GapChangeMm,B.GapChangeMm,1.e-7)
			|| !FMath::IsNearlyEqual(A.FlushChangeMm,B.FlushChangeMm,1.e-7)) { Changed=true; break; }
	}
	Measurements=MoveTemp(Next); bMeasurementsValid=Valid;
	for (int32 i=0;i<Labels.Num();++i) if (Labels[i])
	{
		Labels[i]->SetVisibility(bShowInspection && Valid && Measurements.IsValidIndex(i));
		if (!Valid || !Measurements.IsValidIndex(i)) continue;
		const auto& R=Measurements[i];
		const FString Text=FString::Printf(TEXT("P%d dG:%+.2f dF:%+.2f mm"),i+1,R.GapChangeMm,R.FlushChangeMm);
		if (Labels[i]->Text.ToString()!=Text) Labels[i]->SetText(FText::FromString(Text));
		const FVector Location=R.BodyWorld+R.FlushWorld*8+BodyWorld.TransformVectorNoScale(FVector(0,0,2));
		if (!Labels[i]->GetComponentLocation().Equals(Location,1.e-7)) Labels[i]->SetWorldLocation(Location);
	}
	// Visibility is handled directly; no persistent debug primitives accumulate.
	SetVisibility(bShowInspection);
	if (Changed && IsRegistered()) { UpdateBounds(); MarkRenderStateDirty(); }
	return Valid;
}

FPrimitiveSceneProxy* UDoorInspectionComponent::CreateSceneProxy()
{
	return bMeasurementsValid ? new FDoorInspectionProxy(this) : nullptr;
}

FBoxSphereBounds UDoorInspectionComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	FBox StationBounds(ForceInit);
	for (const auto& R : Measurements) { StationBounds+=R.DoorWorld; StationBounds+=R.BodyWorld; }
	if (!StationBounds.IsValid) StationBounds=FBox(LocalToWorld.GetLocation()-FVector(10),LocalToWorld.GetLocation()+FVector(10));
	return FBoxSphereBounds(StationBounds.ExpandBy(15));
}

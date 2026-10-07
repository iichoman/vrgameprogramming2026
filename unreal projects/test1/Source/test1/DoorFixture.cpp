#include "DoorFixture.h"
#include "CamryAssembly.h"
#include "DoorPoseComponent.h"
#include "Components/ArrowComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"
#if WITH_EDITOR
#include "ScopedTransaction.h"
#endif

namespace
{
	bool Rigid(const FTransform& T)
	{
		return T.IsValid() && T.GetScale3D().Equals(FVector::OneVector, 1.e-6);
	}
	FColor RoleColor(EDoorFixtureRole Role, bool Region)
	{
		if (Region) return FColor(255,140,50);
		switch (Role)
		{
		case EDoorFixtureRole::Support: return FColor::Cyan;
		case EDoorFixtureRole::Stopper: return FColor::Green;
		case EDoorFixtureRole::Clamp: return FColor(255,100,200);
		default: return FColor::Yellow;
		}
	}
}

ADoorFixture::ADoorFixture()
{
	PrimaryActorTick.bCanEverTick = true;
	FixtureRoot = CreateDefaultSubobject<USceneComponent>(TEXT("FixtureRoot"));
	SetRootComponent(FixtureRoot);
	FixtureRoot->SetMobility(EComponentMobility::Movable);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	CylinderMesh = Cylinder.Object;
	CubeMesh = Cube.Object;
	for (int32 i=0; i<MaxContacts; ++i)
	{
		auto* Pad = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("FixturePad%02d"), i+1)));
		auto* Stem = CreateDefaultSubobject<UStaticMeshComponent>(FName(*FString::Printf(TEXT("FixtureStem%02d"), i+1)));
		for (auto* Mesh : {Pad, Stem})
		{
			Mesh->SetupAttachment(FixtureRoot);
			Mesh->SetMobility(EComponentMobility::Movable);
			Mesh->SetStaticMesh(CylinderMesh);
			Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Mesh->SetGenerateOverlapEvents(false);
			Mesh->SetCastShadow(false);
			Mesh->SetVisibility(false);
		}
		auto* Arrow = CreateDefaultSubobject<UArrowComponent>(FName(*FString::Printf(TEXT("FixtureDirection%02d"), i+1)));
		Arrow->SetupAttachment(FixtureRoot);
		Arrow->SetMobility(EComponentMobility::Movable);
		Arrow->SetHiddenInGame(false);
		Arrow->SetArrowSize(0.35f);
		Arrow->SetArrowLength(6.0f);
		Arrow->SetVisibility(false);
		auto* Label = CreateDefaultSubobject<UTextRenderComponent>(FName(*FString::Printf(TEXT("FixtureLabel%02d"), i+1)));
		Label->SetupAttachment(FixtureRoot);
		Label->SetMobility(EComponentMobility::Movable);
		Label->SetRelativeRotation(FRotator(0,90,0)); // Read from the cabin side of the left door.
		Label->SetWorldSize(1.6f);
		Label->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Label->SetVisibility(false);
		Pads.Add(Pad); Stems.Add(Stem); Directions.Add(Arrow); Labels.Add(Label);
	}
}

void ADoorFixture::BindPose()
{
	UDoorPoseComponent* Next = IsValid(Assembly) ? Assembly->GetDoorPoseController(ECamryDoorSide::Left) : nullptr;
	if (BoundPose.Get() == Next) return;
	if (BoundPose.IsValid()) BoundPose->OnDoorPoseChanged.RemoveAll(this);
	BoundPose = Next;
	if (Next) Next->OnDoorPoseChanged.AddUObject(this, &ADoorFixture::PoseChanged);
}

void ADoorFixture::PoseChanged(UDoorPoseComponent*) { RefreshFixture(); }
void ADoorFixture::OnConstruction(const FTransform& Transform) { Super::OnConstruction(Transform); RefreshFixture(); }
void ADoorFixture::PostRegisterAllComponents() { Super::PostRegisterAllComponents(); RefreshFixture(); }
void ADoorFixture::Tick(float DeltaSeconds) { Super::Tick(DeltaSeconds); RefreshFixture(); }
void ADoorFixture::EndPlay(const EEndPlayReason::Type Reason)
{
	if (BoundPose.IsValid()) BoundPose->OnDoorPoseChanged.RemoveAll(this);
	BoundPose.Reset();
	Super::EndPlay(Reason);
}

void ADoorFixture::HidePreview()
{
	for (int32 i=0; i<Pads.Num(); ++i)
	{
		Pads[i]->SetVisibility(false); Stems[i]->SetVisibility(false);
		Directions[i]->SetVisibility(false); Labels[i]->SetVisibility(false);
	}
}

bool ADoorFixture::Fail(const FString& Reason)
{
	bLayoutValid = false; Results.Reset(); LastStatus = Reason; HidePreview(); return false;
}

bool ADoorFixture::AlignToDoorReference()
{
	BindPose();
	if (!IsValid(Assembly) || !BoundPose.IsValid() || !BoundPose->HasReferencePose())
		return Fail(TEXT("Assign CamryAssembly with a saved left-door reference first."));
	auto* Body = Assembly->GetBodyFrame();
	const FTransform Reference = BoundPose->GetReferencePoseInBody();
	if (!Body || !Rigid(Body->GetComponentTransform()) || !Rigid(Reference))
		return Fail(TEXT("Alignment requires valid unscaled body and reference transforms."));
	Modify(); FixtureRoot->Modify();
	if (!AttachToComponent(Body, FAttachmentTransformRules::KeepWorldTransform))
		return Fail(TEXT("Could not attach the fixture to the body frame."));
	FixtureRoot->SetRelativeTransform(Reference);
	return RefreshFixture();
}

bool ADoorFixture::RefreshFixture()
{
	BindPose();
	if (!IsValid(Assembly) || !BoundPose.IsValid() || !BoundPose->HasReferencePose()
		|| !BoundPose->GetControlledDoor()) return Fail(TEXT("Assign an assembly with a saved left-door reference."));
	if (Contacts.IsEmpty() || Contacts.Num()>MaxContacts)
		return Fail(TEXT("Configure 1 to 12 contact candidates."));
	const FTransform FixtureWorld = GetActorTransform();
	const FTransform DoorWorld = BoundPose->GetControlledDoor()->GetComponentTransform();
	if (!Rigid(FixtureWorld) || !Rigid(DoorWorld) || !FMath::IsFinite(PreviewRetractionMm) || PreviewRetractionMm<0)
		return Fail(TEXT("Invalid transform, scale, or preview retraction. Use unit scale and nonnegative retraction."));
	TSet<FName> Names;
	for (const auto& C : Contacts)
	{
		if (C.Name.IsNone() || Names.Contains(C.Name) || C.DoorPointLocalCm.ContainsNaN()
			|| C.OffsetMm.ContainsNaN() || C.OutwardNormalLocal.ContainsNaN() || C.OutwardNormalLocal.IsNearlyZero()
			|| C.PreviewSizeMm.ContainsNaN() || C.PreviewSizeMm.GetMin()<=0
			|| !FMath::IsFinite(C.PreviewStemLengthMm) || C.PreviewStemLengthMm<0)
			return Fail(TEXT("Contacts require unique names, finite positions, nonzero normals, and positive preview sizes."));
		Names.Add(C.Name);
	}
	Results.Reset(Contacts.Num());
	for (int32 i=0; i<Pads.Num(); ++i)
	{
		const bool Active = Contacts.IsValidIndex(i);
		Pads[i]->SetVisibility(bShowFixture && Active);
		Stems[i]->SetVisibility(bShowFixture && Active && Contacts[i].PreviewStemLengthMm>0);
		Directions[i]->SetVisibility(bShowFixture && Active);
		Labels[i]->SetVisibility(bShowFixture && bShowLabels && Active);
		if (!Active) continue;
		const auto& C = Contacts[i];
		const FVector Normal = C.OutwardNormalLocal.GetSafeNormal();
		const FVector Point = C.DoorPointLocalCm + C.OffsetMm/10.0 + Normal*(PreviewRetractionMm/10.0);
		const FVector NormalWorld = FixtureWorld.TransformVectorNoScale(Normal);
		FDoorFixtureContactResult R;
		R.Name = C.Name;
		R.FixtureWorldCm = FixtureWorld.TransformPosition(Point);
		R.DoorWorldCm = DoorWorld.TransformPosition(C.DoorPointLocalCm);
		const FVector Delta = R.DoorWorldCm - R.FixtureWorldCm;
		const double Projection = FVector::DotProduct(Delta, NormalWorld);
		R.NormalDisplacementMm = Projection*10.0;
		R.TangentialDisplacementMm = (Delta - NormalWorld*Projection).Size()*10.0;
		Results.Add(R);
		// Engine basic shapes span 100 cm. The near face touches the source midsurface marker.
		const FQuat PadRotation = FRotationMatrix::MakeFromZ(Normal).ToQuat();
		auto* DesiredMesh = (C.Role==EDoorFixtureRole::Support || C.Role==EDoorFixtureRole::Stopper || C.bRegionOnly)
			? CubeMesh.Get() : CylinderMesh.Get();
		if (Pads[i]->GetStaticMesh()!=DesiredMesh) Pads[i]->SetStaticMesh(DesiredMesh);
		Pads[i]->SetRelativeTransform(FTransform(PadRotation, Point+Normal*(C.PreviewSizeMm.Z/20.0), C.PreviewSizeMm/1000.0));
		Stems[i]->SetRelativeTransform(FTransform(PadRotation,
			Point+Normal*(C.PreviewSizeMm.Z/10.0+C.PreviewStemLengthMm/20.0),
			FVector(0.008,0.008,FMath::Max(C.PreviewStemLengthMm,0.01)/1000.0)));
		Directions[i]->SetRelativeLocation(Point);
		Directions[i]->SetRelativeRotation(Normal.Rotation());
		const FColor Color = RoleColor(C.Role, C.bRegionOnly);
		if (Directions[i]->ArrowColor!=Color) Directions[i]->SetArrowFColor(Color);
		if (Labels[i]->TextRenderColor!=Color) Labels[i]->SetTextRenderColor(Color);
		Labels[i]->SetRelativeLocation(Point+Normal*12+FVector(0,0,2));
		const FString Text = FString::Printf(TEXT("%s %s\nN:%+.2f T:%.2f mm"), *C.Name.ToString(),
			C.bRegionOnly ? TEXT("REGION") : TEXT("CANDIDATE"), R.NormalDisplacementMm, R.TangentialDisplacementMm);
		if (Labels[i]->Text.ToString()!=Text) Labels[i]->SetText(FText::FromString(Text));
	}
	bLayoutValid = true;
	LastStatus = TEXT("Geometry preview only. N/T are marker displacements, not contact forces or verified clearances. Dimensions are illustrative.");
	return true;
}

FVector ADoorFixture::GetContactWorldPosition(FName ContactName) const
{
	for (const auto& R : Results) if (R.Name==ContactName) return R.FixtureWorldCm;
	return FVector::ZeroVector;
}

void ADoorFixture::AlignFixtureButton()
{
#if WITH_EDITOR
	const FScopedTransaction Transaction(NSLOCTEXT("DoorFixture","Align","Align fixture to saved door reference"));
#endif
	AlignToDoorReference();
}
void ADoorFixture::RefreshFixtureButton() { RefreshFixture(); }

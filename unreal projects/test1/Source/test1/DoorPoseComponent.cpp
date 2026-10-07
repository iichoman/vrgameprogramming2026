#include "DoorPoseComponent.h"

#include "Components/SceneComponent.h"
#include "Engine/World.h"

DEFINE_LOG_CATEGORY_STATIC(LogDoorPose, Log, All);

namespace
{
	bool IsRigidPose(const FTransform& Pose)
	{
		return Pose.IsValid() && Pose.GetScale3D().Equals(FVector::OneVector, 1.e-6);
	}
}

UDoorPoseComponent::UDoorPoseComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UDoorPoseComponent::Configure(USceneComponent* InTarget, USceneComponent* InReferenceFrame)
{
	// Changing the controlled door/frame invalidates a reference captured in the old frame.
	if (Target != InTarget || ReferenceFrame != InReferenceFrame)
	{
		bHasReferencePose = false;
		ReferencePoseInBody = FTransform::Identity;
	}
	Target = InTarget;
	ReferenceFrame = InReferenceFrame;
}

bool UDoorPoseComponent::ValidateFrames() const
{
	if (!IsValid(Target) || !IsValid(ReferenceFrame) || Target == ReferenceFrame)
	{
		UE_LOG(LogDoorPose, Warning, TEXT("%s: missing or invalid door/body frame."), *GetName());
		return false;
	}
	if (!IsRigidPose(Target->GetComponentTransform()) || !IsRigidPose(ReferenceFrame->GetComponentTransform())
		|| Target->Mobility != EComponentMobility::Movable)
	{
		UE_LOG(LogDoorPose, Warning, TEXT("%s: movable door and finite, unit-scale rigid frames required."), *GetName());
		return false;
	}
	if (const USceneComponent* Parent = Target->GetAttachParent())
	{
		if (!IsRigidPose(Parent->GetComponentTransform()))
		{
			UE_LOG(LogDoorPose, Warning, TEXT("%s: door parent must have unit scale."), *GetName());
			return false;
		}
	}
	return true;
}

bool UDoorPoseComponent::CaptureReferencePose()
{
	if (!ValidateFrames())
	{
		return false;
	}
#if WITH_EDITOR
	if (GetWorld() && !GetWorld()->IsGameWorld())
	{
		Modify();
	}
#endif
	ReferencePoseInBody = Target->GetComponentTransform().GetRelativeTransform(ReferenceFrame->GetComponentTransform());
	bHasReferencePose = true;
	return true;
}

bool UDoorPoseComponent::ApplyReferenceOffsetMm(FVector OffsetMm)
{
	if (!bHasReferencePose || !ValidateFrames() || OffsetMm.ContainsNaN())
	{
		UE_LOG(LogDoorPose, Warning, TEXT("%s: valid saved reference and finite offset required."), *GetName());
		return false;
	}
	FTransform DesiredPose = ReferencePoseInBody;
	DesiredPose.AddToTranslation(OffsetMm * 0.1); // mm -> Unreal cm, once at this boundary.
	return ApplyWorldPose(DesiredPose * ReferenceFrame->GetComponentTransform());
}

bool UDoorPoseComponent::RestoreReferencePose()
{
	return ApplyReferenceOffsetMm(FVector::ZeroVector);
}

bool UDoorPoseComponent::SetDoorWorldPose(const FTransform& WorldPose)
{
	return ValidateFrames() && ApplyWorldPose(WorldPose);
}

bool UDoorPoseComponent::ApplyWorldPose(const FTransform& WorldPose)
{
	if (!IsRigidPose(WorldPose))
	{
		UE_LOG(LogDoorPose, Warning, TEXT("%s: target pose must be finite, normalized and unit scale."), *GetName());
		return false;
	}
#if WITH_EDITOR
	if (GetWorld() && !GetWorld()->IsGameWorld())
	{
		Target->Modify();
	}
#endif
	Target->SetWorldTransform(WorldPose, false, nullptr, ETeleportType::TeleportPhysics);
	OnDoorPoseChanged.Broadcast(this);
	return true;
}

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DoorPoseComponent.generated.h"

class USceneComponent;
class UDoorPoseComponent;
DECLARE_MULTICAST_DELEGATE_OneParam(FDoorPoseChanged, UDoorPoseComponent*);

/** Immediate rigid-pose control shared by editor tools and future robot/fixture controllers.
 * Reference poses are serialized in the body frame. No automatic capture or animation.
 */
UCLASS(NotBlueprintable)
class TEST1_API UDoorPoseComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDoorPoseComponent();
	void Configure(USceneComponent* InTarget, USceneComponent* InReferenceFrame);

	// Reflection is used for editor automation; these are not Blueprint functions.
	UFUNCTION()
	bool CaptureReferencePose();

	/** Absolute offset from the saved pose along BODY axes, in millimeters. Not cumulative. */
	UFUNCTION()
	bool ApplyReferenceOffsetMm(FVector OffsetMm);

	UFUNCTION()
	bool RestoreReferencePose();

	/** Integration entry point: a robot/fixture may supply an absolute world pose (cm).
	 * Does not overwrite the reference pose, change attachment, or simulate constraints.
	 */
	UFUNCTION()
	bool SetDoorWorldPose(const FTransform& WorldPose);

	UFUNCTION()
	bool HasReferencePose() const { return bHasReferencePose; }

	const FTransform& GetReferencePoseInBody() const { return ReferencePoseInBody; }
	USceneComponent* GetControlledDoor() const { return Target.Get(); }
	FDoorPoseChanged OnDoorPoseChanged;

private:
	bool ValidateFrames() const;
	bool ApplyWorldPose(const FTransform& WorldPose);

	UPROPERTY()
	TObjectPtr<USceneComponent> Target;

	UPROPERTY()
	TObjectPtr<USceneComponent> ReferenceFrame;

	UPROPERTY(VisibleInstanceOnly, Category = "Door Pose")
	bool bHasReferencePose = false;

	UPROPERTY(VisibleInstanceOnly, Category = "Door Pose")
	FTransform ReferencePoseInBody = FTransform::Identity;
};

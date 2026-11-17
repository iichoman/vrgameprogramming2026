#pragma once
#include "CoreMinimal.h"
#include "RobotControlMath.h"

class ARobotArm;
class USceneComponent;
class UStaticMeshComponent;
class UStaticMesh;

/** Joint-space geometry only. The caller retains PID, clock, sensors and process ownership. */
struct TEST1_API FRobotMotionPlanner
{
    using FStateCheck = TFunction<bool(const TArray<double>&)>;
    static double Duration(const TArray<FRobotJointDefinition>& Joints,const TArray<double>& A,const TArray<double>& B);
    /** Select a full-turn equivalent within the configured finite interval. */
    static bool ClosestEquivalent(const FRobotJointDefinition& Joint,double Near,double& Angle);
    static bool Edge(const TArray<double>& A,const TArray<double>& B,const FStateCheck& Clear);
    static bool Plan(const TArray<FRobotJointDefinition>& Joints,const TArray<double>& Start,
        const TArray<double>& Goal,const FStateCheck& Clear,TArray<TArray<double>>& Path);
};

struct FRobotMotionShape
{
    FBox Bounds = FBox(ForceInit);
    FTransform Local;
    int32 Joint = -1;
    bool bPayload = false;
    TWeakObjectPtr<UStaticMesh> Mesh;
};

/** Fresh scene snapshot per request; refreshed while moving for moved/added obstacles.
 * Static meshes are checked even when their UE collision is disabled. Mesh triangles
 * preserve wheel arches; unavailable CPU geometry falls back to conservative bounds.
 */
struct TEST1_API FRobotMotionScene
{
    void Capture(ARobotArm& Arm,USceneComponent* ContactPart,bool Held);
    bool Clear(const ARobotArm& Arm,const TArray<double>& Q,FString& Reason) const;
    void RefreshTransforms();
    bool IsGeometryReady() const;
    bool PrepareGeometry(bool Wait=false) const;
    struct FObstacle
    {
        TWeakObjectPtr<UStaticMeshComponent> Component;
        FTransform World;
        FBox Bounds;
    };
    TArray<FRobotMotionShape> Shapes;
    TArray<FObstacle> Obstacles;
    TWeakObjectPtr<USceneComponent> Contact;
    FTransform ContactInTCP;
    bool bHeld = false;
    bool bHasCollisionPrimitives = false;
};

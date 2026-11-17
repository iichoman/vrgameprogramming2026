#include "RobotMotionPlanner.h"
#include "RobotArm.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "StaticMeshResources.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Engine/OverlapResult.h"
#include "HAL/PlatformTime.h"
#include "Misc/ScopeExit.h"
#include "Misc/App.h"
#include "Async/Async.h"
#include "CoreGlobals.h"

bool FRobotMotionPlanner::ClosestEquivalent(const FRobotJointDefinition& J,double Near,double& Angle)
{
    if(!FMath::IsFinite(Angle)||!FMath::IsFinite(Near)||!FMath::IsFinite(J.MinRadians)||!FMath::IsFinite(J.MaxRadians)||J.MinRadians>J.MaxRadians)return false;
    constexpr double Turn=2*UE_DOUBLE_PI;
    const double Low=FMath::CeilToDouble((J.MinRadians-Angle)/Turn);
    const double High=FMath::FloorToDouble((J.MaxRadians-Angle)/Turn);
    if(Low>High)return false;
    Angle+=Turn*FMath::Clamp(FMath::RoundToDouble((Near-Angle)/Turn),Low,High);
    return true;
}

double FRobotMotionPlanner::Duration(const TArray<FRobotJointDefinition>& J,const TArray<double>& A,const TArray<double>& B)
{
    double T=.25;
    for(int32 I=0;I<J.Num();++I)
    {
        const double D=FMath::Abs(B[I]-A[I]);
        T=FMath::Max(T,1.875*D/J[I].MaxVelocityRadians);
        T=FMath::Max(T,FMath::Sqrt(6*D/J[I].MaxAccelerationRadians));
    }
    return T;
}
bool FRobotMotionPlanner::Edge(const TArray<double>& A,const TArray<double>& B,const FStateCheck& Clear)
{
    if(A.Num()!=B.Num()||A.IsEmpty())return false;
    double Travel=0;for(int I=0;I<A.Num();++I)Travel+=FMath::Abs(A[I]-B[I]);
    // Limit sum of rotations, not just the fastest joint. At 400 cm reach this
    // is at most ~1.75 cm surface travel per sample. Runtime also checks PID steps.
    const int N=FMath::Max(1,FMath::CeilToInt(Travel/FMath::DegreesToRadians(.25)));
    TArray<double> Q=A;
    for(int K=0;K<=N;++K)
    {for(int I=0;I<A.Num();++I)Q[I]=FMath::Lerp(A[I],B[I],double(K)/N);if(!Clear(Q))return false;}
    return true;
}
bool FRobotMotionPlanner::Plan(const TArray<FRobotJointDefinition>& J,const TArray<double>& Start,
    const TArray<double>& Goal,const FStateCheck& Clear,TArray<TArray<double>>& Path)
{
    Path.Reset();const double Deadline=FPlatformTime::Seconds()+3.;
    const FStateCheck Check=[&](const auto& Q){return FPlatformTime::Seconds()<Deadline&&Clear(Q);};
    if(!Check(Start)||!Check(Goal))return false;
    if(Edge(Start,Goal,Check)){Path.Add(Goal);return true;}
    struct FNode{TArray<double> Q;int Parent=-1;};
    TArray<FNode> Trees[2];Trees[0].Add({Start,-1});Trees[1].Add({Goal,-1});FRandomStream Random(719);
    auto Distance=[](const TArray<double>& A,const TArray<double>& B){double D=0;for(int I=0;I<A.Num();++I)D+=FMath::Square(A[I]-B[I]);return D;};
    auto Extend=[&](TArray<FNode>& Tree,const TArray<double>& Target)
    {
        int Nearest=0;double Best=Distance(Tree[0].Q,Target);
        for(int I=1;I<Tree.Num();++I){const double D=Distance(Tree[I].Q,Target);if(D<Best){Best=D;Nearest=I;}}
        if(Best<1.e-12)return Nearest;
        auto Sample=Target;const auto& From=Tree[Nearest].Q;
        const double Scale=FMath::Min(1.,.15/FMath::Sqrt(Best));
        for(int I=0;I<J.Num();++I)Sample[I]=FMath::Lerp(From[I],Sample[I],Scale);
        return Edge(From,Sample,Check)?Tree.Add({Sample,Nearest}):INDEX_NONE;
    };
    for(int Attempt=0;Attempt<480;++Attempt)
    {
        if(FPlatformTime::Seconds()>=Deadline)break;
        const int Side=Attempt%2;auto Sample=Trees[1-Side][0].Q;
        const double Along=Random.FRand();
        if(Attempt%5)for(int I=0;I<J.Num();++I)
            Sample[I]=Attempt%3?FMath::Clamp(FMath::Lerp(Start[I],Goal[I],Along)+Random.FRandRange(-.35,.35),J[I].MinRadians,J[I].MaxRadians)
                :Random.FRandRange(J[I].MinRadians,J[I].MaxRadians);
        const int Added=Extend(Trees[Side],Sample);if(Added==INDEX_NONE)continue;
        int Joined=INDEX_NONE;
        for(int Step=0;Step<64;++Step)
        {
            Joined=Extend(Trees[1-Side],Trees[Side][Added].Q);if(Joined==INDEX_NONE)break;
            if(Distance(Trees[1-Side][Joined].Q,Trees[Side][Added].Q)<1.e-12)break;
            Joined=INDEX_NONE;
        }
        if(Joined==INDEX_NONE)continue;
        const int Ends[]={Side==0?Added:Joined,Side==0?Joined:Added};
        TArray<TArray<double>> Reverse;
        for(int I=Ends[0];I>0;I=Trees[0][I].Parent)Reverse.Add(Trees[0][I].Q);
        for(int I=Reverse.Num()-1;I>=0;--I)Path.Add(Reverse[I]);
        for(int I=Trees[1][Ends[1]].Parent;I>=0;I=Trees[1][I].Parent)Path.Add(Trees[1][I].Q);
        if(Path.IsEmpty()||Distance(Path.Last(),Goal)>1.e-12)Path.Add(Goal);
        // Deterministic shortcut: keep only collision-checked segments.
        auto Last=Start;
        for(int I=0;I<Path.Num();++I)
        {for(int K=Path.Num()-1;K>I;--K)if(Edge(Last,Path[K],Check)){Path.RemoveAt(I,K-I);break;}Last=Path[I];}
        return true;
    }
    return false;
}

namespace
{
struct FTriangle{FVector A,B,C;FBox Bounds;};
struct FNode{FBox Bounds=FBox(ForceInit);int Start=0,Count=0,Left=-1,Right=-1;};
struct FMeshTree
{
    TArray<FTriangle> Triangles;TArray<FNode> Nodes;
    bool bClosed=false;
    TFuture<void> Work;
    int Build(int Start,int Count)
    {
        const int Index=Nodes.AddDefaulted();FNode N;N.Start=Start;N.Count=Count;
        for(int I=Start;I<Start+Count;++I)N.Bounds+=Triangles[I].Bounds;
        if(Count>12)
        {
            const FVector Size=N.Bounds.GetSize();const int Axis=Size.X>Size.Y?(Size.X>Size.Z?0:2):(Size.Y>Size.Z?1:2);
            MakeArrayView(Triangles.GetData()+Start,Count).Sort([Axis](const auto& A,const auto& B){return A.Bounds.GetCenter()[Axis]<B.Bounds.GetCenter()[Axis];});
            N.Left=Build(Start,Count/2);N.Right=Build(Start+Count/2,Count-Count/2);
        }
        Nodes[Index]=N;return Index;
    }
};
using FRobotGeometryTree = TSharedPtr<FMeshTree,ESPMode::ThreadSafe>;
TMap<TWeakObjectPtr<UStaticMesh>,FRobotGeometryTree>& GeometryCache()
{static TMap<TWeakObjectPtr<UStaticMesh>,FRobotGeometryTree> Cache;return Cache;}
bool Ready(const FRobotGeometryTree& Tree){return Tree&&Tree->Work.IsValid()&&Tree->Work.IsReady();}
FRobotGeometryTree Geometry(UStaticMesh* Mesh)
{
    auto& Cache=GeometryCache();if(auto* Found=Cache.Find(Mesh))return *Found;
    auto Tree=MakeShared<FMeshTree,ESPMode::ThreadSafe>();
    // Copy render buffers on the game thread. Workers own plain arrays only;
    // no component/asset/world access can outlive PIE or race garbage collection.
    TArray<FVector3f> Positions;TArray<uint32> Indices;
    const auto* Data=Mesh->GetRenderData();
    if(Data&&!Data->LODResources.IsEmpty())
    {
        const auto& LOD=Data->LODResources[0];const auto& V=LOD.VertexBuffers.PositionVertexBuffer;
        const auto View=LOD.IndexBuffer.GetArrayView();
        if(V.GetNumVertices()>0&&V.GetVertexData()&&View.Num()>0)
        {
            Positions.Reserve(V.GetNumVertices());for(uint32 I=0;I<V.GetNumVertices();++I)Positions.Add(V.VertexPosition(I));
            Indices.Reserve(View.Num());for(int I=0;I<View.Num();++I)Indices.Add(View[I]);
        }
    }
    const FString Name=Mesh->GetName();
    Tree->Work=Async(EAsyncExecution::ThreadPool,[Tree,Positions=MoveTemp(Positions),Indices=MoveTemp(Indices),Name]()
    {
        const double Started=FPlatformTime::Seconds();
        Tree->Triangles.Reserve(Indices.Num()/3);
        for(int I=0;I+2<Indices.Num();I+=3)
        {
            FTriangle T;T.A=FVector(Positions[Indices[I]]);T.B=FVector(Positions[Indices[I+1]]);T.C=FVector(Positions[Indices[I+2]]);
            T.Bounds=FBox(ForceInit);T.Bounds+=T.A;T.Bounds+=T.B;T.Bounds+=T.C;Tree->Triangles.Add(T);
        }
        if(!Tree->Triangles.IsEmpty())
        {
            TMap<FVector,int32> Vertices;TMap<uint64,int32> Edges;
            auto Vertex=[&](FVector P){if(const auto* I=Vertices.Find(P))return *I;const int I=Vertices.Num();Vertices.Add(P,I);return I;};
            for(const auto& T:Tree->Triangles)
            {
                const int V[]={Vertex(T.A),Vertex(T.B),Vertex(T.C)};
                for(int I=0;I<3;++I){const uint32 A=FMath::Min(V[I],V[(I+1)%3]),B=FMath::Max(V[I],V[(I+1)%3]);++Edges.FindOrAdd((uint64(A)<<32)|B);}
            }
            Tree->bClosed=Tree->Triangles.Num()>=4;
            for(const auto& Edge:Edges)if(Edge.Value!=2){Tree->bClosed=false;break;}
            Tree->Build(0,Tree->Triangles.Num());
        }
        UE_LOG(LogTemp,Display,TEXT("MOTION_GEOMETRY_ASYNC %s triangles=%d worker_ms=%.2f"),*Name,Tree->Triangles.Num(),1000*(FPlatformTime::Seconds()-Started));
    });
    Cache.Add(Mesh,Tree);return Tree;
}
bool TriangleBox(const FVector& A,const FVector& B,const FVector& C,const FVector& E)
{
    auto Separates=[&](const FVector& Axis){const double R=FVector::DotProduct(Axis.GetAbs(),E);
        const double X=FVector::DotProduct(A,Axis),Y=FVector::DotProduct(B,Axis),Z=FVector::DotProduct(C,Axis);
        return FMath::Min3(X,Y,Z)>R||FMath::Max3(X,Y,Z)<-R;};
    for(int I=0;I<3;++I){FVector Axis=FVector::ZeroVector;Axis[I]=1;if(Separates(Axis))return false;}
    if(Separates(FVector::CrossProduct(B-A,C-A)))return false;
    for(const FVector Edge:{B-A,C-B,A-C})for(int I=0;I<3;++I)
    {FVector Axis=FVector::ZeroVector;Axis[I]=1;if(Separates(FVector::CrossProduct(Edge,Axis)))return false;}
    return true;
}
bool HitsTree(const FMeshTree& Tree,int Index,const FBox& Query,const FMatrix& ToBox,const FVector& Extent)
{
    const auto& N=Tree.Nodes[Index];if(!N.Bounds.Intersect(Query))return false;
    if(N.Left>=0)return HitsTree(Tree,N.Left,Query,ToBox,Extent)||HitsTree(Tree,N.Right,Query,ToBox,Extent);
    for(int I=N.Start;I<N.Start+N.Count;++I)
    {const auto& T=Tree.Triangles[I];if(T.Bounds.Intersect(Query)&&TriangleBox(FVector(ToBox.TransformPosition(T.A)),FVector(ToBox.TransformPosition(T.B)),FVector(ToBox.TransformPosition(T.C)),Extent))return true;}
    return false;
}
void RayHits(const FMeshTree& Tree,int Index,const FVector& P,const FVector& D,TArray<double>& Hits)
{
    const auto& N=Tree.Nodes[Index];double Near=0,Far=1.e9;
    for(int I=0;I<3;++I)
    {double A=(N.Bounds.Min[I]-P[I])/D[I],B=(N.Bounds.Max[I]-P[I])/D[I];if(A>B)Swap(A,B);Near=FMath::Max(Near,A);Far=FMath::Min(Far,B);if(Near>Far)return;}
    if(N.Left>=0){RayHits(Tree,N.Left,P,D,Hits);RayHits(Tree,N.Right,P,D,Hits);return;}
    for(int I=N.Start;I<N.Start+N.Count;++I)
    {
        const auto& T=Tree.Triangles[I];const FVector E=T.B-T.A,F=T.C-T.A,H=FVector::CrossProduct(D,F);
        const double Det=FVector::DotProduct(E,H);if(FMath::Abs(Det)<1.e-10)continue;
        const FVector S=P-T.A;const double U=FVector::DotProduct(S,H)/Det;if(U<0||U>1)continue;
        const FVector R=FVector::CrossProduct(S,E);const double V=FVector::DotProduct(D,R)/Det;if(V<0||U+V>1)continue;
        const double Distance=FVector::DotProduct(F,R)/Det;if(Distance>1.e-5)Hits.Add(Distance);
    }
}
bool Inside(const FMeshTree& Tree,const FVector& P)
{
    if(!Tree.bClosed||!Tree.Nodes[0].Bounds.IsInside(P))return false;
    TArray<double> Hits;RayHits(Tree,0,P,FVector(1,.137,.071),Hits);Hits.Sort();
    int Count=0;double Last=-1;
    for(double D:Hits)if(D-Last>1.e-4){++Count;Last=D;}
    return Count%2==1;
}
bool TrianglesIntersect(const FTriangle& A,const FVector& B0,const FVector& B1,const FVector& B2)
{
    const FVector EA[]={A.B-A.A,A.C-A.B,A.A-A.C},EB[]={B1-B0,B2-B1,B0-B2};
    const FVector NA=FVector::CrossProduct(EA[0],EA[1]),NB=FVector::CrossProduct(EB[0],EB[1]);
    if(NA.IsNearlyZero(1.e-10)||NB.IsNearlyZero(1.e-10))return false;
    auto Separate=[&](const FVector& Axis)
    {
        const double A0=FVector::DotProduct(A.A,Axis),A1=FVector::DotProduct(A.B,Axis),A2=FVector::DotProduct(A.C,Axis);
        const double X=FVector::DotProduct(B0,Axis),Y=FVector::DotProduct(B1,Axis),Z=FVector::DotProduct(B2,Axis);
        return FMath::Min3(A0,A1,A2)>FMath::Max3(X,Y,Z)+1.e-8||FMath::Min3(X,Y,Z)>FMath::Max3(A0,A1,A2)+1.e-8;
    };
    if(Separate(NA)||Separate(NB))return false;
    for(const auto& E:EA)for(const auto& F:EB)if(Separate(FVector::CrossProduct(E,F)))return false;
    for(const auto& E:EA)if(Separate(FVector::CrossProduct(NA,E)))return false;
    for(const auto& E:EB)if(Separate(FVector::CrossProduct(NB,E)))return false;
    return true;
}
bool MeshesIntersect(const FMeshTree& A,int IA,const FMeshTree& B,int IB,const FMatrix& BtoA,FVector* ContactPoint=nullptr)
{
    const auto& NA=A.Nodes[IA];const auto& NB=B.Nodes[IB];const auto BoxB=NB.Bounds.TransformBy(BtoA);
    if(!NA.Bounds.Intersect(BoxB))return false;
    if(NA.Left>=0&&(NB.Left<0||NA.Count>NB.Count))
        return MeshesIntersect(A,NA.Left,B,IB,BtoA,ContactPoint)||MeshesIntersect(A,NA.Right,B,IB,BtoA,ContactPoint);
    if(NB.Left>=0)return MeshesIntersect(A,IA,B,NB.Left,BtoA,ContactPoint)||MeshesIntersect(A,IA,B,NB.Right,BtoA,ContactPoint);
    for(int J=NB.Start;J<NB.Start+NB.Count;++J)
    {
        const auto& T=B.Triangles[J];const FVector P(BtoA.TransformPosition(T.A)),Q(BtoA.TransformPosition(T.B)),R(BtoA.TransformPosition(T.C));
        FBox Bounds(ForceInit);Bounds+=P;Bounds+=Q;Bounds+=R;
        for(int I=NA.Start;I<NA.Start+NA.Count;++I)if(A.Triangles[I].Bounds.Intersect(Bounds)&&TrianglesIntersect(A.Triangles[I],P,Q,R)){if(ContactPoint)*ContactPoint=(P+Q+R)/3;return true;}
    }
    return false;
}
bool Visible(const UStaticMeshComponent* C)
{
    return IsValid(C)&&C->IsRegistered()&&C->GetStaticMesh()&&C->IsVisible()&&!C->bHiddenInGame
        &&C->GetOwner()&&!C->GetOwner()->IsHidden()&&!C->ComponentHasTag(TEXT("MotionIgnore"));
}
bool InPart(const USceneComponent* C,const USceneComponent* Root){return Root&&(C==Root||C->IsAttachedTo(Root));}
}

void FRobotMotionScene::Capture(ARobotArm& Arm,USceneComponent* Part,bool Held)
{
    Contact=Part;bHeld=Held;Shapes.Reset();Obstacles.Reset();bHasCollisionPrimitives=false;
    if(IsValid(Part))ContactInTCP=Part->GetComponentTransform().GetRelativeTransform(Arm.GetTCPWorld());
    for(const auto& B:Arm.MeshBindings)if(B.Mesh&&B.JointIndex>=0)
        Shapes.Add({B.Mesh->GetBoundingBox(),B.RelativePoseCm,B.JointIndex,false,B.Mesh.Get()});
    for(TActorIterator<AActor> It(Arm.GetWorld());It;++It)
    {
        if(*It==&Arm)continue;
        TInlineComponentArray<UPrimitiveComponent*> Primitives(*It);
        for(auto* C:Primitives)if(!Cast<UStaticMeshComponent>(C)&&C->IsQueryCollisionEnabled()
            &&C->GetCollisionResponseToChannel(ECC_PhysicsBody)==ECR_Block&&!C->ComponentHasTag(TEXT("MotionIgnore")))bHasCollisionPrimitives=true;
        if(It->IsHidden())continue; // Hidden collision volumes above still participate.
        TInlineComponentArray<UStaticMeshComponent*> Components(*It);
        for(auto* C:Components)if(Visible(C))
        {
            if(Held&&InPart(C,Part))
                Shapes.Add({C->GetStaticMesh()->GetBoundingBox(),C->GetComponentTransform().GetRelativeTransform(Arm.GetTCPWorld()),5,true,C->GetStaticMesh()});
            else
            {
                const auto World=C->GetComponentTransform();
                const auto Bounds=C->GetStaticMesh()->GetBoundingBox().TransformBy(World);
                // Broad phase only. An object can move into this volume later;
                // membership is refreshed at 10 Hz while servo transforms at 120 Hz.
                if(Bounds.ComputeSquaredDistanceToPoint(Arm.GetActorLocation())<FMath::Square(800.))
                    Obstacles.Add({C,World,Bounds});
            }
        }
    }
}
bool FRobotMotionScene::IsGeometryReady() const
{
    auto Check=[](UStaticMesh* M){const auto* T=M?GeometryCache().Find(M):nullptr;return !M||(T&&Ready(*T));};
    for(const auto& S:Shapes)if(!Check(S.Mesh.Get()))return false;
    for(const auto& O:Obstacles)if(auto* C=O.Component.Get())if(Visible(C)&&!Check(C->GetStaticMesh()))return false;
    return true;
}
bool FRobotMotionScene::PrepareGeometry(bool Wait) const
{
    if(IsGeometryReady())return true;
    static uint64 Frame=MAX_uint64;static double Used=0;
    const bool Limit=!Wait&&FApp::CanEverRender();
    if(Limit&&Frame!=GFrameCounter){Frame=GFrameCounter;Used=0;}
    if(Limit&&Used>=.002)return false;
    const double Started=FPlatformTime::Seconds(),Deadline=Started+(Limit?.002-Used:.002);
    ON_SCOPE_EXIT{if(Limit)Used+=FPlatformTime::Seconds()-Started;};
    for(auto It=GeometryCache().CreateIterator();It;++It)if(!It.Key().IsValid()&&Ready(It.Value()))It.RemoveCurrent();
    auto Prepare=[&](UStaticMesh* M)
    {
        if(!M)return true;
        if(!GeometryCache().Contains(M)&&!Wait)
        {
            // Bound CPU/memory pressure when a new vehicle enters the station.
            int Running=0;for(const auto& E:GeometryCache())Running+=!Ready(E.Value);
            if(Running>=2||FPlatformTime::Seconds()>=Deadline)return false;
        }
        auto T=Geometry(M);if(Wait)T->Work.Wait();return Ready(T);
    };
    bool Done=true;
    for(const auto& S:Shapes)Done=Prepare(S.Mesh.Get())&&Done;
    for(const auto& O:Obstacles)if(auto* C=O.Component.Get())if(Visible(C))Done=Prepare(C->GetStaticMesh())&&Done;
    return Done;
}
void FRobotMotionScene::RefreshTransforms()
{
    for(auto& O:Obstacles)if(auto* C=O.Component.Get())
    {O.World=C->GetComponentTransform();O.Bounds=C->GetStaticMesh()?C->GetStaticMesh()->GetBoundingBox().TransformBy(O.World):FBox(ForceInit);}
}
bool FRobotMotionScene::Clear(const ARobotArm& Arm,const TArray<double>& Q,FString& Reason) const
{
    const double Started=FPlatformTime::Seconds();
    ON_SCOPE_EXIT{static int SlowLogged=0;const double Ms=1000*(FPlatformTime::Seconds()-Started);if(Ms>8&&SlowLogged++<25)UE_LOG(LogTemp,Display,TEXT("MOTION_QUERY_SLOW %s %.2fms %s"),*Arm.GetName(),Ms,*Reason);};
    FTransform TCP;TArray<FTransform> Frames;
    if(!FRobotKinematics::ForwardKinematics(Arm.GetDefinitions(),Q,Arm.ToolInLastJointCm,TCP,&Frames))return false;
    for(const auto& S:Shapes)
    {
        const FTransform Pose=S.Local*(S.bPayload?TCP:Frames[S.Joint])*Arm.GetActorTransform();
        const FVector Extent=S.Bounds.GetExtent()*Pose.GetScale3D().GetAbs();
        const FTransform BoxPose(Pose.GetRotation(),Pose.TransformPosition(S.Bounds.GetCenter()));
        const FBox BoxBounds=FBox(-Extent,Extent).TransformBy(BoxPose);
        for(const auto& O:Obstacles)
        {
            if(!BoxBounds.Intersect(O.Bounds))continue;
            auto* C=O.Component.Get();if(!Visible(C))continue;
            // The wrist/tool may touch only the explicitly selected part; upper
            // arm/elbow still collide with it. No whole-car collision exemption.
            if(!S.bPayload&&S.Joint>=4&&InPart(C,Contact.Get()))continue;
            const auto& World=O.World;
            const auto Bounds=C->GetStaticMesh()->GetBoundingBox();
            const FMatrix ToMesh=BoxPose.ToMatrixWithScale()*World.ToInverseMatrixWithScale();
            const FBox Query=FBox(-Extent,Extent).TransformBy(ToMesh);
            const auto Tree=Geometry(C->GetStaticMesh());
            const auto Moving=S.Mesh.IsValid()?Geometry(S.Mesh.Get()):FRobotGeometryTree();
            if(!FApp::CanEverRender()){Tree->Work.Wait();if(Moving)Moving->Work.Wait();}
            if(!Ready(Tree)||(Moving&&!Ready(Moving))){Reason=TEXT("충돌 형상 준비 중");return false;}
            bool Hit=false;FVector ContactPoint=FVector::ZeroVector;
            if(Moving&&!Moving->Nodes.IsEmpty()&&!Tree->Nodes.IsEmpty())
            {
                // One precise BVH traversal, rather than first traversing the
                // same mesh against the moving AABB and then doing it again.
                Hit=MeshesIntersect(*Moving,0,*Tree,0,World.ToMatrixWithScale()*Pose.ToInverseMatrixWithScale(),&ContactPoint)
                    ||Inside(*Tree,World.InverseTransformPosition(BoxPose.GetLocation()))
                    ||Inside(*Moving,Pose.InverseTransformPosition(World.TransformPosition(Bounds.GetCenter())));
                if(Hit&&C->GetStaticMesh()->GetPathName()==TEXT("/Engine/BasicShapes/Cube.Cube"))
                {
                    // A mounted tire can touch the floor. Use actual vertices,
                    // not rotated box corners, for a 0.5 mm contact skin on boxes.
                    FBox Actual(ForceInit);const FMatrix IntoObstacle=Pose.ToMatrixWithScale()*World.ToInverseMatrixWithScale();
                    for(const auto& T:Moving->Triangles)
                    {Actual+=FVector(IntoObstacle.TransformPosition(T.A));Actual+=FVector(IntoObstacle.TransformPosition(T.B));Actual+=FVector(IntoObstacle.TransformPosition(T.C));}
                    const FVector Skin=FVector(.05)/World.GetScale3D().GetAbs();
                    const FBox Interior(Bounds.Min+Skin,Bounds.Max-Skin);
                    if(Actual.IsValid&&!Actual.Intersect(Interior))Hit=false;
                }
            }
            else Hit=Tree->Nodes.IsEmpty()?Query.Intersect(Bounds):
                (HitsTree(*Tree,0,Query,World.ToMatrixWithScale()*BoxPose.ToInverseMatrixWithScale(),Extent)||Inside(*Tree,ToMesh.GetOrigin()));
            if(Hit){UE_LOG(LogTemp,Verbose,TEXT("MOTION_CONTACT %s %s approx=%s tcp=%s"),*Arm.GetName(),*C->GetName(),*Pose.TransformPosition(ContactPoint).ToString(),*(TCP*Arm.GetActorTransform()).GetLocation().ToString());Reason=FString::Printf(TEXT("경로 간섭: %s / %s / %s (%s) · 검사 위치 %s · 링크 범위 %s / 구조물 %s"),*C->GetOwner()->GetName(),*C->GetName(),*C->GetStaticMesh()->GetName(),S.bPayload?TEXT("운반 부품"):*FString::Printf(TEXT("링크 %d"),S.Joint+1),*BoxPose.GetLocation().ToCompactString(),*BoxBounds.ToString(),*O.Bounds.ToString());return false;}
        }
        // Landscape, blocking volumes and other primitive collision (static
        // meshes above do not require collision enabled). Sensors normally only
        // block Visibility and are deliberately not physical obstacles.
        if(!bHasCollisionPrimitives)continue;
        TArray<FOverlapResult> Hits;
        FCollisionQueryParams Params(SCENE_QUERY_STAT(RobotMotion),false,&Arm);
        FCollisionObjectQueryParams Objects;Objects.AddObjectTypesToQuery(ECC_WorldStatic);Objects.AddObjectTypesToQuery(ECC_WorldDynamic);Objects.AddObjectTypesToQuery(ECC_PhysicsBody);Objects.AddObjectTypesToQuery(ECC_Pawn);
        Arm.GetWorld()->OverlapMultiByObjectType(Hits,BoxPose.GetLocation(),BoxPose.GetRotation(),Objects,FCollisionShape::MakeBox(Extent),Params);
        for(const auto& Hit:Hits)if(auto* C=Hit.GetComponent())
            if(!Cast<UStaticMeshComponent>(C)&&C->GetCollisionResponseToChannel(ECC_PhysicsBody)==ECR_Block
                &&!C->ComponentHasTag(TEXT("MotionIgnore"))&&!InPart(C,Contact.Get()))
            {Reason=TEXT("지형/충돌 볼륨 간섭: ")+C->GetOwner()->GetName();return false;}
    }
    return true;
}

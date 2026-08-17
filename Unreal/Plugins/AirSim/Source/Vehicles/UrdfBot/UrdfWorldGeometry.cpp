#include "UrdfWorldGeometry.h"

#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Interface_CollisionDataProviderCore.h"
#include "Interfaces/Interface_CollisionDataProvider.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"

#include <map>

DEFINE_LOG_CATEGORY(LogUrdfBot);

namespace UrdfWorldGeometry {

namespace {

constexpr int32 kMaxRejectionSamples = 12;

/// Unreal local-space vertex (cm, component scale not yet applied) -> URDF/Box3D point (m).
///
/// The Y negation is the whole of the handedness change; see UrdfTransform.h, which derives it and
/// is the authority. Scale is baked in here rather than carried on the shape so that a non-uniformly
/// scaled asset — very common in a level — needs no per-shape scale support in the backend.
urdf::Vec3 LocalToUrdf(const FVector& V, const FVector& Scale, float WorldToMeters)
{
    return urdf::Vec3{ static_cast<double>(V.X * Scale.X) / WorldToMeters,
                       static_cast<double>(-V.Y * Scale.Y) / WorldToMeters,
                       static_cast<double>(V.Z * Scale.Z) / WorldToMeters };
}

/// UE and Box3D wind front faces oppositely, and the Y mirror above already swaps them back. So
/// only a genuinely mirrored (negative-determinant) scale needs the winding reversed again.
bool ShouldReverseWinding(const FVector& Scale)
{
    return (Scale.X * Scale.Y * Scale.Z) < 0.0;
}

IInterface_CollisionDataProvider* FindTriMeshProvider(UPrimitiveComponent* Prim)
{
    // Landscape collision components implement the provider themselves.
    if (IInterface_CollisionDataProvider* Direct = Cast<IInterface_CollisionDataProvider>(Prim))
        return Direct;

    if (const UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Prim)) {
        if (UStaticMesh* Mesh = SMC->GetStaticMesh())
            return Cast<IInterface_CollisionDataProvider>(Mesh);
    }
    return nullptr;
}

bool ExtractComplexTriMesh(UPrimitiveComponent* Prim, const FVector& Scale, float WorldToMeters,
                           TArray<urdf::StaticShape>& OutShapes)
{
    IInterface_CollisionDataProvider* Provider = FindTriMeshProvider(Prim);
    if (Provider == nullptr || !Provider->ContainsPhysicsTriMeshData(true)) return false;

    FTriMeshCollisionData TriData;
    if (!Provider->GetPhysicsTriMeshData(&TriData, /*InUseAllTriData=*/true)) return false;
    if (TriData.Vertices.Num() < 3 || TriData.Indices.Num() < 1) return false;

    urdf::StaticShape Shape;
    Shape.kind = urdf::StaticShapeKind::Mesh;
    Shape.points.reserve(TriData.Vertices.Num());
    for (const FVector3f& V : TriData.Vertices)
        Shape.points.push_back(LocalToUrdf(FVector(V), Scale, WorldToMeters));

    const bool bReverse = ShouldReverseWinding(Scale);
    Shape.indices.reserve(TriData.Indices.Num() * 3);
    for (const FTriIndices& Tri : TriData.Indices) {
        Shape.indices.push_back(Tri.v0);
        Shape.indices.push_back(bReverse ? Tri.v2 : Tri.v1);
        Shape.indices.push_back(bReverse ? Tri.v1 : Tri.v2);
    }

    OutShapes.Add(MoveTemp(Shape));
    return true;
}

void AppendHull(TArray<urdf::StaticShape>& OutShapes, const TArray<FVector>& Points,
                const FVector& Scale, float WorldToMeters)
{
    if (Points.Num() < 4) return;

    urdf::StaticShape Shape;
    Shape.kind = urdf::StaticShapeKind::Hull;
    Shape.points.reserve(Points.Num());
    for (const FVector& P : Points)
        Shape.points.push_back(LocalToUrdf(P, Scale, WorldToMeters));

    OutShapes.Add(MoveTemp(Shape));
}

int32 ExtractSimpleCollision(UPrimitiveComponent* Prim, const FVector& Scale, float WorldToMeters,
                             TArray<urdf::StaticShape>& OutShapes)
{
    UBodySetup* Setup = Prim->GetBodySetup();
    if (Setup == nullptr) return 0;

    const FKAggregateGeom& Agg = Setup->AggGeom;
    const int32 Before = OutShapes.Num();

    for (const FKConvexElem& Convex : Agg.ConvexElems) {
        const FTransform ElemTM = Convex.GetTransform();
        TArray<FVector> Points;
        Points.Reserve(Convex.VertexData.Num());
        for (const FVector& V : Convex.VertexData)
            Points.Add(ElemTM.TransformPosition(V));
        AppendHull(OutShapes, Points, Scale, WorldToMeters);
    }

    for (const FKBoxElem& Box : Agg.BoxElems) {
        const FTransform ElemTM(Box.Rotation, Box.Center);
        const FVector He(Box.X * 0.5f, Box.Y * 0.5f, Box.Z * 0.5f);
        TArray<FVector> Corners;
        Corners.Reserve(8);
        for (int32 Sx = -1; Sx <= 1; Sx += 2)
            for (int32 Sy = -1; Sy <= 1; Sy += 2)
                for (int32 Sz = -1; Sz <= 1; Sz += 2)
                    Corners.Add(ElemTM.TransformPosition(FVector(Sx * He.X, Sy * He.Y, Sz * He.Z)));
        AppendHull(OutShapes, Corners, Scale, WorldToMeters);
    }

    // A non-uniformly scaled sphere or capsule cannot stay round, and the minimum axis scale is
    // the conservative choice: the mirrored shape fits inside the visible one, so the robot never
    // collides with something it cannot see.
    const double RadialScale = static_cast<double>(Scale.GetAbsMin());

    for (const FKSphereElem& Sph : Agg.SphereElems) {
        urdf::StaticShape Shape;
        Shape.kind = urdf::StaticShapeKind::Sphere;
        Shape.center_a = LocalToUrdf(Sph.Center, Scale, WorldToMeters);
        Shape.radius = Sph.Radius * RadialScale / WorldToMeters;
        OutShapes.Add(MoveTemp(Shape));
    }

    for (const FKSphylElem& Capsule : Agg.SphylElems) {
        const FTransform ElemTM(Capsule.Rotation, Capsule.Center);
        const float HalfLen = Capsule.Length * 0.5f;
        urdf::StaticShape Shape;
        Shape.kind = urdf::StaticShapeKind::Capsule;
        Shape.center_a = LocalToUrdf(ElemTM.TransformPosition(FVector(0, 0, +HalfLen)), Scale, WorldToMeters);
        Shape.center_b = LocalToUrdf(ElemTM.TransformPosition(FVector(0, 0, -HalfLen)), Scale, WorldToMeters);
        Shape.radius = Capsule.Radius * RadialScale / WorldToMeters;
        OutShapes.Add(MoveTemp(Shape));
    }

    return OutShapes.Num() - Before;
}

/// Shapes for a body that MOVES. Simple collision first (its convex/box/sphere/capsule elements are
/// all legal on a kinematic body); otherwise the tri-mesh vertices are hulled.
///
/// ⚠ Never a Mesh shape. Box3D's mesh shapes are static-only (loose_ends.md #7), so a mesh here
/// would create a body that exists, is reported by the mirror, and collides with nothing — a
/// visible obstacle you drive straight through. The backend refuses them too; this is the layer
/// that stops one ever being built.
int32 ExtractKinematicShapes(UPrimitiveComponent* Prim, const FVector& Scale, float WorldToMeters,
                             TArray<urdf::StaticShape>& OutShapes)
{
    const int32 Before = OutShapes.Num();

    if (ExtractSimpleCollision(Prim, Scale, WorldToMeters, OutShapes) > 0)
        return OutShapes.Num() - Before;

    // No simple collision: hull the cooked tri-mesh's vertices. One convex hull for the whole
    // object, so a concave mirrored obstacle is fatter than it looks.
    TArray<urdf::StaticShape> Tri;
    if (!ExtractComplexTriMesh(Prim, Scale, WorldToMeters, Tri)) return 0;

    for (urdf::StaticShape& Shape : Tri) {
        if (Shape.points.size() < 4) continue;
        urdf::StaticShape Hull;
        Hull.kind = urdf::StaticShapeKind::Hull;
        Hull.points = MoveTemp(Shape.points);  // the point cloud is what b3CreateHull wants
        OutShapes.Add(MoveTemp(Hull));
    }
    return OutShapes.Num() - Before;
}

bool ActorPassesTagFilter(const AActor* Actor, const FMirrorOptions& Options)
{
    for (const FName& Tag : Options.ExcludedTags)
        if (Actor->ActorHasTag(Tag)) return false;

    if (Options.RequiredTags.Num() == 0) return true;

    for (const FName& Tag : Options.RequiredTags)
        if (Actor->ActorHasTag(Tag)) return true;

    return false;
}

/// The per-world memo behind MirrorLevelShared. Weak, so a torn-down level's triangles are not
/// pinned; keyed on the UWorld because that is what "this level" means at runtime.
/// ⚠ Strong, not weak, unlike the cooked-geometry cache. The kinematic source components are
/// TWeakObjectPtr, so nothing here keeps an actor alive; and the result must outlive every robot
/// holding an index into it, which a weak cache cannot promise once the first robot is destroyed.
/// Entries are dropped when the UWorld goes.
std::map<const UWorld*, FMirrorResult>& LevelCache()
{
    static std::map<const UWorld*, FMirrorResult> Cache;
    return Cache;
}

} // namespace

FString FMirrorStats::Report() const
{
    FString S;
    S += TEXT("  actors considered        : ") + FString::FromInt(ActorsConsidered) + TEXT("\n");
    S += TEXT("  components considered    : ") + FString::FromInt(ComponentsConsidered) + TEXT("\n");
    S += TEXT("  components mirrored      : ") + FString::FromInt(ComponentsMirrored) + TEXT("\n");
    S += TEXT("  kinematic (tracked)      : ") + FString::FromInt(ComponentsKinematic)
       + TEXT("  of which vehicles: ") + FString::FromInt(ComponentsVehicle) + TEXT("\n");
    S += TEXT("  shapes                   : ") + FString::FromInt(Shapes) + TEXT("\n");
    S += TEXT("  triangles                : ") + FString::FromInt(static_cast<int32>(Triangles)) + TEXT("\n");
    S += TEXT("  skipped, not blocking    : ") + FString::FromInt(ComponentsSkippedNotBlocking) + TEXT("\n");
    S += TEXT("  skipped, no collision    : ") + FString::FromInt(ComponentsNoCollisionData) + TEXT("\n");
    S += TEXT("  skipped, movable         : ") + FString::FromInt(ComponentsSkippedMovable) + TEXT("\n");
    S += FString::Printf(TEXT("  mirror took              : %.1f ms\n"), ElapsedMs);

    if (ComponentsKinematic > 0) {
        S += TEXT("  ! kinematic bodies track their Unreal pose but are ONE-DIRECTIONAL: they push\n");
        S += TEXT("    the robot, the robot never pushes them. They are also convex hulls, so a\n");
        S += TEXT("    concave one is fatter than it looks.\n");
    }
    if (ComponentsSkippedMovable > 0) {
        S += TEXT("  ! movable colliders are NOT in the physics world - a robot passes through\n");
        S += TEXT("    them however solid they look. Set UrdfMirrorMovable to track them.\n");
        S += TEXT("    First few: ");
        for (int32 i = 0; i < SkippedMovableNames.Num(); ++i)
            S += (i ? TEXT(", ") : TEXT("")) + SkippedMovableNames[i];
        S += TEXT("\n");
    }
    if (bUsedRelaxedFallback) {
        S += TEXT("  ! RELAXED fallback was used: nothing blocked the nominated channel, so any\n");
        S += TEXT("    solid channel was accepted. Usable, but the level's collision setup is not\n");
        S += TEXT("    what was assumed.\n");
    }
    if (ComponentsMirrored == 0 && ComponentsKinematic == 0) {
        S += TEXT("  ! NOTHING was mirrored. The robot has no world to collide with.\n");
    }
    if (RejectionSamples.Num() > 0) {
        S += TEXT("  why components were rejected (sampled):\n");
        for (const FString& R : RejectionSamples)
            S += TEXT("    - ") + R + TEXT("\n");
    }
    return S;
}

namespace {

/// Does a robot plausibly collide with this component?
///
/// Strict: it must block the channel the caller nominated. Relaxed: any of the standard solid
/// channels will do. Both require collision to be *enabled* — a disabled component is not geometry
/// under anybody's reading.
bool AcceptsComponent(UPrimitiveComponent* Prim, const FMirrorOptions& Options, bool bRelaxed)
{
    if (!Prim->IsCollisionEnabled()) return false;

    if (!bRelaxed)
        return Prim->GetCollisionResponseToChannel(Options.BlockingChannel) == ECR_Block;

    static const ECollisionChannel kSolidChannels[] = {
        ECC_Pawn, ECC_Vehicle, ECC_WorldStatic, ECC_WorldDynamic, ECC_PhysicsBody, ECC_Visibility,
    };
    for (ECollisionChannel Channel : kSolidChannels)
        if (Prim->GetCollisionResponseToChannel(Channel) == ECR_Block) return true;

    return false;
}

void MirrorPass(UWorld* World, float WorldToMeters, const FMirrorOptions& Options, bool bRelaxed,
                FMirrorStats& OutStats, FMirrorResult& OutResult);

} // namespace

FMirrorResult MirrorLevel(UWorld* World, float WorldToMeters, const FMirrorOptions& Options,
                          FMirrorStats& OutStats)
{
    OutStats = FMirrorStats();
    FMirrorResult Result;
    if (World == nullptr || WorldToMeters <= 0.0f) return Result;

    MirrorPass(World, WorldToMeters, Options, /*bRelaxed=*/false, OutStats, Result);

    if (OutStats.ComponentsMirrored > 0 || OutStats.ComponentsKinematic > 0 ||
        !Options.bRelaxedFallback)
        return Result;

    UE_LOG(LogUrdfBot, Warning,
           TEXT("No component in the level blocks the nominated channel, so the strict mirror was "
                "empty. Retrying with any solid channel accepted. The level's collision setup is "
                "not what this code assumed - worth fixing properly rather than relying on this."));

    FMirrorStats RelaxedStats;
    FMirrorResult RelaxedResult;
    MirrorPass(World, WorldToMeters, Options, /*bRelaxed=*/true, RelaxedStats, RelaxedResult);
    RelaxedStats.bUsedRelaxedFallback = true;
    OutStats = RelaxedStats;
    return RelaxedResult;
}

namespace {

void MirrorPass(UWorld* World, float WorldToMeters, const FMirrorOptions& Options, bool bRelaxed,
                FMirrorStats& OutStats, FMirrorResult& OutResult)
{

    const double StartSeconds = FPlatformTime::Seconds();

    auto Out = std::make_shared<urdf::StaticWorld>();
    Out->mirrored = true;
    Out->world_to_meters = static_cast<double>(WorldToMeters);

    for (TActorIterator<AActor> It(World); It; ++It) {
        AActor* Actor = *It;
        if (Actor == nullptr || !IsValid(Actor)) continue;

        // ⚠ A vehicle is never STATIC geometry — mirroring a robot into the static world it stands
        // in would weld it to the map. It may still be mirrored KINEMATICALLY, which is what lets a
        // Box3D rover feel a drone or a husky drive into it (§6.0b).
        //
        // A robot must not mirror ITSELF. That exclusion cannot happen here, because one mirror is
        // shared by every robot in the scene; it is applied per robot at registration time instead.
        const bool bIsVehicle = Actor->IsA<APawn>();
        if (bIsVehicle && !Options.bIncludeOtherVehicles) continue;
        if (!ActorPassesTagFilter(Actor, Options)) continue;

        ++OutStats.ActorsConsidered;

        TArray<UPrimitiveComponent*> Prims;
        Actor->GetComponents<UPrimitiveComponent>(Prims);

        for (UPrimitiveComponent* Prim : Prims) {
            if (Prim == nullptr || !IsValid(Prim)) continue;
            ++OutStats.ComponentsConsidered;

            if (!AcceptsComponent(Prim, Options, bRelaxed)) {
                ++OutStats.ComponentsSkippedNotBlocking;
                if (OutStats.RejectionSamples.Num() < kMaxRejectionSamples) {
                    OutStats.RejectionSamples.Add(FString::Printf(
                        TEXT("%s [%s]: not blocking - collision enabled=%d, profile='%s', response=%d"),
                        *GetNameSafe(Actor), *Prim->GetClass()->GetName(),
                        Prim->IsCollisionEnabled() ? 1 : 0,
                        *Prim->GetCollisionProfileName().ToString(),
                        static_cast<int32>(Prim->GetCollisionResponseToChannel(Options.BlockingChannel))));
                }
                continue;
            }

            const bool bMovable = (Prim->Mobility == EComponentMobility::Movable);
            const bool bVehicle = bIsVehicle;

            // Anything that can move is mirrored KINEMATICALLY — tracked frame by frame — rather
            // than baked into the static world at whatever pose it happened to hold at load.
            const bool bKinematic = bMovable || bVehicle;

            if (bKinematic && !((bMovable && Options.bIncludeMovable) ||
                                (bVehicle && Options.bIncludeOtherVehicles))) {
                ++OutStats.ComponentsSkippedMovable;
                if (OutStats.SkippedMovableNames.Num() < 8)
                    OutStats.SkippedMovableNames.Add(GetNameSafe(Actor));
                continue;
            }

            const FTransform ComponentTM = Prim->GetComponentTransform();
            const FVector Scale = ComponentTM.GetScale3D();

            if (bKinematic) {
                TArray<urdf::StaticShape> Shapes;
                if (ExtractKinematicShapes(Prim, Scale, WorldToMeters, Shapes) == 0) {
                    ++OutStats.ComponentsNoCollisionData;
                    continue;
                }

                urdf::KinematicBody Body;
                Body.name = TCHAR_TO_UTF8(*Prim->GetPathName());
                Body.friction = Options.DefaultFriction;
                Body.restitution = Options.DefaultRestitution;

                const FVector KT = ComponentTM.GetTranslation();
                Body.position = urdf::Vec3{ static_cast<double>(KT.X) / WorldToMeters,
                                            static_cast<double>(-KT.Y) / WorldToMeters,
                                            static_cast<double>(KT.Z) / WorldToMeters };
                const FQuat KQ = ComponentTM.GetRotation();
                Body.orientation = urdf::Quat{ -KQ.X, KQ.Y, -KQ.Z, KQ.W };
                Body.shapes.assign(Shapes.GetData(), Shapes.GetData() + Shapes.Num());

                Out->kinematic.push_back(std::move(Body));
                OutResult.KinematicSources.Add(Prim);   // index-aligned with Out->kinematic
                ++OutStats.ComponentsKinematic;
                if (bVehicle) ++OutStats.ComponentsVehicle;
                continue;
            }

            TArray<urdf::StaticShape> Shapes;
            bool bGot = false;
            if (Options.Source != ECollisionSource::Simple)
                bGot = ExtractComplexTriMesh(Prim, Scale, WorldToMeters, Shapes);
            if (!bGot && Options.Source != ECollisionSource::Complex)
                bGot = ExtractSimpleCollision(Prim, Scale, WorldToMeters, Shapes) > 0;

            if (!bGot || Shapes.Num() == 0) {
                ++OutStats.ComponentsNoCollisionData;
                if (OutStats.RejectionSamples.Num() < kMaxRejectionSamples) {
                    UBodySetup* Setup = Prim->GetBodySetup();
                    OutStats.RejectionSamples.Add(FString::Printf(
                        TEXT("%s [%s]: blocks the channel but yielded no geometry - bodysetup=%d, ")
                        TEXT("convex=%d box=%d sphere=%d capsule=%d, trimesh provider=%d"),
                        *GetNameSafe(Actor), *Prim->GetClass()->GetName(), Setup ? 1 : 0,
                        Setup ? Setup->AggGeom.ConvexElems.Num() : -1,
                        Setup ? Setup->AggGeom.BoxElems.Num() : -1,
                        Setup ? Setup->AggGeom.SphereElems.Num() : -1,
                        Setup ? Setup->AggGeom.SphylElems.Num() : -1,
                        FindTriMeshProvider(Prim) ? 1 : 0));
                }
                continue;
            }

            urdf::StaticBody Body;
            Body.name = TCHAR_TO_UTF8(*Prim->GetPathName());
            Body.friction = Options.DefaultFriction;
            Body.restitution = Options.DefaultRestitution;

            // Scale is already baked into the shape points, so only rotation and translation are
            // carried here — mirroring both the frame and the points is what keeps the composition
            // consistent (M(Rp + t) = (MRM)(Mp) + Mt for the Y mirror M).
            const FVector T = ComponentTM.GetTranslation();
            Body.position = urdf::Vec3{ static_cast<double>(T.X) / WorldToMeters,
                                        static_cast<double>(-T.Y) / WorldToMeters,
                                        static_cast<double>(T.Z) / WorldToMeters };

            const FQuat Q = ComponentTM.GetRotation();
            Body.orientation = urdf::Quat{ -Q.X, Q.Y, -Q.Z, Q.W };

            Body.shapes.assign(Shapes.GetData(), Shapes.GetData() + Shapes.Num());
            Out->bodies.push_back(std::move(Body));

            ++OutStats.ComponentsMirrored;
        }
    }

    OutStats.Shapes = static_cast<int32>(Out->shapeCount());
    OutStats.Triangles = static_cast<int64>(Out->triangleCount());
    OutStats.ElapsedMs = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;

    OutResult.World = Out;
}

} // namespace

const FMirrorResult& MirrorLevelShared(UWorld* World, float WorldToMeters,
                                       const FMirrorOptions& Options, FMirrorStats& OutStats,
                                       bool& bOutFromCache)
{
    static const FMirrorResult kEmpty;
    bOutFromCache = false;
    if (World == nullptr) return kEmpty;

    auto& Cache = LevelCache();
    const auto It = Cache.find(World);
    if (It != Cache.end()) {
        bOutFromCache = true;
        return It->second;  // the same StaticWorld pointer — what makes the cook shared downstream
    }

    Cache[World] = MirrorLevel(World, WorldToMeters, Options, OutStats);
    return Cache[World];
}

} // namespace UrdfWorldGeometry

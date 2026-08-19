#include "UrdfWorldGeometry.h"

#include "Chaos/HeightField.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "LandscapeDataAccess.h"
#include "LandscapeHeightfieldCollisionComponent.h"
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
#include <string>
#include <utility>
#include <vector>

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

/// Terrain, read straight out of Chaos.
///
/// ⚠ Landscape is INVISIBLE to every other extractor in this file, and always has been.
/// `ULandscapeHeightfieldCollisionComponent` implements no `IInterface_CollisionDataProvider`
/// (grep the whole UE 5.6 Landscape module: the interface is not mentioned once) and owns no
/// `AggGeom`, so `ExtractComplexTriMesh` and `ExtractSimpleCollision` both come back empty and it
/// lands in the "blocks the channel but yielded no geometry" bucket. The visible `LandscapeComponent`
/// sitting beside it is the RENDER component and is `NoCollision` by design in every map, so the
/// rejection log reads as though the level author disabled terrain collision. Neither reading is
/// true: the terrain is solid in Unreal and was simply never extracted.
///
/// The layout below is the engine's own, from `ExportChaosHeightField` in
/// RecastNavMeshGenerator.cpp — the navmesh builder faces exactly this problem and solves it by
/// walking the Chaos heightfield directly. Copying it means the robot drives on the same surface
/// Chaos collides against, holes and all, rather than on a resampled approximation of it.
///
/// Points come out already in WORLD space, so the emitted body carries an identity transform.
bool ExtractLandscapeHeightfield(UPrimitiveComponent* Prim, float WorldToMeters,
                                 urdf::StaticBody& OutBody)
{
    ULandscapeHeightfieldCollisionComponent* HFC =
        Cast<ULandscapeHeightfieldCollisionComponent>(Prim);
    if (HFC == nullptr) return false;
    if (!HFC->HeightfieldRef.IsValid() || !HFC->HeightfieldRef->HeightfieldGeometry) return false;

    // The full-resolution geometry, not HeightfieldSimpleGeometry. The simple one is a decimated
    // proxy the engine keeps for cheap queries; a rover's wheels are exactly the scale at which
    // that decimation shows up as the ground being in the wrong place.
    const Chaos::FHeightField* HF = HFC->HeightfieldRef->HeightfieldGeometry.GetReference();
    const int32 NumRows = HF->GetNumRows();
    const int32 NumCols = HF->GetNumCols();
    if (NumRows < 2 || NumCols < 2) return false;

    // Samples are stored as bare grid indices and LANDSCAPE_ZSCALE height units; both scales live
    // on the transform rather than in the data. Engine convention, and it is why reading
    // CollisionHeightData by hand gets terrain that is 128x too tall.
    FTransform HFToW = HFC->GetComponentTransform();
    HFToW.MultiplyScale3D(FVector(HFC->CollisionScale, HFC->CollisionScale, LANDSCAPE_ZSCALE));

    urdf::StaticShape Shape;
    Shape.kind = urdf::StaticShapeKind::Mesh;
    Shape.points.reserve(static_cast<size_t>(NumRows) * static_cast<size_t>(NumCols));
    for (int32 Y = 0; Y < NumRows; ++Y) {
        for (int32 X = 0; X < NumCols; ++X) {
            const FVector W = HFToW.TransformPosition(
                FVector(static_cast<double>(X), static_cast<double>(Y),
                        static_cast<double>(HF->GetHeight(Y * NumCols + X))));
            Shape.points.push_back(urdf::Vec3{ static_cast<double>(W.X) / WorldToMeters,
                                               static_cast<double>(-W.Y) / WorldToMeters,
                                               static_cast<double>(W.Z) / WorldToMeters });
        }
    }

    // The engine flips the quad diagonal for a negative-determinant landscape scale; the Y mirror
    // applied above is the same handedness change the rest of this file relies on, so the two
    // cancel exactly as they do in ShouldReverseWinding.
    const bool bMirrored = (HFToW.GetDeterminant() < 0.0);
    Shape.indices.reserve(static_cast<size_t>(NumRows - 1) * static_cast<size_t>(NumCols - 1) * 6);
    for (int32 Y = 0; Y < NumRows - 1; ++Y) {
        for (int32 X = 0; X < NumCols - 1; ++X) {
            // A painted hole is a hole in the solver too, or a robot walks on air over a cave mouth.
            if (HF->IsHole(X, Y)) continue;

            const int32 I0 = Y * NumCols + X;
            int32 I1 = I0 + 1;
            int32 I2 = I0 + NumCols;
            const int32 I3 = I2 + 1;
            if (bMirrored) Swap(I1, I2);

            Shape.indices.push_back(I0);
            Shape.indices.push_back(I3);
            Shape.indices.push_back(I1);
            Shape.indices.push_back(I0);
            Shape.indices.push_back(I2);
            Shape.indices.push_back(I3);
        }
    }
    if (Shape.indices.empty()) return false;  // an entirely holed-out component is not geometry

    OutBody.name = TCHAR_TO_UTF8(*Prim->GetPathName());
    OutBody.position = urdf::Vec3{ 0.0, 0.0, 0.0 };
    OutBody.orientation = urdf::Quat{ 0.0, 0.0, 0.0, 1.0 };
    OutBody.shapes.push_back(std::move(Shape));
    return true;
}

/// One body per INSTANCE of an instanced/foliage mesh component.
///
/// ⚠ Without this, an ISM/HISM is mirrored by the generic path as a single body at the component's
/// own transform. For foliage that transform is the InstancedFoliageActor's, which sits at the map
/// origin — so the level gains one phantom tree at (0,0,0) and loses collision on every real one.
/// The failure is silent and looks like a physics bug: the robot drives through a visible forest
/// and then trips over nothing in an empty field.
///
/// The base geometry is extracted ONCE at unit scale and the per-instance scale is applied to the
/// resulting points. That works because the URDF conversion is a Y sign flip, which commutes with
/// a diagonal scale — and it matters, because `GetPhysicsTriMeshData` is far too expensive to call
/// once per instance on a map with thousands of them.
int32 ExtractInstancedBodies(UInstancedStaticMeshComponent* ISM, float WorldToMeters,
                             const FMirrorOptions& Options, int32 InstanceBudget,
                             std::vector<urdf::StaticBody>& OutBodies, int32& OutOverBudget)
{
    const int32 Count = ISM->GetInstanceCount();
    if (Count <= 0) return 0;

    // ⚠ Instances prefer SIMPLE collision, which is the OPPOSITE of the non-instanced path above.
    // That is not an inconsistency, it is Unreal's own rule: an ISM instance collides against the
    // base asset's simple primitives unless the mesh is authored UseComplexAsSimple. Taking the
    // tri-mesh for every instance would also make the mirror's cost scale as instances x triangles
    // — a few thousand foliage instances of a 5 k-triangle bush is tens of millions of triangles to
    // cook, which freezes the editor at load and is the reason to get this right rather than to be
    // uniformly "accurate".
    UBodySetup* Setup = ISM->GetBodySetup();
    const bool bComplexAsSimple =
        Setup != nullptr && Setup->CollisionTraceFlag == CTF_UseComplexAsSimple;

    TArray<urdf::StaticShape> Base;
    bool bGot = false;
    if (!bComplexAsSimple && Options.Source != ECollisionSource::Complex)
        bGot = ExtractSimpleCollision(ISM, FVector::OneVector, WorldToMeters, Base) > 0;
    if (!bGot && Options.Source != ECollisionSource::Simple)
        bGot = ExtractComplexTriMesh(ISM, FVector::OneVector, WorldToMeters, Base);
    if (!bGot || Base.Num() == 0) return 0;

    int32 Emitted = 0;
    for (int32 i = 0; i < Count; ++i) {
        if (Emitted >= InstanceBudget) {
            OutOverBudget += (Count - i);
            break;
        }

        FTransform InstTM;
        if (!ISM->GetInstanceTransform(i, InstTM, /*bWorldSpace=*/true)) continue;

        const FVector S = InstTM.GetScale3D();
        const bool bReverse = ShouldReverseWinding(S);

        urdf::StaticBody Body;
        Body.name = TCHAR_TO_UTF8(*ISM->GetPathName()) + std::string("#") + std::to_string(i);
        Body.friction = Options.DefaultFriction;
        Body.restitution = Options.DefaultRestitution;

        const FVector T = InstTM.GetTranslation();
        Body.position = urdf::Vec3{ static_cast<double>(T.X) / WorldToMeters,
                                    static_cast<double>(-T.Y) / WorldToMeters,
                                    static_cast<double>(T.Z) / WorldToMeters };
        const FQuat Q = InstTM.GetRotation();
        Body.orientation = urdf::Quat{ -Q.X, Q.Y, -Q.Z, Q.W };

        for (const urdf::StaticShape& Src : Base) {
            urdf::StaticShape Shape = Src;
            for (urdf::Vec3& P : Shape.points) {
                P.x *= static_cast<double>(S.X);
                P.y *= static_cast<double>(S.Y);
                P.z *= static_cast<double>(S.Z);
            }
            // Sphere/capsule carry their extent separately from the point list.
            if (Shape.kind == urdf::StaticShapeKind::Sphere ||
                Shape.kind == urdf::StaticShapeKind::Capsule) {
                Shape.radius *= static_cast<double>(S.GetAbsMin());
                auto ScalePt = [&S](urdf::Vec3& P) {
                    P.x *= static_cast<double>(S.X);
                    P.y *= static_cast<double>(S.Y);
                    P.z *= static_cast<double>(S.Z);
                };
                ScalePt(Shape.center_a);
                ScalePt(Shape.center_b);
            }
            if (bReverse) {
                for (size_t t = 0; t + 2 < Shape.indices.size(); t += 3)
                    std::swap(Shape.indices[t + 1], Shape.indices[t + 2]);
            }
            Body.shapes.push_back(std::move(Shape));
        }

        OutBodies.push_back(std::move(Body));
        ++Emitted;
    }
    return Emitted;
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
    S += TEXT("  landscape components     : ") + FString::FromInt(LandscapeComponentsMirrored)
       + TEXT("  skipped: ") + FString::FromInt(LandscapeComponentsSkipped) + TEXT("\n");
    S += TEXT("  instanced components     : ") + FString::FromInt(InstancedComponents)
       + TEXT("  instances mirrored: ") + FString::FromInt(InstancesMirrored) + TEXT("\n");
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
    if (LandscapeComponentsMirrored == 0 && LandscapeComponentsSkipped > 0) {
        S += TEXT("  ! a landscape exists but NONE of it mirrored - the terrain is not solid to\n");
        S += TEXT("    the solver. Props will still collide, which is what makes this look like a\n");
        S += TEXT("    working mirror right up until the robot falls through the ground.\n");
    }
    if (InstancesSkippedOverBudget > 0) {
        S += FString::Printf(
            TEXT("  ! %d instances were dropped at the UrdfMirrorMaxInstances ceiling. Those\n")
            TEXT("    objects are visible and NOT solid. Raise the ceiling or tag them out.\n"),
            InstancesSkippedOverBudget);
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

            // Kinematic-only pass: everything below builds STATIC geometry, which the memoised
            // mirror already owns. Re-emitting it here would give the robot a second, duplicate
            // copy of the level.
            if (Options.bKinematicOnly) continue;

            // --- terrain ----------------------------------------------------------------------
            //
            // Tried BEFORE the generic extractors, not after, because the generic ones do not fail
            // loudly on a landscape - they return "no geometry" and the component is filed under a
            // rejection reason that is true of the API call and false of the level.
            if (Options.bIncludeLandscape && Prim->IsA<ULandscapeHeightfieldCollisionComponent>()) {
                urdf::StaticBody LandBody;
                LandBody.friction = Options.DefaultFriction;
                LandBody.restitution = Options.DefaultRestitution;
                if (ExtractLandscapeHeightfield(Prim, WorldToMeters, LandBody)) {
                    Out->bodies.push_back(std::move(LandBody));
                    ++OutStats.ComponentsMirrored;
                    ++OutStats.LandscapeComponentsMirrored;
                }
                else {
                    ++OutStats.LandscapeComponentsSkipped;
                    ++OutStats.ComponentsNoCollisionData;
                    if (OutStats.RejectionSamples.Num() < kMaxRejectionSamples) {
                        OutStats.RejectionSamples.Add(FString::Printf(
                            TEXT("%s [%s]: landscape collision present but its Chaos heightfield ")
                            TEXT("is not built - the terrain will have NO collision"),
                            *GetNameSafe(Actor), *Prim->GetClass()->GetName()));
                    }
                }
                continue;
            }

            // --- instanced / foliage geometry -------------------------------------------------
            //
            // Also before the generic path, and for a worse reason: the generic path SUCCEEDS on an
            // ISM component. It reads the base asset's collision and emits one body at the
            // component's transform, which is a plausible-looking wrong answer - a single collider
            // at the foliage actor's origin standing in for every instance in the map.
            if (UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(Prim)) {
                if (!Options.bIncludeInstancedMeshes) {
                    ++OutStats.ComponentsSkippedNotBlocking;
                    continue;
                }
                ++OutStats.InstancedComponents;

                std::vector<urdf::StaticBody> InstanceBodies;
                const int32 Budget = FMath::Max(0, Options.MaxInstances - OutStats.InstancesMirrored);
                const int32 Made = ExtractInstancedBodies(ISM, WorldToMeters, Options, Budget,
                                                          InstanceBodies,
                                                          OutStats.InstancesSkippedOverBudget);
                if (Made == 0) {
                    ++OutStats.ComponentsNoCollisionData;
                    if (OutStats.RejectionSamples.Num() < kMaxRejectionSamples) {
                        OutStats.RejectionSamples.Add(FString::Printf(
                            TEXT("%s [%s]: %d instances, but the base mesh yielded no collision ")
                            TEXT("geometry"),
                            *GetNameSafe(Actor), *Prim->GetClass()->GetName(),
                            ISM->GetInstanceCount()));
                    }
                    continue;
                }

                for (urdf::StaticBody& B : InstanceBodies)
                    Out->bodies.push_back(std::move(B));
                OutStats.InstancesMirrored += Made;
                ++OutStats.ComponentsMirrored;
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

FMirrorResult MirrorVehicles(UWorld* World, float WorldToMeters, const FMirrorOptions& Options,
                             FMirrorStats& OutStats)
{
    FMirrorOptions Opts = Options;
    Opts.bKinematicOnly = true;
    return MirrorLevel(World, WorldToMeters, Opts, OutStats);
}

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

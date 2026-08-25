#include "SimMode/PhysicsCollisionDebugDraw.h"

#include "SimMode/CoordinatedPhysicsScene.h"
#include "Vehicles/UrdfBot/UrdfTransform.h"

#include "Camera/PlayerCameraManager.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace PhysicsCollisionDebugDraw {
namespace {

TAutoConsoleVariable<int32> CVarEnabled(
    TEXT("airsim.PhysicsDebugDraw"), 0,
    TEXT("Draw the physics solver's own collision geometry as wireframes over the level.\n")
    TEXT("0: off (default)\n")
    TEXT("1: on"),
    ECVF_Default);

TAutoConsoleVariable<float> CVarRadius(
    TEXT("airsim.PhysicsDebugDrawRadius"), 15.0f,
    TEXT("Metres around the camera to draw. 0 draws the whole scene, which on a mirrored level is ")
    TEXT("tens of thousands of prisms - bounded on purpose."),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarWorld(
    TEXT("airsim.PhysicsDebugDrawWorld"), 1,
    TEXT("Include world geometry: the mirrored level, the ground height field and any ground plane."),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarRobots(
    TEXT("airsim.PhysicsDebugDrawRobots"), 1,
    TEXT("Include the robots' own link collision geometry."), ECVF_Default);

TAutoConsoleVariable<int32> CVarMaxGeoms(
    TEXT("airsim.PhysicsDebugDrawMaxGeoms"), 4000,
    TEXT("Hard cap on geoms drawn per frame. Overflow is reported, never silently dropped."),
    ECVF_Default);

TAutoConsoleVariable<FString> CVarVehicle(
    TEXT("airsim.PhysicsDebugDrawVehicle"), TEXT(""),
    TEXT("Draw only geometry whose label starts with this vehicle name. Empty draws every ")
    TEXT("vehicle. Use it to look at one engine's robot at a time in a mixed comparison."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarThickness(
    TEXT("airsim.PhysicsDebugDrawThickness"), 0.6f,
    TEXT("Wireframe line thickness, in Unreal units."), ECVF_Default);

TAutoConsoleVariable<int32> CVarHeightField(
    TEXT("airsim.PhysicsDebugDrawHeightField"), 1,
    TEXT("Draw the ground height field grid. Separate from the rest of the world geometry because ")
    TEXT("it is by far the densest thing on screen and is usually not what is being investigated."),
    ECVF_Default);

/// ⚠ Colour carries the BACKEND and the PROVENANCE, which are the two things a side-by-side
/// comparison has to be able to tell apart at a glance. Cyan is Box3D, orange is MuJoCo — so a
/// legacy run with one rover on each engine shows two distinctly coloured robots. Green and yellow
/// are world geometry, and there the split is provenance: green was read back out of the solver,
/// yellow is what we submitted and have not verified it kept. Someone looking at yellow is looking
/// at our claim, and must not mistake it for the solver's answer.
constexpr FColor kBox3DRobot(80, 200, 255);
constexpr FColor kMuJoCoRobot(255, 150, 50);
constexpr FColor kOtherRobot(200, 120, 255);
constexpr FColor kWorldRealised(60, 220, 90);
constexpr FColor kWorldSubmitted(230, 200, 60);
constexpr FColor kHeightField(120, 120, 200);
/// A geom whose contact margin inflates it beyond what is drawn. Red, because it is the one case
/// where the wireframe is honestly not the collision boundary.
constexpr FColor kMargin(255, 70, 70);

FColor ColorFor(const urdf::CollisionShape& Geom, const std::string& Backend)
{
    if (Geom.kind == urdf::CollisionShape::Kind::HeightField) return kHeightField;
    if (Geom.is_world)
        return Geom.provenance == urdf::CollisionShape::Provenance::Realised ? kWorldRealised
                                                                                 : kWorldSubmitted;
    if (Backend == "box3d") return kBox3DRobot;
    if (Backend == "mujoco") return kMuJoCoRobot;
    return kOtherRobot;
}

/// Compose a geom-local point into Unreal world space.
///
/// ⚠ Rotate in the SOLVER frame and mirror once at the end. Mirroring the point and then rotating
/// it by the mirrored quaternion is a different operation, and the difference is a reflection - the
/// geometry lands in a plausible wrong place rather than an obviously wrong one.
FVector LocalToUnreal(const urdf::CollisionShape& Geom, const urdf::Vec3& Local,
                      float WorldToMeters)
{
    const urdf::Quat& q = Geom.orientation;
    const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    const urdf::Vec3 world{
        Geom.position.x + Local.x * (1 - 2 * (yy + zz)) + Local.y * (2 * (xy - wz)) +
            Local.z * (2 * (xz + wy)),
        Geom.position.y + Local.x * (2 * (xy + wz)) + Local.y * (1 - 2 * (xx + zz)) +
            Local.z * (2 * (yz - wx)),
        Geom.position.z + Local.x * (2 * (xz - wy)) + Local.y * (2 * (yz + wx)) +
            Local.z * (1 - 2 * (xx + yy))
    };
    return UrdfTransform::toFVector(world, WorldToMeters);
}

void DrawMesh(UWorld* World, const urdf::CollisionShape& Geom, float WorldToMeters,
              const FColor& Color, float Thickness)
{
    if (Geom.vertices.empty() || Geom.indices.size() < 3) return;

    TArray<FVector> Points;
    Points.Reserve(Geom.vertices.size());
    for (const urdf::Vec3& v : Geom.vertices)
        Points.Add(LocalToUnreal(Geom, v, WorldToMeters));

    // ⚠ Draw each EDGE once. A closed mesh shares every edge between two triangles, so the naive
    // loop draws everything twice - which at 20,000 level prisms is 120,000 redundant lines a
    // frame, and the overlay itself becomes the reason the sim is slow.
    TSet<uint64> Seen;
    Seen.Reserve(Geom.indices.size());
    auto Edge = [&](int32 A, int32 B) {
        if (A == B) return;
        if (!Points.IsValidIndex(A) || !Points.IsValidIndex(B)) return;
        const uint64 Key = (static_cast<uint64>(FMath::Min(A, B)) << 32) |
                           static_cast<uint32>(FMath::Max(A, B));
        bool bAlready = false;
        Seen.Add(Key, &bAlready);
        if (bAlready) return;
        DrawDebugLine(World, Points[A], Points[B], Color, /*bPersistent=*/false, /*LifeTime=*/-1.f,
                      /*DepthPriority=*/0, Thickness);
    };

    for (size_t t = 0; t + 2 < Geom.indices.size(); t += 3) {
        Edge(Geom.indices[t], Geom.indices[t + 1]);
        Edge(Geom.indices[t + 1], Geom.indices[t + 2]);
        Edge(Geom.indices[t + 2], Geom.indices[t]);
    }
}

void DrawHeightField(UWorld* World, const urdf::CollisionShape& Geom, float WorldToMeters,
                     const FVector& FocusUnreal, bool bHaveFocus, float RadiusMetres,
                     float Thickness)
{
    if (Geom.rows < 2 || Geom.cols < 2) return;
    const double half_x = Geom.half_extents.x;
    const double half_y = Geom.half_extents.y;
    const double step_x = 2.0 * half_x / (Geom.cols - 1);
    const double step_y = 2.0 * half_y / (Geom.rows - 1);

    auto At = [&](int row, int col) {
        const size_t i = static_cast<size_t>(row) * Geom.cols + col;
        const double h = i < Geom.heights.size() ? Geom.heights[i] : 0.0;
        return LocalToUnreal(Geom,
                             urdf::Vec3{ -half_x + step_x * col, -half_y + step_y * row, h },
                             WorldToMeters);
    };

    // ⚠ The grid is filtered PER SAMPLE, not per geom. A height field is one geom covering the
    // whole sampled region, so a geom-level radius test either draws all of it or none of it -
    // and all of it is 5,000-odd lines that hide everything else on screen.
    const float RadiusUU = RadiusMetres * WorldToMeters;
    auto Near = [&](const FVector& P) {
        if (!bHaveFocus || RadiusMetres <= 0.f) return true;
        return FVector::DistSquaredXY(P, FocusUnreal) <= RadiusUU * RadiusUU;
    };

    for (int row = 0; row < Geom.rows; ++row) {
        for (int col = 0; col < Geom.cols; ++col) {
            const FVector P = At(row, col);
            if (!Near(P)) continue;
            if (col + 1 < Geom.cols)
                DrawDebugLine(World, P, At(row, col + 1), kHeightField, false, -1.f, 0, Thickness);
            if (row + 1 < Geom.rows)
                DrawDebugLine(World, P, At(row + 1, col), kHeightField, false, -1.f, 0, Thickness);
        }
    }
}

/// The world-space span of one geom, in metres. The single number behind "is the collision bigger
/// than the asset".
double SpanOf(const urdf::CollisionShape& Geom)
{
    using Kind = urdf::CollisionShape::Kind;
    switch (Geom.kind) {
    case Kind::Sphere: return 2.0 * Geom.radius;
    case Kind::Capsule:
    case Kind::Cylinder: return 2.0 * (Geom.radius + Geom.half_length);
    case Kind::Box:
        return 2.0 * std::max(Geom.half_extents.x,
                              std::max(Geom.half_extents.y, Geom.half_extents.z));
    case Kind::Plane:
    case Kind::HeightField: return 2.0 * std::max(Geom.half_extents.x, Geom.half_extents.y);
    case Kind::Mesh: {
        double lo[3] = { 1e300, 1e300, 1e300 };
        double hi[3] = { -1e300, -1e300, -1e300 };
        for (const urdf::Vec3& v : Geom.vertices) {
            const double c[3] = { v.x, v.y, v.z };
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], c[k]);
                hi[k] = std::max(hi[k], c[k]);
            }
        }
        if (Geom.vertices.empty()) return 0.0;
        return std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    }
    }
    return 0.0;
}

} // namespace

bool IsEnabled()
{
    return CVarEnabled.GetValueOnGameThread() != 0;
}

urdf::CollisionDebugFilter MakeFilter(const FVector& FocusUnreal, float WorldToMeters,
                                      bool bHaveFocus)
{
    urdf::CollisionDebugFilter Filter;
    Filter.include_world = CVarWorld.GetValueOnGameThread() != 0;
    Filter.include_robots = CVarRobots.GetValueOnGameThread() != 0;
    Filter.max_geoms = static_cast<size_t>(FMath::Max(1, CVarMaxGeoms.GetValueOnGameThread()));

    const float Radius = CVarRadius.GetValueOnGameThread();
    if (bHaveFocus && Radius > 0.f) {
        Filter.radius = Radius;
        Filter.center = UrdfTransform::toUrdfVec(FocusUnreal, WorldToMeters);
    }
    return Filter;
}

void Draw(UWorld* World, const urdf::CollisionDebugSnapshot& Snapshot, float WorldToMeters)
{
    if (World == nullptr) return;

    const float Thickness = CVarThickness.GetValueOnGameThread();
    const float Radius = CVarRadius.GetValueOnGameThread();
    const bool bDrawHeightField = CVarHeightField.GetValueOnGameThread() != 0;

    // The height field's per-sample filter needs the focus point again; recovering it from the
    // filter would mean re-deriving the mirror, so it is taken from the camera the same way.
    FVector Focus = FVector::ZeroVector;
    bool bHaveFocus = false;
    if (APlayerController* PC = World->GetFirstPlayerController()) {
        if (PC->PlayerCameraManager != nullptr) {
            Focus = PC->PlayerCameraManager->GetCameraLocation();
            bHaveFocus = true;
        }
    }

    // ⚠ Filtered HERE rather than in the readback, because the name is a display convention the
    // solver knows nothing about, and because a filter that reached into the snapshot would make
    // the reported counts disagree with what the log summary says was found.
    const FString OnlyVehicle = CVarVehicle.GetValueOnGameThread();

    using Kind = urdf::CollisionShape::Kind;
    for (const urdf::CollisionShape& Geom : Snapshot.geoms) {
        if (!OnlyVehicle.IsEmpty() && !Geom.is_world) {
            const FString Label = UTF8_TO_TCHAR(Geom.label.c_str());
            if (!Label.StartsWith(OnlyVehicle)) continue;
        }
        const FColor Color = ColorFor(Geom, Snapshot.backend);
        const FVector Origin = UrdfTransform::toFVector(Geom.position, WorldToMeters);
        const FQuat Rotation = UrdfTransform::toFQuat(Geom.orientation);

        switch (Geom.kind) {
        case Kind::Sphere: {
            const urdf::Vec3 Center =
                Geom.vertices.empty() ? urdf::Vec3{ 0, 0, 0 } : Geom.vertices[0];
            DrawDebugSphere(World, LocalToUnreal(Geom, Center, WorldToMeters),
                            Geom.radius * WorldToMeters, 12, Color, false, -1.f, 0, Thickness);
            break;
        }
        case Kind::Capsule: {
            urdf::Vec3 A{ 0, 0, -Geom.half_length };
            urdf::Vec3 B{ 0, 0, Geom.half_length };
            if (Geom.vertices.size() >= 2) {
                A = Geom.vertices[0];
                B = Geom.vertices[1];
            }
            const FVector PA = LocalToUnreal(Geom, A, WorldToMeters);
            const FVector PB = LocalToUnreal(Geom, B, WorldToMeters);
            const FVector Axis = PB - PA;
            const float HalfLen = 0.5f * Axis.Size() + Geom.radius * WorldToMeters;
            const FQuat CapRot = Axis.IsNearlyZero()
                                     ? Rotation
                                     : FRotationMatrix::MakeFromZ(Axis.GetSafeNormal()).ToQuat();
            DrawDebugCapsule(World, 0.5f * (PA + PB), HalfLen, Geom.radius * WorldToMeters, CapRot,
                             Color, false, -1.f, 0, Thickness);
            break;
        }
        case Kind::Cylinder: {
            const FVector Axis = Rotation.RotateVector(FVector(0, 0, 1));
            const FVector Half = Axis * (Geom.half_length * WorldToMeters);
            DrawDebugCylinder(World, Origin - Half, Origin + Half, Geom.radius * WorldToMeters, 12,
                              Color, false, -1.f, 0, Thickness);
            break;
        }
        case Kind::Box:
            DrawDebugBox(World, Origin,
                         FVector(Geom.half_extents.x, Geom.half_extents.y, Geom.half_extents.z) *
                             WorldToMeters,
                         Rotation, Color, false, -1.f, 0, Thickness);
            break;
        case Kind::Plane: {
            // An infinite plane has no wireframe; a grid the size of the stand-in extent says
            // "there is a plane here" without pretending to a boundary it does not have.
            const double hx = Geom.half_extents.x, hy = Geom.half_extents.y;
            constexpr int kLines = 10;
            for (int i = 0; i <= kLines; ++i) {
                const double fx = -hx + 2.0 * hx * i / kLines;
                const double fy = -hy + 2.0 * hy * i / kLines;
                DrawDebugLine(World, LocalToUnreal(Geom, urdf::Vec3{ fx, -hy, 0 }, WorldToMeters),
                              LocalToUnreal(Geom, urdf::Vec3{ fx, hy, 0 }, WorldToMeters), Color,
                              false, -1.f, 0, Thickness);
                DrawDebugLine(World, LocalToUnreal(Geom, urdf::Vec3{ -hx, fy, 0 }, WorldToMeters),
                              LocalToUnreal(Geom, urdf::Vec3{ hx, fy, 0 }, WorldToMeters), Color,
                              false, -1.f, 0, Thickness);
            }
            break;
        }
        case Kind::HeightField:
            if (bDrawHeightField)
                DrawHeightField(World, Geom, WorldToMeters, Focus, bHaveFocus, Radius, Thickness);
            break;
        case Kind::Mesh:
            DrawMesh(World, Geom, WorldToMeters, Color, Thickness);
            break;
        }

        // The contact boundary when it is NOT the drawn surface. Rare, and worth a separate mark
        // rather than a footnote: a margin makes a geom collide at a distance, which reads on
        // screen as the robot floating.
        if (Geom.margin > 1e-6)
            DrawDebugSphere(World, Origin, (SpanOf(Geom) * 0.5 + Geom.margin) * WorldToMeters, 8,
                            kMargin, false, -1.f, 0, Thickness);
    }

}

void LogSummary(const urdf::CollisionDebugSnapshot& Snapshot)
{
    double WorstSpan = 0;
    FString WorstLabel;
    for (const urdf::CollisionShape& Geom : Snapshot.geoms) {
        const double Span = SpanOf(Geom);
        if (Span > WorstSpan) {
            WorstSpan = Span;
            WorstLabel = UTF8_TO_TCHAR(Geom.label.c_str());
        }
    }

    int32 WorldGeoms = 0, RobotGeoms = 0, Submitted = 0;
    for (const urdf::CollisionShape& Geom : Snapshot.geoms) {
        (Geom.is_world ? WorldGeoms : RobotGeoms)++;
        if (Geom.provenance == urdf::CollisionShape::Provenance::Submitted) ++Submitted;
    }

    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("collision overlay [%s]: %d geoms (%d world, %d robot, %d submitted-not-verified), ")
           TEXT("%d triangles, largest span %.3f m on '%s'"),
           UTF8_TO_TCHAR(Snapshot.backend.c_str()), static_cast<int32>(Snapshot.geoms.size()),
           WorldGeoms, RobotGeoms,
           Submitted, static_cast<int32>(Snapshot.triangleCount()), WorstSpan, *WorstLabel);

    if (Snapshot.omitted > 0) {
        FString Kinds;
        for (const std::string& Kind : Snapshot.omitted_kinds) {
            if (!Kinds.IsEmpty()) Kinds += TEXT(", ");
            Kinds += UTF8_TO_TCHAR(Kind.c_str());
        }
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("collision overlay: %d geoms NOT drawn%s%s. Raise ")
               TEXT("airsim.PhysicsDebugDrawMaxGeoms or narrow airsim.PhysicsDebugDrawRadius - what ")
               TEXT("is on screen is not the whole scene."),
               static_cast<int32>(Snapshot.omitted), Kinds.IsEmpty() ? TEXT("") : TEXT(" - kinds: "),
               Kinds.IsEmpty() ? TEXT("") : *Kinds);
    }
}

} // namespace PhysicsCollisionDebugDraw

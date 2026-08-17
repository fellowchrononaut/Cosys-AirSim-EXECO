#include "UrdfBotPawn.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "UrdfWorldGeometry.h"   // LogUrdfBot

#include "UrdfLinkMount.h"
#include "UrdfTransform.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/Material.h"

#include <cmath>
#include <unordered_map>

AUrdfBotPawn::AUrdfBotPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    root_component_ = CreateDefaultSubobject<USceneComponent>(TEXT("UrdfRoot"));
    RootComponent = root_component_;

    static ConstructorHelpers::FObjectFinder<UClass> pip_camera_class(
        TEXT("Blueprint'/AirSim/Blueprints/BP_PIPCamera.BP_PIPCamera_C'"));
    pip_camera_class_ = pip_camera_class.Succeeded() ? pip_camera_class.Object : nullptr;

    // Engine primitives, used to draw links whose geometry is a URDF box/cylinder/sphere. Unit
    // sized: Cube is 100 uu across, Sphere and Cylinder are 100 uu diameter/height, so a scale of
    // (size_in_metres) maps straight onto them at world_to_meters = 100.
    // ⚠ The base material for procedural link meshes is NOT looked up here.
    //
    // It was, via ConstructorHelpers, and that lookup succeeded — which is what made the problem
    // hard to see: it resolved the bare BasicShapeMaterial, whose own parameter defaults need not be
    // anything visible, and having succeeded it also prevented any better-chosen fallback from ever
    // running. Resolution moved to attachMeshGeometry, where the instance is preferred over the base
    // and the outcome is logged unconditionally.

    static ConstructorHelpers::FObjectFinder<UStaticMesh> box(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> cyl(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> sph(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    box_mesh_ = box.Succeeded() ? box.Object : nullptr;
    cylinder_mesh_ = cyl.Succeeded() ? cyl.Object : nullptr;
    sphere_mesh_ = sph.Succeeded() ? sph.Object : nullptr;
}

void AUrdfBotPawn::BeginPlay()
{
    Super::BeginPlay();
}

void AUrdfBotPawn::Tick(float Delta)
{
    Super::Tick(Delta);
}

void AUrdfBotPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    link_components_.clear();
    link_mount_by_name_.clear();
    links_by_index_.Empty();
    link_mounts_.Empty();
    Super::EndPlay(EndPlayReason);
}

void AUrdfBotPawn::NotifyHit(UPrimitiveComponent* MyComp, AActor* Other, UPrimitiveComponent* OtherComp,
                             bool bSelfMoved, FVector HitLocation, FVector HitNormal,
                             FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
                                           HitNormal, NormalImpulse, Hit);
}

void AUrdfBotPawn::initializeForBeginPlay()
{
    // Link components are created later, by buildFromModel, which the sim api calls once the URDF
    // named in settings has been parsed — the pawn is spawned before its settings are available.
    // Input bindings do not depend on the model, so they are installed here.
    setupInputBindings();
}

/******************* bindings *******************/
// This must be on the pawn: Unreal will not bind keys to a non-UObject.
//
// Same keys as the Car and Skid vehicles, so a URDF rover drives like every other ground vehicle in
// this simulator rather than inventing its own scheme. WASD is added alongside the arrow keys
// because the arrow keys are also the editor's default camera controls in some layouts.
void AUrdfBotPawn::setupInputBindings()
{
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveForward", EKeys::Up, 1), this, this, &AUrdfBotPawn::onMoveForward);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveForward", EKeys::Down, -1), this, this, &AUrdfBotPawn::onMoveForward);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveForward", EKeys::W, 1), this, this, &AUrdfBotPawn::onMoveForward);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveForward", EKeys::S, -1), this, this, &AUrdfBotPawn::onMoveForward);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::Right, 1), this, this, &AUrdfBotPawn::onMoveRight);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::Left, -1), this, this, &AUrdfBotPawn::onMoveRight);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::D, 1), this, this, &AUrdfBotPawn::onMoveRight);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::A, -1), this, this, &AUrdfBotPawn::onMoveRight);

    // Stop, rather than a brake. A URDF robot has no brake to model — commanding zero wheel speed
    // is exactly what "stop" means for a velocity-controlled joint, and pretending otherwise would
    // be inventing dynamics the model does not describe.
    UAirBlueprintLib::BindActionToKey("UrdfStop", EKeys::SpaceBar, this, &AUrdfBotPawn::onStop, true);

    // ⚠ Alternative steering keys, on purpose.
    //
    // A/D are claimed by ManualPoseController ("inputManualLeft"/"inputManualRight") and the arrow
    // keys are the editor's camera controls in several layouts. UE axis bindings do not consume
    // input, so in principle both can coexist — but "in principle" is doing a lot of work in a
    // system where several subsystems bind the same physical keys, and steering demonstrably does
    // not reach the robot while throttle does.
    //
    // Numpad 4/6 are claimed by nothing in this plugin. If steering works on these and not on A/D,
    // the defect is key contention rather than anything in the drive path — which is a diagnosis,
    // not just a workaround.
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::NumPadSix, 1), this, this, &AUrdfBotPawn::onMoveRight);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::NumPadFour, -1), this, this, &AUrdfBotPawn::onMoveRight);

    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveForward", EKeys::Gamepad_RightTriggerAxis, 1), this, this, &AUrdfBotPawn::onMoveForward);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveForward", EKeys::Gamepad_LeftTriggerAxis, -1), this, this, &AUrdfBotPawn::onMoveForward);
    UAirBlueprintLib::BindAxisToKey(FInputAxisKeyMapping("UrdfMoveRight", EKeys::Gamepad_LeftX, 1), this, this, &AUrdfBotPawn::onMoveRight);
}

void AUrdfBotPawn::onMoveForward(float value)
{
    drive_input_.throttle.store(value, std::memory_order_relaxed);
}

void AUrdfBotPawn::onMoveRight(float value)
{
    drive_input_.steering.store(value, std::memory_order_relaxed);
}

void AUrdfBotPawn::onStop()
{
    drive_input_.throttle.store(0.0f, std::memory_order_relaxed);
    drive_input_.steering.store(0.0f, std::memory_order_relaxed);
}

UStaticMesh* AUrdfBotPawn::resolveVisualMesh(const urdf::Geometry& geometry) const
{
    switch (geometry.type) {
    case urdf::GeometryType::Box:      return box_mesh_;
    case urdf::GeometryType::Cylinder: return cylinder_mesh_;
    case urdf::GeometryType::Sphere:   return sphere_mesh_;
    case urdf::GeometryType::Mesh:
        // ⚠ No runtime STL/DAE import exists, so a <mesh> visual cannot be drawn as itself. The
        // link still gets a component and is still posed correctly — it simply has no geometry to
        // show or to be traced by a sensor. Reported rather than substituted: a stand-in box would
        // give the LiDAR a surface the real robot does not have, which is a worse lie than absence.
        return nullptr;
    }
    return nullptr;
}

void AUrdfBotPawn::attachGeometry(USceneComponent* link_component, const urdf::Geometry& g,
                                  const urdf::Origin& origin, const FName& name, bool collidable)
{
    UStaticMesh* mesh = resolveVisualMesh(g);
    if (!mesh) return;

    UStaticMeshComponent* c = NewObject<UStaticMeshComponent>(this, name);
    c->SetupAttachment(link_component);
    c->SetStaticMesh(mesh);
    c->SetMobility(EComponentMobility::Movable);

    // ⚠ Unreal physics OFF, collision ON. Box3D owns this link's motion; letting Chaos simulate it
    // too would give the robot two solvers disagreeing about where it is. Collision stays enabled
    // because it is what LiDAR, echo and distance sensors trace — that is the whole of R2, and
    // disabling it would make the robot invisible to its own sensors.
    c->SetSimulatePhysics(false);
    c->SetCollisionEnabled(collidable ? ECollisionEnabled::QueryOnly : ECollisionEnabled::NoCollision);
    if (collidable) c->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);

    // ⚠ The geometry's <origin> is a transform *within* the link, and it must be applied here. The
    // link component sits at the link frame — that is what sensors mount to and what the pose
    // writeback drives — so a wheel whose cylinder is offset from its link origin would otherwise
    // be drawn at the wrong place while its physics stayed correct.
    const float world_to_meters = 100.0f;  // relative offsets only; matches AirSim's fixed scale
    FTransform rel = UrdfTransform::toFTransform(origin, world_to_meters);

    // Scale the unit primitive onto the URDF dimensions. Engine BasicShapes are 100 uu across and
    // centred on their origin, so at world_to_meters = 100 the scale factor is the size in metres.
    FVector scale(1, 1, 1);
    switch (g.type) {
    case urdf::GeometryType::Box:
        scale = FVector(static_cast<float>(g.box_size.x), static_cast<float>(g.box_size.y),
                        static_cast<float>(g.box_size.z));
        break;
    case urdf::GeometryType::Cylinder:
        // Engine Cylinder is Z-aligned, which is URDF's axis for a cylinder too.
        scale = FVector(static_cast<float>(g.radius * 2.0), static_cast<float>(g.radius * 2.0),
                        static_cast<float>(g.length));
        break;
    case urdf::GeometryType::Sphere:
        scale = FVector(static_cast<float>(g.radius * 2.0));
        break;
    case urdf::GeometryType::Mesh:
        return;  // unreachable: resolveVisualMesh already returned null
    }
    rel.SetScale3D(scale);
    c->SetRelativeTransform(rel);
    c->RegisterComponent();
}

bool AUrdfBotPawn::attachStaticMeshAsset(USceneComponent* link_component, const urdf::Geometry& g,
                                         const urdf::Origin& origin, const FName& name,
                                         bool collidable)
{
    if (mesh_asset_dir_.empty()) return false;

    // "…/meshes/base_link.STL" -> "base_link", then "<dir>/base_link.base_link", which is Unreal's
    // package.object form for an asset imported under its own name.
    std::string stem = g.mesh_filename;
    const size_t slash = stem.find_last_of("/\\");
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    const size_t dot = stem.find_last_of('.');
    if (dot != std::string::npos) stem = stem.substr(0, dot);

    const FString asset_path = FString::Printf(TEXT("%s/%s.%s"),
                                               UTF8_TO_TCHAR(mesh_asset_dir_.c_str()),
                                               UTF8_TO_TCHAR(stem.c_str()),
                                               UTF8_TO_TCHAR(stem.c_str()));
    UStaticMesh* asset = LoadObject<UStaticMesh>(nullptr, *asset_path);
    if (asset == nullptr) {
        // Not an error: a partially imported robot is a legitimate state, and the runtime STL path
        // still draws the link. Logged at Verbose so a fully-procedural robot does not shout.
        UE_LOG(LogUrdfBot, Verbose, TEXT("no static mesh asset at %s - using the runtime STL path"),
               *asset_path);
        return false;
    }

    UStaticMeshComponent* c = NewObject<UStaticMeshComponent>(this, name);
    c->SetupAttachment(link_component);
    c->SetMobility(EComponentMobility::Movable);

    // Same pre-registration rule as the procedural path: these are fields, not setters, and the
    // scene proxy is built at registration.
    c->SetCastShadow(mesh_cast_shadow_);
    c->bCastInsetShadow = mesh_inset_shadow_;
    c->bCastShadowAsTwoSided = mesh_two_sided_shadow_;
    c->bCastContactShadow = mesh_contact_shadow_;

    c->SetStaticMesh(asset);
    c->SetSimulatePhysics(false);
    c->SetCollisionEnabled(collidable ? ECollisionEnabled::QueryOnly
                                      : ECollisionEnabled::NoCollision);
    if (collidable) c->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);

    // ⚠ The Y mirror is applied as a NEGATIVE SCALE here, not by rewriting vertices — an imported
    // asset's vertex data is not ours to touch. Unreal flips backface culling automatically for a
    // negative-determinant scale, so the faces stay correct. This is also why the procedural path
    // reverses winding only for a negative <mesh scale>: the two mechanisms have to agree.
    const float world_to_meters = 100.0f;
    FTransform rel = UrdfTransform::toFTransform(origin, world_to_meters);
    const double k = mesh_asset_scale_;
    rel.SetScale3D(FVector(static_cast<float>(g.mesh_scale.x * k),
                           static_cast<float>(-g.mesh_scale.y * k),
                           static_cast<float>(g.mesh_scale.z * k)));
    c->SetRelativeTransform(rel);
    c->RegisterComponent();

    // ⚠ The asset's own materials are kept. Importing a mesh is how an operator supplies the
    // robot's appearance, so overriding it with the URDF's flat <material> colour would discard
    // the very thing the import was for.
    ++mesh_from_asset_;
    UE_LOG(LogUrdfBot, Log, TEXT("link mesh from asset: %s (scale %.3f)"), *asset_path, k);
    return true;
}

bool AUrdfBotPawn::attachMeshGeometry(USceneComponent* link_component, const urdf::Geometry& g,
                                      const urdf::Origin& origin, const urdf::Material& material,
                                      const FName& name, bool collidable)
{
    TArray<FProcMeshTangent> tangents_out;

    const std::string resolved =
        urdf::resolveMeshPath(g.mesh_filename, urdf_dir_, mesh_search_paths_);
    if (resolved.empty()) {
        UAirBlueprintLib::LogMessageString(
            "UrdfBot WARNING: mesh not found ",
            g.mesh_filename + " - set UrdfMeshSearchPaths. The link is posed correctly but has "
            "nothing to draw and nothing for a sensor to trace.",
            LogDebugLevel::Failure);
        return false;
    }

    // Parsed once per FILE, not once per link: ExoMy names six identical wheel STLs.
    auto it = mesh_cache_.find(resolved);
    if (it == mesh_cache_.end()) {
        const double t0 = FPlatformTime::Seconds();
        try {
            it = mesh_cache_.emplace(resolved, urdf::loadStl(resolved)).first;
            mesh_load_seconds_ += FPlatformTime::Seconds() - t0;
        }
        catch (const std::exception& e) {
            // ⚠ STL only. OBJ is a short addition; DAE/Collada is a real one, and a large fraction
            // of published URDFs use it. Named rather than silently skipped, because "the robot is
            // partly invisible" must not be something the operator has to infer.
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: cannot load mesh ",
                resolved + " (" + e.what() + "). Only STL is supported by the runtime loader.",
                LogDebugLevel::Failure);
            return false;
        }
    }

    const urdf::MeshData& mesh = it->second;
    if (mesh.vertices.size() < 3) return false;

    // ⚠ Optional vertex-cluster decimation.
    //
    // ExoMy's STLs are PRINT-resolution: 220 618 triangles for a 30 cm robot, ~7 500 per wheel.
    // That density is the reason it aliases under Virtual Shadow Maps — VSM allocates pages by
    // screen size, so up close the geometry is finer than the shadow texels and self-shadows into
    // blackness. It is also why the symptom disappears at distance, and why raising
    // r.Shadow.Virtual.NormalBias did nothing while lowering the resolution fixed it: this is
    // geometric aliasing, not depth bias.
    //
    // Snap vertices to a grid, keep the first vertex seen in each cell as its representative, and
    // drop triangles whose corners collapse together. O(n), no topology needed, and it preserves
    // silhouette far better than dropping triangles at random.
    //
    // ⚠ This is also the experiment that separates two confounded explanations. The earlier control
    // (rover4 renders correctly) differed from ExoMy in BOTH component type and triangle count, so
    // it could not distinguish "procedural meshes are the problem" from "dense meshes are the
    // problem". Holding the component type fixed and varying only density decides it.
    std::vector<urdf::Vec3> decimated;
    const std::vector<urdf::Vec3>* verts = &mesh.vertices;
    if (mesh_decimate_grid_ > 0.0) {
        const double g = mesh_decimate_grid_;
        auto cell = [g](const urdf::Vec3& v) -> uint64_t {
            const int64_t x = static_cast<int64_t>(std::llround(v.x / g));
            const int64_t y = static_cast<int64_t>(std::llround(v.y / g));
            const int64_t z = static_cast<int64_t>(std::llround(v.z / g));
            return (static_cast<uint64_t>(x & 0x1FFFFF) << 42) ^
                   (static_cast<uint64_t>(y & 0x1FFFFF) << 21) ^
                    static_cast<uint64_t>(z & 0x1FFFFF);
        };
        std::unordered_map<uint64_t, urdf::Vec3> rep;
        rep.reserve(mesh.vertices.size());
        decimated.reserve(mesh.vertices.size());

        for (size_t t = 0; t + 2 < mesh.vertices.size(); t += 3) {
            const uint64_t c0 = cell(mesh.vertices[t]);
            const uint64_t c1 = cell(mesh.vertices[t + 1]);
            const uint64_t c2 = cell(mesh.vertices[t + 2]);
            // Two corners in one cell means the triangle has collapsed to a line: drop it.
            if (c0 == c1 || c1 == c2 || c0 == c2) continue;
            decimated.push_back(rep.emplace(c0, mesh.vertices[t]).first->second);
            decimated.push_back(rep.emplace(c1, mesh.vertices[t + 1]).first->second);
            decimated.push_back(rep.emplace(c2, mesh.vertices[t + 2]).first->second);
        }
        if (decimated.size() >= 3) {
            mesh_triangles_before_ += static_cast<int64>(mesh.vertices.size() / 3);
            mesh_triangles_after_ += static_cast<int64>(decimated.size() / 3);
            verts = &decimated;
        }
    }

    // URDF metres -> Unreal centimetres, with the Y mirror that is the whole of the handedness
    // change (see UrdfTransform.h, which derives it and is the authority).
    const float world_to_meters = 100.0f;
    const double sx = g.mesh_scale.x * world_to_meters;
    const double sy = g.mesh_scale.y * world_to_meters;
    const double sz = g.mesh_scale.z * world_to_meters;

    const int32 vertex_count = static_cast<int32>(verts->size());
    TArray<FVector> positions;
    TArray<int32> indices;
    TArray<FVector> normals;
    positions.Reserve(vertex_count);
    indices.Reserve(vertex_count);
    normals.Reserve(vertex_count);

    for (const urdf::Vec3& v : *verts)
        positions.Add(FVector(v.x * sx, -v.y * sy, v.z * sz));

    // ⚠ Winding is NOT reversed, and this is settled by OBSERVATION, not by derivation.
    //
    // I derived the opposite twice — that the Y mirror flips a triangle's orientation relative to
    // its solid, so the winding must be flipped back — and built it. It was wrong. The robot
    // rendered correctly with the winding left alone and the normal taken as cross(e1, e2); the
    // black robot was ALWAYS the material (the bare BasicShapeMaterial base rather than
    // BasicShapeMaterial_Inst), and it changed the moment the instance was used, with these lines
    // untouched.
    //
    // The derivation failed because it reasoned about "outward" using right-handed intuition while
    // Unreal is left-handed, and the handedness change is exactly what the mirror performs. That is
    // not a detail worth re-deriving: it is worth deferring to the rendered result, which is cheap
    // to check and cannot be argued with. The rule also matches what the level mirror already does
    // for the same conversion — only a mirrored (negative) scale needs reversing.
    // Base rule: only a mirrored (negative) <scale> reverses. mesh_flip_winding_ inverts that
    // decision wholesale, so the alternative can be tested in one run rather than argued about.
    const bool reverse_winding = ((sx * sy * sz) < 0.0) != mesh_flip_winding_;

    normals.SetNum(vertex_count);
    tangents_out.SetNum(vertex_count);

    for (int32 t = 0; t + 2 < vertex_count; t += 3) {
        const int32 i0 = t;
        const int32 i1 = reverse_winding ? t + 2 : t + 1;
        const int32 i2 = reverse_winding ? t + 1 : t + 2;

        indices.Add(i0);
        indices.Add(i1);
        indices.Add(i2);

        // Taken from the EMITTED order, so the shading normal and the face Unreal rasterises stay
        // consistent whichever branch above ran.
        const FVector e1 = positions[i1] - positions[i0];
        const FVector e2 = positions[i2] - positions[i0];
        const FVector n = FVector::CrossProduct(e1, e2).GetSafeNormal();

        // ⚠ Tangents are REQUIRED, not merely for normal mapping: UProceduralMeshComponent builds a
        // per-vertex tangent basis, and a zero-length TangentX leaves it degenerate. Any unit vector
        // perpendicular to the normal serves for flat shading, so the triangle's own edge,
        // Gram-Schmidt'd against it, is enough — and is O(n), unlike CalculateTangentsForMesh, which
        // spatially welds vertices and dominated load time on a triangle soup.
        FVector tx = (e1 - n * FVector::DotProduct(e1, n)).GetSafeNormal();
        if (tx.IsNearlyZero()) tx = n.RotateAngleAxis(90.0f, FVector::UpVector);
        if (tx.IsNearlyZero()) tx = FVector::ForwardVector;

        normals[i0] = normals[i1] = normals[i2] = n;
        tangents_out[i0] = tangents_out[i1] = tangents_out[i2] = FProcMeshTangent(tx, false);
    }

    // Optional smooth normals: average across coincident vertices.
    //
    // ⚠ This is what CalculateTangentsForMesh did implicitly by welding — and that build was
    // reported as correctly shaded — but it does it in O(n) with a hash instead of the spatial weld
    // that dominated load time. Positions are quantised to 1e-3 cm, which is far below any real
    // feature and far above float noise from the mirror-and-scale.
    if (mesh_smooth_normals_) {
        TMap<FIntVector, FVector> accumulated;
        accumulated.Reserve(vertex_count);
        auto key = [](const FVector& p) {
            return FIntVector(FMath::RoundToInt(p.X * 1000.0), FMath::RoundToInt(p.Y * 1000.0),
                              FMath::RoundToInt(p.Z * 1000.0));
        };
        for (int32 i = 0; i < vertex_count; ++i)
            accumulated.FindOrAdd(key(positions[i]), FVector::ZeroVector) += normals[i];

        for (int32 i = 0; i < vertex_count; ++i) {
            const FVector avg = accumulated[key(positions[i])].GetSafeNormal();
            if (avg.IsNearlyZero()) continue;  // opposing faces cancelled: keep the flat normal
            normals[i] = avg;
            FVector tx = (tangents_out[i].TangentX -
                          avg * FVector::DotProduct(tangents_out[i].TangentX, avg)).GetSafeNormal();
            if (!tx.IsNearlyZero()) tangents_out[i] = FProcMeshTangent(tx, false);
        }
    }

    // UVs stay empty: nothing here samples a texture, and unlike tangents they are not part of the
    // shading basis.
    const TArray<FVector2D> uvs;

    UProceduralMeshComponent* c = NewObject<UProceduralMeshComponent>(this, name);
    c->SetupAttachment(link_component);
    c->SetMobility(EComponentMobility::Movable);
    c->bUseComplexAsSimpleCollision = true;

    // ⚠ SHADOW FLAGS MUST BE SET BEFORE RegisterComponent.
    //
    // bCastInsetShadow and bCastShadowAsTwoSided are plain UPROPERTY fields, not setters — nothing
    // marks the render state dirty when they are written. The scene proxy is built at registration,
    // so a value assigned afterwards never reaches the renderer at all.
    //
    // An earlier revision set them after registration, which made the inset-shadow arms of the
    // bisection rig silently identical to the control: the rig reported "no difference" because
    // there was no difference, not because inset shadows do not help. **A flag applied too late
    // fails exactly like a flag that is wrong, and an experiment cannot tell the two apart.**
    //
    // Why inset shadows: a URDF robot is SMALL — ExoMy is about 30 cm — while a cascaded shadow map
    // spans hundreds of metres, so one shadow texel covers a good fraction of the whole robot and it
    // shadows itself everywhere. That is also why the darkening is distance-dependent: cascades only
    // reach so far, so the robot is unshadowed far away and self-shadowed on approach. An inset
    // shadow gives the object its own tight shadow map and, unlike switching shadow casting off,
    // keeps the shadow the robot casts on the ground.
    c->SetCastShadow(mesh_cast_shadow_);
    c->bCastInsetShadow = mesh_inset_shadow_;
    c->bCastShadowAsTwoSided = mesh_two_sided_shadow_;
    // ⚠ Screen-space contact shadows — the only remaining mechanism that is inherently
    // distance-dependent. ContactShadowLength is in SCREEN space unless the light sets
    // ContactShadowLengthInWS, so a nearer object gets a longer ray march and self-shadows harder
    // the closer the camera comes. Turning it off keeps the robot's shadow on the ground and only
    // stops the screen-space pass shadowing the robot against itself.
    c->bCastContactShadow = mesh_contact_shadow_;

    const TArray<FColor> colors;

    // ⚠ bCreateCollision COOKS physics geometry at runtime, per section. On a 23-link robot with
    // detailed STLs that is the dominant cost of a spawn, and it is paid on every PIE start.
    //
    // It cannot simply be dropped: this collision is what LiDAR, echo and distance sensors trace,
    // so turning it off makes the robot invisible to its own sensors and to every other robot —
    // R2, in its most complete form. So it stays on by default and is a deliberate trade, not a
    // silent optimisation.
    const double t_section = FPlatformTime::Seconds();
    c->CreateMeshSection(0, positions, indices, normals, uvs, colors, tangents_out,
                         /*bCreateCollision=*/collidable);
    mesh_section_seconds_ += FPlatformTime::Seconds() - t_section;
    mesh_triangle_total_ += static_cast<int64>(vertex_count / 3);

    // ⚠ A procedural mesh has NO material of its own. The primitive path never needed this, because
    // the engine BasicShapes carry one — which is why an untinted mesh robot rendered black rather
    // than grey. Assigning one is not decoration; without it there is nothing to shade.
    // ⚠ Resolved lazily and with a fallback, not by ConstructorHelpers alone.
    //
    // The previous version looked the base material up in the constructor and, if the lookup failed,
    // silently assigned NO material — which renders black. That is indistinguishable from every
    // other cause of a black robot, and it was never logged, so it survived two rounds of chasing
    // normals and tangents. A missing material must now announce itself.
    if (mesh_material_ == nullptr) {
        // ⚠ Prefer the INSTANCE over the base material.
        //
        // BasicShapeMaterial is the base; BasicShapeMaterial_Inst is what the engine's own
        // BasicShapes actually use, and it is the instance that supplies the sane parameter values.
        // A MID created from the bare base inherits the base's defaults, which need not be
        // anything visible — a black default there renders a correctly-assigned, correctly-tinted
        // material as black, which is precisely the symptom that survived four builds.
        if (!mesh_base_material_)
            mesh_material_ = LoadObject<UMaterialInterface>(
                nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial_Inst.BasicShapeMaterial_Inst"));
        if (mesh_material_ == nullptr)
            mesh_material_ = LoadObject<UMaterialInterface>(
                nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
        if (mesh_material_ == nullptr) {
            // Not black: the engine default is a visible grey, so a failure here degrades to
            // "untinted" rather than to "invisible".
            mesh_material_ = UMaterial::GetDefaultMaterial(MD_Surface);
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("BasicShapeMaterial could not be loaded; falling back to the engine default "
                        "material. Link colours from <material> will NOT be applied."));
        }
        UE_LOG(LogUrdfBot, Log, TEXT("mesh base material: %s"),
               *GetNameSafe(mesh_material_));
    }

    // ⚠ Always log which base material is in use, not only when the lazy path ran. The previous
    // revision logged it inside the "if it was null" block, so a successful ConstructorHelpers
    // lookup produced NO line at all — and the absence of the line was itself read as evidence
    // twice. A diagnostic that only fires on the failure path cannot confirm the success path.
    if (!logged_material_once_) {
        logged_material_once_ = true;
        UE_LOG(LogUrdfBot, Log, TEXT("mesh base material: %s"), *GetNameSafe(mesh_material_));
    }

    if (mesh_material_ != nullptr) {
        UMaterialInstanceDynamic* mid = UMaterialInstanceDynamic::Create(mesh_material_, this);
        if (mid) {
            const FLinearColor tint(static_cast<float>(material.r), static_cast<float>(material.g),
                                    static_cast<float>(material.b), static_cast<float>(material.a));
            // The parameter name differs between engine base materials, and a MID silently ignores
            // one it does not have. Setting both costs nothing and means a base-material change
            // does not turn the robot grey again.
            mid->SetVectorParameterValue(TEXT("Color"), tint);
            mid->SetVectorParameterValue(TEXT("BaseColor"), tint);

            // ⚠ Forced matte, and this is why the rover went black only when the camera got CLOSE.
            //
            // A smooth surface takes most of its colour from reflections. Blocks reports
            // "REFLECTION CAPTURES NEED TO BE REBUILT", so up close the unbuilt local capture is
            // sampled and contributes nothing, while at a distance the sky ambient still dominates
            // — giving a robot that is correctly coloured far away and black near to. Pinning
            // roughness makes a URDF robot's appearance depend on its <material> rather than on
            // whether the operator has built lighting in their level.
            mid->SetScalarParameterValue(TEXT("Roughness"), 0.85f);

            // ⚠ AFTER RegisterComponent, not before. An override material set on an unregistered
            // component is not reliably carried into the scene proxy built at registration.
            c->RegisterComponent();
            c->SetMaterial(0, mid);
            c->MarkRenderStateDirty();

            // Ask the MID what it actually took. GetVectorParameterValue returns false when the
            // parameter does not exist on the material, which is the one thing that distinguishes
            // "the tint was ignored" from "the tint was applied and something else is dark".
            if (!logged_param_once_) {
                logged_param_once_ = true;
                FLinearColor read_back(ForceInit);
                const bool has_color =
                    mid->GetVectorParameterValue(FMaterialParameterInfo(TEXT("Color")), read_back);
                UE_LOG(LogUrdfBot, Log,
                       TEXT("material param check: Color exists=%d, reads back (%.3f, %.3f, %.3f); "
                            "component material is now '%s'"),
                       has_color ? 1 : 0, read_back.R, read_back.G, read_back.B,
                       *GetNameSafe(c->GetMaterial(0)));
                if (!has_color) {
                    UE_LOG(LogUrdfBot, Warning,
                           TEXT("The base material has no 'Color' parameter, so link colours cannot "
                                "be applied through it and every link renders at the material's own "
                                "default."));
                }
            }

            // Logged so a robot that looks wrongly dark can be checked against what its URDF
            // actually says. ExoMy's materials are ~0.298 grey, so a dark rover in shadow is the
            // file being faithful rather than the renderer failing.
            // At Log, not Verbose. The previous revision used Verbose, which is off by default, so
            // the one line that would have shown the colours never appeared — the instrumentation
            // was written and then not readable, which is the same failure as not writing it.
            UE_LOG(LogUrdfBot, Log,
                   TEXT("visual '%s': rgba(%.3f, %.3f, %.3f, %.3f)%s"), *name.ToString(),
                   material.r, material.g, material.b, material.a,
                   material.present ? TEXT("") : TEXT(" [no <material> in URDF - default grey]"));
        }
    }

    // Same contract as the primitive path: Unreal physics OFF (Box3D owns this link's motion),
    // collision ON, because that is what LiDAR, echo and distance sensors trace. This is the half
    // of R2 that was previously missing entirely for a mesh robot — 17 of ExoMy's 23 links were
    // neither drawn nor traceable.
    c->SetSimulatePhysics(false);
    c->SetCollisionEnabled(collidable ? ECollisionEnabled::QueryOnly
                                      : ECollisionEnabled::NoCollision);
    if (collidable) c->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);

    // Scale is baked into the vertices above, so only the <origin> transform is carried here.
    c->SetRelativeTransform(UrdfTransform::toFTransform(origin, world_to_meters));
    if (!c->IsRegistered()) c->RegisterComponent();
    return true;
}

USceneComponent* AUrdfBotPawn::createLinkComponent(const urdf::Link& link, int index)
{
    const FName name(*FString::Printf(TEXT("link_%d_%s"), index, UTF8_TO_TCHAR(link.name.c_str())));

    // The link itself is always a bare scene component sitting exactly at the link frame. Geometry
    // hangs off it as children. Keeping the two separate matters: this is the transform the pose
    // writeback drives and the frame a per-link sensor mounts to, and it must not inherit a
    // visual's <origin> offset or scale.
    USceneComponent* c = NewObject<USceneComponent>(this, name);
    c->SetupAttachment(RootComponent);
    c->SetMobility(EComponentMobility::Movable);
    c->RegisterComponent();

    // Prefer <visual>; fall back to <collision> when there is nothing drawable in it.
    //
    // ⚠ The fallback is not a cosmetic stand-in, and that distinction is the whole reason it is
    // allowed here. There is no runtime STL/DAE importer, so a <mesh> visual cannot be drawn — and
    // on a real robot that is most of them: **all 23 of ExoMy's visuals are meshes**, so without
    // this the robot is entirely invisible and invisible to its own LiDAR. Drawing the <collision>
    // primitive instead shows *exactly what Box3D simulates*. That is truthful rather than
    // decorative, and it makes the rendered surface agree with the solver's — which narrows R2
    // rather than faking it. What is still refused is inventing geometry the robot does not have:
    // a link with neither drawable visual nor collision gets nothing.
    int drawn = 0;
    for (size_t i = 0; i < link.visuals.size(); ++i) {
        const urdf::Visual& v = link.visuals[i];
        const FName vis_name(*FString::Printf(TEXT("%s_vis%d"), *name.ToString(),
                                              static_cast<int>(i)));
        if (v.geometry.type == urdf::GeometryType::Mesh) {
            // Pre-imported asset first, runtime STL as the fallback. The asset path is the one a
            // UStaticMeshComponent takes, which renders correctly under Virtual Shadow Maps where
            // the procedural path does not.
            if (attachStaticMeshAsset(c, v.geometry, v.origin, vis_name, visual_collision_)) {
                ++drawn;
                continue;
            }
            if (attachMeshGeometry(c, v.geometry, v.origin, v.material, vis_name,
                                   visual_collision_)) {
                ++drawn;
                ++mesh_from_stl_;
            }
            continue;
        }
        attachGeometry(c, v.geometry, v.origin, vis_name, /*collidable=*/true);
        ++drawn;
    }

    if (drawn == 0) {
        for (size_t i = 0; i < link.collisions.size(); ++i) {
            const urdf::Collision& col = link.collisions[i];
            if (col.geometry.type == urdf::GeometryType::Mesh) continue;
            attachGeometry(c, col.geometry, col.origin,
                           FName(*FString::Printf(TEXT("%s_col%d"), *name.ToString(),
                                                  static_cast<int>(i))),
                           /*collidable=*/true);
            ++drawn;
        }
    }

    return c;
}

void AUrdfBotPawn::buildFromModel(const urdf::Robot& model, const std::string& urdf_dir,
                                  const std::vector<std::string>& mesh_search_paths)
{
    // Captured rather than discarded: <mesh filename=...> resolves against these, and until a
    // runtime loader existed there was nothing to resolve for.
    urdf_dir_ = urdf_dir;
    mesh_search_paths_ = mesh_search_paths;
    mesh_cache_.clear();
    mesh_load_seconds_ = mesh_section_seconds_ = 0.0;
    mesh_triangle_total_ = mesh_triangles_before_ = mesh_triangles_after_ = 0;
    mesh_from_asset_ = mesh_from_stl_ = 0;
    const double build_start = FPlatformTime::Seconds();

    link_components_.clear();
    links_by_index_.Empty();
    links_by_index_.Reserve(static_cast<int32>(model.links.size()));

    for (size_t i = 0; i < model.links.size(); ++i) {
        USceneComponent* c = createLinkComponent(model.links[i], static_cast<int>(i));
        links_by_index_.Add(c);
        link_components_[model.links[i].name] = c;
    }

    const double build_seconds = FPlatformTime::Seconds() - build_start;

    UAirBlueprintLib::LogMessageString(
        "UrdfBot: built ", Utils::stringf("%d link components for '%s' in %.2f s",
                                          static_cast<int>(model.links.size()), model.name.c_str(),
                                          build_seconds),
        LogDebugLevel::Informational);

    // Broken down, because "loading is slow" is not actionable and "collision cooking took 4.1 s of
    // 4.4 s" is. Runtime tri-mesh collision cooking is normally the dominant term on a mesh robot.
    UE_LOG(LogUrdfBot, Log,
           TEXT("mesh build for '%s': %.2f s total | STL parse %.2f s (%d unique files) | "
                "CreateMeshSection incl. collision cooking %.2f s | %lld triangles"),
           UTF8_TO_TCHAR(model.name.c_str()), build_seconds, mesh_load_seconds_,
           static_cast<int32>(mesh_cache_.size()), mesh_section_seconds_,
           static_cast<long long>(mesh_triangle_total_));
    if (mesh_from_asset_ > 0 || !mesh_asset_dir_.empty()) {
        UE_LOG(LogUrdfBot, Log,
               TEXT("mesh source: %d link(s) from UStaticMesh assets under '%s', %d from runtime "
                    "STL. Asset links render through the same path as any other Unreal mesh."),
               mesh_from_asset_, UTF8_TO_TCHAR(mesh_asset_dir_.c_str()), mesh_from_stl_);
    }
    if (mesh_triangles_before_ > 0) {
        UE_LOG(LogUrdfBot, Log,
               TEXT("decimation: %lld -> %lld triangles (%.1f %% kept) at a %.1f mm grid"),
               static_cast<long long>(mesh_triangles_before_),
               static_cast<long long>(mesh_triangles_after_),
               100.0 * double(mesh_triangles_after_) / double(mesh_triangles_before_),
               mesh_decimate_grid_ * 1000.0);
    }
    if (mesh_section_seconds_ > 1.0) {
        UE_LOG(LogUrdfBot, Warning,
               TEXT("Runtime collision cooking dominates spawn time. Set UrdfVisualCollision=false "
                    "to skip it - but the robot then becomes INVISIBLE to LiDAR, echo and distance "
                    "sensors, and to every other robot's sensors."));
    }
}

USceneComponent* AUrdfBotPawn::getLinkComponent(const std::string& link_name) const
{
    const auto it = link_components_.find(link_name);
    return it == link_components_.end() ? nullptr : it->second;
}

AActor* AUrdfBotPawn::getOrCreateLinkMount(const std::string& link_name)
{
    const auto existing = link_mount_by_name_.find(link_name);
    if (existing != link_mount_by_name_.end()) return existing->second;

    USceneComponent* link = getLinkComponent(link_name);
    if (!link) return nullptr;

    FActorSpawnParameters params;
    params.Owner = this;
    AUrdfLinkMount* mount = GetWorld()->SpawnActor<AUrdfLinkMount>(
        AUrdfLinkMount::StaticClass(), link->GetComponentTransform(), params);
    if (!mount) return nullptr;

    // KeepWorld would leave the mount where it was spawned and then follow; SnapToTarget puts it
    // exactly on the link, which is what "mounted on link X" has to mean before the sensor's own
    // relative pose from settings is applied on top.
    mount->AttachToComponent(link, FAttachmentTransformRules::SnapToTargetIncludingScale);

    link_mounts_.Add(mount);
    link_mount_by_name_[link_name] = mount;
    return mount;
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> AUrdfBotPawn::getCameras() const
{
    // Cameras declared in settings are created by PawnSimApi against the link they name; the pawn
    // itself carries none by default, unlike the fixed camera rigs on the drone and car pawns.
    // A URDF robot's sensible camera mounts are its own links, which only the URDF knows.
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    return cameras;
}

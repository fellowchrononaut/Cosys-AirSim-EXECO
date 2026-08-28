#include "UrdfBotPawn.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "UrdfWorldGeometry.h"   // LogUrdfBot

#include "UrdfLinkMount.h"
#include "UrdfTransform.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Materials/Material.h"
#include "SceneTypes.h"

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


/// Apply this robot's segmentation stencil to a freshly created mesh component.
///
/// ⚠ Called for EVERY drawing path — procedural mesh, imported static-mesh asset, and the
/// box/cylinder/sphere primitives — because a robot that segments correctly only when drawn one
/// particular way is worse than one that never segments at all: the gap moves with the settings.
static void applySegmentationId(UPrimitiveComponent* c, int id)
{
    if (c == nullptr || id < 0) return;
    c->SetCustomDepthStencilValue(id);
    c->SetRenderCustomDepth(true);
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
    applySegmentationId(c, segmentation_id_);
}

void AUrdfBotPawn::ensureMeshMaterial()
{
    if (mesh_material_ != nullptr) return;

    // ⚠ HOISTED OUT OF THE PROCEDURAL PATH. This resolution used to live inline in
    // attachMeshGeometry, so any path that ran BEFORE it saw mesh_material_ == nullptr. The runtime
    // static-mesh path runs first, which left its material slot empty and its SetMaterial call
    // skipped — a mesh that builds correctly, reports sane bounds and one section, and draws
    // nothing. Resolving it up front is what makes the two paths share a material at all.
    if (!mesh_base_material_)
        mesh_material_ = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial_Inst.BasicShapeMaterial_Inst"));
    if (mesh_material_ == nullptr)
        mesh_material_ = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (mesh_material_ == nullptr) {
        mesh_material_ = UMaterial::GetDefaultMaterial(MD_Surface);
        UE_LOG(LogUrdfBot, Warning,
               TEXT("BasicShapeMaterial could not be loaded; falling back to the engine default. "
                    "Link colours from <material> will NOT be applied."));
    }
    UE_LOG(LogUrdfBot, Log, TEXT("mesh base material: %s"), *GetNameSafe(mesh_material_));
}

void AUrdfBotPawn::logMaterialDiagnosticsOnce(UMaterialInterface* assigned,
                                              UPrimitiveComponent* component)
{
    UMaterialInstanceDynamic* mid = Cast<UMaterialInstanceDynamic>(assigned);
    if (mid == nullptr) return;

    // ⚠ ENUMERATE THE PARAMETERS. A MID silently ignores a parameter the material does not expose,
    // so SetVectorParameterValue("Color", ...) on a material whose parameter is named something
    // else is a no-op that looks exactly like a lighting or normals bug.
    if (!logged_material_params_) {
        logged_material_params_ = true;
        TArray<FMaterialParameterInfo> infos;
        TArray<FGuid> guids;

        mid->GetAllVectorParameterInfo(infos, guids);
        FString names;
        for (const FMaterialParameterInfo& i : infos)
            names += (names.IsEmpty() ? TEXT("") : TEXT(", ")) + i.Name.ToString();
        UE_LOG(LogUrdfBot, Log, TEXT("material VECTOR params: [%s]"), *names);

        infos.Reset(); guids.Reset();
        mid->GetAllScalarParameterInfo(infos, guids);
        names.Empty();
        for (const FMaterialParameterInfo& i : infos)
            names += (names.IsEmpty() ? TEXT("") : TEXT(", ")) + i.Name.ToString();
        UE_LOG(LogUrdfBot, Log, TEXT("material SCALAR params: [%s]"), *names);
    }

    // ⚠ Read the value BACK. Setting a parameter always "succeeds"; only a read-back shows whether
    // the material actually holds it.
    //
    // ⚠ And this read-back once LIED: GetScalarParameterValue reported "Metallic exists=1 (0.00)"
    // while the enumeration above proves Metallic is not a parameter of this material at all.
    // BasicShapeMaterial_Inst exposes exactly two — Color and Roughness — so Metallic and Specular
    // were silent no-ops, and Roughness is the only shading lever there is.
    if (!logged_material_readback_) {
        logged_material_readback_ = true;
        FLinearColor got_colour(ForceInit);
        const bool has_colour =
            mid->GetVectorParameterValue(FMaterialParameterInfo(TEXT("Color")), got_colour);
        FLinearColor got_base(ForceInit);
        const bool has_base =
            mid->GetVectorParameterValue(FMaterialParameterInfo(TEXT("BaseColor")), got_base);
        float got_metallic = -1.0f;
        const bool has_metallic =
            mid->GetScalarParameterValue(FMaterialParameterInfo(TEXT("Metallic")), got_metallic);

        UE_LOG(LogUrdfBot, Log,
               TEXT("Color exists=%d (%.3f %.3f %.3f)  BaseColor exists=%d (%.3f %.3f %.3f)  "
                    "Metallic exists=%d (%.2f)"),
               has_colour ? 1 : 0, got_colour.R, got_colour.G, got_colour.B,
               has_base ? 1 : 0, got_base.R, got_base.G, got_base.B,
               has_metallic ? 1 : 0, got_metallic);

        if (!has_colour)
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("The base material has no 'Color' parameter, so link colours cannot be "
                        "applied through it and every link renders at the material's own default."));
    }

    if (!logged_param_once_ && component != nullptr) {
        logged_param_once_ = true;
        UE_LOG(LogUrdfBot, Log, TEXT("component material [0] is now '%s'"),
               *GetNameSafe(component->GetMaterial(0)));
    }
}

UMaterialInterface* AUrdfBotPawn::resolveSectionMaterial(const urdf::Material& mesh_material,
                                                        const urdf::Material& urdf_material)
{
    ensureMeshMaterial();
    if (mesh_material_ == nullptr) return nullptr;
    if (!mesh_tint_) return mesh_material_;   // diagnostic escape hatch: no MIDs at all

    // ⚠ PRECEDENCE LIVES HERE AND NOWHERE ELSE. The procedural and runtime-static paths must not
    // each decide this; they diverged once before over the world_to_meters factor and the symptom
    // was a robot that looked fine on one setting and invisible on the other.
    //
    // The mesh's own material wins by default because it is per-SUBMESH while a URDF <material> is
    // one flat colour for a whole link — so deferring to the URDF cannot express more than one of
    // the colours a .dae declares, and on our two mesh robots what it expresses is wrong anyway:
    // the Go2's URDF says rgba "1 1 1 1" seventeen times for a robot that is mostly matte black,
    // and the Scout's declares no <material> at all.
    //
    // Either way a section the mesh left unpainted falls back to the URDF colour, so this decides
    // only the conflict. UrdfMeshMaterialOverride inverts it for hand-authoring from the URDF.
    const urdf::Material* chosen = nullptr;
    if (mesh_material_override_)
        chosen = urdf_material.present ? &urdf_material
                                       : (mesh_material.present ? &mesh_material : nullptr);
    else
        chosen = mesh_material.present ? &mesh_material
                                       : (urdf_material.present ? &urdf_material : nullptr);

    // Nothing declared a colour: keep the shared parent rather than inventing a grey MID.
    if (chosen == nullptr) return mesh_material_;

    // ⚠ COLOUR SPACE, and why the default passes the numbers through untouched. Collada's
    // profile_COMMON <color> is specified LINEAR, Blender (which exported our .dae files) stores
    // and writes linear, and FLinearColor is linear — so no conversion is the faithful reading.
    //
    // The consequence is worth knowing before it gets reported as a bug: a linear 0.672 displays at
    // roughly sRGB 214/255, so the Go2's grey shell — 0.672 0.692 0.774 across most of its surface,
    // which that robot's own MuJoCo model also names "gray" — reads as off-white. Correct, not
    // washed out. UrdfMeshSrgbColors exists so the alternative can be SEEN in one run rather than
    // argued from a spec.
    auto channel = [this](double v) {
        if (!mesh_srgb_colors_) return static_cast<float>(v);
        const float c = static_cast<float>(FMath::Clamp(v, 0.0, 1.0));
        return c <= 0.04045f ? c / 12.92f : FMath::Pow((c + 0.055f) / 1.055f, 2.4f);
    };
    const FLinearColor tint(channel(chosen->r), channel(chosen->g), channel(chosen->b),
                            static_cast<float>(chosen->a));

    // ⚠ Key the cache on the FINAL tint, not the declared one. Keying on the source colour would
    // make the cache silently wrong the moment a second setting changed what a colour resolves to.
    auto to8 = [](float v) {
        return static_cast<uint32>(FMath::Clamp(FMath::RoundToInt(v * 255.0f), 0, 255));
    };
    const uint32 key = (to8(tint.R) << 24) | (to8(tint.G) << 16) | (to8(tint.B) << 8) | to8(tint.A);
    if (UMaterialInstanceDynamic** found = mid_cache_.Find(key)) return *found;

    UMaterialInstanceDynamic* mid = UMaterialInstanceDynamic::Create(mesh_material_, this);
    if (mid == nullptr) return mesh_material_;

    // This parent material's vector parameter is named Color; enumerating it is what settled that,
    // and the enumeration still runs once in attachMeshGeometry.
    mid->SetVectorParameterValue(TEXT("Color"), tint);
    // ⚠ Roughness is the ONLY other parameter BasicShapeMaterial_Inst exposes — no Metallic, no
    // Specular; setting those is a silent no-op. 1.0 (the default) is fully matte, the faithful
    // reading of a diffuse colour: neither a URDF <material> nor a Collada <lambert> describes
    // gloss. Lowering it adds specular definition, at the risk recorded on the setting.
    mid->SetScalarParameterValue(TEXT("Roughness"),
                                 static_cast<float>(FMath::Clamp(mesh_roughness_, 0.0, 1.0)));

    mid_cache_.Add(key, mid);
    return mid;
}

UStaticMesh* AUrdfBotPawn::buildStaticMeshFromData(const urdf::MeshData& mesh,
                                                  const FString& key, double sx, double sy,
                                                  double sz)
{
    // ⚠ WHY A REAL UStaticMesh AND NOT A PROCEDURAL MESH.
    //
    // Every VSM mechanism was tried against the procedural path and measured: NormalBias does
    // nothing, ResolutionLodBias does nothing, two-sided shadow casting does nothing, inset shadows
    // are disabled by the engine under VSM, and ShadowCacheInvalidationBehavior::Always does
    // nothing. Only disabling VSM or disabling shadow casting lifts the darkness, and both of those
    // are losses rather than fixes. That pattern says the geometry in the shadow map is wrong in a
    // way none of the sampling knobs can reach.
    //
    // A UStaticMesh goes through the engine's own build: bounds, tangents, LODs, distance fields
    // and shadow-cache behaviour are all constructed the way the renderer expects, instead of being
    // reconstructed by us. BuildFromMeshDescriptions is ENGINE_API and NOT editor-only, so this
    // works at runtime and in packaged builds alike.
    //
    // ⚠ Cached per (file, scale). A 23-link robot reuses the same wheel mesh four times, and
    // building it four times would be the dominant cost of a spawn.
    ensureMeshMaterial();
    if (UStaticMesh** found = static_mesh_cache_.Find(key)) return *found;

    if (mesh.vertices.size() < 3) return nullptr;

    FMeshDescription desc;
    FStaticMeshAttributes attrs(desc);
    attrs.Register();

    // ⚠ ONE POLYGON GROUP PER MESH SECTION, and the slot names below must match these exactly.
    // Slots are named by INDEX, not by the material's own name: Collada material names are
    // arbitrary UTF-8 — the Go2's are Chinese — and an FName round-trip through them is a needless
    // way to lose the mapping the renderer depends on.
    const std::vector<urdf::MeshSection> sections = mesh.drawSections();
    TArray<FPolygonGroupID> groups;
    groups.Reserve(static_cast<int32>(sections.size()));
    for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si) {
        const FPolygonGroupID group = desc.CreatePolygonGroup();
        attrs.GetPolygonGroupMaterialSlotNames()[group] =
            FName(*FString::Printf(TEXT("Section%d"), si));
        groups.Add(group);
    }

    // Triangle index -> the group it belongs to. Sections partition the soup in order (asserted by
    // the headless suite), so this is a walk, not a search.
    TArray<int32> triangle_group;
    triangle_group.Init(0, static_cast<int32>(mesh.vertices.size() / 3));
    for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si)
        for (size_t t = sections[si].first_triangle;
             t < sections[si].first_triangle + sections[si].triangle_count &&
             t < mesh.vertices.size() / 3;
             ++t)
            triangle_group[static_cast<int32>(t)] = si;

    TVertexAttributesRef<FVector3f> positions = attrs.GetVertexPositions();
    TVertexInstanceAttributesRef<FVector3f> normals = attrs.GetVertexInstanceNormals();
    TVertexInstanceAttributesRef<FVector2f> uvs = attrs.GetVertexInstanceUVs();

    const int32 tri_count = static_cast<int32>(mesh.vertices.size() / 3);
    desc.ReserveNewVertices(tri_count * 3);
    desc.ReserveNewVertexInstances(tri_count * 3);
    desc.ReserveNewPolygons(tri_count);

    for (int32 t = 0; t < tri_count; ++t) {
        // Use the same baked Y-axis basis conversion and winding rule as the procedural path. For a
        // positive URDF mesh scale the source indices are retained; an additional odd negative scale
        // reverses them. The normal below follows Unreal's Cross(e2,e1) convention.
        const bool reverse = ((sx * sy * sz) < 0.0) != mesh_flip_winding_;
        const int32 src[3] = { t * 3 + 0,
                               reverse ? t * 3 + 2 : t * 3 + 1,
                               reverse ? t * 3 + 1 : t * 3 + 2 };

        FVertexInstanceID inst[3];
        for (int32 k = 0; k < 3; ++k) {
            const urdf::Vec3& v = mesh.vertices[src[k]];
            const FVertexID vid = desc.CreateVertex();
            positions[vid] = FVector3f(static_cast<float>(v.x * sx),
                                       static_cast<float>(-v.y * sy),
                                       static_cast<float>(v.z * sz));
            inst[k] = desc.CreateVertexInstance(vid);
            // ⚠ NOT all-zero. Every vertex sharing one UV means any texture in the material
            // samples a single texel, and it leaves the tangent basis degenerate — the engine
            // derives tangents from UV gradients, so a zero gradient yields no usable basis. The
            // engine's own BasicShapes have proper UVs, which is one concrete difference between
            // the links that shade correctly and the ones that do not. A planar projection is
            // arbitrary but well-formed, which is all a flat-shaded robot needs.
            uvs[inst[k]] = FVector2f(static_cast<float>(v.x), static_cast<float>(v.z));
        }

        // Flat face normal; the engine recomputes tangents during the build.
        const FVector3f e1 = positions[desc.GetVertexInstanceVertex(inst[1])] -
                             positions[desc.GetVertexInstanceVertex(inst[0])];
        const FVector3f e2 = positions[desc.GetVertexInstanceVertex(inst[2])] -
                             positions[desc.GetVertexInstanceVertex(inst[0])];
        const FVector3f n = FVector3f::CrossProduct(e2, e1).GetSafeNormal();
        for (int32 k = 0; k < 3; ++k) normals[inst[k]] = n;

        desc.CreatePolygon(groups[triangle_group[t]],
                           TArray<FVertexInstanceID>({ inst[0], inst[1], inst[2] }));
    }

    UStaticMesh* built = NewObject<UStaticMesh>(this);

    // ⚠ THE SLOT NAME MUST MATCH THE POLYGON GROUP'S. The build maps polygon groups onto material
    // slots BY NAME; an unnamed FStaticMaterial against a group named "Default" leaves the section
    // with no valid material index, and the mesh builds successfully and renders NOTHING. That is
    // exactly how this first appeared — geometry present, bounds correct, invisible.
    for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si) {
        FStaticMaterial slot;
        slot.MaterialSlotName = FName(*FString::Printf(TEXT("Section%d"), si));
        slot.ImportedMaterialSlotName = slot.MaterialSlotName;
        // ⚠ The BUILT MESH is cached per (file, scale) and shared between links, so its slots carry
        // the shared parent only. The per-section colour is applied on the COMPONENT, where two
        // links referencing one .dae can still be tinted differently.
        slot.MaterialInterface = mesh_material_;
        built->GetStaticMaterials().Add(slot);
    }

    UStaticMesh::FBuildMeshDescriptionsParams params;
    // ⚠ Collision is built separately from the URDF's own <collision>, so this must NOT cook
    // physics from the visual mesh — that is the R2 distinction the whole audit exists to keep.
    params.bBuildSimpleCollision = false;
    params.bFastBuild = true;

    if (!built->BuildFromMeshDescriptions({ &desc }, params)) {
        UE_LOG(LogUrdfBot, Warning, TEXT("BuildFromMeshDescriptions failed for '%s'"), *key);
        return nullptr;
    }

    static_mesh_cache_.Add(key, built);
    ++mesh_built_static_;

    // ⚠ Report the BOUNDS. "Built successfully" and "visible" are different claims, and a mesh with
    // empty bounds is culled before it is ever drawn — indistinguishable from a material problem
    // without this line.
    const FBoxSphereBounds b = built->GetBounds();
    UE_LOG(LogUrdfBot, Log,
           TEXT("built static mesh '%s': %d tris, bounds extent (%.2f %.2f %.2f) radius %.2f, "
                "sections %d"),
           *key, tri_count, b.BoxExtent.X, b.BoxExtent.Y, b.BoxExtent.Z, b.SphereRadius,
           built->GetNumSections(0));
    return built;
}

bool AUrdfBotPawn::attachBuiltStaticMesh(USceneComponent* link_component,
                                         const urdf::Geometry& g, const urdf::Origin& origin,
                                         const urdf::Material& material, const FName& name,
                                         bool collidable)
{
    // Resolved and cached exactly as the procedural path does — same directory, same search paths,
    // same per-FILE cache, so the two paths never disagree about which file a link uses.
    const std::string resolved =
        urdf::resolveMeshPath(g.mesh_filename, urdf_dir_, mesh_search_paths_);
    if (resolved.empty()) return false;

    auto it = mesh_cache_.find(resolved);
    if (it == mesh_cache_.end()) {
        try {
            it = mesh_cache_.emplace(resolved, urdf::loadMesh(resolved)).first;
        }
        catch (const std::exception&) {
            return false;   // the procedural path reports this; do not duplicate the message
        }
    }
    const urdf::MeshData& data = it->second;
    if (data.vertices.size() < 3) return false;

    // ⚠ × world_to_meters, EXACTLY as the procedural path does at its own scale computation.
    // URDF vertices are in METRES and Unreal is centimetre-based, so omitting this builds the mesh
    // 100x too small: correct triangles, correct bounds, one section, right material — and half a
    // millimetre across, which looks precisely like "invisible". That is what it was.
    const float world_to_meters = 100.0f;
    const double sx = g.mesh_scale.x * world_to_meters;
    const double sy = g.mesh_scale.y * world_to_meters;
    const double sz = g.mesh_scale.z * world_to_meters;
    const FString key = FString::Printf(TEXT("%s|%f|%f|%f"), UTF8_TO_TCHAR(resolved.c_str()),
                                        sx, sy, sz);
    UStaticMesh* built = buildStaticMeshFromData(data, key, sx, sy, sz);
    if (built == nullptr) return false;

    UStaticMeshComponent* c = NewObject<UStaticMeshComponent>(this, name);
    c->SetupAttachment(link_component);
    c->SetMobility(EComponentMobility::Movable);

    // Same pre-registration rule as everywhere else in this file: these are plain fields, not
    // setters, and the scene proxy is built at registration.
    c->SetCastShadow(mesh_cast_shadow_);
    c->bCastInsetShadow = mesh_inset_shadow_;
    c->bCastShadowAsTwoSided = mesh_two_sided_shadow_;
    c->bCastContactShadow = mesh_contact_shadow_;

    c->SetStaticMesh(built);
    c->SetRelativeTransform(UrdfTransform::toFTransform(origin, world_to_meters));
    c->SetCollisionEnabled(collidable ? ECollisionEnabled::QueryOnly
                                      : ECollisionEnabled::NoCollision);

    // One material per section, through the shared resolver — the same precedence and the same MID
    // cache the procedural path uses, so the two cannot drift apart.
    const std::vector<urdf::MeshSection> sections = data.drawSections();
    for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si)
        if (UMaterialInterface* m = resolveSectionMaterial(sections[si].material, material))
            c->SetMaterial(si, m);
    c->RegisterComponent();
    applySegmentationId(c, segmentation_id_);
    return true;
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

    // ⚠ SHADOW CACHE INVALIDATION — the last VSM-specific mechanism, and the one that fits the
    // evidence. Measured on this robot: disabling VSM lifts the blackness, disabling shadow casting
    // lifts it, but NEITHER r.Shadow.Virtual.NormalBias NOR ResolutionLodBias changes anything.
    // Bias and resolution are both ways of sampling the shadow map better; their total ineffective-
    // ness says the problem is not how we sample it but what is IN it.
    //
    // VSM caches shadow pages across frames and invalidates them from what the renderer can infer —
    // World Position Offset and transform changes. A UProceduralMeshComponent whose geometry is
    // built at runtime and whose transform is driven from physics every frame is exactly the case
    // Epic documents Always for: "a primitive that is using some method of animating that is not
    // known to the system" (SceneTypes.h). With stale pages the robot is shadowed by its own
    // out-of-date depth, which no amount of biasing a lookup can fix.
    //
    // ⚠ Before RegisterComponent, like the flags above — see the note there. This is a plain
    // UPROPERTY with no setter, so a value written after registration never reaches the proxy.
    c->ShadowCacheInvalidationBehavior = mesh_shadow_invalidate_always_
                                             ? EShadowCacheInvalidationBehavior::Always
                                             : EShadowCacheInvalidationBehavior::Auto;
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
    applySegmentationId(c, segmentation_id_);

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
            it = mesh_cache_.emplace(resolved, urdf::loadMesh(resolved)).first;
            mesh_load_seconds_ += FPlatformTime::Seconds() - t0;
        }
        catch (const std::exception& e) {
            // ⚠ Named rather than silently skipped, because "the robot is partly invisible" must
            // not be something the operator has to infer — a link with no geometry is also
            // untraceable by every sensor, so it vanishes from perception as well as from view.
            // STL, Collada and Wavefront are handled (urdf::loadMesh); USD is not.
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: cannot load mesh ",
                resolved + " (" + e.what() + ")",
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
    // ⚠ DECIMATION MUST BE SECTION-AWARE. It drops triangles, so a section's range into the vertex
    // array is invalid the moment it runs — and the failure is not a crash but a SMEAR: every
    // section after the first is painted with a neighbour's colour, which reads as a parser bug and
    // is not one. Decimate each run separately and rebuild the ranges from what survived.
    std::vector<urdf::MeshSection> sections = mesh.drawSections();
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
        // ⚠ ONE representative map across ALL sections, not one per section. Two materials meeting
        // at a seam share vertices there, and giving each run its own map would snap them to
        // different representatives and crack the model open along every colour boundary.
        std::unordered_map<uint64_t, urdf::Vec3> rep;
        rep.reserve(mesh.vertices.size());
        decimated.reserve(mesh.vertices.size());

        std::vector<urdf::MeshSection> kept;
        kept.reserve(sections.size());
        for (const urdf::MeshSection& sec : sections) {
            const size_t out_start = decimated.size() / 3;
            const size_t last = sec.first_triangle + sec.triangle_count;
            for (size_t tri = sec.first_triangle; tri < last && tri * 3 + 2 < mesh.vertices.size();
                 ++tri) {
                const size_t t = tri * 3;
                const uint64_t c0 = cell(mesh.vertices[t]);
                const uint64_t c1 = cell(mesh.vertices[t + 1]);
                const uint64_t c2 = cell(mesh.vertices[t + 2]);
                // Two corners in one cell means the triangle has collapsed to a line: drop it.
                if (c0 == c1 || c1 == c2 || c0 == c2) continue;
                decimated.push_back(rep.emplace(c0, mesh.vertices[t]).first->second);
                decimated.push_back(rep.emplace(c1, mesh.vertices[t + 1]).first->second);
                decimated.push_back(rep.emplace(c2, mesh.vertices[t + 2]).first->second);
            }
            urdf::MeshSection ns = sec;
            ns.first_triangle = out_start;
            ns.triangle_count = decimated.size() / 3 - out_start;
            // A run that decimated away entirely is dropped rather than kept empty: an empty
            // procedural section is a draw call for nothing.
            if (ns.triangle_count > 0) kept.push_back(ns);
        }
        if (decimated.size() >= 3) {
            mesh_triangles_before_ += static_cast<int64>(mesh.vertices.size() / 3);
            mesh_triangles_after_ += static_cast<int64>(decimated.size() / 3);
            verts = &decimated;
            sections = std::move(kept);
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

    // The baked Y-axis conversion changes coordinate basis, while Unreal's mesh helpers define the
    // outward face normal as Cross(e2,e1). Retain source indices for positive mesh scale; reverse
    // only for an additional odd negative scale. mesh_flip_winding_ remains a diagnostic override.
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
        const FVector n = FVector::CrossProduct(e2, e1).GetSafeNormal();

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

    // ⚠ The PROCEDURAL path needs this most — see the long note at the static-mesh site. A runtime-
    // built mesh moved every frame is precisely the primitive VSM's Auto invalidation cannot reason
    // about, and stale cached pages explain why neither NormalBias nor ResolutionLodBias moved the
    // needle while disabling VSM entirely did.
    c->ShadowCacheInvalidationBehavior = mesh_shadow_invalidate_always_
                                             ? EShadowCacheInvalidationBehavior::Always
                                             : EShadowCacheInvalidationBehavior::Auto;

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
    if (sections.size() <= 1) {
        // ⚠ The single-section case keeps the arrays it already built and copies nothing. Every STL
        // robot — ExoMy is 23 links of them — takes this branch, so the cost of materials on the
        // robots that have none is exactly zero.
        c->CreateMeshSection(0, positions, indices, normals, uvs, colors, tangents_out,
                             /*bCreateCollision=*/collidable);
    }
    else {
        // ⚠ CreateMeshSection wants SECTION-LOCAL indices, so each run is sliced out and rebased.
        // Handing it the global arrays with a global index range draws the whole mesh N times.
        TArray<FVector> sec_positions;
        TArray<int32> sec_indices;
        TArray<FVector> sec_normals;
        TArray<FProcMeshTangent> sec_tangents;
        for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si) {
            const int32 first = static_cast<int32>(sections[si].first_triangle) * 3;
            const int32 count = static_cast<int32>(sections[si].triangle_count) * 3;
            if (first < 0 || count <= 0 || first + count > vertex_count) continue;

            sec_positions.Reset(count);
            sec_indices.Reset(count);
            sec_normals.Reset(count);
            sec_tangents.Reset(count);
            for (int32 k = 0; k < count; ++k) {
                sec_positions.Add(positions[first + k]);
                sec_normals.Add(normals[first + k]);
                sec_tangents.Add(tangents_out[first + k]);
                sec_indices.Add(indices[first + k] - first);
            }
            // ⚠ bCreateCollision is PER SECTION, so a five-material mesh cooks five tri-meshes
            // instead of one. Collision cooking already dominates spawn time on a mesh-heavy robot
            // (that is why UrdfVisualCollision exists), and this multiplies it by the section count.
            c->CreateMeshSection(si, sec_positions, sec_indices, sec_normals, uvs, colors,
                                 sec_tangents, /*bCreateCollision=*/collidable);
        }
    }
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
    ensureMeshMaterial();
    if (false) {
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

    // One material per mesh section, through the shared resolver so this path and the runtime-static
    // one cannot disagree about precedence or about which colour wins.
    //
    // ⚠ Set the override BEFORE registration so the first scene proxy is built with it. SetMaterial
    // retains it in OverrideMaterials; no explicit render-state dirty call is needed here.
    for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si) {
        UMaterialInterface* m = resolveSectionMaterial(sections[si].material, material);
        if (m == nullptr) continue;
        c->SetMaterial(si, m);
        logMaterialDiagnosticsOnce(m, c);
    }

    // ⚠ Log what each section RESOLVED TO, not what the URDF said. The two now differ by design,
    // and the whole reason this robot rendered white was that nobody could see the difference:
    // ExoMy's URDF colours are real, the Go2's are seventeen copies of "1 1 1 1", and the Scout has
    // none at all while both carry real liveries inside their .dae files.
    // At Log, not Verbose — Verbose is off by default, and instrumentation that is not readable is
    // the same as instrumentation that was never written.
    {
        FString detail;
        for (int32 si = 0; si < static_cast<int32>(sections.size()); ++si) {
            const urdf::Material& mm = sections[si].material;
            const bool from_mesh = mm.present && !mesh_material_override_;
            const urdf::Material& used = (from_mesh || !material.present) ? mm : material;
            detail += FString::Printf(TEXT("%s[%d] %s rgba(%.3f %.3f %.3f) x%llu tris"),
                                      si == 0 ? TEXT("") : TEXT("; "), si,
                                      used.present ? (from_mesh ? TEXT("mesh") : TEXT("urdf"))
                                                   : TEXT("none"),
                                      used.r, used.g, used.b,
                                      static_cast<unsigned long long>(sections[si].triangle_count));
        }
        UE_LOG(LogUrdfBot, Log, TEXT("visual '%s': %d section(s)  %s"), *name.ToString(),
               static_cast<int32>(sections.size()), *detail);
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
    applySegmentationId(c, segmentation_id_);
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
            // ⚠ Then a UStaticMesh BUILT AT RUNTIME from the same mesh data, before falling back to
            // the procedural path. This is the fix for the shading: a real UStaticMeshComponent
            // gets engine-built bounds, tangents and shadow-cache behaviour, where every VSM knob
            // applied to UProceduralMeshComponent was measured to do nothing.
            if (mesh_runtime_static_ &&
                attachBuiltStaticMesh(c, v.geometry, v.origin, v.material, vis_name,
                                      visual_collision_)) {
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
        "UrdfBot: built ", common_utils::Utils::stringf("%d link components for '%s' in %.2f s",
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

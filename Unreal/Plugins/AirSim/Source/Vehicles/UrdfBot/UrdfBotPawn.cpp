#include "UrdfBotPawn.h"

#include "UrdfLinkMount.h"
#include "UrdfTransform.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/StaticMesh.h"

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
        if (v.geometry.type == urdf::GeometryType::Mesh) continue;
        attachGeometry(c, v.geometry, v.origin,
                       FName(*FString::Printf(TEXT("%s_vis%d"), *name.ToString(),
                                              static_cast<int>(i))),
                       /*collidable=*/true);
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
    unused(urdf_dir);
    unused(mesh_search_paths);

    link_components_.clear();
    links_by_index_.Empty();
    links_by_index_.Reserve(static_cast<int32>(model.links.size()));

    for (size_t i = 0; i < model.links.size(); ++i) {
        USceneComponent* c = createLinkComponent(model.links[i], static_cast<int>(i));
        links_by_index_.Add(c);
        link_components_[model.links[i].name] = c;
    }

    UAirBlueprintLib::LogMessageString(
        "UrdfBot: built ", Utils::stringf("%d link components for '%s'",
                                          static_cast<int>(model.links.size()), model.name.c_str()),
        LogDebugLevel::Informational);
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

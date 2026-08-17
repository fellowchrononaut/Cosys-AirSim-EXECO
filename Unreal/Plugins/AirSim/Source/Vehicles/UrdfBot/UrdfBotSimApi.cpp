#include "UrdfBotSimApi.h"

#include "UrdfTransform.h"
#include "UrdfWorldGeometry.h"

#include "Engine/World.h"
#include "UnrealSensors/UnrealSensorFactory.h"

#include "common/ClockFactory.hpp"
#include "urdf/UrdfCollisionSynthesis.hpp"
#include "urdf/UrdfParser.hpp"

#if WITH_BOX3D_BINDING
#include "urdf/backends/Box3DUrdfBackend.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace
{
    std::string dirOf(const std::string& p)
    {
        const size_t s = p.find_last_of('/');
        return s == std::string::npos ? std::string(".") : p.substr(0, s);
    }

    /// The API the RPC layer and any in-editor controller talk to. Thin: it forwards to the
    /// backend and converts frames, and holds no control state of its own.
    class UrdfBotApi : public msr::airlib::UrdfBotApiBase
    {
    public:
        UrdfBotApi(const urdf::Robot* model, urdf::UrdfRobotBackend* backend,
                   const std::vector<urdf::MimicClassification>* mimic,
                   const NedTransform* ned, msr::airlib::GeoPoint home)
            : model_(model), backend_(backend), mimic_(mimic), ned_(ned), home_(home)
        {
        }

        std::vector<JointInfo> getJoints() const override
        {
            std::vector<JointInfo> out;
            for (size_t i = 0; i < model_->joints.size(); ++i) {
                const urdf::Joint& j = model_->joints[i];
                JointInfo info;
                info.name = j.name;
                switch (j.type) {
                case urdf::JointType::Revolute:   info.type = "revolute"; break;
                case urdf::JointType::Continuous: info.type = "continuous"; break;
                case urdf::JointType::Prismatic:  info.type = "prismatic"; break;
                case urdf::JointType::Fixed:      info.type = "fixed"; break;
                case urdf::JointType::Floating:   info.type = "floating"; break;
                case urdf::JointType::Planar:     info.type = "planar"; break;
                }
                info.has_limit = j.limit.present;
                info.lower = j.limit.lower;
                info.upper = j.limit.upper;
                info.effort = j.limit.effort;
                info.velocity = j.limit.velocity;
                info.mimic_source = j.mimic_source_joint;
                for (const urdf::MimicClassification& c : *mimic_)
                    if (c.joint == static_cast<int>(i)) info.mimic_role = urdf::toString(c.role);
                out.push_back(std::move(info));
            }
            return out;
        }

        std::vector<std::string> getLinkNames() const override
        {
            std::vector<std::string> out;
            for (const urdf::Link& l : model_->links) out.push_back(l.name);
            return out;
        }

        void setJointPosition(const std::string& joint, double v) override
        {
            command(joint, urdf::ControlMode::Position, v);
        }
        void setJointVelocity(const std::string& joint, double v) override
        {
            command(joint, urdf::ControlMode::Velocity, v);
        }
        void setJointEffort(const std::string& joint, double v) override
        {
            command(joint, urdf::ControlMode::Effort, v);
        }
        void setJointPositionGains(const std::string& joint, double hertz, double ratio) override
        {
            backend_->setPositionGains(static_cast<size_t>(requireJoint(joint)), hertz, ratio);
        }

        urdf::JointState getJointState(const std::string& joint) const override
        {
            return backend_->getJointState(static_cast<size_t>(requireJoint(joint)));
        }

        LinkPoseInfo getLinkPose(const std::string& link) const override
        {
            const int i = backend_->findLink(link);
            if (i < 0) throw std::invalid_argument("no such link: '" + link + "'");

            const urdf::LinkPose p = backend_->getLinkPose(static_cast<size_t>(i));
            const urdf::Twist t = backend_->getLinkTwist(static_cast<size_t>(i));

            LinkPoseInfo out;
            out.name = link;
            // URDF (FLU) -> NED is a 180 deg rotation about X: y and z both flip. Every pose
            // crossing an AirSim API is NED, and a link pose is no exception.
            out.pose.position = msr::airlib::Vector3r(static_cast<float>(p.position.x),
                                                      static_cast<float>(-p.position.y),
                                                      static_cast<float>(-p.position.z));
            out.pose.orientation = msr::airlib::Quaternionr(
                static_cast<float>(p.orientation.w), static_cast<float>(p.orientation.x),
                static_cast<float>(-p.orientation.y), static_cast<float>(-p.orientation.z));
            out.twist.linear = msr::airlib::Vector3r(static_cast<float>(t.linear.x),
                                                     static_cast<float>(-t.linear.y),
                                                     static_cast<float>(-t.linear.z));
            out.twist.angular = msr::airlib::Vector3r(static_cast<float>(t.angular.x),
                                                      static_cast<float>(-t.angular.y),
                                                      static_cast<float>(-t.angular.z));
            return out;
        }

        msr::airlib::GeoPoint getHomeGeoPoint() const override { return home_; }

        // ⚠ resetImplementation(), not reset(). UpdatableObject::reset() is non-virtual - it does
        // the ordering bookkeeping and then calls the pure-virtual resetImplementation(). Marking
        // reset() override does not compile, which is the good outcome; overriding it by shadowing
        // would have quietly bypassed that bookkeeping.
        //
        // Nothing to undo here: the joint targets live in the backend, which UrdfBotSimApi resets.
        void resetImplementation() override {}

        void update(float delta = 0) override { UpdatableObject::update(delta); }

    private:
        int requireJoint(const std::string& name) const
        {
            const int i = backend_->findJoint(name);
            if (i < 0) throw std::invalid_argument("no such joint: '" + name + "'");
            return i;
        }
        void command(const std::string& joint, urdf::ControlMode mode, double v)
        {
            backend_->setJointTarget(static_cast<size_t>(requireJoint(joint)), mode, v);
        }

        const urdf::Robot* model_;
        urdf::UrdfRobotBackend* backend_;
        const std::vector<urdf::MimicClassification>* mimic_;
        const NedTransform* ned_;
        msr::airlib::GeoPoint home_;
    };
} // namespace

UrdfBotSimApi::UrdfBotSimApi(const Params& params,
                             const AirSimSettings::VehicleSetting* vehicle_setting)
    : PawnSimApi(params), vehicle_setting_(vehicle_setting)
{
}

void UrdfBotSimApi::initialize()
{
    PawnSimApi::initialize();
    loadModelAndBackend();
}

void UrdfBotSimApi::loadModelAndBackend()
{
#if !WITH_BOX3D_BINDING
    throw std::runtime_error(
        "Vehicle '" + getVehicleName() + "' is of type \"urdfbot\", but this build has no Box3D "
        "dependency (WITH_BOX3D_BINDING=0). Build box3d and re-run build.sh, or remove the "
        "vehicle from settings.json.");
#else
    const std::string urdf_path = vehicle_setting_->urdf_file;
    model_ = urdf::parseFile(urdf_path);

    // Applied BEFORE the audit and before the build, deliberately: the audit's numbers, the
    // backend's shapes and the pawn's <collision> render fallback then all describe one robot
    // rather than three different ones.
    if (vehicle_setting_->urdf_collision_from_visual) {
        const urdf::SynthesisResult synth = urdf::synthesizeCollisionFromVisual(model_);
        if (!synth.empty()) {
            UAirBlueprintLib::LogMessageString("UrdfBot collision synthesis\n", synth.report(),
                                               LogDebugLevel::Informational);
            UE_LOG(LogUrdfBot, Log, TEXT("collision synthesis for '%s':\n%s"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()),
                   UTF8_TO_TCHAR(synth.report().c_str()));
        }
    }

    // R2. Reported at load because it cannot be seen any other way: a URDF whose links are almost
    // all visual-only produces a robot that looks complete, drives on nearly nothing, and is fully
    // visible to LiDAR. On ExoMy that is 17 of 23 links and ~80% of the mass.
    audit_ = urdf::auditCollisionConsistency(model_, dirOf(urdf_path),
                                             vehicle_setting_->urdf_mesh_search_paths);
    if (vehicle_setting_->urdf_report_collision_audit) {
        UAirBlueprintLib::LogMessageString("UrdfBot R2 audit\n", audit_.report(),
                                           LogDebugLevel::Informational);
        // ...and to Blocks.log. LogMessageString draws on screen only and keys by prefix, so a
        // multi-line report scrolls away and is unrecoverable afterwards.
        UE_LOG(LogUrdfBot, Log, TEXT("R2 audit for '%s':\n%s"),
               UTF8_TO_TCHAR(getVehicleName().c_str()), UTF8_TO_TCHAR(audit_.report().c_str()));
    }

    const std::string engine = Utils::toLower(vehicle_setting_->physics_engine);
    if (!engine.empty() && engine != "box3d")
        throw std::invalid_argument("Vehicle '" + getVehicleName() + "': PhysicsEngine \"" +
                                    vehicle_setting_->physics_engine +
                                    "\" is not available. Only \"Box3D\" is implemented.");

    auto box3d = std::make_unique<urdf::Box3DUrdfBackend>();

    urdf::BackendOptions opts;
    // The MultiAgent sim clock advances by exactly getPhysicsLoopPeriod() * 1E-9 per executor
    // iteration (ASimModeWorldMultiAgent::setupClockSpeed overrides the 20 ms ASimModeBase
    // default, which applies to Car/ComputerVision/SkidVehicle only). Matching the backend's
    // internal step to it keeps whole steps consuming whole clock ticks.
    opts.fixed_timestep = 0.003;
    opts.fixed_base = vehicle_setting_->urdf_fixed_base;
    opts.mesh_base_dir = dirOf(urdf_path);
    opts.mesh_search_paths = vehicle_setting_->urdf_mesh_search_paths;
    opts.mimic.allow_servo_follower = vehicle_setting_->urdf_allow_mimic_follower;

    const float world_to_meters = getNedTransform().fromNed(1.0f);

    // Place the robot in the shared world frame. Read before anything moves the pawn, because
    // updateRendering then drives the pawn from the root link — reading it later would compound
    // the robot's own motion into its start pose.
    const FTransform spawn = getPawn()->GetActorTransform();
    opts.root_position = UrdfTransform::toUrdfVec(spawn.GetTranslation(), world_to_meters);
    opts.root_orientation = UrdfTransform::toUrdfQuat(spawn.GetRotation());

    // --- static world geometry ----------------------------------------------------------------
    // Mirrored once per level and shared by every robot: the shared_ptr's identity is what makes
    // the cook shared (Box3DStaticGeometry), so this must go through MirrorLevelShared rather than
    // each robot mirroring for itself.
    if (vehicle_setting_->urdf_mirror_world_geometry) {
        UrdfWorldGeometry::FMirrorOptions mirror_opts;
        mirror_opts.bIncludeMovable = vehicle_setting_->urdf_mirror_movable;
        mirror_opts.bIncludeOtherVehicles = vehicle_setting_->urdf_mirror_other_vehicles;
        for (const std::string& tag : vehicle_setting_->urdf_world_geometry_tags)
            mirror_opts.RequiredTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));

        UrdfWorldGeometry::FMirrorStats stats;
        bool from_cache = false;
        const UrdfWorldGeometry::FMirrorResult& mirror = UrdfWorldGeometry::MirrorLevelShared(
            getPawn()->GetWorld(), world_to_meters, mirror_opts, stats, from_cache);
        static_world_ = mirror.World;

        if (!from_cache) {
            UAirBlueprintLib::LogMessageString("UrdfBot static world geometry\n",
                                               std::string(TCHAR_TO_UTF8(*stats.Report())),
                                               LogDebugLevel::Informational);
            UE_LOG(LogUrdfBot, Log, TEXT("static world geometry mirror:\n%s"), *stats.Report());
            if (stats.ComponentsMirrored == 0) {
                UE_LOG(LogUrdfBot, Error,
                       TEXT("NOTHING mirrored from the level - the robot has no world to collide "
                            "with. See the per-component rejection reasons above."));
            }
        }
        else {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: ",
                getVehicleName() + " reuses the level mirror already made for this world - "
                "one cook serves every robot",
                LogDebugLevel::Informational);
        }

        box3d->setStaticWorld(static_world_);

        // Kinematic bodies: registered per robot, because each robot owns its own b3World and gets
        // its own handles. The mirror itself is shared, so the self-exclusion has to happen here —
        // a robot that mirrored its own pawn would weld itself to an obstacle shaped like itself.
        if (static_world_) {
            for (size_t i = 0; i < static_world_->kinematic.size(); ++i) {
                UPrimitiveComponent* src =
                    (static_cast<int32>(i) < mirror.KinematicSources.Num())
                        ? mirror.KinematicSources[static_cast<int32>(i)].Get()
                        : nullptr;
                if (src == nullptr) continue;
                if (src->GetOwner() == getPawn()) continue;  // never mirror yourself

                KinematicMirror km;
                km.component = mirror.KinematicSources[static_cast<int32>(i)];
                km.handle = box3d->addKinematicBody(static_world_->kinematic[i]);
                kinematic_mirrors_.push_back(km);
            }
            UE_LOG(LogUrdfBot, Log, TEXT("kinematic mirror: tracking %d bodies for '%s'"),
                   static_cast<int32>(kinematic_mirrors_.size()),
                   UTF8_TO_TCHAR(getVehicleName().c_str()));
        }
        UE_LOG(LogUrdfBot, Log,
               TEXT("static world: %d bodies, %d shapes, %d triangles (from_cache=%d)"),
               static_cast<int32>(static_world_ ? static_world_->bodies.size() : 0),
               static_cast<int32>(static_world_ ? static_world_->shapeCount() : 0),
               static_cast<int32>(static_world_ ? static_world_->triangleCount() : 0),
               from_cache ? 1 : 0);
    }

    const bool have_static_world = static_world_ && !static_world_->bodies.empty();
    // --- scaffolding floor: FALLBACK ONLY, now that the level is mirrored ----------------------
    //
    // ⚠ Suppressed whenever real geometry exists. A flat plane and a mirrored level are not
    // additive: the plane is infinite-ish (a 100 m slab) and would sit *through* the map, so a
    // rover driving down a ramp would stop dead in mid-air on an invisible floor. Two floors is a
    // worse failure than none, because none is obvious.
    if (have_static_world) {
        if (vehicle_setting_->urdf_ground_plane_auto ||
            !std::isnan(vehicle_setting_->urdf_ground_plane_z)) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: ",
                getVehicleName() + " asked for a scaffolding ground plane, but the level mirrored "
                "successfully - the plane is suppressed. It would sit through the map.",
                LogDebugLevel::Informational);
        }
    }
    else if (vehicle_setting_->urdf_ground_plane_auto) {
        const FVector start = getPawn()->GetActorLocation();
        FCollisionQueryParams params(FName(TEXT("UrdfGroundProbe")), /*bTraceComplex=*/true);
        params.AddIgnoredActor(getPawn());  // the robot's own link components carry collision

        FHitResult hit;
        const FVector end = start - FVector(0.0f, 0.0f, 1.0e6f);
        if (getPawn()->GetWorld()->LineTraceSingleByChannel(hit, start, end, ECC_Visibility, params)) {
            opts.add_ground_plane = true;
            // Unreal Z and URDF Z both point up, so this is a plain scaled height — absolute now,
            // not relative to the spawn point, because the solver frame is the world frame.
            opts.ground_plane_z = hit.ImpactPoint.Z / world_to_meters;
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: ground probe hit ",
                Utils::stringf("%s - scaffolding floor placed at z = %.3f m. The level did "
                               "NOT mirror, so this flat plane is all the robot can touch.",
                               TCHAR_TO_UTF8(*GetNameSafe(hit.GetActor())), opts.ground_plane_z),
                LogDebugLevel::Informational);
        }
        else {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: ",
                getVehicleName() + " has UrdfGroundPlaneAuto set but the downward probe hit "
                "nothing - no floor was created. Give it UrdfGroundPlaneZ, or spawn it over "
                "something solid.",
                LogDebugLevel::Failure);
        }
    }
    else if (!std::isnan(vehicle_setting_->urdf_ground_plane_z)) {
        opts.add_ground_plane = true;
        opts.ground_plane_z = vehicle_setting_->urdf_ground_plane_z;
    }

    box3d->buildFromUrdf(model_, opts);
    backend_ = std::move(box3d);

    // Cross-check the realised mass against the file. A mismatch means links were merged, dropped
    // or given shape-derived mass, and it is far cheaper to catch here than to explain later.
    const double urdf_mass = model_.totalMass();
    const double realised = backend_->totalMass();
    if (urdf_mass > 0 && std::fabs(realised - urdf_mass) > 1e-4 * std::max(1.0, urdf_mass)) {
        UAirBlueprintLib::LogMessageString(
            "UrdfBot WARNING: realised mass ",
            Utils::stringf("%.6f kg does not match the URDF's %.6f kg", realised, urdf_mass),
            LogDebugLevel::Failure);
    }

    // Two things the backend can only discover while building, both of which change what is
    // actually simulated and neither of which is visible in the render.
    {
        const b3urdf::Box3DRobot& robot =
            static_cast<urdf::Box3DUrdfBackend*>(backend_.get())->robot();

        for (const auto& r : robot.hullBudgetReductions()) {
            // Box3D's hull builder has a hard 255-half-edge ceiling that no vertex budget can
            // predict, so the budget is walked down until one is accepted. A coarser hull is a
            // different shape from the one the operator asked for.
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: coarser hull ",
                Utils::stringf("on link '%s' - %d vertices requested, %d used (Box3D's 255 "
                               "half-edge ceiling)", r.link.c_str(), r.requested, r.used),
                LogDebugLevel::Informational);
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("link '%s': hull budget reduced %d -> %d (255 half-edge ceiling)"),
                   UTF8_TO_TCHAR(r.link.c_str()), r.requested, r.used);
        }

        if (!robot.masslessMarkers().empty()) {
            std::string names;
            for (const std::string& n : robot.masslessMarkers())
                names += (names.empty() ? "" : ", ") + n;
            // Normal for frame markers, but they contribute no mass and are not simulated, so the
            // operator should be told which links those are rather than inferring it.
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: massless frame links ",
                names + " have neither <inertial> nor <collision> - resolved kinematically, not "
                "simulated",
                LogDebugLevel::Informational);
            UE_LOG(LogUrdfBot, Log, TEXT("massless frame links resolved kinematically: %s"),
                   UTF8_TO_TCHAR(names.c_str()));
        }
    }

    auto* pawn = static_cast<AUrdfBotPawn*>(getPawn());
    pawn->setVisualCollision(vehicle_setting_->urdf_visual_collision);
    pawn->setMeshShading(vehicle_setting_->urdf_mesh_cast_shadow,
                         vehicle_setting_->urdf_mesh_smooth_normals,
                         vehicle_setting_->urdf_mesh_flip_winding,
                         vehicle_setting_->urdf_mesh_base_material,
                         vehicle_setting_->urdf_mesh_inset_shadow,
                         vehicle_setting_->urdf_mesh_two_sided_shadow,
                         vehicle_setting_->urdf_mesh_contact_shadow,
                         vehicle_setting_->urdf_mesh_decimate_grid,
                         vehicle_setting_->urdf_mesh_asset_dir,
                         vehicle_setting_->urdf_mesh_asset_scale);
    UE_LOG(LogUrdfBot, Log,
           TEXT("mesh shading [%s]: cast_shadow=%d contact_shadow=%d inset_shadow=%d "
                "two_sided_shadow=%d smooth_normals=%d flip_winding=%d base_material=%d"),
           UTF8_TO_TCHAR(getVehicleName().c_str()),
           vehicle_setting_->urdf_mesh_cast_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_contact_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_inset_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_two_sided_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_smooth_normals ? 1 : 0,
           vehicle_setting_->urdf_mesh_flip_winding ? 1 : 0,
           vehicle_setting_->urdf_mesh_base_material ? 1 : 0);
    pawn->buildFromModel(model_, dirOf(urdf_path), vehicle_setting_->urdf_mesh_search_paths);

    snapshot_.resize(backend_->linkCount());
    render_poses_.resize(backend_->linkCount());

    // Borrowed from the backend, which outlives the api — both are members of this sim api and
    // the backend is declared first, so it is destroyed last.
    const std::vector<urdf::MimicClassification>* mimic =
        &static_cast<urdf::Box3DUrdfBackend*>(backend_.get())->robot().mimicClassifications();

    vehicle_api_ = std::make_unique<UrdfBotApi>(&model_, backend_.get(), mimic, &getNedTransform(),
                                                getGroundTruthEnvironment()->getHomeGeoPoint());

    // Per-link sensor mounting. A mount actor is created only for links a sensor actually names,
    // so a 23-link rover with two sensors gets two mounts rather than 23.
    auto sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    std::map<std::string, AActor*> mounts;
    for (const auto& s : vehicle_setting_->sensors) {
        if (!s.second) continue;
        const std::string link = s.second->settings.getString("Link", "");
        if (link.empty() || mounts.count(link)) continue;
        if (AActor* m = pawn->getOrCreateLinkMount(link)) mounts[link] = m;
    }
    sensor_factory->setLinkMounts(std::move(mounts));

    vehicle_api_->initializeSensors(vehicle_setting_, sensor_factory, *getGroundTruthKinematics(),
                                    *getGroundTruthEnvironment());

    // A sensor that named a link the URDF does not have is now silently on the root. Say so — the
    // data it produces will look entirely reasonable and describe the wrong body.
    for (const std::string& missing : sensor_factory->getUnresolvedLinks()) {
        UAirBlueprintLib::LogMessageString(
            "UrdfBot WARNING: sensor names link '",
            missing + "', which is not in " + model_.name + " - mounted on the root instead",
            LogDebugLevel::Failure);
    }

    // ⚠ An unpinned robot in a world with no ground falls forever, and looks exactly like a
    // broken physics backend rather than a configuration mistake. Say so at load instead of letting
    // the operator watch a rover accelerate into the void and guess why.
    if (!opts.fixed_base && !have_static_world && !opts.add_ground_plane) {
        UAirBlueprintLib::LogMessageString(
            "UrdfBot WARNING: ",
            getVehicleName() + " has UrdfFixedBase=false, the level mirror produced no geometry, "
            "and there is no scaffolding plane - this robot's physics world contains the robot and "
            "NOTHING ELSE, so it will free-fall indefinitely. Check the static-world report above: "
            "if nothing mirrored, the level's colliders may not block the Pawn channel. Otherwise "
            "set UrdfGroundPlaneAuto, or UrdfFixedBase=true to pin it.",
            LogDebugLevel::Failure);
    }

    setupDriveJoints();

    built_ = true;
#endif
}

void UrdfBotSimApi::setupDriveJoints()
{
    const auto& d = vehicle_setting_->urdf_drive;
    if (!d.enabled) return;

    // Resolve names to indices once. A name the URDF does not have is reported here rather than
    // being skipped every step in silence — the whole failure mode this workstream guards against
    // is a control that appears to work and does nothing.
    auto resolve = [&](const std::map<std::string, double>& src, std::vector<DriveMapping>& out,
                       const char* what) {
        for (const auto& kv : src) {
            const int j = backend_->findJoint(kv.first);
            if (j < 0) {
                UAirBlueprintLib::LogMessageString(
                    "UrdfBot WARNING: UrdfDrive names ",
                    std::string(what) + " joint '" + kv.first + "', which is not in " + model_.name +
                        " - ignored",
                    LogDebugLevel::Failure);
                continue;
            }
            out.push_back(DriveMapping{ static_cast<size_t>(j), kv.second });
        }
    };
    resolve(d.drive_joints, drive_joints_, "drive");
    resolve(d.steer_joints, steer_joints_, "steer");

    // Control modes are set once, not per step. ⚠ Setting a mode re-enables the joint's spring or
    // motor, so doing it every step would fight the solver's warm-start rather than merely being
    // wasteful.
    for (const DriveMapping& m : drive_joints_)
        backend_->setJointTarget(m.joint, urdf::ControlMode::Velocity, 0.0);
    for (const DriveMapping& m : steer_joints_) {
        backend_->setJointTarget(m.joint, urdf::ControlMode::Position, 0.0);
        backend_->setPositionGains(m.joint, d.steer_hertz, d.steer_damping_ratio);
    }

    UAirBlueprintLib::LogMessageString(
        "UrdfBot: keyboard drive ready - ",
        Utils::stringf("%d drive joint(s), %d steer joint(s). W/S or Up/Down = throttle, "
                       "A/D or Left/Right or Numpad4/6 = steer, Space = stop.",
                       static_cast<int>(drive_joints_.size()),
                       static_cast<int>(steer_joints_.size())),
        LogDebugLevel::Informational);

    // To Blocks.log, because the on-screen channel cannot be read back after the fact and these
    // three numbers separate the plausible causes of "steering does nothing":
    //   0 steer joints        -> the names in settings did not resolve
    //   max_steer_angle 0     -> the command is computed as zero however good the input is
    //   both fine             -> the axis is not arriving, which is an Unreal input problem
    UE_LOG(LogUrdfBot, Log,
           TEXT("drive setup: %d drive joints, %d steer joints, max_wheel_speed=%.3f, "
                "max_steer_angle=%.3f, steer_hertz=%.1f, steer_damping=%.2f"),
           static_cast<int>(drive_joints_.size()), static_cast<int>(steer_joints_.size()),
           d.max_wheel_speed, d.max_steer_angle, d.steer_hertz, d.steer_damping_ratio);
    for (const DriveMapping& m : steer_joints_) {
        UE_LOG(LogUrdfBot, Log, TEXT("  steer joint[%d] '%s' multiplier=%.2f"),
               static_cast<int>(m.joint), UTF8_TO_TCHAR(backend_->jointName(m.joint).c_str()),
               m.multiplier);
    }
}

void UrdfBotSimApi::applyDriveInput()
{
    if (drive_joints_.empty() && steer_joints_.empty()) return;

    // Read the axes the game thread last wrote. Relaxed loads: a one-frame-old throttle is
    // indistinguishable from a key pressed one frame later, so there is nothing to order against.
    const auto* pawn = static_cast<const AUrdfBotPawn*>(getPawn());
    const auto& in = pawn->getDriveInput();
    const float throttle = in.throttle.load(std::memory_order_relaxed);
    const float steering = in.steering.load(std::memory_order_relaxed);

    const auto& d = vehicle_setting_->urdf_drive;
    for (const DriveMapping& m : drive_joints_)
        backend_->setJointTarget(m.joint, urdf::ControlMode::Velocity,
                                 throttle * d.max_wheel_speed * m.multiplier);
    for (const DriveMapping& m : steer_joints_)
        backend_->setJointTarget(m.joint, urdf::ControlMode::Position,
                                 steering * d.max_steer_angle * m.multiplier);

    // ⚠ Diagnostic, on axis CHANGE only so the log stays readable. "Steering does nothing" has
    // several causes that look identical from the driver's seat, and this line distinguishes them
    // in one press: it prints the axis as received, the target computed from it, and the angle the
    // joint actually reached. Physics has been ruled out headlessly (measure_steering.cpp shows all
    // three ExoMy modes working), so if `steer` stays 0.000 here the axis is not arriving.
    if (std::fabs(throttle - last_logged_throttle_) > 1e-3f ||
        std::fabs(steering - last_logged_steering_) > 1e-3f) {
        last_logged_throttle_ = throttle;
        last_logged_steering_ = steering;

        double target = 0, actual = 0;
        const char* name = "<none>";
        if (!steer_joints_.empty()) {
            const DriveMapping& m = steer_joints_.front();
            target = steering * d.max_steer_angle * m.multiplier;
            actual = backend_->getJointState(m.joint).position;
            name = backend_->jointName(m.joint).c_str();
        }
        UE_LOG(LogUrdfBot, Log,
               TEXT("drive input: throttle=%.3f steer=%.3f -> %s target=%.3f actual=%.3f"),
               throttle, steering, UTF8_TO_TCHAR(name), target, actual);

        // On screen too, one line, replaced in place - so it can be watched live while driving.
        UAirBlueprintLib::LogMessageString(
            "UrdfBot drive input ",
            Utils::stringf("throttle=%.2f  steer=%.2f  ->  %s target=%.3f actual=%.3f",
                           throttle, steering, name, target, actual),
            LogDebugLevel::Informational);
    }
}

// ---------------------------------------------------------------------------------------------
// PHYSICS THREAD
// ---------------------------------------------------------------------------------------------
void UrdfBotSimApi::update(float delta)
{
    // ⚠ `delta` is 0 here. World::worldUpdatorAsync calls update() with no argument, so the
    // default propagates all the way down. dt comes from the clock, exactly as FastPhysicsEngine
    // does it. Trusting the argument would step the backend by zero forever — a robot that loads,
    // reports sane state, and never moves.
    if (built_) {
        // Apply the driver's input before stepping, so the command and the step it affects belong
        // to the same iteration.
        applyDriveInput();

        const auto clock = msr::airlib::ClockFactory::get();
        if (last_update_time_ == 0) last_update_time_ = clock->nowNanos();
        const msr::airlib::TTimeDelta dt = clock->updateSince(last_update_time_);

        steps_taken_ += backend_->step(static_cast<double>(dt));

        // Publish. Held only for the copy — the solve above is deliberately outside the lock, so
        // the game thread never waits on Box3D.
        const size_t n = backend_->linkCount();
        std::vector<urdf::LinkPose> fresh(n);
        for (size_t i = 0; i < n; ++i) fresh[i] = backend_->getLinkPose(i);
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot_.swap(fresh);
        }
    }

    PawnSimApi::update(delta);
}

// ---------------------------------------------------------------------------------------------
// GAME THREAD, under physics_world_->lock()
// ---------------------------------------------------------------------------------------------
void UrdfBotSimApi::updateRenderedState(float dt)
{
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        render_poses_ = snapshot_;
    }

    // ⚠ Kinematic poses are pushed HERE, and this is the only correct place for them.
    //
    // updateRenderedState runs on the GAME thread while physics_world_->lock() is held, so the
    // physics thread is stopped: reading Unreal actor transforms is legal (game thread) and writing
    // to the backend is safe (physics paused). Doing it in update() would read UObject transforms
    // off the physics thread; doing it in updateRendering() would write to the backend outside the
    // lock. This is the same producer/consumer boundary the whole design already rests on.
    //
    // ⚠ Poses therefore refresh at FRAME rate while physics steps at 333 Hz, so a fast-moving
    // mirrored object is up to one frame stale. That is a real limit and not a bug to chase: the
    // source of truth is Unreal's game thread and it cannot be sampled faster than it runs.
    for (const KinematicMirror& km : kinematic_mirrors_) {
        UPrimitiveComponent* c = km.component.Get();
        if (c == nullptr) continue;  // actor destroyed mid-run: stop pushing, do not crash
        const FTransform tm = c->GetComponentTransform();
        const float w2m = getNedTransform().fromNed(1.0f);
        backend_->setKinematicPose(km.handle, UrdfTransform::toUrdfVec(tm.GetTranslation(), w2m),
                                   UrdfTransform::toUrdfQuat(tm.GetRotation()));
    }

    // Feed AirSim's sensors from the pawn's motion, which is the production pattern for every
    // vehicle whose dynamics AirSim does not own (PawnSimApi.cpp:701 — "update kinematics from
    // pawn's movement instead of physics engine"). It is how every Chaos car already works.
    PawnSimApi::updateRenderedState(dt);
}

// ---------------------------------------------------------------------------------------------
// GAME THREAD, outside the lock — this is the only place Unreal objects are touched
// ---------------------------------------------------------------------------------------------
void UrdfBotSimApi::updateRendering(float dt)
{
    if (built_ && !render_poses_.empty()) {
        auto* pawn = static_cast<AUrdfBotPawn*>(getPawn());
        const TArray<USceneComponent*>& links = pawn->getLinkComponentsByIndex();

        // fromNed(1 m) is world_to_meters. Asking NedTransform rather than hard-coding 100 means a
        // rescaled map cannot silently halve the robot.
        const float world_to_meters = getNedTransform().fromNed(1.0f);

        // Box3D poses are in the **world** frame — origin at the Unreal world origin, not at this
        // robot's spawn point — so this is now a pure frame change with nothing composed onto it.
        //
        // ⚠ It used to compose against the pawn's spawn transform, and that had to go before
        // static geometry could work: a level mirrored once is at fixed world coordinates, so a
        // robot whose solver frame hangs off its own spawn point would need its own copy of the
        // level, translated. Placement moved to BackendOptions::root_position instead — the robot
        // is put *into* the shared frame rather than the frame being moved to the robot. See
        // analysis doc §6.0c; it is also what makes the single shared world of stage 3 reachable.
        auto toWorld = [&](const urdf::LinkPose& p) {
            return UrdfTransform::toFTransform(p, world_to_meters);
        };

        // ⚠ Move the pawn actor to the root link first. PawnSimApi::updateKinematics derives the
        // vehicle's pose, velocity and acceleration from the *pawn's* motion — it is what feeds
        // IMU, GPS, barometer and magnetometer. Leaving the actor parked at its spawn point while
        // only the child components moved would give a robot that visibly drives across the map
        // and reports, through every inertial sensor, that it never moved.
        const int root = model_.root_link;
        if (root >= 0 && root < static_cast<int>(render_poses_.size())) {
            const FTransform root_world = toWorld(render_poses_[root]);
            pawn->SetActorLocationAndRotation(root_world.GetLocation(), root_world.GetRotation(),
                                              /*bSweep=*/false, nullptr,
                                              ETeleportType::TeleportPhysics);
        }

        const int32 n = std::min<int32>(links.Num(), static_cast<int32>(render_poses_.size()));
        for (int32 i = 0; i < n; ++i) {
            USceneComponent* c = links[i];
            if (!c) continue;
            const FTransform world = toWorld(render_poses_[i]);
            c->SetWorldLocationAndRotation(world.GetLocation(), world.GetRotation(),
                                           /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);
        }
    }

    PawnSimApi::updateRendering(dt);
}

void UrdfBotSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();
    if (built_) {
        // ⚠ Rebuilds the Box3D world rather than rewriting poses. Box3D has no rollback
        // determinism — contact caches, warm-start impulses and island state survive a pose write,
        // so a pose-restoring reset diverges silently from a fresh build (§6.4). Measured at 0.05 ms.
        backend_->reset();
        last_update_time_ = 0;
        steps_taken_ = 0;
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        for (size_t i = 0; i < snapshot_.size(); ++i) snapshot_[i] = backend_->getLinkPose(i);
    }
}

void UrdfBotSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);
    if (!built_) return;

    reporter.writeValue("URDF", model_.name);
    reporter.writeValue("links", static_cast<int>(backend_->linkCount()));
    reporter.writeValue("joints", static_cast<int>(backend_->jointCount()));
    reporter.writeValue("backend", backend_->backendName());
    reporter.writeValue("box3d steps", static_cast<int>(steps_taken_));
    reporter.writeValue("R2 links w/o collision", audit_.links_visual_only);
}

std::string UrdfBotSimApi::getRecordFileLine(bool is_header_line) const
{
    std::string common = PawnSimApi::getRecordFileLine(is_header_line);
    if (!built_) return common;

    // Joint state is what a URDF robot's recording is *for*; the pawn pose alone says nothing
    // about its configuration.
    std::ostringstream ss;
    for (size_t i = 0; i < backend_->jointCount(); ++i) {
        if (is_header_line) {
            ss << backend_->jointName(i) << "_q\t" << backend_->jointName(i) << "_qd\t";
        }
        else {
            const urdf::JointState s = backend_->getJointState(i);
            ss << s.position << "\t" << s.velocity << "\t";
        }
    }
    return common + ss.str();
}

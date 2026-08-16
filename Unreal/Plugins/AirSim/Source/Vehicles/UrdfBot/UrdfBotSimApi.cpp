#include "UrdfBotSimApi.h"

#include "UrdfTransform.h"

#include "Engine/World.h"
#include "UnrealSensors/UnrealSensorFactory.h"

#include "common/ClockFactory.hpp"
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

    // R2. Reported at load because it cannot be seen any other way: a URDF whose links are almost
    // all visual-only produces a robot that looks complete, drives on nearly nothing, and is fully
    // visible to LiDAR. On ExoMy that is 17 of 23 links and ~80% of the mass.
    audit_ = urdf::auditCollisionConsistency(model_, dirOf(urdf_path),
                                             vehicle_setting_->urdf_mesh_search_paths);
    if (vehicle_setting_->urdf_report_collision_audit)
        UAirBlueprintLib::LogMessageString("UrdfBot R2 audit\n", audit_.report(),
                                           LogDebugLevel::Informational);

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
    opts.mimic.allow_servo_follower = vehicle_setting_->urdf_allow_mimic_follower;
    // Where to put the scaffolding floor.
    //
    // ⚠ Auto is strongly preferred, and the explicit height exists mainly for reproducibility.
    // Asking an operator for a height in the *robot's* frame means asking them to know where
    // PlayerStart sits relative to the map's floor — those are different numbers, and getting it
    // wrong puts the simulated floor underneath the visible one, where the robot lands out of
    // sight and looks like it fell through the world.
    if (vehicle_setting_->urdf_ground_plane_auto) {
        const FVector start = getPawn()->GetActorLocation();
        FCollisionQueryParams params(FName(TEXT("UrdfGroundProbe")), /*bTraceComplex=*/true);
        params.AddIgnoredActor(getPawn());  // the robot's own link components carry collision

        FHitResult hit;
        const FVector end = start - FVector(0.0f, 0.0f, 1.0e6f);
        if (getPawn()->GetWorld()->LineTraceSingleByChannel(hit, start, end, ECC_Visibility, params)) {
            const float world_to_meters = getNedTransform().fromNed(1.0f);
            opts.add_ground_plane = true;
            // Unreal Z and URDF Z both point up, so this is a plain scaled difference.
            opts.ground_plane_z = (hit.ImpactPoint.Z - start.Z) / world_to_meters;
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: ground probe hit ",
                Utils::stringf("%s at %.3f m below the spawn point - scaffolding floor placed there",
                               TCHAR_TO_UTF8(*GetNameSafe(hit.GetActor())), -opts.ground_plane_z),
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

    auto* pawn = static_cast<AUrdfBotPawn*>(getPawn());
    pawn->buildFromModel(model_, dirOf(urdf_path), vehicle_setting_->urdf_mesh_search_paths);

    // The robot's URDF world origin, fixed once. Captured before anything moves the pawn, because
    // updateRendering then drives the pawn itself from the root link — so reading it later would
    // compound the robot's own motion into its origin and make it accelerate away.
    robot_origin_ = pawn->GetActorTransform();

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

    // ⚠ An unpinned robot in a world with no ground falls forever, and looks exactly like a broken
    // physics backend. Until static world geometry is mirrored in, "not fixed base" plus "no ground
    // plane" is almost always a settings mistake rather than an intent — so say so at load instead
    // of letting the operator watch a rover accelerate into the void and guess why.
    if (!opts.fixed_base && !opts.add_ground_plane) {
        UAirBlueprintLib::LogMessageString(
            "UrdfBot WARNING: ",
            getVehicleName() + " has UrdfFixedBase=false and no UrdfGroundPlaneZ. Static world "
            "geometry is not implemented, so this robot's physics world contains the robot and "
            "NOTHING ELSE - it will free-fall indefinitely. Set UrdfGroundPlaneZ (to minus the "
            "spawn height) to give it a floor, or UrdfFixedBase=true to pin it.",
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
                       "A/D or Left/Right = steer, Space = stop.",
                       static_cast<int>(drive_joints_.size()),
                       static_cast<int>(steer_joints_.size())),
        LogDebugLevel::Informational);
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

        // Box3D poses live in the robot's own URDF world, whose origin is wherever AirSim spawned
        // the pawn. Composing against the spawn transform is what lets several robots coexist and
        // keeps PlayerStart meaning what it means for every other vehicle type.
        auto toWorld = [&](const urdf::LinkPose& p) {
            return UrdfTransform::toFTransform(p, world_to_meters) * robot_origin_;
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

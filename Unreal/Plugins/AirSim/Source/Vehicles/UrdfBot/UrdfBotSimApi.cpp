#include "UrdfBotSimApi.h"

#include "UrdfTransform.h"
#include "UrdfWorldGeometry.h"

#include "SimMode/SimModeBase.h"

#include "EngineUtils.h"

#include "Engine/World.h"
#include "UnrealSensors/UnrealSensorFactory.h"

#include "common/ClockFactory.hpp"
#include "urdf/UrdfCollisionSynthesis.hpp"
#include "urdf/UrdfParser.hpp"
#include "urdf/UrdfStaticWorld.hpp"

#if WITH_BOX3D_BINDING
#include "urdf/backends/Box3DUrdfBackend.hpp"
#include "urdf/backends/mujoco/MuJoCoSharedUrdfBackend.hpp"
#endif

#if WITH_MUJOCO_BINDING
#include "urdf/backends/mujoco/MuJoCoUrdfBackend.hpp"
#endif
// ⚠ NOT GUARDED BY A WITH_*_BINDING. The Newton sidecar backend links against nothing: it is POSIX
// mmap and the URDF descriptor headers, and Newton itself is a separate PROCESS. It is built by the
// always-on src/mpm/*.cpp glob, exactly like the rest of the MPM link, so it is available in every
// build and needs no "is it in this build" story.
#include "mpm/NewtonSidecarUrdfBackend.hpp"

#include "HAL/PlatformFileManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopedSlowTask.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <cstring>
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <set>
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
                   const NedTransform* ned, msr::airlib::GeoPoint home,
                   std::mutex* backend_mutex)
            : model_(model), backend_(backend), mimic_(mimic), ned_(ned), home_(home),
              backend_mutex_(backend_mutex)
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

        void setJointTorques(const std::vector<std::string>& joints,
                             const std::vector<double>& values) override
        {
            if (joints.size() != values.size())
                throw std::invalid_argument(
                    "setJointTorques: " + std::to_string(joints.size()) + " joint name(s) but " +
                    std::to_string(values.size()) + " torque(s)");

            std::lock_guard<std::mutex> lock(*backend_mutex_);

            // ⚠ RESOLVE EVERY NAME BEFORE APPLYING ANY TORQUE. A controller that gets eleven of
            // twelve legs actuated is worse than one that gets none: the robot moves, plausibly,
            // and the missing joint is invisible until something falls over. requireJoint throws on
            // a name the robot does not have, so doing the whole lookup pass first makes the call
            // all-or-nothing.
            std::vector<size_t> idx;
            idx.reserve(joints.size());
            for (const std::string& j : joints)
                idx.push_back(static_cast<size_t>(requireJoint(j)));

            for (size_t i = 0; i < idx.size(); ++i)
                backend_->setJointTarget(idx[i], urdf::ControlMode::Effort, values[i]);
        }
        void setJointPositionGains(const std::string& joint, double hertz, double ratio) override
        {
            std::lock_guard<std::mutex> lock(*backend_mutex_);
            backend_->setPositionGains(static_cast<size_t>(requireJoint(joint)), hertz, ratio);
        }

        urdf::JointState getJointState(const std::string& joint) const override
        {
            std::lock_guard<std::mutex> lock(*backend_mutex_);
            return backend_->getJointState(static_cast<size_t>(requireJoint(joint)));
        }

        std::vector<JointStateInfo> getJointStates() const override
        {
            std::lock_guard<std::mutex> lock(*backend_mutex_);
            std::vector<JointStateInfo> out;
            out.reserve(model_->joints.size());
            for (size_t i = 0; i < model_->joints.size(); ++i) {
                const urdf::Joint& j = model_->joints[i];

                // ⚠ FIXED joints are skipped. They have no state, and ROS convention is that
                // joint_states carries only actuatable joints — robot_state_publisher composes
                // fixed transforms from the URDF itself. Publishing them would be harmless noise
                // for RViz but would misrepresent the robot to anything counting DoF.
                if (j.type == urdf::JointType::Fixed) continue;

                // ⚠ Cosmetic <mimic> joints have NO joint in the solver at all — they were
                // resolved in the rendering layer — so findJoint misses them. Omitted rather than
                // reported as zero, because zero would be a plausible-looking lie about a joint
                // that is in fact tracking its source. A client that needs them can see the
                // coupling through getJoints()'s mimic_role and mimic_source fields.
                const int bj = backend_->findJoint(j.name);
                if (bj < 0) continue;

                const urdf::JointState st = backend_->getJointState(static_cast<size_t>(bj));
                JointStateInfo info;
                info.name = j.name;
                info.position = st.position;
                info.velocity = st.velocity;
                info.effort = st.effort;
                out.push_back(std::move(info));
            }
            return out;
        }

        std::string getUrdfXml() const override { return urdf_xml_; }
        void setUrdfXml(std::string xml) { urdf_xml_ = std::move(xml); }

        LinkPoseInfo getLinkPose(const std::string& link) const override
        {
            urdf::LinkPose p;
            urdf::Twist t;
            {
                std::lock_guard<std::mutex> lock(*backend_mutex_);
                const int i = backend_->findLink(link);
                if (i < 0) throw std::invalid_argument("no such link: '" + link + "'");

                p = backend_->getLinkPose(static_cast<size_t>(i));
                t = backend_->getLinkTwist(static_cast<size_t>(i));
            }

            LinkPoseInfo out;
            out.name = link;

            // ⚠ PLAYERSTART-RELATIVE NED, via NedTransform — not a direct URDF-to-NED flip.
            //
            // The direct flip (x, -y, -z) was correct while the solver frame was each robot's own
            // spawn point. It stopped being correct when that frame moved to the UNREAL WORLD
            // ORIGIN so a mirrored level could be shared between robots (§6.0c): the numbers stayed
            // well-formed and silently changed origin. Measured on a live sim, the same rover
            // reported base_link at x = 119 m here and 0.34 m through simGetVehiclePose — two
            // different origins, no error, and a client differencing them would be badly wrong.
            //
            // Routing through NedTransform is what every other AirSim pose already does, so link
            // poses share an origin with simGetVehiclePose and this vehicle's sensors.
            //
            // ⚠ NOT WITH OTHER VEHICLES, despite what this comment used to claim. NedTransform is
            // constructed with THE PAWN as its pivot (PawnSimApi.cpp:18), so toLocalNed subtracts
            // that vehicle's OWN spawn location: every robot reports itself in its own frame, and
            // two robots 1.5 m apart both read ~0. Observed 2026-08-24 with two rovers.
            //
            // Differences are still exact - the offset is captured once at construction and never
            // moves - so displacement, velocity and "did it get pushed" are all trustworthy. What
            // is NOT valid is comparing one robot's POSITION against another's, which is precisely
            // the cross-vehicle question a shared world invites. The coordinator's own snapshots
            // are world-frame (invariant 0); this RPC surface is not, and nothing yet exposes the
            // world-frame state to clients. toGlobalNed() is the shared-origin conversion if that
            // is ever wanted here.
            const float world_to_meters = ned_->fromNed(1.0f);
            const FTransform link_world(UrdfTransform::toFQuat(p.orientation),
                                        UrdfTransform::toFVector(p.position, world_to_meters));
            out.pose = ned_->toLocalNed(link_world);

            // ⚠ Velocities use toLocalNedVelocity, which applies the axis flips and the unit scale
            // but NOT the origin translation — subtracting PlayerStart from a velocity would be
            // meaningless. The metre-to-centimetre scaling in toFVector and the reciprocal inside
            // toLocalNedVelocity cancel, so angular velocity in rad/s survives the round trip
            // unscaled while linear velocity in m/s comes back in m/s.
            out.twist.linear =
                ned_->toLocalNedVelocity(UrdfTransform::toFVector(t.linear, world_to_meters));
            out.twist.angular =
                ned_->toLocalNedVelocity(UrdfTransform::toFVector(t.angular, world_to_meters));
            return out;
        }

        /// Forwarded to the sim api, which publishes the kinematics and the clock reading as one
        /// snapshot from the game thread. Not assembled here: this class runs on the RPC thread,
        /// where the two would be two separate reads and the stamp would not describe the sample.
        UrdfBotState getUrdfBotState() const override
        {
            if (sim_api_ == nullptr)
                throw std::runtime_error("urdfbot state is not available before initialisation");
            return sim_api_->getUrdfBotState();
        }
        void setSimApi(const UrdfBotSimApi* s) { sim_api_ = s; }

        msr::airlib::GeoPoint getHomeGeoPoint() const override { return home_; }

        // ⚠ resetImplementation(), not reset(). UpdatableObject::reset() is non-virtual - it does
        // the ordering bookkeeping and then calls the pure-virtual resetImplementation(). Marking
        // reset() override does not compile, which is the good outcome; overriding it by shadowing
        // would have quietly bypassed that bookkeeping.
        //
        // ⚠ Chains to the base rather than doing nothing. UrdfBotApiBase::resetImplementation()
        // resets the SENSORS; an empty override here silently skipped that. The joint targets need
        // no undoing — they live in the backend, which UrdfBotSimApi resets separately.
        void resetImplementation() override { UrdfBotApiBase::resetImplementation(); }

        // ⚠ No update() override here on purpose. UrdfBotApiBase::update() ticks the sensor
        // collection, and an override that called UpdatableObject::update() directly — as this
        // did — silently skipped it, leaving every sensor frozen at its first sample.

    private:
        int requireJoint(const std::string& name) const
        {
            const int i = backend_->findJoint(name);
            if (i < 0) throw std::invalid_argument("no such joint: '" + name + "'");
            return i;
        }
        void command(const std::string& joint, urdf::ControlMode mode, double v)
        {
            std::lock_guard<std::mutex> lock(*backend_mutex_);
            backend_->setJointTarget(static_cast<size_t>(requireJoint(joint)), mode, v);
        }

        std::string urdf_xml_;
        const urdf::Robot* model_;
        urdf::UrdfRobotBackend* backend_;
        const std::vector<urdf::MimicClassification>* mimic_;
        const NedTransform* ned_;
        msr::airlib::GeoPoint home_;
        std::mutex* backend_mutex_;
        /// The owning sim api. Borrowed, not owned — it outlives this object, which is one of its
        /// members.
        const UrdfBotSimApi* sim_api_ = nullptr;
    };
} // namespace

/// The vehicle's half of one coordinated tick.
///
/// Separate from the sim api because the coordinator owns its participants by `shared_ptr` — a
/// callback must not be able to outlive the object it targets — while the api provider owns sim
/// apis by `unique_ptr`. The api detaches this in its destructor, so a late callback becomes a
/// no-op rather than a dangling call.
class UrdfBotStepParticipant : public msr::airlib::PhysicsSceneParticipant
{
public:
    explicit UrdfBotStepParticipant(UrdfBotSimApi* api) : api_(api) {}

    void detach() { api_ = nullptr; }

    void onManifestFinalize(const msr::airlib::PhysicsManifestContext&) noexcept override
    {
        // Runs after EVERY participant committed, so every shared scene has compiled by now. That
        // ordering is the whole reason this lives on the finalize hook rather than on commit.
        if (api_ != nullptr) api_->onSharedSceneReady();
    }

    void onStepPrepare(const msr::airlib::PhysicsStepContext& context) override
    {
        // ⚠ Commands are latched during the pre-roll too. Their values are whatever reset left
        // them at, which for a rebuilt scene is zero; what must NOT happen here is publishing.
        (void)context;
        if (api_ != nullptr) api_->coordinatedPreSolve();
    }

    void onStepCommit(const msr::airlib::PhysicsStepContext& context) override
    {
        // Commit runs only for a real step. The pre-settle deliberately never reaches it, which is
        // exactly how sensor publication is suppressed for that interval.
        if (api_ != nullptr) api_->coordinatedPostSolve(context.dt);
    }

private:
    UrdfBotSimApi* api_ = nullptr;
};

UrdfBotSimApi::UrdfBotSimApi(const Params& params,
                             const AirSimSettings::VehicleSetting* vehicle_setting,
                             ICoordinatedPhysicsScene* coordinated_scene)
    : PawnSimApi(params), vehicle_setting_(vehicle_setting),
      coordinated_scene_(coordinated_scene)
{
    if (coordinated_scene_ != nullptr) {
        // Registered before the manifest is committed, which is what makes participation part of
        // the frozen scenario rather than something a robot can join later.
        step_participant_ = std::make_shared<UrdfBotStepParticipant>(this);
        coordinated_scene_->registerParticipant(
            ICoordinatedPhysicsScene::vehicleParticipantId(getVehicleName()), 0,
            step_participant_);
    }
}

UrdfBotSimApi::~UrdfBotSimApi()
{
    if (step_participant_) step_participant_->detach();
}

void UrdfBotSimApi::initialize()
{
    PawnSimApi::initialize();
    loadModelAndBackend();

    // Seed the published pair, so a client that calls getUrdfBotState before the first rendered
    // frame gets the spawn pose at a real clock reading rather than a timestamp of 0. A zero stamp
    // is not merely unhelpful: rclcpp::Time(0) is a valid time that TF will happily extrapolate
    // from, so the bad value would propagate rather than announce itself.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        published_state_ = msr::airlib::UrdfBotApiBase::UrdfBotState(*getGroundTruthKinematics(),
                                                                     clock()->nowNanos());
    }
}

void UrdfBotSimApi::loadModelAndBackend()
{
#if !WITH_BOX3D_BINDING && !WITH_MUJOCO_BINDING
    throw std::runtime_error(
        "Vehicle '" + getVehicleName() + "' is of type \"urdfbot\", but this build has NO URDF "
        "physics backend at all (WITH_BOX3D_BINDING=0 and WITH_MUJOCO_BINDING=0). Build box3d and "
        "re-run build.sh - and ./build_thirdparty.sh too if you want the MuJoCo engine - or remove the "
        "vehicle from settings.json.");
#else
    const std::string urdf_path = vehicle_setting_->urdf_file;
    model_ = urdf::parseFile(urdf_path);

    // Kept so getUrdfXml() can serve it to clients that cannot see this path — see
    // UrdfBotApiBase::getUrdfXml. Read separately from the parser rather than re-serialised, so
    // what a client receives is byte-for-byte what was parsed.
    {
        std::ifstream urdf_in(urdf_path);
        if (urdf_in) {
            std::stringstream ss;
            ss << urdf_in.rdbuf();
            urdf_xml_raw_ = ss.str();
        }
    }

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

    // --- which physics engine runs this robot -------------------------------------------------
    //
    // ⚠ Per VEHICLE, not per simulation. Two urdfbots in one settings file may name different
    // engines and each gets its own solver; nothing is shared between them, and neither engine is
    // aware of the other. That is the point: the Go2 trot needs kp = 160 on Box3D against the
    // kp = 20 it is trained at in MuJoCo, and running one robot and one controller against both
    // engines is what turns an 8x discrepancy into something attributable.
    //
    // ⚠ Box3D remains the DEFAULT. An absent "PhysicsEngine", an empty one, or "Box3D" all take
    // the path that existed before MuJoCo was added; every existing settings file keeps its exact
    // previous behaviour.
    const std::string engine = Utils::toLower(vehicle_setting_->physics_engine);

    // Listed for the error message below, so an operator is told what this BUILD can do rather
    // than what the source could do if it had been built differently. "MuJoCo is not available"
    // and "MuJoCo is not a thing" are very different problems and must not read the same.
    std::string available;
#if WITH_BOX3D_BINDING
    available += "\"Box3D\"";
#endif
#if WITH_MUJOCO_BINDING
    available += (available.empty() ? "" : ", ") + std::string("\"MuJoCo\"");
#endif

    std::unique_ptr<urdf::UrdfRobotBackend> backend;

    if (engine.empty() || engine == "box3d") {
#if WITH_BOX3D_BINDING
        auto box3d_backend = std::make_unique<urdf::Box3DUrdfBackend>();
        // ⚠ BEFORE buildFromUrdf. Attaching afterwards would already have created a private
        // b3World, which is the thing coordinated mode exists to eliminate.
        if (coordinated_scene_ != nullptr)
            box3d_backend->attachToScene(&coordinated_scene_->box3dScene());
        backend = std::move(box3d_backend);
#else
        throw std::invalid_argument(
            "Vehicle '" + getVehicleName() + "': PhysicsEngine \"Box3D\" is not in this build "
            "(WITH_BOX3D_BINDING=0). Available: " + available + ". Build box3d and re-run "
            "build.sh.");
#endif
    }
    else if (engine == "mujoco") {
#if WITH_MUJOCO_BINDING
        if (coordinated_scene_ != nullptr) {
            // ⚠ A DIFFERENT CLASS, not a mode of MuJoCoUrdfBackend. The shared facade owns no
            // mjModel/mjData/mjSpec of its own; the world's scene does, and this robot is one
            // namespaced articulation inside it.
            backend = std::make_unique<urdf::MuJoCoSharedUrdfBackend>(
                coordinated_scene_->mujocoScene(),
                ICoordinatedPhysicsScene::vehicleBodyId(getVehicleName()));
        }
        else {
            backend = std::make_unique<urdf::MuJoCoUrdfBackend>();
        }
#else
        // ⚠ The overwhelmingly likely cause, so it is named first. MuJoCo is not built by
        // build.sh - it needs Unreal's own toolchain and has its own script.
        throw std::invalid_argument(
            "Vehicle '" + getVehicleName() + "': PhysicsEngine \"MuJoCo\" is not in this build "
            "(WITH_MUJOCO_BINDING=0). Available: " + available + ". MuJoCo is NOT built by "
            "build.sh: run ./build_thirdparty.sh once, then ./build.sh.");
#endif
    }
    else if (engine == "newtonsidecar") {
        // ⚠ THE VEHICLE IS SOLVED IN ANOTHER PROCESS. Nothing here integrates it; this backend
        // writes joint targets into shared memory and interpolates the link poses that come back.
        // See NewtonSidecarUrdfBackend.hpp for why that is worth a backend rather than a flag:
        // harvested impulses carry only the reaction to MOTION, so a wheel resting on settled sand
        // reports ~0 N, and support and sinkage are unreachable across the old wire.
        msr::airlib::mpm::NewtonSidecarUrdfBackend::Options newton_opts;
        newton_opts.vehicle_name = vehicle_setting_->newton_sidecar_vehicle.empty()
                                       ? Utils::toLower(getVehicleName())
                                       : vehicle_setting_->newton_sidecar_vehicle;
        // ⚠ SAID OUT LOUD, ALWAYS. The name must equal the sidecar's --own-vehicle spec name, and
        // a mismatch presents as a robot that never moves while both ends report healthy — the
        // exact failure this workstream already shipped once with an empty collider registry.
        UE_LOG(LogUrdfBot, Log,
               TEXT("'%s' runs on the Newton SIDECAR; it will look for vehicle '%s' in %s. "
                    "That name must match the sidecar's --own-vehicle spec."),
               UTF8_TO_TCHAR(getVehicleName().c_str()),
               UTF8_TO_TCHAR(newton_opts.vehicle_name.c_str()),
               UTF8_TO_TCHAR(newton_opts.directory.c_str()));
        backend = std::make_unique<msr::airlib::mpm::NewtonSidecarUrdfBackend>(newton_opts);
    }
    else {
        throw std::invalid_argument(
            "Vehicle '" + getVehicleName() + "': PhysicsEngine \"" +
            vehicle_setting_->physics_engine + "\" is not a URDF physics backend. Available: " +
            available + ", \"NewtonSidecar\".");
    }

    UAirBlueprintLib::LogMessageString(
        "UrdfBot physics engine: ",
        getVehicleName() + " -> " + backend->backendName(), LogDebugLevel::Informational);
    UE_LOG(LogUrdfBot, Log, TEXT("'%s' runs on the %s backend"),
           UTF8_TO_TCHAR(getVehicleName().c_str()), UTF8_TO_TCHAR(backend->backendName()));

    urdf::BackendOptions opts;
    // ⚠ MuJoCo builds its model from the URDF **TEXT**, not from our parsed Robot - it has its own
    // URDF reader (xml_urdf.cc) and re-serialising our parse would create a second, quietly
    // divergent definition of the same robot. Box3D ignores this field. Set unconditionally so the
    // two engines are never handed different information.
    opts.urdf_xml = urdf_xml_raw_;
    // The MultiAgent sim clock advances by exactly getPhysicsLoopPeriod() * 1E-9 per executor
    // iteration (ASimModeWorldMultiAgent::setupClockSpeed overrides the 20 ms ASimModeBase
    // default, which applies to Car/ComputerVision/SkidVehicle only). Matching the backend's
    // internal step to it keeps whole steps consuming whole clock ticks.
    opts.fixed_timestep =
        coordinated_scene_ != nullptr ? coordinated_scene_->fixedStepSeconds() : 0.003;
    opts.fixed_base = vehicle_setting_->urdf_fixed_base;
    opts.mesh_base_dir = dirOf(urdf_path);
    opts.mesh_search_paths = vehicle_setting_->urdf_mesh_search_paths;
    opts.mimic.allow_servo_follower = vehicle_setting_->urdf_allow_mimic_follower;

    // --- convex decomposition -----------------------------------------------------------------
    // Shared by both backends. Without CoACD in the build this is a no-op that yields one part per
    // mesh, i.e. exactly the behaviour that existed before, so nothing here needs a guard.
    {
        const std::string mode = Utils::toLower(vehicle_setting_->urdf_static_mesh_mode);
        opts.static_mesh_mode =
            (mode == "whole") ? urdf::BackendOptions::StaticMeshMode::Whole
          : (mode == "split") ? urdf::BackendOptions::StaticMeshMode::Split
                              : urdf::BackendOptions::StaticMeshMode::Auto;
    }
    opts.static_world_radius = vehicle_setting_->urdf_static_world_radius;
    opts.static_world_max_triangles = vehicle_setting_->urdf_static_world_max_triangles;
    opts.decomposition.enabled = vehicle_setting_->urdf_convex_decomposition;
    opts.decomposition.threshold = vehicle_setting_->urdf_convex_decomposition_threshold;
    opts.decomposition.max_hulls = vehicle_setting_->urdf_convex_decomposition_max_hulls;
    opts.decomposition.cache_dir = vehicle_setting_->urdf_convex_decomposition_cache_dir;

    // ⚠ Default the cache INSIDE THE PLUGIN, so a cook survives being moved to another project.
    //
    // The obvious place is next to the URDF, and it was the first thing tried — but a URDF often
    // lives in a read-only reference clone or a shared model directory, and the cook then belongs
    // to the model rather than to the simulator that produced it. Under the plugin, copying
    // Plugins/AirSim into a new project carries every cooked decomposition with it and the first
    // load there is instant instead of 312 s.
    //
    // The fallbacks are ordered by how much we trust them to be writable, and each is announced,
    // because a silently-disabled cache looks exactly like a slow simulator:
    //   1. UrdfConvexDecompositionCacheDir, if set   — the operator's explicit choice
    //   2. <plugin>/Saved/ConvexCache                — travels with the plugin
    //   3. <project>/Saved/ConvexCache               — plugin dir read-only (packaged builds)
    //   4. next to the URDF                          — last resort
    if (opts.decomposition.enabled && opts.decomposition.cache_dir.empty()) {
        TArray<FString> candidates;
        if (const TSharedPtr<IPlugin> plugin = IPluginManager::Get().FindPlugin(TEXT("AirSim")))
            candidates.Add(FPaths::Combine(plugin->GetBaseDir(), TEXT("Saved"), TEXT("ConvexCache")));
        candidates.Add(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("ConvexCache")));
        candidates.Add(UTF8_TO_TCHAR((dirOf(urdf_path) + "/.convex_cache").c_str()));

        IPlatformFile& pf = FPlatformFileManager::Get().GetPlatformFile();
        for (const FString& dir : candidates) {
            if (!pf.DirectoryExists(*dir) && !pf.CreateDirectoryTree(*dir)) continue;

            // Existing is not the same as writable — a plugin copied from a read-only medium, or
            // a packaged build, can have a directory we cannot add to. Prove it with a real file
            // rather than assume, because the alternative is discovering it 312 s into a load.
            const FString probe = FPaths::Combine(dir, TEXT(".writable"));
            if (!FFileHelper::SaveStringToFile(TEXT("x"), *probe)) continue;
            pf.DeleteFile(*probe);

            opts.decomposition.cache_dir = TCHAR_TO_UTF8(*dir);
            break;
        }
    }

    if (opts.decomposition.enabled && opts.decomposition.cache_dir.empty()) {
        UE_LOG(LogUrdfBot, Warning,
               TEXT("no writable convex-decomposition cache could be created for '%s' - EVERY load "
                    "will re-cook from scratch, which is minutes for a mesh-heavy robot. Set "
                    "UrdfConvexDecompositionCacheDir to somewhere writable."),
               UTF8_TO_TCHAR(getVehicleName().c_str()));
    }
    else if (!opts.decomposition.cache_dir.empty()) {
        UE_LOG(LogUrdfBot, Log, TEXT("convex-decomposition cache: %s"),
               UTF8_TO_TCHAR(opts.decomposition.cache_dir.c_str()));

        // ⚠ Flush BEFORE the cook, and say so loudly. Left on in a settings file this re-cooks on
        // every single load, and the only symptom is a simulator that got mysteriously slow.
        if (vehicle_setting_->urdf_convex_decomposition_flush_cache) {
            const FString dir = UTF8_TO_TCHAR(opts.decomposition.cache_dir.c_str());
            TArray<FString> stale;
            IFileManager::Get().FindFiles(stale, *FPaths::Combine(dir, TEXT("*.cvx")), true, false);
            for (const FString& f : stale)
                IFileManager::Get().Delete(*FPaths::Combine(dir, f));

            UAirBlueprintLib::LogMessageString(
                "UrdfBot: convex cache FLUSHED ",
                Utils::stringf("%d entries deleted - this load re-cooks from scratch. Turn "
                               "UrdfConvexDecompositionFlushCache back off.", stale.Num()),
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("flushed %d cached decompositions from '%s' - re-cooking"), stale.Num(),
                   *dir);
        }
    }

    const float world_to_meters = getNedTransform().fromNed(1.0f);

    // Place the robot in the shared world frame. Read before anything moves the pawn, because
    // updateRendering then drives the pawn from the root link — reading it later would compound
    // the robot's own motion into its start pose.
    const FTransform spawn = getPawn()->GetActorTransform();
    opts.root_position = UrdfTransform::toUrdfVec(spawn.GetTranslation(), world_to_meters);
    opts.root_orientation = UrdfTransform::toUrdfQuat(spawn.GetRotation());

    // --- sample the ground as a height grid ---------------------------------------------------
    //
    // ⚠ TRACES, NOT MIRRORED GEOMETRY, AND THAT IS THE WHOLE POINT. Unreal answers "what is the
    // ground height here?" exactly, for landscape, static meshes and BSP alike, with no mirroring,
    // no convex approximation and no dependence on how the level's ground happens to be modelled.
    // Deriving a floor from mirrored geometry failed repeatedly on Blocks, whose ground is one
    // 40 km mesh with a bounding box 442 m tall around a surface at z = 0.36.
    //
    // An explicit plane is an operator override. Do not also sample a height field: the two
    // scaffolds would overlap, and the explicit value must remain deterministic.
    // ⚠ In coordinated mode the floor belongs to the world, not to this robot: the shared scene
    // was created with the authored PhysicsCoordinator.GroundPlane, and the per-robot keys are
    // rejected by the settings loader. Everything below therefore stays inert.
    const bool has_explicit_ground_plane =
        coordinated_scene_ == nullptr && !std::isnan(vehicle_setting_->urdf_ground_plane_z);
    UE_LOG(LogUrdfBot, Log,
           TEXT("ground configuration [%s]: auto=%d explicit=%d z=%.6f m"),
           UTF8_TO_TCHAR(getVehicleName().c_str()),
           vehicle_setting_->urdf_ground_plane_auto ? 1 : 0,
           has_explicit_ground_plane ? 1 : 0,
           vehicle_setting_->urdf_ground_plane_z);

    // Only backends that ask for it get one; Box3D cooks the real triangles and needs nothing.
    if (backend->needsScaffoldingGroundPlane() && vehicle_setting_->urdf_ground_plane_auto &&
        !has_explicit_ground_plane && coordinated_scene_ == nullptr) {
        urdf::BackendOptions::HeightField hf;
        if (sampleGroundHeightField(opts.root_position,
                                    vehicle_setting_->urdf_ground_sample_extent, hf)) {
            opts.ground_height_field = hf;
            height_field_centre_ = opts.root_position;
            height_field_half_extent_ = vehicle_setting_->urdf_ground_sample_extent;
            has_height_field_ = true;

            // ⚠ Report it. Dropped during a refactor, and its absence made a working height field
            // indistinguishable from a silent fallback to the flat plane.
            double lo = hf.min_z, hi = hf.min_z;
            for (float h : hf.heights) hi = std::max(hi, hf.min_z + static_cast<double>(h));
            UE_LOG(LogUrdfBot, Log,
                   TEXT("ground height field: %dx%d over %.0f m centred on (%.1f %.1f), "
                        "elevation %.2f..%.2f m (relief %.2f m)"),
                   hf.rows, hf.cols, 2.0 * hf.half_extent, hf.center_x, hf.center_y, lo, hi,
                   hi - lo);
        }
        else {
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("ground height field: NO traces hit around the spawn - falling back to a "
                        "flat plane"));
        }
    }

    // --- static world geometry ----------------------------------------------------------------
    // Mirrored once per level and shared by every robot: the shared_ptr's identity is what makes
    // the cook shared (Box3DStaticGeometry), so this must go through MirrorLevelShared rather than
    // each robot mirroring for itself.
    // ⚠ ONE POLICY PER WORLD in coordinated mode. A shared solver scene has exactly one static
    // world, so which components are in it cannot be a per-robot choice: whichever robot
    // initialised first would define the level for every other robot. The per-vehicle UrdfMirror*
    // keys are rejected by the settings loader in that mode and these world-level values are used.
    const bool coordinated_mirror = coordinated_scene_ != nullptr;
    const AirSimSettings::StaticWorldMirrorSetting& world_mirror =
        AirSimSettings::singleton().physics_coordinator.static_world_mirror;
    const bool mirror_world_geometry = coordinated_mirror
                                           ? world_mirror.enabled
                                           : vehicle_setting_->urdf_mirror_world_geometry;

    if (mirror_world_geometry) {
        UrdfWorldGeometry::FMirrorOptions mirror_opts;
        if (coordinated_mirror) {
            switch (world_mirror.collision_source) {
            case AirSimSettings::StaticWorldCollisionSource::Simple:
                mirror_opts.Source = UrdfWorldGeometry::ECollisionSource::Simple;
                break;
            case AirSimSettings::StaticWorldCollisionSource::Complex:
                mirror_opts.Source = UrdfWorldGeometry::ECollisionSource::Complex;
                break;
            case AirSimSettings::StaticWorldCollisionSource::Auto:
            default:
                mirror_opts.Source = UrdfWorldGeometry::ECollisionSource::Auto;
                break;
            }
            mirror_opts.bIncludeMovable = world_mirror.include_movable;
            mirror_opts.bIncludeOtherVehicles = world_mirror.include_other_vehicles;
            mirror_opts.bIncludeLandscape = world_mirror.include_landscape;
            mirror_opts.bIncludeInstancedMeshes = world_mirror.include_instanced_meshes;
            mirror_opts.MaxInstances = world_mirror.max_instances;
            mirror_opts.DefaultFriction = world_mirror.default_friction;
            mirror_opts.DefaultRestitution = world_mirror.default_restitution;
            for (const std::string& tag : world_mirror.required_tags)
                mirror_opts.RequiredTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));
            for (const std::string& tag : world_mirror.excluded_tags)
                mirror_opts.ExcludedTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));
        }
        else {
            mirror_opts.bIncludeMovable = vehicle_setting_->urdf_mirror_movable;
            mirror_opts.bIncludeOtherVehicles = vehicle_setting_->urdf_mirror_other_vehicles;
            mirror_opts.bIncludeLandscape = vehicle_setting_->urdf_mirror_landscape;
            mirror_opts.bIncludeInstancedMeshes = vehicle_setting_->urdf_mirror_instanced_meshes;
            mirror_opts.MaxInstances = vehicle_setting_->urdf_mirror_max_instances;
            for (const std::string& tag : vehicle_setting_->urdf_world_geometry_tags)
                mirror_opts.RequiredTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));
        }

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

        // ⚠ The shared scene owns the static world; a shared-scene backend REFUSES setStaticWorld
        // for exactly that reason. The mirror is memoised per UWorld, so every robot presents the
        // same shared_ptr and only the first one attaches it.
        if (coordinated_scene_ != nullptr) {
            // ⚠ TEST FOR THE BACKEND YOU MEAN, NEVER FOR "not the other one". These guards were
            // written as `!is_box3d` when Box3D and MuJoCo were the only backends, which made
            // "not Box3D" and "is MuJoCo" the same statement. Adding NewtonSidecar silently broke
            // that equivalence and every such guard started treating it as MuJoCo — one of them
            // static_cast the pointer and read a std::vector<std::string> out of unrelated memory,
            // which surfaced as "Ran out of memory allocating 4636737291354636296 bytes" inside a
            // string concatenation. RTTI is off in Unreal builds, so nothing catches this at
            // runtime and the name IS the discriminator.
            const bool mirror_is_box3d = (std::strcmp(backend->backendName(), "Box3D") == 0);
            const bool mirror_is_mujoco = (std::strcmp(backend->backendName(), "MuJoCo") == 0);
#if WITH_BOX3D_BINDING
            if (mirror_is_box3d) {
                b3urdf::Box3DPhysicsScene& shared_scene = coordinated_scene_->box3dScene();
                if (!shared_scene.hasStaticWorld()) shared_scene.setStaticWorld(static_world_);
            }
#endif
#if WITH_MUJOCO_BINDING
            if (mirror_is_mujoco) {
                urdf::MuJoCoPhysicsScene& shared_scene = coordinated_scene_->mujocoScene();
                if (!shared_scene.compiled()) shared_scene.setStaticWorld(static_world_);
            }
#endif
        }
        else {
            backend->setStaticWorld(static_world_);
        }

        // ⚠ Said out loud, because the level mirrored SUCCESSFULLY and the report above says so.
        // Without this line the log reads as though this robot has 172 bodies of world to collide
        // with, and the only symptom of the truth is the robot falling out of the map.
        if (!backend->mirrorsStaticWorld()) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: ",
                getVehicleName() + " runs on " + backend->backendName() + ", which does NOT yet "
                "mirror level geometry - the mirror above was built and then ignored. This robot "
                "collides only with its own links and the scaffolding ground plane, and will drive "
                "through walls that stop a Box3D robot.",
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("'%s': %s ignores the static world mirror; falling back to the "
                        "scaffolding ground plane"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()),
                   UTF8_TO_TCHAR(backend->backendName()));
        }
        else if (!backend->mirrorsKinematicBodies()) {
            // Separate message from the one above, because it is a materially different gap: the
            // level IS solid, other vehicles are not. Reported at load rather than discovered by
            // driving through another robot.
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("'%s': %s mirrors the static level but NOT other vehicles - it will pass "
                        "through other robots as if they were not there"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()),
                   UTF8_TO_TCHAR(backend->backendName()));
        }

        // Kinematic bodies: registered per robot, because each robot owns its own b3World and gets
        // its own handles. The mirror itself is shared, so the self-exclusion has to happen here —
        // a robot that mirrored its own pawn would weld itself to an obstacle shaped like itself.
        // ⚠ Keyed on mirrorsKinematicBodies, NOT mirrorsStaticWorld. MuJoCo now mirrors the level
        // but still not the moving bodies; with the old single flag, switching static mirroring on
        // would have silently re-enabled this and gone back to logging "+6 bodies ... now solid to
        // this one" about handles that are all -1.
        // ⚠ ONE OWNER PER SHARED WORLD. In a private world every robot registered its own
        // kinematic copy of every moving prop, which was right because the worlds were private.
        // In one shared world that is N overlapping copies of the same lift, and each robot would
        // be pushed by the other robots' copies of it.
        owns_kinematic_mirror_ =
            coordinated_scene_ == nullptr || coordinated_scene_->claimKinematicMirror();
        if (static_world_ && backend->mirrorsKinematicBodies() && owns_kinematic_mirror_) {
            for (size_t i = 0; i < static_world_->kinematic.size(); ++i) {
                UPrimitiveComponent* src =
                    (static_cast<int32>(i) < mirror.KinematicSources.Num())
                        ? mirror.KinematicSources[static_cast<int32>(i)].Get()
                        : nullptr;
                if (src == nullptr) continue;
                if (src->GetOwner() == getPawn()) continue;  // never mirror yourself

                KinematicMirror km;
                km.component = mirror.KinematicSources[static_cast<int32>(i)];
                km.handle = backend->addKinematicBody(static_world_->kinematic[i]);
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

    // ⚠ "Does THIS ROBOT have a world", not "did the level mirror". The two differ for any backend
    // that ignores the mirror, and conflating them is what dropped the MuJoCo Scout through the
    // floor: the level mirrored 172 bodies, the scaffolding plane was suppressed as redundant, and
    // MuJoCo's setStaticWorld discarded the mirror — leaving a physics world containing the robot
    // and nothing else. Everything below keys off this: the ground-plane suppression, the ground
    // probe, and the free-fall warning.
    const bool have_static_world =
        static_world_ && !static_world_->bodies.empty() && backend->mirrorsStaticWorld();

    // ⚠ A backend can mirror the level AND still need the traced floor. MuJoCo does: it skips the
    // level's vast concave ground mesh, which it cannot represent, and stands on an exact plane
    // instead. Box3D cooks the real ground triangles and must NOT get a second floor through the
    // map. Suppressing the plane purely on "did the level mirror" conflated the two and left the
    // MuJoCo robot with nothing to stand on.
    const bool suppress_automatic_ground_plane =
        have_static_world && !backend->needsScaffoldingGroundPlane();
    // --- scaffolding floor: FALLBACK ONLY, now that the level is mirrored ----------------------
    //
    // ⚠ The automatic plane is suppressed whenever real geometry exists. A flat plane and a
    // mirrored level are not
    // additive: the plane is infinite-ish (a 100 m slab) and would sit *through* the map, so a
    // rover driving down a ramp would stop dead in mid-air on an invisible floor. Two floors is a
    // worse failure than none, because none is obvious.
    // An explicit Z is deliberately different: it is an operator assertion that the mirrored
    // world lacks usable support (for example, a NoCollision Landscape plus solid park props).
    // Honour it even beside a non-empty mirror; project-specific settings carry the flat-floor
    // tradeoff rather than silently dropping the requested floor.
    if (has_explicit_ground_plane) {
        opts.add_ground_plane = true;
        opts.ground_plane_z = vehicle_setting_->urdf_ground_plane_z;
        UE_LOG(LogUrdfBot, Log,
               TEXT("explicit ground plane [%s]: enabled at world z=%.6f m alongside %d "
                    "mirrored shapes"),
               UTF8_TO_TCHAR(getVehicleName().c_str()), opts.ground_plane_z,
               static_cast<int32>(static_world_ ? static_world_->shapeCount() : 0));
        UAirBlueprintLib::LogMessageString(
            "UrdfBot: ground override ",
            Utils::stringf("%s - explicit scaffolding floor placed at z = %.3f m alongside %d "
                           "mirrored shapes.",
                           getVehicleName().c_str(), opts.ground_plane_z,
                           static_cast<int32>(static_world_ ? static_world_->shapeCount() : 0)),
            LogDebugLevel::Informational);
    }
    else if (suppress_automatic_ground_plane) {
        if (vehicle_setting_->urdf_ground_plane_auto) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot: ",
                getVehicleName() + " asked for a scaffolding ground plane, but the level mirrored "
                "successfully - the plane is suppressed. It would sit through the map.",
                LogDebugLevel::Informational);
        }
    }
    else if (vehicle_setting_->urdf_ground_plane_auto && coordinated_scene_ == nullptr) {
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
    // --- progress, because a cold cook freezes the editor for minutes -------------------------
    //
    // ⚠ This work is SYNCHRONOUS on the game thread. buildFromUrdf decomposes every <mesh>
    // collision inline, and on ExoMy that measured 312 s cold — five minutes of a black,
    // unresponsive editor with nothing on screen to say why. It looks exactly like a hang, and the
    // reasonable response to a hang is to kill the process, which throws away the cook and
    // guarantees it happens again next time.
    //
    // Counted up front rather than reported as a spinner: "4 of 17" tells you it is progressing
    // AND roughly how much is left, which a spinner does not. Only mesh collisions are counted
    // because only they decompose.
    size_t mesh_collisions = 0;
    for (const urdf::Link& l : model_.links)
        for (const urdf::Collision& c : l.collisions)
            if (c.geometry.type == urdf::GeometryType::Mesh) ++mesh_collisions;

    // ⚠ Sized to BOTH long jobs. Robot mesh decomposition and static-world emission are separate
    // stages with separate callbacks, and either can dominate: a Scout has 0 mesh collisions and
    // 172 level bodies, ExoMy has 17 mesh collisions and the same level. Counting only one of them
    // is how a five-minute load ended up with no dialog at all.
    const int static_bodies =
        static_cast<int>(static_world_ ? static_world_->bodies.size() : 0);
    const int progress_total = static_cast<int>(mesh_collisions) + static_bodies;

    TUniquePtr<FScopedSlowTask> cook_progress;
    if (progress_total > 0) {
        cook_progress = MakeUnique<FScopedSlowTask>(
            static_cast<float>(progress_total),
            FText::FromString(FString::Printf(
                TEXT("%s: building collision geometry (%d robot meshes, %d level bodies)"),
                UTF8_TO_TCHAR(getVehicleName().c_str()), static_cast<int32>(mesh_collisions),
                static_bodies)));
        // ⚠ bAllowInPIE MUST BE TRUE OR NOTHING IS EVER SHOWN, and its default is false. From
        // Misc/SlowTask.h: "Whether to allow this dialog in PIE. If false, this dialog will not
        // appear during PIE sessions." A urdfbot is built in ASimModeBase::BeginPlay, which is
        // PIE — so the first version of this code asked for a dialog that was suppressed by
        // design, and a 5-minute cook still looked like a hang.
        //
        // ⚠ Important, NOT the default visibility, AND THIS IS WHAT MAKES IT VISIBLE AT ALL.
        //
        // A urdfbot is built inside ASimModeBase::BeginPlay, which runs while PIE ALREADY HAS A
        // SLOW TASK OPEN ("Starting PIE (Begin Play)"). Two consequences, both measured:
        //   * MakeDialog below does nothing — it bails on `!GIsSlowTask` (SlowTask.cpp:235), since
        //     a dialog already exists. Our scope is pushed on the stack but never becomes the
        //     dialog.
        //   * FeedbackContextEditor.cpp:193-210 always shows scope 0 — PIE's message — and then
        //     scans the stack ONLY for scopes marked Important to add as secondary bars. A
        //     Default-visibility scope is second-class and, nested under PIE, showed nothing: the
        //     operator saw a dialog reading "Starting PIE (Begin Play)" for five minutes with no
        //     hint that a convex decomposition was the thing taking the time.
        //
        // Important gets our message its own bar underneath PIE's. The engine warns if such a scope
        // has an empty DefaultMessage; ours is the constructor argument above.
        cook_progress->Visibility = ESlowTaskVisibility::Important;

        // ⚠ IMMEDIATE, NOT MakeDialogDelayed, AND THE REASON IS NON-OBVIOUS. MakeDialogDelayed
        // creates nothing — it only records a threshold. The dialog is built lazily inside
        // EnterProgressFrame -> TickProgress -> MakeDialogIfNeeded (Core/Private/Misc/SlowTask.cpp),
        // so it can only ever appear ON A PROGRESS CALL. We call that once per mesh, and the first
        // call happens BEFORE any work, when elapsed time is still ~0 and below any threshold. The
        // second call comes ~20 s later, after the first mesh is already cooked. Net effect with a
        // 0.5 s threshold: the dialog is born one whole mesh late and the longest stall of the load
        // — the first one — is the one with nothing on screen. Measured; the operator saw it
        // "show for a bit".
        //
        // Creating it up front costs a dialog that appears and vanishes on a warm 65 ms load. That
        // is the lesser evil, and in practice it does not survive long enough to paint.
        //
        // ⚠ bAllowInPIE MUST be true — see above; its default suppresses the dialog entirely here.
        //
        // ⚠ bShowCancelButton stays false. Cancelling mid-cook would leave the robot with some
        // links decomposed and others as single hulls — a silently inconsistent collision model,
        // which is worse than waiting.
        cook_progress->MakeDialog(/*bShowCancelButton=*/false, /*bAllowInPIE=*/true);

        // ⚠ Paint it BEFORE the first mesh, with zero work claimed. EnterProgressFrame is the only
        // thing that pushes text and a redraw into the dialog, so without this the window exists
        // but shows the default message until the first mesh has already finished — which is the
        // exact 20-second blind spot this is all meant to remove.
        cook_progress->EnterProgressFrame(
            0.0f, FText::FromString(FString::Printf(
                      TEXT("Preparing %d items..."), progress_total)));
    }

    // ⚠ Counted here rather than left to the bar's fill fraction. A fraction tells you how far
    // along you are but not how much is left in units you can reason about; "mesh 4 of 17" does,
    // and when each mesh is ~20 s that is the difference between waiting and force-quitting.
    int32 cooked = 0;
    opts.decomposition.progress = [&cook_progress, &cooked,
                                   total = static_cast<int32>(mesh_collisions)](
                                      const std::string& what) {
        if (!cook_progress) return;
        ++cooked;
        cook_progress->EnterProgressFrame(
            1.0f, FText::FromString(FString::Printf(
                      TEXT("Convex decomposition %d/%d:  %s"), cooked, total,
                      UTF8_TO_TCHAR(what.c_str()))));
    };

    opts.build_progress = [&cook_progress](const std::string& stage, int done, int total) {
        if (!cook_progress) return;
        cook_progress->EnterProgressFrame(
            1.0f, FText::FromString(FString::Printf(TEXT("%s  %d/%d"),
                                                    UTF8_TO_TCHAR(stage.c_str()), done, total)));
    };

    backend->buildFromUrdf(model_, opts);
    cook_progress.Reset();
    backend_ = std::move(backend);

    // ⚠ IS SOLVER STATE AVAILABLE YET? Box3D's shared scene builds each robot into the live world
    // as it joins, so it is queryable immediately. MuJoCo's cannot: mjs_attach + mj_compile is a
    // batch operation and there are no bodies until the manifest commits. Everything below that
    // reads state is deferred to onSharedSceneReady() in that case.
    backend_state_available_ =
        (coordinated_scene_ == nullptr) ||
        (std::strcmp(backend_->backendName(), "Box3D") == 0);

    // Declare this robot to the coordinator as a body the shared scene owns and publishes state
    // for. Done here rather than at construction because the scene handle only exists once the
    // robot has actually been built into the shared world.
    if (coordinated_scene_ != nullptr) {
        // ⚠ backendName() is the discriminator, as everywhere else here: Unreal builds with RTTI
        // off, so dynamic_cast is unavailable and a static_cast to the wrong type is undefined
        // behaviour that never announces itself.
        const bool shared_is_box3d = (std::strcmp(backend_->backendName(), "Box3D") == 0);
        const bool shared_is_mujoco = (std::strcmp(backend_->backendName(), "MuJoCo") == 0);
#if WITH_BOX3D_BINDING
        if (shared_is_box3d) {
            coordinated_scene_->publishBox3DBody(
                ICoordinatedPhysicsScene::vehicleBodyId(getVehicleName()),
                static_cast<urdf::Box3DUrdfBackend*>(backend_.get())->sceneHandle());
        }
#endif
#if WITH_MUJOCO_BINDING
        if (shared_is_mujoco) {
            coordinated_scene_->publishMuJoCoBody(
                ICoordinatedPhysicsScene::vehicleBodyId(getVehicleName()),
                static_cast<urdf::MuJoCoSharedUrdfBackend*>(backend_.get())->articulation());
        }
#endif
    }

    // Cross-check the realised mass against the file. A mismatch means links were merged, dropped
    // or given shape-derived mass, and it is far cheaper to catch here than to explain later.
    if (backend_state_available_) {
        const double urdf_mass = model_.totalMass();
        const double realised = backend_->totalMass();
        if (urdf_mass > 0 && std::fabs(realised - urdf_mass) > 1e-4 * std::max(1.0, urdf_mass)) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: realised mass ",
                Utils::stringf("%.6f kg does not match the URDF's %.6f kg", realised, urdf_mass),
                LogDebugLevel::Failure);
        }
    }

    // ⚠ THE ONLY WAY TO ASK "which backend is this?" IN THIS CODEBASE. Unreal builds with RTTI off,
    // so dynamic_cast is unavailable and a static_cast to the wrong type is undefined behaviour
    // that will not announce itself. backendName() is a pure virtual on UrdfRobotBackend and is the
    // discriminator; the same gap on VehicleApiBase is what lets a wrong-family RPC call SIGSEGV
    // the editor, so it is worth not repeating here.
    const bool is_box3d = (std::strcmp(backend_->backendName(), "Box3D") == 0);
    // ⚠ NOT `!is_box3d`. See the note at the static-world mirror above: a third backend exists now
    // and "not Box3D" no longer implies MuJoCo. The two blocks below static_cast on this.
    const bool is_mujoco = (std::strcmp(backend_->backendName(), "MuJoCo") == 0);

    // Two things the BOX3D backend can only discover while building, both of which change what is
    // actually simulated and neither of which is visible in the render. Both are properties of
    // Box3D's hull builder and mass handling, so there is nothing to report for another engine.
    if (is_box3d) {
#if WITH_BOX3D_BINDING
        const b3urdf::Box3DRobot& robot =
            static_cast<urdf::Box3DUrdfBackend*>(backend_.get())->robot();

        // ⚠ WHAT THE LEVEL BECAME IN BOX3D, which until now was never reported at all. The
        // mirror's own report says how many components it read; it says nothing about whether
        // b3CreateMesh accepted any of them. Between those two numbers sits a failure that looks
        // exactly like working software: the load log proudly prints "357 bodies, 7.2 M triangles"
        // and the robot drives through every one of them. Box3DStaticGeometry has counted this
        // since it was written - the header even says the rejects are "reported, never swallowed" -
        // and nothing ever printed them.
        if (const b3urdf::Box3DStaticGeometry* geom = robot.staticGeometry()) {
            UE_LOG(LogUrdfBot, Log,
                   TEXT("Box3D static cook [%s]: %d tri-meshes cooked, %d shapes REJECTED, "
                        "%d triangles, %.1f ms"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()),
                   static_cast<int32>(geom->cookedMeshCount()),
                   static_cast<int32>(geom->rejectedShapeCount()),
                   static_cast<int32>(geom->triangleCount()), geom->cookMilliseconds());

            if (geom->rejectedShapeCount() > 0) {
                UAirBlueprintLib::LogMessageString(
                    "UrdfBot WARNING: level shapes rejected by Box3D ",
                    Utils::stringf("%d of the mirrored shapes did not cook. Those objects are "
                                   "visible and NOT solid.",
                                   static_cast<int>(geom->rejectedShapeCount())),
                    LogDebugLevel::Failure);
            }

            // ⚠ THE SCAFFOLDING FLOOR IS OPAQUE TO EVERYTHING UNDER IT. The slab is 100 x 100 m
            // and 1 m thick, so an explicit UrdfGroundPlaneZ set above the real terrain does not
            // merely add a floor - it ENTOMBS every mirrored prop whose top is below it. A bench
            // is ~0.45 m tall; a floor at 0.70 m swallows it whole, and the robot drives through
            // benches that mirrored, cooked and attached perfectly. Counted rather than reasoned
            // about, because "the mirror worked" and "the robot collides" are different claims.
            if (opts.add_ground_plane && static_world_) {
                const auto rotate = [](const urdf::Quat& q, const urdf::Vec3& p) {
                    const double x = q.x, y = q.y, z = q.z, w = q.w;
                    return urdf::Vec3{
                        p.x*(1-2*(y*y+z*z)) + p.y*(2*(x*y-w*z))   + p.z*(2*(x*z+w*y)),
                        p.x*(2*(x*y+w*z))   + p.y*(1-2*(x*x+z*z)) + p.z*(2*(y*z-w*x)),
                        p.x*(2*(x*z-w*y))   + p.y*(2*(y*z+w*x))   + p.z*(1-2*(x*x+y*y)) };
                };
                int32 buried = 0;
                std::string first;
                for (const urdf::StaticBody& b : static_world_->bodies) {
                    double top = -1e30;
                    for (const urdf::StaticShape& sh : b.shapes)
                        for (const urdf::Vec3& pt : sh.points)
                            top = std::max(top, b.position.z + rotate(b.orientation, pt).z);
                    if (top > -1e29 && top < opts.ground_plane_z) {
                        ++buried;
                        if (first.empty()) first = b.name;
                    }
                }
                if (buried > 0) {
                    UAirBlueprintLib::LogMessageString(
                        "UrdfBot WARNING: props buried under the scaffolding floor ",
                        Utils::stringf("%d mirrored bodies lie ENTIRELY below the %.2f m plane and "
                                       "can never be touched (e.g. %s). Lower the plane, or drop "
                                       "UrdfGroundPlaneZ once the terrain itself mirrors.",
                                       buried, opts.ground_plane_z, first.c_str()),
                        LogDebugLevel::Failure);
                    UE_LOG(LogUrdfBot, Warning,
                           TEXT("scaffolding floor at z=%.3f m buries %d of %d mirrored bodies "
                                "(first: %s)"),
                           opts.ground_plane_z, buried,
                           static_cast<int32>(static_world_->bodies.size()),
                           UTF8_TO_TCHAR(first.c_str()));
                }
            }
        }

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

        // ⚠ The fidelity WIN and the fidelity LOSS, both reported, because both change what is
        // simulated and neither is visible in the render.
        for (const auto& d : robot.decompositions()) {
            UE_LOG(LogUrdfBot, Log,
                   TEXT("link '%s': mesh decomposed into %d convex parts (%.1f s%s)"),
                   UTF8_TO_TCHAR(d.link.c_str()), static_cast<int32>(d.parts), d.seconds,
                   d.from_cache ? TEXT(", from cache") : TEXT(""));
        }
        if (robot.degenerateParts() > 0) {
            // Normal in small numbers — CoACD emits slivers. Logged because it is also the count
            // that used to reach b3CreateHull and take the editor down with SIGSEGV.
            UE_LOG(LogUrdfBot, Log,
                   TEXT("%d convex parts had no volume to hull and were dropped"),
                   static_cast<int32>(robot.degenerateParts()));
        }

        if (!robot.decompositionFallbacks().empty()) {
            std::string list;
            for (const auto& f : robot.decompositionFallbacks())
                list += (list.empty() ? "" : "\n  ") + f.link + " - " + f.note;
            // Failure level, not informational: every entry is a concave link being simulated as a
            // solid block. It looks completely correct until something will not fit through a gap.
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: links still simulated as ONE convex hull ",
                Utils::stringf("(%d) - concave shapes are FATTER THAN THEY LOOK",
                               static_cast<int>(robot.decompositionFallbacks().size())),
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Warning, TEXT("no convex decomposition for:\n  %s"),
                   UTF8_TO_TCHAR(list.c_str()));
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
#endif
    }

#if WITH_MUJOCO_BINDING
    // ⚠ REPORT WHAT THE LEVEL ACTUALLY BECAME. mirrorsStaticWorld() being true SUPPRESSES the
    // scaffolding ground plane, so if these geoms silently fail to appear the robot is left with
    // no world at all and falls forever — which is exactly what happened the first time this ran.
    // The counters existed before this log did, which is why the failure was invisible.
    // ⚠ LEGACY BACKEND ONLY. In a coordinated run this vehicle holds a MuJoCoSharedUrdfBackend,
    // which is a DIFFERENT class - static_cast-ing it to MuJoCoUrdfBackend is undefined behaviour
    // that will not announce itself, because Unreal builds with RTTI off and backendName() returns
    // "MuJoCo" for both. The shared scene reports its own cook through StaticWorldEmitStats.
    if (is_mujoco && coordinated_scene_ == nullptr) {
        const urdf::MuJoCoUrdfBackend* mjw =
            static_cast<const urdf::MuJoCoUrdfBackend*>(backend_.get());
        const size_t emitted = mjw->staticGeomsEmitted();
        const size_t dropped = mjw->staticShapesDropped();

        double gmin[3], gmax[3];
        mjw->staticGeomBounds(gmin, gmax);
        const urdf::Vec3 spawn_m = opts.root_position;

        // ⚠ Compiled count AND bounds AND the spawn, on one line, because the three failures look
        // identical from the outside: nothing compiled, geometry compiled somewhere else, or
        // geometry compiled correctly and the robot spawned outside it.
        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo static world: %d spec geoms -> %d COMPILED world geoms, %d dropped "
                    "(from %d mirrored shapes)"),
               static_cast<int32>(emitted), static_cast<int32>(mjw->staticGeomsCompiled()),
               static_cast<int32>(dropped),
               static_cast<int32>(static_world_ ? static_world_->shapeCount() : 0));
        if (mjw->staticWorstVertex() > 0) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: CORRUPT level geometry rejected ",
                Utils::stringf("a mirrored shape contained a vertex %.0f m from the origin. Left "
                               "in, it poisons every collision and ray query in the model.",
                               mjw->staticWorstVertex()),
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Error,
                   TEXT("rejected static geometry with a vertex %.1f m from the origin"),
                   mjw->staticWorstVertex());
        }
        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo largest mirrored shape: %.1f m across, from '%s'"),
               mjw->staticWorstSpan(), UTF8_TO_TCHAR(mjw->staticWorstSpanBody().c_str()));
        // ⚠ Two lines were removed here, not reworded. They reported "N vast flat shapes converted
        // to infinite planes" and "N shapes skipped as oversize" from counters that NO CODE EVER
        // ASSIGNED, so both printed 0 whatever the level contained - a log that described a
        // feature the emitter does not implement. What actually keeps a 40 km ground mesh out of
        // the broadphase is the region clipping, which is real and is reported below.
        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo static world: ground represented by %s; %d triangles clipped away "
                    "outside the %.1f m region"),
               mjw->usedHeightField() ? TEXT("a sampled HEIGHT FIELD")
                                      : TEXT("a flat traced plane"),
               static_cast<int32>(mjw->staticTrianglesClippedAway()),
               vehicle_setting_->urdf_static_world_radius);
        // ⚠ Geom count is the cost of the per-triangle representation, and it is the number to
        // watch if stepping gets slow. Exactness is bought with geoms rather than with cook time.
        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo static world: %d convex objects taken whole, %d triangles from "
                    "concave ones as thin prisms (exact surface, no decomposition)"),
               static_cast<int32>(mjw->staticConvexObjects()),
               static_cast<int32>(mjw->staticTrianglesEmitted()));
        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo static world: %d inward-facing enclosures detected (always split - a "
                    "room taken whole becomes a solid block around the robot)"),
               static_cast<int32>(mjw->staticEnclosures()));
        if (mjw->staticTrianglesSkipped() > 0) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: level geometry TRUNCATED ",
                Utils::stringf("%d triangles dropped at the %d cap - that geometry is NOT THERE "
                               "for this robot. Reduce UrdfStaticWorldRadius or raise "
                               "UrdfStaticWorldMaxTriangles.",
                               static_cast<int>(mjw->staticTrianglesSkipped()),
                               vehicle_setting_->urdf_static_world_max_triangles),
                LogDebugLevel::Failure);
        }
        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo static world EXTENT (pos+/-rbound): x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]   "
                    "robot spawns at (%.2f %.2f %.2f)"),
               gmin[0], gmax[0], gmin[1], gmax[1], gmin[2], gmax[2],
               spawn_m.x, spawn_m.y, spawn_m.z);

        UE_LOG(LogUrdfBot, Log,
               TEXT("MuJoCo contact masks: world contype=%d conaffinity=%d | robot contype=%d "
                    "conaffinity=%d  (a pair collides only if type1&aff2 or type2&aff1)"),
               mjw->staticContype(), mjw->staticConaffinity(), mjw->robotContype(),
               mjw->robotConaffinity());

        // The decisive one: is there anything under the robot at all?
        if (mjw->groundProbeDistance() < 0) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: NOTHING beneath ",
                getVehicleName() + " - a downward ray from its spawn point hits no static geometry, "
                "so it will fall forever despite the level having compiled.",
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Error,
                   TEXT("MuJoCo ground probe: NO HIT below the spawn point (upward ray: %.3f m - "
                        "a positive value here means the robot spawned UNDER the level)"),
                   mjw->ceilingProbeDistance());
        }
        else {
            UE_LOG(LogUrdfBot, Log,
                   TEXT("MuJoCo ground probe: %.3f m below the spawn, geom %d"),
                   mjw->groundProbeDistance(), mjw->groundProbeGeom());
        }

        if (have_static_world && mjw->staticGeomsCompiled() == 0) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: MuJoCo emitted NO static geometry ",
                getVehicleName() + " mirrored a level but produced no collision geometry from it, "
                "and the scaffolding ground plane was suppressed because the engine claims to "
                "mirror. This robot has NOTHING to stand on and will fall forever.",
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Error,
                   TEXT("'%s': static world mirrored (%d shapes) but 0 geoms emitted - the robot "
                        "has no world"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()),
                   static_cast<int32>(static_world_ ? static_world_->shapeCount() : 0));
        }
    }

    // The MuJoCo equivalent, and it exists for the same reason as the R2 audit: a robot whose
    // collision geometry quietly did not load looks completely normal and collides with nothing.
    // MuJoCo drops what it cannot read WITHOUT reporting it — package:// URIs (it has no notion of
    // ROS packages) and COLLADA meshes (it reads STL, OBJ and MSH) — which between them covers most
    // robot URDFs. Our own loader handles both, so a model that works perfectly on Box3D can arrive
    // here hollow.
    // ⚠ LEGACY BACKEND ONLY. In a coordinated run this vehicle holds a MuJoCoSharedUrdfBackend,
    // which is a DIFFERENT class - static_cast-ing it to MuJoCoUrdfBackend is undefined behaviour
    // that will not announce itself, because Unreal builds with RTTI off and backendName() returns
    // "MuJoCo" for both. The shared scene reports its own cook through StaticWorldEmitStats.
    if (is_mujoco && coordinated_scene_ == nullptr) {
        const urdf::MuJoCoUrdfBackend* mj =
            static_cast<const urdf::MuJoCoUrdfBackend*>(backend_.get());
        const size_t declared = mj->collisionGeomsDeclared();
        const size_t realised = mj->collisionGeomsRealised();

        UE_LOG(LogUrdfBot, Log, TEXT("MuJoCo collision geometry: %d of %d <collision> elements"),
               static_cast<int32>(realised), static_cast<int32>(declared));

        if (!mj->droppedCollisions().empty()) {
            std::string list;
            for (const std::string& d : mj->droppedCollisions())
                list += (list.empty() ? "" : "\n  ") + d;
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: MuJoCo dropped collision geometry ",
                Utils::stringf("- %d of %d <collision> elements compiled. The mesh collisions "
                               "below did not load; MuJoCo cannot resolve package:// and cannot "
                               "read .dae. Those links collide with NOTHING.",
                               static_cast<int>(realised), static_cast<int>(declared)),
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Error,
                   TEXT("MuJoCo dropped mesh collisions (package:// unsupported, .dae unreadable) "
                        "for '%s':\n  %s"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()), UTF8_TO_TCHAR(list.c_str()));
        }
    }
#endif

    auto* pawn = static_cast<AUrdfBotPawn*>(getPawn());
    // ⚠ Custom-depth stencils are OFF by default and are NOT how Segmentation works here.
    //
    // Setting CustomDepthStencilValue on a URDF robot's meshes does not make it appear in
    // Segmentation: this fork drives Segmentation from the annotation system (a ShowOnlyComponents
    // whitelist of UAnnotationComponents), not from raw stencils. That path now works for
    // procedural meshes — see FProceduralAnnotationSceneProxy — so nothing here is needed for it.
    //
    // The setting is kept for an operator who wants a stencil for their own post-processing. It
    // stays off by default because enabling custom depth has a real cost beyond being unnecessary:
    // it puts the link meshes back into screen-space passes.
    const int seg_id = vehicle_setting_->urdf_segmentation_id;
    if (seg_id >= 0) {
        pawn->setSegmentationId(seg_id);
        UE_LOG(LogUrdfBot, Log, TEXT("custom-depth stencil %d for '%s' (does NOT drive AirSim "
                                     "Segmentation - see UrdfMeshAssetDir)"),
               seg_id, UTF8_TO_TCHAR(getVehicleName().c_str()));
    }

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
                         vehicle_setting_->urdf_mesh_asset_scale,
                         vehicle_setting_->urdf_mesh_runtime_static,
                         vehicle_setting_->urdf_mesh_tint,
                         vehicle_setting_->urdf_mesh_material_override,
                         vehicle_setting_->urdf_mesh_roughness,
                         vehicle_setting_->urdf_mesh_srgb_colors);
    UE_LOG(LogUrdfBot, Log,
           TEXT("mesh shading [%s]: cast_shadow=%d contact_shadow=%d inset_shadow=%d "
                "two_sided_shadow=%d smooth_normals=%d flip_winding=%d base_material=%d "
                "runtime_static=%d tint=%d material_override=%d roughness=%.2f srgb=%d"),
           UTF8_TO_TCHAR(getVehicleName().c_str()),
           vehicle_setting_->urdf_mesh_cast_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_contact_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_inset_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_two_sided_shadow ? 1 : 0,
           vehicle_setting_->urdf_mesh_smooth_normals ? 1 : 0,
           vehicle_setting_->urdf_mesh_flip_winding ? 1 : 0,
           vehicle_setting_->urdf_mesh_base_material ? 1 : 0,
           vehicle_setting_->urdf_mesh_runtime_static ? 1 : 0,
           vehicle_setting_->urdf_mesh_tint ? 1 : 0,
           vehicle_setting_->urdf_mesh_material_override ? 1 : 0,
           vehicle_setting_->urdf_mesh_roughness,
           vehicle_setting_->urdf_mesh_srgb_colors ? 1 : 0);
    pawn->buildFromModel(model_, dirOf(urdf_path), vehicle_setting_->urdf_mesh_search_paths);

    // Settled below, AFTER the render buffers exist — see settleAndPublish().
    snapshot_.resize(backend_->linkCount());
    render_poses_.resize(backend_->linkCount());

    settleAndPublish();

    // ⚠ FILL THEM, do not merely size them. resize() default-constructs every LinkPose to position
    // (0,0,0) with identity rotation, and those are what the pawn renders until the first physics
    // update arrives. The robot therefore appears LIMP — every link at identity — for the first
    // frames, which is exactly the "hangs for a moment before landing" symptom, and it survived the
    // settle fix because settling moves the SOLVER while the renderer was still reading defaults.
    if (backend_state_available_) {
        for (size_t i = 0; i < snapshot_.size(); ++i) snapshot_[i] = backend_->getLinkPose(i);
        render_poses_ = snapshot_;
    }

    // Borrowed from the backend, which outlives the api — both are members of this sim api and
    // the backend is declared first, so it is destroyed last.
    //
    // ⚠ Box3D-only, and EMPTY rather than null for anything else. <mimic> classification is
    // currently a Box3DRobot product, not part of the UrdfRobotBackend interface, so a MuJoCo robot
    // has none to report. UrdfBotApi dereferences this pointer unconditionally when answering
    // getJoints(), so the honest answer is an empty list: a MuJoCo robot's joints come back with no
    // mimic_role rather than crashing. Hoisting classification onto the interface is the real fix
    // and is listed in NEXT-SESSION.md.
    static const std::vector<urdf::MimicClassification> no_mimic;
    const std::vector<urdf::MimicClassification>* mimic = &no_mimic;
#if WITH_BOX3D_BINDING
    if (is_box3d)
        mimic = &static_cast<urdf::Box3DUrdfBackend*>(backend_.get())->robot().mimicClassifications();
#endif

    {
        auto api = std::make_unique<UrdfBotApi>(&model_, backend_.get(), mimic, &getNedTransform(),
                                                getGroundTruthEnvironment()->getHomeGeoPoint(),
                                                &backend_mutex_);
        // Hand over the raw URDF so clients that cannot see the file path can still obtain the
        // description — see UrdfBotApiBase::getUrdfXml.
        api->setUrdfXml(urdf_xml_raw_);
        api->setSimApi(this);
        if (urdf_xml_raw_.empty()) {
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("could not re-read '%s' for getUrdfXml; ROS clients will get an EMPTY "
                        "robot_description and no link TF"),
                   UTF8_TO_TCHAR(urdf_path.c_str()));
        }
        vehicle_api_ = std::move(api);
    }

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

    // ⚠ The vehicle api is NOT reset here. UpdatableObject enforces strict reset/update
    // alternation in BOTH directions: update() before any reset throws "reset() must be called
    // first", and two resets with no update between them throw "Multiple reset() calls detected".
    // The framework already calls resetImplementation() at startup, so resetting here as well hits
    // the second error. Reset lives in resetImplementation() alone.

    // ⚠ Per-link CAMERAS. Re-parented here, after buildFromModel, because PawnSimApi creates every
    // camera during PawnSimApi::initialize() — which runs BEFORE the URDF is parsed — and attaches
    // it to the pawn root. The link components do not exist yet at that point, so the mount cannot
    // be chosen when the camera is made; it has to be corrected once the robot exists.
    //
    // KeepRelativeTransform on purpose: the camera's X/Y/Z from settings is an offset from its
    // mount, so it must survive the re-parent. Re-attaching with KeepWorldTransform would silently
    // turn a "20 cm forward of the head" camera into "20 cm forward of wherever the head happened
    // to be at spawn", which is right at spawn and wrong the moment the head moves.
    for (const auto& cam_pair : vehicle_setting_->cameras) {
        const std::string& link = cam_pair.second.link;
        if (link.empty()) continue;  // root-mounted, which is PawnSimApi's default

        USceneComponent* mount = pawn->getLinkComponent(link);
        if (mount == nullptr) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: camera '",
                cam_pair.first + "' names link '" + link + "', which is not in " + model_.name +
                    " - left on the vehicle root",
                LogDebugLevel::Failure);
            UE_LOG(LogUrdfBot, Warning,
                   TEXT("camera '%s' names link '%s', which does not exist - left on the root"),
                   UTF8_TO_TCHAR(cam_pair.first.c_str()), UTF8_TO_TCHAR(link.c_str()));
            continue;
        }

        APIPCamera* camera = getCamera(cam_pair.first);
        if (camera == nullptr) continue;
        camera->AttachToComponent(mount, FAttachmentTransformRules::KeepRelativeTransform);
        UE_LOG(LogUrdfBot, Log, TEXT("camera '%s' mounted on link '%s'"),
               UTF8_TO_TCHAR(cam_pair.first.c_str()), UTF8_TO_TCHAR(link.c_str()));

        // ⚠ Do NOT add an "hide what the camera is inside" rule here. One was written and removed.
        //
        // The symptom that motivated it — a tan border on Scene plus a Depth image reading
        // 0.17-0.6 m across the WHOLE frame — is not occlusion at all. It is a STALE FIRST FRAME:
        // the first request of any ImageType on this camera returns a render target that has not
        // been captured yet. Measured with five consecutive DepthVis calls on 2026-08-17:
        //
        //     Rover call 0: min 0.079  max 0.7    med 0.351   <- cold, wrong
        //     Rover call 1: min 0.008  max 165.0  med 0.184   <- correct, and stable thereafter
        //     Car1  call 0: min 0.010  max 163.8  med 0.082   <- warm already, never wrong
        //
        // Car1's captures are activated during startup; a link-mounted camera's are not, which is
        // why only URDF robots show it. The giveaway that it was never geometry: the cold depth
        // image has the correct SCENE STRUCTURE (building, blocks, drone, horizon) at wrong values.
        //
        // A consumer must therefore discard the first capture from a URDF camera, or warm it once
        // at startup. Hiding link meshes treats a symptom that does not exist and costs real
        // visibility — a wrist camera that should see its own gripper would stop seeing it.
    }

    // ⚠ Instance segmentation is registered HERE, after buildFromModel, and the placement is the
    // whole point. This is the recurring shape of every URDF integration bug so far: anything
    // AirSim sets up in a startup pass misses a URDF robot, because at startup the robot does not
    // exist yet. The annotator walks the level once at BeginPlay, when this pawn has no link
    // components at all — the same reason sensors were never ticked and cameras were on the root.
    //
    // This call was previously removed, on the grounds that registering was worse than not
    // registering: it made simListInstanceSegmentationObjects report 23 Rover_* objects that
    // contributed ZERO pixels, because nothing could paint a procedural mesh. That was true, and it
    // is now fixed at the root — FProceduralAnnotationSceneProxy — so registration once again means
    // what it says.
    {
        ASimModeBase* sim_mode = nullptr;
        for (TActorIterator<ASimModeBase> it(pawn->GetWorld()); it; ++it) {
            sim_mode = *it;
            break;
        }
        if (sim_mode == nullptr) {
            // ⚠ At Error through UE_LOG, not through LogMessageString. An earlier revision used
            // UAirBlueprintLib::FindActor<ASimModeBase>(pawn, TEXT("")), which matches on name or
            // tag and so matches nothing for an empty string, and announced its failure on-screen
            // only — where it was never seen.
            UE_LOG(LogUrdfBot, Error,
                   TEXT("no ASimModeBase in the world; '%s' will be ABSENT from Segmentation"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()));
        }
        else {
            const bool ok = sim_mode->AddNewActorToInstanceSegmentation(pawn);

            // ⚠ Do NOT add ForceUpdateInstanceSegmentation() here. One was added on the theory that
            // the ShowOnlyComponents whitelist was missing these components; it is not.
            // PaintRGBComponent adds each new annotation component to annotation_component_list_
            // as it paints it, and AddNewActorToInstanceSegmentation then pushes that list to every
            // camera. The whitelist is correct; a robot still absent from Segmentation is absent
            // for some other reason, so look at the render path rather than re-adding this.
            UE_LOG(LogUrdfBot, Log, TEXT("instance segmentation registration for '%s': %s"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()),
                   ok ? TEXT("ok") : TEXT("FAILED - robot will be absent from Segmentation"));
        }
    }

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

    // ⚠ NOT gated on d.enabled. The mapping is built whenever the operator declared DriveJoints or
    // SteerJoints; `enabled` gates the KEYBOARD only, in applyDriveInput.
    //
    // These were one flag, and the conflation had teeth: a multi-robot scene must disable the
    // keyboard on all but one robot (three vehicles bound to the same keys drive in lockstep, which
    // during a contact test is indistinguishable from the robots pushing each other) — and doing so
    // silently disabled RPC drive on those robots too. Measured 2026-08-17: with three rovers all
    // commanded identically over RPC, one moved 2.82 m and the other two moved 0.00 m, every call
    // returning success. Keyboard ownership and "can be driven at all" are different questions.
    if (d.drive_joints.empty() && d.steer_joints.empty()) return;

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

    // ⚠ Skid-steer entries are MERGED into drive_joints_, not kept in a list of their own.
    //
    // A wheel that both drives and steers has ONE velocity target, and it is the sum of the two
    // contributions. Two independent loops writing setJointTarget on the same joint would leave
    // whichever ran last, so steering would quietly replace throttle instead of adding to it — the
    // robot would spin on the spot the instant you touched the steering key and lose all forward
    // speed, which reads as a physics problem rather than a bookkeeping one.
    //
    // A joint named ONLY in SkidSteerJoints is legitimate (a wheel that contributes to turning but
    // not to forward drive), so it is appended with multiplier 0.
    for (const auto& kv : d.skid_steer_joints) {
        const int j = backend_->findJoint(kv.first);
        if (j < 0) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: UrdfDrive names skid-steer joint '",
                kv.first + "', which is not in " + model_.name + " - ignored",
                LogDebugLevel::Failure);
            continue;
        }
        auto it = std::find_if(
            drive_joints_.begin(), drive_joints_.end(),
            [&](const DriveMapping& m) { return m.joint == static_cast<size_t>(j); });
        if (it != drive_joints_.end())
            it->skid = kv.second;
        else
            drive_joints_.push_back(DriveMapping{ static_cast<size_t>(j), 0.0, kv.second });
    }

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
        Utils::stringf("%d drive joint(s), %d steer joint(s), %d skid-steer. W/S or Up/Down = "
                       "throttle, A/D or Left/Right or Numpad4/6 = steer, Space = stop.",
                       static_cast<int>(drive_joints_.size()),
                       static_cast<int>(steer_joints_.size()),
                       static_cast<int>(d.skid_steer_joints.size())),
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

    // ⚠ API control wins, and the keyboard stands down completely.
    //
    // This loop writes EVERY drive and steer joint every physics step. Without this check an RPC
    // command is overwritten 3 ms after it is issued — the call succeeds, returns cleanly, and the
    // robot does not move. That is the worst shape of failure: a working API that silently does
    // nothing, with the evidence 3 ms in the past.
    //
    // enableApiControl is the arbiter AirSim already uses for exactly this on cars and drones, so
    // a client that knows those needs no new concept. api_control_enabled_ starts FALSE, so the
    // keyboard keeps working until something asks for control.
    const bool api_control = (vehicle_api_ != nullptr && vehicle_api_->isApiControlEnabled());

    // ⚠ Under API control the drive loop stands down COMPLETELY until a drive command is actually
    // issued. A client that commands individual joints (setJointPosition/Velocity/Effort) must not
    // have those targets overwritten every physics step — that is the silent-no-op failure this
    // gate exists to prevent, and setDriveCommand must not reintroduce it for joint-level clients.
    if (api_control && !vehicle_api_->hasDriveCommand()) return;

    // ONE mapping, TWO sources. The axes come from RPC under API control and from the keyboard
    // otherwise; everything below — the speed and angle limits, the per-joint multipliers, the
    // control modes — is shared. A second copy of this mapping in a client would drift from this
    // one, and the robot would steer differently depending on who was driving it.
    //
    // Relaxed loads: a one-frame-old throttle is indistinguishable from a key pressed one frame
    // later, so there is nothing to order against.
    float throttle, steering;
    if (api_control) {
        throttle = static_cast<float>(vehicle_api_->getDriveThrottle());
        steering = static_cast<float>(vehicle_api_->getDriveSteering());
    }
    else {
        // ⚠ UrdfDrive.Enabled gates the KEYBOARD, and only here. A robot with Enabled=false still
        // drives over RPC — that is how a multi-robot scene gives the keyboard to exactly one
        // vehicle while every vehicle stays commandable from ROS or Python.
        if (!vehicle_setting_->urdf_drive.enabled) return;

        const auto* pawn = static_cast<const AUrdfBotPawn*>(getPawn());
        const auto& in = pawn->getDriveInput();
        throttle = in.throttle.load(std::memory_order_relaxed);
        steering = in.steering.load(std::memory_order_relaxed);
    }

    const auto& d = vehicle_setting_->urdf_drive;
    // ⚠ ONE target per joint, carrying BOTH contributions. On a skid-steer robot the turn IS the
    // difference between the two sides' wheel speeds, so throttle and steering have to be summed
    // here rather than written separately — see the merge in setupDriveJoints.
    for (const DriveMapping& m : drive_joints_)
        backend_->setJointTarget(m.joint, urdf::ControlMode::Velocity,
                                 (throttle * m.multiplier + steering * m.skid) * d.max_wheel_speed);
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
        else {
            // Skid steer has no steering joint to report an angle for, so report the wheel the
            // steering axis actually reaches: its VELOCITY target and the velocity it reached. A
            // line showing target != 0 and actual == 0 is a physics or contact problem; a line
            // showing target == 0 means the axis or the multiplier is missing, not the solver.
            for (const DriveMapping& m : drive_joints_) {
                if (m.skid == 0.0) continue;
                target = (throttle * m.multiplier + steering * m.skid) * d.max_wheel_speed;
                actual = backend_->getJointState(m.joint).velocity;
                name = backend_->jointName(m.joint).c_str();
                break;
            }
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
    // ⚠ In COORDINATED mode this function no longer solves anything. The shared world is advanced
    // once per tick by its scene participant, and this robot's command latch and state publication
    // are its pre/post-solve hooks — so doing any of it here as well would apply commands twice and
    // publish a state that is half a tick old.
    if (isCoordinated()) {
        PawnSimApi::update(delta);
        return;
    }

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

        // ⚠ Tick the vehicle api, which is what drives the SENSORS. Every other vehicle type
        // reaches this through its own chain (CarPawnSimApi -> CarPawnApi -> CarApiBase), and the
        // urdfbot had no equivalent — so its IMU and LiDAR were built, mounted and never updated.
        // They answered every RPC call with their initial sample, which looks exactly like a
        // stationary robot.
        //
        // Called after the step so the sample describes the state the step just produced: the
        // whole point of a urdfbot over a Chaos vehicle is that kinematics and sensor stamps are
        // computed in the same executor iteration (§6.0b), and doing this before the step would
        // throw that away.
        if (vehicle_api_ != nullptr) vehicle_api_->update(delta);

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

void UrdfBotSimApi::onSharedSceneReady() noexcept
{
    // ⚠ noexcept. This runs from the coordinator's manifest-FINALIZE phase, which by contract
    // cannot turn an already-published transaction into a failure. A diagnostic that threw here
    // would abort a scenario whose physics is already committed and correct.
    try {
        if (!built_ || backend_state_available_) return;
        backend_state_available_ = true;

        const double urdf_mass = model_.totalMass();
        const double realised = backend_->totalMass();
        if (urdf_mass > 0 && std::fabs(realised - urdf_mass) > 1e-4 * std::max(1.0, urdf_mass)) {
            UAirBlueprintLib::LogMessageString(
                "UrdfBot WARNING: realised mass ",
                Utils::stringf("%.6f kg does not match the URDF's %.6f kg", realised, urdf_mass),
                LogDebugLevel::Failure);
        }
        UE_LOG(LogUrdfBot, Log, TEXT("'%s': shared scene ready, realised mass %.6f kg"),
               UTF8_TO_TCHAR(getVehicleName().c_str()), realised);

        // ⚠ THE DIAGNOSTIC THAT WAS MISSING. The legacy backend reports dropped collision meshes
        // loudly; guarding its class-specific block for the shared path removed that, and the very
        // next run had two robots fall through the floor with every other number healthy. A robot
        // with zero collision geoms collides with NOTHING - it must never be a quiet condition.
#if WITH_MUJOCO_BINDING
        if (std::strcmp(backend_->backendName(), "MuJoCo") == 0 && coordinated_scene_ != nullptr) {
            const auto* shared = static_cast<const urdf::MuJoCoSharedUrdfBackend*>(backend_.get());
            const urdf::MuJoCoPhysicsScene& scene = coordinated_scene_->mujocoScene();
            const size_t geoms = scene.collisionGeomsRealised(shared->articulation());
            UE_LOG(LogUrdfBot, Log, TEXT("'%s': MuJoCo realised %d collision geoms"),
                   UTF8_TO_TCHAR(getVehicleName().c_str()), static_cast<int32>(geoms));

            if (!scene.unresolvedMeshes().empty()) {
                std::string list;
                for (const std::string& mesh : scene.unresolvedMeshes())
                    list += (list.empty() ? "" : "\n  ") + mesh;
                UAirBlueprintLib::LogMessageString(
                    "UrdfBot WARNING: MuJoCo could not resolve mesh files ",
                    Utils::stringf("%d reference(s) did not resolve to a readable file. Those "
                                   "shapes are NOT in the physics world.",
                                   static_cast<int>(scene.unresolvedMeshes().size())),
                    LogDebugLevel::Failure);
                UE_LOG(LogUrdfBot, Error, TEXT("unresolved mesh references:\n  %s"),
                       UTF8_TO_TCHAR(list.c_str()));
            }
            if (geoms == 0) {
                UAirBlueprintLib::LogMessageString(
                    "UrdfBot WARNING: NO COLLISION ",
                    getVehicleName() + " realised ZERO collision geoms in MuJoCo. It will fall "
                    "through the floor and pass through everything.",
                    LogDebugLevel::Failure);
                UE_LOG(LogUrdfBot, Error,
                       TEXT("'%s': ZERO collision geoms - the robot has no collision at all"),
                       UTF8_TO_TCHAR(getVehicleName().c_str()));
            }
        }
#endif

        // Fill the render buffers, or the robot is drawn LIMP - every link at identity - until the
        // first physics update arrives.
        if (snapshot_.size() != backend_->linkCount()) snapshot_.resize(backend_->linkCount());
        if (render_poses_.size() != backend_->linkCount())
            render_poses_.resize(backend_->linkCount());
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            for (size_t i = 0; i < snapshot_.size(); ++i) snapshot_[i] = backend_->getLinkPose(i);
        }
        render_poses_ = snapshot_;
    }
    catch (const std::exception& ex) {
        UE_LOG(LogUrdfBot, Error, TEXT("'%s': shared-scene finalize failed: %s"),
               UTF8_TO_TCHAR(getVehicleName().c_str()), UTF8_TO_TCHAR(ex.what()));
    }
    catch (...) {
        UE_LOG(LogUrdfBot, Error, TEXT("'%s': shared-scene finalize failed"),
               UTF8_TO_TCHAR(getVehicleName().c_str()));
    }
}

void UrdfBotSimApi::coordinatedPreSolve()
{
    if (!built_) return;

    // Latched before the shared world advances, so the command and the step it affects belong to
    // the same iteration — the same order the private-world update() had inside one call.
    applyDriveInput();
}

void UrdfBotSimApi::coordinatedPostSolve(double dt)
{
    if (!built_) return;

    // ⚠ After the solve, so the sample describes the state the step just produced. The whole point
    // of a urdfbot over a Chaos vehicle is that kinematics and sensor stamps are computed in the
    // same executor iteration (§6.0b).
    if (vehicle_api_ != nullptr) vehicle_api_->update(static_cast<float>(dt));

    const size_t n = backend_->linkCount();
    std::vector<urdf::LinkPose> fresh(n);
    for (size_t i = 0; i < n; ++i) fresh[i] = backend_->getLinkPose(i);
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.swap(fresh);
    }
    ++steps_taken_;
}

// ---------------------------------------------------------------------------------------------
// GAME THREAD, under physics_world_->lock()
// ---------------------------------------------------------------------------------------------
void UrdfBotSimApi::refreshKinematicMirror()
{
    // ⚠ WHY THIS EXISTS. The level mirror is memoised per UWorld and is therefore built exactly
    // once, during the FIRST urdfbot's initialisation. At that moment no urdfbot link components
    // exist — not even the building robot's own — so no URDF geometry is ever in it. Measured
    // 2026-08-17 with three ExoMys: all three tracked the same 7 kinematic bodies (the husky and
    // the drone, which already existed) and none of the 69 URDF link components.
    //
    // That is the entire reason Box3D robots did not interact with each other while they DID
    // interact with Chaos and FastPhysics vehicles. It is a snapshot-timing bug, not the
    // one-directional-coupling limit of §6.0c stage 2, and not something a shared b3World is
    // needed to fix.
    //
    // Re-collecting here, on the game thread with physics paused, once every pawn exists, is the
    // whole fix. The STATIC half is deliberately untouched: its shared_ptr identity is what makes
    // the cook shared between robots, so it stays memoised.
    if (!vehicle_setting_->urdf_mirror_world_geometry) return;
    if (!vehicle_setting_->urdf_mirror_other_vehicles && !vehicle_setting_->urdf_mirror_movable)
        return;

    // ⚠ The SAME capability gate as the initial registration in loadModelAndBackend, and it has to
    // be here too — this is a second, independent path to addKinematicBody. Without it a MuJoCo
    // robot logged "kinematic mirror refresh: +6 bodies (now 6) ... now solid to this one" while
    // every handle was -1 and nothing was solid to anything. Observed 2026-08-19.
    if (!backend_->mirrorsKinematicBodies()) return;

    // ⚠ THE SECOND PATH INTO THE SHARED WORLD. A robot that does not own the mirror must not
    // re-collect here either, or the shared world grows one kinematic copy of every moving prop
    // per robot — the same double-representation the initial registration guards against, arriving
    // later and looking like a physics bug rather than a bookkeeping one.
    if (!owns_kinematic_mirror_) return;

    urdf::UrdfRobotBackend* backend = backend_.get();

    UrdfWorldGeometry::FMirrorOptions opts;
    if (coordinated_scene_ != nullptr) {
        // One world policy, exactly as the initial mirror used.
        const AirSimSettings::StaticWorldMirrorSetting& world_mirror =
            AirSimSettings::singleton().physics_coordinator.static_world_mirror;
        opts.bIncludeMovable = world_mirror.include_movable;
        opts.bIncludeOtherVehicles = world_mirror.include_other_vehicles;
        for (const std::string& tag : world_mirror.required_tags)
            opts.RequiredTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));
        for (const std::string& tag : world_mirror.excluded_tags)
            opts.ExcludedTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));
    }
    else {
        opts.bIncludeMovable = vehicle_setting_->urdf_mirror_movable;
        opts.bIncludeOtherVehicles = vehicle_setting_->urdf_mirror_other_vehicles;
        for (const std::string& tag : vehicle_setting_->urdf_world_geometry_tags)
            opts.RequiredTags.Add(FName(UTF8_TO_TCHAR(tag.c_str())));
    }

    UrdfWorldGeometry::FMirrorStats stats;
    const UrdfWorldGeometry::FMirrorResult mirror = UrdfWorldGeometry::MirrorVehicles(
        getPawn()->GetWorld(), getNedTransform().fromNed(1.0f), opts, stats);
    if (!mirror.World) return;

    // Already-tracked sources, so a repeated pass adds only what is new. Dedup is by COMPONENT
    // identity rather than by name: two ExoMys have identically-named links and a name-keyed set
    // would silently mirror only the first one's.
    std::set<const UPrimitiveComponent*> tracked;
    for (const KinematicMirror& km : kinematic_mirrors_)
        tracked.insert(km.component.Get());

    int added = 0;
    for (size_t i = 0; i < mirror.World->kinematic.size(); ++i) {
        UPrimitiveComponent* src = (static_cast<int32>(i) < mirror.KinematicSources.Num())
                                       ? mirror.KinematicSources[static_cast<int32>(i)].Get()
                                       : nullptr;
        if (src == nullptr) continue;
        if (src->GetOwner() == getPawn()) continue;   // never mirror yourself
        if (tracked.count(src) != 0) continue;        // already registered

        KinematicMirror km;
        km.component = mirror.KinematicSources[static_cast<int32>(i)];
        km.handle = backend->addKinematicBody(mirror.World->kinematic[i]);
        kinematic_mirrors_.push_back(km);
        ++added;
    }

    if (added > 0) {
        UE_LOG(LogUrdfBot, Log,
               TEXT("kinematic mirror refresh for '%s': +%d bodies (now %d) - late-spawned "
                    "vehicles, including other URDF robots, are now solid to this one"),
               UTF8_TO_TCHAR(getVehicleName().c_str()), added,
               static_cast<int32>(kinematic_mirrors_.size()));
    }
}

bool UrdfBotSimApi::sampleGroundHeightField(const urdf::Vec3& centre, double half_extent,
                                            urdf::BackendOptions::HeightField& out)
{
    // ⚠ TRACES, NOT MIRRORED GEOMETRY. Unreal answers "how high is the ground here" exactly, for
    // landscape, static meshes and BSP alike, without any of it being mirrored or approximated as
    // convex. Deriving a floor from mirrored geometry failed repeatedly on Blocks, whose ground is
    // one 40 km mesh with a bounding box 442 m tall around a surface at z = 0.36.
    const int kSamples = 33;                    // 33x33 = 1089 traces, a few ms
    const float world_to_meters = getNedTransform().fromNed(1.0f);

    out = urdf::BackendOptions::HeightField();
    out.rows = kSamples;
    out.cols = kSamples;
    out.center_x = centre.x;
    out.center_y = centre.y;
    out.half_extent = half_extent;
    out.heights.assign(static_cast<size_t>(kSamples) * kSamples, 0.0f);

    FCollisionQueryParams params(FName(TEXT("UrdfGroundGrid")), /*bTraceComplex=*/true);
    params.AddIgnoredActor(getPawn());

    std::vector<double> z(static_cast<size_t>(kSamples) * kSamples,
                          std::numeric_limits<double>::quiet_NaN());
    double lo = std::numeric_limits<double>::max();
    int hits = 0;

    for (int r = 0; r < kSamples; ++r) {
        for (int c = 0; c < kSamples; ++c) {
            // ⚠ Row-major with r along +y and c along +x, matching MuJoCo's hfield indexing.
            // Transposing this yields terrain that is subtly rotated - much harder to notice than
            // terrain that is simply missing.
            const double wx = centre.x - half_extent + 2.0 * half_extent * c / (kSamples - 1);
            const double wy = centre.y - half_extent + 2.0 * half_extent * r / (kSamples - 1);

            const FVector start = UrdfTransform::toFVector(
                urdf::Vec3{ wx, wy, centre.z + 50.0 }, world_to_meters);
            const FVector end = UrdfTransform::toFVector(
                urdf::Vec3{ wx, wy, centre.z - 500.0 }, world_to_meters);

            FHitResult hit;
            if (getPawn()->GetWorld()->LineTraceSingleByChannel(hit, start, end, ECC_Visibility,
                                                                params)) {
                const double h = hit.ImpactPoint.Z / world_to_meters;
                z[static_cast<size_t>(r) * kSamples + c] = h;
                lo = std::min(lo, h);
                ++hits;
            }
        }
    }

    if (hits == 0) return false;

    // ⚠ Holes filled with the LOWEST sample rather than left at zero. A gap - a pit, or a trace
    // that escaped through a doorway - would otherwise become a spike at whatever z = 0 means in
    // this map, and the robot would trip over nothing.
    out.min_z = lo;
    for (size_t i = 0; i < z.size(); ++i)
        out.heights[i] = static_cast<float>((std::isnan(z[i]) ? lo : z[i]) - lo);
    return true;
}

void UrdfBotSimApi::refreshGroundHeightField()
{
    // ⚠ A CLIPPED WORLD MUST FOLLOW THE ROBOT OR IT IS A CLIFF. The patch is sampled once around
    // the spawn; drive past its edge and the ground simply ends. Re-centring when the robot passes
    // the halfway mark keeps a full half-patch of margin ahead of it at all times.
    //
    // ⚠ GAME THREAD ONLY - line traces are not safe from the physics thread. updateRenderedState
    // is the game-thread hook, which is why this is called from there and not from update().
    if (!has_height_field_ || !built_ || backend_ == nullptr) return;

#if WITH_MUJOCO_BINDING
    // ⚠ render_poses_, not the backend. This runs on the game thread; reading the backend here
    // would race the physics thread that is writing it. render_poses_ is the copy made for exactly
    // this purpose at the top of updateRenderedState.
    if (render_poses_.empty()) return;
    const urdf::LinkPose root = render_poses_[0];
    const double dx = root.position.x - height_field_centre_.x;
    const double dy = root.position.y - height_field_centre_.y;
    const double moved = std::sqrt(dx * dx + dy * dy);

    if (moved < 0.5 * height_field_half_extent_) return;

    urdf::BackendOptions::HeightField hf;
    if (!sampleGroundHeightField(root.position, height_field_half_extent_, hf)) return;

    // ⚠ EXPLICIT, for the same reason as every other cast in this file. has_height_field_ is set
    // from settings and says nothing about which backend is behind the pointer.
    if (std::strcmp(backend_->backendName(), "MuJoCo") != 0) return;
    auto* mj = static_cast<urdf::MuJoCoUrdfBackend*>(backend_.get());
    if (mj->updateGroundHeightField(hf)) {
        height_field_centre_ = root.position;
        UE_LOG(LogUrdfBot, Log,
               TEXT("ground height field re-centred on (%.1f %.1f) after %.1f m of travel"),
               root.position.x, root.position.y, moved);
    }
#endif
}

void UrdfBotSimApi::updateRenderedState(float dt)
{
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        render_poses_ = snapshot_;
    }

    // Game-thread only: line traces are not safe from the physics thread, and this is the hook
    // that runs there.
    refreshGroundHeightField();

    // Late-arriving vehicles (notably OTHER URDF robots) get picked up here, before their poses are
    // pushed, so a body registered this frame is driven from this frame.
    if (kinematic_refresh_frames_ > 0) {
        --kinematic_refresh_frames_;
        refreshKinematicMirror();
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

    // ⚠ Publish kinematics and the clock reading TOGETHER, immediately after the kinematics are
    // recomputed, and do it here rather than letting the RPC thread sample them itself.
    //
    // A URDF robot has no state struct carrying a simulator timestamp the way MultirotorState,
    // CarState and ComputerVisionState do, so before this a ROS client had nothing to stamp
    // odometry with and fell back to wall clock in a urdfbot-only scene. The failure was silent:
    // the poses were correct, only their times were, so the data merely failed to register against
    // every other stream.
    //
    // The pairing is what has to be right. PawnSimApi::updateKinematics has just run, so the
    // kinematics describe this frame and nothing else has advanced the clock — sampling both
    // anywhere else, or in two steps, reintroduces exactly the skew this removes.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        published_state_ = msr::airlib::UrdfBotApiBase::UrdfBotState(*getGroundTruthKinematics(),
                                                                     clock()->nowNanos());
    }
}

msr::airlib::UrdfBotApiBase::UrdfBotState UrdfBotSimApi::getUrdfBotState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return published_state_;
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

void UrdfBotSimApi::settleAndPublish()
{
    // ⚠ Never in coordinated mode, and not merely because the parser forces the interval to zero
    // there: settling steps the backend, and a shared-scene backend REFUSES to step alone. The
    // world-level pre-settle inside the global reset transaction is the coordinated equivalent.
    if (!backend_ || isCoordinated() || vehicle_setting_->urdf_settle_seconds <= 0) return;

    // ⚠ Stepped on the LOADING thread, before play, so the robot is already resting when the first
    // frame is drawn. Drive input is untouched and zero, so this settles under gravity and contact
    // only.
    const double settle = vehicle_setting_->urdf_settle_seconds;

    // ⚠ TRACE THE SETTLE, do not just report its endpoint. "root now at z=2.99" after a 0.5 s
    // settle is compatible with two completely different stories — the robot never fell, or it
    // fell and bounced back — and the endpoint cannot tell them apart. ExoMy does not move at all
    // here while a Scout in the same map falls 1.2 m, and that difference is the whole question.
    const urdf::LinkPose before = backend_->getLinkPose(0);
    int steps = 0;
    const int slices = 10;
    for (int i = 0; i < slices; ++i) {
        steps += backend_->step(settle / slices);
        const urdf::LinkPose p = backend_->getLinkPose(0);
        const urdf::Twist t = backend_->getLinkTwist(0);
        UE_LOG(LogUrdfBot, Log,
               TEXT("  settle %s t=%.2f  z=%8.3f  vz=%8.3f  (x %.2f y %.2f)"),
               UTF8_TO_TCHAR(getVehicleName().c_str()), settle * (i + 1) / slices,
               p.position.z, t.linear.z, p.position.x, p.position.y);
    }

    // ⚠ If it did not move, say why it could not have: a fixed base and a zero timestep are the two
    // ways a settle silently does nothing, and both look identical from the endpoint.
    const urdf::LinkPose after0 = backend_->getLinkPose(0);
    if (std::fabs(after0.position.z - before.position.z) < 1e-3) {
        UE_LOG(LogUrdfBot, Warning,
               TEXT("settle moved '%s' by less than 1 mm in %.2f s (%d steps). fixed_base=%d, "
                    "backend=%s - either it is already resting, or it is anchored, or it is not "
                    "being integrated."),
               UTF8_TO_TCHAR(getVehicleName().c_str()), settle, steps,
               vehicle_setting_->urdf_fixed_base ? 1 : 0,
               UTF8_TO_TCHAR(backend_->backendName()));
    }

    // Publish immediately: resize() default-constructs poses to identity, and those are what the
    // pawn renders until the first physics update. Without this the robot is drawn limp regardless
    // of how well it settled.
    if (snapshot_.size() != backend_->linkCount()) snapshot_.resize(backend_->linkCount());
    if (render_poses_.size() != backend_->linkCount()) render_poses_.resize(backend_->linkCount());
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        for (size_t i = 0; i < snapshot_.size(); ++i) snapshot_[i] = backend_->getLinkPose(i);
    }
    render_poses_ = snapshot_;

    const urdf::LinkPose after = backend_->getLinkPose(0);
    UE_LOG(LogUrdfBot, Log,
           TEXT("settled '%s' for %.2f s (%d steps) - root now at (%.2f %.2f %.2f)"),
           UTF8_TO_TCHAR(getVehicleName().c_str()), settle, steps,
           after.position.x, after.position.y, after.position.z);
}

void UrdfBotSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    // ⚠ Outside the built_ guard on purpose. If the framework resets before the model has finished
    // loading, a guarded reset would be skipped and the api would then be updated without ever
    // having been reset — which throws on the physics thread every step and freezes the robot at
    // its spawn pose with no error visible in the sim.
    if (vehicle_api_ != nullptr) vehicle_api_->reset();

    if (built_ && isCoordinated()) {
        // ⚠ The solver rebuild ALREADY HAPPENED. World::resetImplementation runs the coordinator's
        // global transaction — new epoch, every shared scene rebuilt from the frozen manifest, the
        // approved pre-settle — before any member is reset. Rebuilding here as well would rebuild
        // the shared world once per robot, and each rebuild would throw away the previous robots'
        // restored state.
        std::lock_guard<std::mutex> backend_lock(backend_mutex_);
        last_update_time_ = 0;
        steps_taken_ = 0;
        if (snapshot_.size() != backend_->linkCount()) snapshot_.resize(backend_->linkCount());
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            for (size_t i = 0; i < snapshot_.size(); ++i) snapshot_[i] = backend_->getLinkPose(i);
        }
        render_poses_ = snapshot_;
        return;
    }

    if (built_) {
        // ⚠ Rebuilds the Box3D world rather than rewriting poses. Box3D has no rollback
        // determinism — contact caches, warm-start impulses and island state survive a pose write,
        // so a pose-restoring reset diverges silently from a fresh build (§6.4). Measured at 0.05 ms.
        //
        // backend_mutex_ held across the rebuild AND the snapshot refill: an RPC accessor is now
        // blocked out for the ~0.05 ms this takes rather than reading a body mid-teardown. Without
        // this an RPC thread reading backend_ (getJointStates, getLinkPose, ...) while this runs
        // dereferences a body Box3D has already freed — confirmed from a crash dump, 2026-08-20.
        std::lock_guard<std::mutex> backend_lock(backend_mutex_);
        backend_->reset();
        last_update_time_ = 0;
        steps_taken_ = 0;
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            for (size_t i = 0; i < snapshot_.size(); ++i) snapshot_[i] = backend_->getLinkPose(i);
        }
    }

    // ⚠ RE-SETTLE. backend_->reset() rebuilds the world from scratch and puts the robot back at its
    // spawn pose — deliberately, because Box3D has no rollback determinism. But the framework calls
    // resetImplementation() AFTER initialize(), so a settle done only at load is discarded every
    // single time and the robot falls into the scene anyway. That is why settling appeared to do
    // nothing: it worked, and was then thrown away.
    //
    // Settling here as well makes "reset" mean "back to the resting initial state" rather than
    // "back to mid-air", which is what an initial condition should be.
    if (built_) settleAndPublish();
}

bool UrdfBotSimApi::sandRenderOffset(urdf::Vec3& out) const
{
    // ⚠ SAME LOCK AS EVERY OTHER BACKEND TOUCH, and for the same reason: resetImplementation()
    // destroys and rebuilds the backend, and this is read from the render path on the game thread.
    std::lock_guard<std::mutex> lock(backend_mutex_);
    if (backend_ == nullptr) return false;
    // ⚠ NAME THE BACKEND, never "not one of the others" - the negation form is what put a
    // NewtonSidecar pointer through a static_cast to MuJoCoUrdfBackend and crashed the editor.
    if (std::strcmp(backend_->backendName(), "NewtonSidecar") != 0) return false;
    return static_cast<const msr::airlib::mpm::NewtonSidecarUrdfBackend*>(backend_.get())
        ->frameOffset(out);
}

bool UrdfBotSimApi::applyLinkWrench(size_t link_index, const urdf::Wrench& wrench)
{
    // ⚠ SAME LOCK AS EVERY OTHER BACKEND TOUCH. resetImplementation() destroys and rebuilds every
    // solver body; pushing a wrench at a link index while that happens writes through a dangling
    // table. The MPM impulse path runs on the game thread and reset can come from an RPC thread, so
    // this is a real race rather than a theoretical one.
    std::lock_guard<std::mutex> lock(backend_mutex_);
    if (backend_ == nullptr || !backend_state_available_)
        return false;
    backend_->applyExternalWrench(link_index, wrench);
    return true;
}

bool UrdfBotSimApi::setLinkWorldCollision(size_t link_index, bool enabled)
{
    // ⚠ Same lock as applyLinkWrench, for the same reason: resetImplementation() rebuilds every
    // solver body, and this is called from the game thread while reset can come from an RPC thread.
    //
    // ⚠ THE RESULT IS LOAD-BEARING. A caller that believes rigid support was suspended when it was
    // not will read the rigid floor's reaction as the sand carrying the vehicle. Both the backend
    // and this seam return false rather than guessing.
    std::lock_guard<std::mutex> lock(backend_mutex_);
    if (backend_ == nullptr || !backend_state_available_)
        return false;
    return backend_->setLinkWorldCollision(link_index, enabled);
}

bool UrdfBotSimApi::describeColliders(urdf::PhysicsColliderSet& out) const
{
    // ⚠ Same lock as every other backend read. resetImplementation() destroys and rebuilds every
    // solver body; walking the link tables while that happens is a use-after-free.
    std::lock_guard<std::mutex> lock(backend_mutex_);
    if (backend_ == nullptr || !backend_state_available_)
        return false;
    if (!backend_->describeColliders(out))
        return false;

    // ⚠ Qualify with the VEHICLE. A private-world backend names its links from the URDF, and two
    // rovers of one model would otherwise offer the sidecar two colliders with the same stable id
    // — which its registry keys on, so they would silently become one.
    for (urdf::PhysicsColliderDescriptor& collider : out.colliders)
        collider.stable_id = getVehicleName() + "/" + collider.stable_id;
    return true;
}

bool UrdfBotSimApi::collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                          urdf::CollisionDebugSnapshot& out) const
{
    // ⚠ THE SAME LOCK EVERY OTHER BACKEND READ TAKES. resetImplementation() destroys and rebuilds
    // every solver body; walking the shape tables while that happens is a use-after-free, and a
    // debug view is not a good enough reason to be the one caller that skips the guard.
    std::lock_guard<std::mutex> lock(backend_mutex_);
    if (backend_ == nullptr || !backend_state_available_)
        return false;
    if (!backend_->collisionDebugGeometry(filter, out))
        return false;

    // Qualify every label with the VEHICLE, because the whole point of a two-backend comparison is
    // telling the two robots apart, and a private-world backend names its links from the URDF -
    // which is the same URDF in both rovers.
    for (urdf::CollisionShape& geom : out.geoms)
        if (!geom.is_world)
            geom.label = getVehicleName() + "/" + geom.label;
    return true;
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

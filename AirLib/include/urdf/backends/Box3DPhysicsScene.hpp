// One Box3D world containing every Box3D-backed robot in a coordinated physics scene.
//
// This is intentionally additive. Box3DRobot::build() and Box3DUrdfBackend still create and own
// one private world per robot; callers opt into shared ownership only by constructing this class.
#pragma once

#include "urdf/UrdfCollisionDebug.hpp"
#include "urdf/UrdfPhysicsDescriptor.hpp"
#include "urdf/backends/Box3DRobot.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace b3urdf {

/// Solver-wide settings. Per-robot BuildOptions still carry geometry, placement and control
/// choices; these four values come from the scene because a b3World can have only one of each.
struct Box3DSceneOptions {
    int substeps = 4;
    double fixed_timestep = 0.003;
    int worker_count = 1;
    double gravity_z = -9.81;

    /// One scaffolding floor for the whole scene, at the world origin rather than under any one
    /// robot. The per-robot slab could be centred on its own spawn precisely because each robot
    /// owned a private world; in a shared world N such slabs would be N overlapping floors at N
    /// different places. A shared scene therefore gets one, and it is wide enough to be a floor
    /// rather than a robot-sized pad.
    bool add_ground_plane = false;
    double ground_plane_z = 0.0;
    double ground_plane_half_extent = 200.0;
    double ground_friction = 0.9;
};

class Box3DPhysicsScene {
public:
    using RobotHandle = size_t;

    explicit Box3DPhysicsScene(const Box3DSceneOptions& options = {});
    ~Box3DPhysicsScene();

    Box3DPhysicsScene(const Box3DPhysicsScene&) = delete;
    Box3DPhysicsScene& operator=(const Box3DPhysicsScene&) = delete;
    Box3DPhysicsScene(Box3DPhysicsScene&&) = delete;
    Box3DPhysicsScene& operator=(Box3DPhysicsScene&&) = delete;

    /// Build a robot into the live shared world and return its stable scene handle.
    ///
    /// ⚠ IMMEDIATE, not deferred. A simulator assembles vehicles one at a time and each one must be
    /// queryable — mass cross-check, cook audit, first rendered pose — the moment it is created; a
    /// scene that produced nothing until the last vehicle joined would force every one of those
    /// consumers to be rewritten around a second initialisation phase. Assembly stays
    /// deterministic, because the order is the scenario's order.
    ///
    /// World-level fields in `options` (timestep, substeps, workers and gravity) are replaced by
    /// the scene values. `add_ground_plane` is rejected: shared-world infrastructure belongs to
    /// the scene, so use `Box3DSceneOptions::add_ground_plane` or `setStaticWorld` rather than
    /// silently creating one slab per robot.
    RobotHandle addRobot(const urdf::Robot& model, const BuildOptions& options = {});

    /// Cook and attach the one scene-wide static mirror.
    ///
    /// ⚠ Must precede the first robot. Reset rebuilds the world by attaching the static geometry
    /// and then cloning the robots, so setting it later would give the rebuilt world a different
    /// body creation order from the one the run started with.
    void setStaticWorld(std::shared_ptr<const urdf::StaticWorld> world);
    bool hasStaticWorld() const { return static_geometry_ != nullptr; }
    const Box3DStaticGeometry* staticGeometry() const { return static_geometry_.get(); }

    /// Freeze topology. After this no robot may join, which is what makes the reset baseline and
    /// the coordinator's manifest describe the same population.
    void seal();

    /// Rebuild the whole scene in a candidate world, then commit with no-throw state swaps. If any
    /// candidate robot fails to build, the old world and every old robot remain live and unchanged.
    void reset();

    /// Transactional form used by the world coordinator. `prepareReset` builds a complete
    /// candidate without touching the live world. `commitPreparedReset` swaps it live but retains
    /// the old world; a later participant failure can still call `abortPreparedReset` to swap back.
    /// Only `finalizePreparedReset` releases that rollback state after every participant commits.
    void prepareReset();
    void commitPreparedReset();
    void abortPreparedReset() noexcept;
    void finalizePreparedReset() noexcept;
    bool resetPrepared() const { return reset_candidate_ != nullptr; }
    bool resetCommitted() const { return reset_committed_; }

    /// Fixed-step accumulation owned once per world. Each consumed tick prepares every robot,
    /// invokes b3World_Step exactly once, then commits that tick to every robot.
    int step(double dt);
    void stepOnce();

    size_t robotCount() const { return robots_.size(); }
    Box3DRobot& robot(RobotHandle handle);
    const Box3DRobot& robot(RobotHandle handle) const;

    /// Every collision shape in the live world, at this instant, in solver frame.
    ///
    /// ⚠ MIXED PROVENANCE, and the snapshot says which is which. A robot's link shapes are read
    /// back out of Box3D (`b3Shape_GetHull` and friends), so they report what the solver kept
    /// after decomposition. The mirrored level is reported from the StaticWorld that was cooked,
    /// because Box3D exposes no enumeration of the bodies a cook created — it is `Submitted`, and
    /// labelled as such rather than passed off as a readback.
    void collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                urdf::CollisionDebugSnapshot& out) const;

    /// Describe ONE robot's links as registerable colliders (plan §11.1). Per robot, not per
    /// scene: a sidecar registers against a robot, and a whole-scene call would hand it every
    /// other robot's links as though they were this one's.
    ///
    /// ⚠ `stable_id` IS REQUIRED AND THE SCENE CANNOT SUPPLY IT. Box3D's `addRobot` takes a model
    /// and options, not an identity — unlike MuJoCo's `addArticulation`, which is handed a stable
    /// id. Qualifying by the URDF's own name instead produces DUPLICATE COLLIDER IDS the moment
    /// two robots share a model, which is the ordinary case: a sidecar keying its registry on
    /// those would silently merge two rovers into one. Caught by
    /// `coordinated_box3d_describes_links_as_registerable_colliders`.
    void describeColliders(RobotHandle handle, const std::string& stable_id,
                           urdf::PhysicsColliderSet& out) const;

    b3WorldId worldId() const { return world_; }
    int stepsTaken() const { return steps_taken_; }
    const Box3DSceneOptions& options() const { return options_; }
    bool isSealed() const { return sealed_; }

private:
    struct CandidateScene;

    struct Registration {
        urdf::Robot model;
        BuildOptions options;
    };

    b3WorldId createWorld() const;
    BuildOptions normalizeRobotOptions(const BuildOptions& options) const;
    void destroyLiveScene();

    Box3DSceneOptions options_;
    std::vector<Registration> registrations_;
    std::vector<std::unique_ptr<Box3DRobot>> robots_;
    std::shared_ptr<const Box3DStaticGeometry> static_geometry_;
    b3WorldId world_ = b3_nullWorldId;
    double accumulator_ = 0;
    int steps_taken_ = 0;
    bool sealed_ = false;
    std::unique_ptr<CandidateScene> reset_candidate_;
    bool reset_committed_ = false;
};

} // namespace b3urdf

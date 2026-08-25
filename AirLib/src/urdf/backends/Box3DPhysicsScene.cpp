#include "urdf/backends/Box3DPhysicsScene.hpp"

#include "urdf/backends/Box3DCollisionReadback.hpp"
#include "urdf/backends/Box3DMath.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace b3urdf {
/// Candidate ownership is deliberately local to build/reset. Destruction order is robots first,
/// world second: external-world Box3DRobots remove only their own bodies and must see a live world.
struct Box3DPhysicsScene::CandidateScene {
    b3WorldId world = b3_nullWorldId;
    std::vector<std::unique_ptr<Box3DRobot>> robots;
    double accumulator = 0.0;
    int steps_taken = 0;

    ~CandidateScene()
    {
        robots.clear();
        if (b3World_IsValid(world)) b3DestroyWorld(world);
    }
};

Box3DPhysicsScene::Box3DPhysicsScene(const Box3DSceneOptions& options)
    : options_(options)
{
    if (options_.substeps < 4)
        throw std::invalid_argument("Box3DPhysicsScene requires substeps >= 4");
    if (!(options_.fixed_timestep > 0.0) || !std::isfinite(options_.fixed_timestep))
        throw std::invalid_argument("Box3DPhysicsScene fixed_timestep must be finite and > 0");
    if (options_.worker_count < 1)
        throw std::invalid_argument("Box3DPhysicsScene worker_count must be >= 1");
    if (!std::isfinite(options_.gravity_z))
        throw std::invalid_argument("Box3DPhysicsScene gravity_z must be finite");
    if (options_.add_ground_plane &&
        (!std::isfinite(options_.ground_plane_z) ||
         !std::isfinite(options_.ground_plane_half_extent) ||
         options_.ground_plane_half_extent <= 0.0))
        throw std::invalid_argument(
            "Box3DPhysicsScene ground plane needs a finite height and a positive half extent");

    // The world exists from construction, so a robot added during scenario assembly is live and
    // queryable immediately rather than after a second, deferred build phase.
    world_ = createWorld();
}

Box3DPhysicsScene::~Box3DPhysicsScene()
{
    destroyLiveScene();
}

Box3DPhysicsScene::RobotHandle Box3DPhysicsScene::addRobot(
    const urdf::Robot& model, const BuildOptions& options)
{
    if (sealed_)
        throw std::logic_error("cannot add a Box3D robot after the shared scene was sealed");
    if (options.add_ground_plane)
        throw std::invalid_argument(
            "per-robot add_ground_plane is not valid in a shared Box3D world; use "
            "Box3DSceneOptions::add_ground_plane or Box3DPhysicsScene::setStaticWorld");
    if (reset_candidate_)
        throw std::logic_error("cannot add a Box3D robot during a reset transaction");

    Registration registration;
    registration.model = model;
    registration.options = normalizeRobotOptions(options);

    // Build first, record second: a robot that throws while instantiating leaves no half-registered
    // entry behind, and the destructor of the discarded Box3DRobot removes whatever bodies it did
    // create from the still-live world.
    auto robot = std::make_unique<Box3DRobot>();
    robot->buildInWorld(registration.model, registration.options, world_);

    registrations_.push_back(std::move(registration));
    robots_.push_back(std::move(robot));
    return robots_.size() - 1;
}

void Box3DPhysicsScene::setStaticWorld(std::shared_ptr<const urdf::StaticWorld> world)
{
    if (!robots_.empty())
        throw std::logic_error(
            "the shared Box3D static world must be set before the first robot joins, so that the "
            "world rebuilt by reset has the same body creation order as the original");
    static_geometry_ = Box3DStaticGeometry::acquire(std::move(world));
    if (static_geometry_) static_geometry_->attachTo(world_);
}

b3WorldId Box3DPhysicsScene::createWorld() const
{
    b3WorldDef definition = b3DefaultWorldDef();
    definition.gravity = b3Vec3{ 0.0f, 0.0f, static_cast<float>(options_.gravity_z) };
    definition.workerCount = static_cast<uint32_t>(options_.worker_count);
    const b3WorldId world = b3CreateWorld(&definition);
    if (!b3World_IsValid(world)) throw std::runtime_error("b3CreateWorld failed for shared scene");

    if (options_.add_ground_plane) {
        b3BodyDef body = b3DefaultBodyDef();
        body.type = b3_staticBody;
        // Box3D has no infinite half-space, so this is a thick slab: depth is what stops a
        // fast wheel tunnelling through it.
        const float half = static_cast<float>(options_.ground_plane_half_extent);
        body.position = toB3Pos(0.0, 0.0, options_.ground_plane_z - 0.5);
        body.name = "scene_ground_plane";
        const b3BodyId ground = b3CreateBody(world, &body);

        b3ShapeDef shape = b3DefaultShapeDef();
        shape.baseMaterial.friction = static_cast<float>(options_.ground_friction);
        const b3BoxHull box = b3MakeBoxHull(half, half, 0.5f);
        b3CreateHullShape(ground, &shape, &box.base);
    }
    return world;
}

BuildOptions Box3DPhysicsScene::normalizeRobotOptions(const BuildOptions& options) const
{
    BuildOptions normalized = options;
    normalized.substeps = options_.substeps;
    normalized.fixed_timestep = options_.fixed_timestep;
    normalized.worker_count = options_.worker_count;
    normalized.gravity_z = options_.gravity_z;
    normalized.add_ground_plane = false;
    return normalized;
}

void Box3DPhysicsScene::seal()
{
    if (sealed_) throw std::logic_error("Box3DPhysicsScene::seal called twice");
    if (robots_.empty())
        throw std::logic_error("Box3DPhysicsScene::seal requires at least one robot");

    accumulator_ = 0;
    steps_taken_ = 0;
    sealed_ = true;
}

void Box3DPhysicsScene::reset()
{
    prepareReset();
    commitPreparedReset();
    finalizePreparedReset();
}

void Box3DPhysicsScene::prepareReset()
{
    if (!sealed_)
        throw std::logic_error("Box3DPhysicsScene::prepareReset before the scene was sealed");
    if (reset_candidate_)
        throw std::logic_error("Box3DPhysicsScene already has a reset transaction in progress");

    auto candidate = std::make_unique<CandidateScene>();
    candidate->world = createWorld();
    if (static_geometry_) static_geometry_->attachTo(candidate->world);

    candidate->robots.reserve(robots_.size());
    for (const std::unique_ptr<Box3DRobot>& robot : robots_)
        candidate->robots.push_back(robot->cloneIntoWorld(candidate->world));

    reset_candidate_ = std::move(candidate);
    reset_committed_ = false;
}

void Box3DPhysicsScene::commitPreparedReset()
{
    if (!reset_candidate_)
        throw std::logic_error("Box3DPhysicsScene::commitPreparedReset without prepareReset");
    if (reset_committed_)
        throw std::logic_error("Box3DPhysicsScene reset transaction was already committed");

    // Commit. Each swap is noexcept and robot objects themselves stay in place, so handles and
    // references remain stable. The candidate now owns every old body and the old world, but it is
    // deliberately retained until the global transaction finalizes (or swapped back on abort).
    for (size_t i = 0; i < robots_.size(); ++i)
        robots_[i]->swapSharedRuntime(*reset_candidate_->robots[i]);
    std::swap(world_, reset_candidate_->world);
    std::swap(accumulator_, reset_candidate_->accumulator);
    std::swap(steps_taken_, reset_candidate_->steps_taken);
    reset_committed_ = true;
}

void Box3DPhysicsScene::abortPreparedReset() noexcept
{
    if (!reset_candidate_)
        return;

    if (reset_committed_) {
        for (size_t i = 0; i < robots_.size(); ++i)
            robots_[i]->swapSharedRuntime(*reset_candidate_->robots[i]);
        std::swap(world_, reset_candidate_->world);
        std::swap(accumulator_, reset_candidate_->accumulator);
        std::swap(steps_taken_, reset_candidate_->steps_taken);
    }
    reset_candidate_.reset();
    reset_committed_ = false;
}

void Box3DPhysicsScene::finalizePreparedReset() noexcept
{
    if (!reset_candidate_ || !reset_committed_)
        return;
    reset_candidate_.reset();
    reset_committed_ = false;
}

void Box3DPhysicsScene::stepOnce()
{
    if (!sealed_)
        throw std::logic_error("Box3DPhysicsScene::stepOnce before the scene was sealed");

    for (const std::unique_ptr<Box3DRobot>& robot : robots_) robot->prepareSharedStep();
    b3World_Step(world_, static_cast<float>(options_.fixed_timestep), options_.substeps);
    for (const std::unique_ptr<Box3DRobot>& robot : robots_) robot->finishSharedStep();
    ++steps_taken_;
}

int Box3DPhysicsScene::step(double dt)
{
    if (!sealed_)
        throw std::logic_error("Box3DPhysicsScene::step before the scene was sealed");
    if (dt < 0.0 || !std::isfinite(dt))
        throw std::invalid_argument("Box3DPhysicsScene::step dt must be finite and non-negative");

    accumulator_ += dt;
    int taken = 0;
    while (accumulator_ >= options_.fixed_timestep) {
        stepOnce();
        accumulator_ -= options_.fixed_timestep;
        ++taken;
    }
    return taken;
}

Box3DRobot& Box3DPhysicsScene::robot(RobotHandle handle)
{
    return *robots_.at(handle);
}

const Box3DRobot& Box3DPhysicsScene::robot(RobotHandle handle) const
{
    return *robots_.at(handle);
}

void Box3DPhysicsScene::destroyLiveScene()
{
    // Covers teardown during either half of a reset transaction. The candidate owns a distinct
    // complete world whether it is the staged new scene or the committed old rollback scene.
    reset_candidate_.reset();
    reset_committed_ = false;
    // External-world robots must release their bodies while `world_` is still valid.
    robots_.clear();
    if (b3World_IsValid(world_)) b3DestroyWorld(world_);
    world_ = b3_nullWorldId;
    accumulator_ = 0;
    steps_taken_ = 0;
    sealed_ = false;
}

void Box3DPhysicsScene::collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                               urdf::CollisionDebugSnapshot& out) const
{
    out = urdf::CollisionDebugSnapshot();
    out.backend = "box3d";
    if (!b3World_IsValid(world_))
        return;

    if (filter.include_robots) {
        for (size_t r = 0; r < robots_.size(); ++r) {
            // The registration's URDF name, so two copies of one model are still told apart.
            const std::string prefix =
                (r < registrations_.size() && !registrations_[r].model.name.empty()
                     ? registrations_[r].model.name
                     : "robot" + std::to_string(r)) + "/";
            urdf::readBox3DRobotCollision(*robots_[r], prefix, filter, out);
        }
    }

    if (filter.include_world && static_geometry_ != nullptr)
        urdf::readBox3DStaticWorld(static_geometry_->source(), filter, out);
}

void Box3DPhysicsScene::describeColliders(RobotHandle handle, const std::string& stable_id,
                                         urdf::PhysicsColliderSet& out) const
{
    out = urdf::PhysicsColliderSet();
    out.backend = "box3d";
    if (handle >= robots_.size())
        return;
    // Falls back to the handle, never to the URDF name: the handle is at least unique. A model
    // name is not, and the whole point of the parameter is that the scene cannot know an identity
    // it was never given.
    const std::string prefix =
        (stable_id.empty() ? "robot" + std::to_string(handle) : stable_id) + "/";
    urdf::describeBox3DColliders(*robots_[handle], prefix, out);
}

} // namespace b3urdf

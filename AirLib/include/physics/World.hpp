// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_World_hpp
#define airsim_core_World_hpp

#include <functional>
#include <stdexcept>
#include "common/Common.hpp"
#include "common/UpdatableContainer.hpp"
#include "PhysicsEngineBase.hpp"
#include "PhysicsBody.hpp"
#include "PhysicsSceneCoordinator.hpp"
#include "common/common_utils/ScheduledExecutor.hpp"
#include "common/ClockFactory.hpp"

namespace msr
{
namespace airlib
{

    class World : public UpdatableContainer<UpdatableObject*>
    {
    public:
        World(std::unique_ptr<PhysicsEngineBase> physics_engine)
            : physics_engine_(std::move(physics_engine))
        {
            World::clear();
            setName("World");
            if (physics_engine_ != nullptr)
                physics_engine_->setParent(this);
        }

        /** Make one authoritative fixed timestep the solver's dt for every subsequent tick.
         *
         * ⚠ This deliberately decouples simulation time from the executor's measured wall period.
         * Under load the executor falls behind and the sim runs slower than real time; it never
         * integrates a longer, incidental interval to catch up, because a dt that depends on
         * machine load makes a run irreproducible. Legacy mode never calls this and keeps deriving
         * dt from the clock exactly as before.
         *
         * The coordinator pointer is borrowed. Its owner must outlive this World, which
         * ASimModeWorldBase guarantees by destroying the physics world first.
         */
        void enableFixedStep(TTimeDelta fixed_step_seconds, PhysicsSceneCoordinator* coordinator,
                             const PhysicsResetPolicy& reset_policy)
        {
            if (!(fixed_step_seconds > 0))
                throw std::invalid_argument("World: fixed step must be positive");
            fixed_step_seconds_ = fixed_step_seconds;
            coordinator_ = coordinator;
            reset_policy_ = reset_policy;
        }

        bool isFixedStep() const { return fixed_step_seconds_ > 0; }
        TTimeDelta getFixedStep() const { return fixed_step_seconds_; }

        //override updatable interface so we can synchronize physics engine
        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            // The coordinator's global transaction runs FIRST and as one barrier: it advances the
            // reset epoch, rebuilds every shared scene from the frozen manifest, and performs the
            // approved pre-settle with nothing published. Only then may the members restore
            // themselves, because a vehicle api reads the state that transaction produced.
            if (coordinator_ != nullptr && coordinator_->enabled())
                coordinator_->reset("world-reset", reset_policy_);

            UpdatableContainer::resetImplementation();

            if (physics_engine_)
                physics_engine_->reset();
        }

        virtual void update(float delta = 0) override
        {
            if (fixed_step_seconds_ > 0) {
                // One dt, chosen once, propagated everywhere: the clock advances by exactly this
                // amount, the coordinator commits a step of exactly this length, and every member
                // is handed the same value instead of measuring elapsed time for itself.
                ClockFactory::get()->stepBy(fixed_step_seconds_);
                const float fixed_delta = static_cast<float>(fixed_step_seconds_);

                if (coordinator_ != nullptr && coordinator_->enabled())
                    coordinator_->step(fixed_step_seconds_);

                UpdatableContainer::update(fixed_delta);

                if (physics_engine_)
                    physics_engine_->update(fixed_delta);
                return;
            }

            ClockFactory::get()->step();

            //first update our objects
            UpdatableContainer::update(delta);

            //now update kinematics state
            if (physics_engine_)
                physics_engine_->update(delta);
        }

        virtual void reportState(StateReporter& reporter) override
        {
            reporter.writeValue("Sleep", 1.0f / executor_.getSleepTimeAvg());
            if (physics_engine_)
                physics_engine_->reportState(reporter);

            //call base
            UpdatableContainer::reportState(reporter);
        }
        //*** End: UpdatableState implementation ***//

        //override membership modification methods so we can synchronize physics engine
        virtual void clear() override
        {
            if (physics_engine_)
                physics_engine_->clear();
            UpdatableContainer::clear();
        }

        virtual void insert(UpdatableObject* member) override
        {
            if (physics_engine_ && member->getPhysicsBody() != nullptr)
                physics_engine_->insert(static_cast<PhysicsBody*>(member->getPhysicsBody()));

            UpdatableContainer::insert(member);
        }

        virtual void erase_remove(UpdatableObject* member) override
        {
            if (physics_engine_ && member->getPhysicsBody() != nullptr)
                physics_engine_->erase_remove(static_cast<PhysicsBody*>(
                    member->getPhysicsBody()));

            UpdatableContainer::erase_remove(member);
        }

        //async updater thread
        void startAsyncUpdator(uint64_t period)
        {
            //TODO: probably we shouldn't be passing around fixed period
            executor_.initialize(std::bind(&World::worldUpdatorAsync, this, std::placeholders::_1), period);
            executor_.start();
        }
        void stopAsyncUpdator()
        {
            executor_.stop();
        }
        void lock()
        {
            executor_.lock();
        }
        void unlock()
        {
            executor_.unlock();
        }

        virtual ~World()
        {
            executor_.stop();
        }

        void pause(bool is_paused)
        {
            executor_.pause(is_paused);
        }

        bool isPaused() const
        {
            return executor_.isPaused();
        }

        void pauseForTime(double seconds)
        {
            executor_.pauseForTime(seconds);
        }

        void continueForTime(double seconds)
        {
            executor_.continueForTime(seconds);
        }

        void continueForFrames(uint32_t frames)
        {
            executor_.continueForFrames(frames);
        }

        void setFrameNumber(uint32_t frameNumber)
        {
            executor_.setFrameNumber(frameNumber);
        }

    private:
        bool worldUpdatorAsync(uint64_t dt_nanos)
        {
            // ⚠ The executor's measured period is diagnostic, never the solver's dt. In fixed-step
            // mode update() uses the configured timestep; in legacy mode every consumer still
            // derives its own dt from the clock, which is what this path has always done.
            unused(dt_nanos);

            try {
                update();
            }
            catch (const std::exception& ex) {
                //Utils::DebugBreak();
                Utils::log(Utils::stringf("Exception occurred while updating world: %s", ex.what()), Utils::kLogLevelError);
            }
            catch (...) {
                //Utils::DebugBreak();
                Utils::log("Exception occurred while updating world", Utils::kLogLevelError);
            }

            return true;
        }

    private:
        std::unique_ptr<PhysicsEngineBase> physics_engine_ = nullptr;
        common_utils::ScheduledExecutor executor_;

        /// Zero means legacy clock-derived stepping. Any positive value is the one authoritative
        /// simulation timestep for this world.
        TTimeDelta fixed_step_seconds_ = 0;
        /// Borrowed; owned by the sim mode which created this world. Null in legacy mode.
        PhysicsSceneCoordinator* coordinator_ = nullptr;
        PhysicsResetPolicy reset_policy_;
    };
}
} //namespace
#endif

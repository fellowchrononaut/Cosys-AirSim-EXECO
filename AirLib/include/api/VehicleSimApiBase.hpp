// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleSimApiBase_hpp
#define air_VehicleSimApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/ImageCaptureBase.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "common/AirSimSettings.hpp"

/// Forward-declared on purpose: references need no definition, so the generic vehicle
/// interface does not acquire a dependency on the URDF model for the sake of one debug hook.
namespace urdf {
struct CollisionDebugFilter;
struct CollisionDebugSnapshot;
struct PhysicsColliderSet;
struct Wrench;
} // namespace urdf

namespace msr
{
namespace airlib
{

    class VehicleSimApiBase : public msr::airlib::UpdatableObject
    {
    public:
        virtual ~VehicleSimApiBase() = default;

        virtual void update(float delta = 0) override
        {
            UpdatableObject::update(delta);
        }

        //this method is called at every render tick when we want to transfer state from
        //physics engine to render engine. As physics engine is halted while
        //this happens, this method should do minimal processing
        virtual void updateRenderedState(float dt)
        {
            unused(dt);
            //derived class should override if needed
        }
        //called when render changes are required at every render tick
        /// Describe this vehicle's links as registerable MPM colliders (plan §11.1).
        /// Returns false when the vehicle cannot say.
        ///
        /// ⚠ AN INPUT, unlike collisionDebugGeometry below. A Newton MPM sidecar registers
        /// colliders from these numbers and pushes sand with them, so being wrong here deforms
        /// terrain in the wrong place rather than merely drawing a wrong picture.
        ///
        /// Declared on the generic vehicle for the same reason as the debug hook: a caller holding
        /// `VehicleSimApiBase*` cannot ask "are you a urdfbot?" in a build with RTTI off.
        virtual bool describeColliders(urdf::PhysicsColliderSet& /*out*/) const { return false; }

        /// Apply an external wrench to one of this vehicle's links, by the `link_index` that
        /// `describeColliders` reported for it.
        ///
        /// ⚠ INDEX, NOT STABLE ID, deliberately. The caller resolves the id ONCE when the registry
        /// is published; doing a string lookup per link per tick would put a map probe on the hot
        /// path for no gain, and the index is exactly what the descriptor promises is stable "for
        /// cheap per-step updates afterwards".
        ///
        /// ⚠ The default is false — a vehicle that cannot be pushed says so rather than silently
        /// swallowing forces that the caller believes were applied.
        virtual bool applyLinkWrench(size_t /*link_index*/, const urdf::Wrench& /*wrench*/)
        {
            return false;
        }

        /// Collision geometry as THIS vehicle's solver holds it, for the debug overlay.
        /// Returns false when the vehicle cannot say, which is the honest default.
        ///
        /// ⚠ Declared here, on the generic vehicle, rather than on the URDF sim api — because a
        /// caller holding `VehicleSimApiBase*` cannot ask "are you a urdfbot?" in this build.
        /// Unreal compiles with RTTI off, so `dynamic_cast` does not exist and a `static_cast` to
        /// the wrong sim api is silent UB. A virtual that every vehicle can answer is the only
        /// safe discriminator, and it leaves room for the Chaos and FastPhysics vehicles to answer
        /// it too once they have a shared scene.
        ///
        /// ⚠ A VIEW, NEVER AN INPUT. Nothing in the simulation may branch on what this returns.
        virtual bool collisionDebugGeometry(const urdf::CollisionDebugFilter& /*filter*/,
                                            urdf::CollisionDebugSnapshot& /*out*/) const
        {
            return false;
        }

        virtual void updateRendering(float dt)
        {
            unused(dt);
            //derived class should override if needed
        }

        virtual const ImageCaptureBase* getImageCapture() const = 0;
        virtual ImageCaptureBase* getImageCapture()
        {
            return const_cast<ImageCaptureBase*>(static_cast<const VehicleSimApiBase*>(this)->getImageCapture());
        }

        virtual void initialize() = 0;

        virtual bool testLineOfSightToPoint(const GeoPoint& point) const = 0;

        virtual Pose getPose() const = 0;
        virtual void setPose(const Pose& pose, bool ignore_collision) = 0;
        virtual const Kinematics::State* getGroundTruthKinematics() const = 0;
        virtual void setKinematics(const Kinematics::State& state, bool ignore_collision) = 0;
        virtual Kinematics::State getPhysicsRawKinematics() = 0;
        virtual void setPhysicsRawKinematics(const Kinematics::State& state) = 0;
        virtual const msr::airlib::Environment* getGroundTruthEnvironment() const = 0;

        virtual CameraInfo getCameraInfo(const std::string& camera_name) const = 0;
        virtual void setCameraOrientation(const std::string& camera_name, const Quaternionr& orientation) = 0;


        virtual CollisionInfo getCollisionInfo() const = 0;
        virtual CollisionInfo getCollisionInfoAndReset() = 0;
        virtual int getRemoteControlID() const = 0; //which RC to use, 0 is first one, -1 means disable RC (use keyborad)
        virtual RCData getRCData() const = 0; //get reading from RC from simulator's host OS
        virtual std::string getVehicleName() const = 0;
        virtual std::string getRecordFileLine(bool is_header_line) const = 0;
        virtual void toggleTrace() = 0;
        virtual void setTraceLine(const std::vector<float>& color_rgba, float thickness) = 0;

        //use pointer here because of derived classes for VehicleSetting
        const AirSimSettings::VehicleSetting* getVehicleSetting() const
        {
            return AirSimSettings::singleton().getVehicleSetting(getVehicleName());
        }
    };
}
} //namespace
#endif

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarBase_hpp
#define msr_airlib_LidarBase_hpp

#include <memory>
#include <mutex>
#include "sensors/SensorBase.hpp"

namespace msr
{
namespace airlib
{

    class LidarBase : public SensorBase
    {
    public:
        LidarBase(const std::string& sensor_name = "")
            : SensorBase(sensor_name), output_(std::make_shared<const LidarData>())
        {
        }

        // I-S: this hand-off crosses threads and used to be completely unsynchronised.
        //
        // setOutput() runs on the async physics thread (LidarSimple::updateOutput). getOutput() runs
        // on the RPC thread - RpcLibServerBase binds `const auto& lidar_data = ...getLidarData(...)`
        // and serialises it. The old code returned a REFERENCE to the live member and assigned over
        // that same member with no lock, so the reader walked a vector<std::string> of 32,768
        // entries while the writer freed and reallocated them. That corrupts the heap; observed once
        // as a SIGSEGV inside mimalloc with the fault surfacing in updateOutput()'s groundtruth
        // copy, nowhere near the racing code.
        //
        // Fixed by publishing an IMMUTABLE snapshot behind a shared_ptr. Readers take a refcount and
        // then own a copy that can never be mutated underneath them; the writer swaps in a new one.
        // The expensive copy happens OUTSIDE the lock, so the physics thread only ever holds it for
        // a pointer assignment - it can never be stalled by a reader's serialisation work.
        //
        // NOTE: the same unsynchronised pattern exists in GPULidarBase, EchoBase, DistanceBase,
        // WifiBase, MarLocUwbBase and SensorTemplateBase. Only LidarBase is fixed here because only
        // it has a measured failure; the others are the same shape and should follow.

        // Returns a snapshot by value. Deliberately NOT a reference - a reference is what made this
        // racy, since the caller reads it long after any lock would have been released.
        LidarData getOutput() const
        {
            return *getOutputSnapshot();
        }

        // Cheaper variant for callers that only need to read a few fields: shares the snapshot
        // rather than copying the point cloud. The pointee is const and its lifetime is guaranteed
        // by the refcount for as long as the caller holds it.
        std::shared_ptr<const LidarData> getOutputSnapshot() const
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            return output_;
        }

    public:
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            const auto snapshot = getOutputSnapshot();
            reporter.writeValue("Lidar-Timestamp", snapshot->time_stamp);
            reporter.writeValue("Lidar-NumPoints", static_cast<int>(snapshot->point_cloud.size() / 3));
        }

    protected:
        void setOutput(const LidarData& output)
        {
            // Build the new snapshot before taking the lock: the physics thread must not hold it
            // across a multi-megabyte copy.
            auto next = std::make_shared<const LidarData>(output);
            std::lock_guard<std::mutex> lock(output_mutex_);
            output_ = std::move(next);
        }

    private:
        std::shared_ptr<const LidarData> output_;
        mutable std::mutex output_mutex_;
    };
}
} //namespace
#endif

// Phase C - the publish seam.
//
// WHY THIS EXISTS
// ---------------
// Today every sensor payload is PULLED: a client calls simGetImages, the sim renders, serialises
// the pixels through msgpack and ships them back over rpclib. Measured (PHASE-A1-RESULTS §25, §26):
// the sim's own cost for a 16 x 1080p batch is ~123 ms, yet the achieved rate is stuck at ~6 Hz and
// did NOT move when B5 cut sim-side cost by 1.47x. The ceiling is no longer in the render path - it
// is the per-call round trip and the ~99 MB/cycle of serialisation on top of it.
//
// This header is the seam that lets a payload leave the sim WITHOUT that round trip. It is
// deliberately transport-neutral: Phase C validates the seam, the metadata and the threading with a
// loopback sink that goes nowhere. Phase D puts shared memory underneath it.
//
// ⚠ THE RPC PATH IS UNCHANGED AND STAYS THAT WAY. This publishes a COPY alongside the existing
// response; nothing here alters what simGetImages returns, and with no sink installed the entire
// cost is one relaxed atomic load. That is the standing constraint: classic AirSim clients must
// keep working, and the new path is optional.
//
// THREADING. publish() is called from whichever thread finished the capture - the RPC worker for a
// pulled image, and a render-thread drain once B2 pushes on a clock. Sinks must therefore be
// internally thread-safe, and MUST NOT block: a sink that waits on a consumer would reintroduce
// exactly the stall B3 removed. Sinks drop instead, and say so in stats.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{
    /** What a consumer needs to interpret the bytes, with no pointers and no heap: a sink may write
     *  this straight into shared memory, so it has to stay trivially copyable and fixed-size. */
    /** Intrinsics travelling WITH the frame, so the stream is self-describing.
     *
     *  ⚠ WHY IN THE SEGMENT rather than read from settings.json by the consumer. A consumer that
     *  parses settings itself can silently describe a DIFFERENT camera than the one that produced
     *  the pixels — wrong file, edited since launch, or a resolution override in CaptureSettings —
     *  and the result is intrinsics that look entirely plausible and reproject subtly wrong. That
     *  is invisible to every integrity check we have. Carrying them beside the payload makes the
     *  two impossible to disagree.
     *
     *  ⚠ TYPE-TAGGED AND FIXED-WIDTH ON PURPOSE. `params` is model-specific, so a new model costs
     *  an enum value rather than another wire break:
     *      Pinhole        -> param_count 0 (or 5 for radtan k1,k2,p1,p2,k3 if ever needed)
     *      KannalaBrandt  -> param_count 4, params = k1..k4
     *      DoubleSphere   -> param_count 2, params = [xi, alpha]
     *
     *  ⚠ model_width/model_height are the resolution the INTRINSICS WERE AUTHORED AT, which is not
     *  necessarily the frame's. If CaptureSettings renders at another size, fx/fy/cx/cy must be
     *  scaled by the consumer — and it can only know to do that if both sizes are present. */
    struct StreamCameraModel
    {
        //mirrors msr::airlib::cameras::CameraModelType; 0 = None means "no model was declared",
        //which is NOT the same as pinhole-with-defaults and must stay distinguishable.
        uint8_t type = 0;
        uint8_t param_count = 0;
        uint16_t model_width = 0;
        uint16_t model_height = 0;
        uint16_t reserved = 0;

        //float, not double: these are camera intrinsics, where float carries ~7 significant digits
        //and a 1920-px sensor needs 4. Halving the struct matters more than digits nobody has.
        float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
        float params[8] = {};
    };

    struct StreamFrameMeta
    {
        static constexpr int kNameLen = 32;

        uint64_t timestamp_ns = 0;   //the CAPTURE instant, not the publish instant (§23.3)
        uint64_t sequence = 0;       //per-topic, monotonic; a gap is a drop the consumer can see
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t payload_bytes = 0;
        uint8_t image_type = 0;
        uint8_t pixels_as_float = 0; //1 => payload is float32, else uint8 RGB
        uint8_t channels = 3;
        uint8_t reserved = 0;
        char vehicle[kNameLen] = {};
        char camera[kNameLen] = {};

        //⚠ Appended at the END, and ShmHeader::kVersion bumped to 2 alongside it. A consumer built
        //against version 1 reads a struct 56 bytes shorter; the version check is what stops it
        //reading pixels at the wrong offset rather than merely missing the intrinsics.
        StreamCameraModel camera_model{};

        void setNames(const std::string& vehicle_name, const std::string& camera_name)
        {
            copyName(vehicle, vehicle_name);
            copyName(camera, camera_name);
        }

    private:
        static void copyName(char (&dst)[kNameLen], const std::string& src)
        {
            const size_t n = src.size() < (kNameLen - 1) ? src.size() : (kNameLen - 1);
            std::memcpy(dst, src.data(), n);
            dst[n] = '\0';
        }
    };

    //⚠ Pinned at BOTH ends. ros2/src/airsim_shm_bridge/include/airsim_shm_bridge/shm_segment.hpp
    //carries the mirror of these structs and its own matching asserts; a change here that is not
    //made there is a silent wire break, and the symptom is pixels read at the wrong offset.
    static_assert(sizeof(StreamCameraModel) == 56, "StreamCameraModel layout changed - update the ROS bridge mirror");
    static_assert(sizeof(StreamFrameMeta) == 152, "StreamFrameMeta layout changed - update the ROS bridge mirror and bump kVersion");

    /** A destination for published frames. Phase D adds a shared-memory implementation; Phase C
     *  ships only the loopback below, which is enough to prove the seam without a dependency. */
    class StreamSink
    {
    public:
        virtual ~StreamSink() = default;

        /** Returns false if the frame was dropped. MUST NOT block. */
        virtual bool publish(const StreamFrameMeta& meta, const uint8_t* payload, size_t bytes) = 0;
        virtual const char* name() const = 0;

        /** One line per thing the sink wants to report - live topics, ring occupancy, whatever it
         *  has. Exists so diagnostics never need to downcast: ⚠ UE builds with RTTI OFF, so
         *  dynamic_cast and dynamic_pointer_cast do not link. A sink describing itself is also the
         *  better shape - the caller should not need to know which sink is installed. */
        virtual std::vector<std::string> describe() const { return {}; }
    };

    /** Process-wide registry. One sink at a time, swappable at runtime.
     *
     *  ⚠ The hot path is enabled(), which is a relaxed atomic load of a bool. It is checked before
     *  anything is copied, so a build with no sink installed pays that and nothing else - which is
     *  what makes it safe to leave the hook compiled into the shipping RPC path. */
    class SensorStream
    {
    public:
        struct Stats
        {
            uint64_t frames = 0;
            uint64_t bytes = 0;
            uint64_t drops = 0;
        };

        static SensorStream& singleton()
        {
            static SensorStream instance;
            return instance;
        }

        void setSink(std::shared_ptr<StreamSink> sink)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            sink_ = std::move(sink);
            //release so a consumer thread that sees enabled()==true also sees a fully built sink_
            enabled_.store(sink_ != nullptr, std::memory_order_release);
        }

        std::shared_ptr<StreamSink> sink() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return sink_;
        }

        bool enabled() const { return enabled_.load(std::memory_order_relaxed); }

        bool publish(const StreamFrameMeta& meta, const uint8_t* payload, size_t bytes)
        {
            if (!enabled_.load(std::memory_order_acquire))
                return false;

            std::shared_ptr<StreamSink> s;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                s = sink_;
            }
            if (s == nullptr)
                return false;

            const bool ok = s->publish(meta, payload, bytes);
            if (ok) {
                stats_frames_.fetch_add(1, std::memory_order_relaxed);
                stats_bytes_.fetch_add(bytes, std::memory_order_relaxed);
            }
            else {
                stats_drops_.fetch_add(1, std::memory_order_relaxed);
            }
            return ok;
        }

        /** PER-TOPIC sequence. ⚠ It was one global counter, which made the drop detection this
         *  design advertises meaningless: with 5 topics live, consecutive frames on one topic
         *  differ by 5, so a healthy stream reported a gap on EVERY frame (measured: "gaps seen:
         *  50" over 51 frames). A consumer can only tell it missed something if the numbering is
         *  per-topic and contiguous. */
        uint64_t nextSequence(const std::string& topic)
        {
            std::lock_guard<std::mutex> lock(seq_mutex_);
            return topic_sequence_[topic]++;
        }

        Stats stats() const
        {
            Stats s;
            s.frames = stats_frames_.load(std::memory_order_relaxed);
            s.bytes = stats_bytes_.load(std::memory_order_relaxed);
            s.drops = stats_drops_.load(std::memory_order_relaxed);
            return s;
        }

        void resetStats()
        {
            stats_frames_.store(0, std::memory_order_relaxed);
            stats_bytes_.store(0, std::memory_order_relaxed);
            stats_drops_.store(0, std::memory_order_relaxed);
        }

    private:
        SensorStream() = default;
        SensorStream(const SensorStream&) = delete;
        SensorStream& operator=(const SensorStream&) = delete;

        mutable std::mutex mutex_;
        std::shared_ptr<StreamSink> sink_;
        std::atomic<bool> enabled_{ false };
        mutable std::mutex seq_mutex_;
        std::map<std::string, uint64_t> topic_sequence_;
        std::atomic<uint64_t> stats_frames_{ 0 };
        std::atomic<uint64_t> stats_bytes_{ 0 };
        std::atomic<uint64_t> stats_drops_{ 0 };
    };

    /** Phase C validation sink. Keeps only the LAST frame per topic, which is deliberate: it makes
     *  the sink O(1) in memory and it is the same keep-latest policy a live consumer wants. It
     *  exists to answer "does the seam carry correct bytes and correct metadata, from the right
     *  thread, without slowing the RPC path" - not to move data anywhere. */
    class LoopbackSink : public StreamSink
    {
    public:
        struct Frame
        {
            StreamFrameMeta meta;
            std::vector<uint8_t> payload;
        };

        bool publish(const StreamFrameMeta& meta, const uint8_t* payload, size_t bytes) override
        {
            if (payload == nullptr || bytes == 0)
                return false;
            std::lock_guard<std::mutex> lock(mutex_);
            Frame& f = latest_[topicOf(meta)];
            f.meta = meta;
            f.payload.assign(payload, payload + bytes);
            return true;
        }

        const char* name() const override { return "loopback"; }

        /** Copy out the newest frame for a topic. False if nothing has been published for it. */
        bool latest(const std::string& topic, Frame& out) const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = latest_.find(topic);
            if (it == latest_.end())
                return false;
            out = it->second;
            return true;
        }

        std::vector<std::string> topics() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<std::string> out;
            out.reserve(latest_.size());
            for (const auto& kv : latest_)
                out.push_back(kv.first);
            return out;
        }

        std::vector<std::string> describe() const override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<std::string> out;
            out.reserve(latest_.size());
            for (const auto& kv : latest_) {
                const Frame& f = kv.second;
                out.push_back(kv.first + "  " + std::to_string(f.meta.width) + "x" +
                              std::to_string(f.meta.height) + "  seq=" +
                              std::to_string(f.meta.sequence) + "  " +
                              std::to_string(f.meta.payload_bytes) + " bytes  float=" +
                              std::to_string((int)f.meta.pixels_as_float) + "  t=" +
                              std::to_string(f.meta.timestamp_ns));
            }
            return out;
        }

        static std::string topicOf(const StreamFrameMeta& meta)
        {
            return std::string(meta.vehicle) + "/" + std::string(meta.camera) + "/" +
                   std::to_string(static_cast<int>(meta.image_type));
        }

    private:
        mutable std::mutex mutex_;
        std::map<std::string, Frame> latest_;
    };
}
}

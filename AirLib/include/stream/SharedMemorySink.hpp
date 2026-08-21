// Phase D - the shared-memory backend for the Phase C seam.
//
// WHY. §25/§26 located the image-rate ceiling OUTSIDE the render path: sim-side cost for a
// 16 x 1080p batch is ~123 ms, yet the achieved rate sits at ~6 Hz and did not move when B5 cut
// sim-side cost by 1.47x. What is left is the per-call RPC round trip and ~99 MB/cycle of msgpack
// serialisation on top of it. This sink removes both: the pixels are written once, into memory the
// consumer maps directly. No serialise, no socket, no per-consumer copy.
//
// ⚠ THE POINT IS NOT PEAK Hz ON ONE CONSUMER. It is that the SECOND and THIRD consumer cost
// nothing. Today each additional subscriber is another full set of RPC round trips with its own
// serialise and copy; here they all map the same pages. That is what makes multi-agent C-SLAM
// viable, and no amount of render-side work gets there.
//
// DESIGN - one segment per topic, seqlock, latest-wins.
//
//   <dir>/<prefix>_<vehicle>_<camera>_<type>          default dir /dev/shm
//
// ⚠ FILE-BACKED mmap, NOT POSIX shm_open. They are the same thing on Linux - /dev/shm IS tmpfs, so
// a file there is RAM at RAM speed - but shm_open hardcodes /dev/shm, and that is wrong in three
// situations this fork actually meets:
//   * a container WITHOUT the host's shm: Docker gives each container a private 64 MB /dev/shm
//     unless --ipc=host or -v /dev/shm:/dev/shm. (This project's sad_vio_dense happens to bind
//     /dev:/dev, so it sees the host's - but that is incidental, not a design guarantee.)
//   * Windows, which has no /dev/shm and no shm_open at all.
//   * any deployment that wants the segments on a specific shared volume.
// A directory is a parameter; /dev/shm is only the default. Point it at a bind-mounted volume and
// a container with no /dev sharing works, at the cost of that volume's backing store.
//     [Header][Slot 0][Slot 1] ... [Slot N-1]
//
// Each slot is guarded by its own SEQLOCK: the writer makes the sequence ODD, writes meta and
// payload, then makes it EVEN again. A reader takes the sequence, reads, and re-reads it; a value
// that changed or was odd means it was torn, so the reader retries. That gives consistent reads
// with a writer that NEVER BLOCKS and never waits on a consumer - which is the whole requirement,
// because a sink that blocks would put back exactly the stall B3 removed.
//
// Latest-wins, not a queue: a slow consumer misses frames and can SEE that it missed them, via the
// gap in StreamFrameMeta::sequence. That is the right failure mode for perception - a stale frame
// is worse than a dropped one - and it is the same policy that fixed the ROS stereo drift.
//
// ⚠ POSIX shm segments SURVIVE PROCESS EXIT. A crashed editor leaves them in /dev/shm. The sink
// unlinks a stale segment before creating its own, and clear() removes them all; anything left
// after a hard kill can be deleted by hand from /dev/shm.
#pragma once

#include "stream/SensorStream.hpp"

#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
#define AIRSIM_SHM_SUPPORTED 1
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#define AIRSIM_SHM_SUPPORTED 0
#endif

namespace msr
{
namespace airlib
{
    /** On-disk (well, on-/dev/shm) layout. Must stay trivially copyable and stable: a consumer
     *  compiled separately reads these bytes, so any change here is a wire-format change. */
    struct ShmHeader
    {
        static constexpr uint32_t kMagic = 0x41535348u; //'ASSH'
        //⚠ 2 since 2026-08-21: StreamFrameMeta gained camera_model (StreamCameraModel), which
        //moves every payload offset. A version-1 consumer MUST refuse a version-2 segment rather
        //than read pixels from the wrong place.
        static constexpr uint32_t kVersion = 2u;

        uint32_t magic;
        uint32_t version;
        uint32_t slot_count;
        uint32_t slot_stride;     //bytes from one slot to the next
        uint32_t payload_capacity;//max payload a slot can hold
        //⚠ Published rather than assumed: a consumer compiled separately cannot know
        //sizeof(ShmSlotHeader), and guessing it offsets every pixel it reads.
        uint32_t slot_header_bytes;
        uint64_t newest_index;    //monotonic; newest slot is (newest_index - 1) % slot_count
    };

    /** Per-slot seqlock guard. `seq` even and unchanged across the read == the read was clean. */
    struct ShmSlotHeader
    {
        uint64_t seq;
        StreamFrameMeta meta;
    };

#if AIRSIM_SHM_SUPPORTED

    class SharedMemorySink : public StreamSink
    {
    public:
        explicit SharedMemorySink(std::string dir = "/dev/shm", std::string prefix = "airsim",
                                  uint32_t slots = 3,
                                  uint32_t max_payload = 8u * 1024u * 1024u)
            : dir_(std::move(dir)), prefix_(std::move(prefix)),
              slots_(slots < 2 ? 2 : slots), max_payload_(max_payload)
        {
            while (dir_.size() > 1 && dir_.back() == '/')
                dir_.pop_back();
        }

        const std::string& directory() const { return dir_; }

        ~SharedMemorySink() override { clear(); }

        bool publish(const StreamFrameMeta& meta, const uint8_t* payload, size_t bytes) override
        {
            if (payload == nullptr || bytes == 0 || bytes > max_payload_)
                return false;

            Segment* seg = nullptr;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                seg = openOrCreate(LoopbackSink::topicOf(meta), static_cast<uint32_t>(bytes));
                if (seg == nullptr)
                    return false;
            }

            //One writer per topic in practice, but two RPC servers CAN capture the same camera
            //concurrently, so the slot advance is serialised per segment rather than assumed.
            std::lock_guard<std::mutex> wlock(seg->write_mutex);

            ShmHeader* hdr = seg->header();

            //⚠ REJECT STALE FRAMES. Harmless while captures were serial; required the moment they
            //overlap (Phase E pipelining). Two in-flight batches finish in whatever order the GPU
            //and the RPC threads decide, so an OLDER frame can arrive after a newer one - and with
            //latest-wins it would overwrite it, handing the consumer a picture that goes backwards
            //in time. Timestamps are the capture instant, so comparing them is exactly right.
            if (hdr->newest_index > 0) {
                const uint32_t prev = static_cast<uint32_t>((hdr->newest_index - 1) % hdr->slot_count);
                const uint64_t prev_ts = seg->slot(prev)->meta.timestamp_ns;
                if (meta.timestamp_ns != 0 && prev_ts != 0 && meta.timestamp_ns < prev_ts)
                    return false;   //counted as a drop by SensorStream, which is what it is
            }

            const uint32_t idx = static_cast<uint32_t>(hdr->newest_index % hdr->slot_count);
            ShmSlotHeader* slot = seg->slot(idx);

            //seqlock: odd => a write is in flight, readers must retry
            __atomic_store_n(&slot->seq, slot->seq | 1u, __ATOMIC_RELEASE);
            __atomic_thread_fence(__ATOMIC_RELEASE);

            slot->meta = meta;
            slot->meta.payload_bytes = static_cast<uint32_t>(bytes);
            std::memcpy(seg->payload(idx), payload, bytes);

            __atomic_thread_fence(__ATOMIC_RELEASE);
            __atomic_store_n(&slot->seq, (slot->seq | 1u) + 1u, __ATOMIC_RELEASE);

            //publish the slot only after its contents are visible
            __atomic_store_n(&hdr->newest_index, hdr->newest_index + 1u, __ATOMIC_RELEASE);
            return true;
        }

        const char* name() const override { return "sharedmemory"; }

        std::vector<std::string> describe() const override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<std::string> out;
            out.reserve(segments_.size());
            for (const auto& kv : segments_) {
                const Segment& s = *kv.second;
                const ShmHeader* h = const_cast<Segment&>(s).header();
                out.push_back(kv.first + "  " + s.name + "  " +
                              std::to_string(h->slot_count) + " slots x " +
                              std::to_string(h->payload_capacity) + " B  frames=" +
                              std::to_string(h->newest_index));
            }
            return out;
        }

        /** Unmap and unlink every segment this sink created. */
        void clear()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& kv : segments_) {
                Segment& s = *kv.second;
                if (s.base != nullptr && s.base != MAP_FAILED)
                    ::munmap(s.base, s.bytes);
                if (!s.name.empty())
                    ::unlink(s.name.c_str());
            }
            segments_.clear();
        }

    private:
        struct Segment
        {
            std::string name;     //POSIX shm name, leading '/'
            void* base = nullptr;
            size_t bytes = 0;
            uint32_t slot_stride = 0;
            std::mutex write_mutex;

            ShmHeader* header() { return reinterpret_cast<ShmHeader*>(base); }
            ShmSlotHeader* slot(uint32_t i)
            {
                return reinterpret_cast<ShmSlotHeader*>(static_cast<uint8_t*>(base) +
                                                        sizeof(ShmHeader) + i * slot_stride);
            }
            uint8_t* payload(uint32_t i)
            {
                return reinterpret_cast<uint8_t*>(slot(i)) + sizeof(ShmSlotHeader);
            }
        };

        /** A topic maps to one file. '/' and spaces become '_' so a topic never escapes dir_. */
        std::string pathFor(const std::string& topic) const
        {
            std::string leaf = prefix_ + "_" + topic;
            for (char& c : leaf) {
                if (c == '/' || c == ' ' || c == '\\')
                    c = '_';
            }
            if (leaf.size() > 200)
                leaf.resize(200);
            return dir_ + "/" + leaf;
        }

        Segment* openOrCreate(const std::string& topic, uint32_t payload_bytes)
        {
            auto it = segments_.find(topic);
            if (it != segments_.end()) {
                //a frame larger than the segment was sized for cannot be served; drop rather than
                //silently truncate, and say so once
                if (payload_bytes > it->second->header()->payload_capacity)
                    return nullptr;
                return it->second.get();
            }

            const uint32_t capacity = std::min(max_payload_, roundUp(payload_bytes));
            const uint32_t stride = static_cast<uint32_t>(sizeof(ShmSlotHeader)) + capacity;
            const size_t total = sizeof(ShmHeader) + static_cast<size_t>(stride) * slots_;

            auto seg = std::unique_ptr<Segment>(new Segment());
            seg->name = pathFor(topic);
            seg->bytes = total;
            seg->slot_stride = stride;

            //a stale segment from a crashed run would otherwise be reused at the wrong size
            ::unlink(seg->name.c_str());

            const int fd = ::open(seg->name.c_str(), O_CREAT | O_RDWR, 0666);
            if (fd < 0)
                return nullptr;
            if (::ftruncate(fd, static_cast<off_t>(total)) != 0) {
                ::close(fd);
                ::unlink(seg->name.c_str());
                return nullptr;
            }
            seg->base = ::mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            ::close(fd);
            if (seg->base == MAP_FAILED)
                return nullptr;

            std::memset(seg->base, 0, total);
            ShmHeader* h = seg->header();
            h->magic = ShmHeader::kMagic;
            h->version = ShmHeader::kVersion;
            h->slot_count = slots_;
            h->slot_stride = stride;
            h->payload_capacity = capacity;
            h->slot_header_bytes = static_cast<uint32_t>(sizeof(ShmSlotHeader));
            h->newest_index = 0;

            Segment* raw = seg.get();
            segments_.emplace(topic, std::move(seg));
            return raw;
        }

        static uint32_t roundUp(uint32_t v)
        {
            const uint32_t page = 4096u;
            return ((v + page - 1u) / page) * page;
        }

        mutable std::mutex mutex_;
        std::map<std::string, std::unique_ptr<Segment>> segments_;
        std::string dir_;
        std::string prefix_;
        uint32_t slots_;
        uint32_t max_payload_;
    };

#endif //AIRSIM_SHM_SUPPORTED
}
}

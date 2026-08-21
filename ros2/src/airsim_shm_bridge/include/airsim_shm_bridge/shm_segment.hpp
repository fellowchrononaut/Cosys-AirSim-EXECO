// Seqlock reader for AirSim shared-memory image segments.
//
// ⚠ WIRE LAYOUT. These structs mirror AirLib/include/stream/SharedMemorySink.hpp and
// AirLib/include/stream/SensorStream.hpp. They are DECLARED here rather than included so that a
// consumer needs nothing from the AirSim tree - which matters, because the ROS container's copy of
// the repo predates those headers. The static_asserts below are what keeps the two in step: if the
// sim's layout changes, this fails to compile instead of silently decoding garbage.
//
// The sim writes one segment per topic when `airsim.StreamSink 2` is set. Each slot is guarded by
// a SEQLOCK: the writer makes the sequence odd, writes, makes it even. A reader takes the
// sequence, reads, and re-reads it; changed or odd means the read was torn, so retry. The writer
// never blocks and never waits for a consumer, so a slow reader misses frames - and can SEE that
// it did, as a jump in newest_index, which is why the counter is per topic and monotonic.
#ifndef AIRSIM_SHM_BRIDGE__SHM_SEGMENT_HPP_
#define AIRSIM_SHM_BRIDGE__SHM_SEGMENT_HPP_

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace airsim_shm_bridge
{

constexpr uint32_t kMagic = 0x41535348;  // "ASSH"

#pragma pack(push, 1)
struct ShmHeader
{
  uint32_t magic;
  uint32_t version;
  uint32_t slot_count;
  uint32_t slot_stride;
  uint32_t payload_capacity;
  uint32_t slot_header_bytes;
  uint64_t newest_index;   // monotonic frame count, NOT a ring index; slot = (n-1) % slot_count
};

struct StreamFrameMeta
{
  uint64_t timestamp_ns;   // CAPTURE instant, sampled with the camera poses - not publish time
  uint64_t sequence;
  uint32_t width;
  uint32_t height;
  uint32_t payload_bytes;
  uint8_t image_type;
  uint8_t pixels_as_float;
  uint8_t channels;
  uint8_t reserved;
  char vehicle[32];
  char camera[32];
};
#pragma pack(pop)

static_assert(sizeof(ShmHeader) == 32, "ShmHeader layout drifted from the sim's");
static_assert(sizeof(StreamFrameMeta) == 96, "StreamFrameMeta layout drifted from the sim's");

/// msr::airlib::ImageCaptureBase::ImageType
inline const char * imageTypeName(uint8_t t)
{
  switch (t) {
    case 0: return "scene";
    case 1: return "depth_planar";
    case 2: return "depth_perspective";
    case 3: return "depth_vis";
    case 4: return "disparity_normalized";
    case 5: return "segmentation";
    case 6: return "surface_normals";
    case 7: return "infrared";
    case 8: return "optical_flow";
    case 9: return "optical_flow_vis";
    case 10: return "lighting";
    case 11: return "annotation";
    default: return "unknown";
  }
}

/// One topic's mapped ring. Open once, read many. Not thread-safe; one reader per instance.
class Segment
{
public:
  /// Returns nullptr and sets `error` if the file is not a valid segment (e.g. still being
  /// created). A failure here is expected during startup and must not be treated as fatal.
  static Segment * open(const std::string & path, std::string & error);

  ~Segment();

  Segment(const Segment &) = delete;
  Segment & operator=(const Segment &) = delete;

  const std::string & path() const {return path_;}

  uint64_t newestIndex() const
  {
    // Relaxed is enough: a stale value costs one missed poll, and the seqlock - not this - is what
    // makes the payload read safe.
    return reinterpret_cast<const volatile ShmHeader *>(base_)->newest_index;
  }

  /// Seqlock read of the newest slot. Returns false if it kept tearing or there is no frame yet.
  /// `payload` is resized and filled; `meta` is overwritten only on success.
  bool readNewest(StreamFrameMeta & meta, std::vector<uint8_t> & payload, int retries = 8);

private:
  Segment() = default;

  std::string path_;
  int fd_ = -1;
  uint8_t * base_ = nullptr;
  size_t map_size_ = 0;
  ShmHeader header_{};
};

/// Sorted list of `<dir>/airsim_*`.
std::vector<std::string> segmentPaths(const std::string & dir);

}  // namespace airsim_shm_bridge

#endif  // AIRSIM_SHM_BRIDGE__SHM_SEGMENT_HPP_

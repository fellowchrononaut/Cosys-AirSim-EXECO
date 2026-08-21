#include "airsim_shm_bridge/shm_segment.hpp"
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

namespace airsim_shm_bridge
{

namespace
{
uint64_t loadSeq(const volatile uint8_t * slot)
{
  uint64_t v;
  std::memcpy(&v, const_cast<const uint8_t *>(slot), sizeof(v));
  // The writer publishes the even sequence AFTER the payload. Without this fence the compiler or
  // CPU may hoist the payload read above the sequence read, and a torn frame then passes the
  // re-check. This is the whole correctness argument for the seqlock.
  std::atomic_thread_fence(std::memory_order_acquire);
  return v;
}
}  // namespace

Segment * Segment::open(const std::string & path, std::string & error)
{
  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    error = "open failed: " + std::string(std::strerror(errno));
    return nullptr;
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0 || static_cast<size_t>(st.st_size) < sizeof(ShmHeader)) {
    error = "too small to be a segment";
    ::close(fd);
    return nullptr;
  }

  void * addr = ::mmap(nullptr, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    error = "mmap failed: " + std::string(std::strerror(errno));
    ::close(fd);
    return nullptr;
  }

  ShmHeader hdr{};
  std::memcpy(&hdr, addr, sizeof(hdr));
  if (hdr.magic != kMagic) {
    error = "bad magic - not an AirSim stream segment";
    ::munmap(addr, st.st_size);
    ::close(fd);
    return nullptr;
  }
  // ⚠ THERE WAS NO VERSION CHECK UNTIL 2026-08-21, and magic alone is not enough. Wire version 2
  // appended intrinsics to StreamFrameMeta (96 -> 152 bytes), which moved EVERY payload offset. A
  // reader built against version 1 finds the right magic, computes the old offset, and returns
  // pixels from the middle of the previous field — a well-formed, completely wrong image, with
  // nothing to indicate anything went wrong. Refusing loudly is the entire point.
  if (hdr.version != kSupportedVersion) {
    error = "wire version " + std::to_string(hdr.version) + ", this build speaks " +
      std::to_string(kSupportedVersion) +
      " - rebuild the bridge against the sim it is reading";
    ::munmap(addr, st.st_size);
    ::close(fd);
    return nullptr;
  }
  if (hdr.slot_count == 0 || hdr.slot_stride == 0 ||
    hdr.slot_header_bytes < sizeof(uint64_t) + sizeof(StreamFrameMeta))
  {
    error = "implausible header (slots/stride/slot_header)";
    ::munmap(addr, st.st_size);
    ::close(fd);
    return nullptr;
  }

  auto * seg = new Segment();
  seg->path_ = path;
  seg->fd_ = fd;
  seg->base_ = static_cast<uint8_t *>(addr);
  seg->map_size_ = st.st_size;
  seg->header_ = hdr;
  return seg;
}

Segment::~Segment()
{
  if (base_ != nullptr) {
    ::munmap(base_, map_size_);
  }
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool Segment::readNewest(StreamFrameMeta & meta, std::vector<uint8_t> & payload, int retries)
{
  const uint64_t n = newestIndex();
  if (n == 0) {
    return false;
  }
  const uint64_t idx = (n - 1) % header_.slot_count;
  const uint8_t * slot = base_ + sizeof(ShmHeader) + idx * header_.slot_stride;

  for (int attempt = 0; attempt < retries; ++attempt) {
    const uint64_t seq0 = loadSeq(slot);
    if (seq0 & 1u) {                       // writer is mid-update
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      continue;
    }

    StreamFrameMeta m{};
    std::memcpy(&m, slot + sizeof(uint64_t), sizeof(m));
    if (m.payload_bytes == 0 || m.payload_bytes > header_.payload_capacity) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      continue;
    }

    payload.resize(m.payload_bytes);
    std::memcpy(payload.data(), slot + header_.slot_header_bytes, m.payload_bytes);

    if (loadSeq(slot) == seq0) {           // clean read
      meta = m;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  return false;
}

std::vector<std::string> segmentPaths(const std::string & dir)
{
  std::vector<std::string> out;
  DIR * d = ::opendir(dir.c_str());
  if (d == nullptr) {
    return out;
  }
  while (const dirent * e = ::readdir(d)) {
    const std::string name = e->d_name;
    if (name.rfind("airsim_", 0) == 0) {
      out.push_back(dir + "/" + name);
    }
  }
  ::closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace airsim_shm_bridge

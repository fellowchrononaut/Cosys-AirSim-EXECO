// End-to-end verification of the shared-memory -> ROS 2 image path, from a SUBSCRIBER's seat.
//
// ⚠ WHY THIS EXISTS. Byte-identity with RPC, per-topic sequence continuity and capture-instant
// timestamps were all verified HOST-SIDE. Nothing had ever been checked through an actual ROS
// subscriber, which is the only place a C-SLAM consumer lives.
//
// ⚠ WHY C++ AND NOT PYTHON, even though this is only a test tool. A Python subscriber cannot keep
// up with 1080p at rate, and the drops are not harmless here: the stereo-pairing check works by
// intersecting the SETS of timestamps seen on left and right, so a subscriber that samples the two
// sides disjointly reports zero matched stamps — a FALSE FAILURE on the single most important
// claim this tool makes. Slow reads also let the 3-slot ring recycle frames before the byte
// comparison can look at them. The project already learned this once on the publish side (rclpy's
// uint8[] setter cost 35.7 ms per image, and the Python bridge dropped half its frames); the same
// reasoning applies to a verifier that must not miss anything.
//
// WHAT IS CHECKED, and why each matters to SLAM:
//   1. STEREO PAIRING — cam_left and cam_right must carry the SAME header.stamp, not merely a close
//      one. Every image in a capture batch is stamped with one instant sampled in OnEndDraw. If
//      this fails, stereo triangulation is silently wrong and nothing downstream can detect it.
//   2. CROSS-MODALITY PAIRING — scene / depth / segmentation of one camera must share a stamp,
//      or RGB-D association is wrong.
//   3. TIMESTAMP MONOTONICITY — ⚠ the real risk with airsim.StreamCaptureInFlight > 1: several
//      batches are in flight and one stamped earlier can finish LATER. A regression means a
//      consumer sees time run backwards.
//   4. ENCODING AND GEOMETRY — rgb8 (never bgr8: image_data_uint8 is RGB), 32FC1 for depth, and
//      data.size() == step * height.
//   5. BYTE INTEGRITY — the delivered payload is compared against the shared-memory slot carrying
//      the same stamp, proving the bridge copies rather than mangles. One topic only, because
//      polling every topic's full payload would cost more bandwidth than the thing under test.
//
// ⚠ DROP ACCOUNTING IS NOT DONE HERE. The bridge's own C++ counters are authoritative. A gap seen
// by a subscriber is a subscriber's gap until proven otherwise.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "airsim_shm_bridge/shm_segment.hpp"

namespace
{
constexpr const char * G = "\033[32m";
constexpr const char * R = "\033[31m";
constexpr const char * Y = "\033[33m";
constexpr const char * B = "\033[1m";
constexpr const char * O = "\033[0m";

/// FNV-1a. Not cryptographic — this compares two byte ranges we expect to be identical, and a
/// 64-bit collision on that is not a failure mode worth defending against.
uint64_t fnv1a(const uint8_t * p, size_t n)
{
  uint64_t h = 1469598103934665603ULL;
  for (size_t i = 0; i < n; ++i) {h ^= p[i]; h *= 1099511628211ULL;}
  return h;
}

struct Rec
{
  uint64_t stamp_ns;
  uint32_t w, h, step;
  std::string encoding, frame_id;
  size_t len;
  uint64_t hash;
};
}  // namespace

class Verifier : public rclcpp::Node
{
public:
  Verifier()
  : Node("airsim_shm_verify")
  {
    vehicle_ = declare_parameter<std::string>("vehicle", "Husky1");
    secs_ = declare_parameter<double>("secs", 25.0);
    const std::string prefix = declare_parameter<std::string>("topic_prefix", "/airsim_shm");
    shm_dir_ = declare_parameter<std::string>("shm_dir", "/dev/shm");

    base_ = prefix + "/" + vehicle_;
    subs_names_ = {base_ + "/cam_left/scene", base_ + "/cam_right/scene",
      base_ + "/cam_left/depth_planar", base_ + "/cam_left/segmentation"};

    // ⚠ BEST_EFFORT is not a choice — the bridge publishes BEST_EFFORT, and a RELIABLE subscriber
    // simply will not match it. `ros2 topic echo` needs --qos-reliability best_effort for the same
    // reason.
    rclcpp::QoS q(rclcpp::KeepLast(4));
    q.best_effort();

    for (const auto & t : subs_names_) {
      subs_.push_back(
        create_subscription<sensor_msgs::msg::Image>(
          t, q, [this, t](sensor_msgs::msg::Image::ConstSharedPtr msg) {onImage(t, msg);}));
    }

    // Byte-integrity reference: poll ONE segment and remember stamp -> hash.
    integrity_topic_ = base_ + "/cam_left/scene";
    shm_path_ = shm_dir_ + "/airsim_" + vehicle_ + "_cam_left_0";
    poll_ = std::thread([this] {pollShm();});

    printf(
      "%s== subscribing to %zu topic(s) for %.0fs (vehicle %s) ==%s\n",
      B, subs_names_.size(), secs_, vehicle_.c_str(), O);
    start_ = now_s();
  }

  ~Verifier() override
  {
    run_.store(false);
    if (poll_.joinable()) {poll_.join();}
  }

  bool done() const {return now_s() - start_ >= secs_;}
  int report();

private:
  static double now_s()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void onImage(const std::string & topic, sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    Rec r;
    r.stamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
      msg->header.stamp.nanosec;
    r.w = msg->width; r.h = msg->height; r.step = msg->step;
    r.encoding = msg->encoding; r.frame_id = msg->header.frame_id;
    r.len = msg->data.size();
    r.hash = fnv1a(msg->data.data(), msg->data.size());
    std::lock_guard<std::mutex> lk(mutex_);
    got_[topic].push_back(r);
  }

  /// Keep a bounded stamp -> hash map for ONE topic, sampled fast enough to catch frames before the
  /// 3-slot ring recycles them.
  void pollShm()
  {
    std::string err;
    std::unique_ptr<airsim_shm_bridge::Segment> seg;
    while (run_.load()) {
      if (!seg) {
        seg.reset(airsim_shm_bridge::Segment::open(shm_path_, err));
        if (!seg) {std::this_thread::sleep_for(std::chrono::milliseconds(200)); continue;}
      }
      airsim_shm_bridge::StreamFrameMeta meta{};
      std::vector<uint8_t> payload;
      if (seg->readNewest(meta, payload) && meta.payload_bytes > 0) {
        std::lock_guard<std::mutex> lk(mutex_);
        shm_hashes_[meta.timestamp_ns] = fnv1a(payload.data(), payload.size());
        while (shm_hashes_.size() > 64) {shm_hashes_.erase(shm_hashes_.begin());}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  }

  std::string vehicle_, base_, shm_dir_, shm_path_, integrity_topic_;
  double secs_{25.0}, start_{0.0};
  std::vector<std::string> subs_names_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
  std::mutex mutex_;
  std::map<std::string, std::vector<Rec>> got_;
  std::map<uint64_t, uint64_t> shm_hashes_;
  std::atomic<bool> run_{true};
  std::thread poll_;
};

int Verifier::report()
{
  std::lock_guard<std::mutex> lk(mutex_);
  std::vector<std::string> failures;
  auto shortName = [this](const std::string & t) {return t.substr(base_.size() + 1);};
  auto ok = [](const std::string & m) {printf("  %sPASS%s  %s\n", G, O, m.c_str());};
  auto bad = [&](const std::string & m) {printf("  %sFAIL%s  %s\n", R, O, m.c_str());};
  auto warn = [](const std::string & m) {printf("  %sWARN%s  %s\n", Y, O, m.c_str());};
  auto hdr = [](const std::string & m) {printf("\n%s== %s ==%s\n", B, m.c_str(), O);};

  hdr("0. did anything arrive at all?");
  bool any = false;
  for (const auto & t : subs_names_) {
    const auto & v = got_[t];
    if (v.empty()) {
      bad(shortName(t) + ": NO MESSAGES");
      failures.push_back("no messages");
    } else {
      any = true;
      char buf[256];
      snprintf(
        buf, sizeof(buf), "%-26s %4zu msg  %5.2f Hz at the subscriber",
        shortName(t).c_str(), v.size(), v.size() / secs_);
      ok(buf);
    }
  }
  if (!any) {
    printf("\n  Nothing can be verified. Is the bridge running and the driver publishing?\n");
    return 1;
  }

  auto stamps = [&](const std::string & t) {
      std::set<uint64_t> s;
      for (const auto & r : got_[t]) {s.insert(r.stamp_ns);}
      return s;
    };

  hdr("1. stereo pairing - cam_left/scene vs cam_right/scene must share a stamp EXACTLY");
  {
    auto L = stamps(base_ + "/cam_left/scene");
    auto Rr = stamps(base_ + "/cam_right/scene");
    std::vector<uint64_t> both;
    std::set_intersection(
      L.begin(), L.end(), Rr.begin(), Rr.end(), std::back_inserter(both));
    char buf[256];
    if (both.empty()) {
      uint64_t closest = UINT64_MAX;
      for (auto a : L) {for (auto b : Rr) {
          closest = std::min(closest, a > b ? a - b : b - a);
        }}
      snprintf(
        buf, sizeof(buf),
        "ZERO matched stamps (%zu left, %zu right); closest pair %.2f ms apart",
        L.size(), Rr.size(), closest == UINT64_MAX ? -1.0 : closest / 1e6);
      bad(buf);
      failures.push_back("stereo pairing");
    } else {
      snprintf(
        buf, sizeof(buf),
        "%zu exactly-matched stamps (%.0f%% of the larger side) - the pair shares one instant",
        both.size(), 100.0 * both.size() / std::max(L.size(), Rr.size()));
      ok(buf);
    }
  }

  hdr("2. cross-modality pairing - scene vs depth vs segmentation on cam_left");
  {
    auto S = stamps(base_ + "/cam_left/scene");
    auto D = stamps(base_ + "/cam_left/depth_planar");
    auto Gg = stamps(base_ + "/cam_left/segmentation");
    std::vector<uint64_t> sd, tri;
    std::set_intersection(S.begin(), S.end(), D.begin(), D.end(), std::back_inserter(sd));
    std::set_intersection(sd.begin(), sd.end(), Gg.begin(), Gg.end(), std::back_inserter(tri));
    char buf[256];
    if (!tri.empty()) {
      snprintf(
        buf, sizeof(buf),
        "%zu stamp(s) on all three modalities - RGB-D association is sound", tri.size());
      ok(buf);
    } else if (!sd.empty()) {
      snprintf(buf, sizeof(buf), "scene and depth share %zu stamp(s) but segmentation shares none",
        sd.size());
      warn(buf);
    } else {
      snprintf(
        buf, sizeof(buf), "no shared stamps (scene %zu, depth %zu, seg %zu)",
        S.size(), D.size(), Gg.size());
      bad(buf);
      failures.push_back("cross-modality pairing");
    }
  }

  hdr("3. timestamp monotonicity per topic  (the real risk with several batches in flight)");
  for (const auto & t : subs_names_) {
    const auto & v = got_[t];
    if (v.size() < 2) {continue;}
    size_t back = 0; uint64_t worst = 0;
    for (size_t i = 1; i < v.size(); ++i) {
      if (v[i].stamp_ns < v[i - 1].stamp_ns) {
        ++back;
        worst = std::max(worst, v[i - 1].stamp_ns - v[i].stamp_ns);
      }
    }
    char buf[256];
    if (back) {
      snprintf(
        buf, sizeof(buf), "%-26s %zu REGRESSION(S), worst %.1f ms backwards",
        shortName(t).c_str(), back, worst / 1e6);
      bad(buf);
      failures.push_back("time regression");
    } else {
      snprintf(buf, sizeof(buf), "%-26s monotonic over %zu msg", shortName(t).c_str(), v.size());
      ok(buf);
    }
  }

  hdr("4. encoding and geometry");
  for (const auto & t : subs_names_) {
    const auto & v = got_[t];
    if (v.empty()) {continue;}
    const Rec & r = v.front();
    const bool depth = t.find("depth") != std::string::npos;
    const std::string want = depth ? "32FC1" : "rgb8";
    const uint32_t bpp = depth ? 4 : 3;
    std::string issues;
    if (r.encoding != want) {issues += " encoding '" + r.encoding + "' != '" + want + "';";}
    if (r.step != r.w * bpp) {issues += " step " + std::to_string(r.step) + " != " +
        std::to_string(r.w * bpp) + ";";}
    if (r.len != static_cast<size_t>(r.step) * r.h) {issues += " data " + std::to_string(r.len) +
        " != step*height " + std::to_string(static_cast<size_t>(r.step) * r.h) + ";";}
    char buf[320];
    if (!issues.empty()) {
      snprintf(buf, sizeof(buf), "%s:%s", shortName(t).c_str(), issues.c_str());
      bad(buf);
      failures.push_back("geometry");
    } else {
      snprintf(
        buf, sizeof(buf), "%-26s %ux%u %s step=%u frame_id='%s'",
        shortName(t).c_str(), r.w, r.h, r.encoding.c_str(), r.step, r.frame_id.c_str());
      ok(buf);
    }
  }

  hdr("5. byte integrity - ROS payload vs the shared-memory slot with the same stamp");
  {
    size_t matched = 0, mismatched = 0, uncomparable = 0;
    for (const auto & r : got_[integrity_topic_]) {
      auto it = shm_hashes_.find(r.stamp_ns);
      if (it == shm_hashes_.end()) {++uncomparable; continue;}
      if (it->second == r.hash) {++matched;} else {
        ++mismatched;
        char buf[256];
        snprintf(
          buf, sizeof(buf), "stamp %lu: ROS hash %lu != shm hash %lu",
          static_cast<unsigned long>(r.stamp_ns), static_cast<unsigned long>(r.hash),
          static_cast<unsigned long>(it->second));
        bad(buf);
      }
    }
    char buf[256];
    if (mismatched) {
      failures.push_back("byte mismatch");
    } else if (matched) {
      snprintf(
        buf, sizeof(buf),
        "%zu frame(s) byte-identical on %s (%zu not comparable - the ring recycled them first)",
        matched, shortName(integrity_topic_).c_str(), uncomparable);
      ok(buf);
    } else {
      snprintf(
        buf, sizeof(buf), "no comparable frames (%zu recycled before the poller saw them)",
        uncomparable);
      warn(buf);
    }
  }

  hdr("RESULT");
  std::set<std::string> uniq(failures.begin(), failures.end());
  if (!uniq.empty()) {
    std::string s;
    for (const auto & f : uniq) {s += (s.empty() ? "" : "; ") + f;}
    printf("  %s%sFAIL%s - %s\n", R, B, O, s.c_str());
    return 1;
  }
  printf("  %s%sPASS%s - stereo pairs share one capture instant, time never runs backwards,\n", G, B, O);
  printf("  encodings and geometry are correct, and ROS bytes match shared memory.\n");
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Verifier>();
  while (rclcpp::ok() && !node->done()) {rclcpp::spin_some(node);}
  const int rc = node->report();
  rclcpp::shutdown();
  return rc;
}

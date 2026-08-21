// Republish AirSim shared-memory image streams as sensor_msgs/Image.
//
// WHY THIS EXISTS. The classic ROS node pulls images over RPC: every frame crosses msgpack, a
// socket, and the blocking simGetImages call, and the sim's frame rate pays for it. The Phase C/D
// seam writes the same bytes straight into shared memory instead, and this node maps them.
//
// ⚠ WHY C++ AND NOT PYTHON. This was written in rclpy first and it could not keep up. Measured
// 2026-08-21 in this container, 640x512 RGB (983,040 bytes):
//
//     construct 0.06 ms | msg.data = payload 35.72 ms | publish 0.41 ms
//
// rclpy's generated setter for a uint8[] field validates every byte in Python. Six topics at 8 Hz
// needs 1.7 s of work per second, so the node published 4.2 Hz and dropped every other frame,
// while a pure read loop in the same container sustained 8.00 Hz with zero drops. Here `data` is a
// std::vector<uint8_t> and filling it is a memcpy. Do not port this back to Python.
//
// ⚠ Container note. The segments live in the SIM's /dev/shm. sad_vio_dense bind-mounts /dev:/dev,
// so the host's segments are visible inside it with no --ipc=host and no extra volume - verified
// by writing a probe file on the host and reading it back inside. A container WITHOUT that mount
// must point the sim at a shared path (`airsim.StreamDir <dir>`) and mount that instead.
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "airsim_shm_bridge/shm_segment.hpp"

using namespace std::chrono_literals;

namespace
{
std::set<std::string> splitAllowlist(const std::string & spec)
{
  // Comma-separated, not a string array. A YAML list typed on the command line is easy to get
  // subtly wrong, and getting it wrong allowlists nothing while looking exactly like a sim that is
  // not publishing.
  std::set<std::string> out;
  std::stringstream ss(spec);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    const size_t a = tok.find_first_not_of(" \t");
    if (a == std::string::npos) {continue;}
    const size_t b = tok.find_last_not_of(" \t");
    out.insert(tok.substr(a, b - a + 1));
  }
  return out;
}

std::string fixedName(const char * buf, size_t cap)
{
  const size_t n = ::strnlen(buf, cap);
  return std::string(buf, n);
}
}  // namespace

class ShmBridge : public rclcpp::Node
{
public:
  ShmBridge()
  : Node("airsim_shm_bridge")
  {
    stream_dir_ = declare_parameter<std::string>("stream_dir", "/dev/shm");
    const std::string topics = declare_parameter<std::string>("topics", "");
    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "/airsim_shm");
    poll_sec_ = declare_parameter<double>("poll_sec", 0.002);
    const double rescan_sec = declare_parameter<double>("rescan_sec", 2.0);
    reliable_ = declare_parameter<bool>("reliable", false);
    const double report_sec = declare_parameter<double>("report_sec", 5.0);
    frame_id_prefix_ = declare_parameter<std::string>("frame_id_prefix", "");

    allow_ = splitAllowlist(topics);
    while (!topic_prefix_.empty() && topic_prefix_.back() == '/') {topic_prefix_.pop_back();}
    if (poll_sec_ < 0.0005) {poll_sec_ = 0.0005;}

    RCLCPP_INFO(
      get_logger(), "reading %s/airsim_* | QoS %s | allowlist: %s",
      stream_dir_.c_str(), reliable_ ? "RELIABLE" : "BEST_EFFORT",
      allow_.empty() ? "(everything)" : topics.c_str());
    if (!reliable_) {
      // ⚠ A RELIABLE subscriber does not match a BEST_EFFORT publisher, and ROS reports that as an
      // empty topic rather than an error - which is the single most common way this looks broken.
      RCLCPP_INFO(
        get_logger(),
        "BEST_EFFORT: in rviz2 set the Image display's Reliability Policy to Best Effort; "
        "ros2 topic echo needs --qos-reliability best_effort; ros2 topic hz auto-matches");
    }

    rescan();
    running_ = true;
    reader_ = std::thread(&ShmBridge::readLoop, this);

    rescan_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.5, rescan_sec)), [this] {rescan();});
    if (report_sec > 0.0) {
      report_timer_ = create_wall_timer(
        std::chrono::duration<double>(report_sec), [this] {report();});
    }
  }

  ~ShmBridge() override
  {
    running_ = false;
    if (reader_.joinable()) {reader_.join();}
  }

private:
  struct StreamTopic
  {
    std::unique_ptr<airsim_shm_bridge::Segment> segment;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    std::string ros_topic;
    std::string stream_topic;
    uint64_t last_index = 0;
    uint64_t published = 0;
    uint64_t dropped = 0;
    uint64_t torn = 0;
  };

  // ---------------------------------------------------------------- discovery

  void rescan()
  {
    std::set<std::string> found;
    for (const std::string & path : airsim_shm_bridge::segmentPaths(stream_dir_)) {
      found.insert(path);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (topics_.count(path) || skipped_.count(path)) {continue;}
      }

      std::string err;
      std::unique_ptr<airsim_shm_bridge::Segment> seg(
        airsim_shm_bridge::Segment::open(path, err));
      if (!seg) {
        RCLCPP_DEBUG(get_logger(), "skipping %s: %s", path.c_str(), err.c_str());
        continue;                      // being created right now; try again next rescan
      }

      // ⚠ The allowlist is matched against the METADATA, never the filename.
      // airsim_Go2_1_fisheye_0 cannot be split back into (Go2_1, fisheye, 0) by any rule - both
      // the vehicle and the camera name may contain underscores. Parsing the filename produced
      // "Go2/1_fisheye/0", so `topics:=Go2_1/fisheye/0` matched nothing while the published ROS
      // topic name still looked perfectly correct.
      airsim_shm_bridge::StreamFrameMeta meta{};
      std::vector<uint8_t> payload;
      if (!seg->readNewest(meta, payload)) {
        continue;                      // no frame published yet
      }

      const std::string vehicle = fixedName(meta.vehicle, sizeof(meta.vehicle));
      const std::string camera = fixedName(meta.camera, sizeof(meta.camera));
      const std::string stream_topic =
        vehicle + "/" + camera + "/" + std::to_string(static_cast<int>(meta.image_type));

      if (!allow_.empty() && allow_.count(stream_topic) == 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        skipped_.insert(path);
        RCLCPP_INFO(get_logger(), "  (skipping %s - not in allowlist)", stream_topic.c_str());
        continue;
      }

      auto st = std::make_unique<StreamTopic>();
      st->ros_topic = topic_prefix_ + "/" + vehicle + "/" + camera + "/" +
        airsim_shm_bridge::imageTypeName(meta.image_type);
      st->stream_topic = stream_topic;
      st->publisher = create_publisher<sensor_msgs::msg::Image>(st->ros_topic, qos());
      st->segment = std::move(seg);

      RCLCPP_INFO(
        get_logger(), "+ %s  ->  %s  (%ux%u, %s)",
        stream_topic.c_str(), st->ros_topic.c_str(), meta.width, meta.height,
        meta.pixels_as_float ? "float32" : "uint8 rgb");

      std::lock_guard<std::mutex> lock(mutex_);
      topics_.emplace(path, std::move(st));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = topics_.begin(); it != topics_.end(); ) {
      if (found.count(it->first) == 0) {
        RCLCPP_INFO(get_logger(), "- %s (segment gone)", it->second->ros_topic.c_str());
        it = topics_.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = skipped_.begin(); it != skipped_.end(); ) {
      it = (found.count(*it) == 0) ? skipped_.erase(it) : std::next(it);
    }

    if (!allow_.empty()) {
      std::set<std::string> live;
      for (const auto & kv : topics_) {live.insert(kv.second->stream_topic);}
      for (const std::string & want : allow_) {
        if (live.count(want) == 0) {
          RCLCPP_WARN(
            get_logger(), "allowlisted but not found in %s: %s",
            stream_dir_.c_str(), want.c_str());
        }
      }
    }
  }

  rclcpp::QoS qos() const
  {
    rclcpp::QoS q(rclcpp::KeepLast(1));
    if (reliable_) {q.reliable();} else {q.best_effort();}
    q.durability_volatile();
    return q;
  }

  // ---------------------------------------------------------------- the hot path

  void readLoop()
  {
    airsim_shm_bridge::StreamFrameMeta meta{};
    std::vector<uint8_t> payload;

    while (running_ && rclcpp::ok()) {
      std::vector<StreamTopic *> current;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current.reserve(topics_.size());
        for (auto & kv : topics_) {current.push_back(kv.second.get());}
      }

      for (StreamTopic * st : current) {
        const uint64_t n = st->segment->newestIndex();
        if (n == 0 || n == st->last_index) {continue;}
        if (!st->segment->readNewest(meta, payload)) {
          ++st->torn;
          continue;
        }

        // newest_index is monotonic, so its difference is exactly how many frames the writer
        // published while we were not looking. Latest-wins is the design; counting what that costs
        // is the point of publishing the number.
        if (st->last_index != 0 && n > st->last_index + 1) {
          st->dropped += n - st->last_index - 1;
        }
        st->last_index = n;

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        // The CAPTURE instant, sampled in OnEndDraw next to the camera poses - not the instant
        // this node happened to read it. That is what makes images from one batch mutually
        // aligned, and it is why a consumer may trust the stamp for stereo.
        msg->header.stamp.sec = static_cast<int32_t>(meta.timestamp_ns / 1000000000ULL);
        msg->header.stamp.nanosec = static_cast<uint32_t>(meta.timestamp_ns % 1000000000ULL);
        msg->header.frame_id = frame_id_prefix_ +
          fixedName(meta.vehicle, sizeof(meta.vehicle)) + "/" +
          fixedName(meta.camera, sizeof(meta.camera));
        msg->width = meta.width;
        msg->height = meta.height;
        msg->is_bigendian = 0;
        if (meta.pixels_as_float) {
          msg->encoding = "32FC1";                 // metres, DepthPlanar
          msg->step = meta.width * 4;
        } else {
          // ⚠ RGB, not BGR. AirSim's image_data_uint8 is already RGB; the reflexive channel
          // reversal is wrong here and has been got wrong twice before.
          msg->encoding = "rgb8";
          msg->step = meta.width * 3;
        }
        msg->data = payload;                       // vector copy: a memcpy, ~0.1 ms at 1 MB

        st->publisher->publish(std::move(msg));
        ++st->published;
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(poll_sec_));
    }
  }

  // ---------------------------------------------------------------- diagnostics

  void report()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (topics_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "no segments in %s - is `airsim.StreamSink 2` set and `airsim.StreamCaptureHz` > 0?",
        stream_dir_.c_str());
      return;
    }
    for (const auto & kv : topics_) {
      const StreamTopic & st = *kv.second;
      RCLCPP_INFO(
        get_logger(), "%-46s published=%-7lu dropped=%-6lu torn=%lu",
        st.ros_topic.c_str(),
        static_cast<unsigned long>(st.published),
        static_cast<unsigned long>(st.dropped),
        static_cast<unsigned long>(st.torn));
    }
  }

  std::string stream_dir_;
  std::string topic_prefix_;
  std::string frame_id_prefix_;
  std::set<std::string> allow_;
  double poll_sec_ = 0.002;
  bool reliable_ = false;

  std::mutex mutex_;
  std::map<std::string, std::unique_ptr<StreamTopic>> topics_;
  std::set<std::string> skipped_;

  std::atomic<bool> running_{false};
  std::thread reader_;
  rclcpp::TimerBase::SharedPtr rescan_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShmBridge>());
  rclcpp::shutdown();
  return 0;
}

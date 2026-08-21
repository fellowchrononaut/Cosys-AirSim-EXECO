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
#include <algorithm>
#include <cmath>
#include <limits>

#include "sensor_msgs/msg/camera_info.hpp"

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

namespace
{
/// Normalised image radius a Double Sphere camera produces for an incidence angle theta.
///
/// For a unit ray at angle theta:  x = sin(theta), z = cos(theta)
///   d1 = 1,  d2 = sqrt(sin^2(theta) + (xi + cos(theta))^2)
///   r/fx = sin(theta) / (alpha*d2 + (1-alpha)*(xi + cos(theta)))
double dsRadiusNorm(double theta, double xi, double alpha)
{
  const double st = std::sin(theta), ct = std::cos(theta);
  const double d2 = std::sqrt(st * st + (xi + ct) * (xi + ct));
  const double den = alpha * d2 + (1.0 - alpha) * (xi + ct);
  if (den <= 1e-9) {return std::numeric_limits<double>::quiet_NaN();}
  return st / den;
}

/// Least-squares Kannala-Brandt (equidistant) fit to a Double Sphere camera.
///
/// ⚠ THIS IS AN APPROXIMATION AND IS NEVER PUBLISHED WITHOUT ITS RESIDUAL. ROS has no standard
/// distortion_model for double sphere, and many SLAM front-ends (ORB-SLAM3, VINS, OpenVINS) accept
/// only `equidistant`. Emitting a fit silently would be the exact failure this project keeps
/// hitting: intrinsics that look plausible, reproject almost right, and are wrong. So the caller
/// gets max_residual_px back and refuses the fit if it is too large.
///
/// KB4:  r/f = theta + k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9   (linear in k, so this is
/// an ordinary 4x4 normal-equation solve, not an iterative optimisation).
bool fitKannalaBrandt(
  double xi, double alpha, double fx, double corner_radius_px,
  double k_out[4], double & max_residual_px, double & theta_max_out)
{
  // ⚠ STOP AT THE PEAK, NOT AT THE IMAGE CORNER. r(theta) for a double sphere is NOT monotonic:
  // it rises, peaks, and falls back to zero at theta = pi. A wide camera's projected circle can
  // therefore never reach the image CORNER at all — measured on the Go2_1 fisheye, r peaks at
  // 334.9 px against a 409.8 px corner, which is precisely the black corners and ~8% NaN that
  // camera shows. Searching for "where r reaches the corner" then walked all the way to pi and
  // asked the fit to follow a curve that turns around, inflating the residual from 5.3 px to
  // 6.9 px. Past the peak the projection is not invertible and no pixel maps there, so it is not
  // part of the camera.
  double theta_max = 0.0, r_prev = -1.0;
  for (double t = 0.0; t <= 3.14159265358979; t += 0.0005) {
    const double r = dsRadiusNorm(t, xi, alpha);
    if (!std::isfinite(r) || r < r_prev) {break;}   // past the peak: the projection folds back
    r_prev = r;
    theta_max = t;
    if (r * fx >= corner_radius_px) {break;}        // or the image ran out first
  }
  theta_max_out = theta_max;
  if (theta_max < 0.1) {return false;}

  // Normal equations for the 4 odd powers above the linear term.
  double A[4][4] = {}, b[4] = {};
  const int N = 512;
  for (int i = 0; i < N; ++i) {
    const double th = theta_max * (i + 1) / N;
    const double r = dsRadiusNorm(th, xi, alpha);
    if (!std::isfinite(r)) {continue;}
    const double basis[4] = {
      th * th * th, std::pow(th, 5), std::pow(th, 7), std::pow(th, 9)};
    const double target = r - th;
    for (int a2 = 0; a2 < 4; ++a2) {
      b[a2] += basis[a2] * target;
      for (int c = 0; c < 4; ++c) {A[a2][c] += basis[a2] * basis[c];}
    }
  }

  // Gaussian elimination with partial pivoting. Four unknowns; Eigen would be a dependency for
  // nothing.
  for (int c = 0; c < 4; ++c) {
    int piv = c;
    for (int r2 = c + 1; r2 < 4; ++r2) {if (std::fabs(A[r2][c]) > std::fabs(A[piv][c])) {piv = r2;}}
    if (std::fabs(A[piv][c]) < 1e-18) {return false;}
    if (piv != c) {
      for (int k = 0; k < 4; ++k) {std::swap(A[c][k], A[piv][k]);}
      std::swap(b[c], b[piv]);
    }
    for (int r2 = c + 1; r2 < 4; ++r2) {
      const double f = A[r2][c] / A[c][c];
      for (int k = c; k < 4; ++k) {A[r2][k] -= f * A[c][k];}
      b[r2] -= f * b[c];
    }
  }
  for (int c = 3; c >= 0; --c) {
    double v = b[c];
    for (int k = c + 1; k < 4; ++k) {v -= A[c][k] * k_out[k];}
    k_out[c] = v / A[c][c];
  }

  max_residual_px = 0.0;
  for (int i = 0; i <= N; ++i) {
    const double th = theta_max * i / N;
    const double r = dsRadiusNorm(th, xi, alpha);
    if (!std::isfinite(r)) {continue;}
    const double kb = th + k_out[0] * th * th * th + k_out[1] * std::pow(th, 5) +
      k_out[2] * std::pow(th, 7) + k_out[3] * std::pow(th, 9);
    max_residual_px = std::max(max_residual_px, std::fabs(kb - r) * fx);
  }
  return std::isfinite(max_residual_px);
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

    // ⚠ EXPLICIT, AND LOGGED, because there is no right answer and a silent default would be a
    // trap. ROS defines no distortion_model for double sphere.
    //   double_sphere   - the truth. distortion_model="double_sphere", D=[xi,alpha]. Consumers that
    //                     do not know it should refuse; Basalt and Kalibr-derived pipelines do know it.
    //   equidistant_fit - a Kannala-Brandt least-squares fit, for ORB-SLAM3 / VINS / OpenVINS which
    //                     accept only `equidistant`. Published ONLY with its measured residual, and
    //                     refused outright above fisheye_fit_max_px.
    //   none            - publish no CameraInfo for fisheye cameras at all.
    fisheye_model_ = declare_parameter<std::string>("fisheye_camera_info_model", "double_sphere");
    fisheye_fit_max_px_ = declare_parameter<double>("fisheye_fit_max_px", 1.0);
    if (fisheye_model_ != "double_sphere" && fisheye_model_ != "equidistant_fit" &&
      fisheye_model_ != "none")
    {
      RCLCPP_ERROR(
        get_logger(), "fisheye_camera_info_model='%s' is not one of "
        "double_sphere|equidistant_fit|none - falling back to double_sphere",
        fisheye_model_.c_str());
      fisheye_model_ = "double_sphere";
    }
    RCLCPP_INFO(
      get_logger(), "fisheye CameraInfo model: %s (fit refused above %.2f px residual)",
      fisheye_model_.c_str(), fisheye_fit_max_px_);

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
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher;
    sensor_msgs::msg::CameraInfo info;      // built once at discovery; only the stamp changes
    bool info_valid = false;
    std::string ros_topic;
    std::string stream_topic;
    uint64_t last_index = 0;
    uint64_t published = 0;
    uint64_t dropped = 0;
    uint64_t torn = 0;
  };

  /// Build the CameraInfo for one stream from the intrinsics that travelled with the frame.
  /// Returns false when no usable model was declared - in which case NO CameraInfo is published at
  /// all, which is the honest outcome: a consumer that receives nothing knows it received nothing,
  /// whereas a fabricated pinhole guess is indistinguishable from a calibrated camera.
  bool buildCameraInfo(
    const airsim_shm_bridge::StreamFrameMeta & meta, const std::string & frame_id,
    const std::string & stream_topic, sensor_msgs::msg::CameraInfo & info)
  {
    const auto & cm = meta.camera_model;
    if (cm.type == airsim_shm_bridge::kModelNone) {return false;}

    double fx = cm.fx, fy = cm.fy, cx = cm.cx, cy = cm.cy;

    // ⚠ SCALE IF THE FRAME IS NOT THE RESOLUTION THE INTRINSICS WERE AUTHORED AT. CaptureSettings
    // is per image type, so one camera's depth and scene streams can legitimately differ in size
    // while sharing a single CameraModel block. Publishing unscaled intrinsics against a resized
    // frame is silently wrong in exactly the way nothing downstream can detect - which is why both
    // resolutions ride in the segment.
    if (cm.model_width > 0 && cm.model_height > 0 &&
      (cm.model_width != meta.width || cm.model_height != meta.height))
    {
      const double sx = static_cast<double>(meta.width) / cm.model_width;
      const double sy = static_cast<double>(meta.height) / cm.model_height;
      fx *= sx; cx *= sx; fy *= sy; cy *= sy;
      RCLCPP_INFO(
        get_logger(), "  %s: intrinsics authored at %ux%u, frame is %ux%u - scaled by %.4f/%.4f",
        stream_topic.c_str(), cm.model_width, cm.model_height, meta.width, meta.height, sx, sy);
    }

    info = sensor_msgs::msg::CameraInfo();
    info.header.frame_id = frame_id;
    info.width = meta.width;
    info.height = meta.height;
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    // ⚠ R and P are PLACEHOLDERS. They are only meaningful after rectification, and nothing here
    // rectifies. Identity and [K|0] are the conventional "unrectified" fillers; left zeroed they
    // would look like a bug to every consumer that reads them.
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

    switch (cm.type) {
      case airsim_shm_bridge::kModelPinhole:
        info.distortion_model = "plumb_bob";
        info.d.assign(5, 0.0);              // the sim renders an ideal pinhole: no distortion
        return true;

      case airsim_shm_bridge::kModelKannalaBrandt:
        info.distortion_model = "equidistant";
        info.d = {cm.params[0], cm.params[1], cm.params[2], cm.params[3]};
        return true;

      case airsim_shm_bridge::kModelDoubleSphere: {
          if (fisheye_model_ == "none") {return false;}
          if (fisheye_model_ == "double_sphere") {
            // Not a ROS-standard string. That is deliberate: a consumer that does not understand it
            // should refuse rather than quietly treat D as plumb_bob coefficients.
            info.distortion_model = "double_sphere";
            info.d = {cm.params[0], cm.params[1]};
            return true;
          }
          double k[4] = {}, resid = 0.0, theta_max = 0.0;
          const double corner = std::hypot(meta.width * 0.5, meta.height * 0.5);
          if (!fitKannalaBrandt(cm.params[0], cm.params[1], fx, corner, k, resid, theta_max)) {
            RCLCPP_ERROR(
              get_logger(), "  %s: Kannala-Brandt fit FAILED - publishing no CameraInfo",
              stream_topic.c_str());
            return false;
          }
          if (resid > fisheye_fit_max_px_) {
            RCLCPP_ERROR(
              get_logger(),
              "  %s: KB fit residual %.3f px exceeds fisheye_fit_max_px %.2f - REFUSING to publish "
              "an approximation this poor. Use fisheye_camera_info_model:=double_sphere.",
              stream_topic.c_str(), resid, fisheye_fit_max_px_);
            return false;
          }
          RCLCPP_WARN(
            get_logger(),
            "  %s: publishing an APPROXIMATION - Kannala-Brandt fit to double sphere, "
            "max residual %.4f px over theta<=%.1f deg (k=[%.6f %.6f %.6f %.6f])",
            stream_topic.c_str(), resid, theta_max * 180.0 / M_PI, k[0], k[1], k[2], k[3]);
          info.distortion_model = "equidistant";
          info.d = {k[0], k[1], k[2], k[3]};
          return true;
        }

      default:
        return false;                        // Raymap has no closed-form CameraInfo
    }
  }

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

      // ⚠ Sibling of the IMAGE topic, not of the camera. Intrinsics are per camera, but width and
      // height are per stream (CaptureSettings is per image type), so one shared camera_info could
      // describe a different resolution than the image beside it. Correctness beats the
      // <base>/camera_info convention here; remap if your consumer insists.
      const std::string frame_id = frame_id_prefix_ +
        fixedName(meta.vehicle, sizeof(meta.vehicle)) + "/" +
        fixedName(meta.camera, sizeof(meta.camera));
      st->info_valid = buildCameraInfo(meta, frame_id, stream_topic, st->info);
      if (st->info_valid) {
        st->info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>(
          st->ros_topic + "/camera_info", qos());
        RCLCPP_INFO(
          get_logger(), "  %s/camera_info: %s fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
          st->ros_topic.c_str(), st->info.distortion_model.c_str(),
          st->info.k[0], st->info.k[4], st->info.k[2], st->info.k[5]);
      } else {
        RCLCPP_INFO(
          get_logger(), "  %s: no CameraInfo (no usable camera model declared)",
          st->ros_topic.c_str());
      }

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
        // Same stamp as the image, so a consumer can pair them exactly rather than by arrival.
        if (st->info_valid && st->info_publisher) {
          st->info.header.stamp.sec = static_cast<int32_t>(meta.timestamp_ns / 1000000000ULL);
          st->info.header.stamp.nanosec = static_cast<uint32_t>(meta.timestamp_ns % 1000000000ULL);
          st->info_publisher->publish(st->info);
        }
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
  std::string fisheye_model_;
  double fisheye_fit_max_px_ = 1.0;
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

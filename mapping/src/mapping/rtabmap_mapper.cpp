#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>

namespace fs = std::filesystem;

namespace {

constexpr double kRgbDepthToleranceS = 0.005;
constexpr double kPoseToleranceS = 0.050;
constexpr double kKeyframePeriodS = 0.200;
constexpr std::size_t kMaxBufferedMessages = 12;
std::atomic_bool g_running{true};

void onSignal(int) { g_running.store(false); }

template <typename Message>
double messageStamp(const Message &message) {
  if (!message.has_header() || !message.header().has_stamp()) return 0.0;
  const auto &stamp = message.header().stamp();
  return static_cast<double>(stamp.sec()) + stamp.nsec() * 1e-9;
}

std::string envOr(const char *name, const std::string &fallback) {
  const char *value = std::getenv(name);
  return value && *value ? std::string(value) : fallback;
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

void loadParameters(const fs::path &path, rtabmap::ParametersMap &parameters) {
  if (!fs::exists(path)) return;
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    const auto comment = line.find_first_of("#;");
    if (comment != std::string::npos) line.resize(comment);
    const auto separator = line.find('=');
    if (separator == std::string::npos) continue;
    const auto key = trim(line.substr(0, separator));
    const auto value = trim(line.substr(separator + 1));
    if (!key.empty() && !value.empty()) parameters[key] = value;
  }
}

struct PoseSample {
  double stamp = 0.0, x = 0.0, y = 0.0, z = 0.0;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  rtabmap::Transform transform;
};

struct Sample {
  double stamp = 0.0;
  gz::msgs::Image rgb, depth;
  PoseSample pose;
};

struct StampedImage {
  double stamp = 0.0;
  gz::msgs::Image message;
};

class RtabmapAdapter {
public:
  RtabmapAdapter()
      : model_(envOr("RTABMAP_MODEL", "x500_depth_0")),
        rgb_topic_(envOr("RTABMAP_RGB_TOPIC", "/rgbd_camera/image")),
        depth_topic_(envOr("RTABMAP_DEPTH_TOPIC", "/rgbd_camera/depth_image")) {
    const std::string world = envOr("RTABMAP_WORLD", "drone_world");
    pose_topic_ = envOr(
        "RTABMAP_POSE_TOPIC", "/world/" + world + "/dynamic_pose/info");
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string session = envOr(
        "RTABMAP_SESSION_ID", "scan_" + std::to_string(timestamp));
    const fs::path db_dir = envOr("RTABMAP_DB_DIR", "runtime/maps");
    database_path_ = db_dir / (session + ".db");
    output_base_ = database_path_;
    output_base_.replace_extension();
  }

  int run() {
    fs::create_directories(database_path_.parent_path());
    subscribe();
    initialize();
    std::cerr << "[rtabmap_mapper] waiting for RGB-D and pose messages\n";
    while (g_running.load()) {
      Sample sample;
      if (!waitForSample(sample)) continue;
      if (sample.stamp - last_processed_stamp_ < kKeyframePeriodS) {
        ++dropped_frames_;
        continue;
      }
      if (!movedSinceLastKeyframe(sample)) continue;
      process(sample);
    }
    return shutdown() ? 0 : 1;
  }

private:
  template <typename Callback>
  void subscribe(const std::string &topic, Callback callback, const char *name) {
    if (!node_.Subscribe(topic, callback, this)) {
      throw std::runtime_error(std::string("failed to subscribe to ") + name +
                               " topic: " + topic);
    }
  }

  void subscribe() {
    subscribe(rgb_topic_, &RtabmapAdapter::onRgb, "RGB");
    subscribe(depth_topic_, &RtabmapAdapter::onDepth, "depth");
    subscribe(pose_topic_, &RtabmapAdapter::onPose, "pose");
    std::cerr << "[rtabmap_mapper] RGB:   " << rgb_topic_ << "\n"
              << "[rtabmap_mapper] depth: " << depth_topic_ << "\n"
              << "[rtabmap_mapper] pose:  " << pose_topic_ << "\n";
  }

  void initialize() {
    rtabmap::ParametersMap parameters;
    const auto set = [&parameters](const std::string &key, const char *value) {
      parameters[key] = value;
    };
    set(rtabmap::Parameters::kMemIncrementalMemory(), "true");
    set(rtabmap::Parameters::kRGBDCreateOccupancyGrid(), "true");
    set(rtabmap::Parameters::kRtabmapDetectionRate(), "5");
    set(rtabmap::Parameters::kGridSensor(), "1");
    set(rtabmap::Parameters::kGridDepthDecimation(), "2");
    set(rtabmap::Parameters::kGridCellSize(), "0.10");
    set(rtabmap::Parameters::kGridRangeMax(), "8.0");
    set(rtabmap::Parameters::kGridRayTracing(), "true");
    set(rtabmap::Parameters::kGrid3D(), "false");
    set(rtabmap::Parameters::kGridMapFrameProjection(), "true");
    loadParameters(envOr("RTABMAP_CONFIG", "src/mapping/rtabmap.ini"), parameters);
    if (const auto it = parameters.find("RGBD/LinearUpdate");
        it != parameters.end()) linear_update_m_ = std::stod(it->second);
    if (const auto it = parameters.find("RGBD/AngularUpdate");
        it != parameters.end()) angular_update_rad_ = std::stod(it->second);
    rtabmap_.init(parameters, database_path_.string(), false);
    occupancy_grid_ = std::make_unique<rtabmap::OccupancyGrid>(&local_grids_, parameters);
    std::cerr << "[rtabmap_mapper] database: " << database_path_ << "\n"
              << "[rtabmap_mapper] keyframe thresholds: " << linear_update_m_
              << "m, " << angular_update_rad_ << "rad\n";
  }

  void onRgb(const gz::msgs::Image &message) { pushImage(rgb_buffer_, message); }
  void onDepth(const gz::msgs::Image &message) { pushImage(depth_buffer_, message); }

  void onPose(const gz::msgs::Pose_V &message) {
    const double stamp = messageStamp(message);
    if (stamp <= 0.0) {
      ++dropped_frames_;
      return;
    }
    const auto pose = std::find_if(
        message.pose().begin(), message.pose().end(), [this](const auto &value) {
          return value.name() == model_;
        });
    if (pose == message.pose().end() || !pose->has_position() ||
        !pose->has_orientation()) return;
    const auto &p = pose->position();
    const auto &q = pose->orientation();
    std::lock_guard<std::mutex> lock(mutex_);
    pose_buffer_.push_back({
        stamp, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w(),
        rtabmap::Transform(
            static_cast<float>(p.x()), static_cast<float>(p.y()),
            static_cast<float>(p.z()), static_cast<float>(q.x()),
            static_cast<float>(q.y()), static_cast<float>(q.z()),
            static_cast<float>(q.w()))});
    if (pose_buffer_.size() > kMaxBufferedMessages * 2) pose_buffer_.pop_front();
    condition_.notify_one();
  }

  void pushImage(std::deque<StampedImage> &buffer,
                 const gz::msgs::Image &message) {
    const double stamp = messageStamp(message);
    if (stamp <= 0.0) {
      ++dropped_frames_;
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    buffer.push_back({stamp, message});
    while (buffer.size() > kMaxBufferedMessages) {
      buffer.pop_front();
      ++dropped_frames_;
    }
    condition_.notify_one();
  }

  bool waitForSample(Sample &sample) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, std::chrono::milliseconds(100), [this] {
      return !g_running.load() || (!rgb_buffer_.empty() &&
             !depth_buffer_.empty() && !pose_buffer_.empty());
    });
    return g_running.load() && takeSampleLocked(sample);
  }

  bool takeSampleLocked(Sample &sample) {
    if (rgb_buffer_.empty() || depth_buffer_.empty() || pose_buffer_.empty()) {
      return false;
    }

    std::size_t rgb_index = 0;
    std::size_t depth_index = 0;
    double best_difference = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < rgb_buffer_.size(); ++i) {
      for (std::size_t j = 0; j < depth_buffer_.size(); ++j) {
        const double difference =
            std::abs(rgb_buffer_[i].stamp - depth_buffer_[j].stamp);
        if (difference < best_difference) {
          best_difference = difference;
          rgb_index = i;
          depth_index = j;
        }
      }
    }

    if (best_difference > kRgbDepthToleranceS) {
      if (rgb_buffer_.front().stamp < depth_buffer_.front().stamp) {
        rgb_buffer_.pop_front();
      } else {
        depth_buffer_.pop_front();
      }
      ++dropped_frames_;
      return false;
    }

    const double stamp =
        (rgb_buffer_[rgb_index].stamp + depth_buffer_[depth_index].stamp) / 2.0;
    std::size_t pose_index = 0;
    double pose_difference = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < pose_buffer_.size(); ++i) {
      const double difference = std::abs(pose_buffer_[i].stamp - stamp);
      if (difference < pose_difference) {
        pose_difference = difference;
        pose_index = i;
      }
    }
    if (pose_difference > kPoseToleranceS) {
      if (pose_buffer_.front().stamp < stamp) {
        pose_buffer_.pop_front();
      } else {
        rgb_buffer_.pop_front();
        depth_buffer_.pop_front();
      }
      ++dropped_frames_;
      return false;
    }

    sample.stamp = stamp;
    sample.rgb = std::move(rgb_buffer_[rgb_index].message);
    sample.depth = std::move(depth_buffer_[depth_index].message);
    sample.pose = pose_buffer_[pose_index];
    rgb_buffer_.erase(rgb_buffer_.begin(), rgb_buffer_.begin() + rgb_index + 1);
    depth_buffer_.erase(
        depth_buffer_.begin(), depth_buffer_.begin() + depth_index + 1);
    pose_buffer_.erase(pose_buffer_.begin(), pose_buffer_.begin() + pose_index + 1);
    return true;
  }

  static cv::Mat decodeImage(const gz::msgs::Image &message,
                             int bytes_per_pixel, int type, const char *name) {
    const int width = message.width(), height = message.height();
    const int step = message.step() > 0 ? message.step() : width * bytes_per_pixel;
    if (width <= 0 || height <= 0 || step < width * bytes_per_pixel ||
        message.data().size() < static_cast<std::size_t>(step * height)) {
      throw std::runtime_error(std::string("invalid ") + name +
                               " image dimensions or payload");
    }
    return cv::Mat(height, width, type,
                   const_cast<char *>(message.data().data()), step).clone();
  }

  static cv::Mat decodeRgb(const gz::msgs::Image &message) {
    const int format = message.pixel_format_type();
    const bool four_channel = format == gz::msgs::RGBA_INT8 ||
                              format == gz::msgs::BGRA_INT8;
    cv::Mat rgb = decodeImage(message, four_channel ? 4 : 3,
                              four_channel ? CV_8UC4 : CV_8UC3, "RGB");
    if (format == gz::msgs::RGBA_INT8) cv::cvtColor(rgb, rgb, cv::COLOR_RGBA2BGR);
    else if (format == gz::msgs::BGRA_INT8) cv::cvtColor(rgb, rgb, cv::COLOR_BGRA2BGR);
    else if (format != gz::msgs::BGR_INT8) cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
    return rgb;
  }

  static cv::Mat decodeDepth(const gz::msgs::Image &message) {
    if (message.pixel_format_type() != gz::msgs::R_FLOAT32) {
      throw std::runtime_error("depth image is not R_FLOAT32");
    }
    return decodeImage(message, 4, CV_32FC1, "depth");
  }

  static rtabmap::CameraModel cameraModel(int width, int height) {
    constexpr double fov = 1.274;
    const double focal = (width / 2.0) / std::tan(fov / 2.0);
    const rtabmap::Transform sensor_to_body(
        0.0f, 0.0f, 1.0f, 0.13233f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        -1.0f, 0.0f, 0.26078f);
    return rtabmap::CameraModel(
        "OakD_RGBD", focal, focal, width / 2.0, height / 2.0,
        sensor_to_body, 0.0, cv::Size(width, height));
  }

  void process(const Sample &sample) {
    try {
      cv::Mat rgb = decodeRgb(sample.rgb);
      cv::Mat depth = decodeDepth(sample.depth);
      if (rgb.size() != depth.size())
        throw std::runtime_error("RGB and depth sizes do not match");
      rtabmap::SensorData data(
          rgb, depth, cameraModel(rgb.cols, rgb.rows), next_node_id_++, sample.stamp);
      if (!rtabmap_.process(data, sample.pose.transform, 0.0001f, 0.0001f))
        throw std::runtime_error("Rtabmap::process returned false");
      last_processed_stamp_ = sample.stamp;
      last_keyframe_pose_ = sample.pose;
      ++processed_frames_;
      updateOccupancy();
    } catch (const std::exception &exception) {
      ++dropped_frames_;
      std::cerr << "[rtabmap_mapper] dropping frame: " << exception.what() << "\n";
    }
  }

  bool movedSinceLastKeyframe(const Sample &sample) const {
    if (processed_frames_ == 0) return true;
    const double translation = std::hypot(
        sample.pose.x - last_keyframe_pose_.x,
        sample.pose.y - last_keyframe_pose_.y,
        sample.pose.z - last_keyframe_pose_.z);
    const double dot = std::clamp(std::abs(
        sample.pose.qx * last_keyframe_pose_.qx +
        sample.pose.qy * last_keyframe_pose_.qy +
        sample.pose.qz * last_keyframe_pose_.qz +
        sample.pose.qw * last_keyframe_pose_.qw), 0.0, 1.0);
    return translation >= linear_update_m_ ||
           2.0 * std::acos(dot) >= angular_update_rad_;
  }

  void updateOccupancy() {
    const auto &statistics = rtabmap_.getStatistics();
    const auto &signature = statistics.getLastSignatureData();
    if (signature.id() <= 0 || signature.sensorData().gridCellSize() <= 0.0f ||
        !added_grid_nodes_.insert(signature.id()).second) return;
    cv::Mat ground, obstacles, empty;
    signature.sensorData().uncompressDataConst(
        0, 0, 0, 0, &ground, &obstacles, &empty);
    local_grids_.add(signature.id(), ground, obstacles, empty,
                     signature.sensorData().gridCellSize(),
                     signature.sensorData().gridViewPoint());
    occupancy_grid_->update(statistics.poses());
    const int loop_id = rtabmap_.getLoopClosureId();
    if (loop_id > last_loop_closure_id_) {
      last_loop_closure_id_ = loop_id;
      ++loop_closure_count_;
    }
    float x_min = 0.0f, y_min = 0.0f;
    const cv::Mat map = occupancy_grid_->getMap(x_min, y_min);
    publish(map, x_min, y_min, occupancy_grid_->getCellSize());
  }

  void publish(const cv::Mat &map,
               float x_min,
               float y_min,
               float cell_size) {
    if (map.empty() || cell_size <= 0.0f || map.total() > UINT32_MAX) return;
    cv::Mat image(map.size(), CV_8UC1, cv::Scalar(205));
    for (int row = 0; row < map.rows; ++row) {
      for (int column = 0; column < map.cols; ++column) {
        const auto value = map.at<signed char>(row, column);
        image.at<unsigned char>(row, column) =
            value < 0 ? 205 : value >= 65 ? 0 : 254;
      }
    }
    const fs::path image_path = output_base_.string() + ".pgm";
    const fs::path yaml_path = output_base_.string() + ".yaml";
    if (!cv::imwrite(image_path.string(), image))
      throw std::runtime_error("failed to write occupancy image");
    std::ofstream yaml(yaml_path);
    if (!yaml) throw std::runtime_error("failed to write occupancy metadata");
    yaml << "image: " << image_path.filename().string() << "\n"
         << "resolution: " << cell_size << "\n"
         << "origin: [" << x_min << ", " << y_min << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";

  }

  bool shutdown() {
    try {
      rtabmap_.close(true);
      std::cerr << "[rtabmap_mapper] saved database: " << database_path_
                << " dropped=" << dropped_frames_.load()
                << " loops=" << loop_closure_count_ << "\n";
      return true;
    } catch (const std::exception &exception) {
      std::cerr << "[rtabmap_mapper] failed to close database: "
                << exception.what() << "\n";
      return false;
    }
  }

  std::string model_, rgb_topic_, depth_topic_, pose_topic_;
  fs::path database_path_, output_base_;
  gz::transport::Node node_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<StampedImage> rgb_buffer_, depth_buffer_;
  std::deque<PoseSample> pose_buffer_;
  rtabmap::Rtabmap rtabmap_;
  rtabmap::LocalGridCache local_grids_;
  std::unique_ptr<rtabmap::OccupancyGrid> occupancy_grid_;
  std::set<int> added_grid_nodes_;
  PoseSample last_keyframe_pose_;
  double linear_update_m_ = 0.10, angular_update_rad_ = 0.10;
  double last_processed_stamp_ = -std::numeric_limits<double>::max();
  int next_node_id_ = 1, loop_closure_count_ = 0, last_loop_closure_id_ = 0;
  std::uint64_t processed_frames_ = 0;
  std::atomic_uint64_t dropped_frames_{0};
};

}  // namespace

int main() {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  try {
    return RtabmapAdapter().run();
  } catch (const std::exception &exception) {
    std::cerr << "[rtabmap_mapper] fatal: " << exception.what() << "\n";
    return 1;
  }
}

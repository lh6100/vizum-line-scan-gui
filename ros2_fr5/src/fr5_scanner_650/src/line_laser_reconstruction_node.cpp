#include "HikCalibrationCore.h"
#include "world_height_color.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

struct CloudPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  float confidence{0.0F};
};

struct VoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto mix = [](std::uint64_t value) {
        value ^= value >> 30U;
        value *= 0xbf58476d1ce4e5b9ULL;
        value ^= value >> 27U;
        value *= 0x94d049bb133111ebULL;
        return value ^ (value >> 31U);
      };
    return static_cast<std::size_t>(
      mix(static_cast<std::uint64_t>(key.x)) ^
      (mix(static_cast<std::uint64_t>(key.y)) << 1U) ^
      (mix(static_cast<std::uint64_t>(key.z)) << 2U));
  }
};

struct VoxelAccumulator
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double intensity{0.0};
  double confidence{0.0};
  std::uint32_t count{0};
};

using VoxelMap = std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash>;

struct ScanStatisticsSnapshot
{
  std::uint64_t frames_enqueued{0U};
  std::uint64_t profiles_reconstructed{0U};
  std::uint64_t profiles_rejected{0U};
  std::uint64_t queue_dropped{0U};
  std::uint64_t queue_high_water{0U};
  std::uint64_t timestamp_gap_events{0U};
  std::uint64_t estimated_missing_frames{0U};
  std::uint64_t preview_updates_skipped{0U};
  std::uint64_t tf_accepted{0U};
  std::uint64_t tf_rejected{0U};
  std::int64_t first_image_stamp_ns{0};
  std::int64_t last_image_stamp_ns{0};
  std::size_t voxel_count{0U};
  bool valid{true};
  std::vector<std::string> invalid_reasons;
  std::string camera_status_start;
  std::string camera_status_end;
};

struct SaveJob
{
  std::string session_id;
  VoxelMap voxels;
  ScanStatisticsSnapshot statistics;
};

struct ReconstructionTask
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  std::uint64_t sequence{0U};
  std::uint64_t accumulation_epoch{0U};
};

std::string make_scan_session_id()
{
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream text;
  text << "scan_" << std::put_time(&local, "%Y%m%d_%H%M%S") << '_'
       << std::setw(3) << std::setfill('0') << milliseconds.count();
  return text.str();
}

std::string sanitize_ply_comment(std::string value)
{
  std::replace(value.begin(), value.end(), '\n', ' ');
  std::replace(value.begin(), value.end(), '\r', ' ');
  return value;
}

std::string yaml_double_quoted(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size() + 2U);
  escaped.push_back('"');
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped.push_back(character);
        break;
    }
  }
  escaped.push_back('"');
  return escaped;
}

bool host_is_little_endian()
{
  const std::uint16_t value = 1U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 1U;
}

template<typename Value>
void append_little_endian(std::vector<std::uint8_t> * output, Value value)
{
  if (!output) {
    return;
  }
  std::array<std::uint8_t, sizeof(Value)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(Value));
  if (!host_is_little_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  output->insert(output->end(), bytes.begin(), bytes.end());
}

float packed_rgb(const fr5_scanner_650::world_height_color::Rgb & color)
{
  const std::uint32_t packed =
    (static_cast<std::uint32_t>(color.red) << 16U) |
    (static_cast<std::uint32_t>(color.green) << 8U) |
    static_cast<std::uint32_t>(color.blue);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(packed), "PointCloud2 RGB must be 32-bit");
  std::memcpy(&value, &packed, sizeof(value));
  return value;
}

bool compute_height_range(
  const std::vector<CloudPoint> & points, double lower_percentile,
  double upper_percentile, fr5_scanner_650::world_height_color::Range * range,
  std::string * error = nullptr)
{
  std::vector<double> world_z_m;
  world_z_m.reserve(points.size());
  for (const CloudPoint & point : points) {
    world_z_m.push_back(static_cast<double>(point.z));
  }
  return fr5_scanner_650::world_height_color::computeRange(
    std::move(world_z_m), lower_percentile, upper_percentile, range, error);
}

sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<CloudPoint> & points, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp,
  const fr5_scanner_650::world_height_color::Range * height_range = nullptr)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "confidence", 1, sensor_msgs::msg::PointField::FLOAT32,
    "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<float> confidence(cloud, "confidence");
  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb");
  for (const CloudPoint & point : points) {
    *x = point.x;
    *y = point.y;
    *z = point.z;
    *intensity = point.intensity;
    *confidence = point.confidence;
    const auto color = height_range ?
      fr5_scanner_650::world_height_color::colorForWorldZ(point.z, *height_range) :
      fr5_scanner_650::world_height_color::Rgb{};
    *rgb = packed_rgb(color);
    ++x;
    ++y;
    ++z;
    ++intensity;
    ++confidence;
    ++rgb;
  }
  cloud.is_dense = true;
  return cloud;
}

geometry_msgs::msg::Point marker_point(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

class LineLaserReconstructionNode final : public rclcpp::Node
{
public:
  LineLaserReconstructionNode()
  : Node("line_laser_reconstruction"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/scanner_650/image_raw");
    camera_statistics_service_name_ = declare_parameter<std::string>(
      "camera_statistics_service", "/scanner_650/get_camera_statistics");
    profile_topic_ = declare_parameter<std::string>(
      "profile_topic", "/scanner_650/profile_cloud");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scanner_650/scan_cloud");
    preview_topic_ = declare_parameter<std::string>(
      "preview_topic", "/scanner_650/scan_cloud_preview");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/scanner_650/markers");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "hik_camera_optical_frame");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
    intrinsics_path_ = declare_parameter<std::string>("intrinsics_path", "");
    laser_plane_path_ = declare_parameter<std::string>("laser_plane_path", "");
    camera_z_min_mm_ = declare_parameter<double>("camera_z_min_mm", 100.0);
    camera_z_max_mm_ = declare_parameter<double>("camera_z_max_mm", 1000.0);
    voxel_size_m_ = declare_parameter<double>("voxel_size_m", 0.0005);
    preview_voxel_size_m_ = declare_parameter<double>("preview_voxel_size_m", 0.002);
    maximum_voxels_ = declare_parameter<int>("maximum_voxels", 2000000);
    maximum_preview_voxels_ = declare_parameter<int>("maximum_preview_voxels", 200000);
    maximum_pending_save_jobs_ = declare_parameter<int>("maximum_pending_save_jobs", 2);
    publish_every_n_profiles_ = declare_parameter<int>("publish_every_n_profiles", 6);
    scan_publish_period_s_ = declare_parameter<double>("scan_publish_period_s", 0.5);
    expected_camera_fps_ = declare_parameter<double>("expected_camera_fps", 60.0);
    maximum_frame_interval_factor_ = declare_parameter<double>(
      "maximum_frame_interval_factor", 1.8);
    centerline_mode_ = declare_parameter<std::string>("centerline_mode", "legacy");
    reconstruction_queue_capacity_ = declare_parameter<int>(
      "reconstruction_queue_capacity", 64);
    reconstruction_worker_threads_ = declare_parameter<int>(
      "reconstruction_worker_threads", 2);
    clear_cloud_on_accumulation_start_ = declare_parameter<bool>(
      "clear_cloud_on_accumulation_start", true);
    auto_save_on_accumulation_stop_ = declare_parameter<bool>(
      "auto_save_on_accumulation_stop", true);
    require_camera_ready_for_accumulation_ = declare_parameter<bool>(
      "require_camera_ready_for_accumulation", true);
    accumulation_drain_timeout_s_ = declare_parameter<double>(
      "accumulation_drain_timeout_s", 5.0);
    tf_lookup_timeout_s_ = declare_parameter<double>("tf_lookup_timeout_s", 0.10);
    maximum_image_age_s_ = declare_parameter<double>("maximum_image_age_s", 1.0);
    maximum_future_image_lead_s_ = declare_parameter<double>(
      "maximum_future_image_lead_s", 0.02);
    accumulating_ = declare_parameter<bool>("accumulate_on_start", false);
    if (accumulating_) {
      throw std::runtime_error(
              "accumulate_on_start=true is not supported by isolated scan sessions; "
              "start accumulation through /scanner_650/set_accumulation after camera readiness");
    }
    output_ply_ = declare_parameter<std::string>("output_ply", "scan_voxel.ply");
    height_color_lower_percentile_ = declare_parameter<double>(
      "height_color_lower_percentile", 1.0);
    height_color_upper_percentile_ = declare_parameter<double>(
      "height_color_upper_percentile", 99.0);
    if (image_topic_.empty() || camera_statistics_service_name_.empty() ||
      preview_topic_.empty() || camera_frame_.empty() ||
      output_frame_.empty() ||
      intrinsics_path_.empty() || laser_plane_path_.empty() || voxel_size_m_ <= 0.0 ||
      !std::isfinite(preview_voxel_size_m_) || preview_voxel_size_m_ < voxel_size_m_ ||
      !std::isfinite(camera_z_min_mm_) || !std::isfinite(camera_z_max_mm_) ||
      camera_z_min_mm_ <= 0.0 || camera_z_max_mm_ <= camera_z_min_mm_ ||
      maximum_voxels_ < 1 || maximum_preview_voxels_ < 1 ||
      maximum_pending_save_jobs_ < 1 || maximum_pending_save_jobs_ > 16 ||
      publish_every_n_profiles_ < 1 ||
      !std::isfinite(scan_publish_period_s_) || scan_publish_period_s_ <= 0.0 ||
      !std::isfinite(expected_camera_fps_) || expected_camera_fps_ <= 0.0 ||
      !std::isfinite(maximum_frame_interval_factor_) || maximum_frame_interval_factor_ < 1.1 ||
      (centerline_mode_ != "legacy" && centerline_mode_ != "shadow" &&
      centerline_mode_ != "quality") ||
      reconstruction_queue_capacity_ < 2 || reconstruction_queue_capacity_ > 4096 ||
      reconstruction_worker_threads_ < 1 || reconstruction_worker_threads_ > 16 ||
      !std::isfinite(accumulation_drain_timeout_s_) || accumulation_drain_timeout_s_ <= 0.0 ||
      !std::isfinite(tf_lookup_timeout_s_) || tf_lookup_timeout_s_ <= 0.0 ||
      !std::isfinite(maximum_image_age_s_) || maximum_image_age_s_ <= tf_lookup_timeout_s_ ||
      !std::isfinite(maximum_future_image_lead_s_) || maximum_future_image_lead_s_ < 0.001 ||
      !std::isfinite(height_color_lower_percentile_) ||
      !std::isfinite(height_color_upper_percentile_) ||
      height_color_lower_percentile_ < 0.0 ||
      height_color_lower_percentile_ >= height_color_upper_percentile_ ||
      height_color_upper_percentile_ > 100.0)
    {
      throw std::runtime_error("invalid scanner_650 reconstruction parameters");
    }

    load_calibration();
    configure_reconstruction();
    if (camera_z_min_mm_ < laser_metadata_.validCameraZMinMm ||
      camera_z_max_mm_ > laser_metadata_.validCameraZMaxMm)
    {
      RCLCPP_WARN(
        get_logger(),
        "configured camera Z %.1f..%.1f mm extends beyond calibration-validated %.1f..%.1f mm; "
        "extended points have no formal calibration accuracy guarantee",
        camera_z_min_mm_, camera_z_max_mm_,
        laser_metadata_.validCameraZMinMm, laser_metadata_.validCameraZMaxMm);
    }

    profile_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      profile_topic_, rclcpp::QoS(5).reliable());
    scan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::QoS(1).reliable().transient_local());
    preview_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      preview_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).reliable().transient_local());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/scanner_650/reconstruction_status", rclcpp::QoS(10).reliable().transient_local());
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(
        &LineLaserReconstructionNode::on_parameters, this,
        std::placeholders::_1));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::QoS(
        static_cast<std::size_t>(reconstruction_queue_capacity_)).reliable(),
      std::bind(&LineLaserReconstructionNode::on_image, this, std::placeholders::_1));
    camera_status_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions camera_status_options;
    camera_status_options.callback_group = camera_status_callback_group_;
    camera_status_subscription_ = create_subscription<std_msgs::msg::String>(
      "/scanner_650/camera_status", rclcpp::QoS(10).reliable().transient_local(),
      std::bind(
        &LineLaserReconstructionNode::on_camera_status, this, std::placeholders::_1),
      camera_status_options);
    camera_statistics_client_ = create_client<std_srvs::srv::Trigger>(
      camera_statistics_service_name_, rmw_qos_profile_services_default,
      camera_status_callback_group_);

    accumulation_service_ = create_service<std_srvs::srv::SetBool>(
      "/scanner_650/set_accumulation",
      std::bind(
        &LineLaserReconstructionNode::set_accumulation, this,
        std::placeholders::_1, std::placeholders::_2));
    clear_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/clear_cloud",
      std::bind(
        &LineLaserReconstructionNode::clear_cloud, this,
        std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/save_cloud",
      std::bind(
        &LineLaserReconstructionNode::save_cloud, this,
        std::placeholders::_1, std::placeholders::_2));

    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {
        publish_calibration_marker();
        marker_timer_->cancel();
      });
    for (int index = 0; index < reconstruction_worker_threads_; ++index) {
      reconstruction_workers_.emplace_back([this]() {reconstruction_worker();});
    }
    save_worker_ = std::thread([this]() {save_worker_loop();});
    RCLCPP_INFO(
      get_logger(),
      "scanner_650 reconstruction ready: %dx%d, Z %.0f..%.0f mm, output=%s, voxel=%.3f mm, "
      "centerline=%s, workers=%d, queue=%d, preview_voxel=%.3f mm",
      intrinsics_.imageSize.width, intrinsics_.imageSize.height,
      options_.reconstruction.minimumDepthMm, options_.reconstruction.maximumDepthMm,
      output_frame_.c_str(), voxel_size_m_ * 1000.0, centerline_mode_.c_str(),
      reconstruction_worker_threads_, reconstruction_queue_capacity_,
      preview_voxel_size_m_ * 1000.0);
    RCLCPP_INFO(
      get_logger(),
      "motion compensation: image header=DEVICE_TIMESTAMP exposure midpoint; "
      "pose=exact-time tf2 interpolation from controller joint states; timeout=%.0f ms",
      tf_lookup_timeout_s_ * 1000.0);
  }

  ~LineLaserReconstructionNode() override
  {
    {
      std::lock_guard<std::mutex> lock(reconstruction_queue_mutex_);
      reconstruction_stopping_ = true;
      for (const ReconstructionTask & task : reconstruction_queue_) {
        if (task.accumulation_epoch != 0U && pending_accumulation_tasks_ > 0U) {
          --pending_accumulation_tasks_;
        }
      }
      reconstruction_queue_.clear();
    }
    reconstruction_queue_condition_.notify_all();
    accumulation_drained_condition_.notify_all();
    for (std::thread & worker : reconstruction_workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    {
      std::lock_guard<std::mutex> lock(save_queue_mutex_);
      save_stopping_ = true;
    }
    save_queue_condition_.notify_all();
    if (save_worker_.joinable()) {
      save_worker_.join();
    }
  }

private:
  void load_calibration()
  {
    std::string error;
    if (!hik_calibration::loadIntrinsicsYaml(
        intrinsics_path_, &intrinsics_, &intrinsics_metadata_, &error) || !intrinsics_.ok)
    {
      throw std::runtime_error("cannot load scanner_650 intrinsics: " + error);
    }
    hik_calibration::BoardSpec board;
    if (!hik_calibration::loadLaserPlaneYaml(
        laser_plane_path_, &laser_plane_, &board, &laser_metadata_, &error) || !laser_plane_.ok)
    {
      throw std::runtime_error("cannot load scanner_650 laser plane: " + error);
    }
    if (intrinsics_metadata_.frameId != camera_frame_ ||
      laser_metadata_.cameraFrame != camera_frame_)
    {
      throw std::runtime_error("camera_frame does not match scanner_650 calibration metadata");
    }
    const double minimum_depth = laser_metadata_.validCameraZMinMm;
    const double maximum_depth = laser_metadata_.validCameraZMaxMm;
    if (!std::isfinite(minimum_depth) || !std::isfinite(maximum_depth) ||
      minimum_depth <= 0.0 || maximum_depth <= minimum_depth)
    {
      throw std::runtime_error("scanner_650 calibration has an invalid depth range");
    }
  }

  void configure_reconstruction()
  {
    options_.reconstruction.stripe.minimumDifference = 10;
    options_.reconstruction.stripe.thresholdStddevScale = 2.0;
    options_.reconstruction.stripe.minPointCount = 80;
    if (centerline_mode_ == "legacy") {
      options_.reconstruction.stripe.mode = hik_calibration::StripeExtractionMode::Legacy;
    } else if (centerline_mode_ == "shadow") {
      options_.reconstruction.stripe.mode = hik_calibration::StripeExtractionMode::Shadow;
    } else {
      options_.reconstruction.stripe.mode = hik_calibration::StripeExtractionMode::Quality;
    }
    options_.reconstruction.stripe.quality.orientation = hik_stripe::Orientation::Horizontal;
    options_.reconstruction.minReconstructedPoints = 80;
    options_.reconstruction.maxLineRmsMm = 0.50;
    options_.reconstruction.minimumDepthMm = camera_z_min_mm_;
    options_.reconstruction.maximumDepthMm = camera_z_max_mm_;

    // This is the scanner_650 profile from LineLaserDeviceProfile.cpp:
    // normalized ROI [0.0, 0.20, 1.0, 0.58].
    const int width = intrinsics_.imageSize.width;
    const int height = intrinsics_.imageSize.height;
    const int top = std::clamp(
      static_cast<int>(std::floor(0.20 * height)), 0, height - 1);
    const int bottom = std::clamp(
      static_cast<int>(std::ceil(0.78 * height)), top + 1, height);
    options_.reconstruction.stripe.quality.roi = cv::Rect(0, top, width, bottom - top);
    std::string error;
    if (!hik_calibration::buildLaserPlaneValidityMask(
        intrinsics_.imageSize, intrinsics_, laser_plane_,
        options_.reconstruction.minimumDepthMm,
        options_.reconstruction.maximumDepthMm,
        options_.reconstruction.stripe.quality.roi,
        &options_.reconstruction.stripeValidityMask, &error))
    {
      throw std::runtime_error("cannot build scanner_650 valid-depth mask: " + error);
    }
  }

  void on_camera_status(const std_msgs::msg::String::ConstSharedPtr message)
  {
    const bool connected = message->data.find("connected=1") != std::string::npos;
    const bool streaming = message->data.find("streaming=1") != std::string::npos;
    const bool mapping_ready =
      message->data.find("camera_time=DEVICE_TIMESTAMP_MAPPING") != std::string::npos;
    const bool mapping_warming_up =
      message->data.find("camera_time=WARMING_UP") != std::string::npos;
    const bool data_loss = message->data.find("event=data_loss") != std::string::npos;
    {
      std::lock_guard<std::mutex> camera_lock(camera_status_mutex_);
      latest_camera_status_ = message->data;
      if (!connected || !streaming || mapping_warming_up) {
        camera_ready_ = false;
      } else if (mapping_ready) {
        camera_ready_ = true;
      }
      if (data_loss) {
        std::lock_guard<std::mutex> scan_lock(mutex_);
        mark_scan_invalid_locked("upstream camera data loss: " + message->data, 0U);
      }
    }
  }

  void mark_scan_invalid_locked(
    const std::string & reason, std::uint64_t accumulation_epoch)
  {
    if ((accumulation_epoch == 0U && !accumulating_ && !session_sealing_) ||
      (accumulation_epoch != 0U && accumulation_epoch != accumulation_epoch_))
    {
      return;
    }
    scan_valid_ = false;
    if (std::find(scan_invalid_reasons_.begin(), scan_invalid_reasons_.end(), reason) ==
      scan_invalid_reasons_.end())
    {
      scan_invalid_reasons_.push_back(reason);
    }
  }

  void mark_scan_invalid(const std::string & reason, std::uint64_t accumulation_epoch = 0U)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    mark_scan_invalid_locked(reason, accumulation_epoch);
  }

  bool query_camera_statistics(std::string * statistics, std::string * error)
  {
    if (!camera_statistics_client_->wait_for_service(std::chrono::milliseconds(500))) {
      if (error) {*error = "camera statistics service is unavailable";}
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = camera_statistics_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      if (error) {*error = "camera statistics service timed out";}
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      if (error) {*error = response->message;}
      return false;
    }
    if (statistics) {*statistics = response->message;}
    return true;
  }

  static bool unsigned_status_field(
    const std::string & status, const std::string & key, std::uint64_t * value)
  {
    if (!value) {
      return false;
    }
    const std::string pattern = key + "=";
    const std::size_t begin = status.find(pattern);
    if (begin == std::string::npos) {
      return false;
    }
    const std::size_t value_begin = begin + pattern.size();
    const std::size_t value_end = status.find(';', value_begin);
    try {
      *value = std::stoull(status.substr(value_begin, value_end - value_begin));
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void audit_camera_statistics(
    const std::string & start, const std::string & end, std::uint64_t accumulation_epoch)
  {
    if (end.find("connected=1") == std::string::npos ||
      end.find("streaming=1") == std::string::npos ||
      end.find("camera_time=DEVICE_TIMESTAMP_MAPPING") == std::string::npos)
    {
      mark_scan_invalid(
        "camera was not connected, streaming, and timestamp-mapped when the scan sealed",
        accumulation_epoch);
    }
    static const std::array<const char *, 7> loss_fields{
      "frame_id_gaps", "out_of_order", "sdk_rejected", "image_pool_exhausted",
      "publish_queue_dropped", "timestamp_rejected", "non_monotonic_timestamp"};
    for (const char * field : loss_fields) {
      std::uint64_t start_value = 0U;
      std::uint64_t end_value = 0U;
      if (!unsigned_status_field(start, field, &start_value) ||
        !unsigned_status_field(end, field, &end_value))
      {
        mark_scan_invalid(
          "camera statistics are missing required loss counter " + std::string(field),
          accumulation_epoch);
      } else if (end_value != start_value) {
        mark_scan_invalid(
          "camera loss counter changed during scan: " + std::string(field) + " " +
          std::to_string(start_value) + "->" + std::to_string(end_value),
          accumulation_epoch);
      }
    }
    for (const char * field : {"received", "published"}) {
      std::uint64_t start_value = 0U;
      std::uint64_t end_value = 0U;
      if (!unsigned_status_field(start, field, &start_value) ||
        !unsigned_status_field(end, field, &end_value) || end_value < start_value)
      {
        mark_scan_invalid(
          "camera counters reset or became unreadable during scan: " + std::string(field),
          accumulation_epoch);
      }
    }
  }

  static void merge_voxel(VoxelAccumulator * destination, const VoxelAccumulator & source)
  {
    if (!destination) {
      return;
    }
    destination->x += source.x;
    destination->y += source.y;
    destination->z += source.z;
    destination->intensity += source.intensity;
    destination->confidence += source.confidence;
    destination->count += source.count;
  }

  static VoxelKey voxel_key(const CloudPoint & point, double voxel_size_m)
  {
    return VoxelKey{
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.x) / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.y) / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.z) / voxel_size_m))};
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    double requested_minimum = 0.0;
    double requested_maximum = 0.0;
    hik_calibration::SingleFrameProfileOptions configured_options;
    {
      std::lock_guard<std::mutex> lock(reconstruction_options_mutex_);
      requested_minimum = options_.reconstruction.minimumDepthMm;
      requested_maximum = options_.reconstruction.maximumDepthMm;
      configured_options = options_;
    }
    bool depth_changed = false;
    for (const rclcpp::Parameter & parameter : parameters) {
      if (parameter.get_name() == "camera_z_min_mm") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "camera_z_min_mm must be a double";
          return result;
        }
        requested_minimum = parameter.as_double();
        depth_changed = true;
      } else if (parameter.get_name() == "camera_z_max_mm") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "camera_z_max_mm must be a double";
          return result;
        }
        requested_maximum = parameter.as_double();
        depth_changed = true;
      }
    }
    if (!depth_changed) {
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        result.successful = false;
        result.reason =
          "stop point-cloud accumulation before changing the camera-Z range";
        return result;
      }
    }
    if (!std::isfinite(requested_minimum) || !std::isfinite(requested_maximum) ||
      requested_minimum <= 0.0 || requested_maximum <= requested_minimum)
    {
      result.successful = false;
      result.reason = "camera-Z range must satisfy 0 < min < max";
      return result;
    }
    configured_options.reconstruction.minimumDepthMm = requested_minimum;
    configured_options.reconstruction.maximumDepthMm = requested_maximum;
    cv::Mat configured_mask;
    std::string error;
    if (!hik_calibration::buildLaserPlaneValidityMask(
        intrinsics_.imageSize, intrinsics_, laser_plane_,
        requested_minimum, requested_maximum,
        configured_options.reconstruction.stripe.quality.roi,
        &configured_mask, &error))
    {
      result.successful = false;
      result.reason = "cannot rebuild valid-depth mask: " + error;
      return result;
    }
    configured_options.reconstruction.stripeValidityMask = configured_mask;
    {
      std::lock_guard<std::mutex> lock(reconstruction_options_mutex_);
      camera_z_min_mm_ = requested_minimum;
      camera_z_max_mm_ = requested_maximum;
      options_ = configured_options;
    }
    const bool outside_validated =
      requested_minimum < laser_metadata_.validCameraZMinMm ||
      requested_maximum > laser_metadata_.validCameraZMaxMm;
    RCLCPP_WARN(
      get_logger(),
      "camera-Z reconstruction range changed to %.1f..%.1f mm%s",
      requested_minimum, requested_maximum,
      outside_validated ? " (extends beyond calibration validation)" : "");
    std_msgs::msg::String status;
    status.data =
      "camera-Z reconstruction range=" + std::to_string(requested_minimum) +
      ".." + std::to_string(requested_maximum) + " mm";
    status_publisher_->publish(status);
    publish_calibration_marker();
    return result;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const std::uint64_t sequence = ++profile_sequence_;
    if (message->header.frame_id != camera_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "rejecting image in unexpected frame '%s'",
        message->header.frame_id.c_str());
      return;
    }
    if (static_cast<int>(message->width) != intrinsics_.imageSize.width ||
      static_cast<int>(message->height) != intrinsics_.imageSize.height)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "rejecting image with size %ux%u (calibrated %dx%d)",
        message->width, message->height, intrinsics_.imageSize.width, intrinsics_.imageSize.height);
      return;
    }

    std::lock_guard<std::mutex> capture_gate(capture_gate_mutex_);
    std::uint64_t accumulation_epoch = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        accumulation_epoch = accumulation_epoch_;
      }
    }
    bool queue_dropped = false;
    bool timestamp_gap = false;
    std::uint64_t estimated_missing = 0U;
    std::size_t queue_size = 0U;
    {
      std::lock_guard<std::mutex> lock(reconstruction_queue_mutex_);
      if (reconstruction_stopping_) {
        return;
      }
      if (reconstruction_queue_.size() >=
        static_cast<std::size_t>(reconstruction_queue_capacity_))
      {
        ++reconstruction_queue_drops_;
        if (accumulation_epoch != 0U) {
          ++scan_queue_drops_;
        }
        queue_dropped = true;
      } else {
        reconstruction_queue_.push_back(ReconstructionTask{message, sequence, accumulation_epoch});
        queue_size = reconstruction_queue_.size();
        if (accumulation_epoch != 0U) {
          ++pending_accumulation_tasks_;
          const std::uint64_t previous_count = scan_frames_enqueued_.fetch_add(1U);
          const std::int64_t stamp_ns =
            static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(message->header.stamp.nanosec);
          if (previous_count == 0U) {
            scan_first_image_stamp_ns_.store(stamp_ns);
          } else {
            const std::int64_t previous_stamp_ns = scan_last_image_stamp_ns_.load();
            const std::int64_t interval_ns = stamp_ns - previous_stamp_ns;
            const double expected_interval_ns = 1.0e9 / expected_camera_fps_;
            if (interval_ns <= 0 ||
              static_cast<double>(interval_ns) >
              maximum_frame_interval_factor_ * expected_interval_ns)
            {
              timestamp_gap = true;
              ++scan_timestamp_gap_events_;
              if (interval_ns > 0) {
                estimated_missing = std::max<std::uint64_t>(
                  1U, static_cast<std::uint64_t>(std::llround(
                    static_cast<double>(interval_ns) / expected_interval_ns)) - 1U);
              } else {
                estimated_missing = 1U;
              }
              scan_estimated_missing_frames_.fetch_add(estimated_missing);
            }
          }
          scan_last_image_stamp_ns_.store(stamp_ns);
        }
      }
    }
    if (queue_dropped) {
      if (accumulation_epoch != 0U) {
        mark_scan_invalid("reconstruction queue overflow");
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "reconstruction queue full; dropping image (capacity=%d, total_drops=%llu)",
        reconstruction_queue_capacity_,
        static_cast<unsigned long long>(reconstruction_queue_drops_.load()));
      return;
    }
    if (timestamp_gap) {
      mark_scan_invalid(
        "camera image timestamp gap indicates an end-to-end missing or reordered frame");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "camera image timestamp gap detected (events=%llu, estimated_missing=%llu)",
        static_cast<unsigned long long>(scan_timestamp_gap_events_.load()),
        static_cast<unsigned long long>(scan_estimated_missing_frames_.load()));
    }
    if (accumulation_epoch != 0U) {
      std::uint64_t previous_high_water = scan_queue_high_water_.load();
      while (queue_size > previous_high_water &&
        !scan_queue_high_water_.compare_exchange_weak(
          previous_high_water, static_cast<std::uint64_t>(queue_size)))
      {
      }
    }
    reconstruction_queue_condition_.notify_one();
  }

  void reconstruction_worker()
  {
    while (true) {
      ReconstructionTask task;
      {
        std::unique_lock<std::mutex> lock(reconstruction_queue_mutex_);
        reconstruction_queue_condition_.wait(lock, [this]() {
          return reconstruction_stopping_ || !reconstruction_queue_.empty();
        });
        if (reconstruction_stopping_ && reconstruction_queue_.empty()) {
          return;
        }
        task = std::move(reconstruction_queue_.front());
        reconstruction_queue_.pop_front();
      }
      try {
        process_image(task);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "reconstruction worker exception: %s", exception.what());
        if (task.accumulation_epoch != 0U) {
          ++scan_profile_rejected_;
          mark_scan_invalid(
            "reconstruction worker exception: " + std::string(exception.what()),
            task.accumulation_epoch);
        }
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "reconstruction worker caught an unknown exception");
        if (task.accumulation_epoch != 0U) {
          ++scan_profile_rejected_;
          mark_scan_invalid("unknown reconstruction worker exception", task.accumulation_epoch);
        }
      }
      if (task.accumulation_epoch != 0U) {
        {
          std::lock_guard<std::mutex> lock(reconstruction_queue_mutex_);
          if (pending_accumulation_tasks_ > 0U) {
            --pending_accumulation_tasks_;
          }
        }
        accumulation_drained_condition_.notify_all();
      }
    }
  }

  void process_image(const ReconstructionTask & task)
  {
    const sensor_msgs::msg::Image::ConstSharedPtr & message = task.image;

    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(message, "mono8");
    } catch (const cv_bridge::Exception & exception) {
      if (task.accumulation_epoch != 0U) {
        ++scan_profile_rejected_;
        mark_scan_invalid(
          "camera image conversion failed: " + std::string(exception.what()),
          task.accumulation_epoch);
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "cannot convert camera image: %s", exception.what());
      return;
    }

    hik_calibration::StaticProfileResult profile;
    const std::uint64_t sequence = task.sequence;
    const std::string sample_id = "ros2_" + std::to_string(sequence);
    hik_calibration::SingleFrameProfileOptions reconstruction_options;
    {
      std::lock_guard<std::mutex> lock(reconstruction_options_mutex_);
      reconstruction_options = options_;
    }
    if (!hik_calibration::reconstructSingleFrameProfile(
        image->image, sample_id, intrinsics_, laser_plane_, reconstruction_options, &profile) ||
      !profile.ok)
    {
      if (task.accumulation_epoch != 0U) {
        ++scan_profile_rejected_;
        mark_scan_invalid("laser profile reconstruction rejected", task.accumulation_epoch);
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "profile reconstruction rejected: %s",
        profile.error.c_str());
      return;
    }

    std::vector<CloudPoint> camera_points;
    camera_points.reserve(profile.points.size());
    for (const hik_calibration::StaticProfilePoint & point : profile.points) {
      CloudPoint output;
      output.x = static_cast<float>(point.cameraPointMm.x * 0.001);
      output.y = static_cast<float>(point.cameraPointMm.y * 0.001);
      output.z = static_cast<float>(point.cameraPointMm.z * 0.001);
      output.intensity = static_cast<float>(point.stripe.peakDifference);
      output.confidence = static_cast<float>(point.stripe.confidence);
      camera_points.push_back(output);
    }
    profile_publisher_->publish(make_cloud(camera_points, camera_frame_, message->header.stamp));

    if (task.accumulation_epoch == 0U) {
      return;
    }
    ++scan_profiles_reconstructed_;

    const rclcpp::Time image_stamp(message->header.stamp, get_clock()->get_clock_type());
    const double image_age_s = (now() - image_stamp).seconds();
    if (!std::isfinite(image_age_s) || image_age_s < -maximum_future_image_lead_s_ ||
      image_age_s > maximum_image_age_s_)
    {
      ++pose_sync_rejected_;
      mark_scan_invalid("image timestamp exceeded the pose synchronization window", task.accumulation_epoch);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot accumulate profile: exposure timestamp age %.3f s is outside "
        "[-%.3f, %.3f] s (pose_sync_rejected=%llu)",
        image_age_s, maximum_future_image_lead_s_, maximum_image_age_s_,
        static_cast<unsigned long long>(pose_sync_rejected_.load()));
      return;
    }

    geometry_msgs::msg::TransformStamped transform_message;
    try {
      transform_message = tf_buffer_.lookupTransform(
        output_frame_, camera_frame_, image_stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_s_));
    } catch (const tf2::TransformException & exception) {
      ++pose_sync_rejected_;
      mark_scan_invalid(
        "exact-time camera transform was unavailable", task.accumulation_epoch);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot accumulate profile at exposure midpoint: %s "
        "(pose_sync_rejected=%llu)", exception.what(),
        static_cast<unsigned long long>(pose_sync_rejected_.load()));
      return;
    }
    tf2::Transform transform;
    tf2::fromMsg(transform_message.transform, transform);
    std::vector<CloudPoint> base_points;
    base_points.reserve(camera_points.size());
    for (const CloudPoint & point : camera_points) {
      const tf2::Vector3 transformed = transform * tf2::Vector3(point.x, point.y, point.z);
      base_points.push_back(CloudPoint{
        static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
        static_cast<float>(transformed.z()), point.intensity, point.confidence});
    }
    std::size_t voxel_count = 0;
    bool capacity_reached = false;
    bool preview_capacity_reached = false;
    bool preview_update_skipped = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (task.accumulation_epoch != accumulation_epoch_) {
        return;
      }
      for (const CloudPoint & point : base_points) {
        const VoxelKey key = voxel_key(point, voxel_size_m_);
        auto iterator = voxels_.find(key);
        if (iterator == voxels_.end()) {
          if (voxels_.size() >= static_cast<std::size_t>(maximum_voxels_)) {
            capacity_reached = true;
            continue;
          }
          iterator = voxels_.emplace(key, VoxelAccumulator{}).first;
        }
        merge_voxel(
          &iterator->second,
          VoxelAccumulator{
            point.x, point.y, point.z, point.intensity, point.confidence, 1U});
      }
      voxel_count = voxels_.size();
      if (capacity_reached) {
        scan_valid_ = false;
        const std::string reason = "maximum reconstruction voxel count reached";
        if (std::find(scan_invalid_reasons_.begin(), scan_invalid_reasons_.end(), reason) ==
          scan_invalid_reasons_.end())
        {
          scan_invalid_reasons_.push_back(reason);
        }
      }
    }
    // Preview is deliberately lossy and isolated from the archival map. If RViz
    // is snapshotting it, skip this one preview update instead of stalling a
    // reconstruction worker; all full-resolution samples above are retained.
    {
      std::unique_lock<std::mutex> preview_lock(preview_mutex_, std::try_to_lock);
      if (!preview_lock.owns_lock()) {
        preview_update_skipped = true;
        ++preview_updates_skipped_;
      } else {
        for (const CloudPoint & point : base_points) {
          const VoxelKey key = voxel_key(point, preview_voxel_size_m_);
          auto iterator = preview_voxels_.find(key);
          if (iterator == preview_voxels_.end()) {
            if (preview_voxels_.size() >= static_cast<std::size_t>(maximum_preview_voxels_)) {
              preview_capacity_reached = true;
              continue;
            }
            iterator = preview_voxels_.emplace(key, VoxelAccumulator{}).first;
          }
          merge_voxel(
            &iterator->second,
            VoxelAccumulator{
              point.x, point.y, point.z, point.intensity, point.confidence, 1U});
        }
      }
    }
    const std::uint64_t synchronized_count = ++pose_sync_accepted_;
    if (capacity_reached) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "maximum voxel count reached; new voxels are dropped");
    }
    if (preview_capacity_reached) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "maximum preview voxel count reached; preview is capped but full scan continues");
    }
    if (preview_update_skipped) {
      RCLCPP_DEBUG(
        get_logger(), "skipped one preview update while a preview snapshot was being published");
    }
    if (synchronized_count % static_cast<std::uint64_t>(publish_every_n_profiles_) == 0U) {
      const std::int64_t current_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      const std::int64_t minimum_period_ns = static_cast<std::int64_t>(
        std::llround(scan_publish_period_s_ * 1.0e9));
      std::int64_t previous_ns = last_scan_publish_steady_ns_.load();
      if (current_ns - previous_ns >= minimum_period_ns &&
        last_scan_publish_steady_ns_.compare_exchange_strong(previous_ns, current_ns))
      {
        publish_preview_cloud(message->header.stamp);
      }
    }
    if (synchronized_count % 120U == 0U) {
      std_msgs::msg::String status;
      status.data =
        "motion_compensation=DEVICE_TIMESTAMP+TF2_JOINT_INTERPOLATION; accepted=" +
        std::to_string(synchronized_count) + "; rejected=" +
        std::to_string(pose_sync_rejected_.load()) + "; last_image_age_ms=" +
        std::to_string(image_age_s * 1000.0);
      status_publisher_->publish(status);
    }
    RCLCPP_DEBUG(
      get_logger(), "profile %llu: %zu points, accumulated voxels=%zu",
      static_cast<unsigned long long>(sequence), camera_points.size(), voxel_count);
  }

  static std::vector<CloudPoint> cloud_points_from_voxels(const VoxelMap & voxels)
  {
    std::vector<CloudPoint> points;
    points.reserve(voxels.size());
    for (const auto & entry : voxels) {
      const VoxelAccumulator & voxel = entry.second;
      if (voxel.count == 0U) {
        continue;
      }
      const double denominator = static_cast<double>(voxel.count);
      points.push_back(CloudPoint{
        static_cast<float>(voxel.x / denominator),
        static_cast<float>(voxel.y / denominator),
        static_cast<float>(voxel.z / denominator),
        static_cast<float>(voxel.intensity / denominator),
        static_cast<float>(voxel.confidence / denominator)});
    }
    return points;
  }

  std::vector<CloudPoint> snapshot_preview_voxels() const
  {
    std::lock_guard<std::mutex> lock(preview_mutex_);
    return cloud_points_from_voxels(preview_voxels_);
  }

  void publish_preview_cloud(const builtin_interfaces::msg::Time & stamp)
  {
    const std::vector<CloudPoint> points = snapshot_preview_voxels();
    fr5_scanner_650::world_height_color::Range height_range;
    std::string color_error;
    const fr5_scanner_650::world_height_color::Range * range = nullptr;
    if (!points.empty()) {
      if (compute_height_range(
          points, height_color_lower_percentile_, height_color_upper_percentile_,
          &height_range, &color_error))
      {
        range = &height_range;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "cannot color accumulated cloud from base-frame Z: %s", color_error.c_str());
      }
    }
    // The rgb field is the exact same Turbo(base_link Z) value later written
    // to PLY.  RViz must consume this field instead of inventing AxisColor.
    preview_publisher_->publish(make_cloud(points, output_frame_, stamp, range));
  }

  void publish_full_cloud(
    const std::vector<CloudPoint> & points, const builtin_interfaces::msg::Time & stamp)
  {
    fr5_scanner_650::world_height_color::Range height_range;
    std::string color_error;
    const fr5_scanner_650::world_height_color::Range * range = nullptr;
    if (!points.empty()) {
      if (compute_height_range(
          points, height_color_lower_percentile_, height_color_upper_percentile_,
          &height_range, &color_error))
      {
        range = &height_range;
      } else {
        RCLCPP_WARN(
          get_logger(), "cannot color final cloud from base-frame Z: %s", color_error.c_str());
      }
    }
    scan_publisher_->publish(make_cloud(points, output_frame_, stamp, range));
  }

  double scan_input_fps() const
  {
    const std::uint64_t count = scan_frames_enqueued_.load();
    const std::int64_t first_ns = scan_first_image_stamp_ns_.load();
    const std::int64_t last_ns = scan_last_image_stamp_ns_.load();
    if (count < 2U || last_ns <= first_ns) {
      return 0.0;
    }
    return static_cast<double>(count - 1U) * 1.0e9 /
           static_cast<double>(last_ns - first_ns);
  }

  std::string scan_statistics(std::size_t voxel_count) const
  {
    const bool valid = scan_valid_.load();
    std::ostringstream text;
    text << "centerline=" << centerline_mode_
         << "; enqueued=" << scan_frames_enqueued_.load()
         << "; reconstructed=" << scan_profiles_reconstructed_.load()
         << "; profile_rejected=" << scan_profile_rejected_.load()
         << "; queue_dropped=" << scan_queue_drops_.load()
         << "; queue_high_water=" << scan_queue_high_water_.load()
         << "; timestamp_gap_events=" << scan_timestamp_gap_events_.load()
         << "; estimated_missing=" << scan_estimated_missing_frames_.load()
         << "; preview_skipped=" << preview_updates_skipped_.load()
         << "; tf_accepted=" << pose_sync_accepted_.load()
         << "; tf_rejected=" << pose_sync_rejected_.load()
         << "; input_fps=" << std::fixed << std::setprecision(2) << scan_input_fps()
         << "; voxels=" << voxel_count
         << "; valid=" << (valid ? 1 : 0);
    return text.str();
  }

  ScanStatisticsSnapshot capture_scan_statistics(bool finish_sealing = false)
  {
    ScanStatisticsSnapshot snapshot;
    snapshot.frames_enqueued = scan_frames_enqueued_.load();
    snapshot.profiles_reconstructed = scan_profiles_reconstructed_.load();
    snapshot.profiles_rejected = scan_profile_rejected_.load();
    snapshot.queue_dropped = scan_queue_drops_.load();
    snapshot.queue_high_water = scan_queue_high_water_.load();
    snapshot.timestamp_gap_events = scan_timestamp_gap_events_.load();
    snapshot.estimated_missing_frames = scan_estimated_missing_frames_.load();
    snapshot.preview_updates_skipped = preview_updates_skipped_.load();
    snapshot.tf_accepted = pose_sync_accepted_.load();
    snapshot.tf_rejected = pose_sync_rejected_.load();
    snapshot.first_image_stamp_ns = scan_first_image_stamp_ns_.load();
    snapshot.last_image_stamp_ns = scan_last_image_stamp_ns_.load();
    {
      // Lock both domains as one transaction so a camera loss event cannot be
      // accepted but omitted from the sealed manifest.
      std::scoped_lock lock(camera_status_mutex_, mutex_);
      snapshot.voxel_count = voxels_.size();
      snapshot.valid = scan_valid_.load();
      snapshot.invalid_reasons = scan_invalid_reasons_;
      snapshot.camera_status_start = scan_camera_status_start_;
      snapshot.camera_status_end = scan_camera_status_end_.empty() ?
        latest_camera_status_ : scan_camera_status_end_;
      if (finish_sealing) {
        session_sealing_ = false;
      }
    }
    return snapshot;
  }

  static double scan_input_fps(const ScanStatisticsSnapshot & snapshot)
  {
    if (snapshot.frames_enqueued < 2U ||
      snapshot.last_image_stamp_ns <= snapshot.first_image_stamp_ns)
    {
      return 0.0;
    }
    return static_cast<double>(snapshot.frames_enqueued - 1U) * 1.0e9 /
           static_cast<double>(snapshot.last_image_stamp_ns - snapshot.first_image_stamp_ns);
  }

  static std::string scan_statistics(const ScanStatisticsSnapshot & snapshot)
  {
    std::ostringstream text;
    text << "enqueued=" << snapshot.frames_enqueued
         << "; reconstructed=" << snapshot.profiles_reconstructed
         << "; profile_rejected=" << snapshot.profiles_rejected
         << "; queue_dropped=" << snapshot.queue_dropped
         << "; queue_high_water=" << snapshot.queue_high_water
         << "; timestamp_gap_events=" << snapshot.timestamp_gap_events
         << "; estimated_missing=" << snapshot.estimated_missing_frames
         << "; preview_skipped=" << snapshot.preview_updates_skipped
         << "; tf_accepted=" << snapshot.tf_accepted
         << "; tf_rejected=" << snapshot.tf_rejected
         << "; input_fps=" << std::fixed << std::setprecision(2)
         << scan_input_fps(snapshot)
         << "; voxels=" << snapshot.voxel_count
         << "; valid=" << (snapshot.valid ? 1 : 0);
    return text.str();
  }

  void set_accumulation(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) {
      std::string camera_statistics_start;
      if (require_camera_ready_for_accumulation_) {
        {
          std::lock_guard<std::mutex> camera_lock(camera_status_mutex_);
          if (!camera_ready_) {
            response->success = false;
            response->message =
              "cannot start scan: HIK camera streaming and stable device timestamp mapping "
              "have not both been confirmed";
            return;
          }
        }
        std::string camera_error;
        if (!query_camera_statistics(&camera_statistics_start, &camera_error) ||
          camera_statistics_start.find("connected=1") == std::string::npos ||
          camera_statistics_start.find("streaming=1") == std::string::npos ||
          camera_statistics_start.find("camera_time=DEVICE_TIMESTAMP_MAPPING") ==
          std::string::npos)
        {
          response->success = false;
          response->message = "cannot start scan: exact HIK statistics are not ready: " +
            camera_error + "; " + camera_statistics_start;
          return;
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (accumulating_) {
          response->success = true;
          response->message = "scanner_650 accumulation is already enabled; session_id=" +
            current_session_id_ + "; " +
            scan_statistics(voxels_.size());
          return;
        }
        if (session_sealing_) {
          response->success = false;
          response->message =
            "cannot start a new scan while the previous session is still sealing; "
            "call set_accumulation(false) again after its reconstruction queue drains";
          return;
        }
        if (current_session_has_unsaved_data_) {
          response->success = false;
          response->message =
            "cannot start a new scan before the sealed session is saved or explicitly cleared; "
            "session_id=" + current_session_id_;
          return;
        }
      }
      {
        std::lock_guard<std::mutex> save_lock(save_queue_mutex_);
        if (!failed_save_jobs_.empty()) {
          response->success = false;
          response->message =
            "cannot start a new scan while a previous session save has failed; retry save for " +
            failed_save_jobs_.begin()->first;
          return;
        }
        if (pending_save_sessions_.size() >=
          static_cast<std::size_t>(maximum_pending_save_jobs_))
        {
          response->success = false;
          response->message =
            "cannot start a new scan because the background archive queue is full; pending=" +
            std::to_string(pending_save_sessions_.size());
          return;
        }
      }
      {
        std::unique_lock<std::mutex> lock(reconstruction_queue_mutex_);
        if (!accumulation_drained_condition_.wait_for(
            lock, std::chrono::duration<double>(accumulation_drain_timeout_s_),
            [this]() {return pending_accumulation_tasks_ == 0U;}))
        {
          response->success = false;
          response->message =
            "cannot start a new scan while the previous accumulation queue is still draining";
          return;
        }
      }
      // on_image() uses the same gate from epoch capture through queue insertion.
      // Once this lock is held, no frame from the previous state can appear late.
      std::lock_guard<std::mutex> capture_gate(capture_gate_mutex_);
      {
        std::lock_guard<std::mutex> lock(reconstruction_queue_mutex_);
        reconstruction_queue_.erase(
          std::remove_if(
            reconstruction_queue_.begin(), reconstruction_queue_.end(),
            [](const ReconstructionTask & task) {return task.accumulation_epoch == 0U;}),
          reconstruction_queue_.end());
      }
      scan_frames_enqueued_.store(0U);
      scan_profiles_reconstructed_.store(0U);
      scan_profile_rejected_.store(0U);
      scan_queue_drops_.store(0U);
      scan_queue_high_water_.store(0U);
      scan_timestamp_gap_events_.store(0U);
      scan_estimated_missing_frames_.store(0U);
      preview_updates_skipped_.store(0U);
      pose_sync_accepted_.store(0U);
      pose_sync_rejected_.store(0U);
      scan_first_image_stamp_ns_.store(0);
      scan_last_image_stamp_ns_.store(0);
      last_scan_publish_steady_ns_.store(0);
      std::size_t count = 0U;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // Per-scan storage isolation takes precedence over the legacy option.
        voxels_.clear();
        voxels_.reserve(static_cast<std::size_t>(maximum_voxels_));
      }
      {
        std::lock_guard<std::mutex> preview_lock(preview_mutex_);
        preview_voxels_.clear();
        preview_voxels_.reserve(static_cast<std::size_t>(maximum_preview_voxels_));
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++accumulation_epoch_;
        if (accumulation_epoch_ == 0U) {
          ++accumulation_epoch_;
        }
        current_session_id_ = make_scan_session_id();
        scan_valid_ = true;
        scan_invalid_reasons_.clear();
        current_session_has_unsaved_data_ = false;
        session_sealing_ = false;
        accumulating_ = true;
        count = voxels_.size();
      }
      {
        std::lock_guard<std::mutex> camera_lock(camera_status_mutex_);
        scan_camera_status_start_ = camera_statistics_start.empty() ?
          latest_camera_status_ : camera_statistics_start;
        scan_camera_status_end_.clear();
      }
      publish_preview_cloud(now());
      response->success = true;
      response->message = "scanner_650 accumulation enabled; isolated session_id=" +
        current_session_id_ + "; previous in-memory cloud cleared";
      response->message += "; " + scan_statistics(count);
    } else {
      {
        std::lock_guard<std::mutex> capture_gate(capture_gate_mutex_);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!accumulating_ && !session_sealing_) {
          response->success = true;
          response->message = "scanner_650 accumulation is already disabled";
          return;
        }
        if (accumulating_) {
          accumulating_ = false;
          session_sealing_ = true;
        }
      }
      bool drained = false;
      {
        std::unique_lock<std::mutex> lock(reconstruction_queue_mutex_);
        drained = accumulation_drained_condition_.wait_for(
          lock, std::chrono::duration<double>(accumulation_drain_timeout_s_),
          [this]() {return pending_accumulation_tasks_ == 0U;});
      }
      if (!drained) {
        mark_scan_invalid("accumulation queue drain timed out", accumulation_epoch_);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          current_session_has_unsaved_data_ = !voxels_.empty();
        }
        const ScanStatisticsSnapshot pending_statistics = capture_scan_statistics();
        response->success = false;
        response->message =
          "scanner_650 accumulation disabled but session sealing timed out; data remains in "
          "memory and will not be saved until the queue drains; call set_accumulation(false) "
          "again; session_id=" + current_session_id_ + "; " +
          scan_statistics(pending_statistics) + "; scan_result=INVALID";
        std_msgs::msg::String status;
        status.data = response->message +
          "; motion_compensation=DEVICE_TIMESTAMP+TF2_JOINT_INTERPOLATION";
        status_publisher_->publish(status);
        return;
      }
      if (require_camera_ready_for_accumulation_) {
        std::string camera_statistics_end;
        std::string camera_error;
        if (!query_camera_statistics(&camera_statistics_end, &camera_error)) {
          mark_scan_invalid(
            "cannot read HIK loss counters while sealing scan: " + camera_error,
            accumulation_epoch_);
        } else {
          std::string camera_statistics_start;
          {
            std::lock_guard<std::mutex> camera_lock(camera_status_mutex_);
            scan_camera_status_end_ = camera_statistics_end;
            camera_statistics_start = scan_camera_status_start_;
          }
          audit_camera_statistics(
            camera_statistics_start, camera_statistics_end, accumulation_epoch_);
        }
      }
      ScanStatisticsSnapshot statistics = capture_scan_statistics();
      if (statistics.frames_enqueued == 0U) {
        mark_scan_invalid("no camera frames entered this scan", accumulation_epoch_);
      }
      if (statistics.profiles_reconstructed + statistics.profiles_rejected !=
        statistics.frames_enqueued)
      {
        mark_scan_invalid("end-to-end reconstruction frame accounting mismatch", accumulation_epoch_);
      }
      if (statistics.tf_accepted + statistics.tf_rejected !=
        statistics.profiles_reconstructed)
      {
        mark_scan_invalid("pose synchronization frame accounting mismatch", accumulation_epoch_);
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_session_has_unsaved_data_ = !voxels_.empty();
      }
      statistics = capture_scan_statistics(true);
      std::string save_detail;
      bool save_queued = false;
      if (auto_save_on_accumulation_stop_ && statistics.voxel_count != 0U) {
        save_queued = queue_current_scan_for_save(statistics, &save_detail);
      }
      // SetBool.success reports whether the requested state transition was
      // completed. Scan data validity is an independent result carried in the
      // message/status/manifest; conflating the two makes motion supervisors
      // incorrectly report a safety shutdown failure after a detected bad frame.
      response->success = drained;
      response->message = drained ?
        "scanner_650 accumulation disabled after queue drain; " :
        "scanner_650 accumulation drain timed out; ";
      response->message += "session_id=" + current_session_id_ + "; " +
        scan_statistics(statistics);
      response->message += statistics.valid ? "; scan_result=VALID" : "; scan_result=INVALID";
      if (auto_save_on_accumulation_stop_) {
        response->message += save_queued ? "; background save queued: " + save_detail :
          "; background save not queued: " + save_detail;
      }
    }
    std_msgs::msg::String status;
    status.data = response->message +
      "; motion_compensation=DEVICE_TIMESTAMP+TF2_JOINT_INTERPOLATION";
    status_publisher_->publish(status);
  }

  void clear_cloud(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> queue_lock(reconstruction_queue_mutex_);
      if (pending_accumulation_tasks_ != 0U) {
        response->success = false;
        response->message = "wait for the reconstruction queue to drain before clearing the cloud";
        return;
      }
    }
    {
      std::lock_guard<std::mutex> save_lock(save_queue_mutex_);
      if (!current_session_id_.empty() &&
        pending_save_sessions_.count(current_session_id_) != 0U)
      {
        response->success = false;
        response->message =
          "cannot clear a sealed session while its background save is queued or running";
        return;
      }
    }
    std::size_t discarded_failed_saves = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        response->success = false;
        response->message = "stop point-cloud accumulation before clearing the cloud";
        return;
      }
      voxels_.clear();
      current_session_has_unsaved_data_ = false;
      session_sealing_ = false;
    }
    {
      std::lock_guard<std::mutex> save_lock(save_queue_mutex_);
      discarded_failed_saves = failed_save_jobs_.size();
      failed_save_jobs_.clear();
    }
    {
      std::lock_guard<std::mutex> preview_lock(preview_mutex_);
      preview_voxels_.clear();
    }
    publish_preview_cloud(now());
    publish_full_cloud({}, now());
    response->success = true;
    response->message = discarded_failed_saves != 0U ?
      "scanner_650 cloud cleared; " + std::to_string(discarded_failed_saves) +
      " failed in-memory archive(s) were explicitly discarded" :
      "scanner_650 accumulated cloud cleared";
  }

  void save_cloud(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    bool finish_sealing = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        response->success = false;
        response->message = "stop point-cloud accumulation before saving the cloud";
        return;
      }
      finish_sealing = session_sealing_;
    }
    {
      std::lock_guard<std::mutex> lock(reconstruction_queue_mutex_);
      if (pending_accumulation_tasks_ != 0U) {
        response->success = false;
        response->message = "wait for the point-cloud reconstruction queue to drain before saving";
        return;
      }
    }
    {
      std::lock_guard<std::mutex> save_lock(save_queue_mutex_);
      if (!failed_save_jobs_.empty()) {
        if (pending_save_sessions_.size() >=
          static_cast<std::size_t>(maximum_pending_save_jobs_))
        {
          response->success = false;
          response->message =
            "cannot retry failed archive because the background save queue is full";
          return;
        }
        auto failed = failed_save_jobs_.begin();
        const std::string session_id = failed->first;
        save_queue_.push_back(failed->second);
        failed_save_jobs_.erase(failed);
        pending_save_sessions_.insert(session_id);
        save_queue_condition_.notify_one();
        response->success = true;
        response->message = "background save retry queued for failed session_id=" + session_id;
        return;
      }
    }
    ScanStatisticsSnapshot statistics = capture_scan_statistics(finish_sealing);
    std::string detail;
    const bool queued = queue_current_scan_for_save(statistics, &detail);
    if (queued) {
      response->success = true;
      response->message = "background save queued; " + detail;
      return;
    }
    std::lock_guard<std::mutex> lock(save_queue_mutex_);
    const auto saved = saved_session_paths_.find(current_session_id_);
    if (saved != saved_session_paths_.end()) {
      response->success = true;
      response->message = "session already saved to " + saved->second;
    } else if (pending_save_sessions_.count(current_session_id_) != 0U) {
      response->success = true;
      response->message = "session save is already queued or running; session_id=" +
        current_session_id_;
    } else {
      response->success = false;
      response->message = detail.empty() ? "accumulated cloud is empty" : detail;
    }
  }

  bool queue_current_scan_for_save(
    const ScanStatisticsSnapshot & statistics, std::string * detail)
  {
    const std::string session_id = current_session_id_;
    if (session_id.empty()) {
      if (detail) {*detail = "no scan session is available";}
      return false;
    }
    {
      std::lock_guard<std::mutex> save_lock(save_queue_mutex_);
      const auto saved = saved_session_paths_.find(session_id);
      if (saved != saved_session_paths_.end()) {
        if (detail) {*detail = "session already saved to " + saved->second;}
        return false;
      }
      if (pending_save_sessions_.count(session_id) != 0U) {
        if (detail) {*detail = "session save is already queued or running";}
        return false;
      }
      const auto failed = failed_save_jobs_.find(session_id);
      if (failed != failed_save_jobs_.end()) {
        save_queue_.push_back(failed->second);
        failed_save_jobs_.erase(failed);
        pending_save_sessions_.insert(session_id);
        save_queue_condition_.notify_one();
        if (detail) {*detail = "retry queued for session_id=" + session_id;}
        return true;
      }
    }

    auto job = std::make_shared<SaveJob>();
    job->session_id = session_id;
    job->statistics = statistics;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        if (detail) {*detail = "stop point-cloud accumulation before saving";}
        return false;
      }
      if (voxels_.empty()) {
        if (detail) {*detail = "sealed session contains no voxels";}
        return false;
      }
      job->voxels = std::move(voxels_);
      job->statistics.voxel_count = job->voxels.size();
      current_session_has_unsaved_data_ = false;
    }
    {
      std::lock_guard<std::mutex> save_lock(save_queue_mutex_);
      save_queue_.push_back(job);
      pending_save_sessions_.insert(session_id);
    }
    save_queue_condition_.notify_one();
    if (detail) {
      *detail = "session_id=" + session_id + "; voxels=" +
        std::to_string(job->statistics.voxel_count);
    }
    return true;
  }

  bool write_save_job(
    const SaveJob & job, std::string * saved_path, std::string * error)
  {
    try {
      std::vector<CloudPoint> points = cloud_points_from_voxels(job.voxels);
      if (points.empty()) {
        throw std::runtime_error("sealed scan contains no non-empty voxels");
      }
      fr5_scanner_650::world_height_color::Range height_range;
      std::string color_error;
      if (!compute_height_range(
          points, height_color_lower_percentile_, height_color_upper_percentile_,
          &height_range, &color_error))
      {
        throw std::runtime_error("cannot compute world-Z colors: " + color_error);
      }

      std::filesystem::path output_root = std::filesystem::path(output_ply_).parent_path();
      if (output_root.empty()) {
        output_root = std::filesystem::current_path();
      }
      std::filesystem::create_directories(output_root);
      const std::filesystem::path partial_directory =
        output_root / (job.session_id + ".partial");
      const std::filesystem::path final_directory = output_root / job.session_id;
      if (std::filesystem::exists(final_directory)) {
        const std::filesystem::path committed_ply = final_directory / "scan_voxel.ply";
        const std::filesystem::path committed_manifest = final_directory / "manifest.yaml";
        if (!std::filesystem::is_regular_file(committed_ply) ||
          !std::filesystem::is_regular_file(committed_manifest))
        {
          throw std::runtime_error("existing session directory is incomplete; refusing overwrite");
        }
        // A previous attempt may have committed the directory and then failed
        // while updating latest_session.txt. Recover that idempotently.
        const std::filesystem::path latest_temporary =
          output_root / ("latest_session.txt." + job.session_id + ".tmp");
        const std::filesystem::path latest = output_root / "latest_session.txt";
        {
          std::ofstream pointer(latest_temporary, std::ios::out | std::ios::trunc);
          pointer << final_directory.string() << '\n';
          if (!pointer) {
            throw std::runtime_error("write recovered latest-session pointer failed");
          }
        }
        std::filesystem::rename(latest_temporary, latest);
        publish_full_cloud(points, now());
        if (saved_path) {
          *saved_path = committed_ply.string();
        }
        return true;
      }
      std::filesystem::create_directories(partial_directory);
      const std::filesystem::path temporary_ply = partial_directory / "scan_voxel.ply.tmp";
      const std::filesystem::path final_ply_in_partial = partial_directory / "scan_voxel.ply";
      std::ofstream output(temporary_ply, std::ios::out | std::ios::binary | std::ios::trunc);
      if (!output) {
        throw std::runtime_error("cannot open temporary PLY output");
      }
      output << "ply\nformat binary_little_endian 1.0\n"
             << "comment frame_id " << output_frame_ << "\n"
             << "comment units millimeter\n"
             << "comment session_id " << job.session_id << "\n"
             << "comment scan_valid " << (job.statistics.valid ? 1 : 0) << "\n"
             << "comment color_map turbo\n"
             << "comment color_scalar base_z_mm\n"
             << "comment color_frame_id " << output_frame_ << "\n"
             << "comment centerline_mode " << centerline_mode_ << "\n"
             << "comment scan_frames_enqueued " << job.statistics.frames_enqueued << "\n"
             << "comment scan_profiles_reconstructed "
             << job.statistics.profiles_reconstructed << "\n"
             << "comment scan_profile_rejected " << job.statistics.profiles_rejected << "\n"
             << "comment scan_queue_dropped " << job.statistics.queue_dropped << "\n"
             << "comment scan_queue_high_water " << job.statistics.queue_high_water << "\n"
             << "comment scan_timestamp_gap_events "
             << job.statistics.timestamp_gap_events << "\n"
             << "comment scan_estimated_missing_frames "
             << job.statistics.estimated_missing_frames << "\n"
             << "comment preview_updates_skipped "
             << job.statistics.preview_updates_skipped << "\n"
             << "comment scan_tf_accepted " << job.statistics.tf_accepted << "\n"
             << "comment scan_tf_rejected " << job.statistics.tf_rejected << "\n"
             << "comment scan_input_fps " << scan_input_fps(job.statistics) << "\n";
      for (const std::string & reason : job.statistics.invalid_reasons) {
        output << "comment invalid_reason " << sanitize_ply_comment(reason) << "\n";
      }
      output << "comment camera_status_start "
             << sanitize_ply_comment(job.statistics.camera_status_start) << "\n"
             << "comment camera_status_end "
             << sanitize_ply_comment(job.statistics.camera_status_end) << "\n"
             << "comment color_percentile " << height_color_lower_percentile_ << ' '
             << height_color_upper_percentile_ << "\n"
             << "comment color_range_mm " << height_range.lower_z_m * 1000.0 << ' '
             << height_range.upper_z_m * 1000.0 << "\n"
             << "element vertex " << points.size() << "\n"
             << "property double x\nproperty double y\nproperty double z\n"
             << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
             << "property float confidence\nproperty float response\nend_header\n";
      if (!output) {
        throw std::runtime_error("write PLY header failed");
      }

      constexpr std::size_t vertex_bytes = 3U * sizeof(double) + 3U + 2U * sizeof(float);
      constexpr std::size_t chunk_target_bytes = 4U * 1024U * 1024U;
      std::vector<std::uint8_t> chunk;
      chunk.reserve(chunk_target_bytes + vertex_bytes);
      for (const CloudPoint & point : points) {
        const auto color = fr5_scanner_650::world_height_color::colorForWorldZ(
          static_cast<double>(point.z), height_range);
        append_little_endian(&chunk, static_cast<double>(point.x) * 1000.0);
        append_little_endian(&chunk, static_cast<double>(point.y) * 1000.0);
        append_little_endian(&chunk, static_cast<double>(point.z) * 1000.0);
        append_little_endian(&chunk, color.red);
        append_little_endian(&chunk, color.green);
        append_little_endian(&chunk, color.blue);
        append_little_endian(&chunk, point.confidence);
        append_little_endian(&chunk, point.intensity);
        if (chunk.size() >= chunk_target_bytes) {
          output.write(
            reinterpret_cast<const char *>(chunk.data()),
            static_cast<std::streamsize>(chunk.size()));
          chunk.clear();
        }
      }
      if (!chunk.empty()) {
        output.write(
          reinterpret_cast<const char *>(chunk.data()),
          static_cast<std::streamsize>(chunk.size()));
      }
      output.flush();
      if (!output) {
        throw std::runtime_error("write PLY body failed");
      }
      output.close();
      std::filesystem::rename(temporary_ply, final_ply_in_partial);

      const std::filesystem::path temporary_manifest = partial_directory / "manifest.yaml.tmp";
      const std::filesystem::path final_manifest = partial_directory / "manifest.yaml";
      std::ofstream manifest(temporary_manifest, std::ios::out | std::ios::trunc);
      if (!manifest) {
        throw std::runtime_error("cannot open session manifest");
      }
      manifest << "schema_version: 1\n"
               << "session_id: " << yaml_double_quoted(job.session_id) << "\n"
               << "state: \"" << (job.statistics.valid ? "COMPLETE" : "INVALID") << "\"\n"
               << "valid: " << (job.statistics.valid ? "true" : "false") << "\n"
               << "frame_id: " << yaml_double_quoted(output_frame_) << "\n"
               << "cloud_file: \"scan_voxel.ply\"\n"
               << "voxel_size_m: " << voxel_size_m_ << "\n"
               << "statistics:\n"
               << "  frames_enqueued: " << job.statistics.frames_enqueued << "\n"
               << "  profiles_reconstructed: " << job.statistics.profiles_reconstructed << "\n"
               << "  profiles_rejected: " << job.statistics.profiles_rejected << "\n"
               << "  queue_dropped: " << job.statistics.queue_dropped << "\n"
               << "  queue_high_water: " << job.statistics.queue_high_water << "\n"
               << "  timestamp_gap_events: " << job.statistics.timestamp_gap_events << "\n"
               << "  estimated_missing_frames: "
               << job.statistics.estimated_missing_frames << "\n"
               << "  preview_updates_skipped: "
               << job.statistics.preview_updates_skipped << "\n"
               << "  tf_accepted: " << job.statistics.tf_accepted << "\n"
               << "  tf_rejected: " << job.statistics.tf_rejected << "\n"
               << "  input_fps: " << scan_input_fps(job.statistics) << "\n"
               << "  voxels: " << points.size() << "\n";
      if (job.statistics.invalid_reasons.empty()) {
        manifest << "invalid_reasons: []\n";
      } else {
        manifest << "invalid_reasons:\n";
        for (const std::string & reason : job.statistics.invalid_reasons) {
          manifest << "  - " << yaml_double_quoted(reason) << "\n";
        }
      }
      manifest << "camera_status_start: "
               << yaml_double_quoted(job.statistics.camera_status_start) << "\n"
               << "camera_status_end: "
               << yaml_double_quoted(job.statistics.camera_status_end) << "\n";
      manifest.flush();
      if (!manifest) {
        throw std::runtime_error("write session manifest failed");
      }
      manifest.close();
      std::filesystem::rename(temporary_manifest, final_manifest);
      std::filesystem::rename(partial_directory, final_directory);

      const std::filesystem::path latest_temporary =
        output_root / ("latest_session.txt." + job.session_id + ".tmp");
      const std::filesystem::path latest = output_root / "latest_session.txt";
      {
        std::ofstream pointer(latest_temporary, std::ios::out | std::ios::trunc);
        pointer << final_directory.string() << '\n';
        if (!pointer) {
          throw std::runtime_error("write latest-session pointer failed");
        }
      }
      std::filesystem::rename(latest_temporary, latest);
      publish_full_cloud(points, now());
      if (saved_path) {
        *saved_path = (final_directory / "scan_voxel.ply").string();
      }
      return true;
    } catch (const std::exception & exception) {
      if (error) {*error = exception.what();}
      return false;
    }
  }

  void save_worker_loop()
  {
    while (true) {
      std::shared_ptr<SaveJob> job;
      {
        std::unique_lock<std::mutex> lock(save_queue_mutex_);
        save_queue_condition_.wait(lock, [this]() {
          return save_stopping_ || !save_queue_.empty();
        });
        if (save_stopping_ && save_queue_.empty()) {
          return;
        }
        job = std::move(save_queue_.front());
        save_queue_.pop_front();
      }
      std_msgs::msg::String status;
      status.data = "session_id=" + job->session_id + "; storage=SAVING; " +
        scan_statistics(job->statistics);
      status_publisher_->publish(status);

      std::string saved_path;
      std::string error;
      const bool saved = write_save_job(*job, &saved_path, &error);
      {
        std::lock_guard<std::mutex> lock(save_queue_mutex_);
        pending_save_sessions_.erase(job->session_id);
        if (saved) {
          saved_session_paths_[job->session_id] = saved_path;
          failed_save_jobs_.erase(job->session_id);
        } else {
          failed_save_jobs_[job->session_id] = job;
        }
      }
      status.data = saved ?
        "session_id=" + job->session_id + "; storage=COMPLETE; path=" + saved_path +
        "; " + scan_statistics(job->statistics) :
        "session_id=" + job->session_id + "; storage=FAILED; error=" + error +
        "; data retained in memory for retry";
      status_publisher_->publish(status);
      if (saved) {
        RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "%s", status.data.c_str());
      }
    }
  }

  double laser_y_m(double x_m, double z_m) const
  {
    const cv::Vec3d & normal = laser_plane_.plane.normal;
    const double d_m = laser_plane_.plane.dMm * 0.001;
    return -(normal[0] * x_m + normal[2] * z_m + d_m) / normal[1];
  }

  void publish_calibration_marker()
  {
    double minimum_depth_mm = 0.0;
    double maximum_depth_mm = 0.0;
    {
      std::lock_guard<std::mutex> lock(reconstruction_options_mutex_);
      minimum_depth_mm = options_.reconstruction.minimumDepthMm;
      maximum_depth_mm = options_.reconstruction.maximumDepthMm;
    }
    visualization_msgs::msg::Marker plane;
    plane.header.frame_id = camera_frame_;
    plane.header.stamp = now();
    plane.ns = "scanner_650_calibration";
    plane.id = 0;
    plane.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    plane.action = visualization_msgs::msg::Marker::ADD;
    plane.pose.orientation.w = 1.0;
    plane.scale.x = 1.0;
    plane.scale.y = 1.0;
    plane.scale.z = 1.0;
    plane.color.r = 1.0F;
    plane.color.g = 0.08F;
    plane.color.b = 0.04F;
    plane.color.a = 0.22F;
    const double z_min = minimum_depth_mm * 0.001;
    const double z_max = maximum_depth_mm * 0.001;
    const double x_min = -0.24;
    const double x_max = 0.24;
    const auto p00 = marker_point(x_min, laser_y_m(x_min, z_min), z_min);
    const auto p10 = marker_point(x_max, laser_y_m(x_max, z_min), z_min);
    const auto p11 = marker_point(x_max, laser_y_m(x_max, z_max), z_max);
    const auto p01 = marker_point(x_min, laser_y_m(x_min, z_max), z_max);
    plane.points = {p00, p10, p11, p00, p11, p01};

    visualization_msgs::msg::Marker label;
    label.header = plane.header;
    label.ns = plane.ns;
    label.id = 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = marker_point(0.0, laser_y_m(0.0, 0.55), 0.55);
    label.pose.orientation.w = 1.0;
    label.pose.position.z += 0.025;
    label.scale.z = 0.025;
    label.color.r = 1.0F;
    label.color.g = 0.9F;
    label.color.b = 0.2F;
    label.color.a = 1.0F;
    std::ostringstream label_text;
    label_text << "650 nm laser plane / camera Z "
               << std::fixed << std::setprecision(0)
               << minimum_depth_mm << ".." << maximum_depth_mm << " mm";
    label.text = label_text.str();

    visualization_msgs::msg::MarkerArray markers;
    markers.markers = {plane, label};
    marker_publisher_->publish(markers);
  }

  std::string image_topic_;
  std::string camera_statistics_service_name_;
  std::string profile_topic_;
  std::string scan_topic_;
  std::string preview_topic_;
  std::string marker_topic_;
  std::string camera_frame_;
  std::string output_frame_;
  std::string intrinsics_path_;
  std::string laser_plane_path_;
  std::string output_ply_;
  std::string centerline_mode_{"legacy"};
  std::string current_session_id_;
  std::string scan_camera_status_start_;
  std::string scan_camera_status_end_;
  std::string latest_camera_status_;
  double camera_z_min_mm_{100.0};
  double camera_z_max_mm_{1000.0};
  double voxel_size_m_{0.0005};
  double preview_voxel_size_m_{0.002};
  double tf_lookup_timeout_s_{0.10};
  double maximum_image_age_s_{1.0};
  double maximum_future_image_lead_s_{0.02};
  double scan_publish_period_s_{0.5};
  double expected_camera_fps_{60.0};
  double maximum_frame_interval_factor_{1.8};
  double accumulation_drain_timeout_s_{5.0};
  double height_color_lower_percentile_{1.0};
  double height_color_upper_percentile_{99.0};
  int maximum_voxels_{2000000};
  int maximum_preview_voxels_{200000};
  int maximum_pending_save_jobs_{2};
  int publish_every_n_profiles_{6};
  int reconstruction_queue_capacity_{64};
  int reconstruction_worker_threads_{2};
  bool clear_cloud_on_accumulation_start_{true};
  bool auto_save_on_accumulation_stop_{true};
  bool require_camera_ready_for_accumulation_{true};
  mutable std::mutex mutex_;
  mutable std::mutex preview_mutex_;
  mutable std::mutex capture_gate_mutex_;
  mutable std::mutex camera_status_mutex_;
  mutable std::mutex reconstruction_options_mutex_;
  mutable std::mutex reconstruction_queue_mutex_;
  std::condition_variable reconstruction_queue_condition_;
  std::condition_variable accumulation_drained_condition_;
  std::deque<ReconstructionTask> reconstruction_queue_;
  std::vector<std::thread> reconstruction_workers_;
  bool reconstruction_stopping_{false};
  std::size_t pending_accumulation_tasks_{0U};
  bool accumulating_{false};
  bool session_sealing_{false};
  bool camera_ready_{false};
  bool current_session_has_unsaved_data_{false};
  std::uint64_t accumulation_epoch_{0U};
  std::vector<std::string> scan_invalid_reasons_;
  std::atomic<bool> scan_valid_{true};
  std::atomic<std::uint64_t> profile_sequence_{0U};
  std::atomic<std::uint64_t> reconstruction_queue_drops_{0U};
  std::atomic<std::uint64_t> scan_frames_enqueued_{0U};
  std::atomic<std::uint64_t> scan_profiles_reconstructed_{0U};
  std::atomic<std::uint64_t> scan_profile_rejected_{0U};
  std::atomic<std::uint64_t> scan_queue_drops_{0U};
  std::atomic<std::uint64_t> scan_queue_high_water_{0U};
  std::atomic<std::uint64_t> scan_timestamp_gap_events_{0U};
  std::atomic<std::uint64_t> scan_estimated_missing_frames_{0U};
  std::atomic<std::uint64_t> preview_updates_skipped_{0U};
  std::atomic<std::int64_t> scan_first_image_stamp_ns_{0};
  std::atomic<std::int64_t> scan_last_image_stamp_ns_{0};
  std::atomic<std::int64_t> last_scan_publish_steady_ns_{0};
  std::atomic<std::uint64_t> pose_sync_accepted_{0U};
  std::atomic<std::uint64_t> pose_sync_rejected_{0U};
  VoxelMap voxels_;
  VoxelMap preview_voxels_;
  mutable std::mutex save_queue_mutex_;
  std::condition_variable save_queue_condition_;
  std::deque<std::shared_ptr<SaveJob>> save_queue_;
  std::unordered_set<std::string> pending_save_sessions_;
  std::unordered_map<std::string, std::string> saved_session_paths_;
  std::unordered_map<std::string, std::shared_ptr<SaveJob>> failed_save_jobs_;
  std::thread save_worker_;
  bool save_stopping_{false};
  hik_calibration::IntrinsicCalibrationResult intrinsics_;
  hik_calibration::IntrinsicsYamlMetadata intrinsics_metadata_;
  hik_calibration::LaserPlaneFitResult laser_plane_;
  hik_calibration::LaserPlaneYamlMetadata laser_metadata_;
  hik_calibration::SingleFrameProfileOptions options_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_status_subscription_;
  rclcpp::CallbackGroup::SharedPtr camera_status_callback_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr camera_statistics_client_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr profile_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr accumulation_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LineLaserReconstructionNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("line_laser_reconstruction"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

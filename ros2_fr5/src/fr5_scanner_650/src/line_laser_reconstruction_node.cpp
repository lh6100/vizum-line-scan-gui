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
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
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

struct ReconstructionTask
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  std::uint64_t sequence{0U};
  std::uint64_t accumulation_epoch{0U};
};

bool host_is_little_endian()
{
  const std::uint16_t value = 1U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 1U;
}

template<typename Value>
bool write_little_endian(std::ostream * output, Value value)
{
  if (!output) {
    return false;
  }
  std::array<char, sizeof(Value)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(Value));
  if (!host_is_little_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  output->write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return output->good();
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
    profile_topic_ = declare_parameter<std::string>(
      "profile_topic", "/scanner_650/profile_cloud");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scanner_650/scan_cloud");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/scanner_650/markers");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "hik_camera_optical_frame");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
    intrinsics_path_ = declare_parameter<std::string>("intrinsics_path", "");
    laser_plane_path_ = declare_parameter<std::string>("laser_plane_path", "");
    camera_z_min_mm_ = declare_parameter<double>("camera_z_min_mm", 100.0);
    camera_z_max_mm_ = declare_parameter<double>("camera_z_max_mm", 1000.0);
    voxel_size_m_ = declare_parameter<double>("voxel_size_m", 0.0005);
    maximum_voxels_ = declare_parameter<int>("maximum_voxels", 2000000);
    publish_every_n_profiles_ = declare_parameter<int>("publish_every_n_profiles", 6);
    scan_publish_period_s_ = declare_parameter<double>("scan_publish_period_s", 0.5);
    centerline_mode_ = declare_parameter<std::string>("centerline_mode", "legacy");
    reconstruction_queue_capacity_ = declare_parameter<int>(
      "reconstruction_queue_capacity", 64);
    reconstruction_worker_threads_ = declare_parameter<int>(
      "reconstruction_worker_threads", 2);
    clear_cloud_on_accumulation_start_ = declare_parameter<bool>(
      "clear_cloud_on_accumulation_start", true);
    accumulation_drain_timeout_s_ = declare_parameter<double>(
      "accumulation_drain_timeout_s", 5.0);
    tf_lookup_timeout_s_ = declare_parameter<double>("tf_lookup_timeout_s", 0.10);
    maximum_image_age_s_ = declare_parameter<double>("maximum_image_age_s", 1.0);
    accumulating_ = declare_parameter<bool>("accumulate_on_start", false);
    if (accumulating_) {
      accumulation_epoch_ = 1U;
    }
    output_ply_ = declare_parameter<std::string>("output_ply", "scanner_650_cloud.ply");
    height_color_lower_percentile_ = declare_parameter<double>(
      "height_color_lower_percentile", 1.0);
    height_color_upper_percentile_ = declare_parameter<double>(
      "height_color_upper_percentile", 99.0);
    if (image_topic_.empty() || camera_frame_.empty() || output_frame_.empty() ||
      intrinsics_path_.empty() || laser_plane_path_.empty() || voxel_size_m_ <= 0.0 ||
      !std::isfinite(camera_z_min_mm_) || !std::isfinite(camera_z_max_mm_) ||
      camera_z_min_mm_ <= 0.0 || camera_z_max_mm_ <= camera_z_min_mm_ ||
      maximum_voxels_ < 1 || publish_every_n_profiles_ < 1 ||
      !std::isfinite(scan_publish_period_s_) || scan_publish_period_s_ <= 0.0 ||
      (centerline_mode_ != "legacy" && centerline_mode_ != "shadow" &&
      centerline_mode_ != "quality") ||
      reconstruction_queue_capacity_ < 2 || reconstruction_queue_capacity_ > 4096 ||
      reconstruction_worker_threads_ < 1 || reconstruction_worker_threads_ > 16 ||
      !std::isfinite(accumulation_drain_timeout_s_) || accumulation_drain_timeout_s_ <= 0.0 ||
      !std::isfinite(tf_lookup_timeout_s_) || tf_lookup_timeout_s_ <= 0.0 ||
      !std::isfinite(maximum_image_age_s_) || maximum_image_age_s_ <= tf_lookup_timeout_s_ ||
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
    RCLCPP_INFO(
      get_logger(),
      "scanner_650 reconstruction ready: %dx%d, Z %.0f..%.0f mm, output=%s, voxel=%.3f mm, "
      "centerline=%s, workers=%d, queue=%d",
      intrinsics_.imageSize.width, intrinsics_.imageSize.height,
      options_.reconstruction.minimumDepthMm, options_.reconstruction.maximumDepthMm,
      output_frame_.c_str(), voxel_size_m_ * 1000.0, centerline_mode_.c_str(),
      reconstruction_worker_threads_, reconstruction_queue_capacity_);
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

    std::uint64_t accumulation_epoch = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        accumulation_epoch = accumulation_epoch_;
      }
    }
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
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "reconstruction queue full; dropping image (capacity=%d, total_drops=%llu)",
          reconstruction_queue_capacity_,
          static_cast<unsigned long long>(reconstruction_queue_drops_.load()));
        return;
      }
      reconstruction_queue_.push_back(ReconstructionTask{message, sequence, accumulation_epoch});
      if (accumulation_epoch != 0U) {
        ++pending_accumulation_tasks_;
        const std::uint64_t previous_count = scan_frames_enqueued_.fetch_add(1U);
        const std::int64_t stamp_ns =
          static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
          static_cast<std::int64_t>(message->header.stamp.nanosec);
        if (previous_count == 0U) {
          scan_first_image_stamp_ns_.store(stamp_ns);
        }
        scan_last_image_stamp_ns_.store(stamp_ns);
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
        }
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "reconstruction worker caught an unknown exception");
        if (task.accumulation_epoch != 0U) {
          ++scan_profile_rejected_;
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
    if (!std::isfinite(image_age_s) || image_age_s < -0.005 ||
      image_age_s > maximum_image_age_s_)
    {
      ++pose_sync_rejected_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot accumulate profile: exposure timestamp age %.3f s is outside "
        "[-0.005, %.3f] s (pose_sync_rejected=%llu)",
        image_age_s, maximum_image_age_s_,
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
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cannot accumulate profile at exposure midpoint: %s "
        "(pose_sync_rejected=%llu)", exception.what(),
        static_cast<unsigned long long>(pose_sync_rejected_.load()));
      return;
    }
    tf2::Transform transform;
    tf2::fromMsg(transform_message.transform, transform);
    std::size_t voxel_count = 0;
    bool capacity_reached = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (task.accumulation_epoch != accumulation_epoch_) {
        return;
      }
      for (const CloudPoint & point : camera_points) {
        const tf2::Vector3 transformed = transform * tf2::Vector3(point.x, point.y, point.z);
        const VoxelKey key{
          static_cast<std::int64_t>(std::floor(transformed.x() / voxel_size_m_)),
          static_cast<std::int64_t>(std::floor(transformed.y() / voxel_size_m_)),
          static_cast<std::int64_t>(std::floor(transformed.z() / voxel_size_m_))};
        auto iterator = voxels_.find(key);
        if (iterator == voxels_.end()) {
          if (voxels_.size() >= static_cast<std::size_t>(maximum_voxels_)) {
            capacity_reached = true;
            continue;
          }
          iterator = voxels_.emplace(key, VoxelAccumulator{}).first;
        }
        VoxelAccumulator & voxel = iterator->second;
        voxel.x += transformed.x();
        voxel.y += transformed.y();
        voxel.z += transformed.z();
        voxel.intensity += point.intensity;
        voxel.confidence += point.confidence;
        ++voxel.count;
      }
      voxel_count = voxels_.size();
    }
    const std::uint64_t synchronized_count = ++pose_sync_accepted_;
    if (capacity_reached) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "maximum voxel count reached; new voxels are dropped");
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
        publish_scan_cloud(message->header.stamp);
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

  std::vector<CloudPoint> snapshot_voxels() const
  {
    std::vector<CloudPoint> points;
    std::lock_guard<std::mutex> lock(mutex_);
    points.reserve(voxels_.size());
    for (const auto & entry : voxels_) {
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

  void publish_scan_cloud(const builtin_interfaces::msg::Time & stamp)
  {
    const std::vector<CloudPoint> points = snapshot_voxels();
    fr5_scanner_650::world_height_color::Range height_range;
    std::string color_error;
    if (!points.empty() && !compute_height_range(
        points, height_color_lower_percentile_, height_color_upper_percentile_,
        &height_range, &color_error))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "cannot color accumulated cloud from base-frame Z: %s", color_error.c_str());
    }
    // The rgb field is the exact same Turbo(base_link Z) value later written
    // to PLY.  RViz must consume this field instead of inventing AxisColor.
    scan_publisher_->publish(make_cloud(points, output_frame_, stamp, &height_range));
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
    std::ostringstream text;
    text << "centerline=" << centerline_mode_
         << "; enqueued=" << scan_frames_enqueued_.load()
         << "; reconstructed=" << scan_profiles_reconstructed_.load()
         << "; profile_rejected=" << scan_profile_rejected_.load()
         << "; queue_dropped=" << scan_queue_drops_.load()
         << "; tf_accepted=" << pose_sync_accepted_.load()
         << "; tf_rejected=" << pose_sync_rejected_.load()
         << "; input_fps=" << std::fixed << std::setprecision(2) << scan_input_fps()
         << "; voxels=" << voxel_count;
    return text.str();
  }

  void set_accumulation(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (accumulating_) {
          response->success = true;
          response->message = "scanner_650 accumulation is already enabled; " +
            scan_statistics(voxels_.size());
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
      pose_sync_accepted_.store(0U);
      pose_sync_rejected_.store(0U);
      scan_first_image_stamp_ns_.store(0);
      scan_last_image_stamp_ns_.store(0);
      std::size_t count = 0U;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (clear_cloud_on_accumulation_start_) {
          voxels_.clear();
        }
        ++accumulation_epoch_;
        if (accumulation_epoch_ == 0U) {
          ++accumulation_epoch_;
        }
        accumulating_ = true;
        count = voxels_.size();
      }
      response->success = true;
      response->message = "scanner_650 accumulation enabled";
      if (clear_cloud_on_accumulation_start_) {
        response->message += "; previous cloud cleared";
      }
      response->message += "; " + scan_statistics(count);
    } else {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        accumulating_ = false;
      }
      bool drained = false;
      {
        std::unique_lock<std::mutex> lock(reconstruction_queue_mutex_);
        drained = accumulation_drained_condition_.wait_for(
          lock, std::chrono::duration<double>(accumulation_drain_timeout_s_),
          [this]() {return pending_accumulation_tasks_ == 0U;});
      }
      std::size_t count = 0U;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        count = voxels_.size();
      }
      publish_scan_cloud(now());
      response->success = drained;
      response->message = drained ?
        "scanner_650 accumulation disabled after queue drain; " :
        "scanner_650 accumulation drain timed out; ";
      response->message += scan_statistics(count);
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
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        response->success = false;
        response->message = "stop point-cloud accumulation before clearing the cloud";
        return;
      }
      voxels_.clear();
    }
    publish_scan_cloud(now());
    response->success = true;
    response->message = "scanner_650 accumulated cloud cleared";
  }

  void save_cloud(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accumulating_) {
        response->success = false;
        response->message = "stop point-cloud accumulation before saving the cloud";
        return;
      }
    }
    {
      std::lock_guard<std::mutex> lock(reconstruction_queue_mutex_);
      if (pending_accumulation_tasks_ != 0U) {
        response->success = false;
        response->message = "wait for the point-cloud reconstruction queue to drain before saving";
        return;
      }
    }
    const std::vector<CloudPoint> points = snapshot_voxels();
    if (points.empty()) {
      response->success = false;
      response->message = "accumulated cloud is empty";
      return;
    }
    try {
      fr5_scanner_650::world_height_color::Range height_range;
      std::string color_error;
      if (!compute_height_range(
          points, height_color_lower_percentile_, height_color_upper_percentile_,
          &height_range, &color_error))
      {
        throw std::runtime_error("cannot compute world-Z colors: " + color_error);
      }
      const std::filesystem::path path(output_ply_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      std::ofstream output(path, std::ios::out | std::ios::binary | std::ios::trunc);
      if (!output) {
        throw std::runtime_error("cannot open output file");
      }
      output << "ply\nformat binary_little_endian 1.0\n"
             << "comment frame_id " << output_frame_ << "\n"
             << "comment units millimeter\n"
             << "comment color_map turbo\n"
             << "comment color_scalar base_z_mm\n"
             << "comment color_frame_id " << output_frame_ << "\n"
             << "comment centerline_mode " << centerline_mode_ << "\n"
             << "comment scan_frames_enqueued " << scan_frames_enqueued_.load() << "\n"
             << "comment scan_profiles_reconstructed "
             << scan_profiles_reconstructed_.load() << "\n"
             << "comment scan_profile_rejected " << scan_profile_rejected_.load() << "\n"
             << "comment scan_queue_dropped " << scan_queue_drops_.load() << "\n"
             << "comment scan_tf_accepted " << pose_sync_accepted_.load() << "\n"
             << "comment scan_tf_rejected " << pose_sync_rejected_.load() << "\n"
             << "comment scan_input_fps " << scan_input_fps() << "\n"
             << "comment color_percentile " << height_color_lower_percentile_ << ' '
             << height_color_upper_percentile_ << "\n"
             << "comment color_range_mm " << height_range.lower_z_m * 1000.0 << ' '
             << height_range.upper_z_m * 1000.0 << "\n"
             << "element vertex " << points.size() << "\n"
             << "property double x\nproperty double y\nproperty double z\n"
             << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
             << "property float confidence\nproperty float response\nend_header\n";
      if (!output) {
        throw std::runtime_error("write header failed");
      }
      for (const CloudPoint & point : points) {
        const auto color = fr5_scanner_650::world_height_color::colorForWorldZ(
          static_cast<double>(point.z), height_range);
        const double x_mm = static_cast<double>(point.x) * 1000.0;
        const double y_mm = static_cast<double>(point.y) * 1000.0;
        const double z_mm = static_cast<double>(point.z) * 1000.0;
        if (!write_little_endian(&output, x_mm) ||
          !write_little_endian(&output, y_mm) ||
          !write_little_endian(&output, z_mm) ||
          !write_little_endian(&output, color.red) ||
          !write_little_endian(&output, color.green) ||
          !write_little_endian(&output, color.blue) ||
          !write_little_endian(&output, point.confidence) ||
          !write_little_endian(&output, point.intensity))
        {
          throw std::runtime_error("write vertex failed");
        }
      }
      if (!output) {
        throw std::runtime_error("write failed");
      }
      response->success = true;
      std::ostringstream message;
      message << "saved " << points.size() << " voxels as legacy-compatible binary PLY to "
              << output_ply_ << "; RGB color=Turbo(" << output_frame_ << " Z), range="
              << std::fixed << std::setprecision(3)
              << height_range.lower_z_m * 1000.0 << ".."
              << height_range.upper_z_m * 1000.0 << " mm; "
              << scan_statistics(points.size());
      response->message = message.str();
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = "cannot save cloud: " + std::string(exception.what());
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
  std::string profile_topic_;
  std::string scan_topic_;
  std::string marker_topic_;
  std::string camera_frame_;
  std::string output_frame_;
  std::string intrinsics_path_;
  std::string laser_plane_path_;
  std::string output_ply_;
  std::string centerline_mode_{"legacy"};
  double camera_z_min_mm_{100.0};
  double camera_z_max_mm_{1000.0};
  double voxel_size_m_{0.0005};
  double tf_lookup_timeout_s_{0.10};
  double maximum_image_age_s_{1.0};
  double scan_publish_period_s_{0.5};
  double accumulation_drain_timeout_s_{5.0};
  double height_color_lower_percentile_{1.0};
  double height_color_upper_percentile_{99.0};
  int maximum_voxels_{2000000};
  int publish_every_n_profiles_{6};
  int reconstruction_queue_capacity_{64};
  int reconstruction_worker_threads_{2};
  bool clear_cloud_on_accumulation_start_{true};
  mutable std::mutex mutex_;
  mutable std::mutex reconstruction_options_mutex_;
  mutable std::mutex reconstruction_queue_mutex_;
  std::condition_variable reconstruction_queue_condition_;
  std::condition_variable accumulation_drained_condition_;
  std::deque<ReconstructionTask> reconstruction_queue_;
  std::vector<std::thread> reconstruction_workers_;
  bool reconstruction_stopping_{false};
  std::size_t pending_accumulation_tasks_{0U};
  bool accumulating_{false};
  std::uint64_t accumulation_epoch_{0U};
  std::atomic<std::uint64_t> profile_sequence_{0U};
  std::atomic<std::uint64_t> reconstruction_queue_drops_{0U};
  std::atomic<std::uint64_t> scan_frames_enqueued_{0U};
  std::atomic<std::uint64_t> scan_profiles_reconstructed_{0U};
  std::atomic<std::uint64_t> scan_profile_rejected_{0U};
  std::atomic<std::uint64_t> scan_queue_drops_{0U};
  std::atomic<std::int64_t> scan_first_image_stamp_ns_{0};
  std::atomic<std::int64_t> scan_last_image_stamp_ns_{0};
  std::atomic<std::int64_t> last_scan_publish_steady_ns_{0};
  std::atomic<std::uint64_t> pose_sync_accepted_{0U};
  std::atomic<std::uint64_t> pose_sync_rejected_{0U};
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
  hik_calibration::IntrinsicCalibrationResult intrinsics_;
  hik_calibration::IntrinsicsYamlMetadata intrinsics_metadata_;
  hik_calibration::LaserPlaneFitResult laser_plane_;
  hik_calibration::LaserPlaneYamlMetadata laser_metadata_;
  hik_calibration::SingleFrameProfileOptions options_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr profile_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_publisher_;
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
    rclcpp::spin(std::make_shared<LineLaserReconstructionNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("line_laser_reconstruction"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

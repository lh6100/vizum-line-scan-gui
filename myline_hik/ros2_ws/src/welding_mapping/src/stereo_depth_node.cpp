#include "stereo/core/StereoDepthEngine.h"
#include "stereo/core/StereoRigCalibration.h"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <welding_interfaces/srv/get_active_artifact.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

class StereoDepthNode final : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  StereoDepthNode()
  : Node("stereo_depth")
  {
    const std::string stereo_yaml = declare_parameter<std::string>("stereo_yaml", "");
    const std::string left_intrinsics = declare_parameter<std::string>("left_intrinsics_yaml", "");
    const std::string left_handeye = declare_parameter<std::string>("left_handeye_yaml", "");
    const std::string right_intrinsics = declare_parameter<std::string>("right_intrinsics_yaml", "");
    const std::string right_handeye = declare_parameter<std::string>("right_handeye_yaml", "");
    require_active_calibration_ =
      declare_parameter<bool>("require_active_calibration", true);
    expected_calibration_id_ = declare_parameter<std::string>("expected_calibration_package_id", "");
    left_frame_ = declare_parameter<std::string>("left_optical_frame", "camera_650_optical_frame");
    point_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("point_stride", 2)));
    minimum_valid_fraction_ = declare_parameter<double>("minimum_valid_fraction", 0.02);

    hik_stereo::StereoRigCalibration rig;
    std::string error;
    if ((require_active_calibration_ && expected_calibration_id_.empty()) ||
      !hik_stereo::loadStereoRigFromStereoYaml(
        stereo_yaml, left_intrinsics, left_handeye, right_intrinsics, right_handeye, &rig, &error))
    {
      throw std::runtime_error(
        expected_calibration_id_.empty() ?
        "expected_calibration_package_id is required" : "stereo calibration rejected: " + error);
    }
    hik_stereo::StereoDepthOptions options;
    options.processingSize.width = declare_parameter<int>("processing_width", 612);
    options.processingSize.height = declare_parameter<int>("processing_height", 512);
    options.minimumDepthMm = declare_parameter<double>("minimum_depth_mm", 350.0);
    options.maximumDepthMm = declare_parameter<double>("maximum_depth_mm", 3000.0);
    options.blockSize = declare_parameter<int>("block_size", 5);
    options.uniquenessRatio = declare_parameter<int>("uniqueness_ratio", 8);
    options.speckleWindowSize = declare_parameter<int>("speckle_window_size", 100);
    options.speckleRange = declare_parameter<int>("speckle_range", 2);
    options.leftRightMaximumDifferencePx =
      declare_parameter<int>("left_right_maximum_difference_px", 1);
    options.disparityMarginPx = declare_parameter<int>("disparity_margin_px", 16);
    options.maximumNumDisparities = declare_parameter<int>("maximum_num_disparities", 512);
    options.enableLeftRightCheck = declare_parameter<bool>("enable_left_right_check", true);
    options.enableClahe = declare_parameter<bool>("enable_clahe", false);
    options.claheClipLimit = declare_parameter<double>("clahe_clip_limit", 2.0);
    const bool multi_band_enabled = declare_parameter<bool>("multi_band_enabled", false);
    const std::vector<double> band_minimums = declare_parameter<std::vector<double>>(
      "multi_band_minimum_depths_mm", {300.0, 600.0, 1200.0});
    const std::vector<double> band_maximums = declare_parameter<std::vector<double>>(
      "multi_band_maximum_depths_mm", {600.0, 1200.0, 2500.0});
    if (multi_band_enabled) {
      if (band_minimums.empty() || band_minimums.size() != band_maximums.size()) {
        throw std::runtime_error(
                "multi-band minimum/maximum depth arrays must be non-empty and equal-sized");
      }
      options.depthBands.reserve(band_minimums.size());
      for (std::size_t index = 0; index < band_minimums.size(); ++index) {
        options.depthBands.push_back(
          hik_stereo::StereoDepthBand{band_minimums[index], band_maximums[index]});
      }
    }
    if (!engine_.configure(rig, options, &error)) {
      throw std::runtime_error("stereo depth engine configuration rejected: " + error);
    }

    depth_publisher_ = create_publisher<Image>("/welding_robot/stereo/depth", rclcpp::SensorDataQoS());
    preview_publisher_ = create_publisher<Image>("/welding_robot/stereo/depth_preview", rclcpp::SensorDataQoS());
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/welding_robot/stereo/points", rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<std_msgs::msg::String>(
      "/welding_robot/stereo/status", rclcpp::QoS(10));

    if (require_active_calibration_) {
      calibration_client_ = create_client<welding_interfaces::srv::GetActiveArtifact>(
        "/welding_robot/calibration/get_active");
      calibration_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() {refresh_calibration();});
    } else {
      calibration_active_.store(true);
      RCLCPP_WARN(get_logger(),
        "require_active_calibration=false: commissioning preview only; do not feed this cloud to MoveIt");
    }

    left_subscriber_ = std::make_unique<message_filters::Subscriber<Image>>(
      this, declare_parameter<std::string>("left_topic", "/welding_robot/camera_650/image_raw"),
      rmw_qos_profile_sensor_data);
    right_subscriber_ = std::make_unique<message_filters::Subscriber<Image>>(
      this, declare_parameter<std::string>("right_topic", "/welding_robot/camera_450/image_raw"),
      rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *left_subscriber_, *right_subscriber_);
    synchronizer_->registerCallback(
      std::bind(&StereoDepthNode::on_pair, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(),
      "stereo depth configured: baseline %.3f mm, bands=%zu, disparity envelope=%d+%d, CLAHE=%s",
      rig.baselineMm, options.depthBands.empty() ? 1U : options.depthBands.size(),
      engine_.minimumDisparity(), engine_.numberOfDisparities(),
      options.enableClahe ? "on" : "off");
  }

private:
  void refresh_calibration()
  {
    if (!require_active_calibration_) {return;}
    if (calibration_request_pending_ || !calibration_client_->service_is_ready()) {return;}
    calibration_request_pending_ = true;
    auto request = std::make_shared<welding_interfaces::srv::GetActiveArtifact::Request>();
    calibration_client_->async_send_request(
      request,
      [this](rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedFuture future) {
        const auto response = future.get();
        calibration_active_ = response->active && response->artifact.id == expected_calibration_id_;
        calibration_request_pending_ = false;
      });
  }

  void on_pair(const Image::ConstSharedPtr & left, const Image::ConstSharedPtr & right)
  {
    if (!calibration_active_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "stereo frames withheld until the expected validated calibration is active");
      return;
    }
    if (processing_.exchange(true)) {return;}
    struct ProcessingGuard {std::atomic_bool * flag; ~ProcessingGuard() {flag->store(false);}} guard{&processing_};
    cv_bridge::CvImageConstPtr left_cv;
    cv_bridge::CvImageConstPtr right_cv;
    try {
      left_cv = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::MONO8);
      right_cv = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_ERROR(get_logger(), "stereo image conversion failed: %s", exception.what());
      return;
    }
    hik_stereo::StereoDepthResult result;
    if (!engine_.compute(left_cv->image, right_cv->image, &result)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "stereo matching failed: %s", result.error.c_str());
      return;
    }
    std_msgs::msg::Header header = left->header;
    header.frame_id = left_frame_;
    cv::Mat depth_m(result.xyzLeftMm.size(), CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    std::size_t cloud_points = 0;
    for (int row = 0; row < result.xyzLeftMm.rows; row += point_stride_) {
      const auto * xyz = result.xyzLeftMm.ptr<cv::Vec3f>(row);
      const auto * valid = result.validMask.ptr<unsigned char>(row);
      auto * depth = depth_m.ptr<float>(row);
      for (int column = 0; column < result.xyzLeftMm.cols; ++column) {
        if (valid[column]) {depth[column] = xyz[column][2] * 0.001F;}
        if (column % point_stride_ == 0 && valid[column]) {++cloud_points;}
      }
    }
    depth_publisher_->publish(*cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::TYPE_32FC1, depth_m).toImageMsg());
    preview_publisher_->publish(*cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::BGR8, result.depthPreviewBgr).toImageMsg());

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(cloud_points);
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "confidence", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cloud_points);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> confidence(cloud, "confidence");
    for (int row = 0; row < result.xyzLeftMm.rows; row += point_stride_) {
      const auto * xyz = result.xyzLeftMm.ptr<cv::Vec3f>(row);
      const auto * valid = result.validMask.ptr<unsigned char>(row);
      const auto * quality = result.confidence.ptr<float>(row);
      for (int column = 0; column < result.xyzLeftMm.cols; column += point_stride_) {
        if (!valid[column]) {continue;}
        *x = xyz[column][0] * 0.001F;
        *y = xyz[column][1] * 0.001F;
        *z = xyz[column][2] * 0.001F;
        *confidence = quality[column];
        ++x; ++y; ++z; ++confidence;
      }
    }
    if (result.statistics.validFraction >= minimum_valid_fraction_) {
      cloud_publisher_->publish(cloud);
    }
    std_msgs::msg::String status;
    std::ostringstream text;
    text << "{\"valid_fraction\":" << result.statistics.validFraction
         << ",\"median_depth_mm\":" << result.statistics.medianDepthMm
         << ",\"processing_ms\":" << result.statistics.processingMs
         << ",\"band_count\":" << result.statistics.bandCount
         << ",\"total_band_disparities\":" << result.statistics.totalBandDisparities
         << ",\"published_points\":" << cloud_points << '}';
    status.data = text.str();
    diagnostics_publisher_->publish(status);
  }

  hik_stereo::StereoDepthEngine engine_;
  std::string expected_calibration_id_;
  std::string left_frame_;
  bool require_active_calibration_{true};
  int point_stride_{2};
  double minimum_valid_fraction_{0.02};
  std::atomic_bool processing_{false};
  std::atomic_bool calibration_active_{false};
  std::atomic_bool calibration_request_pending_{false};
  std::unique_ptr<message_filters::Subscriber<Image>> left_subscriber_;
  std::unique_ptr<message_filters::Subscriber<Image>> right_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Publisher<Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<Image>::SharedPtr preview_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher_;
  rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedPtr calibration_client_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StereoDepthNode>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "stereo_depth startup failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

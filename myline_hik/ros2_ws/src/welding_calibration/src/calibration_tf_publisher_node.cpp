#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <welding_interfaces/srv/get_active_artifact.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{

bool rigid_transform(const std::vector<double> & values)
{
  if (values.size() != 16U) {return false;}
  for (const double value : values) {if (!std::isfinite(value)) {return false;}}
  for (int row = 0; row < 3; ++row) {
    for (int other = 0; other < 3; ++other) {
      double dot = 0.0;
      for (int column = 0; column < 3; ++column) {
        dot += values[row * 4 + column] * values[other * 4 + column];
      }
      if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1.0e-4) {return false;}
    }
  }
  const double determinant =
    values[0] * (values[5] * values[10] - values[6] * values[9]) -
    values[1] * (values[4] * values[10] - values[6] * values[8]) +
    values[2] * (values[4] * values[9] - values[5] * values[8]);
  return std::abs(determinant - 1.0) < 1.0e-4 &&
    std::abs(values[12]) < 1.0e-9 && std::abs(values[13]) < 1.0e-9 &&
    std::abs(values[14]) < 1.0e-9 && std::abs(values[15] - 1.0) < 1.0e-9;
}

std::filesystem::path role_path(
  const YAML::Node & manifest, const std::filesystem::path & manifest_path,
  const std::string & role)
{
  const std::filesystem::path configured = manifest["files"][role]["path"].as<std::string>();
  return configured.is_absolute() ? configured : manifest_path.parent_path() / configured;
}

geometry_msgs::msg::TransformStamped load_handeye(
  const std::filesystem::path & path, const std::string & parent, const std::string & child)
{
  const YAML::Node calibration = YAML::LoadFile(path.string());
  if (calibration["calibration_type"].as<std::string>() != "eye_in_hand" ||
    calibration["mode"].as<std::string>() != "camera_to_flange" ||
    calibration["translation_unit"].as<std::string>() != "mm")
  {throw std::runtime_error("unsupported hand-eye convention or unit: " + path.string());}
  const auto matrix = calibration["T_flange_camera"].as<std::vector<double>>();
  if (!rigid_transform(matrix)) {throw std::runtime_error("hand-eye matrix is not rigid: " + path.string());}
  tf2::Matrix3x3 rotation(
    matrix[0], matrix[1], matrix[2], matrix[4], matrix[5], matrix[6], matrix[8], matrix[9], matrix[10]);
  tf2::Quaternion quaternion;
  rotation.getRotation(quaternion);
  quaternion.normalize();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = matrix[3] * 0.001;
  transform.transform.translation.y = matrix[7] * 0.001;
  transform.transform.translation.z = matrix[11] * 0.001;
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}

class CalibrationTfPublisher final : public rclcpp::Node
{
public:
  CalibrationTfPublisher()
  : Node("calibration_tf_publisher"),
    flange_frame_(declare_parameter<std::string>("flange_frame", "flange_link")),
    camera_650_frame_(declare_parameter<std::string>("camera_650_frame", "camera_650_optical_frame")),
    camera_450_frame_(declare_parameter<std::string>("camera_450_frame", "camera_450_optical_frame")),
    broadcaster_(*this)
  {
    client_ = create_client<welding_interfaces::srv::GetActiveArtifact>(
      "/welding_robot/calibration/get_active");
    refresh_timer_ = create_wall_timer(1s, [this]() {refresh();});
    publish_timer_ = create_wall_timer(100ms, [this]() {publish();});
  }

private:
  void refresh()
  {
    if (request_pending_ || !client_->service_is_ready()) {return;}
    request_pending_ = true;
    client_->async_send_request(
      std::make_shared<welding_interfaces::srv::GetActiveArtifact::Request>(),
      [this](rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedFuture future) {
        const auto response = future.get();
        request_pending_ = false;
        if (!response->active) {clear(); return;}
        {std::lock_guard<std::mutex> lock(mutex_); if (response->artifact.sha256 == active_hash_) {return;}}
        try {
          const std::filesystem::path manifest_path = response->artifact.manifest_path;
          const YAML::Node manifest = YAML::LoadFile(manifest_path.string());
          std::array<geometry_msgs::msg::TransformStamped, 2> loaded = {
            load_handeye(role_path(manifest, manifest_path, "camera_650_handeye"), flange_frame_, camera_650_frame_),
            load_handeye(role_path(manifest, manifest_path, "camera_450_handeye"), flange_frame_, camera_450_frame_)};
          std::lock_guard<std::mutex> lock(mutex_);
          transforms_ = std::move(loaded);
          active_hash_ = response->artifact.sha256;
          RCLCPP_INFO(get_logger(), "using hand-eye TF from active package %s", response->artifact.id.c_str());
        } catch (const std::exception & exception) {
          clear();
          RCLCPP_ERROR(get_logger(), "active hand-eye TF rejected: %s", exception.what());
        }
      });
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    transforms_.reset();
    active_hash_.clear();
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transforms_) {return;}
    for (auto & transform : *transforms_) {transform.header.stamp = now();}
    broadcaster_.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>(
      transforms_->begin(), transforms_->end()));
  }

  std::string flange_frame_;
  std::string camera_650_frame_;
  std::string camera_450_frame_;
  std::mutex mutex_;
  std::optional<std::array<geometry_msgs::msg::TransformStamped, 2>> transforms_;
  std::string active_hash_;
  bool request_pending_{false};
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationTfPublisher>());
  rclcpp::shutdown();
  return 0;
}

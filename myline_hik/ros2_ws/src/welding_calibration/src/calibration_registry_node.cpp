#include "welding_calibration/calibration_registry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <welding_interfaces/srv/activate_artifact.hpp>
#include <welding_interfaces/srv/get_active_artifact.hpp>

#include <memory>
#include <string>

namespace
{

void fill_reference(
  const welding_calibration::ActiveCalibration & source,
  welding_interfaces::msg::ArtifactRef * target)
{
  target->id = source.manifest.package_id;
  target->version = source.activation_version;
  target->sha256 = source.manifest.manifest_sha256;
  target->status = "active";
  target->manifest_path = source.manifest.manifest_path.string();
}

class CalibrationRegistryNode final : public rclcpp::Node
{
public:
  CalibrationRegistryNode()
  : Node("calibration_registry"),
    registry_(declare_parameter<std::string>("registry_root", "data/registry/calibration"))
  {
    activate_service_ = create_service<welding_interfaces::srv::ActivateArtifact>(
      "/welding_robot/calibration/activate",
      [this](
        const std::shared_ptr<welding_interfaces::srv::ActivateArtifact::Request> request,
        std::shared_ptr<welding_interfaces::srv::ActivateArtifact::Response> response)
      {
        welding_calibration::ActiveCalibration active;
        response->success = registry_.activate(
          request->manifest_path, request->expected_id, request->expected_sha256,
          request->approved_by, &active, &response->error);
        if (response->success) {
          fill_reference(active, &response->active);
          RCLCPP_INFO(get_logger(), "activated calibration %s version %lu",
            active.manifest.package_id.c_str(), active.activation_version);
        } else {
          RCLCPP_ERROR(get_logger(), "calibration activation rejected: %s", response->error.c_str());
        }
      });

    get_service_ = create_service<welding_interfaces::srv::GetActiveArtifact>(
      "/welding_robot/calibration/get_active",
      [this](
        const std::shared_ptr<welding_interfaces::srv::GetActiveArtifact::Request>,
        std::shared_ptr<welding_interfaces::srv::GetActiveArtifact::Response> response)
      {
        const auto active = registry_.active(&response->error);
        response->active = active.has_value();
        if (active) {
          fill_reference(*active, &response->artifact);
        }
      });
  }

private:
  welding_calibration::CalibrationRegistry registry_;
  rclcpp::Service<welding_interfaces::srv::ActivateArtifact>::SharedPtr activate_service_;
  rclcpp::Service<welding_interfaces::srv::GetActiveArtifact>::SharedPtr get_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationRegistryNode>());
  rclcpp::shutdown();
  return 0;
}

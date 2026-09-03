#include "welding_data/session_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <welding_interfaces/srv/create_session.hpp>
#include <welding_interfaces/srv/finalize_session.hpp>
#include <welding_interfaces/srv/approve_trajectory.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

namespace
{

class SessionManagerNode final : public rclcpp::Node
{
public:
  SessionManagerNode()
  : Node("session_manager"), manager_(declare_parameter<std::string>("session_root", "data/sessions"))
  {
    create_service_ = create_service<welding_interfaces::srv::CreateSession>(
      "/welding_robot/session/create",
      [this](
        const std::shared_ptr<welding_interfaces::srv::CreateSession::Request> request,
        std::shared_ptr<welding_interfaces::srv::CreateSession::Response> response) {
        welding_data::SessionInfo session;
        response->success = manager_.create(
          request->task_type, request->requested_session_id, request->config_snapshot_yaml,
          &session, &response->error);
        if (response->success) {
          response->session_id = session.id;
          response->session_directory = session.directory.string();
          RCLCPP_INFO(get_logger(), "created session %s", session.id.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "session creation rejected: %s", response->error.c_str());
        }
      });
    finalize_service_ = create_service<welding_interfaces::srv::FinalizeSession>(
      "/welding_robot/session/finalize",
      [this](
        const std::shared_ptr<welding_interfaces::srv::FinalizeSession::Request> request,
        std::shared_ptr<welding_interfaces::srv::FinalizeSession::Response> response) {
        std::filesystem::path manifest;
        response->success = manager_.finalize(
          request->session_id, request->result, request->summary_yaml,
          &manifest, &response->error);
        if (response->success) {
          response->manifest_path = manifest.string();
          RCLCPP_INFO(get_logger(), "finalized session %s", request->session_id.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "session finalization rejected: %s", response->error.c_str());
        }
      });
    approval_service_ = create_service<welding_interfaces::srv::ApproveTrajectory>(
      "/welding_robot/session/approve_trajectory",
      [this](
        const std::shared_ptr<welding_interfaces::srv::ApproveTrajectory::Request> request,
        std::shared_ptr<welding_interfaces::srv::ApproveTrajectory::Response> response) {
        YAML::Node dependencies;
        dependencies["calibration_package_id"] = request->dependencies.calibration_package_id;
        dependencies["calibration_package_sha256"] = request->dependencies.calibration_package_sha256;
        dependencies["planning_scene_id"] = request->dependencies.planning_scene_id;
        dependencies["planning_scene_version"] = request->dependencies.planning_scene_version;
        dependencies["planning_scene_sha256"] = request->dependencies.planning_scene_sha256;
        dependencies["robot_model_sha256"] = request->dependencies.robot_model_sha256;
        dependencies["srdf_sha256"] = request->dependencies.srdf_sha256;
        dependencies["tool_model_sha256"] = request->dependencies.tool_model_sha256;
        dependencies["workpiece_frame_version"] = request->dependencies.workpiece_frame_version;
        dependencies["planner_config_sha256"] = request->dependencies.planner_config_sha256;
        YAML::Emitter emitter;
        emitter << dependencies;
        std::filesystem::path path;
        response->success = manager_.approve_trajectory(
          request->session_id, request->plan_id, request->trajectory_digest, request->operator_id,
          emitter.c_str(), &response->approval_id, &path, &response->error);
        if (response->success) {
          response->manifest_path = path.string();
          RCLCPP_WARN(get_logger(), "operator %s approved trajectory %s",
            request->operator_id.c_str(), request->plan_id.c_str());
        }
      });
  }

private:
  welding_data::SessionManager manager_;
  rclcpp::Service<welding_interfaces::srv::CreateSession>::SharedPtr create_service_;
  rclcpp::Service<welding_interfaces::srv::FinalizeSession>::SharedPtr finalize_service_;
  rclcpp::Service<welding_interfaces::srv::ApproveTrajectory>::SharedPtr approval_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SessionManagerNode>());
  rclcpp::shutdown();
  return 0;
}

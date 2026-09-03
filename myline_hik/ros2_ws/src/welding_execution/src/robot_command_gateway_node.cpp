#include "welding_execution/execution_guard.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <welding_interfaces/action/execute_motion.hpp>
#include <welding_interfaces/srv/get_active_artifact.hpp>
#include <welding_interfaces/srv/validate_plan_dependencies.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class RobotCommandGateway final : public rclcpp::Node
{
public:
  using ExecuteMotion = welding_interfaces::action::ExecuteMotion;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
  using Follow = control_msgs::action::FollowJointTrajectory;

  RobotCommandGateway()
  : Node("robot_command_gateway"),
    allow_hardware_motion_(declare_parameter<bool>("allow_hardware_motion", false)),
    session_root_(declare_parameter<std::string>("session_root", "data/sessions")),
    validation_timeout_(declare_parameter<double>("validation_timeout_seconds", 3.0))
  {
    display_publisher_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/welding_robot/display_trajectory", rclcpp::QoS(1).reliable().transient_local());
    dependency_client_ = create_client<welding_interfaces::srv::ValidatePlanDependencies>(
      "/welding_robot/planning_scene/validate_dependencies");
    calibration_client_ = create_client<welding_interfaces::srv::GetActiveArtifact>(
      "/welding_robot/calibration/get_active");
    controller_client_ = rclcpp_action::create_client<Follow>(
      this, declare_parameter<std::string>("controller_action", "/joint_trajectory_controller/follow_joint_trajectory"));
    action_server_ = rclcpp_action::create_server<ExecuteMotion>(
      this, "/welding_robot/execute_motion",
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteMotion::Goal> goal) {
        return handle_goal(uuid, std::move(goal));
      },
      [this](std::shared_ptr<GoalHandle> goal) {return handle_cancel(goal);},
      [this](std::shared_ptr<GoalHandle> goal) {handle_accepted(goal);});
    RCLCPP_WARN(get_logger(), "robot command gateway started with hardware motion %s",
      allow_hardware_motion_ ? "ENABLED" : "DISABLED (dry-run only)");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const ExecuteMotion::Goal> & goal)
  {
    if ((!goal->dry_run && !allow_hardware_motion_) || !welding_execution::valid_identifier(goal->session_id) ||
      !welding_execution::valid_identifier(goal->plan_id) ||
      !welding_execution::valid_identifier(goal->approval_id) || goal->trajectory.points.empty() ||
      goal->speed_scale <= 0.0 || goal->speed_scale > 1.0)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal)
  {
    std::thread([this, goal]() {execute(goal);}).detach();
  }

  template<typename FutureT>
  bool wait_ready(const FutureT & future) const
  {
    return future.wait_for(std::chrono::duration<double>(validation_timeout_)) == std::future_status::ready;
  }

  void abort(const std::shared_ptr<GoalHandle> & goal, const std::string & error)
  {
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->success = false;
    result->error = error;
    result->result = "rejected_by_execution_gate";
    goal->abort(result);
    RCLCPP_ERROR(get_logger(), "execution rejected: %s", error.c_str());
  }

  bool validate_preconditions(const ExecuteMotion::Goal & goal, std::string * error)
  {
    const auto session_directory = session_root_ / goal.session_id;
    if (!std::filesystem::is_regular_file(session_directory / "session.yaml")) {
      *error = "task session manifest does not exist";
      return false;
    }
    try {
      if (YAML::LoadFile((session_directory / "session.yaml").string())["state"].as<std::string>() != "open") {
        *error = "task session is not open";
        return false;
      }
    } catch (const std::exception & exception) {
      *error = std::string("task session manifest is invalid: ") + exception.what();
      return false;
    }
    const std::string actual_digest = welding_execution::trajectory_sha256(goal.trajectory);
    if (goal.trajectory_digest.empty() || goal.trajectory_digest != actual_digest) {
      *error = "trajectory digest does not match the approved trajectory";
      return false;
    }
    try {
      const YAML::Node approval = YAML::LoadFile(
        (session_directory / "approvals" / (goal.approval_id + ".yaml")).string());
      const YAML::Node dependencies = approval["dependencies"];
      if (approval["approval_id"].as<std::string>() != goal.approval_id ||
        approval["session_id"].as<std::string>() != goal.session_id ||
        approval["plan_id"].as<std::string>() != goal.plan_id ||
        approval["trajectory_digest"].as<std::string>() != actual_digest ||
        dependencies["calibration_package_id"].as<std::string>() != goal.dependencies.calibration_package_id ||
        dependencies["calibration_package_sha256"].as<std::string>() != goal.dependencies.calibration_package_sha256 ||
        dependencies["planning_scene_id"].as<std::string>() != goal.dependencies.planning_scene_id ||
        dependencies["planning_scene_version"].as<std::uint64_t>() != goal.dependencies.planning_scene_version ||
        dependencies["planning_scene_sha256"].as<std::string>() != goal.dependencies.planning_scene_sha256 ||
        dependencies["robot_model_sha256"].as<std::string>() != goal.dependencies.robot_model_sha256 ||
        dependencies["srdf_sha256"].as<std::string>() != goal.dependencies.srdf_sha256 ||
        dependencies["tool_model_sha256"].as<std::string>() != goal.dependencies.tool_model_sha256 ||
        dependencies["workpiece_frame_version"].as<std::string>() != goal.dependencies.workpiece_frame_version ||
        dependencies["planner_config_sha256"].as<std::string>() != goal.dependencies.planner_config_sha256)
      {
        *error = "approval does not match this trajectory and dependency snapshot";
        return false;
      }
    } catch (const std::exception & exception) {
      *error = std::string("approval record is missing or invalid: ") + exception.what();
      return false;
    }
    if (!dependency_client_->wait_for_service(std::chrono::duration<double>(validation_timeout_))) {
      *error = "planning dependency validator is unavailable";
      return false;
    }
    auto dependency_request = std::make_shared<welding_interfaces::srv::ValidatePlanDependencies::Request>();
    dependency_request->dependencies = goal.dependencies;
    auto dependency_future = dependency_client_->async_send_request(dependency_request);
    if (!wait_ready(dependency_future)) {*error = "planning dependency validation timed out"; return false;}
    const auto dependency_response = dependency_future.get();
    if (!dependency_response->valid) {*error = dependency_response->reason; return false;}

    if (!calibration_client_->wait_for_service(std::chrono::duration<double>(validation_timeout_))) {
      *error = "calibration registry is unavailable";
      return false;
    }
    auto calibration_request = std::make_shared<welding_interfaces::srv::GetActiveArtifact::Request>();
    auto calibration_future = calibration_client_->async_send_request(calibration_request);
    if (!wait_ready(calibration_future)) {*error = "calibration validation timed out"; return false;}
    const auto calibration_response = calibration_future.get();
    if (!calibration_response->active ||
      calibration_response->artifact.id != goal.dependencies.calibration_package_id ||
      calibration_response->artifact.sha256 != goal.dependencies.calibration_package_sha256)
    {
      *error = "trajectory calibration is no longer active";
      return false;
    }
    return true;
  }

  void publish_display(const ExecuteMotion::Goal & goal)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = goal.plan_id;
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory = goal.trajectory;
    display.trajectory.push_back(trajectory);
    display_publisher_->publish(display);
  }

  void execute(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    std::string error;
    if (!validate_preconditions(*goal, &error)) {abort(goal_handle, error); return;}
    const std::string execution_id = goal->session_id + "_" + goal->plan_id;
    if (!lease_.acquire(execution_id, &error)) {abort(goal_handle, error); return;}
    struct LeaseGuard
    {
      welding_execution::MotionLease * lease;
      std::string owner;
      ~LeaseGuard() {std::string ignored; lease->release(owner, &ignored);}
    } lease_guard{&lease_, execution_id};

    publish_display(*goal);
    if (goal->dry_run) {
      const std::size_t count = goal->trajectory.points.size();
      for (std::size_t index = 0; index < count; ++index) {
        if (goal_handle->is_canceling()) {
          auto result = std::make_shared<ExecuteMotion::Result>();
          result->execution_id = execution_id;
          result->result = "dry_run_canceled";
          goal_handle->canceled(result);
          return;
        }
        auto feedback = std::make_shared<ExecuteMotion::Feedback>();
        feedback->current_point = static_cast<std::uint32_t>(index);
        feedback->progress = static_cast<float>(index + 1U) / static_cast<float>(count);
        feedback->state = "DRY_RUN";
        feedback->detail = "visualization only; no controller command sent";
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(20ms);
      }
      auto result = std::make_shared<ExecuteMotion::Result>();
      result->success = true;
      result->execution_id = execution_id;
      result->result = "dry_run_complete";
      goal_handle->succeed(result);
      return;
    }

    if (!controller_client_->wait_for_action_server(std::chrono::duration<double>(validation_timeout_))) {
      abort(goal_handle, "hardware trajectory controller is unavailable");
      return;
    }
    Follow::Goal controller_goal;
    controller_goal.trajectory = goal->trajectory;
    for (auto & point : controller_goal.trajectory.points) {
      const std::int64_t nanoseconds = rclcpp::Duration(point.time_from_start).nanoseconds();
      point.time_from_start = rclcpp::Duration::from_nanoseconds(
        static_cast<std::int64_t>(static_cast<double>(nanoseconds) / goal->speed_scale));
      for (double & velocity : point.velocities) {velocity *= goal->speed_scale;}
      for (double & acceleration : point.accelerations) {acceleration *= goal->speed_scale * goal->speed_scale;}
    }
    auto send_future = controller_client_->async_send_goal(controller_goal);
    if (!wait_ready(send_future)) {abort(goal_handle, "controller did not accept trajectory in time"); return;}
    auto controller_handle = send_future.get();
    if (!controller_handle) {abort(goal_handle, "controller rejected trajectory"); return;}
    auto result_future = controller_client_->async_get_result(controller_handle);
    while (!wait_ready(result_future)) {
      if (goal_handle->is_canceling()) {
        controller_client_->async_cancel_goal(controller_handle);
        auto result = std::make_shared<ExecuteMotion::Result>();
        result->execution_id = execution_id;
        result->result = "hardware_execution_canceled";
        goal_handle->canceled(result);
        return;
      }
    }
    const auto controller_result = result_future.get();
    auto result = std::make_shared<ExecuteMotion::Result>();
    result->execution_id = execution_id;
    result->success = controller_result.code == rclcpp_action::ResultCode::SUCCEEDED;
    result->result = result->success ? "hardware_execution_complete" : "controller_execution_failed";
    if (result->success) {goal_handle->succeed(result);} else {goal_handle->abort(result);}
  }

  bool allow_hardware_motion_;
  std::filesystem::path session_root_;
  double validation_timeout_;
  welding_execution::MotionLease lease_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;
  rclcpp::Client<welding_interfaces::srv::ValidatePlanDependencies>::SharedPtr dependency_client_;
  rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedPtr calibration_client_;
  rclcpp_action::Client<Follow>::SharedPtr controller_client_;
  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  const auto node = std::make_shared<RobotCommandGateway>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

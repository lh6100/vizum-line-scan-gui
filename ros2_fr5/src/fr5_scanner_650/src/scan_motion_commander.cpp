#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fixed_axis_scan_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{

class ScanMotionCommander final : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using TrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  ScanMotionCommander()
  : Node("scan_motion_commander")
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "fairino5_v6_group");
    end_effector_link_ = declare_parameter<std::string>(
      "end_effector_link", "scanner_650_scan_tcp");
    planning_pipeline_ = declare_parameter<std::string>(
      "planning_pipeline", "pilz_industrial_motion_planner");
    planner_id_ = declare_parameter<std::string>("planner_id", "LIN");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "hik_camera_optical_frame");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    direction_frame_ = declare_parameter<std::string>("direction_frame", "camera");
    direction_axis_ = declare_parameter<std::string>("direction_axis", "+y");
    maintain_base_height_ = declare_parameter<bool>("maintain_base_height", true);
    distance_m_ = declare_parameter<double>("distance_m", 0.05);
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.20);
    acceleration_scaling_ = declare_parameter<double>("acceleration_scaling", 0.20);
    planning_time_s_ = declare_parameter<double>("planning_time_s", 10.0);
    allow_execution_ = declare_parameter<bool>("allow_execution", false);
    accumulation_service_name_ = declare_parameter<std::string>(
      "accumulation_service", "/scanner_650/set_accumulation");
    require_laser_control_ = declare_parameter<bool>("require_laser_control", true);
    laser_service_name_ = declare_parameter<std::string>(
      "laser_service", "/scanner_650/set_laser");
    controller_action_name_ = declare_parameter<std::string>(
      "controller_action", "/fairino5_controller/follow_joint_trajectory");
    validate_parameters();
  }

  void initialize(const rclcpp::Node::SharedPtr & node)
  {
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    accumulation_client_ = create_client<std_srvs::srv::SetBool>(
      accumulation_service_name_, rmw_qos_profile_services_default, callback_group_);
    laser_client_ = create_client<std_srvs::srv::SetBool>(
      laser_service_name_, rmw_qos_profile_services_default, callback_group_);
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node, controller_action_name_, callback_group_);

    moveit::planning_interface::MoveGroupInterface::Options options(planning_group_);
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node, options, nullptr, rclcpp::Duration::from_seconds(20.0));
    if (!move_group_->setEndEffectorLink(end_effector_link_)) {
      throw std::runtime_error("MoveIt rejected end_effector_link=" + end_effector_link_);
    }
    move_group_->setPlanningPipelineId(planning_pipeline_);
    move_group_->setPlannerId(planner_id_);
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

    plan_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/plan_linear_scan",
      std::bind(
        &ScanMotionCommander::plan_scan, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/execute_last_plan",
      std::bind(
        &ScanMotionCommander::execute_plan, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/stop_motion",
      std::bind(
        &ScanMotionCommander::stop_motion, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "scan motion ready: group=%s TCP=%s %s/%s distance=%.1f mm execution=%s",
      planning_group_.c_str(), end_effector_link_.c_str(), planning_pipeline_.c_str(),
      planner_id_.c_str(), distance_m_ * 1000.0, allow_execution_ ? "enabled" : "locked");
  }

private:
  void validate_parameters() const
  {
    if (planning_group_.empty() || end_effector_link_.empty() || camera_frame_.empty() ||
      base_frame_.empty() || planning_pipeline_.empty() || planner_id_.empty() ||
      accumulation_service_name_.empty() ||
      controller_action_name_.empty() ||
      (require_laser_control_ && laser_service_name_.empty()))
    {
      throw std::runtime_error("empty MoveIt scanner parameter");
    }
    if (direction_frame_ != "camera" && direction_frame_ != "tool" &&
      direction_frame_ != "base")
    {
      throw std::runtime_error("direction_frame must be 'camera', 'tool' or 'base'");
    }
    (void)axis_vector();
    if (!std::isfinite(distance_m_) || std::fabs(distance_m_) < 0.001 ||
      std::fabs(distance_m_) > 0.5)
    {
      throw std::runtime_error("distance_m must be in [-0.5,-0.001] or [0.001,0.5]");
    }
    if (!std::isfinite(velocity_scaling_) || velocity_scaling_ <= 0.0 || velocity_scaling_ > 1.0 ||
      !std::isfinite(acceleration_scaling_) || acceleration_scaling_ <= 0.0 ||
      acceleration_scaling_ > 1.0 || planning_time_s_ <= 0.0)
    {
      throw std::runtime_error("invalid MoveIt speed/acceleration/planning-time parameter");
    }
  }

  bool refresh_motion_parameters(std::string * error)
  {
    try {
      direction_frame_ = get_parameter("direction_frame").as_string();
      direction_axis_ = get_parameter("direction_axis").as_string();
      maintain_base_height_ = get_parameter("maintain_base_height").as_bool();
      distance_m_ = get_parameter("distance_m").as_double();
      velocity_scaling_ = get_parameter("velocity_scaling").as_double();
      acceleration_scaling_ = get_parameter("acceleration_scaling").as_double();
      planning_time_s_ = get_parameter("planning_time_s").as_double();
      validate_parameters();
      move_group_->setPlanningTime(planning_time_s_);
      move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
      return true;
    } catch (const std::exception & exception) {
      if (error) {
        *error = exception.what();
      }
      return false;
    }
  }

  tf2::Vector3 axis_vector() const
  {
    if (direction_axis_ == "x" || direction_axis_ == "+x") {return {1.0, 0.0, 0.0};}
    if (direction_axis_ == "-x") {return {-1.0, 0.0, 0.0};}
    if (direction_axis_ == "y" || direction_axis_ == "+y") {return {0.0, 1.0, 0.0};}
    if (direction_axis_ == "-y") {return {0.0, -1.0, 0.0};}
    if (direction_axis_ == "z" || direction_axis_ == "+z") {return {0.0, 0.0, 1.0};}
    if (direction_axis_ == "-z") {return {0.0, 0.0, -1.0};}
    throw std::runtime_error("direction_axis must be x/y/z or -x/-y/-z");
  }

  void plan_scan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    std::string parameter_error;
    if (!refresh_motion_parameters(&parameter_error)) {
      last_plan_.reset();
      response->success = false;
      response->message = "invalid linear-scan parameters: " + parameter_error;
      return;
    }
    try {
      move_group_->setStartStateToCurrentState();
      const geometry_msgs::msg::PoseStamped current =
        move_group_->getCurrentPose(end_effector_link_);
      if (maintain_base_height_ && current.header.frame_id != base_frame_) {
        throw std::runtime_error(
                "cannot enforce constant base height: MoveIt planning frame is '" +
                current.header.frame_id + "', expected '" + base_frame_ + "'");
      }
      geometry_msgs::msg::PoseStamped target = current;
      tf2::Vector3 direction = axis_vector();
      if (direction_frame_ == "camera") {
        const geometry_msgs::msg::PoseStamped camera_pose =
          move_group_->getCurrentPose(camera_frame_);
        tf2::Quaternion orientation;
        tf2::fromMsg(camera_pose.pose.orientation, orientation);
        direction = tf2::quatRotate(orientation, direction);
      } else if (direction_frame_ == "tool") {
        tf2::Quaternion orientation;
        tf2::fromMsg(current.pose.orientation, orientation);
        direction = tf2::quatRotate(orientation, direction);
      }
      std::array<double, 3> normalized{};
      std::string direction_error;
      if (!fr5_scanner_650::fixed_axis_scan::normalizeDirection(
          {direction.x(), direction.y(), direction.z()}, maintain_base_height_,
          &normalized, &direction_error))
      {
        throw std::runtime_error(direction_error);
      }
      direction.setValue(normalized[0], normalized[1], normalized[2]);
      target.pose.position.x += distance_m_ * direction.x();
      target.pose.position.y += distance_m_ * direction.y();
      target.pose.position.z += distance_m_ * direction.z();
      if (maintain_base_height_) {
        target.pose.position.z = current.pose.position.z;
      }

      move_group_->setPlanningPipelineId(planning_pipeline_);
      move_group_->setPlannerId(planner_id_);
      move_group_->setPoseTarget(target, end_effector_link_);
      moveit::planning_interface::MoveGroupInterface::Plan candidate;
      const moveit::core::MoveItErrorCode result = move_group_->plan(candidate);
      move_group_->clearPoseTargets();
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        last_plan_.reset();
        response->success = false;
        response->message = "Pilz LIN planning failed; MoveIt code=" + std::to_string(result.val);
        return;
      }
      last_plan_ = std::make_unique<moveit::planning_interface::MoveGroupInterface::Plan>(
        std::move(candidate));
      response->success = true;
      response->message = "planned scanner_650 TCP LIN motion: " +
        std::to_string(distance_m_ * 1000.0) + " mm along " + direction_frame_ +
        " " + direction_axis_ +
        (maintain_base_height_ ? ", projected onto base XY at constant Z" : "") +
        "; call /scanner_650/execute_last_plan explicitly";
    } catch (const std::exception & exception) {
      move_group_->clearPoseTargets();
      last_plan_.reset();
      response->success = false;
      response->message = "planning exception: " + std::string(exception.what());
    }
  }

  bool request_boolean_service(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
    bool enabled, const std::string & label, std::string * error)
  {
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      if (error) {*error = label + " service is unavailable";}
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = client->async_send_request(request);
    // Reconstruction may spend up to 5 s draining its bounded queue while
    // sealing a scan. Keep the caller timeout longer than that contract.
    if (future.wait_for(std::chrono::seconds(7)) != std::future_status::ready) {
      if (error) {*error = label + " service timed out";}
      return false;
    }
    const auto response = future.get();
    if (!response->success && error) {
      *error = response->message;
    }
    return response->success;
  }

  bool request_accumulation(bool enabled, std::string * error)
  {
    return request_boolean_service(
      accumulation_client_, enabled, "reconstruction accumulation", error);
  }

  bool request_laser(bool enabled, std::string * error)
  {
    if (!require_laser_control_) {
      return true;
    }
    return request_boolean_service(laser_client_, enabled, "650 nm laser", error);
  }

  bool start_state_is_current(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    std::string * error)
  {
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.points.empty() ||
      trajectory.points.front().positions.size() != trajectory.joint_names.size())
    {
      if (error) {*error = "planned trajectory has no valid start point";}
      return false;
    }
    const moveit::core::RobotStatePtr current = move_group_->getCurrentState(1.0);
    if (!current) {
      if (error) {*error = "cannot read current robot state";}
      return false;
    }
    constexpr double maximum_start_error_rad = 0.01;
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      const std::string & joint = trajectory.joint_names[index];
      const double difference = std::fabs(
        current->getVariablePosition(joint) - trajectory.points.front().positions[index]);
      if (!std::isfinite(difference) || difference > maximum_start_error_rad) {
        if (error) {
          *error = "robot moved since planning (" + joint + " start error=" +
            std::to_string(difference) + " rad); replan required";
        }
        return false;
      }
    }
    return true;
  }

  moveit::core::MoveItErrorCode execute_controller_trajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan,
    std::string * detail)
  {
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(2))) {
      if (detail) {*detail = "joint trajectory controller action is unavailable";}
      return moveit::core::MoveItErrorCode::CONTROL_FAILED;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = plan.trajectory_.joint_trajectory;
    goal.goal_time_tolerance.sec = 1;
    goal.goal_time_tolerance.nanosec = 0;
    auto goal_future = trajectory_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      if (detail) {*detail = "trajectory controller did not accept/reject the goal in time";}
      return moveit::core::MoveItErrorCode::TIMED_OUT;
    }
    const TrajectoryGoalHandle::SharedPtr goal_handle = goal_future.get();
    if (!goal_handle) {
      if (detail) {*detail = "trajectory controller rejected the goal";}
      return moveit::core::MoveItErrorCode::CONTROL_FAILED;
    }
    const auto & last_time = goal.trajectory.points.back().time_from_start;
    const double planned_seconds =
      static_cast<double>(last_time.sec) + static_cast<double>(last_time.nanosec) * 1.0e-9;
    const auto timeout = std::chrono::duration<double>(
      std::clamp(planned_seconds + 5.0, 5.0, 305.0));
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      (void)trajectory_client_->async_cancel_goal(goal_handle);
      if (detail) {*detail = "trajectory controller result timed out; cancel requested";}
      return moveit::core::MoveItErrorCode::TIMED_OUT;
    }
    const TrajectoryGoalHandle::WrappedResult result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      if (detail) {
        *detail = result.result ? result.result->error_string : "controller returned no result";
      }
      return moveit::core::MoveItErrorCode::CONTROL_FAILED;
    }
    if (detail) {*detail = "joint trajectory controller completed successfully";}
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

  void execute_plan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!allow_execution_) {
      response->success = false;
      response->message =
        "execution is locked; relaunch with allow_execution:=true only after RViz verification";
      return;
    }
    if (!last_plan_) {
      response->success = false;
      response->message = "no stored plan; call /scanner_650/plan_linear_scan first";
      return;
    }

    std::string start_error;
    if (!start_state_is_current(*last_plan_, &start_error)) {
      last_plan_.reset();
      response->success = false;
      response->message = start_error;
      return;
    }

    std::string laser_error;
    if (!request_laser(true, &laser_error)) {
      response->success = false;
      response->message = "motion not started: " + laser_error;
      return;
    }
    std::string accumulation_error;
    if (!request_accumulation(true, &accumulation_error)) {
      std::string ignored;
      (void)request_laser(false, &ignored);
      response->success = false;
      response->message = "motion not started: " + accumulation_error;
      return;
    }
    std::string controller_detail;
    const moveit::core::MoveItErrorCode result =
      execute_controller_trajectory(*last_plan_, &controller_detail);
    std::string disable_error;
    const bool accumulation_stopped = request_accumulation(false, &disable_error);
    std::string laser_off_error;
    const bool laser_stopped = request_laser(false, &laser_off_error);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "trajectory execution failed: " + controller_detail +
        "; code=" + std::to_string(result.val);
      if (!accumulation_stopped) {
        response->message += "; WARNING: " + disable_error;
      }
      if (!laser_stopped) {
        response->message += "; WARNING: " + laser_off_error;
      }
      return;
    }
    last_plan_.reset();
    response->success = accumulation_stopped && laser_stopped;
    response->message = response->success ?
      "scanner_650 TCP scan completed; accumulation stopped and laser confirmed OFF" :
      "motion completed but shutdown was incomplete: " + disable_error + " " + laser_off_error;
  }

  void stop_motion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (trajectory_client_) {
      (void)trajectory_client_->async_cancel_all_goals();
    }
    // Cancel before acquiring motion_mutex_: execute_plan may be waiting for
    // the controller result while holding it.
    std::lock_guard<std::mutex> lock(motion_mutex_);
    move_group_->stop();
    last_plan_.reset();
    std::string error;
    const bool accumulation_stopped = request_accumulation(false, &error);
    std::string laser_error;
    const bool laser_stopped = request_laser(false, &laser_error);
    response->success = accumulation_stopped && laser_stopped;
    response->message = response->success ?
      "MoveIt stop requested, accumulation disabled and laser confirmed OFF" :
      "MoveIt stop requested; shutdown warning: " + error + " " + laser_error;
  }

  std::string planning_group_;
  std::string end_effector_link_;
  std::string planning_pipeline_;
  std::string planner_id_;
  std::string camera_frame_;
  std::string base_frame_;
  std::string direction_frame_;
  std::string direction_axis_;
  std::string accumulation_service_name_;
  std::string laser_service_name_;
  std::string controller_action_name_;
  double distance_m_{0.05};
  double velocity_scaling_{0.20};
  double acceleration_scaling_{0.20};
  double planning_time_s_{10.0};
  bool allow_execution_{false};
  bool require_laser_control_{true};
  bool maintain_base_height_{true};
  std::mutex motion_mutex_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface::Plan> last_plan_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr accumulation_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr laser_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ScanMotionCommander>();
    node->initialize(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("scan_motion_commander"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

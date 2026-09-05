#include "workspace_coarse_scan_core.hpp"
#include "workspace_robot_state_utils.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <execinfo.h>
#include <fcntl.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unistd.h>

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

namespace ws = fr5_scanner_650::workspace_scan;
namespace wrs = fr5_scanner_650::workspace_robot_state;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using TrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using MovePlan = MoveGroup::Plan;
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;
int g_crash_trace_fd = -1;

void crashTraceSignalHandler(int signal_number)
{
  constexpr char message[] =
    "\nFATAL: workspace_coarse_scan_planner received a crash signal; native backtrace follows:\n";
  (void)::write(STDERR_FILENO, message, sizeof(message) - 1U);
  if (g_crash_trace_fd >= 0) {
    (void)::write(g_crash_trace_fd, message, sizeof(message) - 1U);
  }
  void * frames[64];
  const int frame_count = ::backtrace(frames, 64);
  ::backtrace_symbols_fd(frames, frame_count, STDERR_FILENO);
  if (g_crash_trace_fd >= 0) {
    ::backtrace_symbols_fd(frames, frame_count, g_crash_trace_fd);
    (void)::fsync(g_crash_trace_fd);
  }
  // SA_RESETHAND restored the default disposition when this handler began.
  // Re-raise after the handler returns so the normal core-dump path remains active.
  (void)::raise(signal_number);
}

void installCrashTraceHandlers()
{
  if (const char * path = std::getenv("SCANNER_650_CRASH_TRACE_PATH")) {
    g_crash_trace_fd = ::open(path, O_CREAT | O_WRONLY | O_APPEND, 0664);
  }
  struct sigaction action {};
  action.sa_handler = crashTraceSignalHandler;
  (void)::sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESETHAND;
  for (const int signal_number : {SIGABRT, SIGSEGV, SIGBUS, SIGFPE, SIGILL}) {
    (void)::sigaction(signal_number, &action, nullptr);
  }
}

double degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double radians(double degrees_value)
{
  return degrees_value * kPi / 180.0;
}

std::int64_t durationNanoseconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<std::int64_t>(duration.sec) * 1000000000LL + duration.nanosec;
}

builtin_interfaces::msg::Duration durationFromNanoseconds(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return result;
}

geometry_msgs::msg::Pose toRosPose(const ws::Pose & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.position.x;
  result.position.y = pose.position.y;
  result.position.z = pose.position.z;
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

ws::Pose fromEigenPose(const Eigen::Isometry3d & transform)
{
  Eigen::Quaterniond quaternion(transform.rotation());
  quaternion.normalize();
  // q and -q represent the same rotation. Keep the largest component
  // positive so checkpoint signatures do not flip merely because Eigen chose
  // the other equivalent quaternion representation.
  Eigen::Index largest_index = 0;
  quaternion.coeffs().cwiseAbs().maxCoeff(&largest_index);
  if (quaternion.coeffs()[largest_index] < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  ws::Pose result;
  result.position = {
    transform.translation().x(), transform.translation().y(), transform.translation().z()};
  result.orientation = {
    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
  return result;
}

Eigen::Isometry3d toEigenPose(const ws::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  quaternion.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

std::string joinIds(const std::set<int> & ids)
{
  std::ostringstream stream;
  bool first = true;
  for (int id : ids) {
    if (!first) {stream << ',';}
    stream << id;
    first = false;
  }
  return stream.str();
}

struct PathEvaluation
{
  bool passed{false};
  std::size_t sample_count{0U};
  double minimum_joint_margin_deg{std::numeric_limits<double>::infinity()};
  double minimum_normalized_sigma{std::numeric_limits<double>::infinity()};
  double maximum_joint_step_deg{0.0};
  std::string detail;
};

enum class LaneState
{
  Candidate,
  PreliminaryRejected,
  KinematicRejected,
  PilzRejected,
  Planned,
  Completed
};

struct LaneReport
{
  LaneState state{LaneState::Candidate};
  PathEvaluation evaluation;
  std::string detail;
};

struct StoredLane
{
  ws::ScanLane geometry;
  MovePlan transition_plan;
  MovePlan scan_plan;
  bool has_transition{false};
  double estimated_seconds{0.0};
};

struct StoredReturnSegment
{
  MovePlan plan;
  std::string label;
  std::vector<geometry_msgs::msg::Point> marker_points;
  double estimated_seconds{0.0};
};

struct StoredWorkspacePlan
{
  ws::WorkspacePlan geometry;
  std::vector<LaneReport> reports;
  std::vector<StoredLane> executable_lanes;
  std::vector<StoredReturnSegment> return_segments;
  std::set<int> completed_lane_ids;
  ws::Pose initial_tcp_pose;
  std::vector<std::string> initial_joint_names;
  std::vector<double> initial_joint_positions;
  std::string id;
  std::string mode;
  std::string return_status{"not_attempted"};
  double reachable_measurement_length_m{0.0};
  double coverage_ratio{0.0};
  double estimated_seconds{0.0};
  double return_estimated_seconds{0.0};
  bool return_to_start_planned{false};
  bool visualization_ready{false};
  bool environment_ready{false};
  bool approved{false};
};

class WorkspaceCoarseScanPlanner final : public rclcpp::Node
{
public:
  WorkspaceCoarseScanPlanner()
  : Node("workspace_coarse_scan_planner")
  {
    planning_group_ = declare_parameter<std::string>(
      "planning_group", "fairino5_v6_group");
    end_effector_link_ = declare_parameter<std::string>(
      "end_effector_link", "scanner_650_scan_tcp");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    planning_pipeline_ = declare_parameter<std::string>(
      "planning_pipeline", "pilz_industrial_motion_planner");
    planning_time_s_ = declare_parameter<double>("planning_time_s", 10.0);
    scan_speed_m_s_ = declare_parameter<double>("scan_speed_m_s", 0.01);
    transition_speed_m_s_ = declare_parameter<double>("transition_speed_m_s", 0.01);
    pilz_max_translational_speed_m_s_ = declare_parameter<double>(
      "pilz_max_translational_speed_m_s", 0.05);
    acceleration_scaling_ = declare_parameter<double>("acceleration_scaling", 0.30);
    ik_timeout_s_ = declare_parameter<double>("ik_timeout_s", 0.02);
    maximum_angular_sample_step_deg_ = declare_parameter<double>(
      "maximum_angular_sample_step_deg", 5.0);
    minimum_joint_margin_deg_ = declare_parameter<double>(
      "minimum_joint_margin_deg", 10.0);
    minimum_normalized_sigma_ = declare_parameter<double>(
      "minimum_normalized_singular_value", 0.05);
    jacobian_characteristic_length_m_ = declare_parameter<double>(
      "jacobian_characteristic_length_m", 0.10);
    j1_absolute_limit_deg_ = declare_parameter<double>("j1_absolute_limit_deg", 85.0);
    maximum_joint_step_deg_ = declare_parameter<double>("maximum_joint_step_deg", 20.0);
    elbow_joint_name_ = declare_parameter<std::string>("elbow_joint_name", "j3");
    preserve_elbow_sign_ = declare_parameter<bool>("preserve_elbow_sign", true);
    elbow_sign_deadband_deg_ = declare_parameter<double>("elbow_sign_deadband_deg", 2.0);
    allow_execution_ = declare_parameter<bool>("allow_execution", false);
    collision_scene_validated_ = declare_parameter<bool>(
      "collision_scene_validated", false);
    require_environment_objects_ = declare_parameter<bool>(
      "require_environment_collision_objects", true);
    require_laser_control_ = declare_parameter<bool>("require_laser_control", true);
    accumulation_service_name_ = declare_parameter<std::string>(
      "accumulation_service", "/scanner_650/set_accumulation");
    laser_service_name_ = declare_parameter<std::string>(
      "laser_service", "/scanner_650/set_laser");
    controller_action_name_ = declare_parameter<std::string>(
      "controller_action", "/fairino5_controller/follow_joint_trajectory");
    trajectory_start_tolerance_rad_ = declare_parameter<double>(
      "trajectory_start_tolerance_rad", 0.01);
    trajectory_start_correction_limit_rad_ = declare_parameter<double>(
      "trajectory_start_correction_limit_rad", 0.02);
    joint_settle_timeout_s_ = declare_parameter<double>("joint_settle_timeout_s", 3.0);
    joint_settle_sample_period_s_ = declare_parameter<double>(
      "joint_settle_sample_period_s", 0.10);
    joint_settle_delta_rad_ = declare_parameter<double>("joint_settle_delta_rad", 0.001);
    joint_settle_required_samples_ = declare_parameter<int>(
      "joint_settle_required_samples", 3);
    return_to_start_after_scan_ = declare_parameter<bool>(
      "return_to_start_after_scan", true);
    return_velocity_scaling_ = declare_parameter<double>(
      "return_velocity_scaling", 0.10);
    return_acceleration_scaling_ = declare_parameter<double>(
      "return_acceleration_scaling", 0.10);
    return_joint_tolerance_rad_ = declare_parameter<double>(
      "return_joint_tolerance_rad", 0.002);
    return_position_tolerance_m_ = declare_parameter<double>(
      "return_position_tolerance_m", 0.001);
    return_orientation_tolerance_deg_ = declare_parameter<double>(
      "return_orientation_tolerance_deg", 0.2);
    checkpoint_path_ = declare_parameter<std::string>(
      "checkpoint_path",
      "workspace_coarse_checkpoint.yaml");
    local_roi_min_ = declare_parameter<std::vector<double>>(
      "local_rescan_roi_min", {-0.10, -0.10, 0.20});
    local_roi_max_ = declare_parameter<std::vector<double>>(
      "local_rescan_roi_max", {0.10, 0.10, 0.40});
    local_roi_padding_m_ = declare_parameter<double>("local_rescan_roi_padding_m", 0.225);
    use_initial_tcp_height_ = declare_parameter<bool>("use_initial_tcp_height", true);
    scan_surface_mode_ = declare_parameter<std::string>(
      "scan_surface_mode", "tabletop_radial_fan");
    options_.tabletop_downward_scan = scan_surface_mode_ != "vertical_front";
    options_.tabletop_radial_fan_scan = scan_surface_mode_ == "tabletop_radial_fan";
    options_.fan_origin_at_initial_tcp = declare_parameter<bool>(
      "fan_origin_at_initial_tcp", true);
    use_current_tcp_radius_as_inner_ = declare_parameter<bool>(
      "use_current_tcp_radius_as_inner", false);

    options_.azimuth_min_deg = declare_parameter<double>("azimuth_min_deg", -60.0);
    options_.azimuth_max_deg = declare_parameter<double>("azimuth_max_deg", 60.0);
    options_.radial_min_m = declare_parameter<double>("radial_min_m", 0.0);
    options_.radial_max_m = declare_parameter<double>("radial_max_m", 0.72);
    options_.radial_spacing_m = declare_parameter<double>("radial_spacing_m", 0.125);
    options_.angular_lane_spacing_m = declare_parameter<double>(
      "angular_lane_spacing_m", 0.22);
    options_.minimum_radial_scan_length_m = declare_parameter<double>(
      "minimum_radial_scan_length_m", 0.10);
    options_.height_min_m = declare_parameter<double>("height_min_m", 0.475);
    options_.height_max_m = declare_parameter<double>("height_max_m", 0.475);
    options_.height_spacing_m = declare_parameter<double>("height_spacing_m", 0.225);
    options_.lead_in_m = declare_parameter<double>("lead_in_m", 0.04);
    options_.lead_out_m = declare_parameter<double>("lead_out_m", 0.04);
    options_.maximum_sample_step_m = declare_parameter<double>(
      "maximum_cartesian_sample_step_m", 0.02);
    options_.maximum_measurement_arc_span_deg = declare_parameter<double>(
      "maximum_measurement_arc_span_deg", 45.0);
    options_.optical_roll_deg = declare_parameter<double>("optical_roll_deg", 180.0);
    options_.working_distance_m = declare_parameter<double>("working_distance_m", 0.55);
    options_.camera_to_tcp_tangent_m = declare_parameter<double>(
      "camera_to_tcp_tangent_m", 0.0466476147);
    options_.base_exclusion_radius_m = declare_parameter<double>(
      "base_exclusion_radius_m", 0.35);
    options_.maximum_arm_reach_m = declare_parameter<double>("maximum_arm_reach_m", 0.922);
    options_.reach_fraction = declare_parameter<double>("reach_fraction", 0.85);
    options_.shoulder_height_m = declare_parameter<double>("shoulder_height_m", 0.152);
    options_.nominal_laser_line_width_m = declare_parameter<double>(
      "nominal_laser_line_width_m", 0.45);
    options_.maximum_lane_count = static_cast<std::size_t>(
      declare_parameter<int>("maximum_lane_count", 128));
    options_.maximum_samples_per_lane = static_cast<std::size_t>(
      declare_parameter<int>("maximum_samples_per_lane", 4096));
    validateParameters();
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

    MoveGroup::Options move_group_options(planning_group_);
    move_group_ = std::make_unique<MoveGroup>(
      node, move_group_options, nullptr, rclcpp::Duration::from_seconds(20.0));
    if (!move_group_->setEndEffectorLink(end_effector_link_)) {
      throw std::runtime_error("MoveIt rejected end_effector_link=" + end_effector_link_);
    }
    move_group_->setPlanningPipelineId(planning_pipeline_);
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

    planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, "robot_description", "workspace_coarse_scan_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene()) {
      throw std::runtime_error("failed to construct planning scene monitor");
    }
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor("/joint_states");

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/scanner_650/workspace_coarse_scan_markers",
      rclcpp::QoS(1).transient_local().reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/scanner_650/workspace_coarse_scan_status",
      rclcpp::QoS(1).transient_local().reliable());

    plan_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/plan_workspace_coarse_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {planWorkspace(false, response);},
      rmw_qos_profile_services_default, callback_group_);
    local_rescan_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/plan_workspace_local_rescan",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {planWorkspace(true, response);},
      rmw_qos_profile_services_default, callback_group_);
    approve_service_ = create_service<std_srvs::srv::SetBool>(
      "/scanner_650/approve_workspace_coarse_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {approvePlan(request->data, response);},
      rmw_qos_profile_services_default, callback_group_);
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/execute_workspace_coarse_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {executePlan(response);},
      rmw_qos_profile_services_default, callback_group_);
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/stop_workspace_coarse_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {stopExecution(response);},
      rmw_qos_profile_services_default, callback_group_);
    clear_checkpoint_service_ = create_service<std_srvs::srv::Trigger>(
      "/scanner_650/clear_workspace_coarse_checkpoint",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {clearCheckpoint(response);},
      rmw_qos_profile_services_default, callback_group_);

    std::ostringstream startup;
    startup << "workspace coarse scan ready: front=" << options_.azimuth_min_deg << ".."
            << options_.azimuth_max_deg << " deg relative to initial "
            << (options_.tabletop_downward_scan ? "TCP +X scan axis" : "optical axis")
            << ", surface_mode=" << scan_surface_mode_ << ", radial="
            << options_.radial_min_m << ".."
            << options_.radial_max_m << " m, height="
            << (use_initial_tcp_height_ ? "initial TCP Z (single plane)" :
              std::to_string(options_.height_min_m) + ".." +
              std::to_string(options_.height_max_m) + " m")
            << ", sample_step=" << options_.maximum_sample_step_m * 1000.0 << " mm"
            << (options_.tabletop_radial_fan_scan ?
              ", fan_lines_at_most=3" :
              ", arc_segment=" +
              std::to_string(options_.maximum_measurement_arc_span_deg) + " deg")
            << ", start_tolerance=" << trajectory_start_tolerance_rad_ << " rad"
            << ", correction_limit=" << trajectory_start_correction_limit_rad_ << " rad"
            << ", return_to_start="
            << (return_to_start_after_scan_ ? "validated direct PTP" : "disabled")
            << ", return_velocity_scaling=" << return_velocity_scaling_
            << ", return_acceleration_scaling=" << return_acceleration_scaling_
            << ", optical_roll="
            << options_.optical_roll_deg << " deg, execution="
            << (allow_execution_ ? "unlocked" : "locked") << ", collision_scene="
            << (collision_scene_validated_ ? "operator-validated" : "UNVALIDATED");
    publishStatus(startup.str());
    RCLCPP_WARN(get_logger(), "%s", startup.str().c_str());
  }

private:
  void validateParameters() const
  {
    std::string error;
    if (!ws::validateOptions(options_, &error)) {
      throw std::runtime_error("invalid workspace geometry: " + error);
    }
    if (planning_group_.empty() || end_effector_link_.empty() || base_frame_.empty() ||
      planning_pipeline_.empty() || controller_action_name_.empty() ||
      accumulation_service_name_.empty() || (require_laser_control_ && laser_service_name_.empty()))
    {
      throw std::runtime_error("empty workspace coarse scan interface parameter");
    }
    if (scan_surface_mode_ != "tabletop_radial_fan" &&
      scan_surface_mode_ != "tabletop_downward" && scan_surface_mode_ != "vertical_front")
    {
      throw std::runtime_error(
              "scan_surface_mode must be tabletop_radial_fan, tabletop_downward or vertical_front");
    }
    if (!(std::isfinite(scan_speed_m_s_) && scan_speed_m_s_ > 0.0 &&
      scan_speed_m_s_ <= pilz_max_translational_speed_m_s_ &&
      std::isfinite(transition_speed_m_s_) && transition_speed_m_s_ > 0.0 &&
      transition_speed_m_s_ <= pilz_max_translational_speed_m_s_ &&
      std::isfinite(acceleration_scaling_) && acceleration_scaling_ > 0.0 &&
      acceleration_scaling_ <= 0.50 && std::isfinite(ik_timeout_s_) && ik_timeout_s_ > 0.0 &&
      std::isfinite(minimum_joint_margin_deg_) && minimum_joint_margin_deg_ >= 10.0 &&
      std::isfinite(minimum_normalized_sigma_) && minimum_normalized_sigma_ >= 0.05 &&
      std::isfinite(j1_absolute_limit_deg_) && j1_absolute_limit_deg_ <= 85.0 &&
      std::isfinite(maximum_joint_step_deg_) && maximum_joint_step_deg_ > 0.0 &&
      std::isfinite(jacobian_characteristic_length_m_) && jacobian_characteristic_length_m_ > 0.0 &&
      std::isfinite(trajectory_start_tolerance_rad_) &&
      trajectory_start_tolerance_rad_ > 0.0 && trajectory_start_tolerance_rad_ <= 0.02 &&
      std::isfinite(trajectory_start_correction_limit_rad_) &&
      trajectory_start_correction_limit_rad_ >= trajectory_start_tolerance_rad_ &&
      trajectory_start_correction_limit_rad_ <= 0.05 &&
      std::isfinite(joint_settle_timeout_s_) && joint_settle_timeout_s_ >= 0.2 &&
      joint_settle_timeout_s_ <= 10.0 &&
      std::isfinite(joint_settle_sample_period_s_) && joint_settle_sample_period_s_ >= 0.02 &&
      joint_settle_sample_period_s_ <= 1.0 &&
      std::isfinite(joint_settle_delta_rad_) && joint_settle_delta_rad_ > 0.0 &&
      joint_settle_delta_rad_ <= trajectory_start_tolerance_rad_ &&
      joint_settle_required_samples_ >= 1 && joint_settle_required_samples_ <= 10 &&
      std::isfinite(return_velocity_scaling_) && return_velocity_scaling_ > 0.0 &&
      return_velocity_scaling_ <= 0.50 &&
      std::isfinite(return_acceleration_scaling_) && return_acceleration_scaling_ > 0.0 &&
      return_acceleration_scaling_ <= 0.50 &&
      std::isfinite(return_joint_tolerance_rad_) && return_joint_tolerance_rad_ > 0.0 &&
      return_joint_tolerance_rad_ <= trajectory_start_tolerance_rad_ &&
      std::isfinite(return_position_tolerance_m_) && return_position_tolerance_m_ > 0.0 &&
      return_position_tolerance_m_ <= 0.01 &&
      std::isfinite(return_orientation_tolerance_deg_) &&
      return_orientation_tolerance_deg_ > 0.0 && return_orientation_tolerance_deg_ <= 1.0))
    {
      throw std::runtime_error(
              "unsafe speed, joint-margin, J1, IK, acceleration, singularity, settling or "
              "return-to-start parameters");
    }
    if (local_roi_min_.size() != 3U || local_roi_max_.size() != 3U ||
      !std::isfinite(local_roi_padding_m_) || local_roi_padding_m_ < 0.0)
    {
      throw std::runtime_error("local rescan ROI must contain two finite XYZ triples");
    }
  }

  double velocityScaling(double requested_speed_m_s) const
  {
    return std::clamp(requested_speed_m_s / pilz_max_translational_speed_m_s_, 0.01, 1.0);
  }

  bool refreshPlanningGeometry(std::string * error)
  {
    options_.azimuth_min_deg = get_parameter("azimuth_min_deg").as_double();
    options_.azimuth_max_deg = get_parameter("azimuth_max_deg").as_double();
    options_.radial_min_m = get_parameter("radial_min_m").as_double();
    options_.radial_max_m = get_parameter("radial_max_m").as_double();
    options_.radial_spacing_m = get_parameter("radial_spacing_m").as_double();
    options_.angular_lane_spacing_m = get_parameter("angular_lane_spacing_m").as_double();
    options_.minimum_radial_scan_length_m =
      get_parameter("minimum_radial_scan_length_m").as_double();
    options_.height_min_m = get_parameter("height_min_m").as_double();
    options_.height_max_m = get_parameter("height_max_m").as_double();
    options_.height_spacing_m = get_parameter("height_spacing_m").as_double();
    use_initial_tcp_height_ = get_parameter("use_initial_tcp_height").as_bool();
    options_.fan_origin_at_initial_tcp =
      get_parameter("fan_origin_at_initial_tcp").as_bool();
    use_current_tcp_radius_as_inner_ =
      get_parameter("use_current_tcp_radius_as_inner").as_bool();
    scan_surface_mode_ = get_parameter("scan_surface_mode").as_string();
    if (scan_surface_mode_ != "tabletop_radial_fan" &&
      scan_surface_mode_ != "tabletop_downward" && scan_surface_mode_ != "vertical_front")
    {
      if (error) {
        *error =
          "scan_surface_mode must be tabletop_radial_fan, tabletop_downward or vertical_front";
      }
      return false;
    }
    options_.tabletop_downward_scan = scan_surface_mode_ != "vertical_front";
    options_.tabletop_radial_fan_scan = scan_surface_mode_ == "tabletop_radial_fan";
    options_.lead_in_m = get_parameter("lead_in_m").as_double();
    options_.lead_out_m = get_parameter("lead_out_m").as_double();
    options_.maximum_measurement_arc_span_deg =
      get_parameter("maximum_measurement_arc_span_deg").as_double();
    options_.optical_roll_deg = get_parameter("optical_roll_deg").as_double();
    options_.nominal_laser_line_width_m =
      get_parameter("nominal_laser_line_width_m").as_double();
    scan_speed_m_s_ = get_parameter("scan_speed_m_s").as_double();
    transition_speed_m_s_ = get_parameter("transition_speed_m_s").as_double();
    acceleration_scaling_ = get_parameter("acceleration_scaling").as_double();
    trajectory_start_tolerance_rad_ =
      get_parameter("trajectory_start_tolerance_rad").as_double();
    trajectory_start_correction_limit_rad_ =
      get_parameter("trajectory_start_correction_limit_rad").as_double();
    joint_settle_timeout_s_ = get_parameter("joint_settle_timeout_s").as_double();
    joint_settle_sample_period_s_ =
      get_parameter("joint_settle_sample_period_s").as_double();
    joint_settle_delta_rad_ = get_parameter("joint_settle_delta_rad").as_double();
    joint_settle_required_samples_ =
      static_cast<int>(get_parameter("joint_settle_required_samples").as_int());
    return_to_start_after_scan_ = get_parameter("return_to_start_after_scan").as_bool();
    return_velocity_scaling_ = get_parameter("return_velocity_scaling").as_double();
    return_acceleration_scaling_ = get_parameter("return_acceleration_scaling").as_double();
    return_joint_tolerance_rad_ = get_parameter("return_joint_tolerance_rad").as_double();
    return_position_tolerance_m_ = get_parameter("return_position_tolerance_m").as_double();
    return_orientation_tolerance_deg_ =
      get_parameter("return_orientation_tolerance_deg").as_double();
    local_roi_min_ = get_parameter("local_rescan_roi_min").as_double_array();
    local_roi_max_ = get_parameter("local_rescan_roi_max").as_double_array();
    local_roi_padding_m_ = get_parameter("local_rescan_roi_padding_m").as_double();
    if (!ws::validateOptions(options_, error)) {
      return false;
    }
    if (local_roi_min_.size() != 3U || local_roi_max_.size() != 3U ||
      !std::isfinite(local_roi_padding_m_) || local_roi_padding_m_ < 0.0)
    {
      if (error) {*error = "local rescan ROI must contain two finite XYZ triples";}
      return false;
    }
    if (!(std::isfinite(scan_speed_m_s_) && scan_speed_m_s_ > 0.0 &&
      scan_speed_m_s_ <= pilz_max_translational_speed_m_s_ &&
      std::isfinite(transition_speed_m_s_) && transition_speed_m_s_ > 0.0 &&
      transition_speed_m_s_ <= pilz_max_translational_speed_m_s_ &&
      std::isfinite(acceleration_scaling_) && acceleration_scaling_ > 0.0 &&
      acceleration_scaling_ <= 0.50 &&
      std::isfinite(trajectory_start_tolerance_rad_) &&
      trajectory_start_tolerance_rad_ > 0.0 && trajectory_start_tolerance_rad_ <= 0.02 &&
      std::isfinite(trajectory_start_correction_limit_rad_) &&
      trajectory_start_correction_limit_rad_ >= trajectory_start_tolerance_rad_ &&
      trajectory_start_correction_limit_rad_ <= 0.05 &&
      std::isfinite(joint_settle_timeout_s_) && joint_settle_timeout_s_ >= 0.2 &&
      joint_settle_timeout_s_ <= 10.0 &&
      std::isfinite(joint_settle_sample_period_s_) && joint_settle_sample_period_s_ >= 0.02 &&
      joint_settle_sample_period_s_ <= 1.0 &&
      std::isfinite(joint_settle_delta_rad_) && joint_settle_delta_rad_ > 0.0 &&
      joint_settle_delta_rad_ <= trajectory_start_tolerance_rad_ &&
      joint_settle_required_samples_ >= 1 && joint_settle_required_samples_ <= 10 &&
      std::isfinite(return_velocity_scaling_) && return_velocity_scaling_ > 0.0 &&
      return_velocity_scaling_ <= 0.50 &&
      std::isfinite(return_acceleration_scaling_) && return_acceleration_scaling_ > 0.0 &&
      return_acceleration_scaling_ <= 0.50 &&
      std::isfinite(return_joint_tolerance_rad_) && return_joint_tolerance_rad_ > 0.0 &&
      return_joint_tolerance_rad_ <= trajectory_start_tolerance_rad_ &&
      std::isfinite(return_position_tolerance_m_) && return_position_tolerance_m_ > 0.0 &&
      return_position_tolerance_m_ <= 0.01 &&
      std::isfinite(return_orientation_tolerance_deg_) &&
      return_orientation_tolerance_deg_ > 0.0 && return_orientation_tolerance_deg_ <= 1.0))
    {
      if (error) {
        *error = "scan/transition speed, acceleration, settling or return-to-start parameters "
          "are unsafe; return PTP velocity/acceleration scaling must be in (0, 0.50]";
      }
      return false;
    }
    return true;
  }

  bool environmentReady(std::string * detail) const
  {
    if (!collision_scene_validated_) {
      if (detail) {*detail = "collision_scene_validated is false";}
      return false;
    }
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    if (!scene) {
      if (detail) {*detail = "planning scene is unavailable";}
      return false;
    }
    const std::size_t object_count = scene->getWorld()->getObjectIds().size();
    if (require_environment_objects_ && object_count == 0U) {
      if (detail) {*detail = "validated scene contains no environment collision objects/octomap";}
      return false;
    }
    if (detail) {
      *detail = "operator-validated planning scene, world_objects=" + std::to_string(object_count);
    }
    return true;
  }

  double jointValue(const moveit::core::RobotState & state, const std::string & name) const
  {
    const auto & names = state.getRobotModel()->getVariableNames();
    return std::find(names.begin(), names.end(), name) != names.end() ?
           state.getVariablePosition(name) : std::numeric_limits<double>::quiet_NaN();
  }

  bool evaluateState(
    const planning_scene::PlanningSceneConstPtr & scene,
    moveit::core::RobotState * state, const moveit::core::RobotState * previous,
    double elbow_sign, PathEvaluation * evaluation, std::string * error) const
  {
    state->update();
    if (scene->isStateColliding(*state, planning_group_, false)) {
      if (error) {*error = "collision detected";}
      return false;
    }
    const moveit::core::JointModelGroup * group =
      state->getRobotModel()->getJointModelGroup(planning_group_);
    const auto & variable_names = group->getVariableNames();
    for (const std::string & variable : variable_names) {
      const double value = state->getVariablePosition(variable);
      const auto & bounds = state->getRobotModel()->getVariableBounds(variable);
      if (!std::isfinite(value)) {
        if (error) {*error = variable + " is non-finite";}
        return false;
      }
      if (bounds.position_bounded_) {
        const double margin = degrees(std::min(
            value - bounds.min_position_, bounds.max_position_ - value));
        evaluation->minimum_joint_margin_deg = std::min(
          evaluation->minimum_joint_margin_deg, margin);
        if (margin < minimum_joint_margin_deg_) {
          if (error) {
            *error = variable + " joint-limit margin=" + std::to_string(margin) + " deg";
          }
          return false;
        }
      }
      if (previous) {
        const double step = degrees(std::fabs(value - previous->getVariablePosition(variable)));
        evaluation->maximum_joint_step_deg = std::max(evaluation->maximum_joint_step_deg, step);
        if (step > maximum_joint_step_deg_) {
          if (error) {*error = variable + " IK branch jump=" + std::to_string(step) + " deg";}
          return false;
        }
      }
    }
    const double j1 = jointValue(*state, "j1");
    if (!std::isfinite(j1) || std::fabs(degrees(j1)) > j1_absolute_limit_deg_) {
      if (error) {*error = "J1 exceeds +/-" + std::to_string(j1_absolute_limit_deg_) + " deg";}
      return false;
    }
    if (preserve_elbow_sign_ && elbow_sign != 0.0) {
      const double elbow = jointValue(*state, elbow_joint_name_);
      if (!std::isfinite(elbow) ||
        (std::fabs(degrees(elbow)) > elbow_sign_deadband_deg_ && elbow * elbow_sign < 0.0))
      {
        if (error) {*error = "elbow configuration changed sign at " + elbow_joint_name_;}
        return false;
      }
    }

    const moveit::core::LinkModel * link = state->getLinkModel(end_effector_link_);
    Eigen::MatrixXd jacobian;
    if (!link || !state->getJacobian(group, link, Eigen::Vector3d::Zero(), jacobian, false) ||
      jacobian.rows() < 6 || jacobian.cols() == 0)
    {
      if (error) {*error = "MoveIt Jacobian is unavailable";}
      return false;
    }
    jacobian.block(0, 0, 3, jacobian.cols()) /= jacobian_characteristic_length_m_;
    const Eigen::VectorXd singular_values =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian).singularValues();
    const double minimum_sigma = singular_values.minCoeff();
    if (!std::isfinite(minimum_sigma)) {
      if (error) {*error = "Jacobian singular value is non-finite";}
      return false;
    }
    evaluation->minimum_normalized_sigma = std::min(
      evaluation->minimum_normalized_sigma, minimum_sigma);
    if (minimum_sigma < minimum_normalized_sigma_) {
      if (error) {*error = "normalized singular value=" + std::to_string(minimum_sigma);}
      return false;
    }
    ++evaluation->sample_count;
    return true;
  }

  bool evaluatePosePath(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & seed, const std::vector<ws::Pose> & poses,
    double elbow_sign, moveit::core::RobotState * terminal,
    PathEvaluation * evaluation) const
  {
    if (!terminal || !evaluation || poses.empty()) {
      return false;
    }
    *evaluation = {};
    moveit::core::RobotState state(seed);
    const moveit::core::JointModelGroup * group =
      state.getRobotModel()->getJointModelGroup(planning_group_);
    if (!group) {
      evaluation->detail = "planning group is absent from robot model";
      return false;
    }
    const std::vector<double> consistency_limits(
      group->getVariableCount(), radians(maximum_joint_step_deg_));
    for (std::size_t index = 0U; index < poses.size(); ++index) {
      moveit::core::RobotState previous(state);
      const auto collision_callback =
        [&scene, this](moveit::core::RobotState * candidate,
          const moveit::core::JointModelGroup * candidate_group, const double * positions)
        {
          candidate->setJointGroupPositions(candidate_group, positions);
          candidate->update();
          return !scene->isStateColliding(*candidate, planning_group_, false);
        };
      if (!state.setFromIK(
          group, toEigenPose(poses[index]), end_effector_link_, consistency_limits,
          ik_timeout_s_, collision_callback))
      {
        evaluation->detail = "continuous collision-free IK failed at sample " +
          std::to_string(index + 1U) + "/" + std::to_string(poses.size());
        return false;
      }
      std::string state_error;
      if (!evaluateState(scene, &state, &previous, elbow_sign, evaluation, &state_error)) {
        evaluation->detail = "sample " + std::to_string(index + 1U) + "/" +
          std::to_string(poses.size()) + ": " + state_error;
        return false;
      }
    }
    evaluation->passed = true;
    std::ostringstream detail;
    detail << "samples=" << evaluation->sample_count << ", min_joint_margin_deg="
           << evaluation->minimum_joint_margin_deg << ", min_normalized_sigma="
           << evaluation->minimum_normalized_sigma << ", max_joint_step_deg="
           << evaluation->maximum_joint_step_deg;
    evaluation->detail = detail.str();
    *terminal = state;
    return true;
  }

  std::vector<double> discoverRadialOuterLimits(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & current, const ws::Pose & current_pose,
    double elbow_sign) const
  {
    const std::vector<double> azimuths = ws::tabletopRadialAzimuths(options_);
    std::vector<double> limits(
      azimuths.size(), std::numeric_limits<double>::quiet_NaN());
    const double outer_lead = std::max(options_.lead_in_m, options_.lead_out_m);
    const double probe_start_radius = options_.fan_origin_at_initial_tcp ?
      options_.radial_min_m : options_.radial_min_m - outer_lead;
    const double probe_end_radius = options_.radial_max_m + outer_lead;
    // A fan anchored at the taught TCP intentionally starts at zero travel.
    // Only negative radial coordinates are invalid here.
    if (probe_start_radius < 0.0 || probe_end_radius <= probe_start_radius) {
      return limits;
    }
    for (std::size_t index = 0U; index < azimuths.size(); ++index) {
      const ws::Pose probe_start = ws::tabletopRadialPose(
        options_, probe_start_radius, options_.height_min_m, azimuths[index]);
      std::string sample_error;
      const std::vector<ws::Pose> transition_samples = ws::sampleLinearPoses(
        current_pose, probe_start, options_.maximum_sample_step_m,
        maximum_angular_sample_step_deg_, options_.maximum_samples_per_lane, &sample_error);
      moveit::core::RobotState probe_seed(current);
      PathEvaluation transition_evaluation;
      if (transition_samples.empty() || !evaluatePosePath(
          scene, current, transition_samples, elbow_sign,
          &probe_seed, &transition_evaluation))
      {
        RCLCPP_WARN(
          get_logger(), "radial boundary angle %.1f deg has no safe inner transition: %s",
          azimuths[index], transition_evaluation.detail.c_str());
        continue;
      }

      const ws::Pose probe_end = ws::tabletopRadialPose(
        options_, probe_end_radius, options_.height_min_m, azimuths[index]);
      const std::vector<ws::Pose> radial_samples = ws::sampleLinearPoses(
        probe_start, probe_end, options_.maximum_sample_step_m,
        maximum_angular_sample_step_deg_, options_.maximum_samples_per_lane, &sample_error);
      if (radial_samples.empty()) {
        continue;
      }
      moveit::core::RobotState ignored_terminal(probe_seed);
      PathEvaluation radial_evaluation;
      const bool all_safe = evaluatePosePath(
        scene, probe_seed, radial_samples, elbow_sign,
        &ignored_terminal, &radial_evaluation);
      const std::size_t safe_count = all_safe ? radial_samples.size() :
        radial_evaluation.sample_count;
      if (safe_count == 0U) {
        RCLCPP_WARN(
          get_logger(), "radial boundary angle %.1f deg is unsafe at the inner probe: %s",
          azimuths[index], radial_evaluation.detail.c_str());
        continue;
      }
      const ws::Pose & last_safe = radial_samples[safe_count - 1U];
      const double safe_probe_radius = options_.fan_origin_at_initial_tcp ?
        std::hypot(
        last_safe.position.x - options_.initial_tcp_fan_origin.x,
        last_safe.position.y - options_.initial_tcp_fan_origin.y) :
        std::hypot(last_safe.position.x, last_safe.position.y);
      double safe_outer = std::min(
        options_.radial_max_m, safe_probe_radius - outer_lead);
      safe_outer = std::floor(safe_outer * 1000.0 + 1.0e-9) / 1000.0;
      if (safe_outer - options_.radial_min_m >= options_.minimum_radial_scan_length_m) {
        limits[index] = safe_outer;
        RCLCPP_INFO(
          get_logger(), "radial boundary angle %.1f deg: %.3f..%.3f m%s",
          azimuths[index], options_.radial_min_m, safe_outer,
          all_safe ? "" : " (trimmed by continuous safety probe)");
      } else {
        RCLCPP_WARN(
          get_logger(), "radial boundary angle %.1f deg leaves only %.3f m, below minimum",
          azimuths[index], std::max(0.0, safe_outer - options_.radial_min_m));
      }
    }
    return limits;
  }

  bool evaluateJointTrajectory(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & seed, const MovePlan & plan,
    double elbow_sign, PathEvaluation * evaluation) const
  {
    *evaluation = {};
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.joint_names.empty() || trajectory.points.empty()) {
      evaluation->detail = "Pilz returned an empty joint trajectory";
      return false;
    }
    moveit::core::RobotState state(seed);
    moveit::core::RobotState previous(seed);
    for (std::size_t point_index = 0U; point_index < trajectory.points.size(); ++point_index) {
      const auto & point = trajectory.points[point_index];
      if (point.positions.size() != trajectory.joint_names.size()) {
        evaluation->detail = "Pilz trajectory point has inconsistent dimensions";
        return false;
      }
      previous = state;
      for (std::size_t joint_index = 0U; joint_index < trajectory.joint_names.size(); ++joint_index) {
        state.setVariablePosition(trajectory.joint_names[joint_index], point.positions[joint_index]);
      }
      std::string state_error;
      if (!evaluateState(scene, &state, &previous, elbow_sign, evaluation, &state_error)) {
        evaluation->detail = "Pilz point " + std::to_string(point_index + 1U) + "/" +
          std::to_string(trajectory.points.size()) + ": " + state_error;
        return false;
      }
    }
    evaluation->passed = true;
    evaluation->detail = "validated " + std::to_string(evaluation->sample_count) +
      " generated Pilz trajectory points";
    return true;
  }

  bool terminalState(
    const moveit::core::RobotState & seed, const MovePlan & plan,
    moveit::core::RobotState * terminal, std::string * error) const
  {
    if (!terminal) {return false;}
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.joint_names.empty() || trajectory.points.empty() ||
      trajectory.points.back().positions.size() != trajectory.joint_names.size())
    {
      if (error) {*error = "trajectory terminal state is invalid";}
      return false;
    }
    *terminal = seed;
    for (std::size_t index = 0U; index < trajectory.joint_names.size(); ++index) {
      terminal->setVariablePosition(
        trajectory.joint_names[index], trajectory.points.back().positions[index]);
    }
    terminal->update();
    return true;
  }

  bool planJointTarget(
    const moveit::core::RobotState & current,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions,
    const planning_scene::PlanningSceneConstPtr & scene,
    double elbow_sign, const std::string & purpose,
    MovePlan * result_plan, std::string * error)
  {
    if (!result_plan || !scene || joint_names.empty() ||
      joint_names.size() != joint_positions.size())
    {
      if (error) {*error = purpose + " joint target is invalid";}
      return false;
    }
    std::map<std::string, double> target_joints;
    for (std::size_t index = 0U; index < joint_names.size(); ++index) {
      if (!std::isfinite(joint_positions[index])) {
        if (error) {*error = purpose + " contains a non-finite joint target";}
        return false;
      }
      target_joints[joint_names[index]] = joint_positions[index];
    }
    move_group_->setStartState(current);
    move_group_->setPlanningPipelineId(planning_pipeline_);
    move_group_->setPlannerId("PTP");
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setMaxVelocityScalingFactor(return_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(return_acceleration_scaling_);
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    if (!move_group_->setJointValueTarget(target_joints)) {
      if (error) {*error = "MoveIt rejected the " + purpose + " joint target";}
      return false;
    }
    const moveit::core::MoveItErrorCode result = move_group_->plan(*result_plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      if (error) {
        *error = "Pilz PTP " + purpose + " failed with MoveIt code=" +
          std::to_string(result.val);
      }
      return false;
    }
    PathEvaluation evaluation;
    if (!evaluateJointTrajectory(scene, current, *result_plan, elbow_sign, &evaluation)) {
      if (error) {*error = purpose + " safety check failed: " + evaluation.detail;}
      return false;
    }
    return true;
  }

  bool buildReturnPlan(
    StoredWorkspacePlan * workspace_plan,
    const moveit::core::RobotState & final_state,
    const planning_scene::PlanningSceneConstPtr & scene,
    double elbow_sign, std::string * error)
  {
    if (!workspace_plan) {
      if (error) {*error = "workspace return plan output is null";}
      return false;
    }
    workspace_plan->return_segments.clear();
    workspace_plan->return_estimated_seconds = 0.0;
    workspace_plan->return_to_start_planned = false;
    if (!return_to_start_after_scan_ || workspace_plan->executable_lanes.empty()) {
      return true;
    }

    if (workspace_plan->initial_joint_names.size() !=
      workspace_plan->initial_joint_positions.size() ||
      workspace_plan->initial_joint_names.empty())
    {
      if (error) {*error = "stored initial joint state has inconsistent dimensions";}
      return false;
    }
    StoredReturnSegment segment;
    segment.label = "direct PTP return to planning start";
    if (!planJointTarget(
        final_state, workspace_plan->initial_joint_names,
        workspace_plan->initial_joint_positions, scene, elbow_sign,
        segment.label, &segment.plan, error))
    {
      return false;
    }

    // Cache the one direct return polyline while planning. Execution replans
    // it once from the real settled scan endpoint, but never republishes a
    // large marker array while the robot is moving.
    moveit::core::RobotState marker_state(final_state);
    const moveit::core::LinkModel * marker_tcp_link =
      marker_state.getLinkModel(end_effector_link_);
    if (!marker_tcp_link) {
      if (error) {*error = segment.label + " cannot resolve the process TCP for RViz";}
      return false;
    }
    const auto & marker_trajectory = segment.plan.trajectory_.joint_trajectory;
    if (marker_trajectory.points.empty()) {
      if (error) {*error = segment.label + " generated an empty trajectory";}
      return false;
    }
    constexpr std::size_t kMaximumMarkerPoints = 512U;
    const std::size_t marker_stride = std::max<std::size_t>(
      1U, (marker_trajectory.points.size() + kMaximumMarkerPoints - 1U) /
      kMaximumMarkerPoints);
    const auto append_marker_point = [&](std::size_t point_index) -> bool {
        const auto & trajectory_point = marker_trajectory.points[point_index];
        if (trajectory_point.positions.size() != marker_trajectory.joint_names.size()) {
          if (error) {*error = segment.label + " has an invalid RViz trajectory point";}
          return false;
        }
        for (std::size_t joint_index = 0U;
          joint_index < marker_trajectory.joint_names.size(); ++joint_index)
        {
          marker_state.setVariablePosition(
            marker_trajectory.joint_names[joint_index],
            trajectory_point.positions[joint_index]);
        }
        marker_state.update();
        const Eigen::Vector3d position =
          marker_state.getGlobalLinkTransform(marker_tcp_link).translation();
        if (!position.allFinite()) {
          if (error) {*error = segment.label + " generated a non-finite RViz TCP point";}
          return false;
        }
        geometry_msgs::msg::Point point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        segment.marker_points.push_back(point);
        return true;
      };
    for (std::size_t point_index = 0U;
      point_index < marker_trajectory.points.size(); point_index += marker_stride)
    {
      if (!append_marker_point(point_index)) {return false;}
    }
    const std::size_t last_marker_index = marker_trajectory.points.size() - 1U;
    if (last_marker_index % marker_stride != 0U &&
      !append_marker_point(last_marker_index))
    {
      return false;
    }

    moveit::core::RobotState terminal(final_state);
    if (!terminalState(final_state, segment.plan, &terminal, error)) {
      if (error) {*error = segment.label + ": " + *error;}
      return false;
    }
    double maximum_error = 0.0;
    std::string maximum_joint;
    for (std::size_t index = 0U; index < workspace_plan->initial_joint_names.size(); ++index) {
      const double difference = std::fabs(
        terminal.getVariablePosition(workspace_plan->initial_joint_names[index]) -
        workspace_plan->initial_joint_positions[index]);
      if (!std::isfinite(difference)) {
        if (error) {*error = "return terminal joint difference is non-finite";}
        return false;
      }
      if (difference > maximum_error) {
        maximum_error = difference;
        maximum_joint = workspace_plan->initial_joint_names[index];
      }
    }
    if (maximum_error > return_joint_tolerance_rad_) {
      if (error) {
        *error = "direct return path does not close at the planning start (" + maximum_joint + "=" +
          std::to_string(maximum_error) + " rad)";
      }
      return false;
    }
    const auto & points = segment.plan.trajectory_.joint_trajectory.points;
    segment.estimated_seconds = static_cast<double>(
      durationNanoseconds(points.back().time_from_start)) * 1.0e-9;
    segment.estimated_seconds +=
      joint_settle_sample_period_s_ * static_cast<double>(joint_settle_required_samples_);
    workspace_plan->return_estimated_seconds = segment.estimated_seconds;
    workspace_plan->return_segments.push_back(std::move(segment));
    workspace_plan->return_to_start_planned = true;
    workspace_plan->estimated_seconds += workspace_plan->return_estimated_seconds;
    return true;
  }

  bool planMotion(
    const moveit::core::RobotState & start_state, const ws::Pose & goal,
    const std::string & planner_id, double speed_m_s,
    const std::optional<ws::Pose> & circular_interim, MovePlan * plan,
    std::string * error)
  {
    move_group_->setStartState(start_state);
    move_group_->setPlanningPipelineId(planning_pipeline_);
    move_group_->setPlannerId(planner_id);
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setMaxVelocityScalingFactor(velocityScaling(speed_m_s));
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->clearPathConstraints();
    if (circular_interim) {
      moveit_msgs::msg::Constraints constraints;
      constraints.name = "interim";
      moveit_msgs::msg::PositionConstraint position;
      position.header.frame_id = base_frame_;
      position.link_name = end_effector_link_;
      position.weight = 1.0;
      shape_msgs::msg::SolidPrimitive sphere;
      sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      // Pilz overloads path_constraints to carry the CIRC auxiliary point,
      // while MoveIt's Humble planning pipeline later evaluates the same
      // message as an ordinary all-waypoint path constraint. A tiny sphere
      // therefore rejects every valid circle except the auxiliary point.
      // The primitive pose remains the exact interim point consumed by Pilz;
      // a workspace-enclosing carrier radius makes the generic recheck inert.
      // Every generated point is independently rechecked below for collision,
      // joint margin, J1, elbow continuity and singularity.
      sphere.dimensions = {10.0};
      geometry_msgs::msg::Pose center;
      center.position.x = circular_interim->position.x;
      center.position.y = circular_interim->position.y;
      center.position.z = circular_interim->position.z;
      center.orientation.w = 1.0;
      position.constraint_region.primitives.push_back(sphere);
      position.constraint_region.primitive_poses.push_back(center);
      constraints.position_constraints.push_back(position);
      move_group_->setPathConstraints(constraints);
    }
    move_group_->setPoseTarget(toRosPose(goal), end_effector_link_);
    const moveit::core::MoveItErrorCode result = move_group_->plan(*plan);
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      if (error) {
        *error = "Pilz " + planner_id + " failed with MoveIt code=" +
          std::to_string(result.val);
      }
      return false;
    }
    return true;
  }

  bool appendTrajectory(MovePlan * destination, const MovePlan & source, std::string * error) const
  {
    auto & target = destination->trajectory_.joint_trajectory;
    const auto & addition = source.trajectory_.joint_trajectory;
    if (addition.points.empty()) {return true;}
    if (target.points.empty()) {
      *destination = source;
      return true;
    }
    if (target.joint_names != addition.joint_names) {
      if (error) {*error = "cannot merge Pilz trajectories with different joint orders";}
      return false;
    }
    const std::int64_t offset = durationNanoseconds(target.points.back().time_from_start);
    for (std::size_t index = 1U; index < addition.points.size(); ++index) {
      auto point = addition.points[index];
      std::int64_t time = offset + durationNanoseconds(point.time_from_start);
      if (time <= durationNanoseconds(target.points.back().time_from_start)) {
        time = durationNanoseconds(target.points.back().time_from_start) + 1LL;
      }
      point.time_from_start = durationFromNanoseconds(time);
      target.points.push_back(std::move(point));
    }
    return true;
  }

  bool planScanLane(
    const moveit::core::RobotState & start_state, const ws::ScanLane & lane,
    MovePlan * merged, moveit::core::RobotState * terminal, std::string * error)
  {
    moveit::core::RobotState state(start_state);
    std::vector<std::pair<std::string, MovePlan>> components;
    components.reserve(4U);
    auto add_component = [&](
      const std::string & name, const ws::Pose & goal,
      const std::string & planner_id, const std::optional<ws::Pose> & interim) -> bool
      {
        MovePlan component;
        std::string component_error;
        if (!planMotion(
            state, goal, planner_id, scan_speed_m_s_, interim, &component, &component_error))
        {
          if (error) {*error = name + ": " + component_error;}
          return false;
        }
        if (!terminalState(state, component, &state, &component_error)) {
          if (error) {*error = name + ": " + component_error;}
          return false;
        }
        components.emplace_back(name, std::move(component));
        return true;
      };

    if (!add_component("lead-in", lane.measurement_start, "LIN", std::nullopt)) {
      return false;
    }
    if (lane.radial_scan) {
      if (!add_component(
          "radial measurement LIN", lane.measurement_end, "LIN", std::nullopt) ||
        !add_component("lead-out", lane.motion_end, "LIN", std::nullopt))
      {
        return false;
      }
    } else {
      const std::size_t last = lane.measurement_samples.size() - 1U;
      const ws::Pose first_interim = lane.measurement_samples[last / 4U];
      const ws::Pose second_interim = lane.measurement_samples[(3U * last) / 4U];
      if (!add_component("first CIRC", lane.measurement_mid, "CIRC", first_interim) ||
        !add_component("second CIRC", lane.measurement_end, "CIRC", second_interim) ||
        !add_component("lead-out", lane.motion_end, "LIN", std::nullopt))
      {
        return false;
      }
    }
    *merged = components.front().second;
    for (std::size_t index = 1U; index < components.size(); ++index) {
      if (!appendTrajectory(merged, components[index].second, error)) {
        return false;
      }
    }
    *terminal = state;
    return true;
  }

  std::string geometrySignature(
    const ws::WorkspacePlan & geometry, const std::string & mode,
    const std::vector<int> & selected_ids) const
  {
    std::ostringstream canonical;
    canonical << std::setprecision(17) << mode << '|' << geometry.options.azimuth_min_deg << '|'
              << geometry.options.azimuth_max_deg << '|' << geometry.options.azimuth_center_deg << '|'
              << geometry.options.radial_min_m << '|'
              << geometry.options.radial_max_m << '|' << geometry.options.radial_spacing_m << '|'
              << geometry.options.height_min_m << '|' << geometry.options.height_max_m << '|'
              << geometry.options.height_spacing_m << '|' << geometry.options.lead_in_m << '|'
              << geometry.options.lead_out_m << '|'
              << geometry.options.maximum_measurement_arc_span_deg << '|'
              << geometry.options.optical_roll_deg << '|' << geometry.options.working_distance_m
              << '|' << geometry.options.tabletop_downward_scan << '|'
              << geometry.options.tabletop_radial_fan_scan << '|'
              << geometry.options.fan_origin_at_initial_tcp << '|'
              << geometry.options.initial_tcp_fan_origin.x << '|'
              << geometry.options.initial_tcp_fan_origin.y << '|'
              << geometry.options.initial_tcp_fan_origin.z << '|'
              << geometry.options.angular_lane_spacing_m << '|'
              << geometry.options.minimum_radial_scan_length_m << '|'
              << geometry.options.initial_center_orientation.x << '|'
              << geometry.options.initial_center_orientation.y << '|'
              << geometry.options.initial_center_orientation.z << '|'
              << geometry.options.initial_center_orientation.w;
    for (const ws::ScanLane & lane : geometry.lanes) {
      // A radial fan's reachable outer limit is discovered from the current
      // planning scene. Include it in the identity so an old checkpoint can
      // never be resumed against a different safe boundary.
      canonical << '|' << lane.id << '|' << lane.radial_start_m << '|'
                << lane.radial_end_m << '|' << lane.azimuth_start_deg << '|'
                << lane.preliminary_safe;
    }
    for (int id : selected_ids) {canonical << '|' << id;}
    std::uint64_t hash = 1469598103934665603ULL;
    for (unsigned char value : canonical.str()) {
      hash ^= value;
      hash *= 1099511628211ULL;
    }
    std::ostringstream encoded;
    encoded << "workspace-" << std::hex << std::setw(16) << std::setfill('0') << hash;
    return encoded.str();
  }

  std::set<int> readCheckpoint(const std::string & expected_plan_id) const
  {
    std::ifstream input(checkpoint_path_);
    if (!input) {return {};}
    std::string line;
    std::string plan_id;
    std::set<int> completed;
    while (std::getline(input, line)) {
      if (line.rfind("plan_id:", 0U) == 0U) {
        plan_id = line.substr(std::string("plan_id:").size());
        plan_id.erase(0U, plan_id.find_first_not_of(" \t\""));
        const auto end = plan_id.find_last_not_of(" \t\r\n\"");
        if (end != std::string::npos) {plan_id.erase(end + 1U);}
      } else if (line.rfind("completed_lane_ids:", 0U) == 0U) {
        const auto begin = line.find('[');
        const auto end = line.find(']');
        if (begin != std::string::npos && end != std::string::npos && end > begin) {
          std::stringstream values(line.substr(begin + 1U, end - begin - 1U));
          std::string token;
          while (std::getline(values, token, ',')) {
            try {completed.insert(std::stoi(token));} catch (...) {}
          }
        }
      }
    }
    return plan_id == expected_plan_id ? completed : std::set<int>{};
  }

  bool writeCheckpoint(const StoredWorkspacePlan & plan, std::string * error) const
  {
    try {
      const std::filesystem::path target(checkpoint_path_);
      if (target.has_parent_path()) {
        std::filesystem::create_directories(target.parent_path());
      }
      const std::filesystem::path temporary = target.string() + ".tmp";
      std::ofstream output(temporary, std::ios::trunc);
      if (!output) {
        if (error) {*error = "cannot open checkpoint temporary file";}
        return false;
      }
      output << "schema_version: 2\n"
             << "plan_id: \"" << plan.id << "\"\n"
             << "mode: \"" << plan.mode << "\"\n"
             << "return_status: \"" << plan.return_status << "\"\n"
             << "completed_lane_ids: [" << joinIds(plan.completed_lane_ids) << "]\n";
      output.flush();
      if (!output) {
        if (error) {*error = "failed to flush checkpoint";}
        return false;
      }
      output.close();
      std::filesystem::rename(temporary, target);
      return true;
    } catch (const std::exception & exception) {
      if (error) {*error = exception.what();}
      return false;
    }
  }

  void planWorkspace(
    bool local_rescan, const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock() || execution_active_.load()) {
      response->success = false;
      response->message = "another workspace planning/execution operation is active";
      return;
    }
    // A new planning request invalidates the previously approved executable
    // transaction even if this new request later fails.
    active_plan_.reset();
    stop_requested_.store(false);
    publishStatus(local_rescan ? "planning local rescan ROI" : "planning full workspace coarse scan");

    ws::WorkspacePlan geometry;
    std::string error;
    if (!refreshPlanningGeometry(&error)) {
      response->success = false;
      response->message = "workspace parameters failed validation: " + error;
      return;
    }
    const moveit::core::RobotStatePtr current = move_group_->getCurrentState(2.0);
    if (!current) {
      response->success = false;
      response->message = "cannot read the operator-provided initial robot state";
      return;
    }
    const moveit::core::LinkModel * tcp_link = current->getLinkModel(end_effector_link_);
    if (!tcp_link) {
      response->success = false;
      response->message = "end-effector link is absent from robot state";
      return;
    }
    const Eigen::Isometry3d initial_tcp_transform = current->getGlobalLinkTransform(tcp_link);
    const ws::Pose initial_tcp_pose = fromEigenPose(initial_tcp_transform);
    options_.initial_tcp_fan_origin = initial_tcp_pose.position;
    if (use_initial_tcp_height_) {
      const double raw_plane_z_m = initial_tcp_transform.translation().z();
      if (!std::isfinite(raw_plane_z_m) ||
        (!options_.tabletop_downward_scan && raw_plane_z_m < 0.0))
      {
        response->success = false;
        response->message =
          "initial TCP height cannot define a valid " + scan_surface_mode_ + " scan plane";
        publishStatus(response->message);
        return;
      }
      // One-millimetre quantization stabilizes checkpoint identity while
      // keeping the commanded plane effectively at the taught initial height.
      const double plane_z_m = std::round(raw_plane_z_m * 1000.0) / 1000.0;
      options_.height_min_m = plane_z_m;
      options_.height_max_m = plane_z_m;
    }
    options_.initial_center_orientation = initial_tcp_pose.orientation;
    const Eigen::Vector3d initial_optical_axis = initial_tcp_transform.linear().col(2);
    double raw_center_deg = 0.0;
    if (options_.tabletop_downward_scan) {
      if (!std::isfinite(initial_optical_axis.z()) || initial_optical_axis.z() > -0.80) {
        response->success = false;
        std::ostringstream detail;
        detail << "tabletop mode requires scanner TCP +Z to point downward; current base-Z "
               << "component=" << initial_optical_axis.z() << " (required <= -0.80)";
        response->message = detail.str();
        publishStatus(response->message);
        return;
      }
      // The operator-defined tabletop forward direction is calibrated TCP +X.
      // TCP +Y follows the laser line and TCP +Z remains pointed down.
      const Eigen::Vector3d initial_scan_axis = initial_tcp_transform.linear().col(0);
      const double horizontal_scan_norm = std::hypot(
        initial_scan_axis.x(), initial_scan_axis.y());
      if (!std::isfinite(horizontal_scan_norm) || horizontal_scan_norm < 0.80) {
        response->success = false;
        response->message =
          "tabletop mode cannot project scanner TCP +X scan axis onto the table";
        publishStatus(response->message);
        return;
      }
      raw_center_deg = degrees(std::atan2(
        initial_scan_axis.y(), initial_scan_axis.x()));
    } else {
      const double horizontal_optical_norm = std::hypot(
        initial_optical_axis.x(), initial_optical_axis.y());
      if (!std::isfinite(horizontal_optical_norm) || horizontal_optical_norm < 0.20) {
        response->success = false;
        response->message =
          "vertical-front mode requires scanner TCP +Z optical axis to have a horizontal heading";
        publishStatus(response->message);
        return;
      }
      raw_center_deg = degrees(std::atan2(
        initial_optical_axis.y(), initial_optical_axis.x()));
    }
    // Stabilize checkpoint identity against sub-millidegree joint-state noise.
    options_.azimuth_center_deg = std::round(raw_center_deg * 10.0) / 10.0;
    if (std::fabs(options_.azimuth_center_deg) < 0.05) {
      options_.azimuth_center_deg = 0.0;
    } else if (options_.azimuth_center_deg >= 180.0) {
      options_.azimuth_center_deg = -180.0;
    }
    if (options_.tabletop_radial_fan_scan && options_.fan_origin_at_initial_tcp) {
      // The taught process TCP is the fan apex. Radial values now mean travel
      // from that point, so measurement starts at zero by construction.
      options_.radial_min_m = 0.0;
    } else if (options_.tabletop_radial_fan_scan && use_current_tcp_radius_as_inner_) {
      const double current_radius = std::hypot(
        initial_tcp_transform.translation().x(), initial_tcp_transform.translation().y());
      if (!std::isfinite(current_radius)) {
        response->success = false;
        response->message = "current TCP radius is non-finite";
        return;
      }
      const double quantized_current_radius =
        std::ceil(current_radius * 1000.0 - 1.0e-9) / 1000.0;
      options_.radial_min_m = std::max(options_.radial_min_m, quantized_current_radius);
    }
    if (options_.tabletop_radial_fan_scan &&
      options_.radial_max_m - options_.radial_min_m < options_.minimum_radial_scan_length_m)
    {
      response->success = false;
      response->message =
        "safe inner radius leaves less than the minimum requested radial scan length";
      publishStatus(response->message);
      return;
    }

    bool geometry_ok = false;
    if (options_.tabletop_radial_fan_scan) {
      planning_scene_monitor::LockedPlanningSceneRO discovery_scene(planning_scene_monitor_);
      if (!discovery_scene) {
        response->success = false;
        response->message = "planning scene is unavailable for radial boundary discovery";
        return;
      }
      const double current_elbow = jointValue(*current, elbow_joint_name_);
      const double discovery_elbow_sign = !std::isfinite(current_elbow) ||
        std::fabs(degrees(current_elbow)) <= elbow_sign_deadband_deg_ ? 0.0 :
        (current_elbow > 0.0 ? 1.0 : -1.0);
      const std::vector<double> safe_outer_limits = discoverRadialOuterLimits(
        discovery_scene, *current, initial_tcp_pose, discovery_elbow_sign);
      geometry_ok = ws::buildTabletopRadialFanPlan(
        options_, safe_outer_limits, &geometry, &error);
    } else {
      geometry_ok = ws::buildLayeredHemisphericalPlan(options_, &geometry, &error);
    }
    if (!geometry_ok) {
      response->success = false;
      response->message = "workspace geometry failed: " + error;
      publishStatus(response->message);
      return;
    }
    std::vector<int> selected_ids;
    ws::Vec3 roi_min;
    ws::Vec3 roi_max;
    if (local_rescan) {
      roi_min = {local_roi_min_[0], local_roi_min_[1], local_roi_min_[2]};
      roi_max = {local_roi_max_[0], local_roi_max_[1], local_roi_max_[2]};
      if (roi_min.x > roi_max.x || roi_min.y > roi_max.y || roi_min.z > roi_max.z) {
        response->success = false;
        response->message = "local rescan ROI minimum must not exceed maximum";
        return;
      }
      for (const auto & lane : geometry.lanes) {
        if (ws::laneIntersectsAxisAlignedRoi(lane, roi_min, roi_max, local_roi_padding_m_)) {
          selected_ids.push_back(lane.id);
        }
      }
    } else {
      for (const auto & lane : geometry.lanes) {selected_ids.push_back(lane.id);}
    }
    if (selected_ids.empty()) {
      response->success = false;
      response->message = "no scan lane intersects the requested workspace/ROI";
      return;
    }

    StoredWorkspacePlan candidate_plan;
    candidate_plan.geometry = geometry;
    candidate_plan.initial_tcp_pose = initial_tcp_pose;
    candidate_plan.mode = local_rescan ? "local_rescan" : "coarse_scan";
    candidate_plan.id = geometrySignature(geometry, candidate_plan.mode, selected_ids);
    candidate_plan.reports.resize(geometry.lanes.size());
    if (!local_rescan) {
      candidate_plan.completed_lane_ids = readCheckpoint(candidate_plan.id);
    }
    const moveit::core::JointModelGroup * planning_group =
      current->getRobotModel()->getJointModelGroup(planning_group_);
    if (!planning_group) {
      response->success = false;
      response->message = "planning group is absent from the current robot model";
      return;
    }
    candidate_plan.initial_joint_names = planning_group->getVariableNames();
    candidate_plan.initial_joint_positions.reserve(candidate_plan.initial_joint_names.size());
    for (const std::string & joint : candidate_plan.initial_joint_names) {
      candidate_plan.initial_joint_positions.push_back(current->getVariablePosition(joint));
    }
    std::string environment_detail;
    candidate_plan.environment_ready = environmentReady(&environment_detail);

    moveit::core::RobotState chained_state(*current);
    ws::Pose chained_pose = fromEigenPose(initial_tcp_transform);
    const double current_elbow = jointValue(*current, elbow_joint_name_);
    const double elbow_sign = !std::isfinite(current_elbow) ||
      std::fabs(degrees(current_elbow)) <= elbow_sign_deadband_deg_ ? 0.0 :
      (current_elbow > 0.0 ? 1.0 : -1.0);

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
    if (!locked_scene) {
      response->success = false;
      response->message = "planning scene is unavailable";
      return;
    }
    const planning_scene::PlanningSceneConstPtr & scene = locked_scene;
    std::set<int> selected_set(selected_ids.begin(), selected_ids.end());
    for (const ws::ScanLane & lane : geometry.lanes) {
      LaneReport & report = candidate_plan.reports[static_cast<std::size_t>(lane.id)];
      if (selected_set.count(lane.id) == 0U) {
        report.state = LaneState::Candidate;
        report.detail = "outside selected local rescan ROI";
        continue;
      }
      if (candidate_plan.completed_lane_ids.count(lane.id) != 0U) {
        report.state = LaneState::Completed;
        report.detail = "restored from per-lane checkpoint";
        candidate_plan.reachable_measurement_length_m += lane.measurement_length_m;
        continue;
      }
      if (stop_requested_.load()) {
        response->success = false;
        response->message = "workspace planning stopped";
        return;
      }
      if (!lane.preliminary_safe) {
        report.state = LaneState::PreliminaryRejected;
        report.detail = lane.rejection_reason;
        continue;
      }

      std::vector<ws::Pose> transition_samples = ws::sampleLinearPoses(
        chained_pose, lane.motion_start, options_.maximum_sample_step_m,
        maximum_angular_sample_step_deg_, options_.maximum_samples_per_lane, &error);
      if (transition_samples.empty()) {
        report.state = LaneState::KinematicRejected;
        report.detail = "transition sampling failed: " + error;
        continue;
      }
      moveit::core::RobotState prechecked_state(chained_state);
      PathEvaluation transition_evaluation;
      PathEvaluation lane_evaluation;
      if (!evaluatePosePath(
          scene, chained_state, transition_samples, elbow_sign,
          &prechecked_state, &transition_evaluation) ||
        !evaluatePosePath(
          scene, prechecked_state, lane.motion_samples, elbow_sign,
          &prechecked_state, &lane_evaluation))
      {
        report.state = LaneState::KinematicRejected;
        report.evaluation = lane_evaluation.sample_count > 0U ?
          lane_evaluation : transition_evaluation;
        report.detail = transition_evaluation.passed ? lane_evaluation.detail :
          "transition: " + transition_evaluation.detail;
        continue;
      }
      report.evaluation = lane_evaluation;

      StoredLane stored;
      stored.geometry = lane;
      moveit::core::RobotState after_transition(chained_state);
      const bool transition_needed =
        ws::positionDistance(chained_pose, lane.motion_start) > 1.0e-5 ||
        ws::orientationDistanceDeg(chained_pose, lane.motion_start) > 0.01;
      if (transition_needed) {
        if (!planMotion(
            chained_state, lane.motion_start, "LIN", transition_speed_m_s_, std::nullopt,
            &stored.transition_plan, &error) ||
          !terminalState(chained_state, stored.transition_plan, &after_transition, &error))
        {
          report.state = LaneState::PilzRejected;
          report.detail = "laser-OFF transition: " + error;
          continue;
        }
        stored.has_transition = true;
      }
      moveit::core::RobotState after_scan(after_transition);
      if (!planScanLane(after_transition, lane, &stored.scan_plan, &after_scan, &error)) {
        report.state = LaneState::PilzRejected;
        report.detail = error;
        continue;
      }
      PathEvaluation generated_evaluation;
      if ((stored.has_transition && !evaluateJointTrajectory(
          scene, chained_state, stored.transition_plan, elbow_sign, &generated_evaluation)) ||
        !evaluateJointTrajectory(
          scene, after_transition, stored.scan_plan, elbow_sign, &generated_evaluation))
      {
        report.state = LaneState::PilzRejected;
        report.detail = "generated trajectory safety check: " + generated_evaluation.detail;
        continue;
      }

      const auto & scan_points = stored.scan_plan.trajectory_.joint_trajectory.points;
      stored.estimated_seconds = scan_points.empty() ? 0.0 :
        static_cast<double>(durationNanoseconds(scan_points.back().time_from_start)) * 1.0e-9;
      if (stored.has_transition) {
        const auto & points = stored.transition_plan.trajectory_.joint_trajectory.points;
        if (!points.empty()) {
          stored.estimated_seconds +=
            static_cast<double>(durationNanoseconds(points.back().time_from_start)) * 1.0e-9;
        }
      }
      stored.estimated_seconds += 0.4;  // laser/accumulator handshakes
      report.state = LaneState::Planned;
      report.detail = lane_evaluation.detail;
      candidate_plan.estimated_seconds += stored.estimated_seconds;
      candidate_plan.reachable_measurement_length_m += lane.measurement_length_m;
      candidate_plan.executable_lanes.push_back(std::move(stored));
      chained_state = after_scan;
      chained_pose = lane.motion_end;
    }
    if (!buildReturnPlan(
        &candidate_plan, chained_state, scene, elbow_sign, &error))
    {
      response->success = false;
      response->message = "safe return-to-start planning failed: " + error;
      publishStatus(response->message);
      return;
    }
    const double selected_length = std::accumulate(
      geometry.lanes.begin(), geometry.lanes.end(), 0.0,
      [&](double sum, const ws::ScanLane & lane) {
        const double requested_length = lane.requested_measurement_length_m > 0.0 ?
          lane.requested_measurement_length_m : lane.measurement_length_m;
        return sum + (selected_set.count(lane.id) ? requested_length : 0.0);
      });
    candidate_plan.coverage_ratio = selected_length > 0.0 ?
      candidate_plan.reachable_measurement_length_m / selected_length : 0.0;
    candidate_plan.approved = false;
    for (const ws::ScanLane & lane : geometry.lanes) {
      const LaneReport & report = candidate_plan.reports[static_cast<std::size_t>(lane.id)];
      if (selected_set.count(lane.id) != 0U && report.state != LaneState::Planned &&
        report.state != LaneState::Completed)
      {
        RCLCPP_WARN(
          get_logger(),
          "lane %d rejected (az=%.1f deg, radial=%.3f->%.3f m, z=%.3f, reverse=%s): %s",
          lane.id, lane.azimuth_start_deg, lane.radial_start_m, lane.radial_end_m,
          lane.height_m, lane.reverse ? "true" : "false",
          report.detail.c_str());
      }
    }

    active_plan_ = std::make_unique<StoredWorkspacePlan>(std::move(candidate_plan));
    std::string marker_error;
    active_plan_->visualization_ready = publishMarkers(*active_plan_, true, &marker_error);
    std::ostringstream summary;
    summary << "plan_id=" << active_plan_->id << ", mode=" << active_plan_->mode
            << ", surface_mode=" << scan_surface_mode_
            << ", sector_center=" << std::fixed << std::setprecision(1)
            << active_plan_->geometry.options.azimuth_center_deg << " deg"
            << ", plane_z=" << std::setprecision(3)
            << active_plan_->geometry.options.height_min_m << " m"
            << (active_plan_->geometry.options.fan_origin_at_initial_tcp ?
              ", requested_travel_from_initial_tcp=" : ", requested_radial=")
            << active_plan_->geometry.options.radial_min_m << ".."
            << active_plan_->geometry.options.radial_max_m << " m"
            << ", planned_lanes=" << active_plan_->executable_lanes.size() << '/'
            << selected_ids.size() << ", path_coverage=" << std::fixed << std::setprecision(1)
            << active_plan_->coverage_ratio * 100.0 << "% (reachable lane length only), ETA="
            << active_plan_->estimated_seconds / 60.0 << " min, return_to_start="
            << (active_plan_->return_to_start_planned ?
              "validated direct PTP" : "disabled") << "; " << environment_detail
            << (active_plan_->visualization_ready ? "" :
              "; RViz visualization failed: " + marker_error)
            << "; laser data observes surfaces only and does not prove free volume";
    response->success = !active_plan_->executable_lanes.empty() &&
      active_plan_->visualization_ready;
    response->message = summary.str();
    publishStatus(summary.str());
  }

  void approvePlan(
    bool approved, const std::shared_ptr<std_srvs::srv::SetBool::Response> & response)
  {
    if (execution_active_.load()) {
      response->success = false;
      response->message = "cannot change approval during execution";
      return;
    }
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (!active_plan_) {
      response->success = false;
      response->message = "no workspace plan; call plan_workspace_coarse_scan first";
      return;
    }
    if (!approved) {
      active_plan_->approved = false;
      response->success = true;
      response->message = "workspace plan approval cleared";
      publishStatus(response->message);
      return;
    }
    std::string environment_detail;
    active_plan_->environment_ready = environmentReady(&environment_detail);
    if (!active_plan_->environment_ready) {
      active_plan_->approved = false;
      response->success = false;
      response->message = "approval refused: " + environment_detail +
        "; unknown space cannot be authorized by a software checkbox";
      return;
    }
    if (active_plan_->executable_lanes.empty()) {
      response->success = false;
      response->message = "approval refused: plan has no reachable lane";
      return;
    }
    if (!active_plan_->visualization_ready) {
      response->success = false;
      response->message = "approval refused: RViz plan visualization did not publish; replan "
        "before execution";
      return;
    }
    active_plan_->approved = true;
    response->success = true;
    response->message = "approved " + active_plan_->id +
      " after validated collision-scene check; execute service remains separately gated";
    publishStatus(response->message);
  }

  bool requestBooleanService(
    const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr & client,
    bool enabled, const std::string & label, std::string * error)
  {
    if (!client->wait_for_service(2s)) {
      if (error) {*error = label + " service is unavailable";}
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = client->async_send_request(request);
    // Scanner accumulation stop may legitimately use its 5 s drain window.
    if (future.wait_for(7s) != std::future_status::ready) {
      if (error) {*error = label + " service timed out";}
      return false;
    }
    const auto result = future.get();
    if (!result->success && error) {*error = result->message;}
    return result->success;
  }

  bool setInterlocks(bool scan_enabled, std::string * error)
  {
    if (!scan_enabled) {
      std::string accumulation_error;
      const bool accumulation_off = requestBooleanService(
        accumulation_client_, false, "reconstruction accumulation", &accumulation_error);
      std::string laser_error;
      const bool laser_off = !require_laser_control_ || requestBooleanService(
        laser_client_, false, "650 nm laser", &laser_error);
      if ((!accumulation_off || !laser_off) && error) {
        *error = accumulation_error + " " + laser_error;
      }
      return accumulation_off && laser_off;
    }
    std::string laser_error;
    if (require_laser_control_ && !requestBooleanService(
        laser_client_, true, "650 nm laser", &laser_error))
    {
      if (error) {*error = laser_error;}
      return false;
    }
    std::string accumulation_error;
    if (!requestBooleanService(
        accumulation_client_, true, "reconstruction accumulation", &accumulation_error))
    {
      std::string ignored;
      if (require_laser_control_) {
        (void)requestBooleanService(laser_client_, false, "650 nm laser", &ignored);
      }
      if (error) {*error = accumulation_error;}
      return false;
    }
    return true;
  }

  bool trajectoryStartError(
    const MovePlan & plan, const moveit::core::RobotState & current,
    double * maximum_error_rad, std::string * maximum_error_joint,
    std::string * error) const
  {
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.points.empty() ||
      trajectory.points.front().positions.size() != trajectory.joint_names.size())
    {
      if (error) {*error = "trajectory has no valid start state";}
      return false;
    }
    double largest_error = 0.0;
    std::string largest_joint;
    const auto & model_variables = current.getRobotModel()->getVariableNames();
    for (std::size_t index = 0U; index < trajectory.joint_names.size(); ++index) {
      const std::string & joint = trajectory.joint_names[index];
      if (std::find(model_variables.begin(), model_variables.end(), joint) ==
        model_variables.end())
      {
        if (error) {*error = "trajectory contains unknown joint " + joint;}
        return false;
      }
      const double difference = std::fabs(
        current.getVariablePosition(joint) -
        trajectory.points.front().positions[index]);
      if (!std::isfinite(difference)) {
        if (error) {*error = joint + " trajectory start error is non-finite";}
        return false;
      }
      if (difference > largest_error) {
        largest_error = difference;
        largest_joint = joint;
      }
    }
    if (maximum_error_rad) {*maximum_error_rad = largest_error;}
    if (maximum_error_joint) {*maximum_error_joint = largest_joint;}
    return true;
  }

  moveit::core::RobotStatePtr waitForJointStability(
    const std::vector<std::string> & joint_names, std::string * error)
  {
    if (joint_names.empty()) {
      if (error) {*error = "cannot settle an empty joint list";}
      return {};
    }
    moveit::core::RobotStatePtr previous = move_group_->getCurrentState(1.0);
    if (!previous) {
      if (error) {*error = "cannot read current robot state while waiting for settling";}
      return {};
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(joint_settle_timeout_s_));
    int stable_samples = 0;
    double last_maximum_delta = std::numeric_limits<double>::infinity();
    while (std::chrono::steady_clock::now() < deadline) {
      if (stop_requested_.load()) {
        if (error) {*error = "stop requested while waiting for joint settling";}
        return {};
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(joint_settle_sample_period_s_));
      moveit::core::RobotStatePtr current = move_group_->getCurrentState(1.0);
      if (!current) {
        if (error) {*error = "cannot refresh robot state while waiting for settling";}
        return {};
      }
      last_maximum_delta = 0.0;
      for (const std::string & joint : joint_names) {
        const double delta = std::fabs(
          current->getVariablePosition(joint) - previous->getVariablePosition(joint));
        if (!std::isfinite(delta)) {
          if (error) {*error = joint + " settling delta is non-finite";}
          return {};
        }
        last_maximum_delta = std::max(last_maximum_delta, delta);
      }
      stable_samples = last_maximum_delta <= joint_settle_delta_rad_ ?
        stable_samples + 1 : 0;
      if (stable_samples >= joint_settle_required_samples_) {
        return current;
      }
      previous = current;
    }
    if (error) {
      *error = "joint state did not settle within " + std::to_string(joint_settle_timeout_s_) +
        " s (last maximum delta=" + std::to_string(last_maximum_delta) + " rad)";
    }
    return {};
  }

  bool startStateIsCurrent(const MovePlan & plan, std::string * error)
  {
    const moveit::core::RobotStatePtr current = move_group_->getCurrentState(1.0);
    if (!current) {
      if (error) {*error = "cannot read current robot state";}
      return false;
    }
    double maximum_error = 0.0;
    std::string maximum_joint;
    if (!trajectoryStartError(plan, *current, &maximum_error, &maximum_joint, error)) {
      return false;
    }
    if (maximum_error > trajectory_start_tolerance_rad_) {
      if (error) {
        *error = "robot moved since planning (" + maximum_joint + " start error=" +
          std::to_string(maximum_error) + " rad, tolerance=" +
          std::to_string(trajectory_start_tolerance_rad_) + " rad)";
      }
      return false;
    }
    return true;
  }

  bool executeTrajectory(const MovePlan & plan, std::string * error)
  {
    if (!startStateIsCurrent(plan, error)) {return false;}
    if (!trajectory_client_->wait_for_action_server(2s)) {
      if (error) {*error = "joint trajectory controller action is unavailable";}
      return false;
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = plan.trajectory_.joint_trajectory;
    goal.goal_time_tolerance.sec = 2;
    auto goal_future = trajectory_client_->async_send_goal(goal);
    if (goal_future.wait_for(2s) != std::future_status::ready) {
      if (error) {*error = "controller goal acceptance timed out";}
      return false;
    }
    const TrajectoryGoalHandle::SharedPtr goal_handle = goal_future.get();
    if (!goal_handle) {
      if (error) {*error = "controller rejected trajectory";}
      return false;
    }
    const double planned_seconds = static_cast<double>(
      durationNanoseconds(goal.trajectory.points.back().time_from_start)) * 1.0e-9;
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    const auto timeout = std::chrono::duration<double>(
      std::clamp(planned_seconds + 30.0, 30.0, 3600.0));
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      (void)trajectory_client_->async_cancel_goal(goal_handle);
      if (error) {*error = "controller result timed out; cancellation requested";}
      return false;
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      if (error) {
        *error = result.result ? result.result->error_string : "controller returned no result";
      }
      return false;
    }
    return true;
  }

  bool planStartCorrection(
    const moveit::core::RobotState & current, const MovePlan & target_plan,
    double elbow_sign, double velocity_scaling, double acceleration_scaling,
    MovePlan * correction_plan, std::string * error)
  {
    const auto & target_trajectory = target_plan.trajectory_.joint_trajectory;
    if (!correction_plan || target_trajectory.points.empty() ||
      target_trajectory.points.front().positions.size() != target_trajectory.joint_names.size())
    {
      if (error) {*error = "PTP joint target is invalid";}
      return false;
    }
    std::map<std::string, double> target_joints;
    for (std::size_t index = 0U; index < target_trajectory.joint_names.size(); ++index) {
      target_joints[target_trajectory.joint_names[index]] =
        target_trajectory.points.front().positions[index];
    }
    move_group_->setStartState(current);
    move_group_->setPlanningPipelineId(planning_pipeline_);
    move_group_->setPlannerId("PTP");
    move_group_->setPlanningTime(planning_time_s_);
    if (!std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 ||
      velocity_scaling > 1.0 || !std::isfinite(acceleration_scaling) ||
      acceleration_scaling <= 0.0 || acceleration_scaling > 1.0)
    {
      if (error) {*error = "PTP joint-target scaling is outside (0, 1]";}
      return false;
    }
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    if (!move_group_->setJointValueTarget(target_joints)) {
      if (error) {*error = "MoveIt rejected the PTP joint target";}
      return false;
    }
    const moveit::core::MoveItErrorCode result = move_group_->plan(*correction_plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      if (error) {
        *error = "Pilz PTP joint-target planning failed with MoveIt code=" +
          std::to_string(result.val);
      }
      return false;
    }
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    if (!scene) {
      if (error) {*error = "planning scene is unavailable for PTP joint-target validation";}
      return false;
    }
    PathEvaluation evaluation;
    if (!evaluateJointTrajectory(scene, current, *correction_plan, elbow_sign, &evaluation)) {
      if (error) {*error = "PTP joint-target safety check failed: " + evaluation.detail;}
      return false;
    }
    return true;
  }

  bool prepareTrajectoryStart(
    const MovePlan & plan, const std::string & label, double elbow_sign,
    std::string * error)
  {
    const auto & joint_names = plan.trajectory_.joint_trajectory.joint_names;
    moveit::core::RobotStatePtr stable = waitForJointStability(joint_names, error);
    if (!stable) {return false;}
    double maximum_error = 0.0;
    std::string maximum_joint;
    if (!trajectoryStartError(plan, *stable, &maximum_error, &maximum_joint, error)) {
      return false;
    }
    if (maximum_error <= trajectory_start_tolerance_rad_) {
      return true;
    }
    if (maximum_error > trajectory_start_correction_limit_rad_) {
      if (error) {
        *error = label + " start error exceeds correction limit (" + maximum_joint + "=" +
          std::to_string(maximum_error) + " rad, limit=" +
          std::to_string(trajectory_start_correction_limit_rad_) + " rad)";
      }
      return false;
    }
    std::string interlock_error;
    if (!setInterlocks(false, &interlock_error)) {
      if (error) {*error = label + " correction could not enforce laser OFF: " + interlock_error;}
      return false;
    }
    std::ostringstream correction_status;
    correction_status << label << " start correction with laser OFF: " << maximum_joint << '='
                      << std::fixed << std::setprecision(6) << maximum_error << " rad";
    publishStatus(correction_status.str());
    MovePlan correction_plan;
    std::string correction_error;
    if (!planStartCorrection(
        *stable, plan, elbow_sign, velocityScaling(transition_speed_m_s_),
        acceleration_scaling_, &correction_plan, &correction_error) ||
      !executeTrajectory(correction_plan, &correction_error))
    {
      if (error) {*error = label + " start correction failed: " + correction_error;}
      return false;
    }
    stable = waitForJointStability(joint_names, &correction_error);
    if (!stable || !trajectoryStartError(
        plan, *stable, &maximum_error, &maximum_joint, &correction_error))
    {
      if (error) {*error = label + " post-correction verification failed: " + correction_error;}
      return false;
    }
    if (maximum_error > trajectory_start_tolerance_rad_) {
      if (error) {
        *error = label + " remained outside start tolerance after correction (" +
          maximum_joint + "=" + std::to_string(maximum_error) + " rad, tolerance=" +
          std::to_string(trajectory_start_tolerance_rad_) + " rad)";
      }
      return false;
    }
    publishStatus(label + " start correction completed; laser remains OFF");
    return true;
  }

  bool revalidateBeforeExecution(const StoredLane & lane, double elbow_sign, std::string * error)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    const moveit::core::RobotStatePtr current = move_group_->getCurrentState(1.0);
    if (!scene || !current) {
      if (error) {*error = "current planning scene/state is unavailable";}
      return false;
    }
    PathEvaluation evaluation;
    moveit::core::RobotState after_transition(*current);
    if (lane.has_transition) {
      if (!evaluateJointTrajectory(scene, *current, lane.transition_plan, elbow_sign, &evaluation) ||
        !terminalState(*current, lane.transition_plan, &after_transition, error))
      {
        if (error && error->empty()) {*error = evaluation.detail;}
        return false;
      }
    }
    if (!evaluateJointTrajectory(scene, after_transition, lane.scan_plan, elbow_sign, &evaluation)) {
      if (error) {*error = evaluation.detail;}
      return false;
    }
    return true;
  }

  bool measureStartDeviation(
    const StoredWorkspacePlan & plan, const moveit::core::RobotState & actual,
    double * maximum_joint_error_rad, std::string * maximum_joint_name,
    double * tcp_position_error_m, double * tcp_orientation_error_deg,
    std::string * error) const
  {
    if (plan.initial_joint_names.empty() ||
      plan.initial_joint_names.size() != plan.initial_joint_positions.size())
    {
      if (error) {*error = "stored planning-start joint state is invalid";}
      return false;
    }
    const auto & model_variables = actual.getRobotModel()->getVariableNames();
    double largest_error = 0.0;
    std::string largest_joint;
    for (std::size_t index = 0U; index < plan.initial_joint_names.size(); ++index) {
      const std::string & joint = plan.initial_joint_names[index];
      if (std::find(model_variables.begin(), model_variables.end(), joint) ==
        model_variables.end())
      {
        if (error) {*error = "stored planning start contains unknown joint " + joint;}
        return false;
      }
      const double difference = std::fabs(
        actual.getVariablePosition(joint) - plan.initial_joint_positions[index]);
      if (!std::isfinite(difference)) {
        if (error) {*error = joint + " return-to-start error is non-finite";}
        return false;
      }
      if (difference > largest_error) {
        largest_error = difference;
        largest_joint = joint;
      }
    }
    Eigen::Isometry3d actual_tcp_transform = Eigen::Isometry3d::Identity();
    std::string transform_error;
    if (!wrs::updatedGlobalLinkTransform(
        actual, end_effector_link_, &actual_tcp_transform, &transform_error))
    {
      if (error) {
        *error = "cannot update end-effector transform while checking return-to-start: " +
          transform_error;
      }
      return false;
    }
    const ws::Pose actual_tcp = fromEigenPose(actual_tcp_transform);
    const double position_error = ws::positionDistance(actual_tcp, plan.initial_tcp_pose);
    const double orientation_error =
      ws::orientationDistanceDeg(actual_tcp, plan.initial_tcp_pose);
    if (!std::isfinite(position_error) || !std::isfinite(orientation_error)) {
      if (error) {*error = "TCP return-to-start error is non-finite";}
      return false;
    }
    if (maximum_joint_error_rad) {*maximum_joint_error_rad = largest_error;}
    if (maximum_joint_name) {*maximum_joint_name = largest_joint;}
    if (tcp_position_error_m) {*tcp_position_error_m = position_error;}
    if (tcp_orientation_error_deg) {*tcp_orientation_error_deg = orientation_error;}
    return true;
  }

  bool withinReturnTolerance(
    double joint_error_rad, double position_error_m, double orientation_error_deg) const
  {
    return joint_error_rad <= return_joint_tolerance_rad_ &&
           position_error_m <= return_position_tolerance_m_ &&
           orientation_error_deg <= return_orientation_tolerance_deg_;
  }

  std::string returnDeviationText(
    double joint_error_rad, const std::string & joint_name,
    double position_error_m, double orientation_error_deg) const
  {
    std::ostringstream stream;
    stream << "max_joint=" << (joint_name.empty() ? "none" : joint_name) << '='
           << std::fixed << std::setprecision(6) << joint_error_rad << " rad, TCP_position="
           << position_error_m * 1000.0 << " mm, TCP_orientation="
           << orientation_error_deg << " deg";
    return stream.str();
  }

  bool executeDirectReturnToStart(
    const StoredWorkspacePlan & plan, const moveit::core::RobotState & current,
    double elbow_sign, std::string * error)
  {
    std::string interlock_error;
    if (!setInterlocks(false, &interlock_error)) {
      if (error) {*error = "cannot confirm laser/accumulation OFF: " + interlock_error;}
      return false;
    }
    MovePlan start_target;
    auto & trajectory = start_target.trajectory_.joint_trajectory;
    trajectory.joint_names = plan.initial_joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = plan.initial_joint_positions;
    point.time_from_start = durationFromNanoseconds(0);
    trajectory.points.push_back(std::move(point));

    MovePlan correction;
    if (!planStartCorrection(
        current, start_target, elbow_sign, return_velocity_scaling_,
        return_acceleration_scaling_, &correction, error))
    {
      return false;
    }
    if (stop_requested_.load()) {
      if (error) {*error = "stop requested before direct return execution";}
      return false;
    }
    if (!executeTrajectory(correction, error)) {return false;}
    return static_cast<bool>(waitForJointStability(plan.initial_joint_names, error));
  }

  bool executeReturnToStart(
    StoredWorkspacePlan * plan, double elbow_sign, std::string * detail)
  {
    if (!plan || !plan->return_to_start_planned || plan->return_segments.empty()) {
      if (detail) {*detail = "approved plan contains no validated return path";}
      return false;
    }
    std::string error;
    moveit::core::RobotStatePtr stable = waitForJointStability(
      plan->initial_joint_names, &error);
    if (!stable) {
      if (detail) {*detail = "cannot measure final scan position: " + error;}
      return false;
    }

    double joint_error = 0.0;
    double position_error = 0.0;
    double orientation_error = 0.0;
    std::string joint_name;
    if (!measureStartDeviation(
        *plan, *stable, &joint_error, &joint_name, &position_error,
        &orientation_error, &error))
    {
      if (detail) {*detail = "cannot compare scan end with start: " + error;}
      return false;
    }
    const std::string initial_deviation = returnDeviationText(
      joint_error, joint_name, position_error, orientation_error);
    if (withinReturnTolerance(joint_error, position_error, orientation_error)) {
      if (detail) {*detail = "already at planning start; return skipped (" + initial_deviation + ")";}
      return true;
    }

    publishStatus("scan finished away from planning start; " + initial_deviation);
    if (joint_error <= std::numeric_limits<double>::epsilon()) {
      if (detail) {
        *detail = "joint state equals start but TCP deviation is inconsistent; refusing motion (" +
          initial_deviation + ")";
      }
      return false;
    }
    if (stop_requested_.load()) {
      if (detail) {*detail = "stop requested before direct return-to-start";}
      return false;
    }
    std::string environment_detail;
    if (!environmentReady(&environment_detail)) {
      if (detail) {*detail = "direct return refused: " + environment_detail;}
      return false;
    }
    publishStatus(
      "planning one direct PTP motion from the real scan endpoint to the saved start; "
      "laser and accumulation remain OFF");
    if (!executeDirectReturnToStart(*plan, *stable, elbow_sign, &error)) {
      if (detail) {*detail = "direct PTP return failed: " + error;}
      return false;
    }

    stable = waitForJointStability(plan->initial_joint_names, &error);
    if (!stable || !measureStartDeviation(
        *plan, *stable, &joint_error, &joint_name, &position_error,
        &orientation_error, &error))
    {
      if (detail) {*detail = "return final-state verification failed: " + error;}
      return false;
    }
    if (!withinReturnTolerance(joint_error, position_error, orientation_error) &&
      joint_error > return_joint_tolerance_rad_ &&
      joint_error <= trajectory_start_correction_limit_rad_)
    {
      publishStatus("return residual exceeds tolerance; applying final laser-OFF correction");
      if (!executeDirectReturnToStart(*plan, *stable, elbow_sign, &error)) {
        if (detail) {*detail = "final start correction failed: " + error;}
        return false;
      }
      stable = waitForJointStability(plan->initial_joint_names, &error);
      if (!stable || !measureStartDeviation(
          *plan, *stable, &joint_error, &joint_name, &position_error,
          &orientation_error, &error))
      {
        if (detail) {*detail = "post-correction start verification failed: " + error;}
        return false;
      }
    }
    const std::string final_deviation = returnDeviationText(
      joint_error, joint_name, position_error, orientation_error);
    if (!withinReturnTolerance(joint_error, position_error, orientation_error)) {
      if (detail) {*detail = "return ended outside tolerance (" + final_deviation + ")";}
      return false;
    }
    if (detail) {*detail = "returned to planning start (" + final_deviation + ")";}
    return true;
  }

  void executePlan(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::unique_lock<std::mutex> operation_lock(operation_mutex_, std::try_to_lock);
    if (!operation_lock.owns_lock()) {
      response->success = false;
      response->message = "another workspace planning/execution operation is active";
      return;
    }
    if (!allow_execution_) {
      response->success = false;
      response->message = "execution locked; relaunch with allow_execution:=true after commissioning";
      return;
    }
    if (!active_plan_ || !active_plan_->approved) {
      response->success = false;
      response->message = "no approved plan; plan, inspect RViz, then approve explicitly";
      return;
    }
    std::string environment_detail;
    if (!environmentReady(&environment_detail)) {
      active_plan_->approved = false;
      response->success = false;
      response->message = "execution refused: " + environment_detail;
      return;
    }
    execution_active_.store(true);
    stop_requested_.store(false);
    struct ExecutionGuard
    {
      explicit ExecutionGuard(std::atomic<bool> & active) : active_(active) {}
      ~ExecutionGuard() {active_.store(false);}
      std::atomic<bool> & active_;
    } execution_guard(execution_active_);

    const moveit::core::RobotStatePtr current = move_group_->getCurrentState(1.0);
    const double elbow = current ? jointValue(*current, elbow_joint_name_) : 0.0;
    const double elbow_sign = !std::isfinite(elbow) ||
      std::fabs(degrees(elbow)) <= elbow_sign_deadband_deg_ ? 0.0 : (elbow > 0.0 ? 1.0 : -1.0);
    std::size_t completed_now = 0U;
    std::string failure;
    for (StoredLane & lane : active_plan_->executable_lanes) {
      if (stop_requested_.load()) {
        failure = "stop requested";
        break;
      }
      publishStatus(
        "executing lane " + std::to_string(lane.geometry.id) + " of " + active_plan_->id);
      if (!revalidateBeforeExecution(lane, elbow_sign, &failure)) {
        failure = "pre-execution scene revalidation failed: " + failure;
        break;
      }
      if (!setInterlocks(false, &failure)) {
        failure = "laser-OFF transition refused: " + failure;
        break;
      }
      if (lane.has_transition) {
        if (!prepareTrajectoryStart(
            lane.transition_plan, "lane " + std::to_string(lane.geometry.id) + " transition",
            elbow_sign, &failure))
        {
          failure = "transition start preparation failed: " + failure;
          break;
        }
        if (!executeTrajectory(lane.transition_plan, &failure)) {
          failure = "transition failed: " + failure;
          break;
        }
      }
      if (stop_requested_.load()) {
        failure = "stop requested before measurement lane";
        break;
      }
      if (!prepareTrajectoryStart(
          lane.scan_plan, "lane " + std::to_string(lane.geometry.id) + " scan",
          elbow_sign, &failure))
      {
        failure = "scan start preparation failed: " + failure;
        break;
      }
      if (!setInterlocks(true, &failure)) {
        failure = "measurement interlock failed: " + failure;
        break;
      }
      const bool scan_ok = executeTrajectory(lane.scan_plan, &failure);
      std::string shutdown_error;
      const bool shutdown_ok = setInterlocks(false, &shutdown_error);
      if (!scan_ok || !shutdown_ok) {
        failure = scan_ok ? "scan completed but shutdown failed: " + shutdown_error :
          "scan trajectory failed: " + failure + "; shutdown: " + shutdown_error;
        break;
      }
      std::string settle_error;
      if (!waitForJointStability(
          lane.scan_plan.trajectory_.joint_trajectory.joint_names, &settle_error))
      {
        failure = "scan completed with laser OFF but joints did not settle: " + settle_error;
        break;
      }
      active_plan_->completed_lane_ids.insert(lane.geometry.id);
      active_plan_->reports[static_cast<std::size_t>(lane.geometry.id)].state = LaneState::Completed;
      std::string checkpoint_error;
      if (!writeCheckpoint(*active_plan_, &checkpoint_error)) {
        failure = "lane completed but checkpoint failed: " + checkpoint_error;
        break;
      }
      ++completed_now;
    }
    std::string final_shutdown_error;
    const bool final_shutdown = setInterlocks(false, &final_shutdown_error);
    const bool scan_completed = failure.empty();
    std::string return_detail;
    bool return_succeeded = false;
    bool in_progress_checkpoint = true;
    std::string in_progress_checkpoint_error;
    if (scan_completed && final_shutdown) {
      if (return_to_start_after_scan_) {
        active_plan_->return_status = "in_progress";
        in_progress_checkpoint = writeCheckpoint(
          *active_plan_, &in_progress_checkpoint_error);
        if (!in_progress_checkpoint) {
          publishStatus(
            "return is starting, but return_status=in_progress checkpoint failed: " +
            in_progress_checkpoint_error);
        }
        return_succeeded = executeReturnToStart(active_plan_.get(), elbow_sign, &return_detail);
        active_plan_->return_status = return_succeeded ? "completed" : "failed";
      } else {
        return_succeeded = true;
        return_detail = "automatic return-to-start disabled by parameter";
        active_plan_->return_status = "disabled";
      }
    } else if (!scan_completed) {
      active_plan_->return_status = "not_attempted_scan_failure";
    } else {
      active_plan_->return_status = "not_attempted_shutdown_failure";
    }
    std::string post_return_shutdown_error;
    const bool post_return_shutdown =
      final_shutdown && setInterlocks(false, &post_return_shutdown_error);
    std::string final_checkpoint_error;
    const bool final_checkpoint = writeCheckpoint(*active_plan_, &final_checkpoint_error);
    std::string final_marker_error;
    const bool final_marker = publishMarkers(*active_plan_, false, &final_marker_error);
    active_plan_->approved = false;
    response->success = scan_completed && final_shutdown && return_succeeded &&
      post_return_shutdown && final_checkpoint;
    if (!scan_completed) {
      response->message = "workspace scan stopped after " + std::to_string(completed_now) +
        " newly completed lanes: " + failure + "; call plan again to resume from checkpoint";
    } else if (!final_shutdown) {
      response->message = "workspace scan completed " + std::to_string(completed_now) +
        " lanes but return was not attempted because final laser/accumulation shutdown failed: " +
        final_shutdown_error + "; checkpoint=" + checkpoint_path_;
    } else if (!return_succeeded) {
      response->message = "workspace scan completed " + std::to_string(completed_now) +
        " lanes, but return-to-start failed with laser OFF: " + return_detail +
        "; checkpoint=" + checkpoint_path_;
    } else if (!post_return_shutdown) {
      response->message = "workspace scan and return motion completed, but final laser/accumulation "
        "OFF confirmation failed: " + post_return_shutdown_error +
        "; checkpoint=" + checkpoint_path_;
    } else {
      response->message = "workspace scan completed " + std::to_string(completed_now) +
        " lanes; " + return_detail + "; checkpoint=" + checkpoint_path_;
    }
    if (!final_checkpoint) {
      response->message += "; final return-status checkpoint failed: " + final_checkpoint_error;
    }
    if (!in_progress_checkpoint) {
      response->message += "; in-progress checkpoint warning: " + in_progress_checkpoint_error;
    }
    if (!final_marker) {
      response->message += "; non-fatal final RViz update failed: " + final_marker_error;
    }
    publishStatus(response->message);
  }

  void stopExecution(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    stop_requested_.store(true);
    if (trajectory_client_) {(void)trajectory_client_->async_cancel_all_goals();}
    if (move_group_) {move_group_->stop();}
    std::string error;
    const bool shutdown = setInterlocks(false, &error);
    response->success = shutdown;
    response->message = shutdown ?
      "stop/cancel requested; accumulation and laser confirmed OFF" :
      "stop/cancel requested but safe shutdown is unconfirmed: " + error;
    publishStatus(response->message);
  }

  void clearCheckpoint(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (execution_active_.load()) {
      response->success = false;
      response->message = "cannot clear checkpoint during execution";
      return;
    }
    std::lock_guard<std::mutex> lock(operation_mutex_);
    try {
      const bool removed = std::filesystem::remove(checkpoint_path_);
      if (active_plan_) {
        active_plan_->completed_lane_ids.clear();
        active_plan_->approved = false;
      }
      response->success = true;
      response->message = removed ? "workspace checkpoint removed" : "no workspace checkpoint existed";
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = "checkpoint removal failed: " + std::string(exception.what());
    }
  }

  std_msgs::msg::ColorRGBA laneColor(LaneState state) const
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 0.95F;
    switch (state) {
      case LaneState::Completed:
        color.r = 0.10F; color.g = 0.85F; color.b = 0.95F; break;
      case LaneState::Planned:
        color.r = 0.10F; color.g = 0.90F; color.b = 0.25F; break;
      case LaneState::PreliminaryRejected:
        color.r = 0.95F; color.g = 0.10F; color.b = 0.10F; break;
      case LaneState::KinematicRejected:
        color.r = 0.95F; color.g = 0.35F; color.b = 0.05F; break;
      case LaneState::PilzRejected:
        color.r = 0.80F; color.g = 0.10F; color.b = 0.85F; break;
      default:
        color.r = 0.35F; color.g = 0.35F; color.b = 0.35F; break;
    }
    return color;
  }

  bool publishMarkers(
    const StoredWorkspacePlan & plan, bool include_return_path,
    std::string * error = nullptr) noexcept
  {
    try {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.header.frame_id = base_frame_;
      clear.header.stamp = now();
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      for (const ws::ScanLane & lane : plan.geometry.lanes) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = now();
        marker.ns = "workspace_scan_lanes";
        marker.id = lane.id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.012;
        marker.color = laneColor(plan.reports[static_cast<std::size_t>(lane.id)].state);
        for (const ws::Pose & pose : lane.measurement_samples) {
          geometry_msgs::msg::Point point;
          point.x = pose.position.x;
          point.y = pose.position.y;
          point.z = pose.position.z;
          marker.points.push_back(point);
        }
        markers.markers.push_back(std::move(marker));
      }
      if (include_return_path) {
        for (std::size_t segment_index = 0U;
          segment_index < plan.return_segments.size(); ++segment_index)
        {
          const auto & stored_segment = plan.return_segments[segment_index];
          if (stored_segment.marker_points.empty()) {continue;}
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = base_frame_;
          marker.header.stamp = now();
          marker.ns = "workspace_return_path";
          marker.id = static_cast<int>(segment_index);
          marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 0.005;
          marker.color.r = 0.10F;
          marker.color.g = 0.45F;
          marker.color.b = 1.00F;
          marker.color.a = 0.65F;
          marker.points = stored_segment.marker_points;
          markers.markers.push_back(std::move(marker));
        }
      }
      visualization_msgs::msg::Marker summary;
      summary.header.frame_id = base_frame_;
      summary.header.stamp = now();
      summary.ns = "workspace_scan_summary";
      summary.id = 0;
      summary.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      summary.action = visualization_msgs::msg::Marker::ADD;
      summary.pose.position.x = 0.35;
      summary.pose.position.y = 0.0;
      summary.pose.position.z = plan.geometry.options.height_max_m + 0.20;
      summary.pose.orientation.w = 1.0;
      summary.scale.z = 0.055;
      summary.color.r = 1.0F;
      summary.color.g = 1.0F;
      summary.color.b = 1.0F;
      summary.color.a = 1.0F;
      std::ostringstream text;
      text << plan.id << "  reachable lanes " << plan.executable_lanes.size() << "/"
           << plan.geometry.lanes.size() << "  path coverage " << std::fixed
           << std::setprecision(1) << plan.coverage_ratio * 100.0 << "%  ETA "
           << plan.estimated_seconds / 60.0 << " min  return "
           << (include_return_path ?
        (plan.return_to_start_planned ? "validated direct PTP" : "disabled") :
        plan.return_status) << "\n"
           << (plan.geometry.options.tabletop_downward_scan ?
        "initial TCP +X centre " : "initial optical forward ")
           << plan.geometry.options.azimuth_center_deg
           << " deg, relative sector " << plan.geometry.options.azimuth_min_deg << ".."
           << plan.geometry.options.azimuth_max_deg << " deg, plane Z "
           << plan.geometry.options.height_min_m << " m"
           << (plan.geometry.options.tabletop_radial_fan_scan ?
        (plan.geometry.options.fan_origin_at_initial_tcp ?
        ", initial-TCP fan travel " : ", base radial snake ") +
        std::to_string(plan.geometry.options.radial_min_m) + ".." +
        std::to_string(plan.geometry.options.radial_max_m) + " m\n" : "\n")
           << (plan.environment_ready ?
        "collision scene validated" : "PREVIEW ONLY: collision scene unvalidated")
           << "  (surface observations, not free-volume proof)";
      summary.text = text.str();
      markers.markers.push_back(std::move(summary));
      marker_publisher_->publish(markers);
      return true;
    } catch (const std::exception & exception) {
      if (error) {*error = exception.what();}
      RCLCPP_ERROR(get_logger(), "non-fatal RViz marker failure: %s", exception.what());
    } catch (...) {
      if (error) {*error = "unknown exception";}
      RCLCPP_ERROR(get_logger(), "non-fatal RViz marker failure: unknown exception");
    }
    return false;
  }

  void publishStatus(const std::string & text_value) noexcept
  {
    try {
      std_msgs::msg::String message;
      message.data = text_value;
      status_publisher_->publish(message);
      RCLCPP_INFO(get_logger(), "%s", text_value.c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "non-fatal status publication failure: %s", exception.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "non-fatal status publication failure: unknown exception");
    }
  }

  ws::WorkspaceOptions options_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string base_frame_;
  std::string planning_pipeline_;
  std::string elbow_joint_name_;
  std::string accumulation_service_name_;
  std::string laser_service_name_;
  std::string controller_action_name_;
  std::string checkpoint_path_;
  std::string scan_surface_mode_{"tabletop_radial_fan"};
  double planning_time_s_{10.0};
  double scan_speed_m_s_{0.01};
  double transition_speed_m_s_{0.01};
  double pilz_max_translational_speed_m_s_{0.05};
  double acceleration_scaling_{0.30};
  double ik_timeout_s_{0.02};
  double maximum_angular_sample_step_deg_{5.0};
  double minimum_joint_margin_deg_{10.0};
  double minimum_normalized_sigma_{0.05};
  double jacobian_characteristic_length_m_{0.10};
  double j1_absolute_limit_deg_{85.0};
  double maximum_joint_step_deg_{20.0};
  double elbow_sign_deadband_deg_{2.0};
  double local_roi_padding_m_{0.225};
  double trajectory_start_tolerance_rad_{0.01};
  double trajectory_start_correction_limit_rad_{0.02};
  double joint_settle_timeout_s_{3.0};
  double joint_settle_sample_period_s_{0.10};
  double joint_settle_delta_rad_{0.001};
  double return_velocity_scaling_{0.10};
  double return_acceleration_scaling_{0.10};
  double return_joint_tolerance_rad_{0.002};
  double return_position_tolerance_m_{0.001};
  double return_orientation_tolerance_deg_{0.2};
  int joint_settle_required_samples_{3};
  bool use_initial_tcp_height_{true};
  bool use_current_tcp_radius_as_inner_{false};
  bool preserve_elbow_sign_{true};
  bool return_to_start_after_scan_{true};
  bool allow_execution_{false};
  bool collision_scene_validated_{false};
  bool require_environment_objects_{true};
  bool require_laser_control_{true};
  std::vector<double> local_roi_min_;
  std::vector<double> local_roi_max_;

  std::mutex operation_mutex_;
  std::atomic<bool> execution_active_{false};
  std::atomic<bool> stop_requested_{false};
  std::unique_ptr<MoveGroup> move_group_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<StoredWorkspacePlan> active_plan_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr accumulation_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr laser_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr local_rescan_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr approve_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_checkpoint_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  installCrashTraceHandlers();
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<WorkspaceCoarseScanPlanner>();
    node->initialize(node);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("workspace_coarse_scan_planner"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

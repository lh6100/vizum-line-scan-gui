#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <welding_interfaces/msg/robot_feedback.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace
{

class RobotFeedbackBridge final : public rclcpp::Node
{
public:
  RobotFeedbackBridge()
  : Node("robot_feedback_bridge"),
    maximum_age_(declare_parameter<double>("maximum_age_seconds", 0.15))
  {
    publisher_ = create_publisher<welding_interfaces::msg::RobotFeedback>(
      declare_parameter<std::string>("output_topic", "/welding_robot/robot_feedback"), rclcpp::QoS(20));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("pose_topic", "/fairino/flange_pose"), rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {on_pose(*pose);});
    joints_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("joint_state_topic", "/joint_states"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr joints) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joints_ = *joints;
        joints_received_at_ = now();
      });
    protective_subscription_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("protective_stop_topic", "/fairino/protective_stop"), 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr state) {protective_stop_ = state->data;});
    emergency_subscription_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("emergency_stop_topic", "/fairino/emergency_stop"), 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr state) {emergency_stop_ = state->data;});
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {publish();});
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time received = now();
    if (has_pose_) {
      const double dt = (received - pose_received_at_).seconds();
      if (dt > 1.0e-4) {
        const double dx = pose.pose.position.x - latest_pose_.pose.position.x;
        const double dy = pose.pose.position.y - latest_pose_.pose.position.y;
        const double dz = pose.pose.position.z - latest_pose_.pose.position.z;
        linear_speed_mm_s_ = std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0 / dt;
        const auto & a = pose.pose.orientation;
        const auto & b = latest_pose_.pose.orientation;
        const double dot = std::clamp(std::abs(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w), 0.0, 1.0);
        angular_speed_deg_s_ = 2.0 * std::acos(dot) * 180.0 / 3.14159265358979323846 / dt;
      }
    }
    latest_pose_ = pose;
    pose_received_at_ = received;
    has_pose_ = true;
  }

  void publish()
  {
    welding_interfaces::msg::RobotFeedback feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback.header.stamp = now();
      feedback.header.frame_id = latest_pose_.header.frame_id;
      feedback.sequence = ++sequence_;
      feedback.flange_pose = latest_pose_.pose;
      const std::size_t count = std::min<std::size_t>(6U, latest_joints_.position.size());
      for (std::size_t index = 0; index < count; ++index) {
        feedback.joints_deg[index] = latest_joints_.position[index] * 180.0 / 3.14159265358979323846;
      }
      feedback.actual_linear_speed_mm_s = linear_speed_mm_s_;
      feedback.actual_angular_speed_deg_s = angular_speed_deg_s_;
      const auto current = now();
      feedback.valid = has_pose_ && count == 6U &&
        (current - pose_received_at_).seconds() <= maximum_age_ &&
        (current - joints_received_at_).seconds() <= maximum_age_;
      feedback.protective_stop = protective_stop_;
      feedback.emergency_stop = emergency_stop_;
      feedback.enabled = feedback.valid && !protective_stop_ && !emergency_stop_;
      feedback.in_motion = linear_speed_mm_s_ > 0.5 || angular_speed_deg_s_ > 0.5;
      feedback.controller_state = feedback.valid ? (feedback.enabled ? "READY" : "STOPPED") : "STALE";
      if (!feedback.valid) {feedback.fault = "robot pose or joints are stale/incomplete";}
    }
    publisher_->publish(feedback);
  }

  double maximum_age_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  sensor_msgs::msg::JointState latest_joints_;
  rclcpp::Time pose_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time joints_received_at_{0, 0, RCL_ROS_TIME};
  bool has_pose_{false};
  bool protective_stop_{false};
  bool emergency_stop_{false};
  double linear_speed_mm_s_{0.0};
  double angular_speed_deg_s_{0.0};
  std::uint64_t sequence_{0};
  rclcpp::Publisher<welding_interfaces::msg::RobotFeedback>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr protective_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotFeedbackBridge>());
  rclcpp::shutdown();
  return 0;
}

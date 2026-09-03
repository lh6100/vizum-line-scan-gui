#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{

using namespace std::chrono_literals;
using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;

class TableCollisionSceneNode final : public rclcpp::Node
{
public:
  TableCollisionSceneNode()
  : Node("table_collision_scene")
  {
    object_id_ = declare_parameter<std::string>("object_id", "scanner_650_table");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    apply_service_ = declare_parameter<std::string>(
      "apply_planning_scene_service", "/apply_planning_scene");
    surface_z_m_ = declare_parameter<double>("surface_z_m", -0.100);
    center_x_m_ = declare_parameter<double>("center_x_m", 0.200);
    center_y_m_ = declare_parameter<double>("center_y_m", 0.200);
    size_x_m_ = declare_parameter<double>("size_x_m", 0.300);
    size_y_m_ = declare_parameter<double>("size_y_m", 0.200);
    thickness_m_ = declare_parameter<double>("thickness_m", 0.010);
    service_wait_timeout_s_ = declare_parameter<double>("service_wait_timeout_s", 30.0);

    if (object_id_.empty() || frame_id_.empty() || apply_service_.empty() ||
      !finite(surface_z_m_) || !finite(center_x_m_) || !finite(center_y_m_) ||
      !positiveFinite(size_x_m_) || !positiveFinite(size_y_m_) ||
      !positiveFinite(thickness_m_) || !positiveFinite(service_wait_timeout_s_))
    {
      throw std::runtime_error("invalid table collision-scene parameters");
    }

    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/scanner_650/table_collision_scene_status",
      rclcpp::QoS(1).reliable().transient_local());
    apply_client_ = create_client<ApplyPlanningScene>(apply_service_);
    wait_started_ = now();
    timer_ = create_wall_timer(250ms, std::bind(&TableCollisionSceneNode::tryApply, this));

    std::ostringstream status;
    status << "waiting to load table collision object '" << object_id_ << "' in " << frame_id_
           << ": surface_z=" << surface_z_m_ << " m, center_xy=(" << center_x_m_ << ", "
           << center_y_m_ << ") m, size_xyz=(" << size_x_m_ << ", " << size_y_m_ << ", "
           << thickness_m_ << ") m";
    publishStatus(status.str());
  }

private:
  static bool finite(double value)
  {
    return std::isfinite(value);
  }

  static bool positiveFinite(double value)
  {
    return finite(value) && value > 0.0;
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String status;
    status.data = text;
    status_publisher_->publish(status);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  moveit_msgs::msg::PlanningScene makeScene() const
  {
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = frame_id_;
    table.id = object_id_;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size_x_m_, size_y_m_, thickness_m_};

    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x_m_;
    pose.position.y = center_y_m_;
    // The supplied Z is the top surface. MoveIt boxes are positioned at their centre.
    pose.position.z = surface_z_m_ - 0.5 * thickness_m_;
    pose.orientation.w = 1.0;

    table.primitives.push_back(std::move(box));
    table.primitive_poses.push_back(pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.robot_state.is_diff = true;
    scene.world.collision_objects.push_back(std::move(table));
    return scene;
  }

  void tryApply()
  {
    if (applied_ || request_in_flight_) {
      return;
    }
    if (!apply_client_->service_is_ready()) {
      if ((now() - wait_started_).seconds() > service_wait_timeout_s_) {
        publishStatus(
          "table collision object was not loaded: apply_planning_scene service timeout");
        timer_->cancel();
      }
      return;
    }

    auto request = std::make_shared<ApplyPlanningScene::Request>();
    request->scene = makeScene();
    request_in_flight_ = true;
    apply_client_->async_send_request(
      request,
      [this](rclcpp::Client<ApplyPlanningScene>::SharedFuture future) {
        request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (!response || !response->success) {
            publishStatus("MoveIt rejected the table collision object; retrying");
            return;
          }
          applied_ = true;
          timer_->cancel();
          std::ostringstream status;
          status << "loaded table collision object '" << object_id_ << "': frame=" << frame_id_
                 << ", centre=(" << center_x_m_ << ", " << center_y_m_ << ", "
                 << surface_z_m_ - 0.5 * thickness_m_ << ") m, size=(" << size_x_m_ << ", "
                 << size_y_m_ << ", " << thickness_m_ << ") m, top_z=" << surface_z_m_
                 << " m; inspect RViz before declaring the scene validated";
          publishStatus(status.str());
        } catch (const std::exception & exception) {
          publishStatus(
            std::string("table collision-object request failed; retrying: ") + exception.what());
        }
      });
  }

  std::string object_id_;
  std::string frame_id_;
  std::string apply_service_;
  double surface_z_m_{-0.100};
  double center_x_m_{0.200};
  double center_y_m_{0.200};
  double size_x_m_{0.300};
  double size_y_m_{0.200};
  double thickness_m_{0.010};
  double service_wait_timeout_s_{30.0};
  bool request_in_flight_{false};
  bool applied_{false};
  rclcpp::Time wait_started_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TableCollisionSceneNode>());
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "table collision-scene startup failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

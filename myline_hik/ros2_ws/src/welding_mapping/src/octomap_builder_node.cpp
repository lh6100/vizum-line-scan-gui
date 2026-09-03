#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace
{

class OctomapBuilderNode final : public rclcpp::Node
{
public:
  OctomapBuilderNode()
  : Node("octomap_builder"),
    map_frame_(declare_parameter<std::string>("map_frame", "base_link")),
    minimum_range_(declare_parameter<double>("minimum_range_m", 0.25)),
    maximum_range_(declare_parameter<double>("maximum_range_m", 3.5)),
    self_filter_valid_(declare_parameter<bool>("self_filter_valid", false)),
    output_path_(declare_parameter<std::string>("output_bt_path", "")),
    tree_(declare_parameter<double>("resolution_m", 0.02)),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    tree_.setProbHit(declare_parameter<double>("probability_hit", 0.70));
    tree_.setProbMiss(declare_parameter<double>("probability_miss", 0.40));
    preview_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/welding_robot/map/octomap_preview", rclcpp::QoS(1).reliable().transient_local());
    map_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/welding_robot/map/octomap", rclcpp::QoS(1).reliable().transient_local());
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("input_cloud_topic", "/welding_robot/stereo/points"),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {integrate(*cloud);});
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "/welding_robot/map/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        tree_.clear();
        update_count_ = 0;
      });
    const double period = std::max(0.1, declare_parameter<double>("publish_period_seconds", 1.0));
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {publish();});
    if (!self_filter_valid_) {
      RCLCPP_WARN(get_logger(),
        "self_filter_valid=false: map is preview-only and cannot enter MoveIt planning scene");
    }
  }

private:
  void integrate(const sensor_msgs::msg::PointCloud2 & message)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(map_frame_, message.header.frame_id, message.header.stamp, 100ms);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "point cloud transform unavailable: %s", exception.what());
      return;
    }
    tf2::Transform map_from_sensor;
    tf2::fromMsg(transform.transform, map_from_sensor);
    const tf2::Vector3 origin_tf = map_from_sensor.getOrigin();
    const octomap::point3d origin(origin_tf.x(), origin_tf.y(), origin_tf.z());
    octomap::Pointcloud cloud;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
        const tf2::Vector3 source(*x, *y, *z);
        const double range = source.length();
        if (range < minimum_range_ || range > maximum_range_) {continue;}
        const tf2::Vector3 point = map_from_sensor * source;
        cloud.push_back(point.x(), point.y(), point.z());
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR(get_logger(), "invalid PointCloud2 layout: %s", exception.what());
      return;
    }
    if (cloud.size() == 0U) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    tree_.insertPointCloud(cloud, origin, maximum_range_, true, true);
    tree_.updateInnerOccupancy();
    ++update_count_;
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tree_.size() <= 1U || update_count_ == 0U) {return;}
    octomap_msgs::msg::Octomap message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    if (!octomap_msgs::binaryMapToMsg(tree_, message)) {
      RCLCPP_ERROR(get_logger(), "failed to serialize OctoMap");
      return;
    }
    preview_publisher_->publish(message);
    if (self_filter_valid_) {map_publisher_->publish(message);}
    if (!output_path_.empty() && !tree_.writeBinary(output_path_)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "cannot save OctoMap to %s", output_path_.c_str());
    }
  }

  std::string map_frame_;
  double minimum_range_;
  double maximum_range_;
  bool self_filter_valid_;
  std::string output_path_;
  octomap::OcTree tree_;
  std::uint64_t update_count_{0};
  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr preview_publisher_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr map_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

bool scale_coordinates(pcl::PCLPointCloud2 * cloud, double scale)
{
  for (const std::string name : {"x", "y", "z"}) {
    const auto field = std::find_if(cloud->fields.begin(), cloud->fields.end(),
      [&name](const pcl::PCLPointField & value) {return value.name == name;});
    if (field == cloud->fields.end() ||
      (field->datatype != pcl::PCLPointField::FLOAT32 && field->datatype != pcl::PCLPointField::FLOAT64))
    {return false;}
    const std::size_t points = static_cast<std::size_t>(cloud->width) * cloud->height;
    for (std::size_t index = 0; index < points; ++index) {
      unsigned char * bytes = cloud->data.data() + index * cloud->point_step + field->offset;
      if (field->datatype == pcl::PCLPointField::FLOAT32) {
        float value;
        std::memcpy(&value, bytes, sizeof(value));
        value = static_cast<float>(value * scale);
        std::memcpy(bytes, &value, sizeof(value));
      } else {
        double value;
        std::memcpy(&value, bytes, sizeof(value));
        value *= scale;
        std::memcpy(bytes, &value, sizeof(value));
      }
    }
  }
  return true;
}

class PointCloudFilePublisher final : public rclcpp::Node
{
public:
  PointCloudFilePublisher()
  : Node("pointcloud_file_publisher")
  {
    const std::filesystem::path path = declare_parameter<std::string>("file", "");
    const std::string frame = declare_parameter<std::string>("frame_id", "base_link");
    const double scale = declare_parameter<double>("scale_to_meters", 1.0);
    if (!std::filesystem::is_regular_file(path) || !std::isfinite(scale) || scale <= 0.0) {
      throw std::runtime_error("file must be a readable PCD/PLY and scale_to_meters must be positive");
    }
    pcl::PCLPointCloud2 cloud;
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    const int loaded = extension == ".pcd" ? pcl::io::loadPCDFile(path.string(), cloud) :
      extension == ".ply" ? pcl::io::loadPLYFile(path.string(), cloud) : -1;
    if (loaded < 0 || cloud.data.empty() || !scale_coordinates(&cloud, scale)) {
      throw std::runtime_error("cannot load a non-empty PCD/PLY with numeric x/y/z fields");
    }
    pcl_conversions::fromPCL(cloud, message_);
    message_.header.frame_id = frame;
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("topic", "/welding_robot/offline_cloud"),
      rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      message_.header.stamp = now();
      publisher_->publish(message_);
    });
    RCLCPP_INFO(get_logger(), "loaded %u points from %s", cloud.width * cloud.height, path.c_str());
  }

private:
  sensor_msgs::msg::PointCloud2 message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<PointCloudFilePublisher>());} catch (const std::exception & exception) {
    std::fprintf(stderr, "pointcloud_file_publisher failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

#include "stereo/app/StereoCameraRig.h"

#include <QCoreApplication>
#include <QObject>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

namespace
{

class HikStereoCameraNode final : public rclcpp::Node
{
public:
  HikStereoCameraNode()
  : Node("hik_stereo_camera")
  {
    left_ip_ = declare_parameter<std::string>("left_ip", "");
    right_ip_ = declare_parameter<std::string>("right_ip", "");
    left_serial_ = declare_parameter<std::string>("left_expected_serial", "");
    right_serial_ = declare_parameter<std::string>("right_expected_serial", "");
    left_frame_ = declare_parameter<std::string>("left_frame_id", "camera_650_optical_frame");
    right_frame_ = declare_parameter<std::string>("right_frame_id", "camera_450_optical_frame");
    const double shared_exposure_us =
      declare_parameter<double>("exposure_us", -1.0);
    left_exposure_us_ =
      declare_parameter<double>("left_exposure_us", 18000.0);
    right_exposure_us_ =
      declare_parameter<double>("right_exposure_us", 18000.0);
    // A positive shared value is an explicit compatibility override.  Leaving it at
    // -1 keeps the independently tuned left/right values supplied by the launch file.
    if (shared_exposure_us > 0.0) {
      left_exposure_us_ = shared_exposure_us;
      right_exposure_us_ = shared_exposure_us;
    }
    gain_db_ = declare_parameter<double>("gain_db", 0.0);
    fps_ = declare_parameter<double>("frames_per_second", 30.0);
    maximum_skew_ms_ = declare_parameter<double>("maximum_pair_skew_ms", 5.0);
    if (left_ip_.empty() || right_ip_.empty() || left_ip_ == right_ip_ ||
      left_serial_.empty() || right_serial_.empty())
    {throw std::runtime_error("two distinct camera IPs and expected serials are required");}

    left_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("left_topic", "/welding_robot/camera_650/image_raw"),
      rclcpp::SensorDataQoS());
    right_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("right_topic", "/welding_robot/camera_450/image_raw"),
      rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/welding_robot/cameras/status", rclcpp::QoS(10));

    rig_ = std::make_unique<hik_stereo::StereoCameraRig>();
    QObject::connect(rig_.get(), &hik_stereo::StereoCameraRig::identityChanged,
      [this](hik_stereo::StereoCameraSide side, const QString &, const QString & serial, const QString &) {
        const std::string actual = serial.toStdString();
        const std::string expected = side == hik_stereo::StereoCameraSide::Left ? left_serial_ : right_serial_;
        const bool matches = actual == expected;
        if (side == hik_stereo::StereoCameraSide::Left) {left_identity_valid_ = matches;}
        else {right_identity_valid_ = matches;}
        if (!matches) {
          RCLCPP_FATAL(get_logger(), "camera serial mismatch: expected %s, received %s",
            expected.c_str(), actual.c_str());
          rig_->disconnectCameras();
        }
      });
    QObject::connect(rig_.get(), &hik_stereo::StereoCameraRig::connectionChanged,
      [this](hik_stereo::StereoCameraSide side, bool connected, const QString & detail) {
        if (side == hik_stereo::StereoCameraSide::Left) {left_connected_ = connected;}
        else {right_connected_ = connected;}
        RCLCPP_INFO(get_logger(), "%s", detail.toUtf8().constData());
        start_if_ready();
      });
    QObject::connect(rig_.get(), &hik_stereo::StereoCameraRig::pairReady,
      [this](hik_stereo::StereoFramePair pair) {publish_pair(std::move(pair));});
    QObject::connect(rig_.get(), &hik_stereo::StereoCameraRig::error,
      [this](const QString & message) {RCLCPP_ERROR(get_logger(), "%s", message.toUtf8().constData());});
    QObject::connect(rig_.get(), &hik_stereo::StereoCameraRig::log,
      [this](const QString & message) {RCLCPP_INFO(get_logger(), "%s", message.toUtf8().constData());});
    QTimer::singleShot(0, [this]() {
      rig_->connectCameras(QString::fromStdString(left_ip_), QString::fromStdString(right_ip_));
    });
  }

  ~HikStereoCameraNode() override
  {
    if (rig_) {rig_->stop(); rig_->disconnectCameras(); rig_.reset();}
  }

private:
  void start_if_ready()
  {
    if (!started_ && left_connected_ && right_connected_ && left_identity_valid_ && right_identity_valid_) {
      started_ = true;
      rig_->start(
        left_exposure_us_, right_exposure_us_, gain_db_, fps_, maximum_skew_ms_);
      RCLCPP_INFO(
        get_logger(),
        "starting bounded-skew software-paired stereo stream at %.1f fps; "
        "left exposure %.3f us; right exposure %.3f us",
        fps_, left_exposure_us_, right_exposure_us_);
    }
  }

  sensor_msgs::msg::Image make_image(
    const hik_sync::CameraFrame & frame, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id;
    image.height = static_cast<std::uint32_t>(frame.height);
    image.width = static_cast<std::uint32_t>(frame.width);
    image.encoding = "mono8";
    image.is_bigendian = false;
    image.step = static_cast<std::uint32_t>(frame.width);
    if (frame.image && frame.image->bytes.size() >= image.step * image.height) {
      image.data.assign(frame.image->bytes.begin(),
        frame.image->bytes.begin() + static_cast<std::ptrdiff_t>(image.step * image.height));
    }
    return image;
  }

  void publish_pair(hik_stereo::StereoFramePair pair)
  {
    const builtin_interfaces::msg::Time stamp = now();
    auto left = make_image(pair.left, left_frame_, stamp);
    auto right = make_image(pair.right, right_frame_, stamp);
    if (left.data.empty() || right.data.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "paired camera buffer is empty");
      return;
    }
    left_publisher_->publish(left);
    right_publisher_->publish(right);
    std_msgs::msg::String status;
    status.data = "paired; skew_ms=" + std::to_string(pair.skewMs) +
      "; left_frame=" + std::to_string(pair.left.frameId) +
      "; right_frame=" + std::to_string(pair.right.frameId);
    status_publisher_->publish(status);
  }

  std::string left_ip_, right_ip_, left_serial_, right_serial_, left_frame_, right_frame_;
  double left_exposure_us_{18000.0}, right_exposure_us_{18000.0};
  double gain_db_{0.0}, fps_{30.0}, maximum_skew_ms_{5.0};
  bool left_connected_{false}, right_connected_{false};
  bool left_identity_valid_{false}, right_identity_valid_{false}, started_{false};
  std::unique_ptr<hik_stereo::StereoCameraRig> rig_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  QCoreApplication application(argc, argv);
  rclcpp::init(argc, argv);
  std::shared_ptr<HikStereoCameraNode> node;
  try {node = std::make_shared<HikStereoCameraNode>();} catch (const std::exception & exception) {
    std::fprintf(stderr, "hik_stereo_camera startup failed: %s\n", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() {executor.spin();});
  QObject::connect(&application, &QCoreApplication::aboutToQuit, [&executor]() {executor.cancel();});
  QTimer shutdown_monitor;
  QObject::connect(&shutdown_monitor, &QTimer::timeout, [&application]() {
    if (!rclcpp::ok()) {application.quit();}
  });
  shutdown_monitor.start(100);
  const int result = application.exec();
  executor.cancel();
  ros_thread.join();
  node.reset();
  rclcpp::shutdown();
  return result;
}

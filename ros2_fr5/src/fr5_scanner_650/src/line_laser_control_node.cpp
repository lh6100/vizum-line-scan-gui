#include "LineLaserController.h"

#include <QCoreApplication>
#include <QObject>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

const char * connection_state_name(LineLaserConnectionState state)
{
  switch (state) {
    case LineLaserConnectionState::Disconnected: return "disconnected";
    case LineLaserConnectionState::Connecting: return "connecting";
    case LineLaserConnectionState::Ready: return "ready";
    case LineLaserConnectionState::CommandPending: return "command_pending";
    case LineLaserConnectionState::Disconnecting: return "disconnecting";
    case LineLaserConnectionState::Fault: return "fault";
  }
  return "unknown";
}

class LineLaserControlNode final : public rclcpp::Node
{
public:
  LineLaserControlNode()
  : Node("line_laser_control")
  {
    LineLaserConnectionConfig config = LineLaserConnectionConfig::defaults();
    config.host = QString::fromStdString(declare_parameter<std::string>("host", "192.168.1.12"));
    config.port = static_cast<quint16>(declare_parameter<int>("port", 22));
    config.user = QString::fromStdString(declare_parameter<std::string>("user", "laserctl"));
    config.privateKeyPath = QString::fromStdString(
      declare_parameter<std::string>("private_key_path", config.privateKeyPath.toStdString()));
    config.knownHostsPath = QString::fromStdString(
      declare_parameter<std::string>("known_hosts_path", config.knownHostsPath.toStdString()));
    config.autoReconnect = true;
    const bool auto_connect = declare_parameter<bool>("auto_connect", true);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 3.0);
    connection_timeout_s_ = declare_parameter<double>("connection_timeout_s", 8.0);
    const std::string service_name = declare_parameter<std::string>(
      "service_name", "/scanner_650/set_laser");
    const std::string connection_service_name = declare_parameter<std::string>(
      "connection_service_name", "/scanner_650/set_laser_connected");
    if (config.port == 0 || !std::isfinite(command_timeout_s_) ||
      command_timeout_s_ < 0.5 || command_timeout_s_ > 10.0 ||
      !std::isfinite(connection_timeout_s_) || connection_timeout_s_ < 1.0 ||
      connection_timeout_s_ > 30.0 || service_name.empty() ||
      connection_service_name.empty())
    {
      throw std::runtime_error("invalid scanner_650 laser-control parameters");
    }

    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/scanner_650/laser_status", rclcpp::QoS(10).reliable().transient_local());
    controller_ = std::make_unique<LineLaserController>(config);
    QObject::connect(
      controller_.get(), &LineLaserController::connectionStateChanged,
      [this](LineLaserConnectionState state, const QString & detail) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          connection_state_ = state;
          last_detail_ = detail.toStdString();
        }
        condition_.notify_all();
        publish_status(
          "connection=" + std::string(connection_state_name(state)) +
          "; detail=" + detail.toStdString());
      });
    QObject::connect(
      controller_.get(), &LineLaserController::statusChanged,
      [this](LineLaserStatus status) {
        std::string connection;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          status_ = status;
          connection = connection_state_name(connection_state_);
        }
        condition_.notify_all();
        publish_status(
          "connection=" + connection +
          "; reachable=" + std::to_string(status.reachable) +
          "; ttl450=" + std::to_string(status.ttl450High) +
          "; ttl650=" + std::to_string(status.ttl650High) +
          "; lease=" + std::to_string(status.leaseActive));
      });
    QObject::connect(
      controller_.get(), &LineLaserController::commandFinished,
      [this](const QString & command, bool success, const QString & detail) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_command_ = command.toStdString();
          last_command_success_ = success;
          last_detail_ = detail.toStdString();
          ++command_generation_;
        }
        condition_.notify_all();
      });
    QObject::connect(
      controller_.get(), &LineLaserController::faultOccurred,
      [this](const QString & detail) {
        RCLCPP_ERROR(get_logger(), "laser controller: %s", detail.toUtf8().constData());
      });
    QObject::connect(
      controller_.get(), &LineLaserController::logMessage,
      [this](const QString & detail) {
        RCLCPP_INFO(get_logger(), "laser controller: %s", detail.toUtf8().constData());
      });

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_ = create_service<std_srvs::srv::SetBool>(
      service_name,
      std::bind(
        &LineLaserControlNode::set_laser, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    connection_service_ = create_service<std_srvs::srv::SetBool>(
      connection_service_name,
      std::bind(
        &LineLaserControlNode::set_connection, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, callback_group_);
    publish_status("connection=disconnected; detail=laser controller node ready");
    if (auto_connect) {
      QTimer::singleShot(0, [this]() {controller_->connectController();});
    }
  }

  ~LineLaserControlNode() override
  {
    if (controller_) {
      controller_->off();
      controller_->disconnectController();
      controller_.reset();
    }
  }

private:
  void publish_status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_publisher_->publish(message);
  }

  void set_laser(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    response->success = command_laser(request->data, &response->message);
  }

  bool command_laser(bool enabled, std::string * message)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (connection_state_ != LineLaserConnectionState::Ready ||
      !status_.reachable || !status_.leaseActive)
    {
      *message = "laser controller is not ready: " + last_detail_;
      return false;
    }
    const std::uint64_t initial_generation = command_generation_;
    const quint64 off_token = enabled ? 0U : controller_->requestOffTracked();
    if (enabled) {
      controller_->set650();
    }
    const bool completed = condition_.wait_for(
      lock, std::chrono::duration<double>(command_timeout_s_),
      [this, enabled, initial_generation, off_token]() {
        if (command_generation_ <= initial_generation || !last_command_success_) {
          return false;
        }
        if (enabled) {
          return last_command_ == "set650" && status_.reachable && status_.leaseActive &&
                 status_.state == LineLaserState::Laser650 && !status_.ttl450High &&
                 status_.ttl650High;
        }
        return last_command_ == "off" && status_.reachable && status_.leaseActive &&
               status_.state == LineLaserState::Off && !status_.ttl450High &&
               !status_.ttl650High && status_.acknowledgedOffCommandToken >= off_token;
      });
    *message = completed ?
      (enabled ? "650 nm laser confirmed ON; 450 nm confirmed OFF" :
      "both laser TTL outputs confirmed OFF") :
      "laser command was not confirmed before timeout: " + last_detail_;
    return completed;
  }

  void set_connection(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request->data && connection_state_ == LineLaserConnectionState::Ready &&
        status_.reachable && status_.leaseActive)
      {
        response->success = true;
        response->message = "laser controller is already connected and ready";
        return;
      }
      if (!request->data && connection_state_ == LineLaserConnectionState::Disconnected) {
        response->success = true;
        response->message = "laser controller is already disconnected";
        return;
      }
      if (!request->data && connection_state_ != LineLaserConnectionState::Ready) {
        response->success = false;
        response->message =
          "laser controller is not ready; cannot confirm both TTL outputs OFF before disconnect: " +
          last_detail_;
        return;
      }
    }

    if (!request->data) {
      std::string off_message;
      if (!command_laser(false, &off_message)) {
        response->success = false;
        response->message = "disconnect rejected because OFF was not confirmed: " + off_message;
        return;
      }
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (request->data) {
      controller_->connectController();
      const bool ready = condition_.wait_for(
        lock, std::chrono::duration<double>(connection_timeout_s_), [this]() {
          return connection_state_ == LineLaserConnectionState::Ready &&
                 status_.reachable && status_.leaseActive;
        });
      response->success = ready;
      response->message = ready ?
        "laser controller connected; board lease and GPIO status confirmed" :
        "laser controller connection was not confirmed: " + last_detail_;
      return;
    }

    controller_->disconnectController();
    const bool disconnected = condition_.wait_for(
      lock, std::chrono::duration<double>(connection_timeout_s_), [this]() {
        return connection_state_ == LineLaserConnectionState::Disconnected;
      });
    response->success = disconnected;
    response->message = disconnected ?
      "both laser TTL outputs confirmed OFF; controller disconnected and lease released" :
      "laser controller disconnect was not confirmed: " + last_detail_;
  }

  double command_timeout_s_{3.0};
  double connection_timeout_s_{8.0};
  std::mutex mutex_;
  std::condition_variable condition_;
  LineLaserConnectionState connection_state_{LineLaserConnectionState::Disconnected};
  LineLaserStatus status_;
  std::string last_command_;
  std::string last_detail_;
  bool last_command_success_{false};
  std::uint64_t command_generation_{0};
  std::unique_ptr<LineLaserController> controller_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr connection_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

}  // namespace

int main(int argc, char ** argv)
{
  QCoreApplication application(argc, argv);
  rclcpp::init(argc, argv);
  std::shared_ptr<LineLaserControlNode> node;
  try {
    node = std::make_shared<LineLaserControlNode>();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "line_laser_control startup failed: %s\n", exception.what());
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

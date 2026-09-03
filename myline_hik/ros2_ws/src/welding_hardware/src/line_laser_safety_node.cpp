#include "LineLaserController.h"

#include <QCoreApplication>
#include <QObject>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <welding_interfaces/msg/laser_safety_state.hpp>
#include <welding_interfaces/srv/guarded_laser_mode.hpp>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class LineLaserSafetyNode final : public rclcpp::Node
{
public:
  LineLaserSafetyNode()
  : Node("line_laser_safety")
  {
    LineLaserConnectionConfig config = LineLaserConnectionConfig::defaults();
    config.host = QString::fromStdString(declare_parameter<std::string>("host", config.host.toStdString()));
    config.port = static_cast<quint16>(declare_parameter<int>("port", config.port));
    config.user = QString::fromStdString(declare_parameter<std::string>("user", config.user.toStdString()));
    const auto private_key = declare_parameter<std::string>("private_key_path", "");
    const auto known_hosts = declare_parameter<std::string>("known_hosts_path", "");
    if (!private_key.empty()) {config.privateKeyPath = QString::fromStdString(private_key);}
    if (!known_hosts.empty()) {config.knownHostsPath = QString::fromStdString(known_hosts);}
    config.commandTimeoutMs = declare_parameter<int>("command_timeout_ms", 1200);
    maximum_lease_ms_ = declare_parameter<int>("maximum_local_lease_ms", 5000);
    command_timeout_ms_ = config.commandTimeoutMs;
    controller_ = std::make_unique<LineLaserController>(config);
    publisher_ = create_publisher<welding_interfaces::msg::LaserSafetyState>(
      "/welding_robot/laser/state", rclcpp::QoS(1).reliable().transient_local());
    QObject::connect(controller_.get(), &LineLaserController::statusChanged,
      [this](LineLaserStatus status) {update_status(status);});
    QObject::connect(controller_.get(), &LineLaserController::faultOccurred,
      [this](const QString & fault) {
        std::lock_guard<std::mutex> lock(mutex_);
        fault_ = fault.toStdString();
      });
    service_ = create_service<welding_interfaces::srv::GuardedLaserMode>(
      "/welding_robot/laser/set_mode",
      [this](
        const std::shared_ptr<welding_interfaces::srv::GuardedLaserMode::Request> request,
        std::shared_ptr<welding_interfaces::srv::GuardedLaserMode::Response> response)
      {handle_request(*request, response.get());});
    watchdog_ = create_wall_timer(50ms, [this]() {watchdog();});
    controller_->connectController();
  }

  ~LineLaserSafetyNode() override
  {
    if (controller_) {
      const auto token = controller_->requestOffTracked();
      (void)token;
      controller_->disconnectController();
      controller_.reset();
    }
  }

private:
  static bool valid_id(const std::string & value)
  {
    return !value.empty() && value.size() <= 128U && value.find_first_not_of(
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") == std::string::npos;
  }

  std::uint8_t mode_from_status(LineLaserState state) const
  {
    switch (state) {
      case LineLaserState::Off: return welding_interfaces::msg::LaserSafetyState::OFF;
      case LineLaserState::Laser450: return welding_interfaces::msg::LaserSafetyState::LASER_450;
      case LineLaserState::Laser650: return welding_interfaces::msg::LaserSafetyState::LASER_650;
      default: return welding_interfaces::msg::LaserSafetyState::UNKNOWN;
    }
  }

  welding_interfaces::msg::LaserSafetyState current_message_locked() const
  {
    welding_interfaces::msg::LaserSafetyState message;
    message.header.stamp = now();
    message.mode = mode_;
    message.reachable = reachable_;
    message.readback_valid = readback_valid_;
    message.watchdog_healthy = reachable_ && fault_.empty();
    message.lease_active = !lease_id_.empty() && std::chrono::steady_clock::now() < lease_expires_;
    message.lease_id = lease_id_;
    message.lease_remaining_ms = message.lease_active ? static_cast<std::int32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(lease_expires_ - std::chrono::steady_clock::now()).count()) : 0;
    message.fault = fault_;
    return message;
  }

  void update_status(const LineLaserStatus & status)
  {
    welding_interfaces::msg::LaserSafetyState message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reachable_ = status.reachable;
      mode_ = mode_from_status(status.state);
      readback_valid_ = status.state != LineLaserState::Unknown && status.state != LineLaserState::Both;
      status_sequence_ = status.eventSequence;
      acknowledged_off_token_ = status.acknowledgedOffCommandToken;
      fault_ = status.fault.toStdString();
      if (mode_ == welding_interfaces::msg::LaserSafetyState::OFF) {lease_id_.clear();}
      message = current_message_locked();
    }
    publisher_->publish(message);
    condition_.notify_all();
  }

  void handle_request(
    const welding_interfaces::srv::GuardedLaserMode::Request & request,
    welding_interfaces::srv::GuardedLaserMode::Response * response)
  {
    const bool off = request.requested_mode == welding_interfaces::srv::GuardedLaserMode::Request::OFF;
    if (!off && (request.requested_mode != welding_interfaces::srv::GuardedLaserMode::Request::LASER_450 &&
      request.requested_mode != welding_interfaces::srv::GuardedLaserMode::Request::LASER_650))
    {response->error = "unsupported laser mode; simultaneous lasers are forbidden"; return;}
    if (!off && (!valid_id(request.lease_id) || request.requested_lease_ms < 100 ||
      request.requested_lease_ms > maximum_lease_ms_))
    {response->error = "laser-on requires a stable lease_id and bounded lease duration"; return;}

    std::unique_lock<std::mutex> lock(mutex_);
    const std::uint64_t initial_sequence = status_sequence_;
    std::uint64_t off_token = 0;
    if (off) {
      off_token = controller_->requestOffTracked();
    } else if (request.requested_mode == welding_interfaces::srv::GuardedLaserMode::Request::LASER_450) {
      controller_->set450();
    } else {
      controller_->set650();
    }
    const bool confirmed = condition_.wait_for(
      lock, std::chrono::milliseconds(command_timeout_ms_ + 500), [&]() {
        if (!reachable_ || !readback_valid_) {return false;}
        if (off) {return mode_ == welding_interfaces::msg::LaserSafetyState::OFF && acknowledged_off_token_ >= off_token;}
        return status_sequence_ > initial_sequence && mode_ == request.requested_mode;
      });
    if (!confirmed) {
      controller_->requestOffTracked();
      lease_id_.clear();
      response->error = "laser command was not confirmed by fresh GPIO readback; forced OFF";
    } else {
      response->success = true;
      if (!off) {
        lease_id_ = request.lease_id;
        lease_expires_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(request.requested_lease_ms);
      } else {
        lease_id_.clear();
      }
    }
    response->state = current_message_locked();
  }

  void watchdog()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!lease_id_.empty() && std::chrono::steady_clock::now() >= lease_expires_) {
      controller_->requestOffTracked();
      lease_id_.clear();
      fault_ = "local laser lease expired; forced OFF";
    }
  }

  std::unique_ptr<LineLaserController> controller_;
  int maximum_lease_ms_{5000};
  int command_timeout_ms_{1200};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool reachable_{false};
  bool readback_valid_{false};
  std::uint8_t mode_{welding_interfaces::msg::LaserSafetyState::UNKNOWN};
  std::uint64_t status_sequence_{0};
  std::uint64_t acknowledged_off_token_{0};
  std::string lease_id_;
  std::string fault_;
  std::chrono::steady_clock::time_point lease_expires_{};
  rclcpp::Publisher<welding_interfaces::msg::LaserSafetyState>::SharedPtr publisher_;
  rclcpp::Service<welding_interfaces::srv::GuardedLaserMode>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace

int main(int argc, char ** argv)
{
  QCoreApplication application(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineLaserSafetyNode>();
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

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace
{

using namespace std::chrono_literals;

class RvizScanTrajectoryRelay final : public rclcpp::Node
{
public:
  using Follow = control_msgs::action::FollowJointTrajectory;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Follow>;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Follow>;

  RvizScanTrajectoryRelay()
  : Node("rviz_scan_trajectory_relay")
  {
    allow_execution_ = declare_parameter<bool>("allow_execution", false);
    scan_on_execute_ = declare_parameter<bool>("scan_on_execute", false);
    require_laser_control_ = declare_parameter<bool>("require_laser_control", true);
    proxy_action_name_ = declare_parameter<std::string>(
      "proxy_action", "/scanner_650_controller/follow_joint_trajectory");
    controller_action_name_ = declare_parameter<std::string>(
      "controller_action", "/fairino5_controller/follow_joint_trajectory");
    accumulation_service_name_ = declare_parameter<std::string>(
      "accumulation_service", "/scanner_650/set_accumulation");
    laser_service_name_ = declare_parameter<std::string>(
      "laser_service", "/scanner_650/set_laser");
    scan_mode_service_name_ = declare_parameter<std::string>(
      "scan_mode_service", "/scanner_650/set_rviz_scan_mode");
    if (proxy_action_name_.empty() || controller_action_name_.empty() ||
      accumulation_service_name_.empty() || scan_mode_service_name_.empty() ||
      (require_laser_control_ && laser_service_name_.empty()))
    {
      throw std::runtime_error("empty RViz scan relay parameter");
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    accumulation_client_ = create_client<std_srvs::srv::SetBool>(
      accumulation_service_name_, rmw_qos_profile_services_default, callback_group_);
    laser_client_ = create_client<std_srvs::srv::SetBool>(
      laser_service_name_, rmw_qos_profile_services_default, callback_group_);
    controller_client_ = rclcpp_action::create_client<Follow>(
      this, controller_action_name_, callback_group_);
    scan_mode_service_ = create_service<std_srvs::srv::SetBool>(
      scan_mode_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        set_scan_mode(request->data, response);
      },
      rmw_qos_profile_services_default, callback_group_);
    action_server_ = rclcpp_action::create_server<Follow>(
      this, proxy_action_name_,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
        return handle_goal(goal);
      },
      [this](const std::shared_ptr<ServerGoalHandle> &) {
        cancel_requested_.store(true);
        if (controller_client_) {
          (void)controller_client_->async_cancel_all_goals();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ServerGoalHandle> goal) {
        std::thread([this, goal]() {execute(goal);}).detach();
      },
      rcl_action_server_get_default_options(), callback_group_);

    RCLCPP_WARN(
      get_logger(), "RViz trajectory relay: execution=%s mode=%s proxy=%s real=%s",
      allow_execution_ ? "enabled" : "locked",
      scan_on_execute_.load() ? "SCAN (laser + accumulation)" : "POSITION (laser OFF)",
      proxy_action_name_.c_str(), controller_action_name_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(const std::shared_ptr<const Follow::Goal> & goal)
  {
    if (!allow_execution_) {
      RCLCPP_ERROR(get_logger(), "RViz trajectory rejected: execution gate is locked");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal || goal->trajectory.joint_names.empty() || goal->trajectory.points.empty()) {
      RCLCPP_ERROR(get_logger(), "RViz trajectory rejected: trajectory is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> mode_lock(mode_mutex_);
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_ERROR(get_logger(), "RViz trajectory rejected: another trajectory is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void set_scan_mode(
    bool enabled, const std::shared_ptr<std_srvs::srv::SetBool::Response> & response)
  {
    std::lock_guard<std::mutex> mode_lock(mode_mutex_);
    if (goal_active_.load()) {
      response->success = false;
      response->message = "cannot change RViz scan mode while a trajectory is active";
      return;
    }
    if (!enabled) {
      std::string warning;
      finish_interlocks(&warning);
      scan_on_execute_.store(false);
      if (!warning.empty()) {
        response->success = false;
        response->message = "POSITION selected but safe OFF state was not confirmed: " + warning;
        return;
      }
    } else {
      scan_on_execute_.store(true);
    }
    response->success = true;
    response->message = enabled ?
      "RViz Execute mode is SCAN: the complete planned trajectory will be scanned" :
      "RViz Execute mode is POSITION: laser and accumulation confirmed OFF";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  bool request_boolean_service(
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
    const auto response = future.get();
    if (!response->success && error) {*error = response->message;}
    return response->success;
  }

  bool request_accumulation(bool enabled, std::string * error)
  {
    return request_boolean_service(
      accumulation_client_, enabled, "reconstruction accumulation", error);
  }

  bool request_laser(bool enabled, std::string * error)
  {
    if (!require_laser_control_) {return true;}
    return request_boolean_service(laser_client_, enabled, "650 nm laser", error);
  }

  void finish_interlocks(std::string * warning)
  {
    std::string accumulation_error;
    const bool accumulation_off = request_accumulation(false, &accumulation_error);
    std::string laser_error;
    const bool laser_off = request_laser(false, &laser_error);
    if (warning && (!accumulation_off || !laser_off)) {
      *warning = "shutdown incomplete: " + accumulation_error + " " + laser_error;
    }
  }

  void terminate(
    const std::shared_ptr<ServerGoalHandle> & goal, bool canceled,
    int error_code, const std::string & message)
  {
    auto result = std::make_shared<Follow::Result>();
    result->error_code = error_code;
    result->error_string = message;
    if (canceled) {
      goal->canceled(result);
    } else {
      goal->abort(result);
    }
    RCLCPP_ERROR(get_logger(), "RViz trajectory ended without success: %s", message.c_str());
  }

  void execute(const std::shared_ptr<ServerGoalHandle> & server_goal)
  {
    struct ActiveGuard
    {
      explicit ActiveGuard(std::atomic<bool> & active) : active_(active) {}
      ~ActiveGuard() {active_.store(false);}
      std::atomic<bool> & active_;
    } active_guard(goal_active_);

    const auto goal = server_goal->get_goal();
    const bool scan_enabled = scan_on_execute_.load();
    std::string error;
    if (scan_enabled) {
      if (!request_laser(true, &error)) {
        terminate(
          server_goal, false, Follow::Result::INVALID_GOAL,
          "scan not started: " + error);
        return;
      }
      if (!request_accumulation(true, &error)) {
        std::string ignored;
        (void)request_laser(false, &ignored);
        terminate(
          server_goal, false, Follow::Result::INVALID_GOAL,
          "scan not started: " + error);
        return;
      }
    } else {
      // A positioning Execute must never inherit a previously active laser or
      // accumulator.  Refuse motion unless both OFF operations are confirmed.
      std::string accumulation_error;
      const bool accumulation_off = request_accumulation(false, &accumulation_error);
      std::string laser_error;
      const bool laser_off = request_laser(false, &laser_error);
      if (!accumulation_off || !laser_off) {
        terminate(
          server_goal, false, Follow::Result::INVALID_GOAL,
          "positioning not started because safe OFF state was not confirmed: " +
          accumulation_error + " " + laser_error);
        return;
      }
    }

    if (!controller_client_->wait_for_action_server(2s)) {
      finish_interlocks(nullptr);
      terminate(
        server_goal, false, Follow::Result::INVALID_GOAL,
        "real joint trajectory controller is unavailable");
      return;
    }

    Follow::Goal forwarded = *goal;
    typename rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.feedback_callback =
      [server_goal](ClientGoalHandle::SharedPtr, const std::shared_ptr<const Follow::Feedback> feedback) {
        if (server_goal->is_active()) {
          server_goal->publish_feedback(std::make_shared<Follow::Feedback>(*feedback));
        }
      };
    auto accepted_future = controller_client_->async_send_goal(forwarded, options);
    if (accepted_future.wait_for(2s) != std::future_status::ready) {
      finish_interlocks(nullptr);
      terminate(
        server_goal, false, Follow::Result::INVALID_GOAL,
        "real controller did not accept/reject the trajectory in time");
      return;
    }
    const ClientGoalHandle::SharedPtr real_goal = accepted_future.get();
    if (!real_goal) {
      finish_interlocks(nullptr);
      terminate(
        server_goal, false, Follow::Result::INVALID_GOAL,
        "real controller rejected the trajectory");
      return;
    }

    const auto & final_time = forwarded.trajectory.points.back().time_from_start;
    const double planned_seconds = static_cast<double>(final_time.sec) +
      static_cast<double>(final_time.nanosec) * 1.0e-9;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::clamp(planned_seconds + 5.0, 5.0, 305.0)));
    auto result_future = controller_client_->async_get_result(real_goal);
    while (result_future.wait_for(50ms) != std::future_status::ready) {
      if (cancel_requested_.load() || server_goal->is_canceling()) {
        (void)controller_client_->async_cancel_goal(real_goal);
        std::string warning;
        finish_interlocks(&warning);
        terminate(
          server_goal, true, Follow::Result::INVALID_GOAL,
          "RViz trajectory canceled" + (warning.empty() ? "" : "; " + warning));
        return;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        (void)controller_client_->async_cancel_goal(real_goal);
        std::string warning;
        finish_interlocks(&warning);
        terminate(
          server_goal, false, Follow::Result::INVALID_GOAL,
          "real controller result timed out" + (warning.empty() ? "" : "; " + warning));
        return;
      }
    }

    const ClientGoalHandle::WrappedResult real_result = result_future.get();
    std::string warning;
    finish_interlocks(&warning);
    if (real_result.code != rclcpp_action::ResultCode::SUCCEEDED || !real_result.result ||
      real_result.result->error_code != Follow::Result::SUCCESSFUL || !warning.empty())
    {
      const std::string detail = !warning.empty() ? warning :
        (real_result.result ? real_result.result->error_string : "real controller returned no result");
      terminate(
        server_goal, real_result.code == rclcpp_action::ResultCode::CANCELED,
        real_result.result ? real_result.result->error_code : Follow::Result::INVALID_GOAL,
        detail);
      return;
    }

    auto result = std::make_shared<Follow::Result>();
    result->error_code = Follow::Result::SUCCESSFUL;
    result->error_string = scan_enabled ?
      "RViz planned trajectory scanned; accumulation stopped and laser confirmed OFF" :
      "RViz positioning trajectory completed with laser confirmed OFF";
    server_goal->succeed(result);
    RCLCPP_INFO(get_logger(), "%s", result->error_string.c_str());
  }

  bool allow_execution_{false};
  std::atomic<bool> scan_on_execute_{false};
  bool require_laser_control_{true};
  std::string proxy_action_name_;
  std::string controller_action_name_;
  std::string accumulation_service_name_;
  std::string laser_service_name_;
  std::string scan_mode_service_name_;
  std::mutex mode_mutex_;
  std::atomic<bool> goal_active_{false};
  std::atomic<bool> cancel_requested_{false};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr accumulation_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr laser_client_;
  rclcpp_action::Client<Follow>::SharedPtr controller_client_;
  rclcpp_action::Server<Follow>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr scan_mode_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RvizScanTrajectoryRelay>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("rviz_scan_trajectory_relay"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

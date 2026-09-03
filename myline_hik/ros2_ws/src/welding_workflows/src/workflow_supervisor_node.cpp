#include "welding_workflows/workflow_state_machine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <welding_interfaces/action/run_automatic_calibration.hpp>
#include <welding_interfaces/action/run_scan_to_weld.hpp>
#include <welding_interfaces/msg/task_state.hpp>
#include <welding_interfaces/srv/control_task.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class WorkflowSupervisor final : public rclcpp::Node
{
public:
  using Calibration = welding_interfaces::action::RunAutomaticCalibration;
  using CalibrationHandle = rclcpp_action::ServerGoalHandle<Calibration>;
  using ScanToWeld = welding_interfaces::action::RunScanToWeld;
  using ScanHandle = rclcpp_action::ServerGoalHandle<ScanToWeld>;

  WorkflowSupervisor()
  : Node("workflow_supervisor"), session_root_(declare_parameter<std::string>("session_root", "data/sessions"))
  {
    state_publisher_ = create_publisher<welding_interfaces::msg::TaskState>(
      "/welding_robot/workflow/state", rclcpp::QoS(1).reliable().transient_local());
    control_service_ = create_service<welding_interfaces::srv::ControlTask>(
      "/welding_robot/workflow/control",
      [this](const std::shared_ptr<welding_interfaces::srv::ControlTask::Request> request,
        std::shared_ptr<welding_interfaces::srv::ControlTask::Response> response) {control(*request, response.get());});
    calibration_server_ = rclcpp_action::create_server<Calibration>(
      this, "/welding_robot/run_automatic_calibration",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Calibration::Goal> goal) {
        return accept_calibration(*goal);
      },
      [](std::shared_ptr<CalibrationHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<CalibrationHandle> goal) {run_calibration(std::move(goal));});
    scan_server_ = rclcpp_action::create_server<ScanToWeld>(
      this, "/welding_robot/run_scan_to_weld",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ScanToWeld::Goal> goal) {
        return accept_scan(*goal);
      },
      [](std::shared_ptr<ScanHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<ScanHandle> goal) {run_scan(std::move(goal));});
    publish_state();
  }

private:
  bool session_exists(const std::string & id) const
  {
    return !id.empty() && std::filesystem::is_regular_file(session_root_ / id / "session.yaml");
  }

  std::string write_dry_run_report(
    const std::string & session_id, const std::string & name, const std::string & final_state)
  {
    const auto directory = session_root_ / session_id / "reports";
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) {return {};}
    const auto path = directory / (name + ".yaml");
    std::ofstream stream(path, std::ios::trunc);
    stream << "schema_version: 2\nmode: commissioning_dry_run\nworkflow: " << name
           << "\nfinal_state: " << final_state
           << "\nhardware_commands_sent: false\n";
    return stream ? path.string() : std::string();
  }

  rclcpp_action::GoalResponse accept_calibration(const Calibration::Goal & goal)
  {
    if (busy_.exchange(true)) {return rclcpp_action::GoalResponse::REJECT;}
    if (!goal.dry_run || !session_exists(goal.session_id) ||
      !std::filesystem::is_regular_file(goal.board_config_path)) {
      busy_ = false;
      RCLCPP_ERROR(get_logger(), "automatic calibration hardware execution is gated; use dry_run until adapters are commissioned");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse accept_scan(const ScanToWeld::Goal & goal)
  {
    if (busy_.exchange(true)) {return rclcpp_action::GoalResponse::REJECT;}
    if (!goal.dry_run || !goal.stop_before_arc || !session_exists(goal.session_id) ||
      goal.calibration_package_id.empty() || !std::filesystem::is_regular_file(goal.acceptance_profile_path)) {
      busy_ = false;
      RCLCPP_ERROR(get_logger(), "scan-to-weld is accepted only as stop-before-arc dry-run in this commissioning build");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  template<typename HandleT, typename FeedbackT>
  bool advance_dry_run(
    const std::shared_ptr<HandleT> & handle,
    const std::vector<std::string> & stages,
    std::size_t stop_after,
    FeedbackT * feedback)
  {
    for (std::size_t index = 0; index <= stop_after && index < stages.size(); ++index) {
      while (machine_.paused() && rclcpp::ok()) {std::this_thread::sleep_for(20ms);}
      if (handle->is_canceling() || machine_.stage() == "ABORTED") {return false;}
      feedback->progress = machine_.progress();
      feedback->state = machine_.stage();
      feedback->detail = "commissioning dry-run; no hardware command";
      handle->publish_feedback(std::make_shared<FeedbackT>(*feedback));
      publish_state();
      std::this_thread::sleep_for(20ms);
      std::string error;
      if (!machine_.advance(stages[index], &error)) {return false;}
    }
    publish_state();
    return true;
  }

  void run_calibration(std::shared_ptr<CalibrationHandle> handle)
  {
    std::thread([this, handle]() {
      const auto goal = handle->get_goal();
      const auto stages = welding_workflows::automatic_calibration_stages();
      std::string error;
      machine_.start(goal->session_id, "automatic_calibration", "automatic_calibration", stages, &error);
      Calibration::Feedback feedback;
      const bool complete = advance_dry_run(handle, stages, stages.size() - 1U, &feedback);
      auto result = std::make_shared<Calibration::Result>();
      result->success = complete;
      result->report_path = write_dry_run_report(
        goal->session_id, "automatic_calibration_dry_run", machine_.stage());
      if (complete) {handle->succeed(result);} else if (handle->is_canceling()) {handle->canceled(result);} else {
        result->error = "calibration dry-run aborted"; handle->abort(result);
      }
      busy_ = false;
    }).detach();
  }

  void run_scan(std::shared_ptr<ScanHandle> handle)
  {
    std::thread([this, handle]() {
      const auto goal = handle->get_goal();
      const auto stages = welding_workflows::scan_to_weld_stages();
      std::string error;
      machine_.start(goal->session_id, "scan_to_weld", "scan_to_weld", stages, &error);
      ScanToWeld::Feedback feedback;
      feedback.review_required = true;
      const std::size_t review_index = static_cast<std::size_t>(
        std::find(stages.begin(), stages.end(), "WELD_REVIEW") - stages.begin());
      const bool reached_review = advance_dry_run(handle, stages, review_index - 1U, &feedback);
      auto result = std::make_shared<ScanToWeld::Result>();
      result->success = reached_review;
      result->final_state = reached_review ? "WELD_REVIEW" : machine_.stage();
      result->report_path = write_dry_run_report(
        goal->session_id, "scan_to_weld_dry_run", result->final_state);
      if (reached_review) {handle->succeed(result);} else if (handle->is_canceling()) {handle->canceled(result);} else {
        result->error = "scan-to-weld dry-run aborted"; handle->abort(result);
      }
      busy_ = false;
    }).detach();
  }

  void control(
    const welding_interfaces::srv::ControlTask::Request & request,
    welding_interfaces::srv::ControlTask::Response * response)
  {
    if (request.session_id != machine_.session_id() || request.task_id != machine_.task_id()) {
      response->error = "control request does not identify the active task";
      return;
    }
    if (request.expected_state != make_state().state) {
      response->error = "control request expected_state is stale";
      response->state = make_state();
      return;
    }
    std::string error;
    if (request.command == welding_interfaces::srv::ControlTask::Request::PAUSE) {
      response->accepted = machine_.pause(machine_.stage(), &error);
    } else if (request.command == welding_interfaces::srv::ControlTask::Request::RESUME) {
      response->accepted = machine_.resume(&error);
    } else if (request.command == welding_interfaces::srv::ControlTask::Request::ABORT) {
      response->accepted = machine_.abort(&error);
    } else {error = "unknown control command";}
    response->error = error;
    response->state = make_state();
    publish_state();
  }

  welding_interfaces::msg::TaskState make_state() const
  {
    welding_interfaces::msg::TaskState state;
    state.header.stamp = now();
    state.session_id = machine_.session_id();
    state.task_id = machine_.task_id();
    state.workflow = machine_.workflow();
    state.progress = machine_.progress();
    state.detail = machine_.stage();
    state.state = machine_.stage() == "IDLE" ? welding_interfaces::msg::TaskState::IDLE :
      machine_.stage() == "PAUSED" ? welding_interfaces::msg::TaskState::PAUSED :
      machine_.stage() == "ABORTED" ? welding_interfaces::msg::TaskState::ABORTED :
      machine_.stage() == "FAULT" ? welding_interfaces::msg::TaskState::FAULT :
      machine_.stage() == "COMPLETE" ? welding_interfaces::msg::TaskState::COMPLETE :
      welding_interfaces::msg::TaskState::EXECUTING;
    state.motion_lease_active = false;
    state.laser_active = false;
    state.dependencies_valid = false;
    return state;
  }

  void publish_state() {state_publisher_->publish(make_state());}

  std::filesystem::path session_root_;
  std::atomic_bool busy_{false};
  welding_workflows::WorkflowStateMachine machine_;
  rclcpp::Publisher<welding_interfaces::msg::TaskState>::SharedPtr state_publisher_;
  rclcpp::Service<welding_interfaces::srv::ControlTask>::SharedPtr control_service_;
  rclcpp_action::Server<Calibration>::SharedPtr calibration_server_;
  rclcpp_action::Server<ScanToWeld>::SharedPtr scan_server_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  const auto node = std::make_shared<WorkflowSupervisor>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

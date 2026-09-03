#ifndef WELDING_WORKFLOWS__WORKFLOW_STATE_MACHINE_HPP_
#define WELDING_WORKFLOWS__WORKFLOW_STATE_MACHINE_HPP_

#include <mutex>
#include <string>
#include <vector>

namespace welding_workflows
{

class WorkflowStateMachine
{
public:
  bool start(
    const std::string & session_id,
    const std::string & task_id,
    const std::string & workflow,
    const std::vector<std::string> & stages,
    std::string * error = nullptr);
  bool advance(const std::string & expected_stage, std::string * error = nullptr);
  bool pause(const std::string & expected_stage, std::string * error = nullptr);
  bool resume(std::string * error = nullptr);
  bool abort(std::string * error = nullptr);
  bool fault(const std::string & detail, std::string * error = nullptr);
  void reset();

  std::string stage() const;
  std::string session_id() const;
  std::string task_id() const;
  std::string workflow() const;
  std::string detail() const;
  float progress() const;
  bool paused() const;
  bool terminal() const;

private:
  mutable std::mutex mutex_;
  std::string session_id_;
  std::string task_id_;
  std::string workflow_;
  std::vector<std::string> stages_;
  std::size_t index_{0};
  std::string state_{"IDLE"};
  std::string resume_state_;
  std::string detail_;
};

std::vector<std::string> automatic_calibration_stages();
std::vector<std::string> scan_to_weld_stages();

}  // namespace welding_workflows

#endif  // WELDING_WORKFLOWS__WORKFLOW_STATE_MACHINE_HPP_

#include "welding_workflows/workflow_state_machine.hpp"

#include <algorithm>

namespace welding_workflows
{
namespace
{

void set_error(const std::string & value, std::string * error)
{
  if (error != nullptr) {*error = value;}
}

bool is_terminal(const std::string & state)
{
  return state == "COMPLETE" || state == "ABORTED" || state == "FAULT";
}

}  // namespace

bool WorkflowStateMachine::start(
  const std::string & session_id,
  const std::string & task_id,
  const std::string & workflow,
  const std::vector<std::string> & stages,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_terminal(state_) && state_ != "IDLE") {set_error("another workflow is active", error); return false;}
  if (session_id.empty() || task_id.empty() || workflow.empty() || stages.empty() ||
    std::any_of(stages.begin(), stages.end(), [](const std::string & stage) {return stage.empty();}))
  {set_error("workflow identity and stages must be non-empty", error); return false;}
  session_id_ = session_id;
  task_id_ = task_id;
  workflow_ = workflow;
  stages_ = stages;
  index_ = 0;
  state_ = stages_.front();
  resume_state_.clear();
  detail_.clear();
  return true;
}

bool WorkflowStateMachine::advance(const std::string & expected_stage, std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == "PAUSED") {set_error("workflow is paused", error); return false;}
  if (is_terminal(state_) || state_ == "IDLE") {set_error("workflow is not advanceable", error); return false;}
  if (state_ != expected_stage) {set_error("stale stage expectation: active=" + state_, error); return false;}
  ++index_;
  state_ = index_ < stages_.size() ? stages_[index_] : "COMPLETE";
  return true;
}

bool WorkflowStateMachine::pause(const std::string & expected_stage, std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != expected_stage || state_ == "IDLE" || is_terminal(state_) || state_ == "PAUSED") {
    set_error("pause rejected by expected-state guard", error);
    return false;
  }
  resume_state_ = state_;
  state_ = "PAUSED";
  return true;
}

bool WorkflowStateMachine::resume(std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != "PAUSED" || resume_state_.empty()) {set_error("workflow is not paused", error); return false;}
  state_ = resume_state_;
  resume_state_.clear();
  return true;
}

bool WorkflowStateMachine::abort(std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == "IDLE" || is_terminal(state_)) {set_error("workflow is not active", error); return false;}
  state_ = "ABORTED";
  detail_ = "operator abort";
  resume_state_.clear();
  return true;
}

bool WorkflowStateMachine::fault(const std::string & detail, std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == "IDLE" || is_terminal(state_) || detail.empty()) {set_error("fault transition rejected", error); return false;}
  state_ = "FAULT";
  detail_ = detail;
  resume_state_.clear();
  return true;
}

void WorkflowStateMachine::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  session_id_.clear(); task_id_.clear(); workflow_.clear(); stages_.clear(); resume_state_.clear(); detail_.clear();
  index_ = 0; state_ = "IDLE";
}

std::string WorkflowStateMachine::stage() const {std::lock_guard<std::mutex> lock(mutex_); return state_;}
std::string WorkflowStateMachine::session_id() const {std::lock_guard<std::mutex> lock(mutex_); return session_id_;}
std::string WorkflowStateMachine::task_id() const {std::lock_guard<std::mutex> lock(mutex_); return task_id_;}
std::string WorkflowStateMachine::workflow() const {std::lock_guard<std::mutex> lock(mutex_); return workflow_;}
std::string WorkflowStateMachine::detail() const {std::lock_guard<std::mutex> lock(mutex_); return detail_;}
bool WorkflowStateMachine::paused() const {std::lock_guard<std::mutex> lock(mutex_); return state_ == "PAUSED";}
bool WorkflowStateMachine::terminal() const {std::lock_guard<std::mutex> lock(mutex_); return is_terminal(state_);}
float WorkflowStateMachine::progress() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == "COMPLETE") {return 1.0F;}
  return stages_.empty() ? 0.0F : static_cast<float>(index_) / static_cast<float>(stages_.size());
}

std::vector<std::string> automatic_calibration_stages()
{
  return {"DEVICE_CHECK", "SEED_CHECK", "PATH_GENERATION", "PATH_VALIDATION", "DUAL_CAMERA_CAPTURE",
    "CAMERA_650_SOLVE", "CAMERA_450_SOLVE", "HAND_EYE_650_SOLVE", "HAND_EYE_450_SOLVE",
    "STEREO_EXTRINSIC_SOLVE", "TRANSFORM_LOOP_VALIDATION", "LASER_650_CAPTURE", "LASER_650_SOLVE",
    "LASER_450_CAPTURE", "LASER_450_SOLVE", "VALIDATION", "PACKAGE_GENERATION"};
}

std::vector<std::string> scan_to_weld_stages()
{
  return {"SYSTEM_CHECK", "DEVICE_READY", "CALIBRATION_READY", "COARSE_MAP_PLAN", "COARSE_MAP_EXECUTE",
    "SCENE_MAP_BUILD", "COLLISION_MAP_UPDATE", "SEAM_CANDIDATE_DETECTION", "SCAN_PLAN", "SCAN_VALIDATE",
    "SCAN_REVIEW", "SCAN_EXECUTE", "POINTCLOUD_BUILD", "QUALITY_EVALUATION", "RESCAN_DECISION",
    "RESCAN_PLAN", "RESCAN_VALIDATE", "RESCAN_EXECUTE", "POINTCLOUD_FUSION", "SEAM_EXTRACTION",
    "SEAM_REVIEW", "WELD_PLAN", "WELD_VALIDATE", "DRY_RUN", "WELD_REVIEW", "WELD_EXECUTE",
    "REPORT_GENERATION"};
}

}  // namespace welding_workflows

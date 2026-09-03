#include "welding_planning/plan_dependency_validator.hpp"

#include <array>
#include <utility>

namespace welding_planning
{
namespace
{

void set_reason(const std::string & text, std::string * reason)
{
  if (reason != nullptr) {*reason = text;}
}

}  // namespace

bool complete_dependencies(
  const welding_interfaces::msg::PlanDependencies & value,
  std::string * reason)
{
  const std::array<std::pair<const char *, const std::string *>, 9> required = {{
    {"calibration_package_id", &value.calibration_package_id},
    {"calibration_package_sha256", &value.calibration_package_sha256},
    {"planning_scene_id", &value.planning_scene_id},
    {"planning_scene_sha256", &value.planning_scene_sha256},
    {"robot_model_sha256", &value.robot_model_sha256},
    {"srdf_sha256", &value.srdf_sha256},
    {"tool_model_sha256", &value.tool_model_sha256},
    {"workpiece_frame_version", &value.workpiece_frame_version},
    {"planner_config_sha256", &value.planner_config_sha256}}};
  if (value.planning_scene_version == 0U) {
    set_reason("planning_scene_version is zero", reason);
    return false;
  }
  for (const auto & field : required) {
    if (field.second->empty()) {
      set_reason(std::string("missing dependency: ") + field.first, reason);
      return false;
    }
  }
  return true;
}

bool same_dependencies(
  const welding_interfaces::msg::PlanDependencies & expected,
  const welding_interfaces::msg::PlanDependencies & actual,
  std::string * reason)
{
#define CHECK_FIELD(field) \
  if (expected.field != actual.field) {set_reason("stale dependency: " #field, reason); return false;}
  CHECK_FIELD(calibration_package_id)
  CHECK_FIELD(calibration_package_sha256)
  CHECK_FIELD(planning_scene_id)
  CHECK_FIELD(planning_scene_version)
  CHECK_FIELD(planning_scene_sha256)
  CHECK_FIELD(robot_model_sha256)
  CHECK_FIELD(srdf_sha256)
  CHECK_FIELD(tool_model_sha256)
  CHECK_FIELD(workpiece_frame_version)
  CHECK_FIELD(planner_config_sha256)
#undef CHECK_FIELD
  return true;
}

bool PlanDependencyValidator::activate(
  const welding_interfaces::msg::PlanDependencies & dependencies,
  std::string * reason)
{
  if (!complete_dependencies(dependencies, reason)) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = dependencies;
  return true;
}

bool PlanDependencyValidator::validate(
  const welding_interfaces::msg::PlanDependencies & dependencies,
  welding_interfaces::msg::PlanDependencies * active,
  std::string * reason) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    set_reason("no active planning scene", reason);
    return false;
  }
  if (active != nullptr) {*active = *active_;}
  if (!complete_dependencies(dependencies, reason)) {return false;}
  return same_dependencies(dependencies, *active_, reason);
}

std::optional<welding_interfaces::msg::PlanDependencies> PlanDependencyValidator::active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

}  // namespace welding_planning

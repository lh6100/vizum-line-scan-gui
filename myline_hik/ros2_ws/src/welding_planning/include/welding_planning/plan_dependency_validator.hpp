#ifndef WELDING_PLANNING__PLAN_DEPENDENCY_VALIDATOR_HPP_
#define WELDING_PLANNING__PLAN_DEPENDENCY_VALIDATOR_HPP_

#include <welding_interfaces/msg/plan_dependencies.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace welding_planning
{

bool complete_dependencies(
  const welding_interfaces::msg::PlanDependencies & dependencies,
  std::string * reason = nullptr);

bool same_dependencies(
  const welding_interfaces::msg::PlanDependencies & expected,
  const welding_interfaces::msg::PlanDependencies & actual,
  std::string * reason = nullptr);

class PlanDependencyValidator
{
public:
  bool activate(
    const welding_interfaces::msg::PlanDependencies & dependencies,
    std::string * reason = nullptr);
  bool validate(
    const welding_interfaces::msg::PlanDependencies & dependencies,
    welding_interfaces::msg::PlanDependencies * active,
    std::string * reason = nullptr) const;
  std::optional<welding_interfaces::msg::PlanDependencies> active() const;

private:
  mutable std::mutex mutex_;
  std::optional<welding_interfaces::msg::PlanDependencies> active_;
};

}  // namespace welding_planning

#endif  // WELDING_PLANNING__PLAN_DEPENDENCY_VALIDATOR_HPP_

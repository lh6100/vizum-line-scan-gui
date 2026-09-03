#ifndef WELDING_EXECUTION__EXECUTION_GUARD_HPP_
#define WELDING_EXECUTION__EXECUTION_GUARD_HPP_

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace welding_execution
{

std::string trajectory_sha256(const trajectory_msgs::msg::JointTrajectory & trajectory);
bool valid_identifier(const std::string & value);

class MotionLease
{
public:
  bool acquire(const std::string & owner, std::string * error = nullptr);
  bool release(const std::string & owner, std::string * error = nullptr);
  std::optional<std::string> owner() const;

private:
  mutable std::mutex mutex_;
  std::optional<std::string> owner_;
};

}  // namespace welding_execution

#endif  // WELDING_EXECUTION__EXECUTION_GUARD_HPP_

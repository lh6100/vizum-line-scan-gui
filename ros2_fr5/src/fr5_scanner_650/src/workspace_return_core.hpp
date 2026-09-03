#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>

namespace fr5_scanner_650::workspace_return
{

// Build the time-reversed form of a validated joint trajectory. Positions are
// reversed, velocities change sign and accelerations retain their sign.
// Effort feed-forward values are intentionally discarded because their safe
// reverse-time meaning is controller-specific.
bool reverseJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & forward,
  trajectory_msgs::msg::JointTrajectory * reversed,
  std::string * error = nullptr);

}  // namespace fr5_scanner_650::workspace_return

#pragma once

#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>

#include <string>

namespace fr5_scanner_650::workspace_robot_state
{

// RobotState returned by a state monitor may have dirty forward-kinematics
// caches.  MoveIt's const getGlobalLinkTransform() asserts when that happens.
// Work on a local mutable copy so callers can safely inspect a const snapshot
// without mutating it or risking a process-wide SIGABRT.
bool updatedGlobalLinkTransform(
  const moveit::core::RobotState & state, const std::string & link_name,
  Eigen::Isometry3d * transform, std::string * error = nullptr);

}  // namespace fr5_scanner_650::workspace_robot_state

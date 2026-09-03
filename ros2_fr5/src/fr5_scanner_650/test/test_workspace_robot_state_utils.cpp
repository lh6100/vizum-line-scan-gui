#include "workspace_robot_state_utils.hpp"

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_model/types.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace robot_state_utils = fr5_scanner_650::workspace_robot_state;

namespace
{

moveit::core::RobotModelPtr buildTestRobot()
{
  moveit::core::RobotModelBuilder builder("return_state_test_robot", "base");
  geometry_msgs::msg::Pose joint_origin;
  joint_origin.position.x = 0.25;
  joint_origin.orientation.w = 1.0;
  builder.addChain(
    "base->tool", "revolute", {joint_origin}, urdf::Vector3(0.0, 0.0, 1.0));
  if (!builder.isValid()) {
    return {};
  }
  return builder.build();
}

}  // namespace

TEST(WorkspaceRobotStateUtils, UpdatesDirtyFkCacheOnLocalCopy)
{
  const moveit::core::RobotModelPtr model = buildTestRobot();
  ASSERT_TRUE(model);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.update();

  const std::string joint_name = "base-tool-joint";
  const auto & variable_names = model->getVariableNames();
  ASSERT_NE(
    std::find(variable_names.begin(), variable_names.end(), joint_name),
    variable_names.end());
  state.setVariablePosition(joint_name, M_PI / 3.0);
  ASSERT_TRUE(state.dirtyLinkTransforms());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  std::string error;
  ASSERT_TRUE(
    robot_state_utils::updatedGlobalLinkTransform(
      state, "tool", &transform, &error)) << error;

  // Reading a const dirty state used to assert here.  The helper must update a
  // copy and leave the state-monitor snapshot unchanged.
  EXPECT_TRUE(state.dirtyLinkTransforms());
  moveit::core::RobotState expected(state);
  expected.update();
  EXPECT_TRUE(transform.isApprox(expected.getGlobalLinkTransform("tool"), 1.0e-12));
  EXPECT_TRUE(transform.matrix().allFinite());
}

TEST(WorkspaceRobotStateUtils, RejectsUnknownLinkWithoutThrowing)
{
  const moveit::core::RobotModelPtr model = buildTestRobot();
  ASSERT_TRUE(model);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  std::string error;
  EXPECT_FALSE(
    robot_state_utils::updatedGlobalLinkTransform(
      state, "missing_tcp", &transform, &error));
  EXPECT_NE(error.find("missing_tcp"), std::string::npos);
}

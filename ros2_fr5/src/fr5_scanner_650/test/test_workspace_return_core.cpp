#include "workspace_return_core.hpp"

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cstdint>
#include <string>

namespace workspace_return = fr5_scanner_650::workspace_return;

namespace
{

builtin_interfaces::msg::Duration seconds(std::int32_t value)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = value;
  return duration;
}

trajectory_msgs::msg::JointTrajectory sampleTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"j1", "j2"};
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = {0.0, 10.0};
  first.velocities = {1.0, 2.0};
  first.accelerations = {3.0, 4.0};
  first.effort = {5.0, 6.0};
  first.time_from_start = seconds(0);
  trajectory_msgs::msg::JointTrajectoryPoint middle;
  middle.positions = {1.0, 11.0};
  middle.velocities = {0.5, 1.5};
  middle.accelerations = {2.5, 3.5};
  middle.time_from_start = seconds(1);
  trajectory_msgs::msg::JointTrajectoryPoint last;
  last.positions = {3.0, 13.0};
  last.velocities = {0.0, 0.0};
  last.accelerations = {0.0, 0.0};
  last.time_from_start = seconds(3);
  trajectory.points = {first, middle, last};
  return trajectory;
}

}  // namespace

TEST(WorkspaceReturnCore, ReversesPositionsTimeAndDerivatives)
{
  const auto forward = sampleTrajectory();
  trajectory_msgs::msg::JointTrajectory reversed;
  std::string error;
  ASSERT_TRUE(workspace_return::reverseJointTrajectory(forward, &reversed, &error)) << error;
  ASSERT_EQ(reversed.points.size(), 3U);
  EXPECT_EQ(reversed.points[0].positions, (std::vector<double>{3.0, 13.0}));
  EXPECT_EQ(reversed.points[1].positions, (std::vector<double>{1.0, 11.0}));
  EXPECT_EQ(reversed.points[2].positions, (std::vector<double>{0.0, 10.0}));
  EXPECT_EQ(reversed.points[0].time_from_start.sec, 0);
  EXPECT_EQ(reversed.points[1].time_from_start.sec, 2);
  EXPECT_EQ(reversed.points[2].time_from_start.sec, 3);
  EXPECT_EQ(reversed.points[1].velocities, (std::vector<double>{-0.5, -1.5}));
  EXPECT_EQ(reversed.points[1].accelerations, (std::vector<double>{2.5, 3.5}));
  EXPECT_TRUE(reversed.points[2].effort.empty());
}

TEST(WorkspaceReturnCore, DoubleReverseRestoresKinematics)
{
  const auto forward = sampleTrajectory();
  trajectory_msgs::msg::JointTrajectory once;
  trajectory_msgs::msg::JointTrajectory twice;
  ASSERT_TRUE(workspace_return::reverseJointTrajectory(forward, &once));
  ASSERT_TRUE(workspace_return::reverseJointTrajectory(once, &twice));
  ASSERT_EQ(twice.points.size(), forward.points.size());
  for (std::size_t index = 0U; index < forward.points.size(); ++index) {
    EXPECT_EQ(twice.points[index].positions, forward.points[index].positions);
    EXPECT_EQ(twice.points[index].velocities, forward.points[index].velocities);
    EXPECT_EQ(twice.points[index].accelerations, forward.points[index].accelerations);
    EXPECT_EQ(twice.points[index].time_from_start, forward.points[index].time_from_start);
  }
}

TEST(WorkspaceReturnCore, RejectsInvalidTrajectory)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory_msgs::msg::JointTrajectory reversed;
  std::string error;
  EXPECT_FALSE(workspace_return::reverseJointTrajectory(trajectory, &reversed, &error));

  trajectory = sampleTrajectory();
  trajectory.points[1].positions.pop_back();
  EXPECT_FALSE(workspace_return::reverseJointTrajectory(trajectory, &reversed, &error));

  trajectory = sampleTrajectory();
  trajectory.points[2].time_from_start = trajectory.points[1].time_from_start;
  EXPECT_FALSE(workspace_return::reverseJointTrajectory(trajectory, &reversed, &error));
}

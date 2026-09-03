#include "welding_execution/execution_guard.hpp"

#include <gtest/gtest.h>

namespace
{

TEST(ExecutionGuard, DigestChangesWithAnyTrajectoryValue)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint1", "joint2"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {1.0, 2.0};
  point.time_from_start.sec = 1;
  trajectory.points.push_back(point);
  const auto original = welding_execution::trajectory_sha256(trajectory);
  ASSERT_EQ(original.size(), 64U);
  trajectory.points[0].positions[1] = 2.000001;
  EXPECT_NE(original, welding_execution::trajectory_sha256(trajectory));
}

TEST(ExecutionGuard, MotionLeaseHasSingleOwner)
{
  welding_execution::MotionLease lease;
  std::string error;
  EXPECT_TRUE(lease.acquire("scan_task", &error));
  EXPECT_FALSE(lease.acquire("weld_task", &error));
  EXPECT_FALSE(lease.release("weld_task", &error));
  EXPECT_TRUE(lease.release("scan_task", &error));
  EXPECT_TRUE(lease.acquire("weld_task", &error));
}

}  // namespace

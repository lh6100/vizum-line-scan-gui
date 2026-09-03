#include "welding_planning/plan_dependency_validator.hpp"

#include <gtest/gtest.h>

namespace
{

welding_interfaces::msg::PlanDependencies valid_dependencies()
{
  welding_interfaces::msg::PlanDependencies value;
  value.calibration_package_id = "cal_001";
  value.calibration_package_sha256 = "cal_hash";
  value.planning_scene_id = "scene_001";
  value.planning_scene_version = 1;
  value.planning_scene_sha256 = "scene_hash";
  value.robot_model_sha256 = "urdf_hash";
  value.srdf_sha256 = "srdf_hash";
  value.tool_model_sha256 = "tool_hash";
  value.workpiece_frame_version = "fixture_v1";
  value.planner_config_sha256 = "planner_hash";
  return value;
}

TEST(PlanDependencyValidator, RejectsIncompleteAndStalePlans)
{
  welding_planning::PlanDependencyValidator validator;
  std::string reason;
  auto dependencies = valid_dependencies();
  auto incomplete = dependencies;
  incomplete.tool_model_sha256.clear();
  EXPECT_FALSE(validator.activate(incomplete, &reason));
  ASSERT_TRUE(validator.activate(dependencies, &reason)) << reason;
  welding_interfaces::msg::PlanDependencies active;
  EXPECT_TRUE(validator.validate(dependencies, &active, &reason));
  auto stale = dependencies;
  stale.planning_scene_version = 2;
  EXPECT_FALSE(validator.validate(stale, &active, &reason));
  EXPECT_NE(reason.find("planning_scene_version"), std::string::npos);
  stale = dependencies;
  stale.calibration_package_sha256 = "changed";
  EXPECT_FALSE(validator.validate(stale, &active, &reason));
}

}  // namespace

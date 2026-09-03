#include "workspace_coarse_scan_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace ws = fr5_scanner_650::workspace_scan;

TEST(WorkspaceCoarseScanCore, BuildsAlternatingFrontLayeredLanes)
{
  ws::WorkspaceOptions options;
  options.radial_min_m = 0.95;
  options.radial_max_m = 1.20;
  options.radial_spacing_m = 0.125;
  options.height_min_m = 0.30;
  options.height_max_m = 0.70;
  options.height_spacing_m = 0.20;
  options.azimuth_min_deg = -90.0;
  options.azimuth_max_deg = 90.0;
  options.maximum_measurement_arc_span_deg = 180.0;
  ws::WorkspacePlan plan;
  std::string error;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan, &error)) << error;
  ASSERT_EQ(plan.radial_layers_m.size(), 3U);
  ASSERT_EQ(plan.height_layers_m.size(), 3U);
  ASSERT_EQ(plan.lanes.size(), 9U);
  EXPECT_FALSE(plan.lanes[0].reverse);
  EXPECT_TRUE(plan.lanes[1].reverse);
  EXPECT_DOUBLE_EQ(plan.lanes[0].azimuth_start_deg, -90.0);
  EXPECT_DOUBLE_EQ(plan.lanes[0].azimuth_end_deg, 90.0);
  EXPECT_DOUBLE_EQ(plan.lanes[1].azimuth_start_deg, 90.0);
  EXPECT_DOUBLE_EQ(plan.lanes[1].azimuth_end_deg, -90.0);
  EXPECT_NEAR(plan.lanes[0].measurement_length_m, M_PI * 0.95, 1.0e-12);
  EXPECT_GT(plan.total_measurement_length_m, 0.0);
}

TEST(WorkspaceCoarseScanCore, KeepsLaserLineVerticalAndCameraFacingOutward)
{
  ws::WorkspaceOptions options;
  options.maximum_measurement_arc_span_deg = 180.0;
  ws::WorkspacePlan plan;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan));
  const auto & midpoint = plan.lanes.front().measurement_mid;
  // At zero azimuth and the safer 180-degree optical roll, TCP Y follows
  // vertical -Z while TCP Z remains radial +X.
  const auto rotate = [](const ws::Quaternion & q, const ws::Vec3 & v) {
      const ws::Vec3 u{q.x, q.y, q.z};
      const ws::Vec3 uv{
        u.y * v.z - u.z * v.y,
        u.z * v.x - u.x * v.z,
        u.x * v.y - u.y * v.x};
      const ws::Vec3 uuv{
        u.y * uv.z - u.z * uv.y,
        u.z * uv.x - u.x * uv.z,
        u.x * uv.y - u.y * uv.x};
      return ws::Vec3{
        v.x + 2.0 * (q.w * uv.x + uuv.x),
        v.y + 2.0 * (q.w * uv.y + uuv.y),
        v.z + 2.0 * (q.w * uv.z + uuv.z)};
    };
  const ws::Vec3 laser_axis = rotate(midpoint.orientation, {0.0, 1.0, 0.0});
  const ws::Vec3 optical_axis = rotate(midpoint.orientation, {0.0, 0.0, 1.0});
  EXPECT_NEAR(laser_axis.x, 0.0, 1.0e-12);
  EXPECT_NEAR(laser_axis.y, 0.0, 1.0e-12);
  EXPECT_NEAR(laser_axis.z, -1.0, 1.0e-12);
  EXPECT_NEAR(optical_axis.x, 1.0, 1.0e-12);
  EXPECT_NEAR(optical_axis.y, 0.0, 1.0e-12);
  EXPECT_NEAR(optical_axis.z, 0.0, 1.0e-12);
}

TEST(WorkspaceCoarseScanCore, EnforcesCalibrationAndSafetyBounds)
{
  ws::WorkspaceOptions options;
  std::string error;
  options.maximum_sample_step_m = 0.021;
  EXPECT_FALSE(ws::validateOptions(options, &error));
  EXPECT_NE(error.find("20 mm"), std::string::npos);
  options.maximum_sample_step_m = 0.02;
  options.reach_fraction = 0.86;
  EXPECT_FALSE(ws::validateOptions(options, &error));
  options.reach_fraction = 0.85;
  options.working_distance_m = 0.71;
  EXPECT_FALSE(ws::validateOptions(options, &error));
}

TEST(WorkspaceCoarseScanCore, SelectsLanesIntersectingLocalRescanRoi)
{
  ws::WorkspaceOptions options;
  options.maximum_measurement_arc_span_deg = 180.0;
  ws::WorkspacePlan plan;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan));
  const ws::Vec3 minimum{0.90, -0.05, 0.25};
  const ws::Vec3 maximum{1.00, 0.05, 0.35};
  EXPECT_TRUE(ws::laneIntersectsAxisAlignedRoi(plan.lanes.front(), minimum, maximum, 0.01));
  EXPECT_FALSE(ws::laneIntersectsAxisAlignedRoi(plan.lanes.back(), minimum, maximum, 0.01));
}

TEST(WorkspaceCoarseScanCore, SplitsBandsWithoutBreakingSnakeDirection)
{
  ws::WorkspaceOptions options;
  ws::WorkspacePlan plan;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan));
  ASSERT_EQ(plan.lanes.size(), 27U);
  EXPECT_EQ(plan.lanes[0].sector_index, 0);
  EXPECT_EQ(plan.lanes[2].sector_index, 2);
  EXPECT_EQ(plan.lanes[3].sector_index, 2);
  EXPECT_EQ(plan.lanes[5].sector_index, 0);
  EXPECT_DOUBLE_EQ(plan.lanes[0].azimuth_start_deg, -60.0);
  EXPECT_DOUBLE_EQ(plan.lanes[2].azimuth_end_deg, 60.0);
  EXPECT_DOUBLE_EQ(plan.lanes[3].azimuth_start_deg, 60.0);
  EXPECT_DOUBLE_EQ(plan.lanes[5].azimuth_end_deg, -60.0);
}

TEST(WorkspaceCoarseScanCore, RotatesRelativeSectorAroundInitialOpticalHeading)
{
  ws::WorkspaceOptions options;
  options.azimuth_center_deg = 30.0;
  ws::WorkspacePlan plan;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan));
  ASSERT_FALSE(plan.lanes.empty());
  EXPECT_DOUBLE_EQ(plan.lanes.front().azimuth_start_deg, -30.0);
  EXPECT_DOUBLE_EQ(plan.lanes[2].azimuth_end_deg, 90.0);
  EXPECT_NEAR(
    plan.lanes.front().measurement_start.position.x,
    options.radial_min_m * std::cos(-30.0 * M_PI / 180.0), 1.0e-12);
  EXPECT_NEAR(
    plan.lanes.front().measurement_start.position.y,
    options.radial_min_m * std::sin(-30.0 * M_PI / 180.0), 1.0e-12);
}

TEST(WorkspaceCoarseScanCore, BuildsOnlyOneHorizontalMotionPlaneWhenHeightsMatch)
{
  ws::WorkspaceOptions options;
  options.height_min_m = 0.50;
  options.height_max_m = 0.50;
  ws::WorkspacePlan plan;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan));
  ASSERT_EQ(plan.height_layers_m.size(), 1U);
  ASSERT_EQ(plan.lanes.size(), 9U);
  for (const auto & lane : plan.lanes) {
    EXPECT_DOUBLE_EQ(lane.height_m, 0.50);
  }
}

TEST(WorkspaceCoarseScanCore, BuildsDownwardTabletopArcFromInitialPose)
{
  ws::WorkspaceOptions options;
  options.tabletop_downward_scan = true;
  options.azimuth_min_deg = -20.0;
  options.azimuth_max_deg = 20.0;
  options.azimuth_center_deg = 0.0;
  options.radial_min_m = 0.55;
  options.radial_max_m = 0.55;
  options.height_min_m = -0.088;
  options.height_max_m = -0.088;
  options.maximum_measurement_arc_span_deg = 45.0;
  // At the sector centre: TCP +X is the radial forward direction, TCP +Y is
  // the opposite circular tangent, and TCP +Z points down at the table.
  options.initial_center_orientation = {1.0, 0.0, 0.0, 0.0};

  ws::WorkspacePlan plan;
  std::string error;
  ASSERT_TRUE(ws::buildLayeredHemisphericalPlan(options, &plan, &error)) << error;
  ASSERT_EQ(plan.lanes.size(), 1U);
  const auto & lane = plan.lanes.front();
  EXPECT_TRUE(lane.preliminary_safe) << lane.rejection_reason;
  EXPECT_DOUBLE_EQ(lane.height_m, -0.088);
  EXPECT_NEAR(lane.measurement_mid.orientation.x, 1.0, 1.0e-12);
  EXPECT_NEAR(lane.measurement_mid.orientation.y, 0.0, 1.0e-12);
  EXPECT_NEAR(lane.measurement_mid.orientation.z, 0.0, 1.0e-12);
  EXPECT_NEAR(lane.measurement_mid.orientation.w, 0.0, 1.0e-12);
  const auto rotate = [](const ws::Quaternion & q, const ws::Vec3 & v) {
      const ws::Vec3 u{q.x, q.y, q.z};
      const ws::Vec3 uv{
        u.y * v.z - u.z * v.y,
        u.z * v.x - u.x * v.z,
        u.x * v.y - u.y * v.x};
      const ws::Vec3 uuv{
        u.y * uv.z - u.z * uv.y,
        u.z * uv.x - u.x * uv.z,
        u.x * uv.y - u.y * uv.x};
      return ws::Vec3{
        v.x + 2.0 * (q.w * uv.x + uuv.x),
        v.y + 2.0 * (q.w * uv.y + uuv.y),
        v.z + 2.0 * (q.w * uv.z + uuv.z)};
    };
  for (const auto * pose : {
      &lane.measurement_start, &lane.measurement_mid, &lane.measurement_end})
  {
    const double azimuth = std::atan2(pose->position.y, pose->position.x);
    const ws::Vec3 scan_axis = rotate(pose->orientation, {1.0, 0.0, 0.0});
    const ws::Vec3 laser_axis = rotate(pose->orientation, {0.0, 1.0, 0.0});
    const ws::Vec3 optical_axis = rotate(pose->orientation, {0.0, 0.0, 1.0});
    EXPECT_NEAR(scan_axis.x, std::cos(azimuth), 1.0e-12);
    EXPECT_NEAR(scan_axis.y, std::sin(azimuth), 1.0e-12);
    EXPECT_NEAR(laser_axis.x, std::sin(azimuth), 1.0e-12);
    EXPECT_NEAR(laser_axis.y, -std::cos(azimuth), 1.0e-12);
    EXPECT_NEAR(optical_axis.z, -1.0, 1.0e-12);
  }
  for (const auto & pose : lane.motion_samples) {
    EXPECT_NEAR(pose.position.z, -0.088, 1.0e-12);
  }
}

TEST(WorkspaceCoarseScanCore, NegativePlaneRequiresTabletopMode)
{
  ws::WorkspaceOptions options;
  options.height_min_m = -0.10;
  options.height_max_m = -0.10;
  std::string error;
  EXPECT_FALSE(ws::validateOptions(options, &error));
  options.tabletop_downward_scan = true;
  EXPECT_TRUE(ws::validateOptions(options, &error)) << error;
}

TEST(WorkspaceCoarseScanCore, BuildsAlternatingTabletopRadialSnake)
{
  ws::WorkspaceOptions options;
  options.tabletop_downward_scan = true;
  options.tabletop_radial_fan_scan = true;
  options.azimuth_min_deg = -60.0;
  options.azimuth_max_deg = 60.0;
  options.azimuth_center_deg = 0.0;
  options.radial_min_m = 0.45;
  options.radial_max_m = 0.72;
  options.angular_lane_spacing_m = 0.22;
  options.minimum_radial_scan_length_m = 0.10;
  options.height_min_m = -0.088;
  options.height_max_m = -0.088;
  options.initial_center_orientation = {1.0, 0.0, 0.0, 0.0};
  const std::vector<double> safe_outer_limits{0.65, 0.67, 0.65};

  ws::WorkspacePlan plan;
  std::string error;
  ASSERT_TRUE(ws::buildTabletopRadialFanPlan(
      options, safe_outer_limits, &plan, &error)) << error;
  ASSERT_EQ(plan.lanes.size(), 3U);
  EXPECT_NEAR(plan.lanes.front().azimuth_start_deg, -60.0, 1.0e-12);
  EXPECT_NEAR(plan.lanes[1].azimuth_start_deg, 0.0, 1.0e-12);
  EXPECT_NEAR(plan.lanes.back().azimuth_start_deg, 60.0, 1.0e-12);
  for (std::size_t index = 0U; index < plan.lanes.size(); ++index) {
    const auto & lane = plan.lanes[index];
    EXPECT_TRUE(lane.radial_scan);
    EXPECT_EQ(lane.reverse, index % 2U != 0U);
    EXPECT_TRUE(lane.preliminary_safe) << lane.rejection_reason;
    EXPECT_NEAR(lane.azimuth_start_deg, lane.azimuth_end_deg, 1.0e-12);
    EXPECT_NEAR(lane.measurement_length_m, safe_outer_limits[index] - 0.45, 1.0e-12);
    EXPECT_NEAR(lane.requested_measurement_length_m, 0.27, 1.0e-12);
    if (lane.reverse) {
      EXPECT_NEAR(lane.radial_start_m, safe_outer_limits[index], 1.0e-12);
      EXPECT_NEAR(lane.radial_end_m, 0.45, 1.0e-12);
    } else {
      EXPECT_NEAR(lane.radial_start_m, 0.45, 1.0e-12);
      EXPECT_NEAR(lane.radial_end_m, safe_outer_limits[index], 1.0e-12);
    }
    for (const auto & pose : lane.measurement_samples) {
      EXPECT_NEAR(pose.position.z, -0.088, 1.0e-12);
      EXPECT_NEAR(
        std::atan2(pose.position.y, pose.position.x) * 180.0 / M_PI,
        lane.azimuth_start_deg, 1.0e-10);
    }
  }
}

TEST(WorkspaceCoarseScanCore, RejectsRadialLaneWithoutSafeOuterInterval)
{
  ws::WorkspaceOptions options;
  options.tabletop_downward_scan = true;
  options.tabletop_radial_fan_scan = true;
  options.azimuth_min_deg = -10.0;
  options.azimuth_max_deg = 10.0;
  options.radial_min_m = 0.45;
  options.radial_max_m = 0.65;
  options.angular_lane_spacing_m = 0.25;
  options.height_min_m = -0.088;
  options.height_max_m = -0.088;
  options.initial_center_orientation = {1.0, 0.0, 0.0, 0.0};

  ws::WorkspacePlan plan;
  std::string error;
  ASSERT_TRUE(ws::buildTabletopRadialFanPlan(
      options,
      {std::numeric_limits<double>::quiet_NaN(), 0.65, 0.65},
      &plan, &error)) << error;
  ASSERT_EQ(plan.lanes.size(), 3U);
  EXPECT_FALSE(plan.lanes[0].preliminary_safe);
  EXPECT_NE(plan.lanes[0].rejection_reason.find("no contiguous IK-safe"), std::string::npos);
  EXPECT_TRUE(plan.lanes[1].preliminary_safe) << plan.lanes[1].rejection_reason;
  EXPECT_TRUE(plan.lanes[2].preliminary_safe) << plan.lanes[2].rejection_reason;
}

TEST(WorkspaceCoarseScanCore, BuildsFanFromTaughtInitialTcpToSafeOuterLimits)
{
  ws::WorkspaceOptions options;
  options.tabletop_downward_scan = true;
  options.tabletop_radial_fan_scan = true;
  options.fan_origin_at_initial_tcp = true;
  options.initial_tcp_fan_origin = {0.20, 0.0, -0.088};
  options.initial_center_orientation = {1.0, 0.0, 0.0, 0.0};
  options.azimuth_min_deg = -60.0;
  options.azimuth_max_deg = 60.0;
  options.azimuth_center_deg = 0.0;
  options.radial_min_m = 0.0;
  options.radial_max_m = 0.72;
  options.angular_lane_spacing_m = 0.22;
  options.height_min_m = -0.088;
  options.height_max_m = -0.088;
  const std::vector<double> safe_outer_limits{0.45, 0.47, 0.45};

  ws::WorkspacePlan plan;
  std::string error;
  ASSERT_TRUE(ws::buildTabletopRadialFanPlan(
      options, safe_outer_limits, &plan, &error)) << error;
  ASSERT_EQ(plan.lanes.size(), 3U);
  for (std::size_t index = 0U; index < plan.lanes.size(); ++index) {
    const auto & lane = plan.lanes[index];
    const ws::Pose & apex = lane.reverse ? lane.measurement_end : lane.measurement_start;
    const ws::Pose & outer = lane.reverse ? lane.measurement_start : lane.measurement_end;
    EXPECT_TRUE(lane.preliminary_safe) << lane.rejection_reason;
    EXPECT_NEAR(apex.position.x, options.initial_tcp_fan_origin.x, 1.0e-12);
    EXPECT_NEAR(apex.position.y, options.initial_tcp_fan_origin.y, 1.0e-12);
    EXPECT_NEAR(apex.position.z, options.initial_tcp_fan_origin.z, 1.0e-12);
    const double angle = lane.azimuth_start_deg * M_PI / 180.0;
    EXPECT_NEAR(
      outer.position.x,
      options.initial_tcp_fan_origin.x + safe_outer_limits[index] * std::cos(angle),
      1.0e-12);
    EXPECT_NEAR(
      outer.position.y,
      options.initial_tcp_fan_origin.y + safe_outer_limits[index] * std::sin(angle),
      1.0e-12);
    EXPECT_NEAR(lane.measurement_length_m, safe_outer_limits[index], 1.0e-12);
    EXPECT_NEAR(lane.requested_measurement_length_m, 0.72, 1.0e-12);
    if (lane.reverse) {
      EXPECT_NEAR(lane.motion_end.position.x, apex.position.x, 1.0e-12);
      EXPECT_NEAR(lane.motion_end.position.y, apex.position.y, 1.0e-12);
    } else {
      EXPECT_NEAR(lane.motion_start.position.x, apex.position.x, 1.0e-12);
      EXPECT_NEAR(lane.motion_start.position.y, apex.position.y, 1.0e-12);
    }
  }
}

TEST(WorkspaceCoarseScanCore, AlwaysKeepsTcpXCenterWithAtMostThreeFanLanes)
{
  ws::WorkspaceOptions options;

  options.azimuth_min_deg = -90.0;
  options.azimuth_max_deg = 90.0;
  const std::vector<double> full = ws::tabletopRadialAzimuths(options);
  ASSERT_EQ(full.size(), 3U);
  EXPECT_DOUBLE_EQ(full[0], -90.0);
  EXPECT_DOUBLE_EQ(full[1], 0.0);
  EXPECT_DOUBLE_EQ(full[2], 90.0);

  options.azimuth_min_deg = -60.0;
  options.azimuth_max_deg = 60.0;
  const std::vector<double> medium = ws::tabletopRadialAzimuths(options);
  ASSERT_EQ(medium.size(), 3U);
  EXPECT_DOUBLE_EQ(medium[0], -60.0);
  EXPECT_DOUBLE_EQ(medium[1], 0.0);
  EXPECT_DOUBLE_EQ(medium[2], 60.0);

  options.azimuth_min_deg = -45.0;
  options.azimuth_max_deg = 45.0;
  const std::vector<double> narrow = ws::tabletopRadialAzimuths(options);
  ASSERT_EQ(narrow.size(), 3U);
  EXPECT_DOUBLE_EQ(narrow[0], -45.0);
  EXPECT_DOUBLE_EQ(narrow[1], 0.0);
  EXPECT_DOUBLE_EQ(narrow[2], 45.0);
}

TEST(WorkspaceCoarseScanCore, RejectsRadialFanThatOmitsTcpXCenter)
{
  ws::WorkspaceOptions options;
  options.tabletop_downward_scan = true;
  options.tabletop_radial_fan_scan = true;
  options.azimuth_min_deg = 10.0;
  options.azimuth_max_deg = 45.0;
  std::string error;
  EXPECT_FALSE(ws::validateOptions(options, &error));
  EXPECT_NE(error.find("relative 0 degrees"), std::string::npos);
}

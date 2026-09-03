#ifndef FR5_SCANNER_650__WORKSPACE_COARSE_SCAN_CORE_HPP_
#define FR5_SCANNER_650__WORKSPACE_COARSE_SCAN_CORE_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace fr5_scanner_650::workspace_scan
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Vec3 position;
  Quaternion orientation;
};

struct WorkspaceOptions
{
  // Relative offsets from the heading selected by the planner. For a
  // vertical-front scan this is TCP +Z (optical axis). For a tabletop scan it
  // is TCP +X, the calibrated nominal scan/forward direction.
  double azimuth_min_deg{-60.0};
  double azimuth_max_deg{60.0};
  double azimuth_center_deg{0.0};
  double radial_min_m{0.95};
  double radial_max_m{1.20};
  double radial_spacing_m{0.125};
  double height_min_m{0.25};
  double height_max_m{0.70};
  double height_spacing_m{0.225};
  double lead_in_m{0.04};
  double lead_out_m{0.04};
  double maximum_sample_step_m{0.02};
  double maximum_measurement_arc_span_deg{45.0};
  double optical_roll_deg{180.0};
  double working_distance_m{0.55};
  double camera_to_tcp_tangent_m{0.0466476147};
  double base_exclusion_radius_m{0.35};
  double maximum_arm_reach_m{0.922};
  double reach_fraction{0.85};
  double shoulder_height_m{0.152};
  double nominal_laser_line_width_m{0.45};
  // A tabletop radial fan contains only its two requested limits and the
  // TCP +X centre direction. The relative sector must contain zero, so the
  // generated fan always has at most three lanes.
  // Retained for compatibility with older base-centred fan configurations.
  double angular_lane_spacing_m{0.22};
  double minimum_radial_scan_length_m{0.10};
  // Tabletop paths keep the taught downward pose and rotate it about base Z
  // by the same azimuth delta as the process TCP. This preserves pitch/roll
  // and keeps the optical axis pointed at the horizontal work surface.
  bool tabletop_downward_scan{false};
  bool tabletop_radial_fan_scan{false};
  // In the operator-taught fan mode, radial values are travel distances from
  // the planning-time TCP position instead of radii around base_link.
  bool fan_origin_at_initial_tcp{false};
  Vec3 initial_tcp_fan_origin;
  Quaternion initial_center_orientation;
  std::size_t maximum_lane_count{128U};
  std::size_t maximum_samples_per_lane{4096U};
};

struct ScanLane
{
  int id{-1};
  int radial_index{-1};
  int height_index{-1};
  int sector_index{-1};
  double radius_m{0.0};
  double radial_start_m{0.0};
  double radial_end_m{0.0};
  double height_m{0.0};
  double azimuth_start_deg{0.0};
  double azimuth_end_deg{0.0};
  bool reverse{false};
  bool radial_scan{false};
  Pose motion_start;
  Pose measurement_start;
  Pose measurement_mid;
  Pose measurement_end;
  Pose motion_end;
  std::vector<Pose> measurement_samples;
  std::vector<Pose> motion_samples;
  double measurement_length_m{0.0};
  double requested_measurement_length_m{0.0};
  double motion_length_m{0.0};
  double minimum_estimated_head_radius_m{0.0};
  double maximum_estimated_reach_m{0.0};
  bool preliminary_safe{false};
  std::string rejection_reason;
};

struct WorkspacePlan
{
  WorkspaceOptions options;
  std::vector<double> radial_layers_m;
  std::vector<double> height_layers_m;
  std::vector<ScanLane> lanes;
  double total_measurement_length_m{0.0};
  double nominal_surface_area_m2{0.0};
};

bool validateOptions(const WorkspaceOptions & options, std::string * error = nullptr);

bool buildLayeredHemisphericalPlan(
  const WorkspaceOptions & options, WorkspacePlan * plan,
  std::string * error = nullptr);

std::vector<double> tabletopRadialAzimuths(const WorkspaceOptions & options);

Pose tabletopRadialPose(
  const WorkspaceOptions & options, double radius_m, double height_m,
  double azimuth_deg);

bool buildTabletopRadialFanPlan(
  const WorkspaceOptions & options,
  const std::vector<double> & safe_outer_limits_m,
  WorkspacePlan * plan, std::string * error = nullptr);

std::vector<Pose> sampleLinearPoses(
  const Pose & start, const Pose & end, double maximum_step_m,
  double maximum_angular_step_deg, std::size_t maximum_samples,
  std::string * error = nullptr);

bool laneIntersectsAxisAlignedRoi(
  const ScanLane & lane, const Vec3 & minimum, const Vec3 & maximum,
  double padding_m);

double positionDistance(const Pose & first, const Pose & second);
double orientationDistanceDeg(const Pose & first, const Pose & second);

}  // namespace fr5_scanner_650::workspace_scan

#endif  // FR5_SCANNER_650__WORKSPACE_COARSE_SCAN_CORE_HPP_

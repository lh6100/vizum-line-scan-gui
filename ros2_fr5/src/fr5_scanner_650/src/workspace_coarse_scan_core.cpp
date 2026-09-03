#include "workspace_coarse_scan_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace fr5_scanner_650::workspace_scan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

bool finite(double value)
{
  return std::isfinite(value);
}

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

Quaternion normalized(Quaternion value)
{
  const double norm = std::sqrt(
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return {};
  }
  value.x /= norm;
  value.y /= norm;
  value.z /= norm;
  value.w /= norm;
  return value;
}

double dot(const Quaternion & first, const Quaternion & second)
{
  return first.x * second.x + first.y * second.y + first.z * second.z +
         first.w * second.w;
}

Quaternion negate(const Quaternion & value)
{
  return {-value.x, -value.y, -value.z, -value.w};
}

Quaternion multiply(const Quaternion & first, const Quaternion & second)
{
  return normalized({
    first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y,
    first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x,
    first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w,
    first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z});
}

Quaternion slerp(Quaternion first, Quaternion second, double fraction)
{
  first = normalized(first);
  second = normalized(second);
  double cosine = dot(first, second);
  if (cosine < 0.0) {
    second = negate(second);
    cosine = -cosine;
  }
  if (cosine > 0.9995) {
    return normalized({
      first.x + fraction * (second.x - first.x),
      first.y + fraction * (second.y - first.y),
      first.z + fraction * (second.z - first.z),
      first.w + fraction * (second.w - first.w)});
  }
  const double angle = std::acos(clamp(cosine, -1.0, 1.0));
  const double sine = std::sin(angle);
  const double first_weight = std::sin((1.0 - fraction) * angle) / sine;
  const double second_weight = std::sin(fraction * angle) / sine;
  return normalized({
    first_weight * first.x + second_weight * second.x,
    first_weight * first.y + second_weight * second.y,
    first_weight * first.z + second_weight * second.z,
    first_weight * first.w + second_weight * second.w});
}

Quaternion quaternionFromBasis(const Vec3 & x_axis, const Vec3 & y_axis, const Vec3 & z_axis)
{
  // Rotation-matrix columns are the TCP axes expressed in base_link.
  const double r00 = x_axis.x;
  const double r01 = y_axis.x;
  const double r02 = z_axis.x;
  const double r10 = x_axis.y;
  const double r11 = y_axis.y;
  const double r12 = z_axis.y;
  const double r20 = x_axis.z;
  const double r21 = y_axis.z;
  const double r22 = z_axis.z;
  Quaternion quaternion;
  const double trace = r00 + r11 + r22;
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * scale;
    quaternion.x = (r21 - r12) / scale;
    quaternion.y = (r02 - r20) / scale;
    quaternion.z = (r10 - r01) / scale;
  } else if (r00 > r11 && r00 > r22) {
    const double scale = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
    quaternion.w = (r21 - r12) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (r01 + r10) / scale;
    quaternion.z = (r02 + r20) / scale;
  } else if (r11 > r22) {
    const double scale = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
    quaternion.w = (r02 - r20) / scale;
    quaternion.x = (r01 + r10) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (r12 + r21) / scale;
  } else {
    const double scale = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
    quaternion.w = (r10 - r01) / scale;
    quaternion.x = (r02 + r20) / scale;
    quaternion.y = (r12 + r21) / scale;
    quaternion.z = 0.25 * scale;
  }
  return normalized(quaternion);
}

Pose arcPose(
  const WorkspaceOptions & options, double radius_m, double height_m, double azimuth_deg)
{
  const double angle = radians(azimuth_deg);
  const Vec3 radial{std::cos(angle), std::sin(angle), 0.0};
  const Vec3 tangent{-std::sin(angle), std::cos(angle), 0.0};
  const Vec3 vertical{0.0, 0.0, 1.0};
  const double roll = radians(options.optical_roll_deg);
  const Vec3 rolled_x{
    tangent.x * std::cos(roll) + vertical.x * std::sin(roll),
    tangent.y * std::cos(roll) + vertical.y * std::sin(roll),
    tangent.z * std::cos(roll) + vertical.z * std::sin(roll)};
  const Vec3 rolled_y{
    -tangent.x * std::sin(roll) + vertical.x * std::cos(roll),
    -tangent.y * std::sin(roll) + vertical.y * std::cos(roll),
    -tangent.z * std::sin(roll) + vertical.z * std::cos(roll)};
  Pose pose;
  pose.position = {radius_m * radial.x, radius_m * radial.y, height_m};
  if (options.tabletop_downward_scan) {
    // Rotate the complete operator-taught pose around base Z. At the sector
    // centre this is exactly the initial pose; pitch and roll do not change.
    const double half_delta = 0.5 * radians(azimuth_deg - options.azimuth_center_deg);
    const Quaternion yaw{0.0, 0.0, std::sin(half_delta), std::cos(half_delta)};
    pose.orientation = multiply(yaw, options.initial_center_orientation);
  } else {
    // TCP +/-Y follows the laser line vertically and +Z points from camera
    // towards the nominal surface. The configurable 0/180-degree optical
    // roll selects the collision-safer equivalent wrist posture.
    pose.orientation = quaternionFromBasis(rolled_x, rolled_y, radial);
  }
  return pose;
}

Pose radialFanPose(
  const WorkspaceOptions & options, double radial_value_m, double height_m,
  double azimuth_deg)
{
  Pose pose = arcPose(options, radial_value_m, height_m, azimuth_deg);
  if (options.fan_origin_at_initial_tcp) {
    const double angle = radians(azimuth_deg);
    pose.position = {
      options.initial_tcp_fan_origin.x + radial_value_m * std::cos(angle),
      options.initial_tcp_fan_origin.y + radial_value_m * std::sin(angle),
      height_m};
  }
  return pose;
}

Vec3 add(const Vec3 & first, const Vec3 & second)
{
  return {first.x + second.x, first.y + second.y, first.z + second.z};
}

Vec3 subtract(const Vec3 & first, const Vec3 & second)
{
  return {first.x - second.x, first.y - second.y, first.z - second.z};
}

Vec3 scale(const Vec3 & value, double factor)
{
  return {value.x * factor, value.y * factor, value.z * factor};
}

Vec3 rotate(const Quaternion & raw_quaternion, const Vec3 & vector)
{
  const Quaternion quaternion = normalized(raw_quaternion);
  const Vec3 quaternion_vector{quaternion.x, quaternion.y, quaternion.z};
  const Vec3 first_cross{
    quaternion_vector.y * vector.z - quaternion_vector.z * vector.y,
    quaternion_vector.z * vector.x - quaternion_vector.x * vector.z,
    quaternion_vector.x * vector.y - quaternion_vector.y * vector.x};
  const Vec3 second_cross{
    quaternion_vector.y * first_cross.z - quaternion_vector.z * first_cross.y,
    quaternion_vector.z * first_cross.x - quaternion_vector.x * first_cross.z,
    quaternion_vector.x * first_cross.y - quaternion_vector.y * first_cross.x};
  return add(vector, add(scale(first_cross, 2.0 * quaternion.w), scale(second_cross, 2.0)));
}

Pose leadPose(
  const Pose & endpoint, double azimuth_deg, double signed_distance_m,
  bool tabletop_downward_scan)
{
  Pose pose = endpoint;
  const double angle = radians(azimuth_deg);
  const Vec3 tangent = tabletop_downward_scan ?
    Vec3{-std::sin(angle), std::cos(angle), 0.0} :
    rotate(endpoint.orientation, {1.0, 0.0, 0.0});
  pose.position = add(endpoint.position, scale(tangent, signed_distance_m));
  return pose;
}

std::vector<double> layers(double minimum, double maximum, double spacing)
{
  const std::size_t intervals = static_cast<std::size_t>(
    std::ceil((maximum - minimum) / spacing - 1.0e-12));
  std::vector<double> result;
  result.reserve(intervals + 1U);
  for (std::size_t index = 0U; index <= intervals; ++index) {
    result.push_back(index == intervals ? maximum : minimum + static_cast<double>(index) * spacing);
  }
  return result;
}

void appendWithoutDuplicate(std::vector<Pose> * destination, const std::vector<Pose> & source)
{
  if (!destination || source.empty()) {
    return;
  }
  destination->insert(
    destination->end(), source.begin() + (destination->empty() ? 0 : 1), source.end());
}

std::vector<Pose> sampleArc(
  const WorkspaceOptions & options, double radius_m, double height_m,
  double start_deg, double end_deg, double maximum_step_m,
  std::size_t maximum_samples)
{
  const double length = radius_m * std::fabs(radians(end_deg - start_deg));
  const std::size_t intervals = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(length / maximum_step_m)));
  if (intervals + 1U > maximum_samples) {
    return {};
  }
  std::vector<Pose> poses;
  poses.reserve(intervals + 1U);
  Quaternion previous;
  bool have_previous = false;
  for (std::size_t index = 0U; index <= intervals; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(intervals);
    Pose pose = arcPose(
      options, radius_m, height_m, start_deg + fraction * (end_deg - start_deg));
    if (have_previous && dot(previous, pose.orientation) < 0.0) {
      pose.orientation = negate(pose.orientation);
    }
    previous = pose.orientation;
    have_previous = true;
    poses.push_back(pose);
  }
  return poses;
}

bool poseInBox(const Pose & pose, const Vec3 & minimum, const Vec3 & maximum, double padding)
{
  return pose.position.x >= minimum.x - padding && pose.position.x <= maximum.x + padding &&
         pose.position.y >= minimum.y - padding && pose.position.y <= maximum.y + padding &&
         pose.position.z >= minimum.z - padding && pose.position.z <= maximum.z + padding;
}

bool preliminaryPoseSafe(
  const WorkspaceOptions & options, const Pose & pose,
  double * head_radius_m = nullptr, double * estimated_reach_m = nullptr)
{
  const Vec3 tcp_x = rotate(pose.orientation, {1.0, 0.0, 0.0});
  const Vec3 tcp_z = rotate(pose.orientation, {0.0, 0.0, 1.0});
  const Vec3 estimated_head = subtract(
    subtract(pose.position, scale(tcp_x, options.camera_to_tcp_tangent_m)),
    scale(tcp_z, options.working_distance_m));
  const double head_radius = std::hypot(estimated_head.x, estimated_head.y);
  const double reach = std::hypot(
    head_radius, estimated_head.z - options.shoulder_height_m);
  double minimum_head_radius = options.base_exclusion_radius_m;
  if (options.fan_origin_at_initial_tcp) {
    // The operator-taught TCP is already a real robot pose. A base-centred
    // cylindrical exclusion would reject that starting pose even when it is
    // valid at its actual height. Grandfather only the closest scanner-head
    // radius needed by the planned in-place fan orientations; exact MoveIt
    // self/environment collision checks remain mandatory for every sample.
    const double span = options.azimuth_max_deg - options.azimuth_min_deg;
    const std::size_t intervals = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(span / 5.0)));
    for (std::size_t index = 0U; index <= intervals; ++index) {
      const double azimuth = options.azimuth_center_deg + options.azimuth_min_deg +
        span * static_cast<double>(index) / static_cast<double>(intervals);
      const Pose origin_pose = radialFanPose(options, 0.0, options.height_min_m, azimuth);
      const Vec3 origin_tcp_x = rotate(origin_pose.orientation, {1.0, 0.0, 0.0});
      const Vec3 origin_tcp_z = rotate(origin_pose.orientation, {0.0, 0.0, 1.0});
      const Vec3 origin_head = subtract(
        subtract(origin_pose.position, scale(origin_tcp_x, options.camera_to_tcp_tangent_m)),
        scale(origin_tcp_z, options.working_distance_m));
      minimum_head_radius = std::min(
        minimum_head_radius, std::max(0.0, std::hypot(origin_head.x, origin_head.y) - 0.01));
    }
  }
  if (head_radius_m) {*head_radius_m = head_radius;}
  if (estimated_reach_m) {*estimated_reach_m = reach;}
  return head_radius >= minimum_head_radius &&
         reach <= options.maximum_arm_reach_m * options.reach_fraction;
}

void evaluatePreliminarySafety(const WorkspaceOptions & options, ScanLane * lane)
{
  lane->minimum_estimated_head_radius_m = std::numeric_limits<double>::infinity();
  lane->maximum_estimated_reach_m = 0.0;
  const double reach_limit = options.maximum_arm_reach_m * options.reach_fraction;
  lane->preliminary_safe = true;
  for (const Pose & pose : lane->motion_samples) {
    double head_radius = 0.0;
    double estimated_reach = 0.0;
    const bool safe = preliminaryPoseSafe(
      options, pose, &head_radius, &estimated_reach);
    lane->minimum_estimated_head_radius_m = std::min(
      lane->minimum_estimated_head_radius_m, head_radius);
    lane->maximum_estimated_reach_m = std::max(
      lane->maximum_estimated_reach_m, estimated_reach);
    if (!safe && estimated_reach <= reach_limit) {
      lane->preliminary_safe = false;
      lane->rejection_reason = "estimated scanner head enters base exclusion radius";
      return;
    }
    if (!safe && estimated_reach > reach_limit) {
      lane->preliminary_safe = false;
      lane->rejection_reason = "estimated scanner head exceeds safe reach fraction";
      return;
    }
  }
}

}  // namespace

double positionDistance(const Pose & first, const Pose & second)
{
  const Vec3 delta = subtract(first.position, second.position);
  return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
}

double orientationDistanceDeg(const Pose & first, const Pose & second)
{
  const double cosine = std::fabs(dot(normalized(first.orientation), normalized(second.orientation)));
  return 2.0 * std::acos(clamp(cosine, -1.0, 1.0)) * 180.0 / kPi;
}

std::vector<Pose> sampleLinearPoses(
  const Pose & start, const Pose & end, double maximum_step_m,
  double maximum_angular_step_deg, std::size_t maximum_samples,
  std::string * error)
{
  if (!(finite(maximum_step_m) && maximum_step_m > 0.0 &&
    finite(maximum_angular_step_deg) && maximum_angular_step_deg > 0.0 &&
    maximum_samples >= 2U))
  {
    if (error) {*error = "invalid linear pose sampling options";}
    return {};
  }
  const std::size_t translation_intervals = static_cast<std::size_t>(
    std::ceil(positionDistance(start, end) / maximum_step_m));
  const std::size_t angular_intervals = static_cast<std::size_t>(
    std::ceil(orientationDistanceDeg(start, end) / maximum_angular_step_deg));
  const std::size_t intervals = std::max<std::size_t>(
    1U, std::max(translation_intervals, angular_intervals));
  if (intervals + 1U > maximum_samples) {
    if (error) {*error = "linear pose path exceeds maximum sample count";}
    return {};
  }
  std::vector<Pose> result;
  result.reserve(intervals + 1U);
  for (std::size_t index = 0U; index <= intervals; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(intervals);
    Pose pose;
    pose.position = add(start.position, scale(subtract(end.position, start.position), fraction));
    pose.orientation = slerp(start.orientation, end.orientation, fraction);
    result.push_back(pose);
  }
  return result;
}

bool validateOptions(const WorkspaceOptions & options, std::string * error)
{
  const bool finite_values =
    finite(options.azimuth_min_deg) && finite(options.azimuth_max_deg) &&
    finite(options.azimuth_center_deg) &&
    finite(options.radial_min_m) && finite(options.radial_max_m) &&
    finite(options.radial_spacing_m) && finite(options.height_min_m) &&
    finite(options.height_max_m) && finite(options.height_spacing_m) &&
    finite(options.lead_in_m) && finite(options.lead_out_m) &&
    finite(options.maximum_sample_step_m) && finite(options.maximum_measurement_arc_span_deg) &&
    finite(options.optical_roll_deg) && finite(options.working_distance_m) &&
    finite(options.camera_to_tcp_tangent_m) && finite(options.base_exclusion_radius_m) &&
    finite(options.maximum_arm_reach_m) && finite(options.reach_fraction) &&
    finite(options.shoulder_height_m) && finite(options.nominal_laser_line_width_m) &&
    finite(options.angular_lane_spacing_m) &&
    finite(options.minimum_radial_scan_length_m) &&
    finite(options.initial_tcp_fan_origin.x) &&
    finite(options.initial_tcp_fan_origin.y) &&
    finite(options.initial_tcp_fan_origin.z) &&
    finite(options.initial_center_orientation.x) &&
    finite(options.initial_center_orientation.y) &&
    finite(options.initial_center_orientation.z) &&
    finite(options.initial_center_orientation.w);
  if (!finite_values) {
    if (error) {*error = "workspace parameters must be finite";}
    return false;
  }
  if (options.azimuth_min_deg < -90.0 || options.azimuth_max_deg > 90.0 ||
    options.azimuth_max_deg - options.azimuth_min_deg < 1.0)
  {
    if (error) {*error = "relative azimuth must be an ordered subset of -90..+90 degrees";}
    return false;
  }
  if (options.azimuth_center_deg < -180.0 || options.azimuth_center_deg > 180.0) {
    if (error) {*error = "initial sector azimuth center must be within -180..+180 degrees";}
    return false;
  }
  if (options.tabletop_radial_fan_scan &&
    (options.azimuth_min_deg > 0.0 || options.azimuth_max_deg < 0.0))
  {
    if (error) {*error = "tabletop radial fan azimuth must contain TCP +X relative 0 degrees";}
    return false;
  }
  const bool invalid_radial_minimum = options.fan_origin_at_initial_tcp ?
    options.radial_min_m < 0.0 : options.radial_min_m < 0.1;
  if (invalid_radial_minimum || options.radial_max_m < options.radial_min_m ||
    options.radial_spacing_m <= 0.0 ||
    (!options.tabletop_downward_scan && options.height_min_m < 0.0) ||
    options.height_max_m < options.height_min_m || options.height_spacing_m <= 0.0)
  {
    if (error) {*error = "invalid radial or height layer range";}
    return false;
  }
  const double initial_orientation_norm = std::sqrt(
    options.initial_center_orientation.x * options.initial_center_orientation.x +
    options.initial_center_orientation.y * options.initial_center_orientation.y +
    options.initial_center_orientation.z * options.initial_center_orientation.z +
    options.initial_center_orientation.w * options.initial_center_orientation.w);
  if (options.tabletop_downward_scan && initial_orientation_norm < 1.0e-6) {
    if (error) {*error = "tabletop scan initial orientation is invalid";}
    return false;
  }
  if (options.lead_in_m < 0.03 || options.lead_in_m > 0.05 ||
    options.lead_out_m < 0.03 || options.lead_out_m > 0.05)
  {
    if (error) {*error = "lead-in and lead-out must remain within 30..50 mm";}
    return false;
  }
  if (options.maximum_sample_step_m <= 0.0 || options.maximum_sample_step_m > 0.02 ||
    options.working_distance_m < 0.40 || options.working_distance_m > 0.70)
  {
    if (error) {*error = "sample step must be <=20 mm and working distance must be 400..700 mm";}
    return false;
  }
  if (options.maximum_measurement_arc_span_deg < 5.0 ||
    options.maximum_measurement_arc_span_deg > 180.0 ||
    (std::fabs(options.optical_roll_deg) > 1.0e-9 &&
    std::fabs(std::fabs(options.optical_roll_deg) - 180.0) > 1.0e-9))
  {
    if (error) {*error = "arc span must be 5..180 degrees and optical roll must be 0 or 180 degrees";}
    return false;
  }
  if (options.base_exclusion_radius_m < 0.30 || options.base_exclusion_radius_m > 0.40 ||
    options.maximum_arm_reach_m <= 0.0 || options.reach_fraction < 0.5 ||
    options.reach_fraction > 0.85)
  {
    if (error) {*error = "safe reach requires a 300..400 mm base exclusion and <=85% reach fraction";}
    return false;
  }
  if (options.nominal_laser_line_width_m <= 0.0 || options.maximum_lane_count == 0U ||
    options.maximum_samples_per_lane < 4U)
  {
    if (error) {*error = "invalid coverage or resource limits";}
    return false;
  }
  if (options.angular_lane_spacing_m < 0.05 || options.angular_lane_spacing_m > 0.45 ||
    options.minimum_radial_scan_length_m < 0.05 ||
    options.minimum_radial_scan_length_m > 0.50)
  {
    if (error) {
      *error = "legacy spacing must be 50..450 mm and scan length >=50 mm";
    }
    return false;
  }
  return true;
}

bool buildLayeredHemisphericalPlan(
  const WorkspaceOptions & options, WorkspacePlan * plan, std::string * error)
{
  if (!plan) {
    if (error) {*error = "workspace plan output is null";}
    return false;
  }
  std::string validation_error;
  if (!validateOptions(options, &validation_error)) {
    if (error) {*error = validation_error;}
    return false;
  }
  WorkspacePlan result;
  result.options = options;
  result.radial_layers_m = layers(
    options.radial_min_m, options.radial_max_m, options.radial_spacing_m);
  result.height_layers_m = layers(
    options.height_min_m, options.height_max_m, options.height_spacing_m);
  const std::size_t sector_count = static_cast<std::size_t>(std::ceil(
      (options.azimuth_max_deg - options.azimuth_min_deg) /
      options.maximum_measurement_arc_span_deg));
  if (result.radial_layers_m.size() * result.height_layers_m.size() * sector_count >
    options.maximum_lane_count)
  {
    if (error) {*error = "workspace grid exceeds maximum lane count";}
    return false;
  }

  int lane_id = 0;
  int band_id = 0;
  for (std::size_t radial_index = 0U; radial_index < result.radial_layers_m.size(); ++radial_index) {
    for (std::size_t height_order = 0U; height_order < result.height_layers_m.size(); ++height_order) {
      const std::size_t height_index = radial_index % 2U == 0U ? height_order :
        result.height_layers_m.size() - 1U - height_order;
      const bool reverse = band_id++ % 2 != 0;
      const double total_span = options.azimuth_max_deg - options.azimuth_min_deg;
      for (std::size_t sector_order = 0U; sector_order < sector_count; ++sector_order) {
        const std::size_t sector_index = reverse ? sector_count - 1U - sector_order : sector_order;
        const double low_relative = options.azimuth_min_deg +
          total_span * static_cast<double>(sector_index) / static_cast<double>(sector_count);
        const double high_relative = options.azimuth_min_deg +
          total_span * static_cast<double>(sector_index + 1U) / static_cast<double>(sector_count);
        const double low = options.azimuth_center_deg + low_relative;
        const double high = options.azimuth_center_deg + high_relative;
        ScanLane lane;
        lane.id = lane_id++;
        lane.radial_index = static_cast<int>(radial_index);
        lane.height_index = static_cast<int>(height_index);
        lane.sector_index = static_cast<int>(sector_index);
        lane.radius_m = result.radial_layers_m[radial_index];
        lane.radial_start_m = lane.radius_m;
        lane.radial_end_m = lane.radius_m;
        lane.height_m = result.height_layers_m[height_index];
        lane.reverse = reverse;
        lane.azimuth_start_deg = reverse ? high : low;
        lane.azimuth_end_deg = reverse ? low : high;
        lane.measurement_start = arcPose(
          options, lane.radius_m, lane.height_m, lane.azimuth_start_deg);
        lane.measurement_mid = arcPose(
          options, lane.radius_m, lane.height_m,
          0.5 * (lane.azimuth_start_deg + lane.azimuth_end_deg));
        lane.measurement_end = arcPose(
          options, lane.radius_m, lane.height_m, lane.azimuth_end_deg);
        const double direction_sign = lane.reverse ? -1.0 : 1.0;
        lane.motion_start = leadPose(
          lane.measurement_start, lane.azimuth_start_deg,
          -direction_sign * options.lead_in_m, options.tabletop_downward_scan);
        lane.motion_end = leadPose(
          lane.measurement_end, lane.azimuth_end_deg,
          direction_sign * options.lead_out_m, options.tabletop_downward_scan);
        lane.measurement_length_m = lane.radius_m *
          std::fabs(radians(lane.azimuth_end_deg - lane.azimuth_start_deg));
        lane.requested_measurement_length_m = lane.measurement_length_m;
        lane.motion_length_m = lane.measurement_length_m + options.lead_in_m + options.lead_out_m;
        lane.measurement_samples = sampleArc(
          options, lane.radius_m, lane.height_m,
          lane.azimuth_start_deg, lane.azimuth_end_deg, options.maximum_sample_step_m,
          options.maximum_samples_per_lane);
        if (lane.measurement_samples.empty()) {
          if (error) {*error = "arc path exceeds maximum samples per lane";}
          return false;
        }
        std::string sample_error;
        appendWithoutDuplicate(
          &lane.motion_samples,
          sampleLinearPoses(
            lane.motion_start, lane.measurement_start, options.maximum_sample_step_m,
            5.0, options.maximum_samples_per_lane, &sample_error));
        appendWithoutDuplicate(&lane.motion_samples, lane.measurement_samples);
        appendWithoutDuplicate(
          &lane.motion_samples,
          sampleLinearPoses(
            lane.measurement_end, lane.motion_end, options.maximum_sample_step_m,
            5.0, options.maximum_samples_per_lane, &sample_error));
        if (lane.motion_samples.empty() ||
          lane.motion_samples.size() > options.maximum_samples_per_lane)
        {
          if (error) {
            *error = sample_error.empty() ? "motion path exceeds sample limit" : sample_error;
          }
          return false;
        }

        evaluatePreliminarySafety(options, &lane);
        result.total_measurement_length_m += lane.measurement_length_m;
        result.nominal_surface_area_m2 +=
          lane.measurement_length_m * options.nominal_laser_line_width_m;
        result.lanes.push_back(std::move(lane));
      }
    }
  }
  *plan = std::move(result);
  return true;
}

std::vector<double> tabletopRadialAzimuths(const WorkspaceOptions & options)
{
  if (!(std::isfinite(options.azimuth_min_deg) && std::isfinite(options.azimuth_max_deg) &&
    std::isfinite(options.azimuth_center_deg)) ||
    options.azimuth_max_deg <= options.azimuth_min_deg)
  {
    return {};
  }
  std::vector<double> azimuths{options.azimuth_center_deg + options.azimuth_min_deg};
  if (options.azimuth_min_deg < 0.0 && options.azimuth_max_deg > 0.0) {
    azimuths.push_back(options.azimuth_center_deg);
  }
  azimuths.push_back(options.azimuth_center_deg + options.azimuth_max_deg);
  return azimuths;
}

Pose tabletopRadialPose(
  const WorkspaceOptions & options, double radius_m, double height_m,
  double azimuth_deg)
{
  return radialFanPose(options, radius_m, height_m, azimuth_deg);
}

bool buildTabletopRadialFanPlan(
  const WorkspaceOptions & options, const std::vector<double> & safe_outer_limits_m,
  WorkspacePlan * plan, std::string * error)
{
  if (!plan) {
    if (error) {*error = "workspace plan output is null";}
    return false;
  }
  std::string validation_error;
  if (!validateOptions(options, &validation_error)) {
    if (error) {*error = validation_error;}
    return false;
  }
  if (!options.tabletop_downward_scan || !options.tabletop_radial_fan_scan) {
    if (error) {*error = "tabletop radial fan requires downward radial mode";}
    return false;
  }
  const std::vector<double> azimuths = tabletopRadialAzimuths(options);
  if (azimuths.empty() || azimuths.size() > options.maximum_lane_count) {
    if (error) {*error = "tabletop radial fan exceeds lane limits";}
    return false;
  }
  if (!safe_outer_limits_m.empty() && safe_outer_limits_m.size() != azimuths.size()) {
    if (error) {*error = "safe radial limit count does not match fan lanes";}
    return false;
  }

  WorkspacePlan result;
  result.options = options;
  result.radial_layers_m = {options.radial_min_m, options.radial_max_m};
  result.height_layers_m = {options.height_min_m};
  for (std::size_t index = 0U; index < azimuths.size(); ++index) {
    const bool reverse = index % 2U != 0U;
    const double discovered_outer = safe_outer_limits_m.empty() ?
      options.radial_max_m : safe_outer_limits_m[index];
    const bool discovered_safe = std::isfinite(discovered_outer);
    double outer = discovered_safe ?
      std::min(discovered_outer, options.radial_max_m) : options.radial_max_m;

    // The MoveIt probe supplies the exact IK/collision boundary. Independently
    // trim it against the conservative camera-head/base/reach envelope,
    // including the lead-in/out, so a merely reachable TCP endpoint cannot
    // make the complete scan lane unsafe.
    if (discovered_safe) {
      const double outer_lead = std::max(options.lead_in_m, options.lead_out_m);
      const double probe_start = options.fan_origin_at_initial_tcp ?
        options.radial_min_m : options.radial_min_m - outer_lead;
      const double probe_end = outer + outer_lead;
      const std::size_t intervals = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(
          (probe_end - probe_start) / options.maximum_sample_step_m)));
      double last_safe_radius = std::numeric_limits<double>::quiet_NaN();
      for (std::size_t sample = 0U; sample <= intervals; ++sample) {
        const double fraction = static_cast<double>(sample) /
          static_cast<double>(intervals);
        const double radius = probe_start + fraction * (probe_end - probe_start);
        if (!preliminaryPoseSafe(
            options, tabletopRadialPose(
              options, radius, options.height_min_m, azimuths[index])))
        {
          break;
        }
        last_safe_radius = radius;
      }
      if (std::isfinite(last_safe_radius)) {
        outer = std::min(outer, last_safe_radius - outer_lead);
        outer = std::floor(outer * 1000.0 + 1.0e-9) / 1000.0;
      } else {
        outer = std::numeric_limits<double>::quiet_NaN();
      }
    }
    const bool has_safe_interval = discovered_safe && std::isfinite(outer) &&
      outer - options.radial_min_m >= options.minimum_radial_scan_length_m;
    if (!has_safe_interval) {
      // Keep a visible full-length red candidate in RViz. It is never passed
      // to IK/Pilz planning because preliminary_safe is forced false below.
      outer = options.radial_max_m;
    }
    const double start_radius = reverse ? outer : options.radial_min_m;
    const double end_radius = reverse ? options.radial_min_m : outer;
    const double start_lead_radius = reverse ? start_radius + options.lead_in_m :
      (options.fan_origin_at_initial_tcp ? start_radius : start_radius - options.lead_in_m);
    const double end_lead_radius = reverse ?
      (options.fan_origin_at_initial_tcp ? end_radius : end_radius - options.lead_out_m) :
      end_radius + options.lead_out_m;

    ScanLane lane;
    lane.id = static_cast<int>(index);
    lane.radial_index = static_cast<int>(index);
    lane.height_index = 0;
    lane.sector_index = static_cast<int>(index);
    lane.radius_m = outer;
    lane.radial_start_m = start_radius;
    lane.radial_end_m = end_radius;
    lane.height_m = options.height_min_m;
    lane.azimuth_start_deg = azimuths[index];
    lane.azimuth_end_deg = azimuths[index];
    lane.reverse = reverse;
    lane.radial_scan = true;
    lane.motion_start = tabletopRadialPose(
      options, start_lead_radius, lane.height_m, azimuths[index]);
    lane.measurement_start = tabletopRadialPose(
      options, start_radius, lane.height_m, azimuths[index]);
    lane.measurement_mid = tabletopRadialPose(
      options, 0.5 * (start_radius + end_radius), lane.height_m, azimuths[index]);
    lane.measurement_end = tabletopRadialPose(
      options, end_radius, lane.height_m, azimuths[index]);
    lane.motion_end = tabletopRadialPose(
      options, end_lead_radius, lane.height_m, azimuths[index]);
    lane.measurement_length_m = std::fabs(end_radius - start_radius);
    lane.requested_measurement_length_m =
      options.radial_max_m - options.radial_min_m;
    lane.motion_length_m = lane.measurement_length_m +
      (options.fan_origin_at_initial_tcp ?
      (reverse ? options.lead_in_m : options.lead_out_m) :
      options.lead_in_m + options.lead_out_m);
    std::string sample_error;
    lane.measurement_samples = sampleLinearPoses(
      lane.measurement_start, lane.measurement_end, options.maximum_sample_step_m,
      5.0, options.maximum_samples_per_lane, &sample_error);
    appendWithoutDuplicate(
      &lane.motion_samples,
      sampleLinearPoses(
        lane.motion_start, lane.measurement_start, options.maximum_sample_step_m,
        5.0, options.maximum_samples_per_lane, &sample_error));
    appendWithoutDuplicate(&lane.motion_samples, lane.measurement_samples);
    appendWithoutDuplicate(
      &lane.motion_samples,
      sampleLinearPoses(
        lane.measurement_end, lane.motion_end, options.maximum_sample_step_m,
        5.0, options.maximum_samples_per_lane, &sample_error));
    if (lane.measurement_samples.empty() || lane.motion_samples.empty() ||
      lane.motion_samples.size() > options.maximum_samples_per_lane)
    {
      if (error) {
        *error = sample_error.empty() ? "radial path exceeds sample limit" : sample_error;
      }
      return false;
    }
    evaluatePreliminarySafety(options, &lane);
    if (!has_safe_interval) {
      lane.preliminary_safe = false;
      lane.rejection_reason = "no contiguous IK-safe radial interval of minimum length";
    }
    result.total_measurement_length_m += lane.requested_measurement_length_m;
    result.nominal_surface_area_m2 +=
      lane.requested_measurement_length_m * options.nominal_laser_line_width_m;
    result.lanes.push_back(std::move(lane));
  }
  *plan = std::move(result);
  return true;
}

bool laneIntersectsAxisAlignedRoi(
  const ScanLane & lane, const Vec3 & minimum, const Vec3 & maximum, double padding_m)
{
  if (minimum.x > maximum.x || minimum.y > maximum.y || minimum.z > maximum.z ||
    !finite(padding_m) || padding_m < 0.0)
  {
    return false;
  }
  return std::any_of(
    lane.measurement_samples.begin(), lane.measurement_samples.end(),
    [&](const Pose & pose) {return poseInBox(pose, minimum, maximum, padding_m);});
}

}  // namespace fr5_scanner_650::workspace_scan

#include "workspace_return_core.hpp"

#include <builtin_interfaces/msg/duration.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace fr5_scanner_650::workspace_return
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

std::int64_t toNanoseconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<std::int64_t>(duration.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(duration.nanosec);
}

builtin_interfaces::msg::Duration fromNanoseconds(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
  duration.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return duration;
}

bool finiteVector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool optionalDimensionIsValid(
  const std::vector<double> & values, std::size_t expected_size)
{
  return values.empty() || (values.size() == expected_size && finiteVector(values));
}

}  // namespace

bool reverseJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & forward,
  trajectory_msgs::msg::JointTrajectory * reversed,
  std::string * error)
{
  if (!reversed) {
    if (error) {*error = "reverse trajectory output is null";}
    return false;
  }
  if (forward.joint_names.empty() || forward.points.empty()) {
    if (error) {*error = "cannot reverse an empty joint trajectory";}
    return false;
  }

  const std::size_t joint_count = forward.joint_names.size();
  std::int64_t previous_time = -1;
  for (std::size_t index = 0U; index < forward.points.size(); ++index) {
    const auto & point = forward.points[index];
    const std::int64_t point_time = toNanoseconds(point.time_from_start);
    if (point.positions.size() != joint_count || !finiteVector(point.positions) ||
      !optionalDimensionIsValid(point.velocities, joint_count) ||
      !optionalDimensionIsValid(point.accelerations, joint_count) ||
      !optionalDimensionIsValid(point.effort, joint_count))
    {
      if (error) {
        *error = "joint trajectory point " + std::to_string(index) +
          " has invalid dimensions or non-finite values";
      }
      return false;
    }
    if (point_time < 0 || (index > 0U && point_time <= previous_time)) {
      if (error) {
        *error = "joint trajectory timestamps must be non-negative and strictly increasing";
      }
      return false;
    }
    previous_time = point_time;
  }

  const std::int64_t total_time = toNanoseconds(forward.points.back().time_from_start);
  if (total_time <= 0 && forward.points.size() > 1U) {
    if (error) {*error = "multi-point joint trajectory has zero duration";}
    return false;
  }

  *reversed = forward;
  reversed->header.stamp.sec = 0;
  reversed->header.stamp.nanosec = 0U;
  reversed->points.clear();
  reversed->points.reserve(forward.points.size());
  for (auto iterator = forward.points.rbegin(); iterator != forward.points.rend(); ++iterator) {
    auto point = *iterator;
    const std::int64_t reversed_time = total_time - toNanoseconds(iterator->time_from_start);
    point.time_from_start = fromNanoseconds(reversed_time);
    for (double & velocity : point.velocities) {
      velocity = -velocity;
    }
    point.effort.clear();
    reversed->points.push_back(std::move(point));
  }
  return true;
}

}  // namespace fr5_scanner_650::workspace_return

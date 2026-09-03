#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${workspace_dir}/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
required_install_file="${workspace_dir}/install/fr5_scanner_650/share/fr5_scanner_650/config/moveit/fairino5_v6_robot.srdf"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 setup not found: ${ros_setup}" >&2
  exit 1
fi
if [[ ! -f "${workspace_dir}/install/setup.bash" ||
      ! -f "${required_install_file}" ]]; then
  echo "ROS 2 install is missing or older than the scanner MoveIt configuration." >&2
  echo "Rebuild this workspace first:" >&2
  echo "  ${workspace_dir}/build_ros2.sh" >&2
  exit 1
fi

export VIZUM_DATA_DIR="${VIZUM_DATA_DIR:-${project_dir}/myline_hik/data}"
if ! ulimit -c unlimited; then
  echo "Warning: core dumps could not be enabled; native crash trace remains enabled." >&2
fi
crash_trace_dir="${workspace_dir}/log/crash"
mkdir -p "${crash_trace_dir}"
crash_trace_stamp="$(date +%Y%m%d_%H%M%S)"
export SCANNER_650_CRASH_TRACE_PATH="${crash_trace_dir}/workspace_coarse_scan_planner_${crash_trace_stamp}.trace"
set +u
source "${ros_setup}"
source "${workspace_dir}/install/setup.bash"
set -u

exec ros2 launch fr5_scanner_650 scanner_650_moveit.launch.py \
  use_fake_hardware:=true \
  start_camera:=true \
  camera_auto_connect:=false \
  start_laser_control:=true \
  laser_auto_connect:=false \
  "$@"

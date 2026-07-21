#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${workspace_dir}/install/setup.bash"
ros_python="${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 setup not found: ${ros_setup}" >&2
  exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "Workspace is not built: ${workspace_setup}" >&2
  echo "Run ${workspace_dir}/build_ros2.sh first." >&2
  exit 1
fi
if [[ ! -x "${ros_python}" ]]; then
  echo "ROS Python executable not found: ${ros_python}" >&2
  exit 1
fi

set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

# ROS 2 Humble on Ubuntu 22.04 uses the system Python 3.10 ABI. Ensure an
# active Conda environment cannot launch rclpy nodes with another Python ABI.
export PATH="$(dirname "${ros_python}"):${PATH}"

exec ros2 launch fr5_vizum_bringup fr5_live_rviz.launch.py "$@"

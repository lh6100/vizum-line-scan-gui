#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "ROS 2 workspace is not built. Run ${workspace_dir}/build_ros2.sh first." >&2
  exit 2
fi

unset CONDA_DEFAULT_ENV CONDA_EXE CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
unset LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_ETC_DIR
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/${ros_distro}/bin"
set +u
source "${ros_setup}"
source "${workspace_dir}/install/setup.bash"
set -u

if [[ $# -eq 0 ]]; then
  exec ros2 launch welding_bringup core.launch.py
fi
exec "$@"

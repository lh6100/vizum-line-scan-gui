#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

unset CONDA_DEFAULT_ENV CONDA_EXE CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL
unset LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_ETC_DIR
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/${ros_distro}/bin"

set +u
source "${ros_setup}"
set -u
colcon --log-base "${workspace_dir}/log" build \
  --base-paths "${workspace_dir}/src" \
  --build-base "${workspace_dir}/build" \
  --install-base "${workspace_dir}/install" \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -Dfastcdr_DIR="/opt/ros/${ros_distro}/lib/cmake/fastcdr" \
  "$@"

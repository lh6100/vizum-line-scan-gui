#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 setup not found: ${ros_setup}" >&2
  exit 1
fi

set +u
source "${ros_setup}"
set -u
cd "${workspace_dir}"

for vendor_package in fairino_msgs fairino_hardware_v3_9_4 fairino5_v6_moveit2_config; do
  if [[ ! -f "${workspace_dir}/frcobot_ros2-master/${vendor_package}/package.xml" ]]; then
    "${workspace_dir}/fetch_fairino_ros2.sh"
    break
  fi
done

ros_python="${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}"
if [[ ! -x "${ros_python}" ]]; then
  echo "ROS Python executable not found: ${ros_python}" >&2
  exit 1
fi

# Some ROS generators use '#!/usr/bin/env python3' even when CMake was given
# Python3_EXECUTABLE.  Put the matching system Python first so an active Conda
# environment cannot run Humble's Python 3.10 modules with another ABI.
export PATH="$(dirname "${ros_python}"):${PATH}"
# The upstream FAIRINO command server is one very large translation unit.
# Building it concurrently with MoveIt packages can exhaust RAM on deployment
# PCs. Keep the reproducible default conservative; power users can override it.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon_executor="${COLCON_EXECUTOR:-sequential}"

cmake_args=(
  --cmake-args
  "-DPython3_EXECUTABLE=${ros_python}"
)
if [[ -n "${FAIRINO_SDK_DIR:-}" ]]; then
  cmake_args+=("-DFAIRINO_SDK_DIR=${FAIRINO_SDK_DIR}")
fi
if [[ -n "${HIK_MVS_ROOT:-}" ]]; then
  cmake_args+=("-DHIK_MVS_ROOT=${HIK_MVS_ROOT}")
fi

colcon build \
  --executor "${colcon_executor}" \
  --paths \
    "${workspace_dir}/src/*" \
    "${workspace_dir}/frcobot_ros2-master/fairino_msgs" \
    "${workspace_dir}/frcobot_ros2-master/fairino_hardware_v3_9_4" \
    "${workspace_dir}/frcobot_ros2-master/fairino5_v6_moveit2_config" \
  --symlink-install \
  --packages-up-to fr5_vizum_bringup fr5_scanner_650 \
  "$@" \
  "${cmake_args[@]}"

echo
echo "Build complete. Start with:"
echo "  ${workspace_dir}/run_scanner_650_moveit_mock.sh"

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

ros_python="${ROS_PYTHON_EXECUTABLE:-/usr/bin/python3}"
if [[ ! -x "${ros_python}" ]]; then
  echo "ROS Python executable not found: ${ros_python}" >&2
  exit 1
fi

# Some ROS generators use '#!/usr/bin/env python3' even when CMake was given
# Python3_EXECUTABLE.  Put the matching system Python first so an active Conda
# environment cannot run Humble's Python 3.10 modules with another ABI.
export PATH="$(dirname "${ros_python}"):${PATH}"

cmake_args=(
  --cmake-args
  "-DPython3_EXECUTABLE=${ros_python}"
)
if [[ -n "${FAIRINO_SDK_DIR:-}" ]]; then
  cmake_args+=("-DFAIRINO_SDK_DIR=${FAIRINO_SDK_DIR}")
fi

colcon build \
  --base-paths src \
  --symlink-install \
  --packages-up-to fr5_vizum_bringup \
  "$@" \
  "${cmake_args[@]}"

echo
echo "Build complete. Start with:"
echo "  ${workspace_dir}/run_fr5_live_rviz.sh robot_ip:=192.168.1.200"

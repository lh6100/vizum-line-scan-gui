#!/usr/bin/env bash
set -euo pipefail

ros_distro="${ROS_DISTRO:-humble}"
if ! command -v apt-get >/dev/null 2>&1; then
  echo "This helper supports Debian/Ubuntu apt systems only." >&2
  exit 1
fi
if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Install ROS 2 ${ros_distro} from the official ROS repository first." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build git pkg-config patch ripgrep \
  qtbase5-dev libopencv-dev libeigen3-dev \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-opencv python3-pyqt5 python3-yaml \
  "ros-${ros_distro}-moveit" \
  "ros-${ros_distro}-pilz-industrial-motion-planner" \
  "ros-${ros_distro}-ros2-control" \
  "ros-${ros_distro}-ros2-controllers"

echo "System dependencies installed. Vendor SDKs must still be installed separately."

#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${project_dir}/ros2_ws/run_ros2.sh" \
  ros2 launch welding_bringup stereo_depth_preview.launch.py "$@"

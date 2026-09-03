#!/usr/bin/env bash
set -uo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
mvs_root="${HIK_MVS_ROOT:-/opt/MVS}"
fairino_root="${FAIRINO_SDK_DIR:-${project_dir}/SDK/fairino-cpp-sdk-3.9.4}"
errors=0
warnings=0

pass() { printf '[ OK ] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*"; warnings=$((warnings + 1)); }
fail() { printf '[FAIL] %s\n' "$*"; errors=$((errors + 1)); }

require_command() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "command: $1"
  else
    fail "missing command: $1"
  fi
}

require_file() {
  if [[ -f "$1" ]]; then
    pass "file: ${1#${project_dir}/}"
  else
    fail "missing file: $1"
  fi
}

echo "Project: ${project_dir}"
for command_name in git cmake c++ pkg-config python3 rg; do
  require_command "${command_name}"
done

for scanner_id in scanner_450 scanner_650; do
  for calibration_name in hik_intrinsics.yaml hik_laser_plane.yaml hik_handeye.yaml synchronization.yaml; do
    require_file "${project_dir}/myline_hik/config/devices/${scanner_id}/${calibration_name}"
  done
done

if [[ -f "${project_dir}/ros2_fr5/frcobot_ros2-master/fairino_msgs/package.xml" &&
      -f "${project_dir}/ros2_fr5/frcobot_ros2-master/fairino_hardware_v3_9_4/package.xml" &&
      -f "${project_dir}/ros2_fr5/frcobot_ros2-master/fairino5_v6_moveit2_config/package.xml" ]]; then
  pass "pinned FAIRINO ROS 2 vendor checkout"
else
  warn "FAIRINO ROS 2 vendor is not fetched yet; build_ros2.sh will fetch it"
fi

if pkg-config --exists Qt5Core opencv4 2>/dev/null; then
  pass "Qt5 and OpenCV development packages"
else
  fail "Qt5/OpenCV development packages are not visible to pkg-config"
fi
if [[ -d /usr/include/eigen3/Eigen ]] || pkg-config --exists eigen3 2>/dev/null; then
  pass "Eigen3 development package"
else
  fail "Eigen3 development package is missing"
fi

if [[ -f "${mvs_root}/include/MvCameraControl.h" &&
      -f "${mvs_root}/lib/64/libMvCameraControl.so" ]]; then
  pass "Hikrobot MVS SDK: ${mvs_root}"
else
  fail "Hikrobot MVS SDK incomplete: ${mvs_root}"
fi

if [[ -f "${fairino_root}/libfairino/src/include/Robot-CN/robot.h" ]]; then
  pass "FAIRINO 3.9.4 source SDK: ${fairino_root}"
else
  fail "FAIRINO source SDK not found: ${fairino_root}"
fi

if [[ -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  pass "ROS 2 setup: /opt/ros/${ros_distro}/setup.bash"
else
  fail "ROS 2 setup missing: /opt/ros/${ros_distro}/setup.bash"
fi
require_command colcon

if rg -n '/home/zhulong' \
    "${project_dir}/myline_hik/config" \
    "${project_dir}/ros2_fr5/src/fr5_scanner_650/config" \
    "${project_dir}/ros2_fr5/src/fr5_scanner_650/launch" >/dev/null 2>&1; then
  fail "runtime configuration still contains a developer-specific /home/zhulong path"
else
  pass "runtime configuration has no developer-specific home path"
fi

local_file_found=false
if git -C "${project_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if git -C "${project_dir}" ls-files |
      rg -q '(^|/)(id_ed25519|known_hosts|\.env\.local)$'; then
    local_file_found=true
  fi
elif find "${project_dir}" -type f \
    \( -name id_ed25519 -o -name known_hosts -o -name .env.local \) \
    -print -quit | rg -q .; then
  local_file_found=true
fi

if [[ "${local_file_found}" == "true" ]]; then
  fail "a local credential/environment file is tracked by Git"
else
  pass "no SSH key or local environment file is included"
fi

printf '\nResult: %d error(s), %d warning(s).\n' "${errors}" "${warnings}"
if (( errors > 0 )); then
  exit 1
fi

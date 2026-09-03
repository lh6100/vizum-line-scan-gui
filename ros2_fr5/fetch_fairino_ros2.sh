#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vendor_dir="${FAIRINO_ROS2_VENDOR_DIR:-${workspace_dir}/frcobot_ros2-master}"
vendor_url="https://github.com/FAIR-INNOVATION/frcobot_ros2.git"
vendor_commit="5bed0b0263c8f1e95f51aa45079f904d463c5c50"
runtime_patch="${workspace_dir}/patches/fairino_ros2_runtime_install.patch"
dependency_patch="${workspace_dir}/patches/fairino_moveit_hardware_dependency.patch"
hardware_patch="${workspace_dir}/patches/fairino_hardware_safety.patch"
required_packages=(fairino_msgs fairino_hardware_v3_9_4 fairino5_v6_moveit2_config)

vendor_complete=true
for package_name in "${required_packages[@]}"; do
  if [[ ! -f "${vendor_dir}/${package_name}/package.xml" ]]; then
    vendor_complete=false
  fi
done

if [[ "${vendor_complete}" != "true" ]]; then
  if [[ -e "${vendor_dir}" ]]; then
    echo "Incomplete FAIRINO vendor directory already exists: ${vendor_dir}" >&2
    echo "Move it aside, then run this script again." >&2
    exit 1
  fi
  temporary_dir="$(mktemp -d /tmp/vizum-fairino-ros2.XXXXXX)"
  trap 'rm -rf -- "${temporary_dir}"' EXIT
  checkout_dir="${temporary_dir}/frcobot_ros2"
  git clone --filter=blob:none --no-checkout "${vendor_url}" "${checkout_dir}"
  git -C "${checkout_dir}" sparse-checkout init --cone
  git -C "${checkout_dir}" sparse-checkout set "${required_packages[@]}"
  git -C "${checkout_dir}" checkout --detach "${vendor_commit}"
  mkdir -p "$(dirname -- "${vendor_dir}")"
  mv -- "${checkout_dir}" "${vendor_dir}"
fi

if [[ -d "${vendor_dir}/.git" ]]; then
  actual_commit="$(git -C "${vendor_dir}" rev-parse HEAD)"
  if [[ "${actual_commit}" != "${vendor_commit}" ]]; then
    echo "Unexpected FAIRINO vendor commit: ${actual_commit}" >&2
    echo "Expected: ${vendor_commit}" >&2
    exit 1
  fi
fi

if ! rg -q 'INSTALL_RPATH "\$ORIGIN/\.\."' \
    "${vendor_dir}/fairino_hardware_v3_9_4/CMakeLists.txt"; then
  patch --batch --forward --ignore-whitespace \
    -d "${vendor_dir}" -p1 < "${runtime_patch}"
fi

if ! rg -q '<exec_depend>fairino_hardware_v3_9_4</exec_depend>' \
    "${vendor_dir}/fairino5_v6_moveit2_config/package.xml"; then
  patch --batch --forward --ignore-whitespace \
    -d "${vendor_dir}" -p1 < "${dependency_patch}"
fi

if ! rg -q '_servo_period_s = 0\.008' \
    "${vendor_dir}/fairino_hardware_v3_9_4/include/fairino_hardware/fairino_hardware_interface.hpp"; then
  patch --batch --forward --ignore-whitespace \
    -d "${vendor_dir}" -p1 < "${hardware_patch}"
fi

echo "FAIRINO ROS 2 vendor ready: ${vendor_dir}"
echo "Pinned commit: ${vendor_commit}"

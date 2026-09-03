#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${workspace_dir}/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
label="${1:-pm45}"

if [[ ! "${label}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Recording label may contain only letters, digits, '.', '_' and '-'." >&2
  exit 2
fi

set +u
source "${ros_setup}"
source "${workspace_dir}/install/setup.bash"
set -u

data_root="${VIZUM_DATA_DIR:-${project_dir}/myline_hik/data}"
output_root="${SCANNER_650_ROSBAG_ROOT:-${data_root}/rosbags/scanner_650}"
timestamp="$(date +%Y%m%d_%H%M%S)"
session_dir="${output_root}/${timestamp}_${label}"
bag_dir="${session_dir}/bag"
metadata_dir="${session_dir}/metadata"
config_dir="${metadata_dir}/config_snapshot"
mkdir -p "${metadata_dir}" "${config_dir}"
touch "${metadata_dir}/recording_started.marker"
printf '%s\n' "${session_dir}" >"${output_root}/latest_session.txt"

required_topics=(
  /scanner_650/image_raw
  /scanner_650/camera_info
  /joint_states
  /tf
  /tf_static
)
for topic in "${required_topics[@]}"; do
  if ! timeout 3 ros2 topic type "${topic}" >/dev/null 2>&1; then
    echo "Required topic is unavailable: ${topic}" >&2
    echo "Start the scanner_650 hardware stack before recording." >&2
    exit 3
  fi
done

available_kib="$(df -Pk "${output_root}" | awk 'NR == 2 {print $4}')"
available_gib="$((available_kib / 1024 / 1024))"
minimum_free_gib="${SCANNER_650_MIN_FREE_GB:-20}"
if (( available_gib < minimum_free_gib )); then
  echo "Insufficient free disk space: ${available_gib} GiB available; ${minimum_free_gib} GiB required." >&2
  exit 4
fi

capture_parameters() {
  local suffix="$1"
  local node safe_name
  for node in \
    /hik_single_camera \
    /line_laser_reconstruction \
    /line_laser_control \
    /table_collision_scene \
    /workspace_coarse_scan_planner \
    /controller_manager \
    /move_group
  do
    safe_name="${node#/}"
    ros2 param dump "${node}" >"${metadata_dir}/${safe_name}_params_${suffix}.yaml" 2>"${metadata_dir}/${safe_name}_params_${suffix}.err" || true
  done
}

date --iso-8601=seconds >"${metadata_dir}/start_time.txt"
printf '%s\n' "${session_dir}" >"${metadata_dir}/session_path.txt"
printf 'label=%s\nfree_disk_gib=%s\n' "${label}" "${available_gib}" >"${metadata_dir}/recording_settings.txt"
ros2 node list | sort >"${metadata_dir}/nodes_start.txt"
ros2 topic list -t | sort >"${metadata_dir}/topics_start.txt"
ros2 service list -t | sort >"${metadata_dir}/services_start.txt"
ros2 action list -t | sort >"${metadata_dir}/actions_start.txt"
capture_parameters start

cp "${workspace_dir}/src/fr5_scanner_650/config/scanner_650.yaml" "${config_dir}/"
cp "${workspace_dir}/src/fairino_description/urdf/fairino5_v6.urdf" "${config_dir}/"
for calibration in hik_intrinsics.yaml hik_laser_plane.yaml hik_handeye.yaml synchronization.yaml; do
  source_path="${project_dir}/myline_hik/config/devices/scanner_650/${calibration}"
  if [[ -f "${source_path}" ]]; then
    cp "${source_path}" "${config_dir}/"
  fi
done
find "${config_dir}" -maxdepth 1 -type f -print0 | sort -z | xargs -0 sha256sum >"${metadata_dir}/config_sha256.txt"
git -C "${project_dir}" rev-parse HEAD >"${metadata_dir}/git_commit.txt" 2>/dev/null || true
git -C "${project_dir}" status --short >"${metadata_dir}/git_status.txt" 2>/dev/null || true
uname -a >"${metadata_dir}/uname.txt"
df -h "${output_root}" >"${metadata_dir}/disk_start.txt"

scanner_topics="image_raw|camera_info|camera_status|laser_status|reconstruction_status|profile_cloud|markers|workspace_coarse_scan_markers|workspace_coarse_scan_status|table_collision_scene_status"
if [[ "${SCANNER_650_RECORD_SCAN_CLOUD:-0}" == "1" ]]; then
  scanner_topics="${scanner_topics}|scan_cloud"
  printf 'record_scan_cloud=true\n' >>"${metadata_dir}/recording_settings.txt"
else
  printf 'record_scan_cloud=false\n' >>"${metadata_dir}/recording_settings.txt"
fi

topic_regex="^/(scanner_650/(${scanner_topics})|tf|tf_static|joint_states|dynamic_joint_states|fairino5_controller/(controller_state|state|joint_trajectory)|attached_collision_object|collision_object|display_contacts|display_planned_path|monitored_planning_scene|motion_plan_request|planning_scene|planning_scene_world|robot_description|robot_description_semantic|trajectory_execution_event|parameter_events|rosout)$"
printf 'topic_regex=%s\n' "${topic_regex}" >>"${metadata_dir}/recording_settings.txt"

qos_overrides="${workspace_dir}/src/fr5_scanner_650/config/scanner_650_rosbag_qos.yaml"
bag_args=(
  --regex "${topic_regex}"
  --output "${bag_dir}"
  --storage sqlite3
  --polling-interval 100
  --max-cache-size 1073741824
  --max-bag-size 4294967296
  --qos-profile-overrides-path "${qos_overrides}"
)

compression="${SCANNER_650_ROSBAG_COMPRESSION:-none}"
case "${compression}" in
  none)
    ;;
  zstd)
    bag_args+=(
      --compression-mode file
      --compression-format zstd
      --compression-queue-size 4
      --compression-threads 2
    )
    ;;
  *)
    echo "SCANNER_650_ROSBAG_COMPRESSION must be 'none' or 'zstd'." >&2
    exit 5
    ;;
esac
printf 'compression=%s\n' "${compression}" >>"${metadata_dir}/recording_settings.txt"

echo "Session: ${session_dir}"
echo "Free disk: ${available_gib} GiB"
echo "Recording raw images and scanner/robot state. Press Ctrl-C only after the scan has stopped and the laser is OFF."
if [[ "${SCANNER_650_RECORD_SCAN_CLOUD:-0}" != "1" ]]; then
  echo "The repeatedly accumulated /scanner_650/scan_cloud topic is excluded to control bag size; raw images and per-frame profiles are recorded."
fi

set +e
ros2 bag record "${bag_args[@]}"
record_status=$?
set -e

date --iso-8601=seconds >"${metadata_dir}/end_time.txt"
capture_parameters end
ros2 topic list -t | sort >"${metadata_dir}/topics_end.txt" || true
df -h "${output_root}" >"${metadata_dir}/disk_end.txt"
ros2 bag info "${bag_dir}" >"${metadata_dir}/rosbag_info.txt" 2>"${metadata_dir}/rosbag_info.err" || true

saved_cloud="${data_root}/scans/scanner_650/ros2_scan_cloud.ply"
if [[ -f "${saved_cloud}" && "${saved_cloud}" -nt "${metadata_dir}/recording_started.marker" ]]; then
  mkdir -p "${session_dir}/artifacts"
  cp "${saved_cloud}" "${session_dir}/artifacts/"
fi

if [[ ${record_status} -ne 0 && ${record_status} -ne 130 ]]; then
  echo "ros2 bag record exited with status ${record_status}; inspect ${metadata_dir}." >&2
  exit "${record_status}"
fi

echo "Recording complete: ${session_dir}"
echo "Bag summary: ${metadata_dir}/rosbag_info.txt"

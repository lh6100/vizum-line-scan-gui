#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"
isaaclab_root="${ISAACLAB_ROOT:-/home/zhulong/IsaacLab-3.0.0-beta2}"
venv_root="${isaaclab_root}/env_isaaclab"
converter="${isaaclab_root}/scripts/tools/convert_urdf.py"
urdf="${repo_root}/ros2_fr5/src/fairino_description/urdf/fairino5_v6.urdf"
mesh_dir="${repo_root}/ros2_fr5/src/fairino_description/meshes/fairino5_v6"
robot_output="${repo_root}/isaacSim/generated/fr5_v6"

if [[ ! -x "${isaaclab_root}/isaaclab.sh" || ! -x "${venv_root}/bin/python" ]]; then
  echo "Isaac Lab 3.0 environment not found at: ${isaaclab_root}" >&2
  echo "Set ISAACLAB_ROOT to the correct installation directory." >&2
  exit 2
fi
if [[ ! -f "${urdf}" ]]; then
  echo "FR5 URDF not found: ${urdf}" >&2
  exit 2
fi
if [[ ! -f "${converter}" || ! -f "${isaaclab_root}/VERSION" ]]; then
  echo "Isaac Lab converter/version metadata is incomplete under: ${isaaclab_root}" >&2
  exit 2
fi

if ! compgen -G "${mesh_dir}/*.STL" >/dev/null; then
  echo "No FR5 STL meshes found under: ${mesh_dir}" >&2
  exit 2
fi

# A shared Python implementation is also used by launch-time validation, so
# the build and validator cannot silently disagree about source provenance.
source_hash="$(
  "${venv_root}/bin/python" "${script_dir}/validate_twin.py" \
    --print-robot-source-hash --isaaclab-root "${isaaclab_root}"
)"

mkdir -p "${robot_output}"
if ! command -v flock >/dev/null; then
  echo "flock is required to serialize FR5 asset publication." >&2
  exit 2
fi
exec 9>"${robot_output}/.${source_hash}.lock"
flock 9

reuse_robot=false
shopt -s nullglob
cache_candidates=("${robot_output}/${source_hash}-"*/fairino5_v6/fairino5_v6.usda)
shopt -u nullglob
for candidate in "${cache_candidates[@]}"; do
  if "${venv_root}/bin/python" "${script_dir}/validate_twin.py" --robot-only "${candidate}"; then
    robot_usd="${candidate}"
    reuse_robot=true
    break
  fi
  echo "Ignoring invalid immutable cache: ${candidate}" >&2
done

if [[ "${reuse_robot}" != true ]]; then
  temporary_conversion="$(mktemp -d "${robot_output}/.convert-${source_hash}.XXXXXX")"
  if ! env \
    VIRTUAL_ENV="${venv_root}" \
    PATH="${venv_root}/bin:${PATH}" \
    ROS_PACKAGE_PATH="${repo_root}/ros2_fr5/src" \
    "${isaaclab_root}/isaaclab.sh" -p \
    "${converter}" \
    "${urdf}" "${temporary_conversion}" \
    --fix-base --joint-stiffness 1000 --joint-damping 50 --viz none; then
    failed_conversion="${temporary_conversion}.failed"
    mv "${temporary_conversion}" "${failed_conversion}"
    echo "URDF converter failed; partial output preserved at: ${failed_conversion}" >&2
    exit 3
  fi

  temporary_robot_usd="${temporary_conversion}/fairino5_v6/fairino5_v6.usda"
  if [[ ! -f "${temporary_robot_usd}" ]]; then
    failed_conversion="${temporary_conversion}.failed"
    mv "${temporary_conversion}" "${failed_conversion}"
    echo "URDF converter output is incomplete; preserved at: ${failed_conversion}" >&2
    exit 3
  fi
  if ! "${venv_root}/bin/python" "${script_dir}/validate_twin.py" --robot-only "${temporary_robot_usd}"; then
    failed_conversion="${temporary_conversion}.failed"
    mv "${temporary_conversion}" "${failed_conversion}"
    echo "Invalid URDF conversion preserved at: ${failed_conversion}" >&2
    exit 3
  fi
  post_conversion_source_hash="$(
    "${venv_root}/bin/python" "${script_dir}/validate_twin.py" \
      --print-robot-source-hash --isaaclab-root "${isaaclab_root}"
  )"
  if [[ "${post_conversion_source_hash}" != "${source_hash}" ]]; then
    failed_conversion="${temporary_conversion}.source-changed"
    mv "${temporary_conversion}" "${failed_conversion}"
    echo "FR5 sources changed during conversion; output preserved at: ${failed_conversion}" >&2
    exit 3
  fi
  asset_hash="$(
    "${venv_root}/bin/python" "${script_dir}/validate_twin.py" \
      --robot-only "${temporary_robot_usd}" --print-robot-asset-hash
  )"
  conversion_root="${robot_output}/${source_hash}-${asset_hash}"
  robot_usd="${conversion_root}/fairino5_v6/fairino5_v6.usda"
  if [[ -e "${conversion_root}" ]]; then
    echo "Validated conversion already exists unexpectedly: ${conversion_root}" >&2
    exit 3
  fi
  mv "${temporary_conversion}" "${conversion_root}"
else
  echo "Reusing content-matched FR5 USD: ${robot_usd}"
fi

twin_output_dir="${repo_root}/isaacSim/generated/fr5_hik_twin"
twin_output="${twin_output_dir}/fr5_hik_line_laser_twin.usda"
mkdir -p "${twin_output_dir}"
temporary_twin="$(mktemp --suffix=.usda "${twin_output_dir}/.twin-candidate.XXXXXX")"
rm -f "${temporary_twin}"

if ! "${venv_root}/bin/python" \
  "${script_dir}/build_twin.py" \
  --robot-usd "${robot_usd}" \
  --robot-source-hash "${source_hash}" \
  --output "${temporary_twin}"; then
  echo "Twin build failed; previous formal USD was preserved: ${twin_output}" >&2
  exit 4
fi

if ! "${venv_root}/bin/python" \
  "${script_dir}/validate_twin.py" \
  --robot-usd "${robot_usd}" \
  --output "${temporary_twin}" \
  --isaaclab-root "${isaaclab_root}"; then
  failed_twin="${temporary_twin}.failed"
  mv "${temporary_twin}" "${failed_twin}"
  echo "Invalid twin candidate preserved at: ${failed_twin}" >&2
  echo "Previous formal USD was preserved: ${twin_output}" >&2
  exit 4
fi

mv "${temporary_twin}" "${twin_output}"
"${venv_root}/bin/python" "${script_dir}/validate_twin.py" \
  --robot-usd "${robot_usd}" \
  --output "${twin_output}" \
  --isaaclab-root "${isaaclab_root}"

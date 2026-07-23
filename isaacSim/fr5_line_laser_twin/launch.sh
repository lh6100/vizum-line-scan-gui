#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"
usd="${repo_root}/isaacSim/generated/fr5_hik_twin/fr5_hik_line_laser_twin.usda"
isaac_experience="${ISAAC_SIM_EXPERIENCE:-isaacsim.exp.full}"
isaaclab_root="${ISAACLAB_ROOT:-/home/zhulong/IsaacLab-3.0.0-beta2}"
isaac_python="${ISAAC_SIM_PYTHON:-${isaaclab_root}/env_isaaclab/bin/python}"

if [[ ! -f "${usd}" ]]; then
  echo "Twin USD is missing. Run isaacSim/fr5_line_laser_twin/build.sh first." >&2
  exit 2
fi
if [[ ! -x "${isaac_python}" ]]; then
  echo "Isaac Sim Python environment not found: ${isaac_python}" >&2
  echo "Set ISAAC_SIM_PYTHON or ISAACLAB_ROOT to the correct installation directory." >&2
  exit 2
fi

"${isaac_python}" "${script_dir}/validate_twin.py" --isaaclab-root "${isaaclab_root}"
# A USD passed after the pip `isaacsim` experience is not opened automatically.
# The Python launcher opens it after the Full experience has finished startup.
exec "${isaac_python}" "${script_dir}/launch_twin.py" \
  --usd "${usd}" \
  --experience "${isaac_experience}" \
  "$@"

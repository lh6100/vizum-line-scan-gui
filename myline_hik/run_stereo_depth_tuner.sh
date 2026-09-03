#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MVS_ROOT="${HIK_MVS_ROOT:-/opt/MVS}"
export PYTHONPATH="${MVS_ROOT}/Samples/64/Python/MvImport${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="${MVS_ROOT}/lib/64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec python3 "${project_dir}/tools/stereo_depth_tuner.py" "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCAN_CALIBRATION_ARGS=(--use-device-calibration-as-is)

# The field launcher uses the numeric calibration values currently stored in
# config/devices. Set HIK_STRICT_CALIBRATION=1 only when provenance hash
# equality should be restored as a hard startup requirement.
if [[ "${HIK_STRICT_CALIBRATION:-0}" == "1" ]]; then
    SCAN_CALIBRATION_ARGS=()
fi

exec "${SCRIPT_DIR}/tools/run_system_qt_target.sh" \
    HikConstantLaserScan "${SCAN_CALIBRATION_ARGS[@]}" "$@"

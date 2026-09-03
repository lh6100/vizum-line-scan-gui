#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${1:-}"
if [[ "${TARGET}" != "HikLineLaserCalibration" &&
      "${TARGET}" != "HikConstantLaserScan" &&
      "${TARGET}" != "HikStereoCalibration" &&
      "${TARGET}" != "HikStereoMapper" &&
      "${TARGET}" != "HikExposureTest" &&
      "${TARGET}" != "HikExposureSweep" ]]; then
    echo "usage: $0 HikLineLaserCalibration|HikConstantLaserScan|HikStereoCalibration|HikStereoMapper|HikExposureTest|HikExposureSweep [arguments...]" >&2
    exit 2
fi
shift

BUILD_DIR="${SCRIPT_DIR}/build"
SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
CMAKE_BIN="${CMAKE_BIN:-/usr/bin/cmake}"
if [[ ! -x "${CMAKE_BIN}" ]]; then
    echo "System CMake not found: ${CMAKE_BIN}" >&2
    exit 1
fi

FAIRINO_ROOT="${FAIRINO_SDK_DIR:-${SCRIPT_DIR}/../SDK/fairino-cpp-sdk-3.9.4}"
if [[ -d "${FAIRINO_ROOT}/libfairino/src" ]]; then
    "${SCRIPT_DIR}/tools/build_patched_fairino_sdk.sh"
fi

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
    PATH="${SYSTEM_PATH}" \
    "${CMAKE_BIN}" -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Release

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
    PATH="${SYSTEM_PATH}" \
    "${CMAKE_BIN}" --build "${BUILD_DIR}" --target "${TARGET}" \
        -j"$(/usr/bin/nproc)"

exec env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u PKG_CONFIG_PATH PATH="${SYSTEM_PATH}" \
    "${BUILD_DIR}/${TARGET}" "$@"

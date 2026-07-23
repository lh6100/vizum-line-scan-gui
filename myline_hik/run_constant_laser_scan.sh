#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
SYSTEM_PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
SYSTEM_QT5_DIR="/usr/lib/x86_64-linux-gnu/cmake/Qt5"
SYSTEM_OPENCV_DIR="/usr/lib/x86_64-linux-gnu/cmake/opencv4"

if [[ -d "${SCRIPT_DIR}/../SDK/fairino-cpp-sdk-3.9.4/libfairino/src" ]]; then
    "${SCRIPT_DIR}/tools/build_patched_fairino_sdk.sh"
fi

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
    PATH="${SYSTEM_PATH}" \
    /usr/bin/cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DQt5_DIR="${SYSTEM_QT5_DIR}" \
        -DQt5Core_DIR="${SYSTEM_QT5_DIR}Core" \
        -DQt5Gui_DIR="${SYSTEM_QT5_DIR}Gui" \
        -DQt5Widgets_DIR="${SYSTEM_QT5_DIR}Widgets" \
        -DOpenCV_DIR="${SYSTEM_OPENCV_DIR}"

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
    PATH="${SYSTEM_PATH}" \
    /usr/bin/cmake --build "${BUILD_DIR}" --target HikConstantLaserScan -j"$(/usr/bin/nproc)"

exec env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
    PATH="${SYSTEM_PATH}" \
    "${BUILD_DIR}/HikConstantLaserScan" "$@"

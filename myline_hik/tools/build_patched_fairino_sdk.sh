#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_SDK_DIR="$(cd -- "${PROJECT_DIR}/../SDK" && pwd)/fairino-cpp-sdk-3.9.4"
SDK_DIR="${FAIRINO_SDK_DIR:-${DEFAULT_SDK_DIR}}"
PATCH_FILE="${PROJECT_DIR}/patches/fairino-cpp-sdk-3.9.4-realtime-callback.patch"
SOURCE_HEADER="${SDK_DIR}/libfairino/src/include/Robot-CN/robot.h"
PUBLIC_HEADER="${SDK_DIR}/linux/libfairino/include/robot.h"
SOURCE_FILE="${SDK_DIR}/libfairino/src/src/Robot/robot.cpp"
SDK_SOURCE_DIR="${SDK_DIR}/libfairino"
SDK_BUILD_DIR="${SDK_SOURCE_DIR}/build-codex"
SDK_LIBRARY="${SDK_SOURCE_DIR}/LinuxBuild/bin/libfairino.so.2.3.4"

for required in "${PATCH_FILE}" "${SOURCE_HEADER}" "${PUBLIC_HEADER}" \
                "${SOURCE_FILE}" "${SDK_SOURCE_DIR}/CMakeLists.txt"; do
    if [[ ! -f "${required}" ]]; then
        echo "Missing FAIRINO SDK input: ${required}" >&2
        exit 1
    fi
done

if ! /usr/bin/grep -q "SetRobotRealtimeStateCallback" "${SOURCE_HEADER}"; then
    # FAIRINO 3.9.4 is distributed with CRLF files. Normalize only the two
    # patched source files so the tracked unified diff applies reproducibly.
    /usr/bin/sed -i 's/\r$//' "${SOURCE_HEADER}" "${SOURCE_FILE}"
    /usr/bin/patch --batch --forward --no-backup-if-mismatch \
        -d "${SDK_DIR}" -p1 < "${PATCH_FILE}"
fi

/usr/bin/cp "${SOURCE_HEADER}" "${PUBLIC_HEADER}"

if [[ -f "${SDK_LIBRARY}" ]] &&
   /usr/bin/nm -D -C "${SDK_LIBRARY}" 2>/dev/null |
       /usr/bin/grep "FRRobot::SetRobotRealtimeStateCallback" >/dev/null; then
    echo "FAIRINO realtime callback SDK is already built: ${SDK_LIBRARY}"
    exit 0
fi

if [[ -f "${SDK_LIBRARY}" && ! -f "${SDK_LIBRARY}.original" ]]; then
    /usr/bin/cp "${SDK_LIBRARY}" "${SDK_LIBRARY}.original"
fi

/usr/bin/cmake -S "${SDK_SOURCE_DIR}" -B "${SDK_BUILD_DIR}"
/usr/bin/cmake --build "${SDK_BUILD_DIR}" --target fairino \
    -j"$(/usr/bin/nproc)"

if ! /usr/bin/nm -D -C "${SDK_LIBRARY}" |
     /usr/bin/grep "FRRobot::SetRobotRealtimeStateCallback" >/dev/null; then
    echo "Patched FAIRINO library does not export the realtime callback" >&2
    exit 1
fi

echo "Built patched FAIRINO SDK: ${SDK_LIBRARY}"

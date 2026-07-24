#!/usr/bin/env bash
# One-time host + LubanCat provisioning for passwordless daily laser control.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DEPLOY_DIR="${PROJECT_DIR}/deploy/lubancat4-v1"

HOST_ADDRESS="192.168.1.12"
ADMIN_USER="cat"
SSH_PORT="22"
CONFIG_BASE="${XDG_CONFIG_HOME:-${HOME:?HOME is not set}/.config}"
KEY_DIR="${CONFIG_BASE}/myline_hik/laser-control"
CONTROLLER_CIDR=""
REFRESH_HOST_KEY=0

usage() {
    cat <<'EOF'
用法:
  ./tools/setup_laser_control.sh [选项]

选项:
  --host IP                 鲁班猫地址，默认 192.168.1.12
  --admin-user USER         一次性安装所用管理账号，默认 cat
  --port PORT               SSH 端口，默认 22
  --key-dir DIR             专用密钥与 known_hosts 目录
  --controller-cidr CIDR    限制控制密钥来源；默认自动检测本机 IPv4/32
  --refresh-host-key        重新人工确认并替换本服务的固定 host key

本脚本只在首次板端安装时可能要求 cat SSH 密码和 sudo 密码。安装完成后，
GUI 使用专用受限密钥，不会请求 SSH 或 sudo 密码。
EOF
}

while (($# > 0)); do
    case "$1" in
        --host)
            HOST_ADDRESS="${2:?--host requires an address}"
            shift 2
            ;;
        --admin-user)
            ADMIN_USER="${2:?--admin-user requires a user}"
            shift 2
            ;;
        --port)
            SSH_PORT="${2:?--port requires a number}"
            shift 2
            ;;
        --key-dir)
            KEY_DIR="${2:?--key-dir requires a directory}"
            shift 2
            ;;
        --controller-cidr)
            CONTROLLER_CIDR="${2:?--controller-cidr requires an address}"
            shift 2
            ;;
        --refresh-host-key)
            REFRESH_HOST_KEY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf '未知参数: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ ! "${HOST_ADDRESS}" =~ ^[A-Za-z0-9.-]+$ ]]; then
    printf '错误: --host 格式无效。\n' >&2
    exit 2
fi
if [[ ! "${ADMIN_USER}" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ ]]; then
    printf '错误: --admin-user 格式无效。\n' >&2
    exit 2
fi
if [[ ! "${SSH_PORT}" =~ ^[0-9]+$ ||
      "${SSH_PORT}" -lt 1 || "${SSH_PORT}" -gt 65535 ]]; then
    printf '错误: --port 必须为 1–65535。\n' >&2
    exit 2
fi
if [[ -n "${CONTROLLER_CIDR}" &&
      ! "${CONTROLLER_CIDR}" =~ ^[0-9A-Fa-f:.]+(/[0-9]{1,3})?$ ]]; then
    printf '错误: --controller-cidr 格式无效。\n' >&2
    exit 2
fi
for required in \
    line_laser_daemon.py line_laser_gateway.py line-laser-daemon.service \
    install.sh uninstall.sh README.md; do
    if [[ ! -f "${DEPLOY_DIR}/${required}" ]]; then
        printf '错误: 缺少部署文件: %s/%s\n' "${DEPLOY_DIR}" "${required}" >&2
        exit 1
    fi
done
for command in ssh scp ssh-keygen ssh-keyscan mktemp; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        printf '错误: 本机缺少命令: %s\n' "${command}" >&2
        exit 1
    fi
done

install -d -m 0700 "${KEY_DIR}"
PRIVATE_KEY="${KEY_DIR}/id_ed25519"
PUBLIC_KEY="${PRIVATE_KEY}.pub"
KNOWN_HOSTS="${KEY_DIR}/known_hosts"

if [[ ! -f "${PRIVATE_KEY}" ]]; then
    printf '生成仅限线激光协议的 Ed25519 密钥: %s\n' "${PRIVATE_KEY}"
    ssh-keygen -q -t ed25519 -N '' \
        -C 'myline-hik-line-laser-control' -f "${PRIVATE_KEY}"
elif [[ ! -f "${PUBLIC_KEY}" ]]; then
    ssh-keygen -y -f "${PRIVATE_KEY}" >"${PUBLIC_KEY}"
fi
chmod 0600 "${PRIVATE_KEY}"
chmod 0644 "${PUBLIC_KEY}"

SETUP_TEMP="$(mktemp -d /tmp/myline-laser-host-setup.XXXXXX)"
cleanup_local() {
    rm -rf -- "${SETUP_TEMP}"
}
trap cleanup_local EXIT

if [[ ! -s "${KNOWN_HOSTS}" || "${REFRESH_HOST_KEY}" -eq 1 ]]; then
    CANDIDATE_HOSTS="${SETUP_TEMP}/known_hosts.candidate"
    printf '读取 %s:%s 的 Ed25519 SSH host key...\n' \
        "${HOST_ADDRESS}" "${SSH_PORT}"
    ssh-keyscan -T 5 -t ed25519 -p "${SSH_PORT}" \
        "${HOST_ADDRESS}" >"${CANDIDATE_HOSTS}" 2>/dev/null
    if [[ ! -s "${CANDIDATE_HOSTS}" ]]; then
        printf '错误: 无法读取板端 SSH host key。\n' >&2
        exit 1
    fi
    printf '\n待固定的 host key 指纹：\n'
    ssh-keygen -lf "${CANDIDATE_HOSTS}"
    printf '%s\n' \
        '请在鲁班猫本地控制台核对 /etc/ssh/ssh_host_ed25519_key.pub 的指纹。'
    printf '确认指纹完全一致后输入 yes: '
    read -r HOST_KEY_CONFIRMATION
    if [[ "${HOST_KEY_CONFIRMATION}" != "yes" ]]; then
        printf '已取消；没有更新 known_hosts。\n' >&2
        exit 1
    fi
    install -m 0600 "${CANDIDATE_HOSTS}" "${KNOWN_HOSTS}"
else
    printf '沿用已固定的专用 known_hosts: %s\n' "${KNOWN_HOSTS}"
    ssh-keygen -lf "${KNOWN_HOSTS}"
fi

if [[ -z "${CONTROLLER_CIDR}" ]] && command -v ip >/dev/null 2>&1; then
    ROUTE_OUTPUT="$(ip -4 route get "${HOST_ADDRESS}" 2>/dev/null || true)"
    if [[ "${ROUTE_OUTPUT}" =~ src[[:space:]]+([0-9.]+) ]]; then
        CONTROLLER_CIDR="${BASH_REMATCH[1]}/32"
    fi
fi
if [[ -z "${CONTROLLER_CIDR}" ]]; then
    printf '%s\n' \
        '错误: 无法自动检测控制机 IPv4；请显式提供 --controller-cidr。' >&2
    exit 1
fi
printf '控制密钥来源限制: %s\n' "${CONTROLLER_CIDR}"

SSH_COMMON=(
    -p "${SSH_PORT}"
    -o "UserKnownHostsFile=${KNOWN_HOSTS}"
    -o StrictHostKeyChecking=yes
)
SCP_COMMON=(
    -P "${SSH_PORT}"
    -o "UserKnownHostsFile=${KNOWN_HOSTS}"
    -o StrictHostKeyChecking=yes
)

printf '\n开始一次性板端安装；此阶段可能提示 cat SSH/sudo 密码。\n'
REMOTE_TEMP="$(
    ssh "${SSH_COMMON[@]}" \
        "${ADMIN_USER}@${HOST_ADDRESS}" \
        'mktemp -d /tmp/myline-laser-setup.XXXXXX'
)"
REMOTE_TEMP="${REMOTE_TEMP//$'\r'/}"
REMOTE_TEMP="${REMOTE_TEMP//$'\n'/}"
if [[ ! "${REMOTE_TEMP}" =~ ^/tmp/myline-laser-setup\.[A-Za-z0-9]+$ ]]; then
    printf '错误: 板端返回了异常临时目录: %s\n' "${REMOTE_TEMP}" >&2
    exit 1
fi

scp "${SCP_COMMON[@]}" \
    "${DEPLOY_DIR}/line_laser_daemon.py" \
    "${DEPLOY_DIR}/line_laser_gateway.py" \
    "${DEPLOY_DIR}/line-laser-daemon.service" \
    "${DEPLOY_DIR}/install.sh" \
    "${DEPLOY_DIR}/uninstall.sh" \
    "${DEPLOY_DIR}/README.md" \
    "${PUBLIC_KEY}" \
    "${ADMIN_USER}@${HOST_ADDRESS}:${REMOTE_TEMP}/"

REMOTE_PUBLIC_KEY="${REMOTE_TEMP}/$(basename -- "${PUBLIC_KEY}")"
ssh -t "${SSH_COMMON[@]}" \
    "${ADMIN_USER}@${HOST_ADDRESS}" \
    "sudo /bin/bash '${REMOTE_TEMP}/install.sh' \
        --source-dir '${REMOTE_TEMP}' \
        --public-key-file '${REMOTE_PUBLIC_KEY}' \
        --confirm-v1-pin-map \
        --controller-cidr '${CONTROLLER_CIDR}'"

# The path is accepted only after the strict /tmp prefix validation above.
ssh "${SSH_COMMON[@]}" \
    "${ADMIN_USER}@${HOST_ADDRESS}" \
    "/bin/rm -rf -- '${REMOTE_TEMP}'" || \
    printf '警告: 请手工清理板端临时目录 %s\n' "${REMOTE_TEMP}" >&2

CONTROL_SSH=(
    -T
    -p "${SSH_PORT}"
    -i "${PRIVATE_KEY}"
    -o BatchMode=yes
    -o IdentitiesOnly=yes
    -o PasswordAuthentication=no
    -o KbdInteractiveAuthentication=no
    -o StrictHostKeyChecking=yes
    -o "UserKnownHostsFile=${KNOWN_HOSTS}"
)
STATUS_RESPONSE="$(
    printf '%s\n' '{"v":1,"id":1,"op":"status"}' |
        ssh "${CONTROL_SSH[@]}" "laserctl@${HOST_ADDRESS}"
)"
if [[ "${STATUS_RESPONSE}" != *'"ok":true'* ||
      "${STATUS_RESPONSE}" != *'"state":"off"'* ]]; then
    printf '错误: 免密码安全状态验证失败: %s\n' "${STATUS_RESPONSE}" >&2
    exit 1
fi

printf '\n安装及免密码协议验证完成。\n'
printf '私钥: %s\nknown_hosts: %s\n' "${PRIVATE_KEY}" "${KNOWN_HOSTS}"
printf '板端当前确认：Pin11/GPIO15 与 Pin7/GPIO16 均为 LOW。\n'

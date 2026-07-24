#!/usr/bin/env bash
# Install the fail-safe line-laser service on a LubanCat-4 V1.
set -euo pipefail

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PUBLIC_KEY_FILE=""
CONTROLLER_CIDR=""
ALLOW_UNKNOWN_BOARD=0
CONFIRMED_V1_PIN_MAP=0

usage() {
    cat <<'EOF'
用法:
  sudo ./install.sh --public-key-file controller_key.pub \
    --confirm-v1-pin-map [选项]

选项:
  --source-dir DIR          从指定目录安装部署文件
  --controller-cidr CIDR   将控制密钥限制到指定源 IP/CIDR
  --confirm-v1-pin-map     确认本板 Pin11=GPIO15、Pin7=GPIO16
  --allow-unknown-board    仅开发调试：跳过 LubanCat-4 型号检查

安装只会将两路置为 LOW，不会自动点亮激光。安装前仍须确认板型为
LubanCat-4 V1，且 Pin11=GPIO15、Pin7=GPIO16。
EOF
}

while (($# > 0)); do
    case "$1" in
        --source-dir)
            SOURCE_DIR="${2:?--source-dir requires a directory}"
            shift 2
            ;;
        --public-key-file)
            PUBLIC_KEY_FILE="${2:?--public-key-file requires a path}"
            shift 2
            ;;
        --controller-cidr)
            CONTROLLER_CIDR="${2:?--controller-cidr requires an address}"
            shift 2
            ;;
        --confirm-v1-pin-map)
            CONFIRMED_V1_PIN_MAP=1
            shift
            ;;
        --allow-unknown-board)
            ALLOW_UNKNOWN_BOARD=1
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

if [[ "${EUID}" -ne 0 ]]; then
    printf '错误: 安装必须以 root 运行，请使用 sudo。\n' >&2
    exit 1
fi
if [[ -z "${PUBLIC_KEY_FILE}" || ! -f "${PUBLIC_KEY_FILE}" ]]; then
    printf '错误: 必须提供存在的 --public-key-file。\n' >&2
    exit 2
fi
if [[ "${CONFIRMED_V1_PIN_MAP}" -ne 1 ]]; then
    printf '%s\n' \
        '错误: 必须显式提供 --confirm-v1-pin-map，防止在非 V1 板上驱动错误引脚。' >&2
    exit 2
fi
for required in line_laser_daemon.py line_laser_gateway.py line-laser-daemon.service README.md; do
    if [[ ! -f "${SOURCE_DIR}/${required}" ]]; then
        printf '错误: 缺少部署文件 %s/%s\n' "${SOURCE_DIR}" "${required}" >&2
        exit 1
    fi
done
for command in install groupadd useradd usermod runuser systemctl sshd python3; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        printf '错误: 缺少命令 %s\n' "${command}" >&2
        exit 1
    fi
done

BOARD_MODEL="$(tr -d '\000' </proc/device-tree/model 2>/dev/null || true)"
if [[ "${ALLOW_UNKNOWN_BOARD}" -ne 1 && "${BOARD_MODEL}" != *LubanCat-4* ]]; then
    printf '错误: 检测到的设备不是 LubanCat-4: %s\n' "${BOARD_MODEL:-未知}" >&2
    exit 1
fi
if [[ ! -c /dev/gpiochip0 ]]; then
    printf '错误: GPIO 字符设备不存在: /dev/gpiochip0\n' >&2
    exit 1
fi
if [[ -n "${CONTROLLER_CIDR}" &&
      ! "${CONTROLLER_CIDR}" =~ ^[0-9A-Fa-f:.]+(/[0-9]{1,3})?$ ]]; then
    printf '错误: --controller-cidr 格式不安全: %s\n' "${CONTROLLER_CIDR}" >&2
    exit 2
fi

PUBLIC_KEY="$(head -n 1 -- "${PUBLIC_KEY_FILE}")"
if [[ ! "${PUBLIC_KEY}" =~ ^ssh-ed25519[[:space:]][A-Za-z0-9+/=]+([[:space:]].*)?$ ]]; then
    printf '错误: 控制密钥必须是单行 ssh-ed25519 公钥。\n' >&2
    exit 1
fi

python3 -m py_compile \
    "${SOURCE_DIR}/line_laser_daemon.py" \
    "${SOURCE_DIR}/line_laser_gateway.py"

if systemctl list-unit-files line-laser-daemon.service >/dev/null 2>&1; then
    systemctl stop line-laser-daemon.service >/dev/null 2>&1 || true
fi

# Release GPIOs left exported by the old commissioning script.  Always drive
# LOW first; never unexport a HIGH laser TTL.
if [[ -w /sys/class/gpio/unexport ]]; then
    for gpio in 15 16; do
        gpio_dir="/sys/class/gpio/gpio${gpio}"
        if [[ -d "${gpio_dir}" ]]; then
            if [[ -w "${gpio_dir}/direction" ]]; then
                # "low" selects output mode and LOW as one operation, avoiding
                # an unwanted HIGH pulse while taking over an old export.
                printf 'low' >"${gpio_dir}/direction" || true
            fi
            if [[ -w "${gpio_dir}/value" ]]; then
                printf '0' >"${gpio_dir}/value" || true
            fi
            printf '%s' "${gpio}" >/sys/class/gpio/unexport || true
        fi
    done
fi

if ! getent group line-laser-control >/dev/null; then
    groupadd --system line-laser-control
fi
if ! id laserctl >/dev/null 2>&1; then
    useradd --system --create-home \
        --home-dir /var/lib/line-laser-ssh \
        --shell /bin/bash \
        --gid line-laser-control \
        laserctl
else
    usermod --home /var/lib/line-laser-ssh \
        --shell /bin/bash \
        --gid line-laser-control \
        laserctl
fi

install -d -o root -g root -m 0755 /usr/local/libexec/myline-hik
install -o root -g root -m 0755 \
    "${SOURCE_DIR}/line_laser_daemon.py" \
    /usr/local/libexec/myline-hik/line_laser_daemon.py
install -o root -g root -m 0755 \
    "${SOURCE_DIR}/line_laser_gateway.py" \
    /usr/local/libexec/myline-hik/line_laser_gateway.py
install -d -o root -g root -m 0755 /usr/local/share/doc/myline-hik-line-laser
install -o root -g root -m 0644 \
    "${SOURCE_DIR}/README.md" \
    /usr/local/share/doc/myline-hik-line-laser/README.md
install -o root -g root -m 0644 \
    "${SOURCE_DIR}/line-laser-daemon.service" \
    /etc/systemd/system/line-laser-daemon.service

install -d -o laserctl -g line-laser-control -m 0750 \
    /var/lib/line-laser-ssh
install -d -o laserctl -g line-laser-control -m 0700 \
    /var/lib/line-laser-ssh/.ssh
AUTHORIZED_PREFIX='restrict,command="/usr/local/libexec/myline-hik/line_laser_gateway.py"'
if [[ -n "${CONTROLLER_CIDR}" ]]; then
    AUTHORIZED_PREFIX="from=\"${CONTROLLER_CIDR}\",${AUTHORIZED_PREFIX}"
fi
printf '%s %s\n' "${AUTHORIZED_PREFIX}" "${PUBLIC_KEY}" \
    >/var/lib/line-laser-ssh/.ssh/authorized_keys
chown laserctl:line-laser-control \
    /var/lib/line-laser-ssh/.ssh/authorized_keys
chmod 0600 /var/lib/line-laser-ssh/.ssh/authorized_keys

install -d -o root -g root -m 0755 /etc/ssh/sshd_config.d
cat >/etc/ssh/sshd_config.d/90-line-laser-control.conf <<'EOF'
Match User laserctl
    AuthenticationMethods publickey
    PasswordAuthentication no
    KbdInteractiveAuthentication no
    PubkeyAuthentication yes
    PermitTTY no
    X11Forwarding no
    AllowAgentForwarding no
    AllowTcpForwarding no
    PermitTunnel no
    ForceCommand /usr/local/libexec/myline-hik/line_laser_gateway.py
EOF
chmod 0644 /etc/ssh/sshd_config.d/90-line-laser-control.conf

/usr/sbin/sshd -t
systemctl daemon-reload
systemctl enable --now line-laser-daemon.service
systemctl reload ssh.service 2>/dev/null || systemctl reload sshd.service

STATUS_RESPONSE="$(
    printf '%s\n' '{"v":1,"id":1,"op":"status"}' |
        runuser -u laserctl -- \
            /usr/local/libexec/myline-hik/line_laser_gateway.py
)"
if [[ "${STATUS_RESPONSE}" != *'"ok":true'* ||
      "${STATUS_RESPONSE}" != *'"state":"off"'* ]]; then
    printf '错误: 服务已安装，但安全状态检查失败: %s\n' \
        "${STATUS_RESPONSE}" >&2
    systemctl stop line-laser-daemon.service || true
    exit 1
fi

printf '安装完成：%s\n' "${BOARD_MODEL:-型号检查已跳过}"
printf 'Pin11/GPIO15(450nm) 与 Pin7/GPIO16(650nm) 当前均为 LOW。\n'
printf '日常控制用户: laserctl；仅允许受限公钥协议，无密码和远程 shell。\n'

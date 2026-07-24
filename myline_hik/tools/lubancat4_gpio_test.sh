#!/usr/bin/env bash
#
# Remotely control LubanCat-4 V1 physical header pins 7 and 11 over SSH.
#
# LubanCat-4 V1 40-pin mapping (different from the original LubanCat-4):
#   physical pin 7  -> GPIO0_C0 -> Linux GPIO 16
#   physical pin 11 -> GPIO0_B7 -> Linux GPIO 15
#
# High level is nominally 3.3 V. Never apply 5 V to either GPIO pin.

set -euo pipefail

readonly DEFAULT_HOST="192.168.1.12"
readonly DEFAULT_USER="cat"
readonly DEFAULT_PORT="22"

HOST="${LUBANCAT_HOST:-$DEFAULT_HOST}"
SSH_USER="${LUBANCAT_USER:-$DEFAULT_USER}"
SSH_PORT="${LUBANCAT_PORT:-$DEFAULT_PORT}"

usage() {
    cat <<'EOF'
用法:
  ./tools/lubancat4_gpio_test.sh <7|11|all> <high|low|pulse|cycle|status|release> [秒数]

适用板型:
  LubanCat-4 V1（物理 Pin 7=GPIO0_C0，物理 Pin 11=GPIO0_B7）

动作:
  high     持续输出高电平（约 3.3 V）
  low      持续输出低电平（约 0 V）
  pulse    输出高电平指定秒数，然后回到低电平
  cycle    依次测试 Pin 7、Pin 11；结束后两路均为低电平
  status   查看当前导出状态、方向和逻辑电平
  release  先拉低，再切为输入并释放 GPIO

示例:
  ./tools/lubancat4_gpio_test.sh 7 high
  ./tools/lubancat4_gpio_test.sh 7 low
  ./tools/lubancat4_gpio_test.sh 11 pulse 2
  ./tools/lubancat4_gpio_test.sh all cycle 2
  ./tools/lubancat4_gpio_test.sh all status
  ./tools/lubancat4_gpio_test.sh all release

连接参数可通过环境变量覆盖:
  LUBANCAT_HOST=192.168.1.12
  LUBANCAT_USER=cat
  LUBANCAT_PORT=22

官方镜像默认 SSH 用户为 cat。脚本不会保存密码；SSH 和 sudo 会按需提示输入。
EOF
}

if (( $# < 2 || $# > 3 )); then
    usage
    exit 2
fi

TARGET="$1"
ACTION="$2"
DELAY="${3:-2}"

case "$TARGET" in
    7|11|all) ;;
    *)
        printf '错误: 引脚只能是 7、11 或 all，当前值: %s\n' "$TARGET" >&2
        exit 2
        ;;
esac

case "$ACTION" in
    high|low|pulse|cycle|status|release) ;;
    *)
        printf '错误: 不支持的动作: %s\n' "$ACTION" >&2
        usage
        exit 2
        ;;
esac

if [[ "$TARGET" == "all" &&
      ( "$ACTION" == "high" || "$ACTION" == "pulse" ) ]]; then
    printf '%s\n' \
        '错误: 两路激光 TTL 禁止同时 HIGH；请一次只测试 Pin 7 或 Pin 11。' >&2
    exit 2
fi

if [[ ! "$DELAY" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    printf '错误: 秒数必须是非负数字，当前值: %s\n' "$DELAY" >&2
    exit 2
fi

read -r -d '' REMOTE_SCRIPT <<'REMOTE_EOF' || true
set -eu

target="$1"
action="$2"
delay="$3"

PIN7_GPIO=16
PIN11_GPIO=15
GPIO_ROOT=/sys/class/gpio

model="$(tr -d '\000' </proc/device-tree/model 2>/dev/null || true)"
case "$model" in
    *LubanCat-4*) ;;
    *)
        printf '错误: 远端设备不是 LubanCat-4/V1（检测到: %s）\n' "${model:-未知}" >&2
        exit 1
        ;;
esac

if [ ! -w "$GPIO_ROOT/export" ] || [ ! -w "$GPIO_ROOT/unexport" ]; then
    printf '错误: 板端 sysfs GPIO 接口不可用: %s\n' "$GPIO_ROOT" >&2
    exit 1
fi

gpio_number() {
    case "$1" in
        7) printf '%s\n' "$PIN7_GPIO" ;;
        11) printf '%s\n' "$PIN11_GPIO" ;;
        *) return 1 ;;
    esac
}

export_gpio() {
    number="$1"
    gpio_dir="$GPIO_ROOT/gpio$number"

    if [ ! -d "$gpio_dir" ]; then
        printf '%s' "$number" >"$GPIO_ROOT/export"
        count=0
        while [ ! -d "$gpio_dir" ]; do
            count=$((count + 1))
            if [ "$count" -ge 20 ]; then
                printf '错误: GPIO %s 导出超时\n' "$number" >&2
                return 1
            fi
            sleep 0.05
        done
    fi
}

set_pin() {
    pin="$1"
    value="$2"
    number="$(gpio_number "$pin")"
    gpio_dir="$GPIO_ROOT/gpio$number"

    export_gpio "$number"

    if [ "$(cat "$gpio_dir/direction")" != "out" ]; then
        # "high"/"low" selects output direction and its initial value together,
        # avoiding a short unwanted pulse while changing direction.
        if [ "$value" = "1" ]; then
            printf 'high' >"$gpio_dir/direction"
        else
            printf 'low' >"$gpio_dir/direction"
        fi
    else
        printf '%s' "$value" >"$gpio_dir/value"
    fi

    actual="$(cat "$gpio_dir/value")"
    if [ "$actual" != "$value" ]; then
        printf '错误: Pin %s 写入 %s 后读回 %s\n' "$pin" "$value" "$actual" >&2
        return 1
    fi

    if [ "$value" = "1" ]; then
        printf 'Pin %-2s (GPIO %s) -> HIGH，标称约 3.3 V\n' "$pin" "$number"
    else
        printf 'Pin %-2s (GPIO %s) -> LOW，标称约 0 V\n' "$pin" "$number"
    fi
}

show_pin() {
    pin="$1"
    number="$(gpio_number "$pin")"
    gpio_dir="$GPIO_ROOT/gpio$number"

    if [ ! -d "$gpio_dir" ]; then
        printf 'Pin %-2s (GPIO %s): 未导出（高阻/由其他驱动管理）\n' "$pin" "$number"
        return
    fi

    direction="$(cat "$gpio_dir/direction")"
    value="$(cat "$gpio_dir/value")"
    printf 'Pin %-2s (GPIO %s): direction=%s, value=%s\n' \
        "$pin" "$number" "$direction" "$value"
}

release_pin() {
    pin="$1"
    number="$(gpio_number "$pin")"
    gpio_dir="$GPIO_ROOT/gpio$number"

    if [ ! -d "$gpio_dir" ]; then
        printf 'Pin %-2s (GPIO %s) 已经处于释放状态\n' "$pin" "$number"
        return
    fi

    if [ "$(cat "$gpio_dir/direction")" = "out" ]; then
        printf '0' >"$gpio_dir/value"
    fi
    printf 'in' >"$gpio_dir/direction"
    printf '%s' "$number" >"$GPIO_ROOT/unexport"
    printf 'Pin %-2s (GPIO %s) -> LOW 后已释放\n' "$pin" "$number"
}

selected_pins() {
    case "$target" in
        7) printf '7\n' ;;
        11) printf '11\n' ;;
        all) printf '7\n11\n' ;;
    esac
}

printf '已连接: %s (%s)\n' "$model" "$(hostname)"
printf '板型映射: LubanCat-4 V1，Pin 7=GPIO 16，Pin 11=GPIO 15\n'

case "$action" in
    high)
        # Break before make: a single-channel HIGH first forces the other
        # laser channel LOW.
        if [ "$target" = "7" ]; then
            set_pin 11 0
        else
            set_pin 7 0
        fi
        for pin in $(selected_pins); do
            set_pin "$pin" 1
        done
        ;;
    low)
        for pin in $(selected_pins); do
            set_pin "$pin" 0
        done
        ;;
    pulse)
        if [ "$target" = "7" ]; then
            set_pin 11 0
        else
            set_pin 7 0
        fi
        for pin in $(selected_pins); do
            set_pin "$pin" 1
        done
        printf '保持 %s 秒...\n' "$delay"
        sleep "$delay"
        for pin in $(selected_pins); do
            set_pin "$pin" 0
        done
        ;;
    cycle)
        # Start and finish in the safe LOW state, then test each output alone.
        set_pin 7 0
        set_pin 11 0
        for pin in 7 11; do
            printf '%s\n' "--- 测试 Pin $pin ---"
            set_pin "$pin" 1
            sleep "$delay"
            set_pin "$pin" 0
            sleep "$delay"
        done
        printf '%s\n' '循环测试完成；Pin 7 和 Pin 11 均保持 LOW。'
        ;;
    status)
        for pin in $(selected_pins); do
            show_pin "$pin"
        done
        ;;
    release)
        for pin in $(selected_pins); do
            release_pin "$pin"
        done
        ;;
esac
REMOTE_EOF

REMOTE_SCRIPT_B64="$(printf '%s' "$REMOTE_SCRIPT" | base64 | tr -d '\n')"

printf '连接 %s@%s:%s ...\n' "$SSH_USER" "$HOST" "$SSH_PORT"
printf '提示: GPIO 高电平是逻辑输出，标称约 3.3 V；请勿接入 5 V。\n'

ssh \
    -tt \
    -p "$SSH_PORT" \
    -o ConnectTimeout=5 \
    -o ServerAliveInterval=10 \
    -o ServerAliveCountMax=3 \
    -o StrictHostKeyChecking=accept-new \
    "${SSH_USER}@${HOST}" \
    "printf '%s' '$REMOTE_SCRIPT_B64' | base64 -d | sudo bash -s -- '$TARGET' '$ACTION' '$DELAY'"

#!/usr/bin/env bash
# Remove only the dedicated line-laser service and SSH account.
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    printf '错误: 卸载必须以 root 运行，请使用 sudo。\n' >&2
    exit 1
fi

# Ask the live daemon to go LOW before stopping it.  Service shutdown repeats
# the LOW write, so either path is safe if one fails.
if [[ -S /run/myline-hik/laser.sock &&
      -x /usr/local/libexec/myline-hik/line_laser_gateway.py ]]; then
    printf '%s\n' '{"v":1,"id":1,"op":"off"}' |
        /usr/local/libexec/myline-hik/line_laser_gateway.py \
        >/dev/null 2>&1 || true
fi

systemctl disable --now line-laser-daemon.service >/dev/null 2>&1 || true

# Best effort: retain both lines as LOW through the remainder of this boot.
# A physical pull-down is still required for reboot, power loss, and early boot.
if [[ -w /sys/class/gpio/export ]]; then
    for gpio in 15 16; do
        gpio_dir="/sys/class/gpio/gpio${gpio}"
        if [[ ! -d "${gpio_dir}" ]]; then
            printf '%s' "${gpio}" >/sys/class/gpio/export 2>/dev/null || true
            for _attempt in {1..20}; do
                [[ -d "${gpio_dir}" ]] && break
                sleep 0.05
            done
        fi
        if [[ -w "${gpio_dir}/direction" ]]; then
            printf 'low' >"${gpio_dir}/direction" 2>/dev/null || true
        fi
        if [[ -w "${gpio_dir}/value" ]]; then
            printf '0' >"${gpio_dir}/value" 2>/dev/null || true
        fi
    done
fi

rm -f /etc/systemd/system/line-laser-daemon.service
rm -f /etc/ssh/sshd_config.d/90-line-laser-control.conf
rm -rf /usr/local/libexec/myline-hik
rm -rf /usr/local/share/doc/myline-hik-line-laser
rm -rf /run/myline-hik

if id laserctl >/dev/null 2>&1; then
    userdel --remove laserctl >/dev/null 2>&1 || userdel laserctl
fi
if getent group line-laser-control >/dev/null; then
    groupdel line-laser-control >/dev/null 2>&1 || true
fi

systemctl daemon-reload
/usr/sbin/sshd -t
systemctl reload ssh.service 2>/dev/null || systemctl reload sshd.service

printf '卸载完成。两路已尽力保持 LOW；重启前后仍必须依赖硬件下拉保证关光。\n'

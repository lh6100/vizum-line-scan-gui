# LubanCat-4 V1 双线激光 GPIO 服务

本目录部署一个最小权限、失效关光的控制通道：

```text
Qt GUI -> 持久 ssh -T -> forced-command gateway
       -> /run/myline-hik/laser.sock
       -> systemd Python daemon -> /dev/gpiochip0
```

固定映射：

| 激光 | 物理引脚 | RK GPIO | gpiochip0 offset | TTL 开启 |
| --- | ---: | ---: | ---: | --- |
| 450 nm | Pin 11 | GPIO15 | 15 | HIGH |
| 650 nm | Pin 7 | GPIO16 | 16 | HIGH |

这里必须使用
[野火官方 LubanCat-4-V1 专用 40Pin 图](https://doc.embedfire.com/linux/rk3588/quick_start/zh/latest/quick_start/40pin/40pin.html#lubancat-4-v1)：
V1 的 Pin11 是 GPIO0_B7（offset 15），Pin7 是 GPIO0_C0（offset 16）；普通
LubanCat-4 的同号物理引脚映射不同，不能混用。

daemon 用一个 GPIO character-device handle 同时申请两路，任何一次写入都只允许
`off`、`laser450`、`laser650` 三种状态。两路不会同时为 HIGH；从一个波长切换到
另一个波长时严格按“当前通道 LOW → 回读两路 LOW → 目标通道 HIGH”执行。

## 失效行为

- daemon 启动时首先将两路设为 LOW。
- 新控制连接握手时先将两路设为 LOW，不会恢复上次开启状态。
- SSH/Unix socket 连接结束时立即 LOW。
- GUI 每 500 ms 发心跳；2 秒没有有效心跳时 daemon 自动 LOW 并释放控制租约。
- daemon 收到 SIGTERM、服务停止或正常退出时再次 LOW。
- 只有一个连接可以持有控制租约，第二个连接不能抢占。

GPIO 逻辑读回不能证明排针电压或激光实际出光。投入使用前必须断开激光，用万用表或
示波器分别验收 Pin11、Pin7 相对 GND 的 LOW/HIGH；随后逐路连接 TTL 验证。

软件不是安全回路。每路 TTL 必须有约 10 kΩ 硬件下拉，激光设备还应使用符合风险等级
的物理钥匙、遮光罩、急停或安全继电器。否则在断电、内核崩溃和启动早期无法保证关光。

## 安装

推荐从控制电脑执行仓库根目录下的：

```bash
./tools/setup_laser_control.sh
```

它会生成一把只用于激光协议的无口令 Ed25519 密钥，要求人工确认板端 SSH host key，
然后通过现有 `cat` 管理账户执行一次需要 sudo 的安装。安装后日常 GUI 操作不再输入
SSH 或 sudo 密码。

也可以在板端手工安装：

```bash
sudo ./install.sh --public-key-file /path/controller_key.pub \
  --confirm-v1-pin-map \
  --controller-cidr '<控制电脑IPv4>/32'
```

`<控制电脑IPv4>` 必须替换为板端实际看到的控制电脑源地址，不能照抄示例。优先使用上面的
`setup_laser_control.sh`，它会从到板端的路由自动探测该地址。

安装程序不会自动开启任何激光。它创建专用 `laserctl` 账号，该账号没有普通 SSH
shell，密钥被 `restrict`、`ForceCommand` 和禁止转发配置共同限制为本协议。

## 卸载

```bash
sudo ./uninstall.sh
```

卸载会先请求两路 LOW，停止服务，并尽力通过 sysfs 在本次开机期间继续保持 LOW；仍不
能替代硬件下拉。卸载只删除专用账号、密钥授权、systemd 单元和本服务文件，不修改
`cat` 管理账户或板端 SSH host key。

## 协议

每行一个最大 4096 字节的 UTF-8 JSON 对象，协议版本固定为 1：

```json
{"v":1,"id":1,"op":"hello","client":"scan-gui"}
{"v":1,"id":2,"op":"set","state":"laser450"}
{"v":1,"id":3,"op":"heartbeat"}
{"v":1,"id":4,"op":"off"}
{"v":1,"id":5,"op":"status"}
{"v":1,"id":6,"op":"goodbye"}
```

回复始终带相同 `id`、`ok` 和完整状态。状态中的 `ttl450_high`、`ttl650_high` 是 GPIO
逻辑读回，不是光学反馈。

## 不接硬件的测试

```bash
python3 -m py_compile line_laser_daemon.py line_laser_gateway.py
python3 test_line_laser_daemon.py -v
```

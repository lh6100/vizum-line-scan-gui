# 新电脑部署与建图

本仓库是一个 monorepo：`myline_hik` 保留已验证的 60 FPS 连续建图，
`ros2_fr5` 负责 FR5/MoveIt/操作台/RViz，并复用同一套 scanner 650 标定和
legacy 条纹中心算法。

## 1. 前置条件

建议系统为 Ubuntu 22.04 + ROS 2 Humble。首先按 ROS 官方方式安装 ROS 2，
然后安装项目依赖：

```bash
git clone https://github.com/lh6100/vizum-line-scan-gui.git
cd vizum-line-scan-gui
./scripts/install_ubuntu_dependencies.sh
```

两个设备 SDK 不能用空文件替代：

1. 安装 Hikrobot MVS Linux SDK，默认根目录为 `/opt/MVS`，其下需有
   `include/MvCameraControl.h` 和 `lib/64/libMvCameraControl.so`。
2. 将 FAIRINO C++ SDK 3.9.4 解压到本机任意目录。`myline_hik` 需要其完整
   `libfairino` 源码，以打入项目所需的 20004 实时回调补丁。

复制并修改本机环境：

```bash
cp .env.example .env.local
# 编辑 FAIRINO_SDK_DIR，必要时编辑 HIK_MVS_ROOT
set -a
source .env.local
set +a
./scripts/check_environment.sh
```

`.env.local`、SDK、SSH 私钥、点云、图像和 rosbag 都不会进入 Git。

## 2. 构建和验证 myline_hik

```bash
cmake -S myline_hik -B myline_hik/build -DCMAKE_BUILD_TYPE=Release
cmake --build myline_hik/build -j"$(nproc)"
ctest --test-dir myline_hik/build --output-on-failure
```

现场 GUI 启动脚本会规避 Conda Qt/OpenCV ABI 混用，并在找到 FAIRINO SDK
源码时自动应用/构建实时回调补丁：

```bash
./myline_hik/run_constant_laser_scan.sh
# 或标定 GUI
./myline_hik/run_calibration_gui.sh
```

正式标定位于 `myline_hik/config/devices/scanner_450` 和 `scanner_650`。不要用其他
电脑上的临时 YAML 覆盖它们。scanner 650 手眼文件保留了历史内参哈希；
现场启动脚本默认使用当前数值并记录该差异。

## 3. 构建 ROS 2 工作区

```bash
cd ros2_fr5
./fetch_fairino_ros2.sh
rosdep install --from-paths src \
  frcobot_ros2-master/fairino_msgs \
  frcobot_ros2-master/fairino_hardware_v3_9_4 \
  frcobot_ros2-master/fairino5_v6_moveit2_config \
  --ignore-src -r -y
./build_ros2.sh
```

`build_ros2.sh` 只构建需要的 FR5 包，不会扫描工作区外的其他实验项目。
构建时会把 scanner 650 正式标定安装到 ROS 包的 share 目录，因此启动后
不依赖原开发机的绝对路径。默认按包顺序构建且每包最多 2 个编译任务，
避免 FAIRINO 大型源文件与 MoveIt 同时编译时内存不足。高配电脑可覆盖
`CMAKE_BUILD_PARALLEL_LEVEL` 和 `COLCON_EXECUTOR`。

先运行仿真硬件模式：

```bash
./run_scanner_650_moveit_mock.sh
```

真机启动前，把激光控制机 SSH 私钥和 `known_hosts` 放到：

```text
${VIZUM_CONFIG_DIR:-$HOME/.config/myline_hik}/laser-control/id_ed25519
${VIZUM_CONFIG_DIR:-$HOME/.config/myline_hik}/laser-control/known_hosts
```

检查机器人、负载、TCP、电缆、工装/工件碰撞模型和急停后，再显式解锁：

```bash
CONFIRM_FR5_HARDWARE=YES ./run_scanner_650_moveit_hardware.sh
```

## 4. 运行时目录

通过仓库脚本启动时，ROS 点云默认保存到
`myline_hik/data/scans/scanner_650/ros2_scan_cloud.ply`。可以在 `.env.local` 中设置
`VIZUM_DATA_DIR` 把大数据放到其他磁盘。直接调用已安装 launch 时，默认遵循
`XDG_DATA_HOME/vizum-line-scan-gui`。

## 5. 公开发布注意

本仓库不再分发许可不明的 FAIRINO ROS 适配源码或 `libfairino` 二进制。
`build_ros2.sh` 首次运行时会从 FAIRINO 官方 GitHub 稀疏拉取三个所需包，并固定在
commit `5bed0b0263c8f1e95f51aa45079f904d463c5c50`。离线部署时，先在有网络的
机器运行 `ros2_fr5/fetch_fairino_ros2.sh`，再将生成的 `frcobot_ros2-master` 目录随
离线安装包传递。详见 `ros2_fr5/vendor/README.md`。

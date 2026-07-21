# FR5、焊枪 TCP 与 Vizum 相机 RViz 显示（ROS 2 Humble）

这套 ROS 2 工作区只读连接 Fairino FR5，完成三件事：

1. 读取真实机械臂的 6 轴关节角，在 RViz2 中实时更新官方 FR5 模型。
2. 读取控制器报告的法兰 `XYZ/RPY`，同时发布标准 ROS 位姿、原始毫米/度消息、TF，并在 RViz 中显示坐标文字。
3. 从项目 `config/` 读取焊枪 TCP 和 Vizum 左目相机手眼标定，在 RViz 中显示随法兰运动的坐标轴、彩色原点和文字标签。

节点不会上伺服、切换模式或发送运动指令。

## 一、文件结构

```text
ros2_fr5/src/
├── fairino_description/   # Fairino 官方 FR5 URDF 和 STL meshes
├── fr5_vizum_msgs/        # 原始法兰毫米/度消息
├── fr5_vizum_driver/      # 只读 Fairino SDK 实时状态节点
└── fr5_vizum_bringup/     # robot_state_publisher、标定 TF/点、RViz 和 launch
```

launch 的默认控制器地址与项目 `config/robot_config.yaml` 当前配置一致：`192.168.1.200`。

## 二、首次构建

环境要求：Ubuntu 22.04、ROS 2 Humble、RViz2，以及本项目已有的 Fairino C++ SDK：

```text
/home/zhulong/lh/vizum-line-scan-gui/SDK/fairino-cpp-sdk-3.9.4
```

构建：

```bash
cd /home/zhulong/lh/vizum-line-scan-gui/ros2_fr5
./build_ros2.sh
```

如果 SDK 放在其他位置，确保头文件和动态库来自同一个 SDK 版本，再这样构建：

```bash
FAIRINO_SDK_DIR=/absolute/path/to/fairino-cpp-sdk ./build_ros2.sh
```

## 三、启动真实 FR5 和 RViz

```bash
cd /home/zhulong/lh/vizum-line-scan-gui/ros2_fr5
./run_fr5_live_rviz.sh robot_ip:=192.168.1.200
```

启动后，终端每秒输出一次法兰原始坐标，RViz 显示：

- 随真实关节状态运动的 FR5 模型；
- `fairino_flange_reported` 法兰坐标轴；
- `weld_gun_tcp` 焊枪 TCP：橙红色原点、坐标轴和标签；
- `vizum_left_camera_optical_frame` Vizum 左目光心：蓝色原点、坐标轴和标签；
- 法兰旁的实时 `XYZ mm / RPY deg` 文字。

无图形界面时不启动 RViz，仅启动机器人模型与状态发布相关节点：

```bash
./run_fr5_live_rviz.sh use_rviz:=false
```

不使用脚本时，等价命令是：

```bash
source /opt/ros/humble/setup.bash
source /home/zhulong/lh/vizum-line-scan-gui/ros2_fr5/install/setup.bash
ros2 launch fr5_vizum_bringup fr5_live_rviz.launch.py \
  robot_ip:=192.168.1.200
```

## 四、实时法兰坐标接口

| Topic / TF | 类型 | 单位与用途 |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节位置为 rad，驱动 RViz 机器人模型 |
| `/fairino/flange_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` 下的位置为 m，姿态为 quaternion |
| `/fairino/flange_pose_mm_deg` | `fr5_vizum_msgs/msg/FlangePose` | 控制器原始易读值：XYZ mm，固定轴 RPY deg |
| `/fairino/flange_marker` | `visualization_msgs/msg/Marker` | RViz 中的实时法兰坐标文字 |
| `/fairino/calibrated_points` | `visualization_msgs/msg/MarkerArray` | 焊枪 TCP 与相机原点的彩色球和文字标签 |
| `base_link -> fairino_flange_reported` | TF | 控制器报告的实时法兰坐标系 |
| `fairino_flange_reported -> weld_gun_tcp` | TF Static | `config/tool_config.yaml` 中标定的法兰到焊枪 TCP |
| `fairino_flange_reported -> vizum_left_camera_optical_frame` | TF Static | `config/handeye_config.yaml` 中标定的法兰到 Vizum 左目光心 |

终端查看原始法兰坐标：

```bash
source /opt/ros/humble/setup.bash
source /home/zhulong/lh/vizum-line-scan-gui/ros2_fr5/install/setup.bash
ros2 topic echo /fairino/flange_pose_mm_deg
```

查看标准 ROS 位姿和 TF：

```bash
ros2 topic echo --once /fairino/flange_pose
ros2 run tf2_ros tf2_echo base_link fairino_flange_reported
ros2 run tf2_ros tf2_echo fairino_flange_reported weld_gun_tcp
ros2 run tf2_ros tf2_echo fairino_flange_reported vizum_left_camera_optical_frame
ros2 topic hz /joint_states
```

## 五、常用启动参数

```bash
./run_fr5_live_rviz.sh \
  robot_ip:=192.168.1.200 \
  publish_rate_hz:=20.0 \
  log_pose_period_ms:=1000 \
  publish_flange_tf:=true \
  publish_flange_marker:=true \
  publish_calibrated_frames:=true \
  publish_calibrated_markers:=true
```

- `publish_rate_hz`：关节和法兰发布频率，范围 0.1–200 Hz，默认 20 Hz。
- `log_pose_period_ms`：终端坐标打印周期；设为 `0` 关闭。
- `publish_calibrated_frames`：是否读取标定配置并发布 TCP/相机静态 TF。
- `publish_calibrated_markers`：是否发布 TCP/相机彩色原点和文字；坐标轴由 RViz 的 TF/Axes 显示。
- `use_rviz`：是否启动 RViz2。

## 六、重要限制与排查

### 标定点的数据来源

启动时直接读取项目根目录下两份当前配置，不使用两个全零的 `*_offset_template.csv`：

- `/home/zhulong/lh/vizum-line-scan-gui/config/tool_config.yaml`：`T_flange_tcp`，平移单位 mm、固定轴 RPY 单位 deg，当前为 `tool_id: 4`；
- `/home/zhulong/lh/vizum-line-scan-gui/config/handeye_config.yaml`：手眼矩阵单位 mm。当前 `mode: camera_to_flange` 按项目既有约定表示矩阵本身就是 `T_flange_camera`。

相机标定点是 **Vizum 左目光心/左相机坐标原点**，不是相机外壳中心、双目中点或激光器原点。配置缺键、含非有限数、旋转矩阵非刚体或模式非法时，标定显示节点会报错并让 launch 退出，不会悄悄使用默认值。

`tool_config.yaml` 是项目内保存的工具 4 标定值；控制器当前激活的工具号可能不同。本界面按你的要求显示配置文件中的焊枪 TCP，不把它冒充成控制器活动工具的在线核验值。

如需使用其他标定文件，可显式覆盖：

```bash
./run_fr5_live_rviz.sh \
  tool_config_path:=/absolute/path/to/tool_config.yaml \
  handeye_config_path:=/absolute/path/to/handeye_config.yaml
```

### 同一时间只能有一个 Fairino SDK 连接所有者

不要同时运行以下程序：

- 本 ROS 2 `fairino_state_publisher`；
- VizumScanGUI 中已连接机械臂的 `FairinoRobotClient`；
- Fairino 官方 `ros2_cmd_server`；
- 其他直接调用 `FRRobot::RPC()` 的程序。

第二个连接可能出现 `FRRobot::RPC failed, err=-2`。需要 GUI 和 RViz 同时运行时，应让一个进程独占机器人连接，再通过 ROS topic 共享状态。

### RViz 模型末端与报告法兰

Fairino 官方 `fairino5_v6.urdf` 只定义到 `wrist3_link`，没有独立的法兰 link。本实现因此保留官方模型不猜测机械尺寸，并把控制器报告值发布成独立的 `fairino_flange_reported` TF；没有把估算偏移硬编码进 URDF。

### 网络检查

```bash
ping -c 3 192.168.1.200
```

若节点连接或读取失败，launch 会退出而不会在同一进程中自动重连。先确认电脑和控制器在同一网段、其他 Fairino 程序已经退出，再重新执行启动命令。这样可以避免 3.9.4 SDK 的分离后台线程在进程内重复创建。

## 七、实现说明

节点在同一个 `FRRobot` 连接和同一个定时回调中读取：

- `GetActualJointPosDegree()`：关节角 deg 转 rad；
- `GetActualToolFlangePose()`：XYZ mm 转 m，固定轴 X/Y/Z 角度转 quaternion。

Fairino 3.9.4 SDK 的这两个读取函数都只复制后台实时线程维护的状态缓存，不会发出两次网络 RPC。没有直接调用 `GetRobotRealTimeState()`，因为现场 SDK 动态库与其整包结构体头文件存在 ABI 尺寸不一致，直接调用会触发内存保护退出。

标定显示节点不访问 Fairino SDK。它只从 YAML 加载 `T_flange_tcp` 与 `T_flange_camera`，发布 `fairino_flange_reported` 的两个静态子 TF 和 RViz Marker；因此不会创建第二个机器人连接。

该 SDK 还使用无法 `join` 的分离后台线程。退出时节点会先调用 `CloseRPC()`，再让单个 `FRRobot` 对象保留到进程结束，由操作系统回收，避免关闭阶段的 SDK 线程访问已析构对象。

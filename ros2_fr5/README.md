# FR5、焊枪 TCP、双 HIK 相机与线激光平面 RViz 显示（ROS 2 Humble）

这套 ROS 2 工作区包含两条相互独立的链路：原有 `fr5_vizum_bringup`
只读显示真实 FR5；新增 `fr5_scanner_650` 通过独立 Qt 操作台、MoveIt 2、Pilz LIN 和
`ros2_control` 控制单相机单线激光扫描。RViz 只承担机器人、碰撞场景、规划轨迹和点云
可视化，日常参数输入与执行按钮集中在操作台中。以下前三项是原有只读链路：

1. 读取真实机械臂的 6 轴关节角，在 RViz2 中实时更新官方 FR5 模型。
2. 读取控制器报告的法兰 `XYZ/RPY`，同时发布标准 ROS 位姿、原始毫米/度消息、TF，并在 RViz 中显示坐标文字。
3. 从项目配置读取焊枪 TCP 和两套线激光平面标定，在 RViz 中显示随法兰运动的焊枪、两台 HIK 相机坐标系及其有效激光平面。

节点不会上伺服、切换模式或发送运动指令。

## 一、文件结构

```text
ros2_fr5/src/
├── fairino_description/   # Fairino 官方 FR5 URDF 和 STL meshes
├── fr5_vizum_msgs/        # 原始法兰毫米/度消息
├── fr5_vizum_driver/      # 只读 Fairino SDK 实时状态节点
└── fr5_vizum_bringup/     # robot_state_publisher、标定 TF/几何、RViz 和 launch
```

launch 的默认控制器地址与项目 `config/robot_config.yaml` 当前配置一致：`192.168.1.200`。

## 二、首次构建

环境要求：Ubuntu 22.04、ROS 2 Humble、RViz2，以及本项目已有的 Fairino C++ SDK：

```text
/path/to/vizum-line-scan-gui/SDK/fairino-cpp-sdk-3.9.4
```

构建：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
./build_ros2.sh
```

如果 SDK 放在其他位置，确保头文件和动态库来自同一个 SDK 版本，再这样构建：

```bash
FAIRINO_SDK_DIR=/absolute/path/to/fairino-cpp-sdk ./build_ros2.sh
```

## 三、启动真实 FR5 和 RViz

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
./run_fr5_live_rviz.sh robot_ip:=192.168.1.200
```

启动后，终端每秒输出一次法兰原始坐标，RViz 显示：

- 随真实关节状态运动的 FR5 模型；
- `fairino_flange_reported` 法兰坐标轴；
- `weld_gun_tcp` 焊枪 TCP：橙红色原点、坐标轴和标签；
- `hik_camera_optical_frame`：`scanner_650` HIK 相机 optical 坐标轴及红色半透明线激光平面；
- `hik_450_camera_optical_frame`：`scanner_450` HIK 相机 optical 坐标轴及蓝色半透明线激光平面；
- 法兰旁的实时 `XYZ mm / RPY deg` 文字。

无图形界面时不启动 RViz，仅启动机器人模型与状态发布相关节点：

```bash
./run_fr5_live_rviz.sh use_rviz:=false
```

不使用脚本时，等价命令是：

```bash
source /opt/ros/humble/setup.bash
source /path/to/vizum-line-scan-gui/ros2_fr5/install/setup.bash
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
| `/fairino/calibrated_points` | `visualization_msgs/msg/MarkerArray` | 焊枪 TCP 标签及双 HIK 标定线激光平面 |
| `base_link -> fairino_flange_reported` | TF | 控制器报告的实时法兰坐标系 |
| `fairino_flange_reported -> weld_gun_tcp` | TF Static | `config/tool_config.yaml` 中标定的法兰到焊枪 TCP |
| `fairino_flange_model -> hik_camera_optical_frame` | URDF fixed TF | 650 nm HIK 相机手眼标定 |
| `fairino_flange_model -> hik_450_camera_optical_frame` | URDF fixed TF | 450 nm HIK 相机手眼标定 |

终端查看原始法兰坐标：

```bash
source /opt/ros/humble/setup.bash
source /path/to/vizum-line-scan-gui/ros2_fr5/install/setup.bash
ros2 topic echo /fairino/flange_pose_mm_deg
```

查看标准 ROS 位姿和 TF：

```bash
ros2 topic echo --once /fairino/flange_pose
ros2 run tf2_ros tf2_echo base_link fairino_flange_reported
ros2 run tf2_ros tf2_echo fairino_flange_reported weld_gun_tcp
ros2 run tf2_ros tf2_echo fairino_flange_model hik_camera_optical_frame
ros2 run tf2_ros tf2_echo fairino_flange_model hik_450_camera_optical_frame
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
  publish_calibrated_markers:=true \
  publish_laser_planes:=true
```

- `publish_rate_hz`：关节和法兰发布频率，范围 0.1–200 Hz，默认 20 Hz。
- `log_pose_period_ms`：终端坐标打印周期；设为 `0` 关闭。
- `publish_calibrated_frames`：是否读取焊枪标定配置并发布 TCP 静态 TF。
- `publish_calibrated_markers`：是否发布焊枪 TCP 彩色原点、文字和标定几何；坐标轴由 RViz 的 TF/Axes 显示。
- `publish_laser_planes`：是否在各自 HIK optical frame 下显示线激光平面；它只控制 Marker，不增加 TF。
- `use_rviz`：是否启动 RViz2。

## 六、重要限制与排查

### 标定数据来源

启动时直接读取项目根目录下的焊枪工具配置：

- `/path/to/vizum-line-scan-gui/config/tool_config.yaml`：`T_flange_tcp`，平移单位 mm、固定轴 RPY 单位 deg，当前为 `tool_id: 4`；
- `myline_hik/config/devices/scanner_650/` 与 `scanner_450/`：各自的 `hik_laser_plane.yaml` 和 `hik_intrinsics.yaml`。

两台 HIK 相机的 `T_flange_camera` 已以 fixed joint 写入本地 URDF，来源为同目录下各自的手眼标定。激光平面方程在相机 optical frame 中定义，显示范围严格使用配置的 `camera_z_min_mm=400` 到 `camera_z_max_mm=700`，并用对应相机内参裁到图像水平视场。加载时会检查相机 frame 和内参文件 SHA-256，防止把两套标定混用。

激光平面只确定一个几何平面，不唯一确定平面内 X/Y 轴，因此这里发布半透明 RViz Marker，而不发布容易误解的“激光坐标系 TF”。

`tool_config.yaml` 是项目内保存的工具 4 标定值；控制器当前激活的工具号可能不同。本界面按你的要求显示配置文件中的焊枪 TCP，不把它冒充成控制器活动工具的在线核验值。

如需使用其他标定文件，可显式覆盖：

```bash
./run_fr5_live_rviz.sh \
  tool_config_path:=/absolute/path/to/tool_config.yaml
```

### 同一时间只能有一个 Fairino SDK 连接所有者

不要同时运行以下程序：

- 本 ROS 2 `fairino_state_publisher`；
- VizumScanGUI 中已连接机械臂的 `FairinoRobotClient`；
- Fairino 官方 `ros2_cmd_server`；
- 其他直接调用 `FRRobot::RPC()` 的程序。

第二个连接可能出现 `FRRobot::RPC failed, err=-2`。需要 GUI 和 RViz 同时运行时，应让一个进程独占机器人连接，再通过 ROS topic 共享状态。

### RViz 模型末端与报告法兰

Fairino 官方 `fairino5_v6.urdf` 只定义到 `wrist3_link`，没有独立的法兰 link。本工程用实机扫描记录中的关节角和控制器法兰位姿交叉验证后，在本地 URDF 中增加了 `wrist3_link -> fairino_flange_model` 的 `Z +100 mm` 固定变换，并把 `scanner_650` 与 `scanner_450` 的 optical frame 按各自 `T_flange_camera` 挂在该模型法兰下。

`fairino_flange_model` 是由 URDF 关节正运动学产生的模型 frame；`fairino_flange_reported` 仍是控制器直接报告的实时 frame。两者故意使用不同名称，用于对齐校验并避免 TF 子坐标系重复发布。如果另一个节点要从 `fairino_flange_reported` 发布同名的两个相机 frame，必须改用不同的 child frame 名称。

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

标定显示节点不访问 Fairino SDK。它从 YAML 加载 `T_flange_tcp`，发布 `fairino_flange_reported -> weld_gun_tcp` 静态 TF；同时加载双 HIK 激光平面和内参并发布 RViz Marker。两台 HIK 相机 TF 由 URDF 的 fixed joint 发布。

该 SDK 还使用无法 `join` 的分离后台线程。退出时节点会先调用 `CloseRPC()`，再让单个 `FRRobot` 对象保留到进程结束，由操作系统回收，避免关闭阶段的 SDK 线程访问已析构对象。

## 八、scanner_650：用相机/激光 TCP 做 MoveIt 线扫

### 设计结果

MoveIt 规划组现在是 `base_link -> scanner_650_scan_tcp`，不再以法兰为末端。
`scanner_650_scan_tcp` 位于相机 optical frame 中的标定激光平面上，取正式有效
深度中点 `z=550 mm`：

```text
fairino_flange_model
├── scanner_650_body                 # 保守碰撞包络
└── hik_camera_optical_frame         # T_flange_camera 手眼标定
    └── scanner_650_scan_tcp         # 激光平面上的虚拟工艺 TCP
```

标定数据直接使用：

```text
/path/to/vizum-line-scan-gui/myline_hik/config/devices/scanner_650
```

相机身份固定为 `MV-CS016-10GM / DA8784601 / 192.168.7.45`，图像为
`1440×1080 Mono8`；线激光为 650 nm。重建节点复用了 `myline_hik` 的
标定 YAML 读取、畸变校正和射线/激光平面求交实现。正式点云默认使用与 scanner_650
连续建图实测配置一致的 `legacy` 中心线；`shadow`/`quality` 仍可通过
`centerline_mode` 显式启用，但不能在当前工件验证前替代正式几何。

### 构建和假硬件验证

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
./build_ros2.sh
./run_scanner_650_moveit_mock.sh
```

启动后会同时打开两个窗口：

- **FR5 线激光扫描操作台**：输入扫描角度、行程、速度和返程参数，按顺序完成规划、
  人工确认、批准与执行；也可执行相机坐标系定高 Pilz LIN 扫描、切换 RViz 定位/扫描模式、
  清空或保存点云，以及请求停止运动并关激光。右侧“设备连接”区可单独连接/断开 HIK
  相机和线激光控制器。launch 默认启动两个连接服务但不自动连接硬件，因此按钮可用且
  设备保持断开；相机连接成功会自动开始连续取流，激光控制器断开前必须先取得
  两路 TTL 均为低电平的关光确认。
- **RViz / MoveIt**：只用于检查机械臂实时姿态、碰撞场景、绿色/拒绝轨迹和累积点云。

操作台的工作区粗扫流程固定为：

1. 输入参数并点击“生成轨迹并在 RViz 预览”；
2. 在 RViz 和现场检查轨迹、碰撞体、扫描头线缆、净空及急停；
3. 勾选安全确认并点击“批准当前轨迹”；
4. 点击“执行扫描”，在最终确认框中确认后才发送执行请求。

参数修改会立即使旧规划和旧批准失效，必须重新规划。界面的“停止运动并关激光”是
ROS 2 软件停止，会同时请求停止两类运动、关闭点云累积和关闭激光；真实紧急情况仍必须
使用机械臂硬件急停。

完整 launch 默认启动操作台。只需要 ROS 后端和 RViz 时可关闭：

```bash
./run_scanner_650_moveit_mock.sh use_control_gui:=false
```

如果扫描后端已经在运行，也可以单独启动或重新打开操作台；它只使用 ROS topic、service
和参数接口，不会建立第二个 Fairino SDK 连接：

```bash
source /opt/ros/humble/setup.bash
source /path/to/vizum-line-scan-gui/ros2_fr5/install/setup.bash
ros2 run fr5_scanner_650 scanner_control_gui
```

RViz 的 MotionPlanning 面板中选择 `fairino5_v6_group`。交互标记控制的是
`scanner_650_scan_tcp`；同时可看到相机坐标轴、扫描 TCP、红色标定激光平面、
当前轮廓和 `base_link` 累积点云。

MoveGroup 的直接轨迹执行默认关闭。显式传入 `allow_rviz_execution:=true` 后，
RViz Execute 会经过扫描联锁代理再到真实关节控制器。默认
`rviz_execution_scans:=false`，该通道只用于激光关闭状态下的定位运动；设置为
`true` 时，RViz Execute 会对拖动后 Plan 得到的完整轨迹开激光并累计点云。
相机坐标系定高直线扫描仍可通过 `/scanner_650/execute_last_plan` 使用；`allow_execution`
与 `allow_rviz_execution` 是两个独立开关。

定高线扫默认取 `hik_camera_optical_frame` 的 `+Y/-Y`。这个方向垂直于标定线激光条纹，
规划时先变换到 `base_link`，再投影到 `base_link` 的 XY 平面并归一化。因此扫描 TCP、
相机和激光器在整条 Pilz LIN 路径中保持相同的 `base_link` Z 高度；扫描头姿态不变。
相机 optical `+Z` 是观测深度/朝向工件的方向，不是线扫运动方向。

真实硬件完整流程（拖动定位后再扫描）：

```bash
CONFIRM_FR5_HARDWARE=YES ./run_scanner_650_moveit_hardware.sh \
  allow_rviz_execution:=true \
  rviz_execution_scans:=false \
  allow_execution:=false
```

在 RViz MotionPlanning 中选择 `fairino5_v6_group`，拖动
`scanner_650_scan_tcp` 的交互标记到扫描起点，使用默认 `OMPL` 先 Plan、确认碰撞与
轨迹后 Execute；此时是关光定位。然后在另一个终端切换扫描模式并清空点云：

```bash
ros2 service call /scanner_650/set_rviz_scan_mode std_srvs/srv/SetBool '{data: true}'
ros2 service call /scanner_650/clear_cloud std_srvs/srv/Trigger '{}'
```

再次拖动扫描 TCP 到终点，Plan、确认后 Execute。代理会原样转发这条规划轨迹，并
执行“开650 nm激光、开始累计、运动、停止累计、确认关光”。扫描后需要继续关光
定位时，可运行：

```bash
ros2 service call /scanner_650/set_rviz_scan_mode std_srvs/srv/SetBool '{data: false}'
```

因此这种模式不再需要调用 `plan_linear_scan` 或 `execute_last_plan`；扫描完成后调用
`save_cloud`。
这里的“拖动”用于指定终点，实际扫描的是 MoveIt 的 Plan 结果，不是鼠标移动的
手势轨迹。`scan_distance_m`、`scan_direction_frame`、`scan_direction_axis` 只影响
定高直线扫描服务，不限制 RViz 规划轨迹；RViz 轨迹速度请在 MotionPlanning 面板中
设置，FR5 配置的默认速度和加速度比例均为 0.05。

验证预设的 50 mm 相机 `+Y` 方向定高直线扫描：

```bash
ros2 service call /scanner_650/plan_linear_scan std_srvs/srv/Trigger '{}'
```

默认只规划、不执行。需要在假硬件中连执行也一起验证时：

```bash
./run_scanner_650_moveit_mock.sh allow_execution:=true
ros2 service call /scanner_650/plan_linear_scan std_srvs/srv/Trigger '{}'
ros2 service call /scanner_650/execute_last_plan std_srvs/srv/Trigger '{}'
```

可在启动时改变扫描定义，例如沿相机 `-Y` 反向扫描 80 mm：

```bash
./run_scanner_650_moveit_mock.sh \
  scan_direction_frame:=camera scan_direction_axis:=-y scan_distance_m:=0.08
```

### 真实机械臂

先退出 Qt 程序、旧只读 Fairino 节点和官方 `ros2_cmd_server`，保证只有
`ros2_control` 硬件插件持有 SDK 连接。确认急停、控制器模式、机器人使能、
负载/TCP、工件碰撞体和现场净空后启动：

```bash
CONFIRM_FR5_HARDWARE=YES ./run_scanner_650_moveit_hardware.sh
```

这个命令会连接 FR5，并启动 HIK 相机与鲁班猫安全 TTL daemon 的连接服务。相机和激光
控制器默认保持断开，需在操作台“设备连接”区分别点击连接；如需恢复启动时自动连接，传入
`camera_auto_connect:=true laser_auto_connect:=true`。机械臂的
`execute_last_plan` 仍然锁住。先在 RViz 验证当前姿态和规划结果；需要真正执行时
重新启动并显式解锁：

```bash
CONFIRM_FR5_HARDWARE=YES ./run_scanner_650_moveit_hardware.sh \
  allow_execution:=true scan_distance_m:=0.05
```

执行服务的顺序是：确认 650 nm TTL 高且 450 nm 低 → 确认 HIK 连续取流与设备时钟映射
稳定 → 创建独立扫描 session 并开始点云累积 → 执行 Pilz LIN → 封口并等待重建队列排空
→ 把完整体素表移交后台归档 → 确认两路 TTL 均低。新 session 不复用上一轮的内存点云；
保存失败的数据仍保留在内存并阻止开始下一轮，避免覆盖或静默混合。

### 点云与服务

| 接口 | 说明 |
| --- | --- |
| `/scanner_650/image_raw` | 标定分辨率的 Mono8 图像 |
| `/scanner_650/set_camera_connected` | `SetBool`：连接并启动 HIK 连续取流，或停止取流后断开相机 |
| `/scanner_650/get_camera_statistics` | `Trigger`：读取相机连接、时钟映射及全部累计丢帧计数 |
| `/scanner_650/set_laser_connected` | `SetBool`：连接激光控制器，或确认两路 TTL 关闭后断开并释放租约 |
| `/scanner_650/set_laser` | `SetBool`：在控制通道就绪时确认打开 650 nm，或确认关闭两路激光 |
| `/scanner_650/profile_cloud` | 每帧轮廓，相机 optical frame，单位 m |
| `/scanner_650/scan_cloud_preview` | 2 mm 粗体素实时预览，best-effort；拥塞时允许跳过预览更新，不影响归档数据 |
| `/scanner_650/scan_cloud` | session 成功写盘后才发布的 0.5 mm 完整点云，reliable + transient-local |
| `/scanner_650/plan_linear_scan` | 只规划一次扫描 TCP 的 Pilz LIN |
| `/scanner_650/execute_last_plan` | 显式执行最后一次成功规划 |
| `/scanner_650/stop_motion` | 请求 MoveIt 停止、停止累积并关光 |
| `/scanner_650/set_rviz_scan_mode` | 切换 RViz Execute 的关光定位/整轨迹扫描模式 |
| `/scanner_650/clear_cloud` | 清空累积点云 |
| `/scanner_650/save_cloud` | 后台保存/重试当前 session；自动保存已排队时返回其状态 |

手动保存：

```bash
ros2 service call /scanner_650/save_cloud std_srvs/srv/Trigger '{}'
```

默认在停止累积后自动保存，无需再手动调用；上述服务主要用于关闭自动保存后的手动归档，
或写盘失败后的重试。保存前点已经按逐帧 TF 累积到配置的 `output_frame`，当前配置为
`base_link`。实时
`PointCloud2.rgb` 与 PLY 的 `red/green/blue` 共用同一组 Turbo 颜色：较低的世界 Z 为
紫/蓝色，较高的世界 Z 为橙/红色；RViz 直接读取 RGB，不再另外使用 `AxisColor` 重算。
默认用 Z 的 `1%..99%` 分位数作为色阶，避免少量离群点压缩主体颜色。服务响应会同时返回
入队帧、成功重建帧、轮廓拒绝、队列丢帧、队列高水位、TF 接受/拒绝和体素数；这些统计、
扫描有效性、失效原因及首尾相机状态同时写入 PLY 头和 `manifest.yaml`。累计尚未停止或
队列尚未排空时拒绝保存，避免生成不完整文件。PLY 与 `myline_hik` 连续扫描器一样使用
`binary_little_endian`，XYZ 按毫米
写入，并将 RGB 紧跟在 XYZ 后面，避免 CloudCompare 等软件优先显示 intensity/confidence
标量场而看起来像“高度颜色错误”。需要调整时可设置 `height_color_lower_percentile` 和
`height_color_upper_percentile`，但颜色只影响显示，不会改变 XYZ、强度或置信度。

图像订阅回调只负责把帧放入 64 深度的有界队列，默认由两个工作线程重建。每个工作线程
先在私有连续缓冲中完成坐标变换，再批量合并到 session 的 0.5 mm 完整体素表，避免重复的
逐帧哈希表分配。
RViz 每 0.5 秒只接收独立的 2 mm 粗预览；预览快照占锁时允许跳过该帧的预览合并，但完整
体素仍全部合并。停止时先禁止新帧进入当前 epoch，再等待 `pending=0`，以 O(1) move 将体素
表交给专用保存线程。点云转换、着色和 4 MiB 分块写盘均不占用 ROS 回调或重建线程。

每次结果写入：

```text
${VIZUM_DATA_DIR}/scans/scanner_650/
├── latest_session.txt
└── scan_YYYYMMDD_HHMMSS_mmm/
    ├── scan_voxel.ply
    └── manifest.yaml
```

写入期间目录名带 `.partial`，PLY 和 manifest 都先写临时文件；全部完成后才将目录重命名为
最终 session，旧扫描不会被覆盖。`manifest.yaml` 的 `valid: false` 表示检测到相机帧号跳变、
SDK/图像池/DDS 前置队列丢帧、相邻图像时间戳超过 30 ms（60 FPS 端到端缺帧检查）、
重建队列溢出、轮廓拒绝、TF 拒绝、计数不守恒或体素上限。
系统保证这些情况不会被静默当作完整扫描；硬件、操作系统或进程崩溃仍无法由软件承诺零丢失。

相机侧也沿用 legacy 连续扫描器的非阻塞回调结构：MVS SDK 回调只把已复制的共享图像
缓冲放进 16 深度的有界 ROS 发布队列，ROS 消息复制和 DDS 发布由独立线程完成，不再在
SDK 回调里同步发布 1.55 MB 图像。`/scanner_650/camera_status` 每 120 个发布帧报告一次
设备实测 FPS、接收/发布帧数、相机帧号缺口、SDK 拒绝、图像池耗尽、发布队列高水位和
发布队列丢帧数；任一数据损失还会即时发布 `event=data_loss`，使当前 session 标为无效。
重建节点还会在 session 开始和封口时同步读取一次这些计数并检查增量，因此最后一帧丢失也
不依赖异步状态消息恰好及时到达。
相机到重建节点的图像链路使用有界
reliable QoS，避免原来深度 5 best-effort 在 60 FPS 大图像流下静默漏帧；RViz 仍可用
best-effort 订阅读取 reliable 发布端。

相机节点现在复用 Qt 连续扫描器的稳健时钟拟合：把 GigE 设备 tick 映射到主机
`CLOCK_MONOTONIC_RAW`，再把曝光中点写入图像 header。默认
`require_device_timestamp_mapping=true`，启动后的约前 30 帧用于拟合而不发布，映射不稳定、
时间倒退或设备 tick 无效的帧也不会进入点云。重建节点按该曝光中点查询精确时刻 TF；
`robot_state_publisher` 在 125 Hz 实际关节反馈对应的前后变换之间插值，查不到时间包络时
拒绝该帧，不会退回“最新姿态”。状态话题会出现
`DEVICE_TIMESTAMP_MAPPING` 和
`motion_compensation=DEVICE_TIMESTAMP+TF2_JOINT_INTERPOLATION`。这消除了主机图像回调抖动
直接造成的姿态错配；正式精度仍应使用标准平板做静止/运动对照验证。

## 九、相机向下的桌面单平面径向蛇形扫描

`workspace_coarse_scan_planner` 默认使用 `scan_surface_mode=tabletop_radial_fan`。操作者先
示教一个相机朝下的初始姿态，再调用规划服务。节点把位于标定激光平面上的
`scanner_650_scan_tcp` 当作桌面测量点，因此当桌面低于 `base_link` 时，它的 Z 为负数
是正常现象。机械臂只在这个初始 TCP 高度形成的水平面内运动，不执行低、中、高层
转场。

桌面扇区和姿态采用以下定义：

- TCP `+Z` 是相机光轴，必须朝下，基座 Z 分量必须 `<= -0.80`；
- TCP `+X` 是名义线扫/前方轴，用它的水平投影定义扇区中心 `0°`；
- TCP `+Y` 继续沿标定激光线；
- 每个径向姿态都由完整初始姿态绕 `base_link` Z 轴同步旋转，因此相机向下的俯仰和
  滚转保持不变；
- `fan_origin_at_initial_tcp=true` 时，扇形顶点就是调用规划服务时的 TCP 位置，
  `radial_min_m=0` 表示从该位置立即开始测量，不再先关光走到基座半径 `0.45 m`；
- 默认相对角度 `-60°..+60°`，`radial_max_m=0.72 m` 是从初始 TCP 出发的最大探测
  行程上限，不是相对 `base_link` 的半径；
- 每个角度独立向外执行连续 IK、碰撞、关节余量、肘部构型和奇异性探测，实际外径
  可能小于 `radial_max_m`，不能把配置上限理解成保证可达；
- 径向扇形固定最多 3 根线：最小角、TCP `+X` 对应的相对 `0°` 中心线、最大角；
  扇区必须包含 `0°`，否则规划直接拒绝；
- `-90°..+90°`、`-60°..+60°` 和 `-45°..+45°` 分别生成
  `[-90,0,90]`、`[-60,0,60]` 和 `[-45,0,45]`，线数不随最大径向行程变化；
- 扫描平面 Z 取规划时初始 TCP 高度并按 `1 mm` 量化；
- `40 mm` lead-in/lead-out、`20 mm` 连续检查采样；
- 初验扫描和转场速度均为 `10 mm/s`，Pilz 加速度比例为 `0.30`。

初始 TCP 顶点是操作者已经示教并实际到达的机器人姿态，因此顶点侧不再增加向内的
lead-in；只在外边界保留 40 mm lead。保守粗筛不允许扫描头比扇形所需的顶点姿态继续
靠近基座，并按相机头真实高度和 FR5 名义臂长 85% 外边界截短每条线。最终结果仍以
MoveIt 连续 IK、Planning Scene 碰撞、10° 关节余量、J1 ±85°、同一肘部构型和
归一化奇异值 0.05 为准。

所有扫描线的内端点都是规划时初始 TCP 位置。路径顺序是
`最小角顶点→外、下一角外→顶点、再下一角顶点→外`，形成径向蛇形。每条扫描线
使用 `Pilz LIN lead-in → LIN measurement → LIN lead-out`；两条线之间以关光的 Pilz
LIN 弦线转场。TCP 位置连续且不会瞬移，但每条线和转场是独立轨迹，端点会按控制流程
停启，不是单条不停顿的样条曲线。某个角度不可达只拒绝该条线，并允许检查点按线续扫。
路径每次规划都会在
RViz 的 `/scanner_650/workspace_coarse_scan_markers` 中显示：绿色为可执行候选，
红色为安全包络拒绝，橙色为 IK/碰撞/奇异性拒绝，紫色为 Pilz 规划失败，青色为
检查点记录的已完成带。

### 只规划和预览

假硬件或真实硬件都可以在执行锁关闭时规划：

```bash
# 先示教安全初始姿态：TCP +Z 朝下，TCP +X 指向扇区中心
ros2 service call /scanner_650/plan_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
```

规划开始后，初始 TCP +X 中心角和完整初始姿态会固化进 `plan_id`。若光轴没有充分
朝下，或 TCP +X 无法投影到桌面，规划会直接拒绝。中断续扫前应回到同一初始姿态再
重新规划；中心角按 `0.1°` 量化。

返回的 `path_coverage` 是“通过全部检查的候选扫描带长度 / 请求扫描带长度”，不是
表面实际覆盖率，更不是空体积已扫描证明。线激光只能提供看见的表面观测；没有
目标 CAD/mask 和遮挡模型时，完全没看见的区域不能由点云反证为已覆盖。

几何和低速验证参数可以在下一次规划前修改，例如：

```bash
ros2 param set /workspace_coarse_scan_planner scan_surface_mode tabletop_radial_fan
ros2 param set /workspace_coarse_scan_planner use_initial_tcp_height true
ros2 param set /workspace_coarse_scan_planner fan_origin_at_initial_tcp true
ros2 param set /workspace_coarse_scan_planner use_current_tcp_radius_as_inner false
ros2 param set /workspace_coarse_scan_planner azimuth_min_deg -30.0
ros2 param set /workspace_coarse_scan_planner azimuth_max_deg 30.0
ros2 param set /workspace_coarse_scan_planner radial_min_m 0.0
ros2 param set /workspace_coarse_scan_planner radial_max_m 0.72
ros2 param set /workspace_coarse_scan_planner minimum_radial_scan_length_m 0.10
ros2 param set /workspace_coarse_scan_planner scan_speed_m_s 0.01
ros2 param set /workspace_coarse_scan_planner transition_speed_m_s 0.01
ros2 param set /workspace_coarse_scan_planner return_to_start_after_scan true
ros2 param set /workspace_coarse_scan_planner return_velocity_scaling 0.10
ros2 param set /workspace_coarse_scan_planner return_acceleration_scaling 0.10
ros2 service call /scanner_650/plan_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
```

这里的最多 3 根是粗扫线数量上限规则，不等同于毫米级无缝覆盖保证。径向越远，
相邻射线的横向间隔越大；应通过实际点云检查遮挡和漏扫区域，再执行局部补扫。

真实执行的每个轨迹边界都会等待关节状态连续稳定，再用最新关节状态检查下一段
起点。默认起点误差不超过 `0.01 rad` 时直接执行；误差在 `0.01..0.02 rad` 时保持
激光关闭，规划并安全检查一条 Pilz PTP 小幅校正轨迹，校正后重新等待和校验；超过
`0.02 rad`、校正规划失败或校正后仍超过 `0.01 rad` 都会停止扫描。相关参数为
`joint_settle_*`、`trajectory_start_tolerance_rad` 和
`trajectory_start_correction_limit_rad`。

速度只允许 `0..50 mm/s`，加速度比例只允许 `0..0.5`。首次真实运动保持默认
`10 mm/s`，分级验收后再提高到 `30..50 mm/s`。

### 正负 45° 实扫的 rosbag2 实验记录

独立的逐终端完整操作、录后掉帧检查、离线回看和重建流程见
[`docs/scanner_650_rosbag_full_workflow.md`](docs/scanner_650_rosbag_full_workflow.md)。

`record_scanner_650_scan.sh` 用于记录可离线复现的扫描实验。除了 rosbag，它还会保存
扫描开始/结束时的节点参数、话题/服务列表、标定与 URDF 快照、SHA-256、Git 状态和
磁盘信息。默认记录原始 `1440×1080 Mono8` 图像、每帧激光轮廓、相机信息、TF、关节与
控制器状态、规划场景、扫描/激光/重建状态、参数事件和 ROS 日志。

终端 A 启动硬件系统。曝光只能在相机启动时应用；不同曝光实验必须分别重启并录制，
不能用同一袋原始像素离线模拟另一种曝光：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
CONFIRM_FR5_HARDWARE=YES ./run_scanner_650_moveit_hardware.sh \
  allow_execution:=true \
  collision_scene_validated:=true \
  require_environment_collision_objects:=true \
  camera_exposure_us:=1825.0 \
  camera_gain_db:=0.0
```

终端 B 设置正负 45°和首次低速参数，然后开始录制。脚本启动后保持该终端不动：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 param set /workspace_coarse_scan_planner azimuth_min_deg -45.0
ros2 param set /workspace_coarse_scan_planner azimuth_max_deg 45.0
ros2 param set /workspace_coarse_scan_planner radial_min_m 0.0
ros2 param set /workspace_coarse_scan_planner radial_max_m 0.72
ros2 param set /workspace_coarse_scan_planner scan_speed_m_s 0.01
ros2 param set /workspace_coarse_scan_planner transition_speed_m_s 0.01

./record_scanner_650_scan.sh pm45_exp1825
```

终端 C 在录制已经显示 `Recording...` 后规划、人工检查、批准并执行：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash

# Humble rosbag2 不记录 service request/response；用终端日志补齐这部分审计记录。
SESSION_DIR="$(<../myline_hik/data/rosbags/scanner_650/latest_session.txt)"
script -a "${SESSION_DIR}/metadata/operator_terminal.log"

ros2 service call /scanner_650/plan_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
# 在 RViz 和现场确认候选轨迹、扫描头/线缆/工件/安全区及急停后：
ros2 service call /scanner_650/approve_workspace_coarse_scan \
  std_srvs/srv/SetBool '{data: true}'
ros2 service call /scanner_650/execute_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
ros2 service call /scanner_650/save_cloud \
  std_srvs/srv/Trigger '{}'

# 退出 script 子 shell，完成 operator_terminal.log。
exit
```

机械臂已经停稳、累积结束且激光确认关闭后，在终端 B 按 `Ctrl-C`。会话保存在：

```text
myline_hik/data/rosbags/scanner_650/<时间>_pm45_exp1825/
├── bag/
├── artifacts/scan_YYYYMMDD_HHMMSS_mmm/
│   ├── scan_voxel.ply                  # 本次录制期间完成的扫描 session
│   └── manifest.yaml
└── metadata/
    ├── rosbag_info.txt
    ├── *_params_start.yaml
    ├── *_params_end.yaml
    ├── config_snapshot/
    └── config_sha256.txt
```

`/scanner_650/scan_cloud_preview` 是反复刷新的实时粗预览，默认不写入 rosbag；
`/scanner_650/scan_cloud` 每个 session 只在归档成功后发布一次。默认记录可重新提线的原图、
逐帧 `profile_cloud` 并复制最终 session 目录。确实需要同时记录归档完整点云话题时使用：

```bash
SCANNER_650_RECORD_SCAN_CLOUD=1 ./record_scanner_650_scan.sh pm45_full
```

Mono8 原图的数据率约 `93 MB/s`，即未压缩约 `5.6 GB/min`。默认不在线压缩以降低丢帧
风险；磁盘不足且 CPU 已经现场验证有余量时可使用文件级 zstd：

```bash
SCANNER_650_ROSBAG_COMPRESSION=zstd ./record_scanner_650_scan.sh pm45_exp1825
```

离线查看前先完全退出真实机械臂硬件栈，避免回放的 `/joint_states`、TF 和控制器状态与
实机话题冲突。查看袋内容和回放原图可用：

```bash
ros2 bag info <会话目录>/bag
ros2 bag play <会话目录>/bag --topics \
  /scanner_650/image_raw \
  /scanner_650/camera_info \
  /scanner_650/profile_cloud \
  /joint_states /tf /tf_static
```

同一 bag 可以离线调整 ROI、亮度/饱和门槛、条纹中心算法和置信度阈值；实际曝光、增益、
焦距和光圈改变后必须重新采集。建议依次录制 `exp1200`、`exp1825`、`exp2500` 等独立
会话，再以有效条纹点数、饱和率、中心抖动和最终平面误差比较。

### 碰撞场景、批准与执行

默认 `collision_scene_validated:=false`、`allow_execution:=false`，所以节点只能预览。
真实自动执行前必须先加载并现场核验机器人、扫描头、线缆、夹具、工件和安全区的
碰撞几何。完成该独立验收后重新启动：

```bash
CONFIRM_FR5_HARDWARE=YES ./run_scanner_650_moveit_hardware.sh \
  allow_execution:=true \
  collision_scene_validated:=true \
  require_environment_collision_objects:=true \
  start_table_collision_scene:=true \
  table_surface_z_m:=-0.100 \
  table_center_x_m:=0.200 \
  table_center_y_m:=0.200 \
  table_size_x_m:=0.300 \
  table_size_y_m:=0.200 \
  table_thickness_m:=0.010
```

节点还会检查 Planning Scene 中确实存在环境碰撞对象或 Octomap。未知空间不能靠
点击批准绕过。上例会把实测桌面加载为 `base_link` 中的 Box：中心
`(0.200, 0.200, -0.105) m`，尺寸 `(0.300, 0.200, 0.010) m`，因此上表面为
`Z=-0.100 m`。`start_table_collision_scene` 默认关闭，避免在另一套现场误用该尺寸。
启动后必须先在 RViz Scene Objects 中确认桌面方向、位置与真实现场一致；桌面之外的工件、
夹具和固定障碍物仍需另行建模。完整操作顺序是：

```bash
ros2 service call /scanner_650/plan_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
# 在 RViz 检查所有绿色、不可达和彩色拒绝带及现场净空
ros2 service call /scanner_650/approve_workspace_coarse_scan \
  std_srvs/srv/SetBool '{data: true}'
ros2 service call /scanner_650/execute_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
```

默认启用 `return_to_start_after_scan=true`。规划时节点保存真实起始关节状态和 TCP，并从
预计扫描终点生成一条直接回到起始关节姿态的 Pilz PTP 返程，不再倒序重放三条扫描线和
转场轨迹。直接返程仍使用相同的连续关节余量、J1、肘部构型、奇异性和碰撞条件验证；
验证不通过时整个规划请求失败，不能批准执行。ETA 已包含这条直接返程。

直接返程 PTP 使用独立的 `return_velocity_scaling` 和
`return_acceleration_scaling`，默认均为 `0.10`，其含义是机械臂关节速度/加速度上限的
10%。`transition_speed_m_s` 只表示 LIN 转场的笛卡尔速度（m/s），不再被换算成返程 PTP
的关节速度比例。两个返程缩放参数允许范围为 `(0, 0.50]`，修改后必须重新规划并批准。

最后一条扫描线完成后，节点先确认激光和点云累积关闭并等待真实关节状态稳定，然后比较
当前位置与规划起点：默认最大关节偏差不超过 `0.002 rad`、TCP 位置不超过 `1 mm` 且姿态
不超过 `0.2°` 时跳过返程。超过任一容差时，以停稳后的真实关节状态为起点重新生成一条
直接 Pilz PTP 返程，并重新检查当前 Planning Scene 和整条生成轨迹；不会先校正到预计扫描
终点，也不会沿扫描线倒着走。返程失败不会重新打开激光，也不会抹掉已完成扫描带的检查点；
服务响应会明确区分“扫描失败”和“扫描完成但返程失败”。
进入返程前，节点会先原子写入 `return_status: "in_progress"`；返程结束后再更新为
`completed` 或 `failed`。禁用返程或扫描阶段失败时则记录 `disabled` 或对应的未执行原因。

RViz 中细蓝线为规划时验证的直接 PTP 激光关闭返程，粗绿线仍是开激光的测量路径。蓝线
只在规划时由已验证轨迹生成并发布一次；执行扫描和返程时不会重新做 FK、重建或发布完整
返程 Marker。最后一条扫描完成后会优先执行返程，返程尝试结束后才刷新一次扫描状态
Marker。Marker 和状态话题发布均有异常隔离，非关键 RViz 发布错误只写日志和服务警告，
不会中断已经开始的机械臂运动。

硬件与 mock 启动脚本会尝试启用 core dump，并给节点设置原生崩溃回溯文件。若节点再次因
`SIGABRT`、`SIGSEGV` 等信号退出，可先检查：

```bash
ls -lt /path/to/vizum-line-scan-gui/ros2_fr5/log/crash/
sed -n '1,160p' \
  /path/to/vizum-line-scan-gui/ros2_fr5/log/crash/workspace_coarse_scan_planner_*.trace
```

可执行文件保留调试符号和 frame pointer，并导出符号，便于用 trace 或 core dump 定位具体
调用栈。若系统把 core 交给 apport，trace 文件仍会直接保存在上述目录。

急停之外的软件停止入口为：

```bash
ros2 service call /scanner_650/stop_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
```

节点在每条带成功完成且确认关光后原子更新检查点。中断后重新调用完整规划服务，
相同几何参数会恢复已完成带；再次检查、批准和执行即可续扫。清空进度需要显式调用
`/scanner_650/clear_workspace_coarse_checkpoint`。

### 局部补扫

质量分析或操作者可以给出 `base_link` 中的缺失区域 AABB。节点用理论激光半宽做
外扩，只重规划与该区域相交的扫描带：

```bash
ros2 param set /workspace_coarse_scan_planner local_rescan_roi_min \
  "[0.70, -0.20, 0.35]"
ros2 param set /workspace_coarse_scan_planner local_rescan_roi_max \
  "[1.10, 0.20, 0.60]"
ros2 service call /scanner_650/plan_workspace_local_rescan \
  std_srvs/srv/Trigger '{}'
```

局部补扫使用同样的碰撞、批准、执行和逐带检查点门禁。它是基于外部缺失区域的
有限补扫，不会把“无回波”自动解释成空空间。

### 新增接口

| 接口 | 说明 |
| --- | --- |
| `/scanner_650/plan_workspace_coarse_scan` | 生成、完整过滤并显示初始高度单平面扇形线扫 |
| `/scanner_650/plan_workspace_local_rescan` | 只规划与缺失区域 AABB 相交的扫描带 |
| `/scanner_650/approve_workspace_coarse_scan` | 在碰撞场景验证通过后记录人工批准 |
| `/scanner_650/execute_workspace_coarse_scan` | 分带执行，测量开光、转场关光 |
| `/scanner_650/stop_workspace_coarse_scan` | 取消控制器目标、停止累积并关光 |
| `/scanner_650/clear_workspace_coarse_checkpoint` | 显式清除续扫检查点 |
| `/scanner_650/workspace_coarse_scan_markers` | RViz 全部候选带、拒绝带和摘要 |
| `/scanner_650/workspace_coarse_scan_status` | 规划、执行、错误和检查点状态 |



cd /home/zhulong/lh/vizum-line-scan-gui/ros2_fr5

CONFIRM_FR5_HARDWARE=YES \
./run_scanner_650_moveit_hardware.sh allow_execution:=true

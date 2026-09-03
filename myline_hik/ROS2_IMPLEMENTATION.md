# 焊接一体化软件 ROS 2 实现

本目录的 `ros2_ws` 是方案 V2 的第一条可运行纵向闭环。旧 Qt 标定/线扫程序保留作实机
算法与设备基线，新代码通过稳定接口逐步替换人工操作，不直接改坏旧工具。

## 已实现

- 版本化接口：CalibrationPackage、PlanningScene 依赖、Session、审批、任务状态及 Actions；
- 标定包原子激活、7 类文件 SHA-256 复核、运行时篡改失效；
- 激活包中的两份手眼矩阵以米制 TF 发布；夹具 URDF 的 nominal 帧不会冒充实测外参；
- Session 隔离、事件日志、审批记录、结束时全部产物哈希清单；
- 两台海康相机的 ROS 驱动，复用原有有界偏差软件配对内核，严格校验 IP 和序列号；
- 专用双目标定外参加载、分段 SGBM 置信度融合、米制 PointCloud2；
- OctoMap 粗地图；机器人自滤波未验收时只发布 preview，不进入 MoveIt；
- PlanningScene 版本和轨迹依赖锁，标定/地图/URDF/SRDF/工具/规划器任一变化即拒绝旧轨迹；
- 唯一运动命令网关，默认只允许 dry-run；轨迹哈希、人工审批和活动依赖全部通过后才转发；
- 鲁班猫激光 GPIO 回读、租约和本地超时关光；只读机器人反馈桥；
- 分层、可暂停/终止工作流状态机；当前硬件工作流只开放 commissioning dry-run；
- PCD/PLY 直接发布到 RViz，CloudCompare 不再是生产查看必需步骤。

## 构建

当前机器的 Conda Qt、MVS 自带旧 `libusb` 和 ROS 2 库会互相污染。必须使用封装脚本：

```bash
./ros2_ws/build_ros2.sh
```

脚本固定使用系统 Python 和 ROS 2 Humble 的 Fast-CDR，并清理 Conda/MVS 的继承环境。

## 启动基础服务

```bash
./ros2_ws/run_ros2.sh
```

默认 `allow_hardware_motion=false`。不得为了“先跑起来”直接改成 true；实机前必须完成本文末尾
的验收项，并传入真实 URDF/SRDF/工具/规划器摘要。

## 查看已有点云

终端一：

```bash
./ros2_ws/run_ros2.sh ros2 launch welding_bringup visualization.launch.py
```

终端二（现有 PLY 坐标是毫米，因此比例为 0.001）：

```bash
./ros2_ws/run_ros2.sh ros2 launch welding_bringup offline_cloud.launch.py \
  file:=/absolute/path/to/cloud.ply scale_to_meters:=0.001 frame_id:=base_link
```

## 启动双目硬件与建图

### 先验证双目深度（不接入 MoveIt）

关闭 Qt 标定程序和 MVS 后，使用刚批准的 `config/hik_stereo.yaml` 启动 commissioning
预览：

```bash
./run_ros2_stereo_depth.sh
```

启动文件会读取 `config/stereo_depth_tuner.yaml`。当前保存值为左相机 `22000 us`、右相机
`20000 us`、`5 fps` 和 `32 ms` 自由运行配对门槛；处理尺寸、SGBM、左右检查与 CLAHE
参数也来自该文件。深度不使用调优页面最后一次试验的 `700–900 mm` 单段范围，而是使用
同一 YAML 中的三段配置 `300–600 / 600–1200 / 1200–2500 mm`。每段各自计算有效视差
ROI，再按光度与左右一致性置信度逐像素融合，因此远距离段不会被 300 mm 近距段的巨大视差
搜索范围裁成窄条。

节点发布 `/welding_robot/stereo/status`，其中 `band_count=3` 表示三段匹配已经生效；同时发布
`/welding_robot/stereo/depth`（`32FC1`，米）、
`/welding_robot/stereo/depth_preview` 和 `/welding_robot/stereo/points`（米，左相机光学坐标）。
此模式明确设置 `require_active_calibration=false`，只用于静止场景验证，点云不得进入 MoveIt。
自由运行双相机在机械臂运动时不能代替共同硬件触发。

可在每次启动时统一覆盖曝光（单位为微秒）；修改后需要重启节点：

```bash
./run_ros2_stereo_depth.sh exposure_us:=25000.0
```

也可以单独设置左侧 160 万相机和右侧 130 万相机的曝光：

```bash
./run_ros2_stereo_depth.sh left_exposure_us:=22000 right_exposure_us:=30000
```

也可以同时关闭 RViz 或覆盖帧率等现场参数：

```bash
./run_ros2_stereo_depth.sh use_rviz:=false exposure_us:=25000.0 frames_per_second:=5.0
```

查看实时融合统计：

```bash
./ros2_ws/run_ros2.sh ros2 topic echo /welding_robot/stereo/status
```

修改分段时编辑 `config/stereo_depth_tuner.yaml` 的 `multi_band.ranges_mm`，保存后重启 ROS2。
各段必须连续覆盖总范围，不能留空档。

### 标定注册后接入粗地图

先创建并激活一个 `validated` CalibrationPackage，再启动硬件。硬件配置示例位于
`welding_hardware/config/hardware.yaml`，正式使用时应复制到部署目录并保护密钥权限。

```bash
./ros2_ws/run_ros2.sh ros2 launch welding_bringup hardware.launch.py \
  hardware_config:=/absolute/path/to/hardware.yaml

./ros2_ws/run_ros2.sh ros2 launch welding_bringup mapping.launch.py \
  calibration_package_id:=cal_YYYYMMDD \
  stereo_yaml:=/absolute/path/hik_stereo.yaml \
  left_intrinsics_yaml:=/absolute/path/hik_intrinsics_650.yaml \
  left_handeye_yaml:=/absolute/path/hik_handeye_650.yaml \
  right_intrinsics_yaml:=/absolute/path/hik_intrinsics_450.yaml \
  right_handeye_yaml:=/absolute/path/hik_handeye_450.yaml
```

`self_filter_valid` 默认 false。只有完成机器人点云自滤波回放测试后才允许设 true。
MoveIt 的 `PointCloudOctomapUpdater` 参数模板位于
`welding_planning/config/sensors_3d.yaml`；应先检查其
`/welding_robot/map/self_filtered_points` 输出确实去除了 FR5 与夹具，再批准该门禁。

## 仍受门禁的功能

下面不是“已完成但没开按钮”，而是必须继续做实机集成与验收的边界：

- 自动标定 Action 已有状态机和 dry-run，但尚未把 MoveIt 候选姿态、双相机采样和两路激光
  采样全部接成无人值守实机闭环；旧 Qt 算法仍是求解基线；
- 焊缝候选、质量地图、NBS 补扫算法在旧核心中已有部分实现，尚未全部包装成独立 ROS Action；
- FR5 生产驱动、MoveIt 控制器、完整 FR5+夹具 URDF/SRDF 需要与现场控制柜配置联合验收；
- 焊机 READY/GAS/ARC I/O 状态机未接入，因此工作流强制 `stop_before_arc=true`；
- `allow_hardware_motion=true` 只能在 HIL、低速、急停与碰撞场景验收全部通过后启用。

这套门禁是有意设计的：缺少任何关键依赖时程序应拒绝动作，而不是带着默认值运行。

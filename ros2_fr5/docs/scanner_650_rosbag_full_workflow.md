# scanner_650 正负 45° 自动扫描与 rosbag2 完整录制流程

本文用于记录一次从示教初始姿态开始、以 TCP `+X` 为中心线、相对方向
`-45° / 0° / +45°` 的单平面径向扫描。录制结果用于：

- 回看原始 Mono8 图像和激光线；
- 核对关节、TF、规划、激光开关、扫描与返程时序；
- 离线修改提线、ROI、阈值、置信度和体素参数后重新重建；
- 对比不同真实曝光和增益实验。

## 1. 重要限制

1. rosbag 能保存相机已经输出的原始像素，但不能把一组曝光离线变成另一组真实曝光。
   曝光改变会同时改变饱和、散粒噪声、运动模糊和相机响应。因此
   `1200 us / 1825 us / 2500 us` 必须分别重启相机、分别扫描和录包。
2. ROS 2 Humble 的 rosbag2 不记录本项目的 service request/response。必须同时保存操作者
   终端日志，才能知道何时调用了规划、批准、执行、停止和保存点云服务。
3. 默认不录制不断增大的 `/scanner_650/scan_cloud`。它会反复写入整个累计点云，数据量很大。
   原始图像、逐帧 `/scanner_650/profile_cloud` 和最终 PLY 足以满足通常的离线分析。
4. `1440 × 1080 × 60 FPS` Mono8 原图约为 `93 MB/s`，即约 `5.6 GB/min`。必须使用经过
   测试的高速本地磁盘，并在正式扫描前做一次短录制掉帧检查。
5. rosbag 只负责记录，不会提高机械臂安全性。实扫仍须确认碰撞场景、扫描头、线缆、工件、
   安全区和急停；未知空间不能因正在录包而绕过执行门禁。

## 2. 默认会记录什么

推荐使用工作空间中的专用脚本：

```text
/path/to/vizum-line-scan-gui/ros2_fr5/record_scanner_650_scan.sh
```

它记录以下主要数据：

| 类别 | 话题或文件 |
| --- | --- |
| 原始相机 | `/scanner_650/image_raw`、`/scanner_650/camera_info`、`camera_status` |
| 激光与重建 | `laser_status`、`reconstruction_status`、`profile_cloud`、标定 Marker |
| 自动扫描 | `workspace_coarse_scan_status`、`workspace_coarse_scan_markers` |
| 机器人 | `/joint_states`、`/dynamic_joint_states`、控制器状态、`/tf`、`/tf_static` |
| MoveIt | 规划场景、规划显示、碰撞对象、运动计划请求等话题 |
| 诊断 | `/parameter_events`、`/rosout` |
| 会话快照 | 节点参数、标定 YAML、URDF、SHA-256、Git 状态、节点/话题/服务列表 |
| 扫描产物 | 调用 `/scanner_650/save_cloud` 后生成的 `ros2_scan_cloud.ply` |

脚本要求至少有 `20 GiB` 可用空间，默认每 `4 GiB` 分割一个 SQLite3 文件，并使用专门的
相机 QoS 配置。输出根目录默认为：

```text(base) zhulong@zhulong:~/lh/vizum-line-scan-gui/ros2_fr5$ ros2 service call /scanner_650/approve_workspace_coarse_scan \
  std_srvs/srv/SetBool '{data: true}'

ros2 service call /scanner_650/execute_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
waiting for service to become available...
requester: making request: std_srvs.srv.SetBool_Request(data=True)

response:
std_srvs.srv.SetBool_Response(success=False, message='approval refused: validated scene contains no environment collision objects/octomap; unknown space cannot be authorized by a software checkbox')

waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=False, message='no approved plan; plan, inspect RViz, then approve explicitly')

(base) user@host:/path/to/vizum-line-scan-gui/ros2_fr5$

/path/to/vizum-line-scan-gui/myline_hik/data/rosbags/scanner_650/
```

## 3. 实扫前检查

### 3.1 放置机械臂

1. 把机械臂移动到希望作为扇形原点的初始姿态。
2. 相机保持朝下，扫描 TCP 的 `+Z` 朝向桌面。
3. TCP 水平投影的 `+X` 是扫描中心方向，规划将保留中心线。
4. 检查从当前位置向 `-45° / 0° / +45°` 延伸时扫描头和线缆有足够净空。
5. 保持急停可触达，首次仍使用 `0.01 m/s`。

### 3.2 磁盘与系统

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
df -h ../myline_hik/data/rosbags/scanner_650
source /opt/ros/humble/setup.bash
source install/setup.bash
```

建议至少保留 `30 GiB`，长时间录制按 `6 GB/min` 以上估算并留出余量。

## 4. 终端 A：启动真实硬件、相机和激光控制

曝光和增益必须在相机启动时确定。以下示例使用 `1825 us`、`0 dB`：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5

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
  table_thickness_m:=0.010 \
  camera_exposure_us:=1825.0 \
  camera_gain_db:=0.0
```

`collision_scene_validated:=true` 只能在真实碰撞对象已经加载并由操作者检查后使用。如果场景
尚未验证，可以保持默认值做规划和录包预览，但不能批准真实执行。

上述桌面参数表示 X 方向长 `300 mm`、Y 方向宽 `200 mm`、厚 `10 mm`，中心 X/Y 为
`200/200 mm`。节点自动把 Box 中心放在 `Z=-105 mm`，使上表面严格位于 `Z=-100 mm`。
启动后等待并检查：

```bash
ros2 topic echo --once --full-length \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /scanner_650/table_collision_scene_status
```

必须看到 `loaded table collision object`，并在 RViz Scene Objects 中确认桌面位置和方向，
然后才可规划和批准。桌面之外的工件、夹具、人员和固定障碍物不会由该 Box 自动覆盖；
人员必须位于机械臂可达范围之外。

不要在扫描过程中关闭终端 A。启动后检查关键节点和数据：

```bash
ros2 node list | sort
ros2 topic hz --window 120 /scanner_650/image_raw
ros2 topic bw --window 120 /scanner_650/image_raw
ros2 topic echo --once /scanner_650/camera_status
ros2 topic echo --once /scanner_650/laser_status
```

图像接收率应接近配置的 `60 FPS`。`ros2 topic hz` 本身也会消耗资源，确认后用 `Ctrl-C`
停止该检查，不要让它与正式 rosbag 长时间并行。

## 5. 终端 B：设置正负 45°参数并开始录包

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 param set /workspace_coarse_scan_planner scan_surface_mode tabletop_radial_fan
ros2 param set /workspace_coarse_scan_planner use_initial_tcp_height true
ros2 param set /workspace_coarse_scan_planner fan_origin_at_initial_tcp true
ros2 param set /workspace_coarse_scan_planner use_current_tcp_radius_as_inner false

ros2 param set /workspace_coarse_scan_planner azimuth_min_deg -45.0
ros2 param set /workspace_coarse_scan_planner azimuth_max_deg 45.0
ros2 param set /workspace_coarse_scan_planner radial_min_m 0.0
ros2 param set /workspace_coarse_scan_planner radial_max_m 0.72

ros2 param set /workspace_coarse_scan_planner scan_speed_m_s 0.01
ros2 param set /workspace_coarse_scan_planner transition_speed_m_s 0.04
ros2 param set /workspace_coarse_scan_planner acceleration_scaling 0.30
ros2 param set /workspace_coarse_scan_planner return_to_start_after_scan true
ros2 param set /workspace_coarse_scan_planner return_velocity_scaling 0.10
ros2 param set /workspace_coarse_scan_planner return_acceleration_scaling 0.10
```

当前径向规划强制保留 TCP `+X` 中心线，并且所有角度最多三根线，因此正负 45°的候选方向
为 `-45°、0°、+45°`。每条线会独立缩短到连续 IK、关节、奇异性、碰撞和可达边界。

如果这是一次新的完整实验，而不是续扫，显式清除旧检查点：

```bash
ros2 service call /scanner_650/clear_workspace_coarse_checkpoint \
  std_srvs/srv/Trigger '{}'
```

确认关键参数：

```bash
ros2 param get /workspace_coarse_scan_planner azimuth_min_deg
ros2 param get /workspace_coarse_scan_planner azimuth_max_deg
ros2 param get /workspace_coarse_scan_planner radial_max_m
ros2 param get /workspace_coarse_scan_planner scan_speed_m_s
ros2 param get /workspace_coarse_scan_planner return_to_start_after_scan
ros2 param get /workspace_coarse_scan_planner return_velocity_scaling
ros2 param get /workspace_coarse_scan_planner return_acceleration_scaling
```

`transition_speed_m_s` 是 LIN 转场的笛卡尔速度；直接返程 PTP 单独使用后两个无量纲关节
缩放参数，`0.10` 表示关节速度和加速度上限的 10%。

开始录制。标签只能使用字母、数字、点、下划线和连字符：

```bash
./record_scanner_650_scan.sh pm45_exp1825_gain0
```

看到 rosbag 输出 `Recording...` 后再进入下一步。终端 B 保持运行，不要提前按 `Ctrl-C`。

不推荐首次实验在线压缩。只有确认 CPU 有余量且不掉帧后，才使用：

```bash
SCANNER_650_ROSBAG_COMPRESSION=zstd \
  ./record_scanner_650_scan.sh pm45_exp1825_gain0_zstd
```

确实需要累计点云话题时使用以下模式，但 bag 会明显增大：

```bash
SCANNER_650_RECORD_SCAN_CLOUD=1 \
  ./record_scanner_650_scan.sh pm45_exp1825_gain0_fullcloud
```

## 6. 终端 C：保存操作者日志并执行扫描

Humble rosbag2 不保存服务调用，因此先用 `script` 记录整个操作者终端：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash

SESSION_DIR="$(<../myline_hik/data/rosbags/scanner_650/latest_session.txt)"
script -a "${SESSION_DIR}/metadata/operator_terminal.log"
```

现在处于 `script` 启动的子 shell 中。依次执行：

```bash
ros2 service call /scanner_650/plan_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
```

规划后必须同时检查响应、RViz 和现场：

- 响应应为 `success=True`；
- 期望完整三方向时应看到 `planned_lanes=3/3`；
- 蓝色返程路径应已生成；
- 红色/橙色/紫色方向是不可执行方向；
- 现场扫描头、线缆、桌面和安全区必须与碰撞场景一致；
- 若只得到 `2/3`，不要把覆盖率当成三方向完整扫描，应调整初始姿态后重新规划。

人工确认后批准并执行：

```bash
ros2 service call /scanner_650/approve_workspace_coarse_scan \
  std_srvs/srv/SetBool '{data: true}'

ros2 service call /scanner_650/execute_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
```
QCoreApplication
不要在执行服务尚未返回时停止 rosbag。正常流程是三条扫描完成、激光关闭、写入
`return_status: "in_progress"`、返回示教起点，然后写入 `completed`。

执行返回后保存累计点云，并检查检查点：

```bash
ros2 service call /scanner_650/save_cloud \
  std_srvs/srv/Trigger '{}'

cat /path/to/vizum-line-scan-gui/myline_hik/data/scans/scanner_650/workspace_coarse_checkpoint.yaml
ros2 topic echo --once /scanner_650/laser_status
```

确认 `return_status: "completed"` 且激光状态明确为 OFF。随后退出 `script` 子 shell：

```bash
exit
```

## 7. 终端 B：停止录包

只有在以下条件全部满足后才在终端 B 按 `Ctrl-C`：

- execute 服务已经返回；
- 机械臂已经停稳；
- 返程已经完成，或返程失败原因已被完整记录；
- 点云累积已关闭；
- 激光已确认关闭；
- `save_cloud` 已调用。

脚本会在 rosbag 停止后保存结束参数、bag 信息，并把本次新生成的 PLY 复制到会话目录。

## 8. 检查本次录制是否完整

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash

SESSION_DIR="$(<../myline_hik/data/rosbags/scanner_650/latest_session.txt)"
echo "${SESSION_DIR}"
ros2 bag info "${SESSION_DIR}/bag"
du -sh "${SESSION_DIR}"
find "${SESSION_DIR}" -maxdepth 3 -type f | sort
```

重点核对：

1. `/scanner_650/image_raw`、`camera_info`、`profile_cloud`、`joint_states`、`tf` 和
   `tf_static` 的消息数都不是 0。
2. `image_raw` 数量应与 `camera_info` 数量接近。因为二者按同一相机帧发布，建议比例至少
   达到 `95%`；明显更低表示录包磁盘或 CPU 跟不上。
3. `image_raw / bag duration` 应接近 `60 FPS`。录制前后相机持续工作，因此可以直接估算。
4. `metadata/operator_terminal.log` 中包含规划、批准、执行和保存点云的完整响应。
5. `artifacts/ros2_scan_cloud.ply` 存在且大小不是 0。
6. `metadata/*_params_start.yaml` 和 `*_params_end.yaml` 存在。

可选地为 bag 数据库生成校验值：

```bash
sha256sum "${SESSION_DIR}"/bag/*.db3 \
  > "${SESSION_DIR}/metadata/bag_sha256.txt"
```

如果原图掉帧，不能通过插值恢复真实激光条纹。优先使用高速本地 NVMe、关闭在线 zstd、
停止无关图像订阅和屏幕录制；仍无法达到目标时，先降低相机 FPS 再重新采集，并在实验标签
中明确写出 FPS。

## 9. 离线回看原始激光线

回放前必须完全退出真实硬件 launch，避免 bag 中的 `/joint_states`、TF 和控制器状态与实机
话题冲突。不要在连接真实机械臂的 ROS 域中回放机器人话题。

终端 D 启动图像查看器：

```bash
source /opt/ros/humble/setup.bash
rqt_image_view
```

在界面中选择 `/scanner_650/image_raw`。另一个终端慢速回放原图：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash
SESSION_DIR="$(<../myline_hik/data/rosbags/scanner_650/latest_session.txt)"

ros2 bag play "${SESSION_DIR}/bag" \
  --rate 0.25 \
  --topics /scanner_650/image_raw /scanner_650/camera_info
```

可加 `--loop` 循环播放，或加 `--start-paused` 后用空格控制暂停。建议重点观察：

- 激光开启前后的背景亮度；
- 激光中心是否清晰且连续；
- 是否出现大面积 `255` 饱和平台；
- 线宽是否随表面反射发生显著变化；
- 转场和返程阶段激光是否保持关闭；
- 运动方向改变时条纹是否产生明显拖影。

## 10. 在 RViz 回看逐帧轮廓和轨迹

启动 RViz，Fixed Frame 可使用 `base_link`，添加 PointCloud2 并选择
`/scanner_650/profile_cloud`：

```bash
rviz2
```

回放必要话题：

```bash
ros2 bag play "${SESSION_DIR}/bag" --rate 0.25 --topics \
  /scanner_650/profile_cloud \
  /scanner_650/workspace_coarse_scan_markers \
  /scanner_650/workspace_coarse_scan_status \
  /scanner_650/laser_status \
  /joint_states /tf /tf_static
```

`profile_cloud` 位于相机坐标系，包含 `intensity` 和 `confidence` 字段。RViz 可分别按这些字段
着色，检查条纹强度、置信度和轮廓连续性。

## 11. 用同一个 bag 离线重新重建

这一流程只启动重建节点，不启动相机、激光控制、MoveIt 或真实机械臂。终端 E：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash
SESSION_DIR="$(<../myline_hik/data/rosbags/scanner_650/latest_session.txt)"

ros2 run fr5_scanner_650 line_laser_reconstruction_node --ros-args \
  --params-file src/fr5_scanner_650/config/scanner_650.yaml \
  -p use_sim_time:=true \
  -p voxel_size_m:=0.0005 \
  -p output_ply:="${SESSION_DIR}/artifacts/reprocessed_voxel0p5mm.ply" \
  -p profile_topic:=/offline/scanner_650/profile_cloud \
  -p scan_topic:=/offline/scanner_650/scan_cloud \
  -p marker_topic:=/offline/scanner_650/markers
```

另一个终端清空并开启离线累积：

```bash
source /opt/ros/humble/setup.bash
source /path/to/vizum-line-scan-gui/ros2_fr5/install/setup.bash

ros2 service call /scanner_650/clear_cloud std_srvs/srv/Trigger '{}'
ros2 service call /scanner_650/set_accumulation std_srvs/srv/SetBool '{data: true}'
```

然后慢速播放原图和 TF。不要同时播放袋内原有的 `profile_cloud`，否则会与离线输出混淆：

```bash
ros2 bag play "${SESSION_DIR}/bag" \
  --clock 60 \
  --rate 0.25 \
  --topics /scanner_650/image_raw /scanner_650/camera_info /tf /tf_static
```

播放结束后关闭累积并保存：

```bash
ros2 service call /scanner_650/set_accumulation \
  std_srvs/srv/SetBool '{data: false}'
ros2 service call /scanner_650/save_cloud \
  std_srvs/srv/Trigger '{}'
```

当前节点已暴露 `voxel_size_m`、`maximum_voxels`、`publish_every_n_profiles` 等 ROS 参数。
提线的 `minimumDifference=10`、`thresholdStddevScale=2.0`、ROI 和部分质量门槛目前在
`line_laser_reconstruction_node.cpp` 的 `configure_reconstruction()` 中配置，不是可动态设置的
ROS 参数。调整这些算法项时需要修改/参数化代码、重新编译，再使用同一 bag 回放对比。

## 12. 曝光和算法参数调优建议

### 12.1 真实曝光实验

推荐至少录制以下独立会话：

```text
pm45_exp1200_gain0
pm45_exp1825_gain0
pm45_exp2500_gain0
```

每组实验必须：

1. 完全停止硬件 launch；
2. 使用新的 `camera_exposure_us` 重启；
3. 把机械臂恢复到相同示教初始姿态；
4. 清除旧扫描检查点；
5. 保持角度、速度、工件、环境光和增益一致；
6. 启动新的 rosbag 会话后重新规划、批准和执行。

不要用 `ros2 param set /hik_single_camera exposure_us ...` 代替重启。当前相机在启动时把曝光
写入设备，运行中只修改 ROS 参数值不会重新配置相机采集。

### 12.2 同一 bag 可调项目

- 激光条纹 ROI；
- 背景估计窗口；
- 最小原始亮度和激光响应阈值；
- 饱和、多峰、线宽、连续性和置信度门槛；
- 中心线拟合方法；
- TF/时间同步算法；
- 体素大小、离群点过滤、邻域平滑和表面重建。

比较指标建议至少包括：有效条纹点数、原图掉帧率、饱和点比例、中心位置抖动、逐帧轮廓
中断数、TF 拒绝帧数、最终点密度以及已知平面的 RMS 误差。

## 13. 扫描异常或返程异常时

如果机械臂仍在危险运动，优先使用现场急停，不要等待 rosbag 或软件服务。

非紧急的软件停止流程：

```bash
ros2 service call /scanner_650/stop_workspace_coarse_scan \
  std_srvs/srv/Trigger '{}'
ros2 service call /scanner_650/set_laser \
  std_srvs/srv/SetBool '{data: false}'
ros2 service call /scanner_650/set_accumulation \
  std_srvs/srv/SetBool '{data: false}'
```

确认机械臂停稳和激光关闭后，再停止 rosbag。保留异常前后的数据，不要立即删除会话。可把
检查点和崩溃回溯复制到当前会话：

```bash
SESSION_DIR="$(<../myline_hik/data/rosbags/scanner_650/latest_session.txt)"
cp -a ../myline_hik/data/scans/scanner_650/workspace_coarse_checkpoint.yaml \
  "${SESSION_DIR}/metadata/" 2>/dev/null || true
cp -a log/crash "${SESSION_DIR}/metadata/" 2>/dev/null || true
```

这样 rosbag、终端日志、检查点和原生调用栈处于同一个实验目录中。

## 14. 不使用封装脚本的最小 rosbag 命令

只为快速查看激光原图时，可以直接录制最小话题集：

```bash
cd /path/to/vizum-line-scan-gui/ros2_fr5
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record \
  --output /tmp/scanner_650_debug_bag \
  --storage sqlite3 \
  --max-cache-size 1073741824 \
  --qos-profile-overrides-path \
    src/fr5_scanner_650/config/scanner_650_rosbag_qos.yaml \
  /scanner_650/image_raw \
  /scanner_650/camera_info \
  /scanner_650/profile_cloud \
  /scanner_650/laser_status \
  /scanner_650/workspace_coarse_scan_status \
  /joint_states /tf /tf_static /rosout
```

该命令不会保存参数、标定快照、Git 状态、终端服务响应或最终 PLY。正式实验仍应使用
`record_scanner_650_scan.sh`。

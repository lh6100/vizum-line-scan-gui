# FR5 搭载海康相机与 650 nm 红色线激光建立点云实施方案

## 1. 目标

使用 FR5 机械臂带动已标定的 `scanner_650`（160 万海康面阵相机、6 mm 镜头和
650 nm 红色线激光）扫描头运动：

```text
FR5 法兰
   └── 刚性支架
       ├── 海康相机 + 镜头
       └── 红色线激光器
```

每张相机图像通过线激光三角测量重建一条三维轮廓；机械臂移动后，将不同位姿下的三维轮廓统一变换到 `base_link`，累计形成点云。

总体数据链：

```text
海康 Mono8 原图
  -> 激光条纹亚像素中心
  -> 去畸变后的相机射线
  -> 射线与激光平面求交
  -> 相机坐标系单条三维轮廓
  -> 当帧 FR5 法兰位姿 + 海康手眼矩阵
  -> base_link 下的三维轮廓
  -> 多帧累计、过滤、降采样
  -> PLY/PCD 与 ROS PointCloud2
```

## 2. 当前已确认条件

### 2.1 硬件与网络

- 机器人：Fairino FR5，当前控制器地址 `192.168.1.200`。
- 相机：Hikrobot `MV-CS016-10GM`，GigE 面阵相机。
- 相机地址：最近实机枚举为 `192.168.7.45`。
- 主机相机网卡：`enp4s0 = 192.168.7.247/24`。
- 当前图像：`1440 x 1080`、`Mono8`。
- 当前测试状态：约 `65 fps`、曝光 `1825 us`、增益 `0 dB`。
- 当前满幅带宽：约 `820 Mbps`，已经接近千兆以太网的有效上限。
- MVS Linux SDK 已安装在 `/opt/MVS`：
  - 头文件：`/opt/MVS/include/MvCameraControl.h`
  - 动态库：`/opt/MVS/lib/64/libMvCameraControl.so`
  - C++ 示例：`/opt/MVS/Samples/64/C++/General/GrabImage/`
  - Python/OpenCV 示例：`/opt/MVS/Samples/64/OpenCV/Python/GrabImage_Cv/`

### 2.2 仍需补充的硬件参数

在最终确定扫描范围和精度之前，需要记录：

- 镜头焦距；
- 最小、典型、最大工作距离；
- 实际扫描宽度、深度范围和长度；
- 650 nm 激光的功率、扇角和激光等级；
- 650 nm 激光器 TTL 输入的电气门槛、默认失效状态和硬件下拉；
- 相机是否有可用的硬触发输入和 `ExposureActive` 输出；
- 目标点间距、允许的缺点率和最终测量精度。

这些值必须实测或从设备铭牌/说明书取得，不应猜测后写入标定配置。

## 3. 坐标系与单位约定

建议使用以下坐标系：

| 名称 | 含义 |
| --- | --- |
| `B` / `base_link` | FR5 基座坐标系 |
| `F` / `fairino_flange_reported` | 控制器报告的实时法兰坐标系 |
| `C` / `hik_camera_optical_frame` | 海康相机光学坐标系，X 向右、Y 向下、Z 向前 |
| `S` / `hik_scan_tcp` | 扫描头安全运动 TCP，由机器人控制器配置 |

核心变换：

```text
T_BC(t) = T_BF(t) * T_FC
P_B     = T_BF(t) * T_FC * P_C
```

其中：

- `T_BF(t)`：图像曝光时刻的 FR5 实际法兰位姿；
- `T_FC`：重新标定得到的法兰到海康相机手眼矩阵；
- `P_C`：线激光三角测量得到的相机坐标系三维点；
- `P_B`：最终输出到 `base_link` 的点。

单位规则：

- 标定和离线重建核心统一使用 `mm`；
- Fairino 位姿使用 `mm / deg`；
- ROS TF 和 `sensor_msgs/msg/PointCloud2` 发布边界转换为 `m / quaternion`；
- 每份 YAML 必须明确写出单位和矩阵方向，禁止只写一个无语义的 `matrix`。

## 4. 推荐工程目录

```text
myline_hik/
├── CMakeLists.txt
├── README.md
├── config/
│   ├── hik_camera.yaml
│   ├── hik_intrinsics.yaml
│   ├── hik_laser_plane.yaml
│   ├── hik_handeye.yaml
│   ├── hik_scan_tool.yaml
│   └── hik_scan.yaml
├── include/myline_hik/
│   ├── HikCamera.h
│   ├── LaserStripeExtractor.h
│   ├── LaserTriangulator.h
│   ├── HikHandEye.h
│   └── RobotScanSession.h
├── src/
├── tools/
│   ├── hik_grab_once.cpp
│   ├── hik_intrinsic_calibrate.cpp
│   ├── hik_stripe_extract.cpp
│   ├── hik_laser_plane_calibrate.cpp
│   ├── hik_profile_reconstruct.cpp
│   ├── hik_handeye_calibrate.cpp
│   ├── hik_robot_step_scan.cpp
│   └── hik_continuous_scan.cpp
└── data/
    ├── raw/
    ├── calibration/
    ├── sessions/
    └── output/
```

标定结果、扫描参数与程序分开管理。原始数据不可被处理后的结果覆盖。

## 5. 分阶段实施步骤

### 阶段 0：完成扫描头机械安装与安全检查

#### 操作

1. 将相机、镜头和激光器固定在同一块刚性金属支架上。
2. 让激光平面与相机光轴形成明显三角测量夹角，可从约 `20°–40°` 的机械布局开始评估。
3. 在全部工作距离内确认激光线都能落入相机视场。
4. 锁紧镜头焦距、焦点和光圈。
5. 固定激光器角度；标定完成后禁止单独调整激光器。
6. 给扫描头建立独立的机器人工具坐标 `hik_scan_tcp`。
7. 建立扫描头的简化碰撞外形、线缆活动范围和安全工作空间。
8. 按激光等级配置护目镜、遮光、联锁和物理急停旁站。

#### 禁止事项

- 不使用当前焊枪 `tool_id: 4` 代替扫描头 TCP；
- 不复用 Vizum 相机的手眼矩阵；
- 不在标定完成后再次调焦或改变相机/激光相对位置。

#### 阶段验收

- [ ] 相机与激光器在支架上无可感知松动；
- [ ] 最近和最远工作距离均可清晰成像；
- [ ] 扫描短路径无碰撞、无奇异位形、线缆不拉扯；
- [ ] 扫描头 TCP 已在控制器中单独定义并核验。

### 阶段 1：实现海康相机稳定取图

此阶段不连接机器人、不做点云。

#### 第一版程序

实现 `hik_grab_once`：

1. 按序列号或 IP 打开 `MV-CS016-10GM`；
2. 设置 `Mono8`；
3. 关闭自动曝光和自动增益；
4. 固定曝光、增益、帧率和 ROI；
5. 第一版使用软件触发；
6. 每次触发等待一张新帧；
7. 保存原始 PNG 和元数据；
8. 正常释放图像缓存、停止取流并关闭设备。

建议起始参数：

```text
PixelFormat = Mono8
ExposureAuto = Off
GainAuto = Off
ExposureTime = 1825 us
Gain = 0 dB
TriggerMode = On
TriggerSource = Software
AcquisitionFrameRate = 20–30 fps
```

满幅 65 fps 约占 `820 Mbps`，第一阶段先降低帧率。后续连续扫描应围绕激光可能出现的区域设置 ROI，但不能窄到裁掉深度变化引起的横向位移。

#### 每帧必须保存

```text
scan_id
frame_number
device_timestamp
host_receive_timestamp
exposure_us
gain_db
width, height
roi_x, roi_y
trigger_mode
raw_image_path
receive_status
```

#### 测试

1. 软件触发 1000 次；
2. 检查每次只返回一张新帧；
3. 检查帧号是否连续；
4. 连续运行至少 10 分钟；
5. 统计超时、重复帧、帧号缺口和网络错误。

#### 阶段验收

- [ ] 1000 次软件触发没有旧帧或重复帧；
- [ ] 所有帧均有设备时间戳和帧号；
- [ ] 激光条纹没有大面积灰度截顶；
- [ ] 背景与激光信号具有稳定对比度；
- [ ] 丢帧和超时均被检测并写入日志。

> 运行自研程序前必须关闭 MVS 客户端，避免两个控制端争抢相机。

### 阶段 2：标定海康相机内参与畸变

#### 采集要求

1. 关闭激光；
2. 使用尺寸准确的 ChArUco 或棋盘格标定板；
3. 保持最终使用的焦距、焦点、光圈、分辨率和 ROI；
4. 拍摄 `20–30` 张清晰图像；
5. 标定板覆盖画面中心和四角；
6. 覆盖最近、典型和最远工作距离；
7. 包含绕多个轴的倾斜，不能全是正对相机的图像。

#### 输出

```text
myline_hik/config/hik_intrinsics.yaml
```

至少包含：

```yaml
image_width: 1440
image_height: 1080
unit: pixel
camera_matrix: [<fx>, 0, <cx>, 0, <fy>, <cy>, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients: [<k1>, <k2>, <p1>, <p2>, <k3>]
reprojection_rms_px: <measured_value>
```

#### 阶段验收

- [ ] 总体重投影 RMS 初始目标 `< 0.3 px`；
- [ ] 单张最大误差初始目标 `< 1 px`；
- [ ] 去畸变后的直线没有明显弯曲；
- [ ] 标定图覆盖实际扫描深度和完整视场。

重新调焦、换镜头、改变 binning 或缩放图像后必须重新标定。只裁 ROI 时也必须正确更新主点坐标。

### 阶段 3：提取激光条纹亚像素中心

截图中的激光线接近竖直，因此第一版可按图像行提取：每行输出一个亚像素横坐标 `u`。

#### 推荐流程

```text
原始 Mono8
  -> 激光关闭背景差分或局部背景抑制
  -> 有效 ROI
  -> 每行多个局部峰值候选
  -> 灰度重心/抛物线/Steger 亚像素中心
  -> 动态规划选择最连续路径
  -> 异常宽度、饱和、跳变和低置信度过滤
```

每个候选点记录：

```text
row
u_subpixel
peak_intensity
width_px
saturation_ratio
gradient_symmetry
confidence
accepted/rejected
reject_reason
```

#### 实施顺序

1. 在漫反射平板上先使用局部峰值 + 灰度重心；
2. 增加连通性和前后行连续约束；
3. 在金属高反光场景升级为 Steger/Hessian 候选；
4. 使用动态规划/Viterbi 从多候选中选择主激光路径；
5. 保存带中心线覆盖图，便于人工检查。

#### 阶段验收

- [ ] 漫反射平板有效行提取率 `> 95%`；
- [ ] 静态重复拍摄时亚像素中心抖动满足目标精度；
- [ ] 过曝反光区域不会被强制输出为有效激光点；
- [ ] 激光断裂时保留缺口，不使用错误插值伪造数据。

### 阶段 4：标定相机坐标系中的激光平面

激光平面表示为：

```text
n_x X + n_y Y + n_z Z + d = 0
```

其中 `n` 必须归一化，`d` 的单位与重建单位一致，建议为 `mm`。

#### 采集方法

使用阶段 2 的平面标定板，在 `10–20` 个不同距离和倾角下采集：

1. 激光关闭，识别 ChArUco，求 `T_camera_board`；
2. 标定板保持不动，打开激光，提取亚像素条纹；
3. 将条纹像素去畸变为相机射线；
4. 将射线与已知标定板平面求交，得到相机坐标系三维点；
5. 汇总所有姿态的点；
6. 使用 RANSAC 去除异常点；
7. 使用最小二乘/SVD 拟合固定激光平面；
8. 使用未参与拟合的标定板姿态做独立验证。

#### 输出

```text
myline_hik/config/hik_laser_plane.yaml
```

示例结构：

```yaml
frame_id: hik_camera_optical_frame
unit: mm
normal: [<nx>, <ny>, <nz>]
d: <d_mm>
fit_rms_mm: <measured_value>
fit_p95_mm: <measured_value>
calibration_sample_count: <count>
```

#### 阶段验收

- [ ] 标定板姿态覆盖完整工作距离和视场；
- [ ] 法向量长度为 1；
- [ ] 独立验证样本无明显近端/远端系统偏差；
- [ ] 初始工程目标：点到激光平面 RMS `< 0.2–0.3 mm`、P95 `< 0.5 mm`。

上述数值是第一轮工程门槛，不代表当前硬件的保证精度，最终必须按实际工件公差重新设定。

### 阶段 5：完成静态单帧三角重建

机器人保持静止。对于每个去畸变后的条纹像素 `(u, v)`：

```text
r = K^-1 * [u, v, 1]^T
lambda = -d / (n^T * r)
P_C = lambda * r
```

过滤条件：

- `lambda <= 0`；
- `abs(n^T * r)` 太小；
- 超出标定深度范围；
- 条纹置信度低；
- 条纹过曝或宽度异常；
- 三维点超出扫描工作空间。

输出一条 `hik_camera_optical_frame` 下的三维轮廓，并保存为 CSV/PLY。

#### 阶段验收

- [ ] 平板轮廓拟合无明显弯曲；
- [ ] 已知台阶或量块高度方向正确；
- [ ] 静态重复拍摄的轮廓重合；
- [ ] 单帧轮廓精度达标后才进入机械臂拼接阶段。

### 阶段 6：重新标定海康相机手眼关系

现有 `config/handeye_config.yaml` 属于 Vizum 左相机，不能复用。

目标是求：

```text
T_flange_hik_camera = T_FC
```

#### 采集方法

1. 将 ChArUco 标定板固定在机器人基座附近，全程不能移动；
2. 关闭激光；
3. 机器人采集至少 `15–25` 个不同姿态；
4. 每个姿态必须完全停稳；
5. 读取实际 `T_base_flange`；
6. 软件触发一张新的海康原图；
7. 从图像求 `T_camera_board`；
8. 再次读取实际 `T_base_flange`，前后变化必须不大于 `0.10 mm / 0.05 deg`；
9. 姿态包含绕至少两个轴的充分旋转；
10. 比较 OpenCV 五种 `calibrateHandEye` 方法并鲁棒剔除离群样本；
11. 使用未参与求解的姿态独立验证。

验证关系：

```text
T_base_board_i = T_base_flange_i * T_flange_hik_camera * T_camera_board_i
```

固定标定板在所有机器人姿态下计算出的 `T_base_board_i` 应保持一致。

#### 输出

```text
myline_hik/config/hik_handeye.yaml
```

文件必须明确：

```yaml
parent_frame: fairino_flange_reported
child_frame: hik_camera_optical_frame
mode: camera_to_flange
translation_unit: mm
T_flange_camera: [<16 row-major values>]
quality:
  base_board_translation_rms_mm: <measured_value>
  base_board_rotation_rms_deg: <measured_value>
```

#### 阶段验收

- [ ] 初始目标：标定板 base 下平移一致性 RMS `< 1 mm`；
- [ ] 初始目标：旋转一致性 RMS `< 0.3 deg`；
- [ ] 多个独立验证姿态无固定方向偏差；
- [ ] 将海康相机 TF 显示到 RViz 后，方向与实际安装一致。

### 阶段 7：实现停稳式机器人扫描 MVP

第一版必须使用“移动一点、停稳、拍一帧”的方式。不要直接做连续运动。

当前软件实现（2026-07-16）：独立程序 `HikConstantLaserScan` 和启动脚本 `run_constant_laser_scan.sh` 已实现常亮单帧、直线路径 dry-run、同一 Fairino SDK 会话非阻塞 `MoveL`/`StopMotion`、采图前后法兰静止检查、`T_BF·T_FC·P_C` 累计、清单以及 raw/voxel PLY。核心算法和无设备启动已通过自动测试；真实 FR5 短路径仍必须按本节安全步骤现场验收，不能把“编译通过”等同于运动安全通过。

普通工件轮廓可以合法地包含弧面和台阶，因此三维轮廓的“最佳拟合直线 RMS”只作为记录项；仅在标准平板验收模式中才允许作为硬停止门槛。常亮单帧还必须记录条纹中心饱和比例，饱和比例偏高时优先降低曝光或激光功率，而不是放宽几何误差。

#### 每个采样点的严格顺序

```text
生成目标 TCP
  -> MoveL
  -> 等待 GetRobotMotionDone
  -> 静置 100–300 ms
  -> 读取 T_base_flange_before
  -> 海康软件触发
  -> 获取与触发对应的新帧
  -> 读取 T_base_flange_after
  -> 检查前后位姿差是否足够小
  -> 提取条纹并重建 P_C
  -> P_B = T_BF * T_FC * P_C
  -> 保存原始数据和累计点云
```

当前 MVS 采集器按“每次单帧重新开始软件触发取流”工作，并通过返回帧号和设备时间戳留痕；连续取流版本仍需要显式清理缓存和核对触发序号。

#### 首轮扫描参数

1. 路径长度先限制在 `20–50 mm`；
2. 调试步距先使用 `5–10 mm`；
3. 流程稳定后先按当前目标使用 `2 mm`，有实际横向分辨率和重复性依据后再逐渐减小到 `0.5–1 mm`；
4. 机器人使用低速；
5. 扫描方向大致垂直于工件表面的激光线方向；
6. 保持扫描姿态不变，只做平移；
7. 所有真运动前先打印完整目标列表并做 dry-run；
8. 物理急停旁站，先在空场完成短行程验证。

#### 可复用代码

- `RobotOffsetCapturePlanner::buildOffsetSamples()`：生成包含终点的等间距采样；
- `flangeAxisDirectionInBase()`：把法兰局部轴转到 base；
- `offsetTcpPose()`：保持 TCP 姿态，只改变位置；
- `FairinoRobotClient::moveL()`：非阻塞 MoveL；
- `waitRobotMotionDone()`：等待到位；
- `getCurrentFlangePose()`：读取当前实际法兰位姿；
- `TransformUtils`：矩阵乘法与点变换。

只能复用运动和数学骨架。原 Vizum 相机取图、双目 Q 矩阵、手眼数值和“线激光固定”几何均不能用于海康扫描头。

### 阶段 8：点云累计、保存与 RViz 显示

建议 ROS 2 接口：

```text
/hik/image_raw                 sensor_msgs/Image
/hik/camera_info               sensor_msgs/CameraInfo
/hik/laser/profile_cloud       sensor_msgs/PointCloud2
/hik/scan_cloud                sensor_msgs/PointCloud2
```

TF：

```text
base_link
  └── fairino_flange_reported
      └── hik_camera_optical_frame
```

单帧轮廓可使用 `hik_camera_optical_frame`，累计扫描云必须使用：

```text
frame_id = base_link
```

点字段建议保留：

```text
x, y, z
intensity
confidence
frame_number
profile_index
```

累计后按顺序处理：

1. 工作空间裁剪；
2. 低置信度点剔除；
3. 半径或统计离群点过滤；
4. 体素降采样；
5. 保存原始云和处理后云；
6. 发布到 RViz。

不要使用 ICP 掩盖错误的手眼标定或时间同步。机器人位姿和标定正确时，各条轮廓首先应该自然对齐；ICP 只能作为可选的微调步骤。

### 阶段 9：升级为连续运动扫描

只有停稳扫描精度和重复性达标后，才能进入连续模式。

#### 连续扫描的核心要求

每张图必须绑定“曝光中点时刻”的实际法兰位姿，不能使用图像回调到达时的最新位姿。

优先同步方案：

1. 机器人或 PLC 输出同源硬触发；
2. 相机使用硬件 `FrameStart`；
3. 相机 `ExposureActive` 控制激光 TTL；
4. 记录相机设备时间戳和帧号；
5. 机器人位姿进入带时间戳的环形缓冲区；
6. 平移线性插值；
7. 旋转使用 quaternion SLERP；
8. 标定相机与机器人时钟固定偏移；
9. 正向和反向扫描同一平面检查时间延迟。

关系式：

```text
相邻轮廓间距 delta_s = 机器人速度 v / 相机帧率 f
曝光期间运动距离   = v * exposure_time
```

示例：

```text
30 fps，希望轮廓间距 0.5 mm
v = 30 * 0.5 = 15 mm/s
```

当前满幅 65 fps 约占 `820 Mbps`，连续扫描时应优先设置合理 ROI、降低帧率或使用独立相机网卡，并持续检查帧号缺口。

当前 ROS 法兰节点默认 `20 Hz`，并使用 PC 读取时刻给 SDK 缓存状态打时间戳。它适合 RViz 和停稳扫描，不足以直接作为 65 fps 连续扫描的高精度同步源。

## 6. 每帧原始数据契约

每次扫描必须保存可离线重算的完整数据，而不是只保存最终 PLY。

建议每帧记录：

```text
scan_id
profile_index
camera_frame_number
camera_device_timestamp
host_receive_timestamp
estimated_exposure_midpoint
raw_mono8_path
exposure_us
gain_db
roi
trigger_mode
stripe_points_2d
stripe_confidence
T_base_flange_before
T_base_flange_after
T_base_flange_used
joint_positions
camera_intrinsics_file + hash
laser_plane_file + hash
handeye_file + hash
P_camera_profile_path
P_base_profile_path
accepted/rejected
reject_reason
```

这样调整条纹算法、标定或过滤参数后可以离线重算，不需要重新驱动机器人。

## 7. 初始验收方案

以下是第一轮工程目标，最终标准必须按照真实工件公差调整。

| 项目 | 初始目标 |
| --- | --- |
| 相机内参重投影 RMS | `< 0.3 px` |
| 激光平面独立验证 RMS | `< 0.2–0.3 mm` |
| 手眼标定平移一致性 RMS | `< 1 mm` |
| 手眼标定旋转一致性 RMS | `< 0.3 deg` |
| 标准平板点云拟合 RMS | `< 0.3 mm` |
| 已知台阶/量块高度误差 | `< 0.5 mm` |
| 同一路径重复 5 次点到面 RMS | `< 0.3 mm` |
| 正向/反向扫描 P95 差异 | `< 0.5 mm` |
| ROI 内有效轮廓完整率 | `> 95%` |
| 连续采集丢帧率 | `< 0.1%`，且全部缺口可追踪 |

验收顺序：

1. 静态平板单帧轮廓；
2. 静态已知台阶；
3. 不同机器人静止姿态扫描同一平板；
4. 停稳式短路径扫描；
5. 同路径重复扫描；
6. 正向/反向扫描；
7. 最后验证连续扫描。

## 8. 当前必须先处理的安全与软件问题

### 8.1 单一 Fairino SDK 所有者

同一时间只允许一个程序直接连接 FR5：

- VizumScanGUI 的 `FairinoRobotClient`；
- `ros2_fr5` 的 `fairino_state_publisher`；
- Fairino 官方命令节点；
- 新的 Hik 扫描程序。

这些程序不能同时各自调用 `FRRobot::RPC()`。第一版停稳扫描应由一个进程同时拥有机器人运动会话和当点法兰位姿，其他程序只消费它发布的数据。

### 8.2 `GetRobotRealTimeState()` ABI 风险

当前 GUI 安全前检使用 `GetRobotRealTimeState()` 读取整包 `ROBOT_STATE_PKG`。现场 Fairino 3.9.4 动态库与头文件存在结构体 ABI 尺寸不一致风险，已经确认直接调用可能触发内存保护退出。

在真实 MoveL 扫描之前必须：

1. 使用与动态库完全匹配的 SDK 头文件；或
2. 将安全状态读取改为经过验证的单项 getter；
3. 保留同一 SDK 会话上的 `StopMotion()`；
4. 保持 MoveL 非阻塞并持续轮询运动完成状态。

该问题未解决前，不允许依赖现有 GUI 安全前检执行真实扫描。

### 8.3 运动许可

- 标定、静态取图和算法调试期间保持 `dry_run: true`；
- 相机内参、激光平面、静态轮廓和手眼标定全部验收后，才允许短距离真实运动；
- 软件 `StopMotion` 不等于物理急停；
- 扫描头外形、工件、夹具和线缆必须纳入碰撞检查；
- 首次真运动使用低速、短路径、空场并有人旁站物理急停。

## 9. 最容易犯的错误

- 把二维激光像素直接当成三维点，或给所有点固定 Z；
- 复用 Vizum 相机的内参或手眼矩阵；
- 复用焊枪 TCP 作为扫描头 TCP；
- 混淆 `T_FC` 与 `T_CF`；
- 混淆 mm/m、deg/rad 和矩阵左右乘顺序；
- 把相机外壳坐标系当作 ROS optical frame；
- 调焦或移动激光器后不重新标定；
- 只在单一距离标定激光平面；
- 使用整数最亮点代替亚像素中心；
- 对过曝或多次反射仍强制输出一个激光点；
- 机器人刚到位就读取相机缓存旧帧；
- 连续扫描使用回调时的“最新法兰姿态”；
- 全幅 65 fps 运行却不检查 GigE 带宽和帧号；
- 同时运行 MVS 客户端和自研相机程序；
- 同时启动两个 Fairino SDK 客户端；
- 只保存最终点云，不保存原图、帧号、位姿和标定版本；
- 使用 ICP 把错误标定产生的错位表面上强行对齐。

## 10. 推荐开发顺序与完成清单

严格按以下顺序推进：

```text
1. hik_grab_once
2. hik_intrinsic_calibrate
3. hik_stripe_extract
4. hik_laser_plane_calibrate
5. hik_profile_reconstruct
6. hik_handeye_calibrate
7. hik_robot_step_scan
8. hik_point_cloud_rviz
9. hik_continuous_scan
```

总检查表：

- [ ] 扫描头刚性安装和安全 TCP 完成；
- [ ] 海康软件触发、帧号和时间戳稳定；
- [ ] 海康内参与畸变标定通过；
- [ ] 激光亚像素中心提取通过真实表面验证；
- [ ] 激光平面标定通过独立样本验证；
- [ ] 静态单帧三维轮廓精度达标；
- [ ] 海康专用手眼矩阵标定通过；
- [ ] Fairino SDK ABI 和单一连接问题解决；
- [ ] dry-run 输出的扫描目标和路径正确；
- [ ] 停稳式短路径扫描点云精度达标；
- [ ] PLY/PCD 和 RViz `PointCloud2` 输出正常；
- [ ] 重复扫描和正反向扫描通过验收；
- [ ] 最后再实现硬触发连续扫描。

## 11. 下一步

当前下一项不是重新实现取图，而是执行常亮停稳扫描的分级现场验证：

```text
1. ./run_constant_laser_scan.sh
2. 单点常亮验证（机器人不移动）
3. 示教 20–50 mm 起终点并保持 dry-run，核对全部 base 法兰目标
4. 5 mm 步距低速真运动，检查 images、manifest、scan_raw.ply
5. 改为 2 mm 步距并重复同一路径
6. 反向扫描标准平板，检查是否存在方向相关错层
```

现场验证前仍需完成扫描头碰撞外形、线缆范围、起点直达路径和物理急停旁站确认。只有标准平板各轮廓无需 ICP 即自然重合、重复/反向结果无系统偏差后，才进入 ROS 2 `PointCloud2` 与 RViz 显示。

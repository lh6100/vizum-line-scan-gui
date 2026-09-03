# 海康相机与线激光独立标定工具

这个目录提供一个独立的 Qt5 GUI，用于完成：

1. 海康面阵相机的内参和畸变标定；
2. 相机光学坐标系下、当前设备组对应的线激光平面标定；
3. 海康相机到 FR5 法兰的 eye-in-hand 手眼标定；
4. FR5 带动相机的自动停稳标定、留出校验与双相机相对外参生成；
5. 固定内参的同步 ChArUco `stereoCalibrate`、极线质量校验与正式双目 R/T 审批；
6. 灰度双目 SGBM 深度、FR5 曝光时刻位姿插值、base 坐标三维占据体素和二维栅格导出；
7. 不依赖 ROS 2 的 FR5 常亮线激光停稳扫描验证与 base 坐标系 PLY 累计。

标定 GUI 不依赖 ROS 2。“FR5 手眼标定”页仍是只读法兰、人工换姿态；新增的
“自动相机/手眼”页会在默认 dry-run、完整路径预检、显式安全勾选和二次确认后，发送低速
`MoveL` 与 `StopMotion`。它不会自动使能机器人或切换控制器模式。独立的
`HikConstantLaserScan` 扫描程序使用相同的安全运动边界。四个 GUI 都通过鲁班猫 4 V1
受限控制服务管理两路激光 TTL，日常操作不使用 SSH 密码或 `sudo`。

当前提供两个完全隔离的设备组：

| 设备组 | 光学系统 | TTL | 正式标定 |
| --- | --- | --- | --- |
| `scanner_450` | 450 nm、130 万相机、12.5 mm | Pin 11 / GPIO15 | `config/devices/scanner_450/` |
| `scanner_650` | 650 nm、160 万相机、6 mm | Pin 7 / GPIO16 | `config/devices/scanner_650/` |

标定与扫描程序默认各显示两个独立标签页，也可用
`--profile scanner_450` 或 `--profile scanner_650` 只打开一组。第一阶段一次只允许一组
执行线激光真实扫描；新增双目程序会同时占用两台相机，当前采用有偏差门槛的软件时间配对，
尚不等同于共同硬件触发的严格同步。详细边界和验收项见
[`docs/双线激光双相机系统需求与实施.md`](docs/双线激光双相机系统需求与实施.md)。
高反光场景的 `Legacy/Shadow/Quality` 策略、同脊峰合并、局部
min-marginal/K 分支、`AMBIGUOUS_MULTIPATH` 硬隔离、base_link V 坡口唯一性验证、
质量/rejected 点云和离线回放方法见
[`docs/stripe_quality_pipeline.md`](docs/stripe_quality_pipeline.md)。
scanner_650 固定全局蛇形、`base_link` 质量地图、局部 greedy/beam NBS、
FR5 路径评估、配置和安全边界见
[`docs/scanner_650_adaptive_scan.md`](docs/scanner_650_adaptive_scan.md)。

正式配置按 profile 固定映射：

| 配置 | `scanner_450` | `scanner_650` |
| --- | --- | --- |
| 内参 | `config/devices/scanner_450/hik_intrinsics.yaml` | `config/devices/scanner_650/hik_intrinsics.yaml` |
| 激光平面 | `config/devices/scanner_450/hik_laser_plane.yaml` | `config/devices/scanner_650/hik_laser_plane.yaml` |
| 手眼 | `config/devices/scanner_450/hik_handeye.yaml` | `config/devices/scanner_650/hik_handeye.yaml` |
| 连续同步 | `config/devices/scanner_450/synchronization.yaml` | `config/devices/scanner_650/synchronization.yaml` |
| 自适应扫描 | 不启用 | `config/devices/scanner_650/adaptive_scan_650.yaml` |
| 相机 frame | `hik_450_camera_optical_frame` | `hik_camera_optical_frame` |

> 2026-08-04 现场依赖审计：`scanner_450` 的内参、激光平面和手眼 SHA-256
> 一致；`scanner_650` 的激光平面已绑定新内参，但正式手眼仍声明旧内参哈希
> `7a73017a…`，与当前内参哈希 `2f25f837…` 不一致。按当前现场要求，线扫启动脚本
> 默认原样使用 `config/devices/scanner_650/` 中的数值建图，不重新求解、也不改写
> 标定文件；页面显示橙色来源警告，并把差异写入每个 session。

直接启动即可使用设备目录中现有的 `T_flange_camera`：

```bash
./run_constant_laser_scan.sh
```

该模式只放宽手眼文件声明的内参 SHA-256 不一致；激光平面必须仍绑定当前内参，
相机型号、SN 和坐标系检查也不会放宽。界面以橙色显示，并在
`session_metadata.json` 中记录 `provenance_override_active=true`。
需要恢复严格哈希阻塞时使用
`HIK_STRICT_CALIBRATION=1 ./run_constant_laser_scan.sh`。

两台相机共同的相对外参写入 `config/hik_dual_camera_extrinsics.yaml`；文件同时记录两份
手眼源文件及 SHA-256，任一手眼重新标定后都必须重新生成。
用于视差计算的专用双目外参另存为 `config/hik_stereo.yaml`。它必须由两台相机同步看到
同一 ChArUco 板的图像执行 `stereoCalibrate(..., CALIB_FIX_INTRINSIC)` 得到；程序不会把
两份独立手眼推导的相对变换静默当成高精度双目 R/T。

## 1. 开始前必须确认

- 将相机、镜头和线激光固定到最终使用的机械结构上。完成标定后不能再改变镜头焦距、对焦、光圈、相机分辨率或 ROI，否则需要重新标定。
- 运行本工具前，先在 MVS 中停止取流、关闭设备并完全退出 MVS；同时关闭其他相机采集程序。本工具以独占模式打开相机，MVS 仍占用时会连接失败。
- `scanner_650` 最近实机枚举地址是 `192.168.7.45`，并绑定型号
  `MV-CS016-10GM`、序列号 `DA8784601`；`scanner_450` 已绑定地址
  `192.168.1.46`、型号 `MV-CS013-60GN`、序列号 `DB0403208`。
- 内参采集时必须确认两路线激光均关闭。
- 激光平面标定的每一组 `laser-off` / `laser-on` 必须使用完全相同的曝光和增益，而且两帧之间相机与标定板都不能移动。
- 标定 GUI 与扫描 GUI 共用鲁班猫受限 TTL 控制器。除“自动相机/手眼”页外，标定 GUI
  其他页面不控制机械臂运动；自动页只在 dry-run 预检和两次人工确认后发送低速 MoveL。
  实时内参/手眼采样会先确认两路 LOW，激光平面和静态轮廓实时配对会自动切换
  off/on，并且只有板端 ACK、LubanCat-4 V1 引脚映射和 GPIO 逻辑回读一致时才会触发
  相机。GUI 另提供“同时开启 450 + 650 nm”手动观察按钮；双开状态不会被扫描流程
  视为本组激光就绪。on 帧返回、错误、切换设备页或退出时都会请求两路 LOW。
- GPIO 逻辑回读不能证明排针电压或激光实际出光。首次部署后必须先断开激光 TTL 负载，用万用表或示波器验收 Pin 11、Pin 7；软件不能代替硬件下拉、急停和激光安全回路。
- Fairino SDK 同一时间只能由一个进程占用。进入手眼或扫描页面前，必须停止
  `ros2_fr5` 的 `fairino_state_publisher`、旧 GUI 或其他 FR5 SDK 客户端。同一 GUI
  内的两个 profile 共用唯一 FR5 会话，空闲切页无需重连；主动断开或连接失败后因
  3.9.4 后台线程限制仍需重启该 GUI 才能再次连接。

## 2. 独立构建和运行

要求：C++17、CMake、Qt5 Widgets、Eigen3、OpenCV（包含 `calib3d` 和 `aruco`）、海康 MVS SDK，以及手眼实时采集所需的 Fairino C++ SDK。代码默认查找：

```text
/opt/MVS/include/MvCameraControl.h
/opt/MVS/lib/64/libMvCameraControl.so
/path/to/vizum-line-scan-gui/SDK/fairino-cpp-sdk-3.9.4/
```

本工程需要对 FAIRINO 3.9.4 增加20004逐包回调。两个启动脚本会自动调用
`tools/build_patched_fairino_sdk.sh`：应用受版本控制的补丁、备份原始
`libfairino.so.2.3.4`，并重建SDK。手工配置CMake前应先执行：

```bash
./tools/build_patched_fairino_sdk.sh
```

补丁只新增 `RecvPkg()` 成功返回处的 `CLOCK_MONOTONIC_RAW` 时间戳、完整包回调和
互斥保护的getter快照；不会改变机器人运动指令。

直接构建并运行：

```bash
cd /path/to/vizum-line-scan-gui/myline_hik
./run_calibration_gui.sh
```

运行常亮单帧停稳扫描验证程序：

```bash
cd /path/to/vizum-line-scan-gui/myline_hik
./run_constant_laser_scan.sh
```

先运行双目专用外参标定，再运行深度/环境地图：

```bash
cd /path/to/vizum-line-scan-gui/myline_hik
./run_stereo_calibration.sh
./run_stereo_mapper.sh
```

运行双相机/双线激光曝光参数测试页：

```bash
cd /path/to/vizum-line-scan-gui/myline_hik
./run_exposure_test.sh
```

该页面可同时连接 `scanner_450` 与 `scanner_650` 两台相机，分别设置曝光时间和增益，
并独立控制 450/650 nm 两路 TTL。支持单台拍摄或两台同时拍摄。图片保存到页面指定目录，
文件名包含设备组、相机型号/SN、实际分辨率、`Mono8`、实际曝光、实际增益和毫秒时间戳；
同一目录下的 `captures.csv` 还会记录帧号、相机/主机原始时间戳及拍摄时的 TTL 回读，
供后续筛选最优曝光参数。

自动执行“双激光同时亮、双相机、1000--5000 us、步长 100 us、0 dB”的完整扫频：

```bash
cd /path/to/vizum-line-scan-gui/myline_hik
./run_exposure_sweep.sh
```

程序默认先确认两台相机型号/SN，再确认 450/650 nm 两路 TTL 均为 HIGH，等待光源稳定
1 秒；每个曝光档先丢弃 1 组稳定帧，再并行触发两台相机并各保存 1 张。结束后校验每台
相机均有 41 张 PNG、`captures.csv` 有 82 行数据，并等待板端确认两路 TTL 关闭。
输出目录还包含 `sweep_plan.json` 和 `sweep_summary.json`，中途失败不会被误报为完整数据。

只检查计划而不连接硬件：

```bash
./run_exposure_sweep.sh --dry-run
```

可选参数包括 `--start-us`、`--stop-us`、`--step-us`、`--gain-db`、
`--warmup-frames`、`--frames-per-exposure` 和 `--output`。例如每档保存 3 帧用于分析
激光功率波动：

```bash
./run_exposure_sweep.sh --frames-per-exposure 3
```

首次在鲁班猫安装免密码、失效关光的 TTL 服务：

```bash
./tools/setup_laser_control.sh
```

这一步需要人工核对 SSH host key，并可能只在安装阶段要求 `cat`/`sudo` 密码。完成后
GUI 使用专用 forced-command 密钥；启动、状态查询、开关光和退出均不再要求密码。
板端实现、引脚映射和失效行为见
[`deploy/lubancat4-v1/README.md`](deploy/lubancat4-v1/README.md)。
当前 GUI 固定使用本项目现场值 `192.168.1.12:22` 和默认专用密钥目录；若安装脚本使用
了非默认 `--host`、`--port` 或 `--key-dir`，还必须通过代码中的
`LineLaserConnectionConfig` 同步配置，不能只改板端。

两个程序都会占用海康相机、鲁班猫 TTL 控制租约和 Fairino SDK，不能同时运行。
启动扫描程序前应退出标定 GUI、MVS、`fairino_state_publisher` 以及其他 FR5 SDK
客户端。

启动脚本会主动隔离 Anaconda/Conda 的 Qt 和库路径，强制使用 Ubuntu 系统 Qt、系统 OpenCV，再运行程序。需要分开构建时，可使用另一个构建目录执行同样的隔离：

```bash
cd /path/to/vizum-line-scan-gui/myline_hik
SYSTEM_PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
  PATH="$SYSTEM_PATH" \
  /usr/bin/cmake -S . -B build-system \
    -DCMAKE_BUILD_TYPE=Release \
    -DQt5_DIR=/usr/lib/x86_64-linux-gnu/cmake/Qt5 \
    -DQt5Core_DIR=/usr/lib/x86_64-linux-gnu/cmake/Qt5Core \
    -DQt5Gui_DIR=/usr/lib/x86_64-linux-gnu/cmake/Qt5Gui \
    -DQt5Widgets_DIR=/usr/lib/x86_64-linux-gnu/cmake/Qt5Widgets \
    -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
  PATH="$SYSTEM_PATH" \
  /usr/bin/cmake --build build-system -j"$(/usr/bin/nproc)"

ctest --test-dir build-system --output-on-failure

env -u CONDA_PREFIX -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
  PATH="$SYSTEM_PATH" \
  ./build-system/HikLineLaserCalibration
```

如果手工 CMake 找到了 Anaconda 自带的 Qt，当前工程会直接终止配置，避免把 Conda Qt 和系统 OpenCV 混合链接。此时应优先使用 `./run_calibration_gui.sh`，或使用上面的完整手工命令，不要忽略这个错误。

如果配置时显示 `GUI will build in offline-only mode`，说明没有找到 MVS SDK。离线导入仍可用于求解和保存候选，但由于无法验证相机身份及采集参数，不能提升为正式配置。

如果显示 `Fairino SDK not found`，内参、激光平面和静态轮廓仍可使用，但手眼页面不能实时读取法兰。SDK 位于其他目录时，在启动前设置 `FAIRINO_SDK_DIR=/绝对路径`。

## 3. 统一标定板定义：11×8、24/18 mm

`scanner_450` 与 `scanner_650` 统一使用以下标定板参数：

| 参数 | 正确值 |
| --- | ---: |
| OpenCV `squaresX` | 11 |
| OpenCV `squaresY` | 8 |
| 方格边长 | 24 mm |
| ArUco Marker 边长 | 18 mm |
| 字典 | `DICT_4X4_50` |
| 图案尺寸 | 264 mm × 192 mm |
| ChArUco 内角点数 | 70 |

这里使用的是 OpenCV 定义：`squaresX` 是标定板自身 X 方向的方格列数，`squaresY` 是自身 Y 方向的方格行数。因此：

```text
宽度  = 11 × 24 mm = 264 mm
高度  = 8 × 24 mm = 192 mm
角点数 = (11 - 1) × (8 - 1) = 70
```

设计图中“8×11（行×列）”对应 OpenCV 参数 `squaresX=11、squaresY=8`。相机画面中的横竖方向不决定 X/Y；把板在镜头前旋转 90°，参数仍保持 11×8，不能改成 8×11。GUI 的“恢复统一参数”按钮会恢复上述唯一标准。

添加任一内参、激光或手眼样本后，板参数会锁定。要修改它，必须清空这些样本并取消尚未完成的采集事务。

本工具按 OpenCV 4.5 的 legacy ChArUco 布局生成三维角点。当前 CMake 会拒绝 OpenCV 4.6 或更高版本，避免新版坐标约定与 YAML 中声明的 legacy 布局不一致。

## 4. 第一步：连接相机并确认图像

1. 完全退出 MVS。
2. 启动 GUI，确认当前设备组的 IP。`scanner_650` 默认填入最近实机枚举值
   `192.168.7.45`；`scanner_450` 使用已绑定地址 `192.168.1.46`。
3. 设置曝光、增益和超时。界面默认值分别为 `1825 us`、`0 dB` 和 `3000 ms`，它们只是起点，应以现场图像为准。
4. 点击“连接”。成功后程序记录相机型号、序列号和 IP，将缩放复位为 1、ROI 复位为全幅，并工作在 `Mono8`、软件触发模式。
5. 点击“单帧”，检查图像尺寸、清晰度、亮度以及是否过曝。
6. 固定好镜头对焦、焦距和光圈后，不要再改变它们。

## 4A. 自动完成内参、手眼外参与留出校验

该功能按设备页运行：先在 `scanner_450` 页完成一次，再在 `scanner_650` 页完成一次。
两个相机刚性安装在同一法兰上，但每台相机仍使用自己的 IP、SN、内参和
`T_flange_camera`。当两份正式手眼都批准后，程序会从共同法兰坐标推导并原子更新
`config/hik_dual_camera_extrinsics.yaml`，其中
`T_first_camera_second_camera` 把第二台相机坐标变换到第一台相机坐标。

开始前：

1. 将标准 ChArUco 板牢固固定在工作台，不能随机器人运动；把板放在当前相机画面中心，
   确保对焦、曝光正常。
2. 以当前法兰为中心，为相机和线缆预留至少约 `450 mm` 的完整运动包络；移走焊枪附近、
   板周围和机器人路径上的人员与物体，确认物理急停可触达。
3. 连接当前海康相机、鲁班猫 TTL 和共享 FR5。自动规划需要当前 profile 已有一份已批准
   内参/手眼作为“运动种子”；种子只用于定位固定板和生成安全的相机视角，不会把旧数值
   复制到新求解。完全首次安装没有种子时，应先在手动页完成一次粗内参和粗手眼。
4. 进入“自动相机/手眼”，先保持“仅规划/预检”勾选并开始。程序只采一张种子图，生成
   36 个采样姿态和返回起点路径，并逐段检查 FR5 IK、软限位余量和数值奇异性；不会运动。
5. 检查表格和运动包络。当前没有经过验证的工位几何碰撞模型，因此预检中的碰撞状态始终
   是 `UNKNOWN`，不能把 IK 通过理解为无碰撞。
6. 需要真实执行时取消 dry-run，设置速度（默认 `20 mm/s`）、加速度和停稳时间，勾选完整
   路径/线缆/急停安全确认，再点击开始并在最终对话框确认。

真实执行固定使用以下事务：

```text
完整路径预检
  -> 低速 MoveL 到目标
  -> FR5 20004 连续状态确认停止
  -> 等待机械稳定
  -> 读取采图前法兰
  -> 复核两路 TTL LOW 并采集 ChArUco
  -> 读取采图后法兰并检查 ≤0.10 mm / 0.05°
  -> 下一个姿态
  -> 返回起始法兰
  -> 30 组训练求解 + 6 组独立留出校验
```

任一相机、GPIO、机器人、IK、奇异性、停止确认或静止性关键检查失败都会中止；运动中点击
“停止运动/中止”会提交 `StopMotion`，并等待新鲜 20004 状态包确认机械停止。软件停止不能
替代物理急停。

自动结果不会直接覆盖正式配置。程序先在 session 中保存新的内参和手眼候选，并显示：

- 训练集内参 RMS、视图覆盖和近/中/远距离覆盖；
- 训练集固定板一致性与手眼姿态多样性；
- 未参与任何求解的 6 个留出姿态的重投影 RMS、固定板平移 RMS 和旋转 RMS。

只有内参正式门槛、手眼正式门槛以及留出门槛（重投影 RMS `≤0.60 px`、平移 RMS
`≤1.50 mm`、旋转 RMS `≤0.50°`）全部通过，自动页才显示整体通过。随后先在“相机内参”
页批准正式内参，再在“FR5 手眼标定”页批准正式手眼。第二台相机也完成后会刷新双相机
相对外参文件。批准新内参时，程序会把依赖旧内参的正式激光平面和旧手眼文件标记为
`.stale_*`；自动流程已经生成与新内参一致的手眼候选，但每一路激光平面仍需在“激光平面”
页重新采集/批准后才能恢复三维扫描。这里的相机间外参由两份共同法兰手眼推导，只用于
坐标统一；传统双目的专用同步 `stereoCalibrate` 和深度建图由下一节的独立程序完成。

## 4B. 双目专用外参、深度图与环境栅格

双目深度必须先有专用极线外参。`hik_dual_camera_extrinsics.yaml` 适合把独立测量结果转换
到共同坐标系，但两次手眼误差会叠加，不能保证亚像素极线对齐。按以下顺序操作：

本机双目左右顺序按传感器头的真实安装位置固定：左相机是上方左侧的 160 万
`MV-CS016-10GM / DA8784601 / 192.168.7.45`（读取 `scanner_650` 相机标定记录），
右相机是上方右侧的 130 万 `MV-CS013-60GN / DB0403208 / 192.168.1.46`
（读取 `scanner_450` 相机标定记录）。`scanner_650`、`scanner_450` 是已有线扫配置名，
不代表双目图像的左右顺序。标定程序和 Mapper 都按上述相机 SN 强制校验，不能交换 IP。

1. 固定 ChArUco 板，确认两台相机能同时看到同一批板角点；先完成并批准两台正式内参和
   手眼标定。
2. 退出其他 MVS/FR5 客户端，运行 `./run_stereo_calibration.sh`，连接两台相机、FR5 和
   TTL。程序取得 FR5 独占只读位姿租约，并在两路 TTL LOW 的因果 ACK 后启动同步预览。
3. 每次让机械臂完全停稳后点击“采集当前双目板”。程序要求 200 ms 内的新鲜 20004 包、
   连续 15 包线/角速度停稳、成对帧回调偏差默认不超过 30 ms、两幅图各有至少 12 个公共
   角点，且
   相邻样本至少平移 15 mm 或旋转 4°。原始左右图保存到
   `data/calibration/stereo/session_*`。
4. 至少采集 15 组，覆盖画面中心/边缘、不同倾角以及至少 100 mm 深度跨度。停止预览后
   求解；程序固定两台内参执行 `stereoCalibrate`，逐视图剔除极线外点，并要求整体
   stereo RMS `≤0.60 px`、极线 RMS `≤0.50 px`。候选还必须实际通过 612×512 和
   1224×1024 两种深度校正/视差范围检查。
5. 人工批准后原子写入 `config/hik_stereo.yaml`。任何相机、镜头、ROI、支架或手眼关系
   改变后，这份文件都必须作废并重标。

随后运行 `./run_stereo_mapper.sh`：

```text
两台 Mono8 连续帧
  -> CLOCK_MONOTONIC_RAW 回调时间配对
  -> 曝光中点与 FR5 20004 位姿插值
  -> 双目去畸变/极线校正
  -> StereoSGBM + 左右一致性 + 深度范围过滤
  -> reprojectImageTo3D（左相机坐标，mm）
  -> T_base_flange * T_flange_left_camera
  -> base_link 稀疏 log-odds 射线体素更新
  -> occupied_voxels.ply + occupancy_grid.pgm/yaml
```

默认快速模式是 `612×512`、`450–3000 mm`、左右一致性检查、`25 mm` 地图体素和每隔
4 像素投一条射线。精细模式使用 `1224×1024`；视差搜索上限固定为 512，若当前基线、
焦距和最小深度需要更大范围，程序会拒绝启动而不是截断近距离深度。处理线程只有一个有界
在途帧，算法来不及时会显式计数“忙丢帧”，不会无限堆积相机缓冲。

当前两台相机使用软件连续采集，以单调时钟回调时间按最大偏差配对。双目标定仅允许 FR5
停稳后采集，因此标定界面默认允许 30 ms，并可在 1–100 ms 内调整；该数值不能照搬到运动
建图。Mapper 另提供可标定的相机→机器人时间偏置。停稳采集和静态环境可以直接使用；连续 J1 扫描若要求高速毫米级
融合，应增加同一硬件触发源并现场标定传输延迟。`HikStereoMapper` 本身只读取 FR5 实时
位姿，不发送 J1/MoveJ/MoveL，机械臂扫掠路径必须由经过碰撞验证的机器人程序或人工示教
执行。导出的二维 YAML 使用 `base_link`、米单位分辨率和选择的高度切片，可被常见占据栅格
读取器使用。

每次实时采集的原图会立即保存到本次 session，同时把帧号、时间戳、实际曝光/增益、
图像尺寸、相机身份、激光状态、两路 GPIO 回读、状态源单调时间和 daemon generation
追加到 `capture_manifest.csv`。激光配对还会显式记录 `pair_id`；同一对文件使用相同
ID，命名为 `laser_<时间戳>_off.png` 和 `laser_<时间戳>_on.png`。表格中删除样本或
点击“清空”只会移除当前计算数据，不会删除已保存的原图。

## 5. 第二步：标定相机内参

### 5.1 采集 30–40 张图像

1. 确认鲁班猫 TTL 状态栏已经连接。点击实时采样时，程序还会再次请求并确认两路
   LOW；未收到板端 ACK/回读时不会采图。离线导入不访问 GPIO。
2. 切换到“相机内参”页。
3. 使用“实时采样（激光关闭）”逐张采集；已有图片也可用“批量导入图片”。
4. 建议采集 30–40 张清晰图像，不要连续拍摄几乎相同的姿态。应覆盖：
   - 画面中心、四角和四条边；
   - `400–500`、`500–600`、`600–700 mm` 三个实际工作距离档；
   - 绕标定板 X/Y 轴的正负倾角，以及少量平面内旋转；
   - 大、中、小不同的画面占比。
5. 每张图都检查表格的 Marker、角点、覆盖率、边距、饱和率、清晰度和状态。模糊、反光、过曝、严重裁边或姿态重复的图片应删除并补拍。

单张图的当前自动检测门槛是：

- 至少 6 个 ArUco Marker；
- 至少 12 个 ChArUco 角点；
- 角点凸包至少覆盖图像面积的 2%；
- 角点距图像边缘至少 8 px；
- 灰度达到 250 的区域不超过标定板区域的 1%；
- 拉普拉斯清晰度方差至少为 30。

“检测通过”只是单张图最低门槛，仍需人工放大预览，剔除运动模糊、失焦、重复姿态和局部强反光图像。

### 5.2 求解和保存

1. 至少有 15 张图像通过检测后，“求解内参”才会启用。
2. 点击“求解内参”。求解器会计算每张图的重投影误差，并进行最多 3 轮 MAD 异常视图剔除；单视图 RMS 的硬上限为 0.8 px。
3. 查看求解结果和表格中的“标定采用/标定剔除”。如果姿态覆盖不足或误差偏大，先补拍数据再重新求解，不要仅靠删除所有高误差图像来压低 RMS。
4. 成功求解后，程序会自动在当前 session 中保存一份候选 YAML；“保存 session 候选 YAML”可以另外保存带时间戳的候选。
5. 如果要继续标定激光平面，切换到“激光平面”页，点击“沿用本页刚求出的内参”；也可以点击“加载内参 YAML”加载之前保存的文件。
6. 只有结果通过软件质量门槛后，“保存通过结果到当前 profile 正式内参”才会启用；实际路径见前面的 profile 映射表。

当前正式内参文件的软件提升门槛为：

- 至少 15 个最终采用视图；
- 相机矩阵和畸变参数均为有限数；
- 总体重投影 RMS 不大于 0.45 px；
- 采用视图 RMS 的 P95 不大于 0.60 px，最大值不大于 0.80 px；
- 所有采用视图的角点范围至少覆盖图像宽、高各 70%，标定板中心至少占到九宫格中的 5 格；
- 最大/最小角点凸包面积比至少为 1.40；
- 板法向的最大夹角差至少 12°，深度跨度至少为 30 mm 或平均深度的 6%（取较大值）。
- `400–500`、`500–600`、`600–700 mm` 各至少 3 个最终采用视图，并且至少
  有一组板深度不大于 425 mm、一组不小于 675 mm。

未通过上述门槛的结果仍可作为 session 候选保存，但不能覆盖正式配置。实际工程验收建议继续争取总体 RMS `< 0.3 px`，并确认去畸变后的直线无明显弯曲、近中远距离均无系统偏差。

## 6. 第三步：标定激光平面

### 6.1 加载内参

进入“激光平面”页后，先执行以下任一项：

- 点击“沿用本页刚求出的内参”；
- 点击“加载内参 YAML”，选择当前 profile 已批准的正式内参或 session 候选。

内参、图像分辨率、镜头状态和标定板定义必须与激光平面采集一致。当前内参或 session 候选可用于调试和生成激光候选；只有明确加载当前 profile 已批准的正式内参，激光平面才可能提升为正式配置。

### 6.2 严格采集 laser-off / laser-on 配对

每个标定板姿态都必须按下面顺序完成一组配对。推荐直接点“一键采当前姿态
（自动 off → on）”；它只自动控制 GPIO 和相机，不发送 FR5 运动指令。需要诊断时
也可使用下面两个分步按钮：

1. 把标定板放到目标距离和倾角，确保当前设备组对应的激光能够落在标定板有效区域内，然后等待结构完全静止。
2. 点击“手动分步 1：采 laser-off”。程序请求两路 LOW，核对板型、
   Pin 11/GPIO15、Pin 7/GPIO16、控制租约与逻辑回读，等待界面设置的稳定时间后采图。
3. 确认界面显示 `laser-off 已保存`。从这一刻起，严禁移动相机、标定板、支架或工作台，也不要改变曝光、增益、对焦、光圈、分辨率和 ROI。
4. 点击“手动分步 2：采 laser-on”。程序只开启当前 profile 对应通道，
   保持另一通道 LOW；ACK/回读与稳定延时通过后采图，图像返回后立即请求两路 LOW。
5. 查看该行的共同角点、像素位移、板平移/旋转、条纹点、3D 点和状态。被拒绝的配对应删除后整组重拍，不能把不同姿态的 off/on 强行组合。
6. 移动标定板到下一个姿态，重复以上步骤。

离线数据可用“离线导入成对图”，但文件也必须是真正同姿态、同曝光的 off/on 原图。界面会先让你选择一批 off 图，再选择一批 on 图，然后分别按文件名排序并逐项配对；两批数量必须相同，命名必须保证排序后一一对应。离线导入缺少可信的相机身份和曝光元数据，因此只允许保存候选，不允许覆盖正式配置。

当前每组配对的自动质量检查包括：

- off/on 两张图都通过 ChArUco 检测；
- `laser-off` 板区域饱和比例不超过 1%；`laser-on` 允许预期的窄激光线使该比例达到 5%，但后续条纹宽度和连续性门槛仍保持不变；
- `scanner_650` 的 Legacy 标定保持原完整半高宽质心定义；Quality 中心对窄且对称的
  饱和条纹使用左右半高交点中点，宽平顶或非对称饱和直接拒绝，不能以单侧下降沿
  作为中心；
- 至少 8 个共同角点；
- 角点位移均值不大于 0.35 px，P95 不大于 0.75 px；
- 两次估计的板姿态平移差不大于 0.5 mm，旋转差不大于 0.2°；
- 单次板姿态重投影 RMS 不大于 0.4 px，最大点误差不大于 0.8 px；
- 差分条纹至少提取 80 个有效点，条纹宽度不大于 20 px；
- 至少 60 条相机射线在标定板物理范围内形成有效 3D 交点。

### 6.3 采集姿态覆盖

当前项目按实际 `400–700 mm` 工作范围启用了专用采集引导。界面不是用标定板原点
`tvec.z` 粗略判断深度，而是用每组有效激光 3D 交点的**中位相机 Z**自动分档：

| 目标深度 | 自动分档范围 | 目标有效组数 |
| --- | --- | ---: |
| 425 mm（近端） | `[400, 500) mm` | 5 |
| 550 mm（中段） | `[500, 600) mm` | 5 |
| 675 mm（远端） | `[600, 700] mm` | 5 |

总目标为 15 个有效姿态，并要求实际样本覆盖到 `≤425 mm` 和 `≥675 mm`。
每完成一组通过质量检查的配对，GUI 会同步显示：

- 该组的中位 Z 和所属深度档，超出引导范围时明确标为“范围外”；
- 标定板中心的相机坐标 X/Y；
- 标定板法向相对相机 Z 轴的 X/Y 有符号偏角；
- 激光条纹像素位置的 U/V 中位数；
- 每档已采数量、X/Y 跨度、倾角范围、条纹位置跨度和下一项缺口；
- 下一组优先深度以及左/右、上/下、正/负倾角或改变激光落点的建议。

采完 `laser-off` 后，界面会先用板中心 Z 显示近似深度档；如果明显放错距离，可以取消
当前配对后重新摆板。完成 `laser-on` 后才会改用激光 3D 点中位 Z 给出最终档位。
预览图也会叠加 `Zmed`、深度档、板中心、法向偏角和条纹位置，便于拍完立即确认。
删除样本、清空数据集或离线导入后，引导表会从当前有效样本自动重算，不保留人工计数。
“范围外”的有效组只保留作诊断，不参与激光平面拟合，也不会扩大 YAML 有效 Z。
新批准文件的有效范围固定写为 `400–700 mm`。

为了防止“数量够了但姿态几乎相同”，每个深度档除达到目标组数外，还必须满足：

- 标定板中心 X 跨度至少 20 mm，Y 跨度至少 20 mm；
- 板法向 X、Y 偏角都覆盖小于等于 `-2°` 和大于等于 `+2°`；
- 各组激光条纹 U/V 中位位置的跨度，至少有一个方向达到对应图像尺寸的 6%。

最低 8 个有效姿态后仍允许提前求解并保存 session 候选，便于检查条纹和平面残差；
但三个深度档、两端覆盖及上述多样性未全部完成时，不能覆盖正式
当前 profile 的正式激光平面文件。

### 6.4 求解激光平面

1. 只有状态为通过的配对参与求解。
2. 界面 RANSAC 距离阈值默认是 0.3 mm。没有明确实验依据时先保持默认值，不要为了提高内点率盲目放宽。
3. 至少 8 个范围内、通过质量检查的不同姿态后，“求解激光平面”会启用；正式结果应完成上面的 15 组三档引导。
4. 点击“求解激光平面”。算法要求每个参与姿态至少 60 个 3D 点，每个姿态最多均衡抽取 250 点，执行 2000 次跨姿态 RANSAC，再以按姿态平衡的 SVD 精化。程序会从近/中/远各保留一整组姿态不参与训练，用训练结果计算真正的留出残差，然后再用全量范围内数据拟合最终平面。
5. 拟合的最小内点比例是 90%。检查结果中的姿态数、点数、内点比例、RMS、P95 和最大距离。
6. 成功求解后先保存 session 候选；只有达到正式质量门槛后，才能提升到当前 profile 的正式激光平面文件。

当前正式激光平面文件的软件提升门槛为：

- 有有效的平面求解结果；
- 必须加载当前 profile 已批准的正式内参，且其中至少采用 15 个视图、总 RMS 不大于 0.45 px；
- 所用内参文件的 SHA-256 与加载时一致，相机序列号与实时设备一致，采集清单存在且可校验；
- 至少 8 个有效板姿态；
- 完成 425、550、675 mm 三个深度档各 5 组采集、覆盖 `≤425 mm` 与
  `≥675 mm`，并满足每档的 X/Y、正负倾角和条纹位置多样性；
- 去除平移差小于 5 mm 且法向差小于 3°的近重复样本后，仍至少有 8 个姿态；
- 内点比例不低于 90%；
- 法向量长度与 1 的差不超过 `1e-6`；
- 平面系数、`d`、内点 RMS/P95 都是有限数；
- 平面内点距离 RMS 不大于 0.20 mm，P95 不大于 0.30 mm；
- 近/中/远各一整组的留出验证 RMS 不大于 0.25 mm，P95 不大于 0.40 mm；
- 每个姿态内点率至少 80%、RMS 不大于 0.30 mm、P95 不大于 0.50 mm；
- 板法向的最大夹角差至少 12°，深度跨度至少为 30 mm 或平均深度的 6%（取较大值）。

正式按钮可用仍不等于整机三维精度已验收。建议用 4–6 个未参与拟合的姿态以及已知高度标准件继续做独立验证。

激光平面采用相机光学坐标系：X 向右、Y 向下、Z 向前，长度单位为 mm：

```text
nx * X + ny * Y + nz * Z + d = 0
```

法向量会归一化为长度 1，并固定符号使 `d <= 0`。

## 7. 第四步：静态单帧三维轮廓

进入“静态三维轮廓”页。程序启动时会自动加载并交叉校验：

- 当前 profile 的正式内参；
- 当前 profile 的正式激光平面；
- 激光平面记录的内参 SHA-256 与当前正式内参是否一致；
- 正式内参绑定的相机序列号、图像尺寸和激光平面有效 Z 范围。

实时采集顺序：

1. 将标准平板放在标定有效深度内，机器人、相机和平板全部保持静止；
2. 使用激光平面标定时合适的曝光与增益；
3. 点击“1. 自动关光并采 off”，等待板端确认两路 LOW 后完成采图；
4. 不移动任何物体、不修改曝光/增益；
5. 点击“2. 保持不动，自动开光并采 on”，程序采图后自动请求关光；
6. 程序执行 on-off 差分、逐行连续峰值选择和灰度重心亚像素定位；
7. 像素经正式内参去畸变为相机射线，并与正式激光平面求交；
8. 有效点自动保存为相机光学坐标系下的 PLY 和 CSV。

条纹扫描方向由设备 profile 固定：水平条纹按“每个图像列一个中心”，竖直条纹按
“每个图像行一个中心”；`Auto` 只用于尚未固定方向的标定阶段。输出坐标为当前
profile 的相机 frame（见映射表），X 向右、Y 向下、Z 向前，单位 mm。超出激光
平面 YAML 所记录有效 Z 范围的点会被拒绝，而不是外推。

建议在有效范围的近、中、远位置分别采至少 3 组，检查静态重复性。表格中的指标含义不同：

- `直线 RMS`：单条三维轮廓对最佳拟合直线的距离，只能评价轮廓直线度；
- `平板 RMS`：只有 off 图中检测到 ChArUco 板、并且至少 60 个轮廓点落在板物理区域内时才提供；这是重建点到独立 ChArUco 板平面的距离，可按 `< 0.3 mm` 做首轮验收；
- 普通无纹理平板只有一条交线，不能从单帧唯一拟合三维平面，因此程序不会把直线 RMS 冒充为平板 RMS。

也可以点击“离线导入 off/on”复算已有原图；离线导入无法验证曝光/增益元数据，必须由操作者确认两张图确实同姿态、同参数。

## 8. 第五步：FR5 eye-in-hand 手眼标定

### 8.1 现场布置与连接

1. 将 ChArUco 板牢固固定在机器人基座附近的工作台上；从第一张到最后一张都不能移动板。
2. 确认两路线激光均关闭，保持当前 profile 正式内参标定后的分辨率、ROI、镜头焦距、对焦和光圈不变。
3. 停止 `ros2_fr5` 的 FR5 状态发布节点和其他 Fairino SDK 客户端。
4. 连接海康相机，然后进入“FR5 手眼标定”页，以默认 `192.168.1.200` 连接 FR5。
5. 可先点“读取当前法兰”，确认显示的是控制器实际 `T_base_flange`，单位为 mm/deg。

本节的手动手眼页面只读取机器人，不发送运动；每次都由操作员以安全方式手动换姿态，
机器人完全停止后再点击“采集一个手眼样本”。需要程序带动相机时只能使用前述
“自动相机/手眼”页，并遵守其 dry-run、路径预检与安全确认流程。

### 8.2 采集 15–25 个姿态

每次点击采样，程序自动执行：

```text
读取 T_base_flange_before
  -> 软件触发海康单帧并立即保存
  -> 检测 ChArUco 并求 T_camera_board
  -> 读取 T_base_flange_after
  -> 检查机器人静止
  -> 保存一组手眼样本
```

前后法兰变化超过 `0.10 mm` 或 `0.05°` 时，样本会被拒绝。板位姿要求至少看到全部 ChArUco 内角点的 75%（且不少于 12 个；例如 11×8 方格板为至少 53/70 个）、重投影 RMS 不大于 `0.40 px`、最大点误差不大于 `0.80 px`。这样可阻止只看到局部标定板、但像素重投影误差看似很小的样本进入手眼求解。原图即使被拒绝也会保留，避免无证据地丢失现场数据。

标定板中心相机 Z 必须位于 `400–700 mm`；范围外样本会立即拒绝。正式手眼结果还要求
`400–500`、`500–600`、`600–700 mm` 各至少 3 个最终采用样本，并覆盖
`≤425 mm` 和 `≥675 mm`。

姿态应同时变化位置和方向，特别要包含绕至少两个旋转轴的正负倾斜。软件求解最低要求为：

- 至少 15 个有效样本；
- 法兰位置最大跨度至少 50 mm；
- 法兰姿态最大跨度至少 20°；
- 第二旋转轴离散度至少 5°，避免所有姿态只绕一个轴变化。

不要只沿一条直线平移，也不要让所有图片的标定板几乎正对相机。建议采集 20–25 个姿态，为自动离群剔除留余量。

### 8.3 求解、验证和保存

点击“鲁棒求解 `T_flange_camera`”。程序同时计算 OpenCV 的 TSAI、PARK、HORAUD、ANDREFF 和 DANIILIDIS 五种方法，用固定板在 base 坐标系下的一致性选择最佳结果，并最多进行 3 轮鲁棒离群剔除。

求解后界面会列出固定板平移残差最高的 3 个样本，并在表格中用橙/红色标出较大的单样本残差。该提示只用于逐张检查原图、板是否松动及机器人姿态；程序不会自动删除这些样本。应每次只处理一个确有问题的样本、重采后重新求解，不能通过批量删样本人为压低 RMS。

使用的关系和输出方向是：

```text
T_base_board = T_base_flange · T_flange_camera · T_camera_board
```

`T_flange_camera` 将当前 profile 相机 frame 中的点变换到 FR5 法兰坐标系，平移单位为 mm。正式写入当前 profile 手眼文件的门槛为：

- 最终采用样本不少于 15 个；
- 固定板 base 坐标平移一致性 RMS 不大于 1.0 mm；
- 固定板 base 坐标旋转一致性 RMS 不大于 0.30°；
- 当前相机序列号与正式内参一致；
- 正式内参的 SHA-256 在采集期间未变化。

候选结果可先保存在 session；质量通过后才可原子更新正式文件。正式扫描前还应额外采集 4–6 个不参加求解的验证姿态，确认 `T_base_board` 没有随姿态产生固定方向漂移，并在 RViz 中检查相机光学轴方向与实际安装一致。

## 9. 第六步：常亮单帧停稳扫描验证

### 9.1 先做完全不运动的单点验证

1. 退出标定 GUI、MVS、ROS 2 法兰发布节点和其他 Fairino SDK 客户端。
2. 将工件放在当前 profile 激光平面的有效相机 Z 范围内；只以该 YAML 的
   `validity.camera_z_min_mm` / `camera_z_max_mm` 为准，不跨 profile 借用范围。
3. 运行 `./run_constant_laser_scan.sh`，连接鲁班猫 TTL、当前 profile 相机和 FR5；点击“开启本组 TTL”，等待板端 ACK、租约和两路 GPIO 逻辑回读均通过。
4. 点击“单点常亮验证（不移动）”。它只读取采图前后法兰位姿，不发送运动。
5. 放大检查预览中的绿色中心线必须只覆盖真实激光条纹。确认点数、深度范围和条纹饱和比例合理后，再考虑机器人运动。

常亮模式没有 laser-off 图。程序使用 `31×3` 横向形态学背景核去除缓慢变化的表面亮度，再按每行提取一个窄亮脊的亚像素中心。因此激光线应大致沿图像竖直方向。强镜面反射、第二条亮边、焊缝高光或激光断裂仍可能产生错误中心，原始 PNG 和绿色叠加预览必须人工复核。初次验证优先调整曝光/光圈，使激光中心明显但不形成大面积饱和。

单点验证输出的点已按以下坐标链变换到 `base_link`：

```text
P_base = T_base_flange · T_flange_camera · P_camera
```

程序会硬性校验三份正式标定的内参 SHA-256、相机序列号和坐标系关系。正式文件在加载后发生变化、实时相机 SN 不一致、轮廓少于 80 点、点超出标定深度，或采图前后法兰变化超过 `0.10 mm / 0.05°` 时，本次扫描会停止。

`直线 RMS` 描述的是当前三维轮廓对最佳拟合直线的偏离，普通工件本来就可能有弧面、台阶或坡度，所以普通扫描模式只记录并警告，不会因此中止。只有扫描标准平板并明确勾选“标准平板模式：直线 RMS 超限时停止”时，才按界面门槛（默认 `0.50 mm`）硬拒绝。条纹中心灰度达到 250–255 的比例会显示并写入清单；比例超过约 30% 时应尝试降低曝光或激光功率，以减少饱和平台对亚像素中心的影响。

### 9.2 dry-run 和首次短路径

1. 先手动将机器人移动到安全起点，点“读取并设为起点”。
2. 再手动移动到安全终点，点“读取并设为终点”。终点只提供 XYZ；整条扫描的 RPY 固定为起点 RPY。
   示教后可使用“编辑起点数值”和“编辑终点数值”二次修改 XYZ/RPY。起点 RPY 决定整条扫描姿态；终点 RPY 只记录、不参与插值。一次扫描结束后，可点击“交换起点/终点（反向扫描）”将两端 XYZ 互换，直接从旧终点沿反方向重复扫描；为了避免换向时意外转腕，交换前的起点 RPY 会继续用于新起点和整条反向路径。交换不会立即下发运动，但会清除旧目标/FR5 评估并取消安全确认，必须重新核对路径和执行 dry-run。
3. 首次路径建议 `20–50 mm`，步距先用 `5 mm`。界面默认开启 `100 mm` 的“验证路径长度保护”和 `201` 个的“目标数量保护”；需要更长、更密的路径时可提高对应保护值，或明确取消保护。
4. 保持 `dry-run` 勾选，点击“生成并打印目标列表”或“开始停稳扫描”，逐项核对所有 base 法兰目标。
5. 在控制器侧完成使能和自动模式确认，核对 `tool=0、user=0` 表示目标是 base 下的法兰位姿，而不是焊枪 TCP 或其他工具坐标。
6. 确认扫描头、工件、夹具、线缆和奇异位形均安全，人员守在物理急停旁；再取消 dry-run、勾选安全确认并接受最终确认框。
7. 首次真运动使用默认 `5%` 速度、`20%` 加速度、`250 ms` 停稳，确认流程稳定后再把步距改为 `2 mm`。

例如路径为 `182.703 mm`、步距 `0.5 mm` 时，会生成 `ceil(182.703 / 0.5) + 1 = 367` 个目标。可把“最大验证路径”设为 `200 mm`、把“最大目标数量”设为 `400`，无需关闭保护，也不会丢点。367 次停稳、取图和写盘耗时较长，必须先用较大步距验证完整路径，再生成并核对 0.5 mm 的 dry-run。

每个点严格执行：非阻塞 `MoveL`、轮询到位、停稳等待、读取法兰 before、软件触发一帧、读取法兰 after、静止校验、三角重建和 base 坐标累计。扫描工具不会做碰撞规划，也不会自动从当前位置规划到起点；第一次到起点同样是一段直线 `MoveL`。界面“停止”会终止采集状态机，并在活动运动期间通过同一个 SDK 会话发送 `StopMotion`，但它不能替代控制柜物理急停。`StopMotion` 返回 0 后还会等待新鲜 20004 状态连续确认 `motion_done=1` 且 TCP 线/角速度接近零；扫描完成或中止后，只有相机空闲、FR5 停止和两路 TTL LOW 三项都确认，另一组和下一次扫描才会解锁。

### 9.3 输出与首轮判断

每次单点或扫描会话保存在：

```text
data/scans/<profile>/scan_YYYYMMDD_HHMMSS_mmm/
├── images/profile_*.png
├── session_metadata.json
├── session_result.json
├── scan_manifest.csv
├── scan_raw.ply
└── scan_voxel.ply
```

`scan_manifest.csv` 记录 profile、波长、TTL Pin、板端 generation、相机身份、图像、帧号、设备/主机时间戳、实际曝光和增益、采图前后完整法兰 XYZ/RPY、静止差、直线 RMS、是否启用平板门槛、条纹饱和比例及三份标定 SHA-256。`session_metadata.json` 固化会话设备与标定身份，`session_result.json` 原子记录完整/中止、原因、结束时间、最终计数，以及 TTL LOW、FR5 停止和相机空闲的终态互锁结果。`scan_raw.ply` 是未降采样 base 点云，`scan_voxel.ply` 默认按 `0.5 mm` 体素平均；PLY 单位为 mm，并保留置信度、响应、轮廓序号和原像素坐标。

第一轮不要使用 ICP。扫描同一标准平板时，各轮廓应凭手眼与机器人位姿自然重合。建议依次验证：单点、`20–50 mm / 5 mm` 步距、`2 mm` 步距、同路径重复、反向扫描。若条纹间出现随运动方向变化的错层，优先检查手眼、法兰坐标定义、结构松动和采图前后位姿，不要靠点云配准掩盖。

## 10. 候选结果和正式配置的提升规则

- “session 候选 YAML”是可追溯的中间结果。成功求解后即允许保存，即使它尚未达到正式工程验收目标，也不会覆盖 `config/` 中正在使用的配置。
- “保存通过结果到 config/...”只在当前结果通过代码中的硬质量门槛时启用。
- 点击正式保存后，程序先在当前 session 写一份带时间戳的 `approved` YAML，再以原子写入方式更新 `config/` 中的正式文件。
- 批准新内参前，已有正式激光平面会先改名为 `.stale_时间戳`，防止新内参与旧激光平面被混用。
- 质量不通过、写入失败或提升失败时，现有正式配置保持不变。
- 正式按钮可用不等于整套系统已经验收。必须结合独立验证姿态、实际工作距离和最终工件精度要求决定是否投入扫描。

## 11. 输出目录和 YAML

每次启动 GUI 都会建立独立会话：

```text
myline_hik/
├── data/calibration/
│   └── <profile>/session_YYYYMMDD_HHMMSS_mmm/
│       ├── capture_manifest.csv
│       ├── automatic/
│       │   ├── automatic_*_plan.csv
│       │   ├── automatic_*_samples.csv
│       │   ├── automatic_*_summary.json
│       │   └── auto_*.png
│       ├── intrinsics/
│       │   ├── intrinsic_*.png
│       │   ├── hik_intrinsics_candidate_*.yaml
│       │   └── hik_intrinsics_approved_*.yaml
│       ├── laser/
│       │   ├── laser_*_off.png
│       │   ├── laser_*_on.png
│       │   ├── hik_laser_plane_candidate_*.yaml
│       │   └── hik_laser_plane_approved_*.yaml
│       ├── profiles/
│           ├── profile_manifest.csv
│           ├── profile_*_off.png
│           ├── profile_*_on.png
│           ├── profile_*.ply
│           └── profile_*.csv
│       └── handeye/
│           ├── handeye_manifest.csv
│           ├── handeye_*.png
│           ├── hik_handeye_candidate_*.yaml
│           └── hik_handeye_approved_*.yaml
├── data/scans/
│   └── <profile>/scan_YYYYMMDD_HHMMSS_mmm/
│       ├── images/profile_*.png
│       ├── session_metadata.json
│       ├── session_result.json
│       ├── scan_manifest.csv
│       ├── scan_raw.ply
│       └── scan_voxel.ply
└── config/
    ├── hik_dual_camera_extrinsics.yaml     # 两台相机共同法兰推导的相对外参
    ├── hik_stereo.yaml                     # 专用双目标定结果
    └── devices/
        ├── scanner_450/                    # scanner_450 标定 + 同步配置
        └── scanner_650/                    # scanner_650 标定 + 同步 + 自适应配置
```

导入的原图也会原样复制到当前 session。`data/calibration/` 默认不提交到 Git。

`hik_intrinsics.yaml` 使用普通 YAML，主要记录图像尺寸、相机矩阵、`plumb_bob` 畸变系数、标定板定义、采用/剔除视图数和重投影 RMS。

`hik_laser_plane.yaml` 使用普通 YAML，主要记录归一化平面系数 `[nx, ny, nz, d]`、相机坐标约定、内参来源及 SHA-256、采集清单及 SHA-256、标定板定义、拟合姿态/点/内点统计、RMS/P95 以及有效相机 Z 范围。文件不使用 OpenCV 的 `!!opencv-matrix` 私有标签。

`hik_handeye.yaml` 明确记录 `mode: camera_to_flange`、行优先 `T_flange_camera`、mm 单位、所选 OpenCV 方法、采用/剔除样本、固定板平移/旋转一致性、内参与清单 SHA-256，以及每个样本的残差。

## 12. 验收门槛

建议按下面顺序验收，不要只看“求解成功”：

| 项目 | 软件最低门槛 | 建议工程验收 |
| --- | --- | --- |
| 内参采集 | 15 张实时采集且通过检测；离线导入仅能生成候选 | 采集 30–40 张并覆盖完整视场、距离和倾角 |
| 内参正式提升 | 总 RMS ≤ 0.45 px、视图 P95 ≤ 0.60 px、最大 ≤ 0.80 px，并满足覆盖/尺度/姿态多样性 | 总 RMS < 0.3 px，去畸变无系统弯曲 |
| 激光配对 | 静止、同相机、同曝光/增益、条纹、3D 检查全部通过；离线导入仅能生成候选 | 每组预览无错线，off/on 确认同姿态同曝光 |
| 激光训练姿态 | 至少 8 个通过姿态 | 12–16 个训练姿态，覆盖实际深度和倾角 |
| 平面内点 | 比例 ≥ 90% | 不应靠放宽 0.3 mm RANSAC 阈值通过 |
| 激光正式提升 | 内参合格、≥8 姿态、单位法向量、RMS ≤ 0.20 mm、P95 ≤ 0.30 mm，并满足姿态多样性 | 再执行下一行独立验证 |
| 独立验证 | 当前工具不自动划分验证集 | 4–6 个未参与拟合姿态无近远端系统偏差 |
| 静态轮廓 | 至少 80 个有效点，且所有点位于标定 Z 范围 | 近/中/远各 ≥3 组；ChArUco 平板 RMS < 0.3 mm，重复轮廓稳定重合 |
| 手眼采集 | ≥15 样本、位置跨度 ≥50 mm、姿态跨度 ≥20°、第二旋转轴离散度 ≥5°，单样本前后法兰变化 ≤0.10 mm/0.05° | 20–25 个姿态，绕至少两个轴充分变化 |
| 手眼正式提升 | 固定板平移 RMS ≤1.0 mm、旋转 RMS ≤0.30°，相机 SN/内参哈希一致 | 4–6 个独立姿态验证，并在 RViz 核对安装方向 |
| 常亮停稳扫描 | 每条轮廓 ≥80 点、Z 在有效范围、采图前后法兰变化 ≤0.10 mm/0.05°；平板模式另要求直线 RMS ≤设定门槛 | 尽量避免条纹饱和；标准平板各轮廓自然重合；重复和正反向扫描无系统错层 |

最终还应使用标准平板或已知高度件验证三维重建。相机内参和激光平面通过，并不代表手眼标定、机器人同步和运动扫描已经完成。

## 13. 常见错误

### 相机不能连接或提示被占用

完全退出 MVS，并确认没有其他程序打开相机。只在 MVS 中停止取流但仍保持设备打开，也可能阻止独占访问。

### 找不到配置中的相机 IP

检查相机供电、网线、主机网卡地址和子网掩码；确认 GUI 中输入的是相机当前 IP。日志会列出实际枚举到的 GigE 相机。

### 提示未编译 MVS 支持

确认 `/opt/MVS/include/MvCameraControl.h` 和 `/opt/MVS/lib/64/libMvCameraControl.so` 存在，然后重新运行 CMake。缺少 SDK 时只能离线导入。

### CMake/链接使用了 Anaconda Qt

退出 Conda，清理旧构建缓存后重新生成，或按第 2 节指定系统 `Qt5_DIR`。若报错路径指向用户的 `anaconda3/lib/libQt5...`，应优先排查这个问题。

### ChArUco 检测一直失败

依次检查 OpenCV 11×8 定义（设计图 8 行×11 列）、24/18 mm、`DICT_4X4_50`、legacy 图案、打印缩放、对焦、曝光、反光和裁边。内参图必须关闭激光。

### 板参数不能修改

先清空内参样本、激光样本，并取消尚未完成的 `laser-off` 配对。原图仍保留在 session。

### “求解内参”按钮不可用

当前通过自动检测质量的图像少于 15 张。查看表格最后一列的拒绝原因，补拍或导入合格图像。

### off/on 配对被判定移动

整组重拍。两帧之间只能开激光，不能碰标定板、相机或支架，也不能修改曝光、增益或镜头参数。

### 条纹点或 3D 点不足

确认激光线确实落在标定板物理区域内；避免条纹太暗、过曝、过宽或断裂；保持 off/on 曝光一致，并确保两张图都能稳定识别标定板。

### 正式保存按钮不可用

当前结果只完成了求解，但没有通过正式质量门槛。先查看结果标签和日志中的原因；可以保留 session 候选用于排查，但不要手工复制候选覆盖正式配置。

### 静态轮廓只有直线 RMS，没有平板 RMS

这是正常的：一条三维交线不能唯一确定一个平面。需要让 laser-off 图看见与正式配置一致的 ChArUco 板，且激光线落在板物理区域内，程序才能利用板位姿计算独立平板残差。

### 静态轮廓提示点全部超出深度范围

当前平板不在当前 profile 激光平面 YAML 的 `validity.camera_z_min_mm` 到 `camera_z_max_mm` 范围内。应移动平板进入标定覆盖范围；不要手工放宽范围来掩盖外推。

### FR5 连接报 `RPC err=-2` 或提示被占用

Fairino SDK 同时只能由一个客户端持有连接。先停止 `ros2_fr5` 的
`fairino_state_publisher`、其他机器人 GUI 和测试程序，再重启本工具。标定程序和扫描
程序内部都只创建一个进程级 FR5 会话；`scanner_450`、`scanner_650` 两个页面共享该
连接，空闲时切页无需断开或重连。手眼采样、真实扫描及其安全收尾期间由发起页面持有
独占命令租约，另一页只能查看共享连接状态。受 Fairino 3.9.4 后台线程生命周期限制，
主动断开或连接失败后，本进程仍不能再次建立 RPC；此时才需要重启程序。

### 手眼样本提示机器人未静止

采图前后实际法兰变化超过 `0.10 mm / 0.05°`。等待机械臂完全停止，避免拖动示教、外力振动或在采集过程中碰支架后重拍；不要通过放宽阈值掩盖图像与位姿错配。

### 手眼求解提示姿态多样性不足

样本数量够但位姿跨度不足。增加绕 X/Y 等至少两个轴的正负倾斜，并同时改变距离和画面位置；仅平移或所有姿态近似平行无法可靠约束 eye-in-hand 旋转。

## 14. 当前边界与下一步

本工具当前完成两组独立的相机内参、相机坐标系下激光平面、静态 off/on 单帧三维轮廓、
FR5 eye-in-hand 手眼标定、自动停稳采样与留出校验、共同法兰推导的双相机相对外参、
受限免密 TTL 控制，以及独立的常亮线激光停稳/连续直线扫描；仍不包含：

- 激光功率调节或光学出光反馈（当前只有 TTL GPIO 逻辑控制与回读）；
- FR5 自动使能、控制器模式切换、碰撞规划、物理急停或安全等级激光联锁；
- 相机触发线与机器人控制器之间的硬件时间同步（当前已实现软件时间同步）；
- 任意曲面轨迹规划、ROS 2 点云发布和 RViz 实时显示。

完成低速短路径、重复和正反向扫描验证后，下一阶段才是 ROS 2/RViz 接入。当前连续模式使用设备时间戳映射、控制器 `robotTime` 映射和位姿插值，精度仍受相机/网络固定延迟影响；需要更严格同步时应增加硬件触发。

## 15. 60 fps 连续扫描的软件时间同步

`HikConstantLaserScan` 在保留原“停稳扫描”的同时，增加了“开始 60fps 连续同步扫描”。它使用以下数据流：

```text
MVS 自由运行回调 → 有界相机队列 → 曝光中点时间
                     └→ 2 个独立 PNG 写线程
FR5 SDK 20004 RecvPkg → CLOCK_MONOTONIC_RAW逐包时间戳 → SPSC队列
                     → robotTime 映射 → 5 秒位姿环形缓冲
曝光中点 → 二分查找前后状态 → XYZ 线性插值 + quaternion SLERP
          → synchronization.csv + SynchronizedFrame
          → 非阻塞固定容量队列 → 2 个独立三维重建线程
          → 激光中心/相机点 → T_base_camera → base_link 连续点云
独立元数据 WriterThread → robot_raw.csv、camera_raw.csv、session_summary.json
```

同步配置按 profile 隔离，分别位于
`config/devices/scanner_650/synchronization.yaml` 和
`config/devices/scanner_450/synchronization.yaml`。450 的初始配置目标为 60 fps、
曝光 1825 μs、默认速度 10 mm/s；文件存在只开放连续同步功能，不代表真机持续帧率
和写盘吞吐已经验收，首次使用必须执行低速短直线测试。电脑侧同步时间全部来自
`CLOCK_MONOTONIC_RAW`；UTC 日期只用于会话摘要和目录名。两组当前配置的 FR5 CNDE
请求周期保持 10 ms；当前控制器经逐包实测的正常反馈周期是约 12 ms（83.3 Hz），由独立的
`robot.expected_feedback_period_ms` 表达。环形缓冲为 2048 条且至少覆盖 5 秒。同步
会话在机器人到达扫描起点后才创建，避免把移到起点的长时间运动混入连续扫描统计。

连续重建默认使用 2 个独立工作线程和 64 帧固定容量队列。`SynchronizedFrame` 回调只使用 `try_lock` 做一次无等待入队，不执行 OpenCV、坐标变换或文件写入；队列锁忙或队列已满时只丢弃该帧的三维重建任务并累计诊断，不会反压相机回调、FR5 20004接收、同步CSV或原图写盘。扫描停止且同步队列完全清空后，主线程才等待后台重建收尾并保存PLY。

### 15.1 SDK 字段和当前可确认边界

海康路径实际使用 `MV_CC_RegisterImageCallBackEx`、`MV_CC_StartGrabbing`、`MV_FRAME_OUT_INFO_EX`、`MV_CC_GetIntValueEx` 和 `MV_CC_StopGrabbing`。每帧读取：

- `nFrameNum`；
- `nDevTimeStampHigh/nDevTimeStampLow`；
- `nWidth/nHeight`（或扩展宽高）、`enPixelType`；
- 帧内 `fExposureTime`；
- `nLostPacket`。

设备时间戳频率通过相机实际 GenICam 节点 `GevTimestampTickFrequency` 查询。节点可读时，将 raw ticks 换算成 ns 并建立滑动仿射映射；节点不可读时明确记录 `HOST_CALLBACK_FALLBACK`。当前 MVS SDK 头文件没有定义 `nDevTimeStamp` 对应曝光开始、曝光结束还是其他相机事件，因此由 `camera.timestamp_reference` 显式配置，默认 `exposure_start`，需结合具体相机手册或实验确认。

FR5 路径实际使用：

- `SetRobotRealtimeStateSamplePeriod(10)` / `GetRobotRealtimeStateSamplePeriod`；
- 补丁新增的 `SetRobotRealtimeStateCallback(...)` 逐包回调；
- `GetRobotRealTimeState(ROBOT_STATE_PKG*)` 一致快照及调用耗时诊断；
- 状态中的 `frame_cnt`、`jt_cur_pos`、`flange_cur_pos`、`actual_TCP_Speed`、`actual_TCP_CmpSpeed` 和 `robotTime`。

原始SDK 3.9.4 没有公开20004逐包回调或包到达时间。本工程的受版本控制补丁在
`RobotStateRoutineThread()` 中紧接 `RecvPkg()` 成功返回处记录
`CLOCK_MONOTONIC_RAW`，再通过非阻塞SPSC队列把每个完整包交给应用线程。
SDK逐包序号是判断回调/SPSC链路是否缺包的依据；`GetRobotRealTimeState()` 只作为
互斥一致快照和阻塞耗时诊断，不再承担新包检测。

`robot.time_mode` 默认 `controller_timestamp`：将控制器 `robotTime` 仿射映射到主机单调时钟，拟合稳定前自动使用 `HOST_RECEIVE`。`frame_cnt` 只作为循环计数诊断字段，它的跳变不直接判定同步无效。前后位姿样本的 SDK 逐包序号必须连续，否则标为 `ROBOT_PACKET_SEQUENCE_GAP`；序号连续时再按时间跨度判断，超过 25 ms 才标为 `ROBOT_GAP_TOO_LARGE`。默认正常接收间隔按 10–14 ms 识别，主机接收间隔超过 18 ms 才告警。因此控制器偶发 `robotTime` 跨 21 ms、但SDK序号连续的样本仍可在恒速段合理插值。

FAIRINO 3.9.4 的 `robot_types.h` 明确法兰 RPY 为绕固定 X/Y/Z 轴、单位 deg。工程沿用已有手眼标定约定：`R = Rz(rz) * Ry(ry) * Rx(rx)`，先转四元数再做 SLERP，不线性插值欧拉角。

### 15.2 启动

正常启动或用命令行覆盖同步质量判定的目标扫描速度：

```bash
./run_constant_laser_scan.sh
./run_constant_laser_scan.sh --scan-speed 10
./run_constant_laser_scan.sh --scan-speed 20
./run_constant_laser_scan.sh --scan-speed 30
./run_constant_laser_scan.sh --scan-speed 40
./run_constant_laser_scan.sh --scan-speed 50
```

超出 `10–50 mm/s` 会在创建窗口前拒绝启动。连续起点→终点段使用 FAIRINO 3.9.4 `MoveL(..., ovl=speed_mm_s, ..., oacc=acceleration_mm_s2, velAccParamMode=1)`，按物理速度和物理加速度下发；`scan.acceleration_mm_s2` 默认 100。界面中的“速度 %”只用于移到连续扫描起点和原停稳扫描。反馈的 `actual_TCP_CmpSpeed[0]` 仍独立参与质量检查，不能仅凭命令值认定匀速。速度判定使用默认 15 点窗口：对反馈速度取中值，同时对三维法兰位置随控制器时间作线性回归，再通过带迟滞的状态机标记 `WARMUP/STOPPED/ACCELERATING/CONSTANT_SPEED/DECELERATING/UNSTABLE`。默认只把 `CONSTANT_SPEED` 作为有效建图段。连续扫描流程为：示教起终点、完成 dry-run、取消 dry-run、勾选安全确认，然后点击“开始 60fps 连续同步扫描”。

启动时日志会打印理论线间距 `speed / fps` 和曝光运动量 `speed * exposure_us / 1e6`；它们只用于检查参数，不替代插值得到的实际法兰位姿。

### 15.3 输出和质量检查

默认输出到 `data/scan/<profile>/sync_scan_<时间>/`：

- `session_metadata.json`：profile、相机、TTL、正式标定路径与 SHA-256；
- `session_result.json`：完成/中止、原因、最终计数，以及 TTL、FR5、相机终态互锁；
- `robot_raw.csv`：SDK逐包序号、原始及展开后的 `frame_cnt`、`RecvPkg`成功返回时刻、getter耗时/结果、控制器时刻、法兰、关节、原始/滤波/位置拟合速度和运动阶段；
- `camera_raw.csv`：FrameID、设备 raw/ns 时间戳、回调时刻、曝光、尺寸、连续性、图片名和写盘入队状态；
- `synchronization.csv`：曝光中点、前后状态、alpha、间隔、插值四元数、原始/滤波速度、运动阶段、前后SDK逐包序号和质量；
- `session_summary.json`：设备帧率、软件接受帧率、20004完整包频率、SDK逐包序号/frame_cnt诊断、getter平均/P99/最大耗时、队列溢出、有效帧和相机/机器人时钟拟合残差；
- `images/frame_<FrameID>.png`：默认由 2 个独立后台线程以 PNG 压缩级别 1 保存的 Mono8 原图。
- `continuous_raw.ply`：每个有效同步帧使用曝光中点 `T_base_camera` 变换后累计的 `base_link` 原始点云；界面默认使用 CloudCompare 兼容的 `binary_little_endian` PLY，也可取消勾选改回 ASCII；
- `continuous_voxel.ply`：按界面体素尺寸降采样后的连续点云；RGB 按 `base_link` 世界 Z 高度使用 Turbo 色带自动生成（低处紫/蓝、高处黄/红），并采用 1%–99% 分位范围抑制离群点，CloudCompare 可直接显示；默认体素仍为 `0.5 mm`，需要更快、更小的显示点云时可明确改为 `1.0 mm`；
- `continuous_reconstruction.csv`：每个已入队帧的重建结果、点数、线RMS、处理耗时和失败原因；
- `continuous_reconstruction_summary.json`：重建线程/队列配置、同步无效跳过、非阻塞队列丢弃、成功/失败帧、点数、耗时和三份标定SHA-256。

`scanner_450` 和 `scanner_650` 当前都采用连续扫描精简输出：PLY 只保存
`continuous_raw.ply` 和 `continuous_voxel.ply`，不再写
optical/filtered/rejected/quality_voxel 四份附加质量 PLY。连续重建默认使用 4 个
工作线程和 256 帧非阻塞队列。扫描结束后，清空重建队列、体素化和 PLY 写盘全部
在独立后台线程完成；只有 `scanner_650` 明确选择“自适应质量建图”时才会在其中
执行 V 槽时序验证。执行区显示 0–100% 阶段进度，GUI 可继续滚动和
查看日志，但所有新运动保持锁定。收尾期间关闭窗口时，程序会先写完完整 PLY 再
自动关闭。

450 的正式 Quality 安全门控仍参与 raw 生成，但不会收集/提升 V 槽候选，也不再
收集附加 rejected 点云或执行邻帧支持分类。650 页面的“正式中心线策略”对单点、
停稳和连续扫描统一生效：

- `Legacy｜连续优先` 保留全部 Legacy 正式点，不执行多路径区硬遮罩；
- `Shadow｜反射安全` 保持 Legacy 坐标定义，但屏蔽质量算法判定的多路径歧义区；
- 选择“自适应质量建图”时，因为局部 NBS 依赖多路径候选证据，界面会自动切换并
  锁定为 `Shadow`；回到普通快速建图后可再次手动选择；
- 实际选择会写入 `session_metadata.json` 的
  `device_profile.centerline_policy` 和
  `continuous_reconstruction.formal_centerline_policy`。

650 页面还提供“连续建图模式”：

- `普通快速建图（仅 raw / voxel）` 为默认值，跳过质量候选、V 槽时序验证、相邻
  profile 支持和 rejected 收集，结果不能直接用于局部 NBS；
- `自适应质量建图（保留局部 NBS 证据）` 才在内存保留
  quality accepted/rejected 和真实视点，不写附加 PLY；固定全局蛇形粗扫会自动选择
  此模式。质量地图、ROI 和候选延迟到点击局部 greedy/beam 时按需生成。

查看有效同步记录：

```bash
column -s, -t < data/scan/scanner_650/sync_scan_*/synchronization.csv | less -S
awk -F, 'NR==1 || $20 != "VALID"' data/scan/scanner_650/sync_scan_*/synchronization.csv
```

`actual_camera_device_fps` 由设备时间戳和 FrameID 跨度计算，`accepted_camera_callback_fps` 表示软件实际接受吞吐。`camera_raw.csv` 的 `frame_id_continuous=0`，或摘要中的 `camera_frame_id_skips/camera_queue_overflows/image_pool_exhaustions/image_queue_overflows` 非零，表示相机链路存在缺图。`robot_receive_sequence_gaps` 才表示SDK回调到SPSC消费链路确实缺少完整包；`robot_frame_counter_skips/robot_frame_counter_duplicates/robot_frame_counter_out_of_order` 仅为控制器内部计数诊断，不参与逐包去重。机器人同步有效性先检查 `ROBOT_PACKET_SEQUENCE_GAP`，再检查25 ms位姿跨度门槛。83.3 Hz与配置的12 ms预期相符时只打印正常诊断，不再按CNDE请求的100 Hz持续告警。加减速段仍保存，但默认标为 `SPEED_NOT_STABLE`，不计入有效建图帧。连续点云只处理 `VALID` 帧；若 `queue_full_drops` 或 `queue_contention_drops` 非零，说明三维重建算力不足，但不表示采图或机器人状态丢失。

### 15.4 固定时间偏差标定

最终图像时刻为：

```text
aligned_time = exposure_mid_time + camera.fixed_time_offset_us * 1000
```

用同一台阶做正、反向扫描后，可按下式估计固定延迟：

```text
delta_t_s = (x_forward_mm - x_reverse_mm) / (2 * speed_mm_s)
fixed_time_offset_us = delta_t_s * 1e6
```

辅助工具：

```bash
./tools/estimate_time_offset.py \
  --forward-mm 101.2 --reverse-mm 100.8 --speed-mm-s 20
```

保留输出符号写入 `camera.fixed_time_offset_us`，然后必须重新做正反向验证。该方法要求两个位置来自相反方向、使用相同速度和同一坐标定义。

### 15.5 无硬件测试

```bash
cmake --build build --target HikSynchronizationCoreTest -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

仿真覆盖 100 Hz 机器人、60 fps 相机、控制器时间映射、滤波恒速识别、匀速插值、时间缺口、SDK逐包序号断档、FrameID 跳变、多线程图片写盘，以及独立连续重建线程池生成 raw/voxel PLY，不连接相机或 FR5。
# ROS 2 一体化重构

按照“机器人焊接自动标定与主动扫描焊接一体化软件方案 V2”实现的新工作区、运行方式与
安全边界见 [ROS2_IMPLEMENTATION.md](ROS2_IMPLEMENTATION.md)。原 Qt 工具继续保留作算法和
实机基线。

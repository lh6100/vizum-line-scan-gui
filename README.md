# Vizum Line Scan GUI

Qt-based desktop GUI for Vizum VzNL SDK line-laser 3D reconstruction cameras. It wraps the SDK flow for device connection, dust-cover control, swing-motor line scan, and PLY point-cloud export.

## Features

- Connect, disconnect, and reboot a Vizum Ethernet laser robot camera.
- Read SDK, device, firmware, algorithm, hardware, and swing-motor version information.
- Capture and preview an RGB frame from supported Vizum RGB cameras.
- Open and close the camera dust cover.
- Run one swing-motor line scan and save the reconstructed point cloud as a `.ply` file.
- Automatically display the latest saved PLY in the weld seam fitting page after each successful scan.
- Export a welding-pipeline input sidecar `<scan>_points.csv` with clean camera-frame points in metres.
- Repeat scans in the same session without reconnecting the camera.
- Keep SDK callbacks lightweight by deep-copying laser lines into a queue and writing PLY data from the worker thread.
- Load a PLY point cloud, click near a weld seam, fit a 3D line segment, and drag the segment endpoints for manual correction.

## SDK Layout

This repository includes the Vizum headers and Linux x64 SDK libraries needed to build the GUI:

```text
VizumScanGUI/
  SDK/
    VzNLSDK/
      Inc/
      Linux/x64/
  src/
```

The CMake file uses the bundled SDK first:

- headers: `SDK/VzNLSDK/Inc`
- libraries: `SDK/VzNLSDK/Linux/x64`

It also keeps compatibility with the original sibling layout at `../SDK/VzNLSDK` for local development.

## Build

Install Qt 5, VTK Qt support, PCL, Eigen, and CMake, then build:

```bash
sudo apt install cmake qtbase5-dev libvtk9-dev libvtk9-qt-dev libpcl-dev libeigen3-dev
```

```bash
cd VizumScanGUI
cmake -S . -B build
cmake --build build -j$(nproc)
```

Run:

```bash
./build/VizumScanGUI
```

## Usage

1. Connect the camera and network adapter.
2. Start the GUI.
3. Click `连接设备`.
4. Click `开盖` if the device supports dust-cover control.
5. Click `获取 RGB 图片` to capture and preview one RGB frame if the camera supports RGB.
6. Click `线扫建图并保存 PLY`, choose an output path, and wait for the scan to finish.
7. The saved PLY is loaded automatically in the `焊缝拟合` tab.
8. Click the scan button again to perform another scan and save another PLY file.
9. Click `关盖` and `关闭设备` when finished.

## Weld Seam Fitting

Open the `焊缝拟合` tab:

1. Click `加载 PLY` and select a scanned point cloud.
2. Middle-click a point near the weld seam.
3. The tool uses a KDTree radius search around the picked point and fits a 3D line with RANSAC.
4. The red line is the fitted seam segment. The green sphere is the start point and the red sphere is the end point.
5. Use the mouse wheel to zoom, right-drag to rotate, and left-drag either sphere to manually adjust the segment endpoint.
6. Copy the result to get the start/end coordinates in camera coordinates.

The viewer automatically downsamples the displayed points for interaction when a cloud is large, but fitting still uses the full-resolution point cloud.

Each successful scan writes three files next to the selected `.ply`:

```text
<scan>.ply          # SDK PLY export
<scan>_points.csv   # VIZUM point-cloud input for the welding pipeline
<scan>.csv          # robot flange pose placeholder: x,y,z,rx,ry,rz
```

`<scan>_points.csv` contains SDK raw coordinates and metre-normalized coordinates:

```text
line_idx,point_idx,frame_idx,timestamp,swing_angle,
x_raw,y_raw,z_raw,x_m,y_m,z_m,rgb_uint,r,g,b
```

The welding pipeline consumes `x_m/y_m/z_m` as camera-frame metres. The default raw-to-metre scale is `0.001`, which matches millimetre SDK output. If your VIZUM SDK/PLY already outputs metres, start the GUI with:

```bash
VIZUM_POINT_UNIT_SCALE=1 ./build/VizumScanGUI
```

## Welding Pipeline Integration

For `~/lh/bianxierobot/project-weld-anything-20260529`, copy or save the scan files into its `data/` directory:

```text
project-weld-anything-20260529/data/<scan>_points.csv
project-weld-anything-20260529/data/<scan>.csv
```

The welding project has been adapted to treat `*_points.csv + .csv` as a data group. RVC depth input still works as before; VIZUM groups bypass the old depth/intrinsics path and use direct camera-frame points.

Calibration placeholders are provided in `config/`:

```text
config/camera_offset_template.csv  # flange -> camera, fill after hand-eye calibration
config/tool_offset_template.csv    # flange -> TCP/tool, fill after TCP calibration
```

After calibration, place the filled files in the welding project root as `camera_offset.csv` and `tool_offset.csv`. Until then, identity placeholders are valid for software testing but not robot execution.

## 中文操作流程：接入免示教焊接工程

目标是把替换点放在“点云输入层”：VIZUM 负责输出干净的相机坐标系点云，焊接工程负责把点云变成钢筋线、焊缝点和机器人坐标。焊接路径生成层不用改。

### 1. 编译 VIZUM 程序

```bash
cd ~/lh/vizum-line-scan-gui
cmake -S . -B build
cmake --build build -j$(nproc)
```

### 2. 启动 VIZUM 程序

如果 VIZUM SDK 输出坐标是毫米，直接启动：

```bash
./build/VizumScanGUI
```

如果确认 VIZUM SDK 输出坐标已经是米，启动时改成：

```bash
VIZUM_POINT_UNIT_SCALE=1 ./build/VizumScanGUI
```

默认比例是 `0.001`，也就是把 SDK 原始坐标从毫米转换成米。焊接工程内部的几何算法统一按米计算，例如 `0.003` 表示 3 mm、`0.02` 表示 20 mm。最终保存给机器人执行的 `camera_points/base_points/flange_points` 仍然会转回毫米。

### 3. 扫描并保存到焊接工程 data 目录

在 GUI 中依次操作：

1. 点击 `连接设备`
2. 如果设备支持防尘盖，点击 `开盖`
3. 点击 `线扫建图并保存 PLY`
4. 保存路径建议直接选到焊接工程的 `data` 目录，例如：

```text
~/lh/bianxierobot/project-weld-anything-20260529/data/vizum_test_001.ply
```

扫描成功后会自动生成：

```text
data/vizum_test_001.ply
data/vizum_test_001_points.csv
data/vizum_test_001.csv
```

含义如下：

```text
vizum_test_001.ply          # VIZUM SDK 输出的 PLY 点云
vizum_test_001_points.csv   # 焊接工程读取的 VIZUM 相机坐标系点云
vizum_test_001.csv          # 当前帧机器人法兰位姿，默认先写 0 占位
```

`*_points.csv` 中的关键列是：

```text
x_m,y_m,z_m
```

这三列是相机坐标系下的点，单位是米。

### 4. 填入采集时机器人法兰位姿

打开刚生成的位姿文件：

```text
~/lh/bianxierobot/project-weld-anything-20260529/data/vizum_test_001.csv
```

默认内容是：

```csv
x,y,z,rx,ry,rz
0,0,0,0,0,0
```

请替换成采集这帧点云时法奥机器人的法兰位姿。这里保持焊接工程原来的格式：

```text
x/y/z：毫米
rx/ry/rz：角度
```

如果只是验证点云显示、平面拟合、直线提取，可以暂时用 0 占位；如果要转换到机器人坐标或执行焊接，必须填真实位姿。

### 5. 准备手眼标定和 TCP 标定文件

焊接工程根目录需要两个标定文件：

```text
~/lh/bianxierobot/project-weld-anything-20260529/camera_offset.csv
~/lh/bianxierobot/project-weld-anything-20260529/tool_offset.csv
```

本工程提供了模板：

```text
config/camera_offset_template.csv
config/tool_offset_template.csv
```

格式都是：

```csv
x,y,z,rx,ry,rz
0,0,0,0,0,0
```

含义：

```text
camera_offset.csv：法兰坐标系 -> VIZUM 相机坐标系，也就是 flange -> camera
tool_offset.csv：法兰坐标系 -> 焊枪 TCP/tool，也就是 flange -> TCP
```

未标定前可以先用全 0 占位验证软件流程，但不能用于真实机器人执行。

### 6. 在焊接工程中加载 VIZUM 点云

启动焊接工程：

```bash
cd ~/lh/bianxierobot/project-weld-anything-20260529
python app.py
```

进入点云/路径生成界面后：

1. 点击 `刷新`
2. 选择刚才的数据组，例如 `vizum_test_001`
3. 切到 `点云` 视图
4. 点击 `点云：拟合平面`
5. 如果平板区域显示为浅绿色，说明 VIZUM 点云输入层已经正常
6. 点击 `法线区域提取`
7. 点击 `直线端点提取`
8. 检查钢筋线是否正确
9. 再继续点击：

```text
计算钢筋焊接点
焊枪回缩
远离平面
焊枪摆动
保存
```

也可以直接点击：

```text
一键钢筋生成
```

对于 VIZUM 数据组，焊接工程会自动走纯点云流程：

```text
VIZUM 点云
  -> 点云拟合平面
  -> 法线变化提取钢筋线
  -> 原有焊点计算
  -> 原有回缩/远离平面/摆动
  -> 保存机器人运行点
```

如本 RVC 的 `png + tif + csv` 数据仍按原流程运行，不受 VIZUM 点云输入分支影响。

### 7. 现场首先检查两件事

第一，检查点云单位是否正确。如果点云看起来大 1000 倍或小 1000 倍，就说明 `VIZUM_POINT_UNIT_SCALE` 需要调整：

```bash
VIZUM_POINT_UNIT_SCALE=1 ./build/VizumScanGUI      # SDK 原始点已经是米
./build/VizumScanGUI                              # SDK 原始点是毫米，默认转米
```

第二，先只做 `点云：拟合平面`。如果平面拟合稳定，再继续做直线提取和焊点生成。不要在手眼标定和 TCP 标定未完成时直接执行机器人焊接。

## 中文操作流程：VIZUM 点到法奥 TCP 目标

本工程现在增加了一个独立的机器人坐标转换/运动验证层，不改变 VIZUM 扫描 GUI，也不改变焊接路径生成层。它的输入是相机坐标系点，单位毫米；输出是法奥基坐标系焊缝点和 TCP 运动目标，单位毫米/角度。

核心坐标链为：

```text
P_base = T_base_flange * T_flange_camera * P_camera
```

默认不反转手眼矩阵。只有当你的标定结果是 `T_camera_flange` 时，才把 `config/handeye_config.yaml` 中的 `mode` 改成：

```yaml
mode: flange_to_camera
```

### 1. 检查配置文件

机器人 IP：

```text
config/robot_config.yaml
```

默认内容：

```yaml
robot_ip: 192.168.1.200
enable_robot_motion: false
```

手眼矩阵：

```text
config/handeye_config.yaml
```

默认矩阵是当前待标定值，表示 `T_flange_camera`，单位毫米：

```text
[-0.98,  0.00, -0.21, -114.12]
[-0.03, -0.99,  0.10,  -44.02]
[-0.21,  0.10,  0.97,  125.38]
[ 0.00,  0.00,  0.00,    1.00]
```

焊枪 TCP/tool3：

```text
config/tool_config.yaml
```

默认值：

```yaml
tool_id: 3
x: 0.326
y: -194.553
z: 368.943
rx: -169.517
ry: 8.355
rz: 1.904
```

如果程序已经连接机器人，会优先调用法奥 SDK 的 `GetToolCoordWithID(tool_id)` 读取工具坐标；没有连接时使用上面的手动配置。

运动安全开关：

```text
config/weld_motion_config.yaml
```

默认是安全干运行：

```yaml
dry_run: true
enable_robot_motion: false
enable_arc: false
```

### 2. 编译并运行离线验证

```bash
cd ~/lh/vizum-line-scan-gui
cmake -S . -B build
cmake --build build --target CameraToBaseTest -j$(nproc)
./build/CameraToBaseTest
```

期望看到接近下面的输出：

```text
Start_base(mm): [278.08, -509.08, -48.82]
End_base(mm):   [296.96, -515.72, -47.85]
Line length(mm): 20.04
Dry-run: no robot motion sent.
Camera-to-base test passed.
```

这说明相机点 `Start_camera=[35.615,61.376,376.638]` 和 `End_camera=[49.096,76.015,374.845]` 已经按法奥浮动坐标 ZYX 顺序转换到了基坐标系。当前程序使用的欧拉角公式是：

```text
R = Rz(RZ) * Ry(RY) * Rx(RX)
```

### 3. 真机运动前必须确认

先只连接机器人读取当前法兰/TCP，不运动。修改：

```yaml
# config/robot_config.yaml
robot_ip: 192.168.1.200
connect_robot: true
enable_robot_motion: false

# config/weld_motion_config.yaml
dry_run: true
enable_robot_motion: false
enable_arc: false
```

编译并运行现场连接验证程序：

```bash
cd ~/lh/vizum-line-scan-gui
cmake -S . -B build
cmake --build build --target FairinoWeldMoveDemo -j$(nproc)
./build/FairinoWeldMoveDemo
```

也可以直接传入 VIZUM 焊缝起点/终点，相机坐标系，单位毫米：

```bash
./build/FairinoWeldMoveDemo sx sy sz ex ey ez
```

例如：

```bash
./build/FairinoWeldMoveDemo 35.615 61.376 376.638 49.096 76.015 374.845
```

程序会读取并打印：

```text
Current flange pose
Current TCP pose
Tool3 coord
```

然后保持干运行，观察打印出的三个 TCP 目标：

```text
Approach TCP   # 起点上方 safe_height_mm
Start TCP      # 焊缝起点
End TCP        # 焊缝终点
```

确认无误后，才允许同时打开两个开关：

```yaml
# config/robot_config.yaml
connect_robot: true
enable_robot_motion: true

# config/weld_motion_config.yaml
dry_run: false
enable_robot_motion: true
```

如果需要程序自动上使能和切自动模式，再打开：

```yaml
# config/robot_config.yaml
auto_enable: true
auto_mode: true
```

程序会通过法奥 SDK 调用：

```text
RPC("192.168.1.200")
RobotEnable(1)     # 仅 auto_enable: true 时调用
Mode(0)            # 仅 auto_mode: true 时调用
GetActualToolFlangePose
GetActualTCPPose
GetToolCoordWithID(3)
MoveL
```

默认速度很低，且 `enable_arc` 仍为 `false`，当前版本只做 TCP 直线运动验证，不启动焊机。

### 4. 在 VizumScanGUI 里操作机械臂

启动 GUI：

```bash
./build/VizumScanGUI
```

界面现在有三个页签：

```text
线扫建图
焊缝拟合
机械臂控制
```

推荐现场顺序：

1. 在 `线扫建图` 页扫描并保存 PLY。
2. 在 `焊缝拟合` 页加载 PLY，点击焊缝附近点，得到 Start/End Camera 端点。
3. 切到 `机械臂控制` 页，端点会自动同步到 `焊缝 Camera 线段端点(mm)`。
4. 点击 `连接机械臂`。
5. 点击 `读取当前法兰/TCP`。
6. 点击 `计算焊枪 TCP 点位`，先看日志里的：

```text
Start_base
End_base
Approach TCP
Start TCP
End TCP
```

7. 保持 `干运行：只打印，不发运动` 勾选时，点击 `执行 MoveL 三点` 只会打印，不会动机器人。
8. 真运动前，必须确认点位安全，再取消干运行并勾选 `允许真运动 MoveL`。

机械臂控制页按钮含义：

```text
连接机械臂              RPC 连接 192.168.1.200
读取当前法兰/TCP        读取 GetActualToolFlangePose 和 GetActualTCPPose
上伺服                  RobotEnable(1)
下伺服                  RobotEnable(0)
切自动模式              Mode(0)
复位错误                ResetAllError
停止运动/急停移动       StopMotion，软件停止当前运动
暂停运动                PauseMotion
继续运动                ResumeMotion
计算焊枪 TCP 点位       只计算并打印目标点
执行 MoveL 三点          Approach -> Start -> End
```

`停止运动/急停移动` 是 SDK 的 `StopMotion` 软件停止命令，不替代控制柜硬件急停。

## Implementation Notes

The scan path follows the SDK sequence:

```text
VzNL_Init
VzNL_ResearchDevice
VzNL_BindEthernetEye if needed
VzNL_OpenDevice
VzNL_BeginDetectLaser
VzNL_SetTriggerMode
VzNL_EnableSwingMotor
VzNL_SetSwingScanMode(Once)
VzNL_StartAutoDetectEx
laser callback: deep-copy line data only
worker thread: VzNL_WriteLaserFile
VzNL_StopAutoDetect
VzNL_CloseLaserFile
VzNL_EndDetectLaser
VzNL_CloseDevice
VzNL_Destroy
```

The callback deliberately avoids blocking SDK calls, file I/O, and UI work. A bounded queue protects the process if disk writing cannot keep up with camera output.

Point CSV writing happens in the worker thread while draining the same deep-copied queue that feeds `VzNL_WriteLaserFile`, so SDK callbacks remain lightweight.

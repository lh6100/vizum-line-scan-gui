# 海康相机与线激光独立标定工具

这个目录提供一个独立的 Qt5 GUI，用于完成：

1. 海康面阵相机的内参和畸变标定；
2. 相机光学坐标系下的红色线激光平面标定；
3. 海康相机到 FR5 法兰的 eye-in-hand 手眼标定；
4. 不依赖 ROS 2 的 FR5 常亮线激光停稳扫描验证与 base 坐标系 PLY 累计。

标定 GUI 不依赖 ROS 2。它的手眼页面通过 Fairino SDK 只读连接 FR5，界面不提供运动按钮。独立的 `HikConstantLaserScan` 扫描程序会在操作者明确取消 dry-run、勾选安全确认并再次确认后发送低速 `MoveL` 和 `StopMotion`；它不会自动使能机器人、切换控制器模式或控制激光 IO。

## 1. 开始前必须确认

- 将相机、镜头和线激光固定到最终使用的机械结构上。完成标定后不能再改变镜头焦距、对焦、光圈、相机分辨率或 ROI，否则需要重新标定。
- 运行本工具前，先在 MVS 中停止取流、关闭设备并完全退出 MVS；同时关闭其他相机采集程序。本工具以独占模式打开相机，MVS 仍占用时会连接失败。
- 默认相机 IP 是 `192.168.1.56`。主机相机网卡必须和它位于同一网段。
- 内参采集时必须关闭红色线激光。
- 激光平面标定的每一组 `laser-off` / `laser-on` 必须使用完全相同的曝光和增益，而且两帧之间相机与标定板都不能移动。
- 标定 GUI 不会自动开关激光或控制机械臂运动。常亮扫描程序要求激光由人工保持开启，并会按示教路径真实控制机械臂。
- Fairino SDK 同一时间只能由一个进程占用。进入手眼页面前，必须停止 `ros2_fr5` 的 `fairino_state_publisher`、旧 GUI 或其他 FR5 SDK 客户端。

## 2. 独立构建和运行

要求：C++17、CMake、Qt5 Widgets、Eigen3、OpenCV（包含 `calib3d` 和 `aruco`）、海康 MVS SDK，以及手眼实时采集所需的 Fairino C++ SDK。代码默认查找：

```text
/opt/MVS/include/MvCameraControl.h
/opt/MVS/lib/64/libMvCameraControl.so
/home/zhulong/lh/vizum-line-scan-gui/SDK/fairino-cpp-sdk-3.9.4/
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
cd /home/zhulong/lh/vizum-line-scan-gui/myline_hik
./run_calibration_gui.sh
```

运行常亮单帧停稳扫描验证程序：

```bash
cd /home/zhulong/lh/vizum-line-scan-gui/myline_hik
./run_constant_laser_scan.sh
```

两个程序都会占用海康相机和 Fairino SDK，不能同时运行。启动扫描程序前应退出标定 GUI、MVS、`fairino_state_publisher` 以及其他 FR5 SDK 客户端。

启动脚本会主动隔离 Anaconda/Conda 的 Qt 和库路径，强制使用 Ubuntu 系统 Qt、系统 OpenCV，再运行程序。需要分开构建时，可使用另一个构建目录执行同样的隔离：

```bash
cd /home/zhulong/lh/vizum-line-scan-gui/myline_hik
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

## 3. 标定板定义：不要混淆 5×7 和 7×5

默认板参数为：

| 参数 | 默认值 |
| --- | ---: |
| OpenCV `squaresX` | 5 |
| OpenCV `squaresY` | 7 |
| 方格边长 | 22 mm |
| ArUco Marker 边长 | 16 mm |
| 字典 | `DICT_4X4_50` |
| 实际板尺寸 | 110 mm × 154 mm |
| ChArUco 内角点数 | 24 |

这里使用的是 OpenCV 定义：`squaresX` 是标定板自身 X 方向的方格列数，`squaresY` 是自身 Y 方向的方格行数。因此：

```text
宽度  = 5 × 22 mm = 110 mm
高度  = 7 × 22 mm = 154 mm
角点数 = (5 - 1) × (7 - 1) = 24
```

相机画面中的横竖方向不决定 X/Y。把标定板在镜头前旋转 90°，不需要切换成 7×5。只有实际打印文件或生成程序采用了 `squaresX=7、squaresY=5` 时，才应在尚无样本时点击界面的“无样本时切换 5×7 / 7×5”；转置后的物理尺寸是 154 mm × 110 mm，角点仍为 24 个。

添加任一内参、激光或手眼样本后，板参数会锁定。要修改它，必须清空这些样本并取消尚未完成的采集事务。

本工具按 OpenCV 4.5 的 legacy ChArUco 布局生成三维角点。当前 CMake 会拒绝 OpenCV 4.6 或更高版本，避免新版坐标约定与 YAML 中声明的 legacy 布局不一致。

## 4. 第一步：连接相机并确认图像

1. 完全退出 MVS。
2. 启动 GUI，确认 IP。当前默认值为 `192.168.1.56`。
3. 设置曝光、增益和超时。界面默认值分别为 `1825 us`、`0 dB` 和 `3000 ms`，它们只是起点，应以现场图像为准。
4. 点击“连接”。成功后程序记录相机型号、序列号和 IP，将缩放复位为 1、ROI 复位为全幅，并工作在 `Mono8`、软件触发模式。
5. 点击“单帧”，检查图像尺寸、清晰度、亮度以及是否过曝。
6. 固定好镜头对焦、焦距和光圈后，不要再改变它们。

每次实时采集的原图会立即保存到本次 session，同时把帧号、时间戳、实际曝光/增益、图像尺寸和相机身份追加到 `capture_manifest.csv`。激光配对还会显式记录 `pair_id`；同一对文件使用相同 ID，命名为 `laser_<时间戳>_off.png` 和 `laser_<时间戳>_on.png`。表格中删除样本或点击“清空”只会移除当前计算数据，不会删除已保存的原图。

## 5. 第二步：标定相机内参

### 5.1 采集 30–40 张图像

1. 关闭红色线激光。
2. 切换到“相机内参”页。
3. 使用“实时采样（激光关闭）”逐张采集；已有图片也可用“批量导入图片”。
4. 建议采集 30–40 张清晰图像，不要连续拍摄几乎相同的姿态。应覆盖：
   - 画面中心、四角和四条边；
   - 最近、常用和最远扫描工作距离；
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
6. 只有结果通过软件质量门槛后，“保存通过结果到 `config/hik_intrinsics.yaml`”才会启用。

当前正式内参文件的软件提升门槛为：

- 至少 15 个最终采用视图；
- 相机矩阵和畸变参数均为有限数；
- 总体重投影 RMS 不大于 0.45 px；
- 采用视图 RMS 的 P95 不大于 0.60 px，最大值不大于 0.80 px；
- 所有采用视图的角点范围至少覆盖图像宽、高各 70%，标定板中心至少占到九宫格中的 5 格；
- 最大/最小角点凸包面积比至少为 1.40；
- 板法向的最大夹角差至少 12°，深度跨度至少为 30 mm 或平均深度的 6%（取较大值）。

未通过上述门槛的结果仍可作为 session 候选保存，但不能覆盖正式配置。实际工程验收建议继续争取总体 RMS `< 0.3 px`，并确认去畸变后的直线无明显弯曲、近中远距离均无系统偏差。

## 6. 第三步：标定激光平面

### 6.1 加载内参

进入“激光平面”页后，先执行以下任一项：

- 点击“沿用本页刚求出的内参”；
- 点击“加载内参 YAML”，选择已批准的 `config/hik_intrinsics.yaml` 或 session 候选。

内参、图像分辨率、镜头状态和标定板定义必须与激光平面采集一致。当前内参或 session 候选可用于调试和生成激光候选；只有明确加载已批准的 `config/hik_intrinsics.yaml`，激光平面才可能提升为正式配置。

### 6.2 严格采集 laser-off / laser-on 配对

每个标定板姿态都必须按下面顺序完成一组配对：

1. 把标定板放到目标距离和倾角，确保红色激光能够落在标定板有效区域内，然后等待结构完全静止。
2. 关闭激光，点击“1. 采 laser-off”。
3. 确认界面显示 `laser-off 已保存`。从这一刻起，严禁移动相机、标定板、支架或工作台，也不要改变曝光、增益、对焦、光圈、分辨率和 ROI。
4. 只打开激光，点击“2. 板不动，采 laser-on”。
5. 查看该行的共同角点、像素位移、板平移/旋转、条纹点、3D 点和状态。被拒绝的配对应删除后整组重拍，不能把不同姿态的 off/on 强行组合。
6. 移动标定板到下一个姿态，重复以上步骤。

离线数据可用“离线导入成对图”，但文件也必须是真正同姿态、同曝光的 off/on 原图。界面会先让你选择一批 off 图，再选择一批 on 图，然后分别按文件名排序并逐项配对；两批数量必须相同，命名必须保证排序后一一对应。离线导入缺少可信的相机身份和曝光元数据，因此只允许保存候选，不允许覆盖正式配置。

当前每组配对的自动质量检查包括：

- off/on 两张图都通过 ChArUco 检测；
- 至少 8 个共同角点；
- 角点位移均值不大于 0.35 px，P95 不大于 0.75 px；
- 两次估计的板姿态平移差不大于 0.5 mm，旋转差不大于 0.2°；
- 单次板姿态重投影 RMS 不大于 0.4 px，最大点误差不大于 0.8 px；
- 差分条纹至少提取 80 个有效点，条纹宽度不大于 20 px；
- 至少 60 条相机射线在标定板物理范围内形成有效 3D 交点。

### 6.3 采集姿态覆盖

建议采集 12–16 个用于拟合的不同姿态，而不是只满足界面允许求解的最低数量。姿态应覆盖：

- 实际扫描的最近、常用和最远深度；
- 标定板在视场左、中、右及上、中、下的位置；
- 绕 X/Y 轴的正负倾角，避免全部正对相机；
- 激光线在标定板上的不同位置。

不要让某一个距离或倾角占据绝大多数样本。条件允许时，再单独采集 4–6 个不参加拟合的验证姿态，用于检查近端/远端是否存在系统偏差。

### 6.4 求解激光平面

1. 只有状态为通过的配对参与求解。
2. 界面 RANSAC 距离阈值默认是 0.3 mm。没有明确实验依据时先保持默认值，不要为了提高内点率盲目放宽。
3. 至少 8 个通过质量检查的不同姿态后，“求解激光平面”才会启用；工程采集仍建议达到上面的 12–16 个训练姿态。
4. 点击“求解激光平面”。算法要求每个参与姿态至少 60 个 3D 点，每个姿态最多均衡抽取 250 点，执行 2000 次跨姿态 RANSAC，再以按姿态平衡的 SVD 精化。
5. 拟合的最小内点比例是 90%。检查结果中的姿态数、点数、内点比例、RMS、P95 和最大距离。
6. 成功求解后先保存 session 候选；只有达到正式质量门槛后，才能提升到 `config/hik_laser_plane.yaml`。

当前正式激光平面文件的软件提升门槛为：

- 有有效的平面求解结果；
- 必须加载已批准的 `config/hik_intrinsics.yaml`，且其中至少采用 15 个视图、总 RMS 不大于 0.45 px；
- 所用内参文件的 SHA-256 与加载时一致，相机序列号与实时设备一致，采集清单存在且可校验；
- 至少 8 个有效板姿态；
- 去除平移差小于 5 mm 且法向差小于 3°的近重复样本后，仍至少有 8 个姿态；
- 内点比例不低于 90%；
- 法向量长度与 1 的差不超过 `1e-6`；
- 平面系数、`d`、内点 RMS/P95 都是有限数；
- 平面内点距离 RMS 不大于 0.20 mm，P95 不大于 0.30 mm；
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

- `config/hik_intrinsics.yaml`；
- `config/hik_laser_plane.yaml`；
- 激光平面记录的内参 SHA-256 与当前正式内参是否一致；
- 正式内参绑定的相机序列号、图像尺寸和激光平面有效 Z 范围。

实时采集顺序：

1. 将标准平板放在标定有效深度内，机器人、相机和平板全部保持静止；
2. 使用激光平面标定时合适的曝光与增益；
3. 关闭激光，点击“1. 关闭激光，采 off”；
4. 不移动任何物体、不修改曝光/增益，打开激光；
5. 点击“2. 保持不动，打开激光采 on”；
6. 程序执行 on-off 差分、逐行连续峰值选择和灰度重心亚像素定位；
7. 像素经正式内参去畸变为相机射线，并与正式激光平面求交；
8. 有效点自动保存为相机光学坐标系下的 PLY 和 CSV。

当前条纹提取器按“每个图像行一个中心”工作，因此激光线应大致沿图像竖直方向并覆盖至少 80 行。输出坐标为 `hik_camera_optical_frame`，X 向右、Y 向下、Z 向前，单位 mm。超出激光平面 YAML 所记录有效 Z 范围的点会被拒绝，而不是外推。

建议在有效范围的近、中、远位置分别采至少 3 组，检查静态重复性。表格中的指标含义不同：

- `直线 RMS`：单条三维轮廓对最佳拟合直线的距离，只能评价轮廓直线度；
- `平板 RMS`：只有 off 图中检测到 ChArUco 板、并且至少 60 个轮廓点落在板物理区域内时才提供；这是重建点到独立 ChArUco 板平面的距离，可按 `< 0.3 mm` 做首轮验收；
- 普通无纹理平板只有一条交线，不能从单帧唯一拟合三维平面，因此程序不会把直线 RMS 冒充为平板 RMS。

也可以点击“离线导入 off/on”复算已有原图；离线导入无法验证曝光/增益元数据，必须由操作者确认两张图确实同姿态、同参数。

## 8. 第五步：FR5 eye-in-hand 手眼标定

### 8.1 现场布置与连接

1. 将 ChArUco 板牢固固定在机器人基座附近的工作台上；从第一张到最后一张都不能移动板。
2. 关闭红色线激光，保持正式内参标定后的分辨率、ROI、镜头焦距、对焦和光圈不变。
3. 停止 `ros2_fr5` 的 FR5 状态发布节点和其他 Fairino SDK 客户端。
4. 连接海康相机，然后进入“FR5 手眼标定”页，以默认 `192.168.1.200` 连接 FR5。
5. 可先点“读取当前法兰”，确认显示的是控制器实际 `T_base_flange`，单位为 mm/deg。

机器人连接是只读的。工具不提供运动按钮；每次都由操作员以安全方式手动换姿态，机器人完全停止后再点击“采集一个手眼样本”。

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

前后法兰变化超过 `0.10 mm` 或 `0.05°` 时，样本会被拒绝。板位姿要求至少 12 个 ChArUco 角点、重投影 RMS 不大于 `0.40 px`、最大点误差不大于 `0.80 px`。原图即使被拒绝也会保留，避免无证据地丢失现场数据。

姿态应同时变化位置和方向，特别要包含绕至少两个旋转轴的正负倾斜。软件求解最低要求为：

- 至少 15 个有效样本；
- 法兰位置最大跨度至少 50 mm；
- 法兰姿态最大跨度至少 20°；
- 第二旋转轴离散度至少 5°，避免所有姿态只绕一个轴变化。

不要只沿一条直线平移，也不要让所有图片的标定板几乎正对相机。建议采集 20–25 个姿态，为自动离群剔除留余量。

### 8.3 求解、验证和保存

点击“鲁棒求解 `T_flange_camera`”。程序同时计算 OpenCV 的 TSAI、PARK、HORAUD、ANDREFF 和 DANIILIDIS 五种方法，用固定板在 base 坐标系下的一致性选择最佳结果，并最多进行 3 轮鲁棒离群剔除。

使用的关系和输出方向是：

```text
T_base_board = T_base_flange · T_flange_camera · T_camera_board
```

`T_flange_camera` 将 `hik_camera_optical_frame` 中的点变换到 FR5 法兰坐标系，平移单位为 mm。正式写入 `config/hik_handeye.yaml` 的门槛为：

- 最终采用样本不少于 15 个；
- 固定板 base 坐标平移一致性 RMS 不大于 1.0 mm；
- 固定板 base 坐标旋转一致性 RMS 不大于 0.30°；
- 当前相机序列号与正式内参一致；
- 正式内参的 SHA-256 在采集期间未变化。

候选结果可先保存在 session；质量通过后才可原子更新正式文件。正式扫描前还应额外采集 4–6 个不参加求解的验证姿态，确认 `T_base_board` 没有随姿态产生固定方向漂移，并在 RViz 中检查相机光学轴方向与实际安装一致。

## 9. 第六步：常亮单帧停稳扫描验证

### 9.1 先做完全不运动的单点验证

1. 退出标定 GUI、MVS、ROS 2 法兰发布节点和其他 Fairino SDK 客户端。
2. 将工件放在激光平面的有效相机 Z 范围内；当前正式文件记录约为 `484.41–557.14 mm`。
3. 让线激光保持常亮，运行 `./run_constant_laser_scan.sh`，连接相机和 FR5。
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
   示教后可使用“编辑起点数值”和“编辑终点数值”二次修改 XYZ/RPY。起点 RPY 决定整条扫描姿态；终点 RPY 只记录、不参与插值。每次手动修改后必须重新生成 dry-run。
3. 首次路径建议 `20–50 mm`，步距先用 `5 mm`。界面默认开启 `100 mm` 的“验证路径长度保护”和 `201` 个的“目标数量保护”；需要更长、更密的路径时可提高对应保护值，或明确取消保护。
4. 保持 `dry-run` 勾选，点击“生成并打印目标列表”或“开始停稳扫描”，逐项核对所有 base 法兰目标。
5. 在控制器侧完成使能和自动模式确认，核对 `tool=0、user=0` 表示目标是 base 下的法兰位姿，而不是焊枪 TCP 或其他工具坐标。
6. 确认扫描头、工件、夹具、线缆和奇异位形均安全，人员守在物理急停旁；再取消 dry-run、勾选安全确认并接受最终确认框。
7. 首次真运动使用默认 `5%` 速度、`20%` 加速度、`250 ms` 停稳，确认流程稳定后再把步距改为 `2 mm`。

例如路径为 `182.703 mm`、步距 `0.5 mm` 时，会生成 `ceil(182.703 / 0.5) + 1 = 367` 个目标。可把“最大验证路径”设为 `200 mm`、把“最大目标数量”设为 `400`，无需关闭保护，也不会丢点。367 次停稳、取图和写盘耗时较长，必须先用较大步距验证完整路径，再生成并核对 0.5 mm 的 dry-run。

每个点严格执行：非阻塞 `MoveL`、轮询到位、停稳等待、读取法兰 before、软件触发一帧、读取法兰 after、静止校验、三角重建和 base 坐标累计。扫描工具不会做碰撞规划，也不会自动从当前位置规划到起点；第一次到起点同样是一段直线 `MoveL`。界面“停止”会终止采集状态机，并在活动运动期间通过同一个 SDK 会话发送 `StopMotion`，但它不能替代控制柜物理急停。

### 9.3 输出与首轮判断

每次单点或扫描会话保存在：

```text
data/scans/scan_YYYYMMDD_HHMMSS_mmm/
├── images/profile_*.png
├── scan_manifest.csv
├── scan_raw.ply
└── scan_voxel.ply
```

`scan_manifest.csv` 记录图像、帧号、设备/主机时间戳、实际曝光和增益、采图前后完整法兰 XYZ/RPY、静止差、直线 RMS、是否启用平板门槛、条纹饱和比例及三份标定 SHA-256。`scan_raw.ply` 是未降采样 base 点云，`scan_voxel.ply` 默认按 `0.5 mm` 体素平均；PLY 单位为 mm，并保留置信度、响应、轮廓序号和原像素坐标。

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
│   └── session_YYYYMMDD_HHMMSS_mmm/
│       ├── capture_manifest.csv
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
│   └── scan_YYYYMMDD_HHMMSS_mmm/
│       ├── images/profile_*.png
│       ├── scan_manifest.csv
│       ├── scan_raw.ply
│       └── scan_voxel.ply
└── config/
    ├── hik_intrinsics.yaml
    ├── hik_laser_plane.yaml
    └── hik_handeye.yaml
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

### 找不到 `192.168.1.56`

检查相机供电、网线、主机网卡地址和子网掩码；确认 GUI 中输入的是相机当前 IP。日志会列出实际枚举到的 GigE 相机。

### 提示未编译 MVS 支持

确认 `/opt/MVS/include/MvCameraControl.h` 和 `/opt/MVS/lib/64/libMvCameraControl.so` 存在，然后重新运行 CMake。缺少 SDK 时只能离线导入。

### CMake/链接使用了 Anaconda Qt

退出 Conda，清理旧构建缓存后重新生成，或按第 2 节指定系统 `Qt5_DIR`。若报错路径含 `/home/zhulong/anaconda3/lib/libQt5...`，应优先排查这个问题。

### ChArUco 检测一直失败

依次检查 5×7/7×5 定义、22/16 mm、字典、legacy 图案、打印缩放、对焦、曝光、反光和裁边。内参图必须关闭激光。

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

当前平板不在 `hik_laser_plane.yaml` 的 `validity.camera_z_min_mm` 到 `camera_z_max_mm` 范围内。应移动平板进入标定覆盖范围；不要手工放宽范围来掩盖外推。

### FR5 连接报 `RPC err=-2` 或提示被占用

Fairino SDK 同时只能由一个客户端持有连接。停止 `ros2_fr5` 的 `fairino_state_publisher`、其他机器人 GUI 和测试程序，然后完全退出并重启本标定工具。SDK 失败后不能在同一进程安全重连，界面会要求重启，这是为了规避旧 SDK 后台线程竞态。

### 手眼样本提示机器人未静止

采图前后实际法兰变化超过 `0.10 mm / 0.05°`。等待机械臂完全停止，避免拖动示教、外力振动或在采集过程中碰支架后重拍；不要通过放宽阈值掩盖图像与位姿错配。

### 手眼求解提示姿态多样性不足

样本数量够但位姿跨度不足。增加绕 X/Y 等至少两个轴的正负倾斜，并同时改变距离和画面位置；仅平移或所有姿态近似平行无法可靠约束 eye-in-hand 旋转。

## 14. 当前边界与下一步

本工具当前完成相机内参、相机坐标系下的激光平面、静态 off/on 单帧三维轮廓、FR5 eye-in-hand 手眼标定，以及独立的常亮线激光停稳式直线扫描；仍不包含：

- 激光器开关控制；
- FR5 自动使能、控制器模式切换、碰撞规划、物理急停或激光 IO 控制；
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
          → synchronization.csv + 下游 SynchronizedFrame 接口
独立元数据 WriterThread → robot_raw.csv、camera_raw.csv、session_summary.json
```

同步配置在 `config/synchronization.yaml`。电脑侧同步时间全部来自 `CLOCK_MONOTONIC_RAW`；UTC 日期只用于会话摘要和目录名。默认相机 60 fps、曝光 1825 μs、机器人反馈请求周期 10 ms（100 Hz），环形缓冲为 2048 条且至少覆盖 5 秒。同步会话在机器人到达扫描起点后才创建，避免把移到起点的长时间运动混入连续扫描统计。

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

`robot.time_mode` 默认 `controller_timestamp`：将控制器 `robotTime` 仿射映射到主机单调时钟，拟合稳定前自动使用 `HOST_RECEIVE`。`frame_cnt` 只作为循环计数诊断字段；它的跳变不再直接判定同步无效，真正的数据缺口由相邻位姿的时间间隔和 `ROBOT_GAP_TOO_LARGE` 判定。

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

默认输出到 `data/scan/sync_scan_<时间>/`：

- `robot_raw.csv`：SDK逐包序号、原始及展开后的 `frame_cnt`、`RecvPkg`成功返回时刻、getter耗时/结果、控制器时刻、法兰、关节、原始/滤波/位置拟合速度和运动阶段；
- `camera_raw.csv`：FrameID、设备 raw/ns 时间戳、回调时刻、曝光、尺寸、连续性、图片名和写盘入队状态；
- `synchronization.csv`：曝光中点、前后状态、alpha、间隔、插值四元数、原始/滤波速度、运动阶段和质量；
- `session_summary.json`：设备帧率、软件接受帧率、20004完整包频率、SDK逐包序号/frame_cnt诊断、getter平均/P99/最大耗时、队列溢出、有效帧和相机/机器人时钟拟合残差；
- `images/frame_<FrameID>.png`：默认由 2 个独立后台线程以 PNG 压缩级别 1 保存的 Mono8 原图。

查看有效同步记录：

```bash
column -s, -t < data/scan/sync_scan_*/synchronization.csv | less -S
awk -F, 'NR==1 || $20 != "VALID"' data/scan/sync_scan_*/synchronization.csv
```

`actual_camera_device_fps` 由设备时间戳和 FrameID 跨度计算，`accepted_camera_callback_fps` 表示软件实际接受吞吐。`camera_raw.csv` 的 `frame_id_continuous=0`，或摘要中的 `camera_frame_id_skips/camera_queue_overflows/image_pool_exhaustions/image_queue_overflows` 非零，表示相机链路存在缺图。`robot_receive_sequence_gaps` 才表示SDK回调到SPSC消费链路确实缺少完整包；`robot_frame_counter_skips/robot_frame_counter_duplicates/robot_frame_counter_out_of_order` 仅为控制器内部计数诊断，不参与逐包去重。机器人同步有效性以 `robot_gap_max_ms` 和 `ROBOT_GAP_TOO_LARGE` 为准。加减速段仍保存，但默认标为 `SPEED_NOT_STABLE`，不计入有效建图帧。

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

仿真覆盖 100 Hz 机器人、60 fps 相机、控制器时间映射、滤波恒速识别、匀速插值、时间缺口、FrameID 跳变和多线程图片写盘，不连接相机或 FR5。

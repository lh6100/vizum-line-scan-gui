# 海康相机—FR5 线激光标定与扫描改动说明

## 目标

本次改动新增一套独立的 `myline_hik` 工具链，用于海康 Mono8 相机、线激光器和 Fairino FR5 机器人的实机标定与停稳扫描，同时补充 FR5 ROS 2 可视化、实施文档和高反光工件研究方案。

## 主要代码改动

### 标定工具

- 新增 ChArUco 相机内参采集、鲁棒求解、覆盖率检查和正式 YAML 审批流程。
- 新增严格配对的 `laser-off` / `laser-on` 激光平面标定，校验相机身份、曝光、增益、板位姿静止性和数据来源。
- 激光条纹同时支持水平、竖直方向，自动选择有效中心点更多的方向；水平条纹按列计算亚像素中心。
- 激光平面使用跨姿态平衡 RANSAC 和加权精修。精修采用 `3×3` 加权散布矩阵的最小特征向量，避免对 `N×3` 数据执行 `FULL_UV` 时生成巨型 `N×N` 矩阵造成卡死。
- 新增眼在手上手眼标定，比较 OpenCV 的 TSAI、PARK、HORAUD、ANDREFF 和 DANIILIDIS 方法，并以固定标定板在 `base_link` 下的一致性选择结果。
- 正式内参、激光平面和手眼文件包含相机序列号、坐标系约定、输入清单及 SHA-256 绑定，避免不同批次标定混用。

### 常亮线激光扫描

- 新增 FR5 停稳路径扫描：逐点 `MoveL`、到位等待、采图前后法兰读取、静止性检查和单帧三角重建。
- 点按照以下坐标链累计到机器人基坐标系：

  ```text
  P_base = T_base_flange · T_flange_camera · P_camera
  ```

- 输出原始点云、体素点云、逐帧原图和扫描清单；记录曝光、增益、机器人静止量、深度范围、轮廓直线度及条纹饱和比例。
- 常亮单帧背景抑制兼容水平和竖直激光，并保留深度有效范围检查，禁止未经验证的激光平面外推。

### FR5 ROS 2 支持

- 新增 FR5 机器人描述、网格、实时关节/法兰状态发布和 RViz 启动配置。
- 新增标定坐标系发布节点，便于核对 `base_link`、法兰和相机光学坐标系关系。
- 构建、安装和日志目录由仓库忽略规则排除。

## 配置文件

`myline_hik/config` 保存当前实机正式标定：

- `hik_intrinsics.yaml`：相机内参及畸变参数。
- `hik_laser_plane.yaml`：相机光学坐标系下的激光平面、拟合质量及有效深度范围。
- `hik_handeye.yaml`：`T_flange_camera` 及固定板一致性指标。

这些文件绑定具体相机和数据清单。更换相机、镜头、分辨率或机械安装后必须重新标定，不能只复制矩阵或手工放宽有效深度范围。

## 数据与构建产物

以下内容不进入 Git：

- `myline_hik/build*`；
- `myline_hik/data/calibration`；
- `myline_hik/data/scans`；
- ROS 2 的 `build`、`install`、`log`；
- PLY、PCD、LAS、日志和临时备份。

## 验证

核心测试命令：

```bash
cmake --build myline_hik/build --target HikCalibrationCoreTest HikLineLaserCalibration HikConstantLaserScan -j2
ctest --test-dir myline_hik/build --output-on-failure -R HikCalibrationCoreTest
```

测试覆盖 ChArUco 检测、内参与 YAML 往返、水平/竖直条纹提取、激光平面拟合、静态轮廓重建、单帧常亮扫描和手眼求解。

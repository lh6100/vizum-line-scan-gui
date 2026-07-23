# FR5 + 海康相机 + 单线激光数字孪生

该目录把仓库中的 FR5 URDF、海康相机内参、手眼矩阵和激光平面组合为 Isaac Sim 6.0 USD 场景。

## 构建与启动

```bash
cd /home/zhulong/lh/vizum-line-scan-gui
./isaacSim/fr5_line_laser_twin/build.sh
./isaacSim/fr5_line_laser_twin/launch.sh
```

构建脚本对 URDF、全部 STL、转换参数、转换器源码以及 Isaac Lab/Isaac Sim 版本计算路径无关的内容哈希。输入未变化时复用经过结构校验的 USD；任一输入变化后在临时目录转换、验证，再发布到新的哈希目录。组合场景也先写同目录临时文件，完整重开验证后才原子替换正式文件。

`launch.sh` 启动前会重新核对当前三份标定 YAML、孪生配置和引用的机器人 USD 哈希，避免误开旧标定生成的场景。若 Isaac Lab 安装在其他位置，可通过 `ISAACLAB_ROOT=/path/to/IsaacLab` 指定。

本机使用的是 pip 安装版 `isaacsim`。`launch.sh` 等 `isaacsim.exp.full` 完成初始化后再显式打开生成的场景，随后确认根层路径及 `/World/FR5`，并自动把视口框选到整台机器人。不能只把 USD 写在 `isaacsim.exp.full` 后面作为第二个位置参数：pip 启动器不会因此打开 Stage，表现就是只有空白画面；也不能在 Full experience 初始化中途过早打开，否则初始空 Stage 会再次覆盖它。需要换 experience 时可设置 `ISAAC_SIM_EXPERIENCE=/path/to/experience.kit`（也接受 `isaacsim.exp.full` 这样的内置名称）；需要换 Python 环境时设置 `ISAAC_SIM_PYTHON=/path/to/python`。

启动成功时，终端会明确出现：

```text
[FR5 Twin] Stage opened successfully; robot found at /World/FR5
[FR5 Twin] Viewport framed on /World/FR5
```

若第一行存在而第二行提示自动框选失败，场景仍已正确打开，可在 Stage 树中选择 `/World/FR5` 后按 `F`。

生成文件位于：

```text
isaacSim/generated/fr5_hik_twin/fr5_hik_line_laser_twin.usda
```

`isaacSim/generated/` 是可再生输出，已从 Git 排除。

## 已导入内容

- FR5 V6 的 6 轴 articulation、STL 视觉/碰撞几何、关节限位和惯性；
- `hik_handeye.yaml` 的 `T_flange_camera`，平移从 mm 转为 m；
- `hik_intrinsics.yaml` 的 1440×1080 分辨率、K、五参数 `plumb_bob` 畸变，并写入 Isaac Sim 6 的 `OmniLensDistortionOpenCvPinholeAPI` 和同分辨率 RenderProduct；
- `hik_laser_plane.yaml` 的相机光学坐标系平面和有效 Z 范围；
- 可直接 Play 的 `PhysicsScene`、Z 向上重力和 60 Hz 时间基准；
- 蓝色半透明四边形作为标定激光平面的调试可视化；它默认 `guide + invisible`，不会遮挡 RGB、深度或分割图。需要观察时可在 Stage 属性中临时把 `visibility` 改为 `inherited`。

相机光学坐标保持 OpenCV 约定：X 向右、Y 向下、Z 向前。USD Camera 子节点额外绕 X 轴旋转 180°，用于映射 USD 的“沿 -Z 看、Y 向上”约定。

相机物理焦距由 `fx/fy × 3.45 μm` 得到约 6.115 mm。脚本已按 USD Camera 的“十分之一场景单位”规则换算；在 `metersPerUnit=1` 时 raw focal 为约 `0.06115`。基础 Camera 保持主点居中，精确 `cx/cy` 只由 OpenCV schema 提供，防止偏移被重复应用。

## 校验

完整构建会自动运行 `validate_twin.py`。也可单独执行：

```bash
/home/zhulong/IsaacLab-3.0.0-beta2/env_isaaclab/bin/python \
  isaacSim/fr5_line_laser_twin/validate_twin.py
```

它会检查 6 个旋转关节、articulation/刚体、USD 依赖层、坐标变换方向、mm→m、相机 K/D schema、USD 相机物理单位、激光平面残差、调试面的传感器可见性、PhysicsScene 以及全部输入哈希。纯单元测试命令为：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
  /home/zhulong/IsaacLab-3.0.0-beta2/env_isaaclab/bin/python -m pytest \
  -q -p no:cacheprovider isaacSim/fr5_line_laser_twin/test_build_twin.py
```

## 当前不能当作精度真值的部分

法奥官方 FR5 URDF 用 `wrist3_link` 表示第六关节后的末端运动刚体，但没有另建固定的 `flange/tool0` frame；这在 URDF 中是允许的，并不表示模型来源不官方。控制器和手眼标定使用的是 SDK 报告的物理法兰 frame，官方 URDF 没有明确声明它与 `wrist3_link` 是否同原点、同轴。当前 `config.yaml` 暂用单位矩阵作为待验证候选，并明确写入 `verified: false`。在完成核对前，该场景可用于接口开发和可视化，不能用于毫米级仿真—实机误差结论。

完成 20–30 个姿态的 Isaac FK、ROS FK 和控制器法兰对照后，把实测 `T_wrist3_flange`（把 flange 坐标映射到 wrist3 坐标）写入 `config.yaml`，并将 `verified` 改为 `true`，然后重新运行 `build.sh`。如果控制器基座和 URDF 基座未先对齐，应联合估计基座变换，不能把基座误差吸收到末端固定变换。

激光平面不能唯一确定激光器光心和姿态，因此第一版只建立解析平面，不虚构一个物理 `RectLight` 投影器。要生成逼真的 laser-on 图像，还需实测投影器 6D 外参、线宽/发散角、功率与相机曝光响应。

Isaac Sim 6 可以读取此处写入的 OpenCV 针孔 schema，场景也已包含 1440×1080 RenderProduct；但这不等于已经创建 Replicator annotator 或持续采集任务。程序化采集时仍需在 Kit 内绑定所需 RGB/depth annotator，并确认 schema 已注册。首次用于计量前，必须用覆盖全画幅的标定板比较 RTX 像素与 `cv2.projectPoints`，建议门槛 `<0.5 px`；在完成这项渲染实测前，解析射线重建仍以 YAML 中的原始 K/D 为准。

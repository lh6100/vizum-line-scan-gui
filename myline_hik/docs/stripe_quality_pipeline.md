# 高反光条纹质量提取与点云使用说明

本文面向标定和扫描软件的实际使用者，说明 `Legacy`、`Shadow`、`Quality`
三种线激光中心策略、质量指标、离线回放和标定边界。

本文描述的是质量控制，不代表软件可以从已经严重饱和、遮挡或多径混叠的图像中
恢复真实表面。遇到无法区分的多峰时，正确行为是拒绝该段并保留缺口，而不是为了
点数完整而强制生成中心点。

## 1. 两套设备当前策略

| 设备组 | 硬件 | 当前标定状态 | 标定中心策略 | 扫描中心策略 |
| --- | --- | --- | --- | --- |
| `scanner_650` | 650 nm、Pin 7、160 万相机、6 mm 镜头 | 已有正式内参、激光平面和手眼参数 | `Legacy` | `Shadow` |
| `scanner_450` | 450 nm、Pin 11、130 万相机、12.5 mm 镜头 | 尚未完成正式标定 | `Quality` | `Quality` |

### 1.1 scanner_650：正式结果保持 Legacy

`scanner_650` 的正式激光平面是基于原有中心定义拟合得到的，因此当前扫描采用：

```text
Legacy 中心 -> 正式三维点和正式点云
Quality 中心 -> 同帧并行计算、质量统计和新旧中心偏移
```

这就是 `Shadow` 模式。Quality 提取失败或拒绝较多点时，不会悄悄替换正式的
Legacy 三维结果。Shadow 数据必须经过不同深度、材料、曝光和视场位置的离线验证，
确认不存在不可接受的系统性偏移后，才能考虑改变正式输出策略。

`scanner_650` 的标定页面仍使用 Legacy，避免后续补采数据混入另一套中心定义。

### 1.2 scanner_450：先固定 Quality 定义，再完成标定

`scanner_450` 尚未标定，可以从一开始使用 Quality 中心定义完成后续激光平面标定。
推荐顺序是：

1. 完成并批准相机内参；
2. 固定条纹方向、软件 ROI、Quality 算法版本和参数；
3. 使用同一套 Quality 中心定义采集并拟合激光平面；
4. 完成或验证手眼；
5. 使用相同方向、ROI、算法版本和参数进行扫描。

不得先用一种中心定义拟合激光平面，再在正式扫描中无记录地切换到另一种中心
定义。

## 2. 三种中心策略的含义

### Legacy

使用原有阈值、局部峰和完整半高宽灰度质心算法。它是 `scanner_650` 当前正式
激光平面所对应的中心定义。

### Shadow

同一帧同时运行 Legacy 和 Quality：

- `result.points` 和正式连续点云仍取 Legacy；
- Quality 结果保存在质量诊断和新旧中心对比中；
- 可报告匹配点数、带符号平均偏移、绝对偏移 P50/P95/max；
- Quality 不会在 Shadow 模式下静默改变正式几何。

### Quality

使用局部 Median/MAD 背景估计、多候选质量门控和允许 GAP 的全局路径。只有质量
路径通过点数、深度和三维重建门槛后才形成正式输出。无法形成可信路径时应失败
关闭，不回退到错误峰。

当前算法版本是：

```text
quality-v2-local-mad-dp-gaussian-taylor
```

每个候选会明确记录实际使用的亚像素中心方法：

- `gaussian_derivative_taylor`：用于未饱和条纹。先沿条纹法向做一维高斯平滑，
  再用平滑响应的一、二阶中心差分执行 Steger 型 Taylor 定位
  `offset = -I'/I''`。亮脊必须具有负二阶曲率，位移必须在源像素
  `±0.5 px` 内并位于半高宽双翼之间；
- `saturated_half_height_midpoint`：用于存在饱和像素的条纹。饱和平顶的导数
  不能可靠表示光学脊线，因此分别插值左右真实半高交点，再取双翼中点；
- `background_centroid`：未饱和候选无法满足 Taylor 曲率、边界或有限值条件时，
  回退到减去局部背景后的完整半高支撑区灰度质心。最终防御逻辑不会让 NaN/Inf
  进入路径优化。

这里的 `background_centroid` 是同一个候选内部的亚像素估计回退，不是 Quality
失败后切回 Legacy 正式链；候选仍必须通过全部质量和路径门槛。

这里使用的是 Steger 型的一维高斯导数/Taylor 亚像素思想，不应误称为已经实现了
任意方向、任意尺度的完整二维 Hessian Steger 算法。

算法版本可通过 `hik_stripe::algorithmVersion()` 或回放工具输出查看。算法版本、
中心方法和参数应与数据集、激光平面结果一同留档。

## 3. 固定方向和 ROI

方向和 ROI 是测量配置的一部分，不能把它们当作每帧随意调整的图像美化参数。

### scanner_650

当前固定配置是：

```text
方向：horizontal
归一化 ROI：[x=0.00, y=0.20, width=1.00, height=0.58]
```

对 `1440 × 1080` 图像，软件 ROI 对应：

```text
x = 0..1439
y = 216..842
命令行表示：--roi 0,216,1440,627
```

Quality 模式还会把该 ROI 与正式激光平面的有效深度走廊取交集。ROI 只是在原始
标定图像坐标中屏蔽候选，不裁剪或缩放图像，也不修改相机内参主点。

### scanner_450

当前未标定阶段暂为 `Auto + 全图 ROI`。在采集正式激光平面数据之前，应通过真实
图像确定条纹主方向和工作区，然后将方向和 ROI 固定。完成标定后再改变方向、ROI、
图像缩放、binning 或相机硬件 ROI，都必须重新检查图像坐标和标定一致性。

对于已经固定的设备，优先使用明确的 `Horizontal` 或 `Vertical`，避免强竖向或
横向镜面鬼线使 `Auto` 在不同帧选择不同方向。

## 4. Quality 指标

Quality 会为候选中心记录以下指标：

| 字段 | 含义 | 一般判读 |
| --- | --- | --- |
| `rawPeak` | 原始图像峰值 | 接近 255 时检查饱和 |
| `responsePeak` | 背景抑制或开关差分后的峰值 | 只代表响应强度，不等于可信度 |
| `localBaseline` | 候选附近的局部中位背景 | 用于计算局部峰突出度 |
| `localNoiseMad` | 局部 MAD 换算的鲁棒噪声 | 越小越稳定 |
| `prominence` | 峰值减局部背景 | 必须高于最小突出度和噪声门槛 |
| `snr` | `prominence / localNoiseMad` | 越大越好，但高 SNR 鬼线仍可能是假线 |
| `fwhmPx` | 局部半高全宽 | 必须在允许宽度范围内 |
| `saturatedFraction` | 半高支撑区内的饱和比例 | 越大越需要警惕中心偏移 |
| `saturatedPlateauWidthPx` | 连续饱和平顶宽度 | 宽平顶会失去可靠中心信息 |
| `secondPeakRatio` | 次峰与主峰突出度之比 | 越接近 1，多峰歧义越强 |
| `gradientAsymmetry` | 中心左右能量不对称度 | 越小越接近对称条纹 |
| `fitResidual` | 中心两侧轮廓残差 | 越小越可信 |
| `quality` | 综合质量分数，范围约为 0–1 | 越大越好，不能只看这一项 |
| `centerSigmaPx` | 根据噪声、饱和和不对称估计的中心不确定度 | 越小越好，远距离还应换算为三维误差 |
| `centerMethod` / `center_method` | 该候选实际采用的亚像素中心方法 | 应区分 Taylor、饱和双翼中点和质心回退 |
| `taylorOffsetPx` / `taylor_offset_px` | Taylor 中心相对整数峰的亚像素位移 | Taylor 方法必须在 `±0.5 px` 内；其他方法通常为 0 |
| `smoothedFirstDerivative` | 高斯平滑响应在整数峰处的一阶导数 | 与二阶导数共同决定 Taylor 位移 |
| `smoothedSecondDerivative` | 高斯平滑响应在整数峰处的二阶导数 | 未饱和亮脊的有效 Taylor 定位应为负值 |
| `rejectFlags` | 一个候选可同时具有多个拒绝原因 | 应保留位标志，不要只保存第一个原因 |

路径级诊断还包括：

- 扫描线总数、有候选扫描线数和候选总数；
- 单点质量通过数、最终路径点数和 GAP 数；
- 饱和候选比例、最终路径饱和比例和多峰扫描线比例；
- 最佳/次佳路径代价；
- `pathCostMarginPerPoint`：最佳与次佳路径的单位点代价差，过小表示路径歧义；
- 最终选中点的平均 `quality`、FWHM、SNR、梯度不对称度、拟合残差和次峰比。

亮度不是置信度。镜面鬼线可能比真实激光线更亮，必须结合宽度、饱和、对称性、
多峰和路径连续性判断。

## 5. 拒绝原因

| 拒绝标志 | 含义 |
| --- | --- |
| `LOW_PROMINENCE` | 相对局部背景或局部噪声不够突出 |
| `WIDTH_OUT_OF_RANGE` | 半高宽过窄、过宽或触及有效区边界 |
| `SATURATED_WIDE_PLATEAU` | 饱和比例或连续饱和平顶宽度超限 |
| `SATURATED_ASYMMETRIC` | 饱和条纹左右明显不对称 |
| `MULTI_PEAK_AMBIGUOUS` | 同一扫描线存在强度接近的竞争峰 |
| `PROFILE_ASYMMETRIC` | 未饱和轮廓左右明显不对称 |
| `FIT_RESIDUAL_HIGH` | 中心两侧轮廓残差过大 |
| `QUALITY_LOW` | 综合质量分低于门槛 |
| `OUTSIDE_ROI` | 候选不在固定软件 ROI 内 |
| `OUTSIDE_VALIDITY_MASK` | 候选不在激光平面有效深度走廊等有效掩膜内 |
| `PATH_JUMP` | 与允许的路径步长或连续性不符 |
| `PATH_AMBIGUOUS` | 最佳和次佳全局路径不足以可靠区分 |

一个候选可能同时满足多项拒绝条件。质量分析应统计各标志出现次数和组合，不能把
所有失败统一写成“低置信度”。

## 6. 点云文件的含义

保存链明确分为“正式链”和“质量并行链”。质量文件使用独立文件名，不会覆盖
`raw/voxel` 正式文件。

### 6.1 停稳/单点扫描输出

| 文件 | 含义 |
| --- | --- |
| `scan_raw.ply` | 按设备当前正式中心策略累计的逐 profile 三维点，未体素化 |
| `scan_voxel.ply` | `scan_raw.ply` 对应正式点的体素降采样结果 |
| `scan_quality_optical.ply` | 通过 Quality 图像层中心提取和三维有效性检查、尚未进行相邻 profile 支持过滤的质量点 |
| `scan_quality_filtered.ply` | `scan_quality_optical.ply` 中获得足够相邻 profile 支持的保留点 |
| `scan_quality_rejected.ply` | 相邻支持不足或三维坐标无效而被点云层拒绝的质量点 |
| `scan_quality_voxel.ply` | 对 `scan_quality_filtered.ply` 进行置信度加权体素降采样的结果 |

多位置停稳扫描会对质量点执行相邻 profile 支持分类。单点验证没有相邻 profile，
因此该过滤不启用，光学质量点全部进入 filtered；软件仍保存同名质量文件，使下游
工具获得稳定的文件契约。

### 6.2 连续扫描输出

| 文件 | 含义 |
| --- | --- |
| `continuous_raw.ply` | 各有效同步帧按当前正式中心策略重建并变换到 `base_link` 后的逐轮廓累计点 |
| `continuous_voxel.ply` | `continuous_raw.ply` 对应正式点的体素降采样结果 |
| `continuous_quality_optical.ply` | 连续帧中通过 Quality 光学门控和三维有效性检查、尚未进行相邻支持过滤的点 |
| `continuous_quality_filtered.ply` | 获得足够相邻 profile 支持的质量保留点 |
| `continuous_quality_rejected.ply` | 相邻支持不足或三维坐标无效的质量拒绝点 |
| `continuous_quality_voxel.ply` | 对 filtered 质量点进行置信度加权体素降采样的结果 |

在 `Shadow` 或 `Quality` 启用且存在有效质量点时，连续管线会写出质量
optical/filtered/rejected/voxel 文件，并在
`continuous_reconstruction.csv` 与
`continuous_reconstruction_summary.json` 中记录帧级质量指标、过滤参数、点数、
保存状态和路径。连续模式中某一分类点集为空时，对应 PLY 可以不生成；应以 summary
中的 `*_point_count`、`*_ply_saved` 和 `*_ply_path` 判断，不要仅靠文件是否存在
猜测处理状态。

### 6.3 正式链保护

- `scanner_650` 在 Shadow 阶段的 `scan_raw/scan_voxel` 和
  `continuous_raw/continuous_voxel` 仍使用 Legacy 正式中心；
- 同一会话中的 `*_quality_*` 文件只用于并行比较和质量验收，不会替换上述正式
  文件；
- `scanner_450` 完成标定后处于 Quality 模式，此时正式 raw 链本身使用 Quality
  中心，但质量过滤前、过滤后和拒绝点仍用独立文件保存；
- 名称中的 `raw` 指“重建点云未体素化”，不是相机原始图像。它已经经过当前正式
  中心策略、射线/激光平面求交、有效深度和同步有效性检查。

精度分析和新旧算法对比应同时追溯原始 PNG、候选 CSV、正式 raw 点云和
`*_quality_optical.ply`，不能只比较两个体素点云。

### 6.4 optical、filtered、rejected 和 voxel

- `optical`：Quality 中心路径通过图像层门控并成功重建的点；这些点带有
  `CLOUD_QUALITY_OPTICAL_ACCEPTED`；
- `filtered`：在 optical 点的基础上，通过相邻 profile 支持过滤的 `kept` 点；
- `rejected`：相邻支持不足或坐标无效的质量点，保留它们便于检查误杀和调整参数；
- `quality_voxel`：对 filtered 点进行置信度加权体素化，用于显示和后续轻量处理。

普通正式 `voxel` 和质量 `quality_voxel` 都属于降采样结果。体素降采样本身不是
高反光质量过滤；连续且稠密的假面同样会被体素化保留下来。

### 6.5 相邻 profile 支持过滤的边界

相邻轮廓支持分类使用三维半径、最少支持 profile 数和最大 profile 间隔：

- `kept`：在指定半径和 profile 间隔内，得到足够多其他轮廓支持；
- `rejected`：支持不足或三维坐标无效；
- 对应质量位包括 `CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED`、
  `CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT` 和
  `CLOUD_QUALITY_REJECTED_INVALID_BASE_POINT`。

最重要的边界是：

> 相邻帧支持只能有效删除孤立或短暂错误点；如果镜面多径产生了一张在相邻帧中
> 连续、稠密且位置一致的假面，它也会获得相邻 profile 支持并通过该过滤。

这种假面必须依靠图像层多峰/饱和/路径歧义、不同观察角度、光学抑制、已知 CAD
包络或其他独立证据识别。普通半径离群点和体素降采样不能解决这一问题。

PLY 新增字段位于原有字段之后：

```text
quality_flags
observation_count
```

`quality_flags` 是位组合：

| 数值 | 标志 |
| ---: | --- |
| `1` | `CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED` |
| `2` | `CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT` |
| `4` | `CLOUD_QUALITY_REJECTED_INVALID_BASE_POINT` |
| `8` | `CLOUD_QUALITY_VOXEL_AGGREGATED` |
| `16` | `CLOUD_QUALITY_OPTICAL_ACCEPTED` |

`quality_flags = 0` 只表示尚未被这些点云阶段分类，不能解释为“已经证明是真实
表面”。

`observation_count` 不是“相邻支持 profile 数”；相邻支持数量应从过滤统计单独读取。

## 7. ReplayStripeQuality 离线回放

### 7.1 构建

```bash
cmake --build build --target ReplayStripeQuality -j2
```

### 7.2 回放 scanner_650 单帧

```bash
./build/ReplayStripeQuality \
  --input data/scan/scanner_650/<session>/images/frame_000000000170.png \
  --output data/analysis/stripe_quality/frame_170 \
  --orientation horizontal \
  --roi 0,216,1440,627
```

### 7.3 回放整个图像目录

```bash
./build/ReplayStripeQuality \
  --input data/scan/scanner_650/<session>/images \
  --output data/analysis/stripe_quality/<session> \
  --orientation horizontal \
  --roi 0,216,1440,627
```

工具只读输入 PNG，所有结果写入显式指定的输出目录。建议输出目录与原始 session
分开，禁止用脚本覆盖或重命名原始图片。

默认使用与常亮单帧扫描一致的形态学背景抑制：

```text
--response-mode morphology
--background-width 31
--background-height 3
--minimum-raw 60
```

如果输入本身已经是预先计算的响应图，才使用：

```text
--response-mode identity
```

其他选项：

```text
--orientation auto|horizontal|vertical
--roi x,y,width,height
--no-overlay
```

### 7.4 输出内容

```text
<output>/
├── stripe_quality_summary.csv
├── csv/
│   └── 000000_<image>_quality.csv
└── overlay/
    └── 000000_<image>_overlay.png
```

汇总 CSV 包括：

- Legacy/Quality 点数和匹配点数；
- 新旧中心全量带符号均值、绝对偏移 P50/P95/max；
- 基于 signed median/MAD 门槛的 robust 匹配数、gross 换峰数、robust signed
  median/mean 和实际门槛；
- 候选总数和拒绝数；
- 饱和候选、多峰扫描线和最终路径饱和比例；
- 最终选中点的平均质量、FWHM、SNR、梯度不对称度、拟合残差和次峰比；
- GAP、路径代价差和各拒绝原因次数；
- Legacy/Quality 错误信息。

对应的 selected 均值列为：

```text
mean_selected_quality
mean_selected_fwhm_px
mean_selected_snr
mean_selected_gradient_asymmetry
mean_selected_fit_residual
mean_selected_second_peak_ratio
```

逐候选 CSV 保存像素中心、全部质量字段、是否进入最终路径、拒绝位和与 Legacy
中心的偏移。v2 还会输出：

```text
center_method
taylor_offset_px
smoothed_first_derivative
smoothed_second_derivative
```

`center_method` 的可见值是：

```text
gaussian_derivative_taylor
saturated_half_height_midpoint
background_centroid
```

检查这些字段可以确认一个点使用了未饱和 Taylor 中心、饱和双翼中点还是质心
回退，不能只根据最终坐标猜测方法。

叠加图颜色：

```text
绿色：Legacy
黄色：Quality 最终路径
红色：被质量门控拒绝的候选
蓝色矩形：软件 ROI
```

不要只看输出点数。正式评估至少同时比较：

```text
有效覆盖率
候选拒绝率
饱和比例
多峰比例
新旧偏移 P50/P95/max
标准件三维误差
错误假面面积
```

## 8. 系统性像素偏移和重新拟合边界

`scanner_650` 从 Legacy 切换到 Quality 前，必须对 Shadow 结果检查：

1. 在多个深度、视场位置、材料、曝光和入射角统计匹配点；
2. 同时查看 robust signed median/mean、gross 换峰数和绝对偏移 P50/P95/max；
   全量 signed mean 会被少量远处反光换峰严重拉偏，不能单独用于判断系统误差；
3. 在原图叠加图中确认偏移来自中心定义，而不是错误峰匹配；
4. 把像素偏移通过当前三角测量 Jacobian 换算为深度误差；
5. 使用独立平板、台阶和长度标准验证三维结果。

如果 Quality 相对 Legacy 在正常、非反光条纹上表现为稳定的系统性偏移，旧激光
平面与新中心定义可能不再完全匹配。此时应使用固定后的 Quality 算法重新计算或
重新采集激光平面数据，并优先只重拟合激光平面。

如果偏移只发生在多峰、饱和或镜面区域，不能通过重拟合激光平面把错误峰“吸收”
进标定参数；应调整曝光、质量门槛、ROI、观察角度或直接拒绝这些区域。

在以下条件未改变时，不应无依据重做相机内参或手眼：

- 相机、镜头、焦距、对焦、分辨率和图像坐标没有变化；
- 相机与末端安装没有移动；
- 相机序列号、内参哈希和坐标系定义一致。

中心算法改变会直接影响激光平面拟合，但不会自动改变相机针孔模型或
`T_flange_camera`。因此“硬件未动”时的默认处理是：

```text
内参：保持并验证
手眼：保持并验证
激光平面：根据新旧中心系统偏移决定是否重拟合
```

只有独立验证明确指出内参或手眼不合格，或者相机、镜头、焦点、分辨率、坐标裁剪
或安装发生变化时，才进入相应的重新标定流程。

## 9. 建议的启用检查表

### scanner_650 从 Shadow 评估到 Quality

- [ ] 固定 Horizontal 和正式 ROI；
- [ ] 不同深度、材料和曝光都有 Shadow 数据；
- [ ] 原始 PNG、候选 CSV、叠加图和算法版本可追溯；
- [ ] 饱和、多峰、GAP 和各拒绝原因分别统计；
- [ ] 新旧中心系统偏移已经解释；
- [ ] 必要时只重拟合并批准新的激光平面；
- [ ] 内参和手眼在硬件未动时只验证、不盲目重做；
- [ ] 标准平板、台阶、长度和高反光错误率达到验收要求；
- [ ] Quality 覆盖不足时保留缺口，不通过放宽门槛制造假面。

### scanner_450 正式标定前

- [ ] 确定并固定条纹方向；
- [ ] 确定并固定软件 ROI；
- [ ] 固定 Quality 算法版本和参数；
- [ ] 使用同一中心定义完成激光平面采集和拟合；
- [ ] 标定文件记录设备、相机序列号、算法版本和数据集；
- [ ] 标定完成后扫描侧不再切换回另一套中心定义。

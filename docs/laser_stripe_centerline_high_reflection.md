# 高反光场景下激光线中心提取问题总结

本文档针对当前 Vizum 左/右目激光线图像中的两个问题：

- 高反光区域会把中心点识别拉偏。
- 下方弱激光线区域出现漏检、断线。

当前测试数据来自：

```text
data/laser_feature_once_20260708_164803/left.png
data/laser_feature_once_20260708_164803/right.png
```

当前 demo 输出：

```text
left_bright_points.png
right_bright_points.png
left_bright_points.csv
right_bright_points.csv
bright_points_summary.txt
```

## 当前现象

当前实现已经从“直线拟合”改为“激光中心点集提取”，并增加了连通域过滤和行间连续跟踪。重算当前样例后，诊断信息如下：

```text
left:  raw_candidate_rows=1675, component_count=58, kept_component_count=5, point_count=1564
right: raw_candidate_rows=1508, component_count=43, kept_component_count=3, point_count=1393
```

这说明算法已经删除了大量短小反光连通域，但仍存在两个局限：

1. 如果高反光区域和主激光条纹在图像上连成同一个连通域，单纯连通域过滤无法把它们分开。
2. 如果下方激光线亮度低、宽度窄、接近阈值，固定阈值会导致漏检。

## 为什么当前方法会失败

当前方法本质是：

1. 根据灰度阈值提取亮像素。
2. 做连通域过滤。
3. 在保留连通域中逐行计算灰度重心。

这种方法对清晰、亮度稳定、背景较暗的激光线有效，但对焊缝/金属表面会遇到问题：

- **高反光不是普通噪声**：反光区域可能比激光线更亮，也可能和激光线连接在一起。
- **弱激光和过曝同时存在**：同一张图里既有饱和高亮，也有低亮度细线，单一曝光和单一阈值很难兼顾。
- **条纹截面不是稳定高斯形状**：反光会让截面变成多峰、偏峰、截顶峰，灰度重心和最大值法都会偏。

## 常见激光线中心提取算法

### 1. 极值法

每一行取最大灰度点：

```text
x = argmax I(x, y)
```

优点：

- 实现简单。
- 速度最快。

缺点：

- 对高反光最敏感。
- 反光点比激光线亮时会直接选错。
- 只能得到像素级或近似亚像素结果。

结论：不适合当前高反光焊接场景。

### 2. 灰度重心法

在条纹宽度方向计算加权中心：

```text
x_center = sum(x * I(x, y)) / sum(I(x, y))
```

优点：

- 实现简单。
- 可得到亚像素中心。
- 对普通噪声比极值法稳定。

缺点：

- 条纹附近有高反光时，重心会被拉偏。
- 条纹截面多峰时会输出错误中心。
- 需要较可靠的条纹区域分割。

结论：可作为基础方法，但必须配合 ROI、局部窗口、动态路径约束。

### 3. 改进灰度重心 / 梯度加权重心

先粗定位中心，再根据局部灰度、边界、梯度等信息确定有效条纹区域，最后加权求中心。

优点：

- 比普通灰度重心更抗反光。
- 适合金属表面散射噪声。
- 工程实现难度中等。

缺点：

- 参数较多。
- 如果粗定位被反光带偏，后续仍会错。

结论：适合当前项目作为下一步算法升级方案。

### 4. 高斯拟合 / 抛物线拟合

对每一行的条纹截面拟合高斯或二次曲线，取峰值中心。

优点：

- 对理想高斯条纹精度高。
- 可输出亚像素峰值。

缺点：

- 高反光会导致截面多峰或截顶，拟合会失败。
- 需要先正确选中条纹窗口。

结论：适合弱反光、条纹形状较稳定的区域；不建议单独用于当前场景。

### 5. Steger / Hessian 脊线检测

使用高斯二阶导数或 Hessian 矩阵寻找亮条纹的脊线中心。它不是找最亮点，而是找局部亮条纹的几何中心。

优点：

- 经典亚像素线中心算法。
- 适合弯曲激光线。
- 比逐行最大值和普通重心法更合理。

缺点：

- 对噪声、反光和饱和仍敏感。
- 可能产生冗余中心和漏检，需要路径跟踪/聚类/连通性后处理。
- 参数包括高斯尺度、响应阈值、非极大值抑制等。

结论：适合作为当前项目的主算法候选，但必须配合路径选择。

### 6. 骨架细化

先二值分割激光区域，再做 skeleton/thinning 得到中心线。

优点：

- 可处理弯曲条纹。
- 对连通性直观。

缺点：

- 依赖二值分割质量。
- 通常是像素级，亚像素精度需要后续 refine。
- 分叉和反光连接会导致骨架错误。

结论：可做可视化或辅助路径，不建议作为最终亚像素中心方法。

### 7. 多曝光 / HDR 融合

对同一位置采集多张不同曝光图片，将低曝光中的高反光区域和高曝光中的弱线区域融合。

优点：

- 从采集源头解决“高反光 + 弱线”同时存在的问题。
- 能减少饱和截顶和暗部漏检。
- 与 Steger、梯度加权重心、高斯拟合都可以组合。

缺点：

- 需要相机和机器人/工件在多曝光采集期间保持相对静止，或保证同步。
- 动态扫描时需要处理运动带来的时间差。

结论：这是当前场景最值得优先验证的方向。

## 对高反光场景更好的组合

### 推荐方案 A：多曝光融合 + 梯度加权重心

采集多张曝光，例如：

```text
exposure = 260, 900, 3000, 9000
```

融合原则：

- 饱和像素降低权重。
- 过暗区域降低权重。
- 条纹截面更平滑、更接近单峰的区域提高权重。

融合后再进行：

```text
局部 ROI -> 梯度加权中心 -> 连续路径跟踪
```

优点：

- 工程实现相对可控。
- 对当前高反光和弱线漏检都有效。

### 推荐方案 B：Steger/Hessian 候选点 + 动态路径选择

流程：

```text
图像预处理
-> Hessian/Steger 提取亚像素候选点
-> 按行/按列组织候选点
-> 动态规划/Viterbi 选择最连续路径
-> 输出主激光中心线
```

候选点评分可以包括：

- Hessian 响应强度。
- 与上一行中心的距离。
- 局部宽度是否合理。
- 是否饱和。
- 局部梯度是否对称。
- 与预测方向是否一致。

优点：

- 比当前连通域方法更适合弯曲线。
- 能处理每行多个候选点。

缺点：

- 实现复杂度高于灰度重心。
- 参数需要用真实数据调试。

### 推荐方案 C：深度学习分割 + 传统亚像素中心

流程：

```text
U-Net/轻量分割网络 -> 激光条纹 mask -> Steger/重心/拟合
```

优点：

- 对复杂反光和纹理最有潜力。

缺点：

- 需要标注数据。
- 当前项目阶段成本较高。

结论：先不建议作为第一步。

## 当前项目的建议路线

### 第一阶段：改成多候选 + 动态路径

不要每行只取一个最亮点。应改为：

```text
每行找多个局部峰值候选
-> 对每个候选计算亚像素中心
-> 用动态规划选择整条最连续路径
```

每个候选点记录：

```text
row
x_subpixel
peak_intensity
width_px
area
saturation_ratio
gradient_symmetry
score
```

路径代价：

```text
cost = -candidate_score
       + lambda_dx * abs(x_i - x_{i-1})
       + lambda_ddx * abs((x_i - x_{i-1}) - (x_{i-1} - x_{i-2}))
       + saturation_penalty
```

这样高反光横向区域即使很亮，也会因为路径不连续、宽度异常、梯度不对称而被排除。

### 第二阶段：引入多曝光

如果相机和 SDK 支持快速切换曝光，建议采集：

```text
low exposure:  260 / 500
mid exposure:  900 / 1500
high exposure: 3000 / 9000
```

输出：

```text
left_exp260.png
left_exp900.png
left_exp3000.png
left_fused.png
left_centerline.csv
```

融合后再用第一阶段路径算法。

### 第三阶段：Steger/Hessian 替换局部峰值

在动态路径框架稳定后，将候选点来源从“局部峰值 + 重心”升级为 Steger/Hessian。

保留动态路径选择，因为 Steger 在高反光处仍可能产生多余候选。

## 和当前代码的关系

当前代码位置：

```text
tests/vizum_dynamic_profile_once_demo.cpp
```

当前核心函数：

```cpp
ImageCenterlineResult extractBrightCenterlineFromImage(const ImageCopy& image)
```

当前输出：

```text
left_bright_points.csv
right_bright_points.csv
bright_points_summary.txt
```

下一步不建议继续只调这些参数：

```text
threshold
component height
component width
```

因为当前问题不是单纯参数问题，而是需要从“单点选择”升级到“多候选路径选择”。

## 推荐实现优先级

1. 保留当前输出文件，新增候选点调试图：

```text
left_candidates.png
left_selected_path.png
left_centerline_candidates.csv
```

2. 实现每行多峰候选提取。

3. 实现动态规划主路径选择。

4. 将选中路径的中心点输出到现有：

```text
left_bright_points.csv
right_bright_points.csv
```

5. 再实现多曝光采集和融合。

## 参考资料

- Xu et al., "Line structured light calibration method and centerline extraction: A review", Results in Physics, 2020.
  该综述将中心线提取方法归为灰度重心、Steger、骨架提取等大类。
  https://doi.org/10.1016/j.rinp.2020.103637

- An et al., "A modified multi-exposure fusion method for laser measurement of specular surfaces", Optics Communications, 2023.
  该文指出金属/镜面表面会同时出现过曝高亮、伪影和欠曝漏检，并提出多曝光融合改善动态范围。
  https://doi.org/10.1016/j.optcom.2023.129627

- Song et al., "Center extraction method for reflected metallic surface fringes based on line structured light", JOSA A, 2024.
  该文针对金属表面散射噪声，使用自适应阈值、灰度重心粗定位和梯度加权做亚像素中心提取。
  https://opg.optica.org/josaa/abstract.cfm?uri=josaa-41-3-550

- Su et al., "A real-time laser stripe center extraction method for line-structured light system based on FPGA", Journal of Measurement Science and Instrumentation, 2023.
  该文讨论 Hessian 方法可能出现冗余中心和漏检，并通过非极大值抑制等方式增强实时提取。
  https://doi.org/10.62756/jmsi.1674-8042.2023050

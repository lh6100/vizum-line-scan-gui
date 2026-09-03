# 传感器夹具模型

`sensor_rig_macro.xacro` 只描述夹具的保守碰撞包络和命名约定。生产部署必须用实测/CAD
安装尺寸覆盖宏参数，再与 FR5 URDF 合并。相机光学外参以激活的 CalibrationPackage 为准，
不能把这里的名义尺寸当作标定结果。

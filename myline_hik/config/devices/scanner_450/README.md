# scanner_450 正式标定目录

此目录只属于 `scanner_450`（450 nm、物理 Pin 11、130 万相机、
12.5 mm 镜头）。

当前设备尚未标定，因此这里故意不放置占位 YAML。请在标定 GUI 的
`scanner_450` 页面依次完成并批准：

1. `hik_intrinsics.yaml`
2. `hik_laser_plane.yaml`
3. `hik_handeye.yaml`

连续扫描还需要根据这台 130 万相机的实测帧率、曝光、写盘能力和工作速度单独创建
`synchronization.yaml`。同步文件只控制连续模式：三份正式标定齐备后，即使该文件
缺失也仍可做单点/停稳扫描验证，但 450 nm 连续同步按钮会禁用；三份正式标定本身
缺失时，标定 GUI 可继续完成标定，扫描 GUI 则禁止任何三维建图。不要让它回退使用
160 万相机的
`config/synchronization.yaml`。

扫描程序在三份文件齐备、哈希依赖链一致、相机 frame 与实机身份校验通过前，
会禁止本设备组生成三维点云。不得复制或软链接
`scanner_650` 的 `config/hik_*.yaml` 到此目录。

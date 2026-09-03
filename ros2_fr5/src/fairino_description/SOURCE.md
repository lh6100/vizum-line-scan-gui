# fairino_description Source

This package is a minimal FR5 model subset copied from Fairino's public ROS2 repository.

- Source repository: https://github.com/FAIR-INNOVATION/frcobot_ros2
- Source snapshot commit: `8bc2fa21eab848da9637b027938df8f6b81d3db2`
- Source paths:
  - `fairino_description/CMakeLists.txt`
  - `fairino_description/package.xml`
  - `fairino_description/urdf/fairino5_v6.urdf`
  - `fairino_description/meshes/fairino5_v6/`

Only the standard `fairino5_v6` FR5 URDF and meshes were copied. Other Fairino robot models and MoveIt/hardware-control packages were intentionally left out so this workspace can start with RViz visualization without pulling in unrelated dependencies.

## Local extensions

The local URDF is intentionally no longer byte-for-byte identical to the
upstream file:

- the upstream `wrist2_link` collision typo `<origins>` was corrected to
  `<origin>`;
- a fixed `wrist3_link -> fairino_flange_model` frame was added at
  `xyz="0 0 0.1"`, based on cross-checking recorded joint states against the
  controller-reported flange pose;
- the `scanner_650` and `scanner_450` optical frames were added under
  `fairino_flange_model` using the project eye-in-hand `T_flange_camera`
  calibrations in `myline_hik/config/devices/`.

These added frame definitions are project calibration data, not dimensions or
frames claimed by the upstream Fairino model. Keep `fairino_flange_model`
distinct from the runtime `fairino_flange_reported` frame published by the
controller driver.

The upstream `package.xml` currently declares placeholder license metadata. Check Fairino's repository or vendor documentation before redistributing these model files outside this project.

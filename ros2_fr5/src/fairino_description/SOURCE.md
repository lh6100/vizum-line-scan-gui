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

The upstream `package.xml` currently declares placeholder license metadata. Check Fairino's repository or vendor documentation before redistributing these model files outside this project.

cmake_minimum_required(VERSION 3.8)
project(welding_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArtifactRef.msg"
  "msg/PlanDependencies.msg"
  "msg/TaskState.msg"
  "msg/RobotFeedback.msg"
  "msg/LaserSafetyState.msg"
  "srv/CreateSession.srv"
  "srv/FinalizeSession.srv"
  "srv/ApproveTrajectory.srv"
  "srv/ActivateArtifact.srv"
  "srv/GetActiveArtifact.srv"
  "srv/CreatePlanningSceneVersion.srv"
  "srv/ValidatePlanDependencies.srv"
  "srv/GuardedLaserMode.srv"
  "srv/ControlTask.srv"
  "action/ExecuteMotion.action"
  "action/RunAutomaticCalibration.action"
  "action/BuildCoarseMap.action"
  "action/RunScanToWeld.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

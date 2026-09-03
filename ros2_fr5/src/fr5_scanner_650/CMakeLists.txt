cmake_minimum_required(VERSION 3.8)
project(fr5_scanner_650)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(MYLINE_HIK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../myline_hik"
  CACHE PATH "Path to the calibrated myline_hik source tree")
set(LEGACY_SOURCE_ROOT "${MYLINE_HIK_ROOT}/src")
if(NOT EXISTS "${LEGACY_SOURCE_ROOT}/HikCalibrationCore.cpp")
  message(FATAL_ERROR "MYLINE_HIK_ROOT is invalid: ${MYLINE_HIK_ROOT}")
endif()

add_library(scanner_650_calibration_core
  "${LEGACY_SOURCE_ROOT}/HikCalibrationCore.cpp"
  "${LEGACY_SOURCE_ROOT}/HikCalibrationCore.h"
  "${LEGACY_SOURCE_ROOT}/StripeCenterlineExtractor.cpp"
  "${LEGACY_SOURCE_ROOT}/StripeCenterlineExtractor.h")
target_include_directories(scanner_650_calibration_core PUBLIC
  "${LEGACY_SOURCE_ROOT}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(scanner_650_calibration_core PUBLIC ${OpenCV_LIBS})

add_library(world_height_color
  src/world_height_color.cpp
  src/world_height_color.hpp)
target_include_directories(world_height_color PUBLIC src)

add_library(fixed_axis_scan_core
  src/fixed_axis_scan_core.cpp
  src/fixed_axis_scan_core.hpp)
target_include_directories(fixed_axis_scan_core PUBLIC src)

add_executable(line_laser_reconstruction_node
  src/line_laser_reconstruction_node.cpp)
target_link_libraries(line_laser_reconstruction_node
  scanner_650_calibration_core world_height_color ${OpenCV_LIBS})
ament_target_dependencies(line_laser_reconstruction_node
  cv_bridge geometry_msgs rclcpp sensor_msgs std_msgs std_srvs tf2
  tf2_geometry_msgs tf2_ros visualization_msgs)

add_executable(scan_motion_commander src/scan_motion_commander.cpp)
target_link_libraries(scan_motion_commander fixed_axis_scan_core)
ament_target_dependencies(scan_motion_commander
  control_msgs geometry_msgs moveit_ros_planning_interface rclcpp rclcpp_action
  std_srvs tf2 tf2_geometry_msgs)

add_executable(table_collision_scene_node src/table_collision_scene_node.cpp)
ament_target_dependencies(table_collision_scene_node
  geometry_msgs moveit_msgs rclcpp shape_msgs std_msgs)

add_library(workspace_coarse_scan_core
  src/workspace_coarse_scan_core.cpp
  src/workspace_coarse_scan_core.hpp)
target_include_directories(workspace_coarse_scan_core PUBLIC src)

add_library(workspace_return_core
  src/workspace_return_core.cpp
  src/workspace_return_core.hpp)
target_include_directories(workspace_return_core PUBLIC src)
ament_target_dependencies(workspace_return_core trajectory_msgs)

add_library(workspace_robot_state_utils
  src/workspace_robot_state_utils.cpp
  src/workspace_robot_state_utils.hpp)
target_include_directories(workspace_robot_state_utils PUBLIC src)
target_link_libraries(workspace_robot_state_utils PUBLIC
  Eigen3::Eigen moveit_core::moveit_robot_state)

add_executable(workspace_coarse_scan_planner
  src/workspace_coarse_scan_planner.cpp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(workspace_coarse_scan_planner PRIVATE
    -g -fno-omit-frame-pointer)
  target_link_options(workspace_coarse_scan_planner PRIVATE -rdynamic)
endif()
target_link_libraries(workspace_coarse_scan_planner
  workspace_coarse_scan_core workspace_robot_state_utils
  Eigen3::Eigen)
ament_target_dependencies(workspace_coarse_scan_planner
  control_msgs geometry_msgs moveit_msgs moveit_ros_planning
  moveit_ros_planning_interface rclcpp rclcpp_action shape_msgs std_msgs
  std_srvs visualization_msgs)

add_executable(rviz_scan_trajectory_relay src/rviz_scan_trajectory_relay.cpp)
ament_target_dependencies(rviz_scan_trajectory_relay
  control_msgs rclcpp rclcpp_action std_srvs)

add_executable(scanner_control_gui src/scanner_control_gui.cpp)
target_link_libraries(scanner_control_gui Qt5::Core Qt5::Gui Qt5::Widgets)
ament_target_dependencies(scanner_control_gui
  rcl_interfaces rclcpp sensor_msgs std_msgs std_srvs)

add_library(scanner_650_laser_backend
  "${LEGACY_SOURCE_ROOT}/LineLaserController.cpp"
  "${LEGACY_SOURCE_ROOT}/LineLaserController.h")
target_include_directories(scanner_650_laser_backend PUBLIC "${LEGACY_SOURCE_ROOT}")
target_link_libraries(scanner_650_laser_backend PUBLIC Qt5::Core pthread)
add_executable(line_laser_control_node src/line_laser_control_node.cpp)
target_link_libraries(line_laser_control_node scanner_650_laser_backend Qt5::Core)
ament_target_dependencies(line_laser_control_node rclcpp std_msgs std_srvs)

set(DEFAULT_HIK_MVS_ROOT "/opt/MVS")
if(DEFINED ENV{HIK_MVS_ROOT} AND NOT "$ENV{HIK_MVS_ROOT}" STREQUAL "")
  set(DEFAULT_HIK_MVS_ROOT "$ENV{HIK_MVS_ROOT}")
endif()
set(HIK_MVS_ROOT "${DEFAULT_HIK_MVS_ROOT}" CACHE PATH
  "Hikrobot MVS SDK installation directory")
find_path(HIK_MVS_INCLUDE_DIR MvCameraControl.h
  PATHS "${HIK_MVS_ROOT}/include" NO_DEFAULT_PATH)
find_library(HIK_MVS_LIBRARY MvCameraControl
  PATHS "${HIK_MVS_ROOT}/lib/64" NO_DEFAULT_PATH)
if(HIK_MVS_INCLUDE_DIR AND HIK_MVS_LIBRARY)
  add_library(scanner_650_camera_backend
    "${LEGACY_SOURCE_ROOT}/HikSynchronizationCore.cpp"
    "${LEGACY_SOURCE_ROOT}/HikSynchronizationCore.h"
    "${LEGACY_SOURCE_ROOT}/HikCameraWorker.cpp"
    "${LEGACY_SOURCE_ROOT}/HikCameraWorker.h")
  target_compile_definitions(scanner_650_camera_backend PUBLIC HAVE_HIK_MVS=1)
  target_include_directories(scanner_650_camera_backend PUBLIC
    "${LEGACY_SOURCE_ROOT}" ${HIK_MVS_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(scanner_650_camera_backend PUBLIC
    Qt5::Core Qt5::Gui Eigen3::Eigen ${OpenCV_LIBS} ${HIK_MVS_LIBRARY} pthread)

  add_executable(hik_single_camera_node src/hik_single_camera_node.cpp)
  target_link_libraries(hik_single_camera_node
    scanner_650_camera_backend scanner_650_calibration_core Qt5::Core Qt5::Gui)
  ament_target_dependencies(hik_single_camera_node rclcpp sensor_msgs std_msgs std_srvs)
  set_target_properties(hik_single_camera_node PROPERTIES
    BUILD_RPATH "${HIK_MVS_ROOT}/lib/64"
    INSTALL_RPATH "${HIK_MVS_ROOT}/lib/64")
  install(TARGETS scanner_650_camera_backend hik_single_camera_node
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "Hikrobot MVS SDK not found; hik_single_camera_node will not be built")
endif()

install(TARGETS scanner_650_calibration_core scanner_650_laser_backend
  fixed_axis_scan_core
  workspace_coarse_scan_core workspace_return_core workspace_robot_state_utils
  world_height_color
  line_laser_reconstruction_node line_laser_control_node
  scan_motion_commander table_collision_scene_node workspace_coarse_scan_planner
  rviz_scan_trajectory_relay scanner_control_gui
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  "${MYLINE_HIK_ROOT}/config/devices/scanner_650/"
  DESTINATION share/${PROJECT_NAME}/config/calibration/scanner_650
  FILES_MATCHING PATTERN "*.yaml")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(workspace_coarse_scan_core_test
    test/test_workspace_coarse_scan_core.cpp)
  target_link_libraries(workspace_coarse_scan_core_test workspace_coarse_scan_core)
  ament_add_gtest(workspace_return_core_test
    test/test_workspace_return_core.cpp)
  target_link_libraries(workspace_return_core_test workspace_return_core)
  ament_add_gtest(workspace_robot_state_utils_test
    test/test_workspace_robot_state_utils.cpp)
  target_link_libraries(workspace_robot_state_utils_test
    workspace_robot_state_utils moveit_core::moveit_test_utils)
  ament_add_gtest(world_height_color_test
    test/test_world_height_color.cpp)
  target_link_libraries(world_height_color_test world_height_color)
  ament_add_gtest(fixed_axis_scan_core_test
    test/test_fixed_axis_scan_core.cpp)
  target_link_libraries(fixed_axis_scan_core_test fixed_axis_scan_core)
  ament_add_pytest_test(scanner_650_model_test
    test/test_scanner_650_model.py
    TIMEOUT 30)
endif()

ament_package()

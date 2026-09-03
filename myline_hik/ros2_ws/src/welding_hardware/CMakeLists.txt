cmake_minimum_required(VERSION 3.8)
project(welding_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(welding_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

set(LEGACY_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../src")

add_library(line_laser_backend
  "${LEGACY_SOURCE_ROOT}/LineLaserController.cpp"
  "${LEGACY_SOURCE_ROOT}/LineLaserController.h")
target_include_directories(line_laser_backend PUBLIC "${LEGACY_SOURCE_ROOT}")
target_link_libraries(line_laser_backend Qt5::Core)

add_executable(line_laser_safety_node src/line_laser_safety_node.cpp)
target_link_libraries(line_laser_safety_node line_laser_backend Qt5::Core)
ament_target_dependencies(line_laser_safety_node rclcpp welding_interfaces)

add_executable(robot_feedback_bridge_node src/robot_feedback_bridge_node.cpp)
ament_target_dependencies(robot_feedback_bridge_node
  rclcpp sensor_msgs geometry_msgs std_msgs welding_interfaces)

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
  add_library(hik_stereo_camera_backend
    "${LEGACY_SOURCE_ROOT}/HikSynchronizationCore.cpp"
    "${LEGACY_SOURCE_ROOT}/HikSynchronizationCore.h"
    "${LEGACY_SOURCE_ROOT}/HikCameraWorker.cpp"
    "${LEGACY_SOURCE_ROOT}/HikCameraWorker.h"
    "${LEGACY_SOURCE_ROOT}/stereo/core/StereoFrameSynchronizer.cpp"
    "${LEGACY_SOURCE_ROOT}/stereo/core/StereoFrameSynchronizer.h"
    "${LEGACY_SOURCE_ROOT}/stereo/app/StereoCameraRig.cpp"
    "${LEGACY_SOURCE_ROOT}/stereo/app/StereoCameraRig.h")
  target_compile_definitions(hik_stereo_camera_backend PUBLIC HAVE_HIK_MVS=1)
  target_include_directories(hik_stereo_camera_backend PUBLIC
    "${LEGACY_SOURCE_ROOT}" ${HIK_MVS_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(hik_stereo_camera_backend
    Qt5::Core Qt5::Gui Eigen3::Eigen ${OpenCV_LIBS} ${HIK_MVS_LIBRARY} pthread)

  add_executable(hik_stereo_camera_node src/hik_stereo_camera_node.cpp)
  target_link_libraries(hik_stereo_camera_node hik_stereo_camera_backend Qt5::Core Qt5::Gui)
  ament_target_dependencies(hik_stereo_camera_node rclcpp sensor_msgs std_msgs)
  install(TARGETS hik_stereo_camera_backend hik_stereo_camera_node
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "Hikrobot MVS SDK not found; hik_stereo_camera_node will not be built")
endif()

install(TARGETS line_laser_backend line_laser_safety_node robot_feedback_bridge_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(welding_mapping)
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(welding_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(PCL REQUIRED COMPONENTS common io)

set(LEGACY_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../src")
add_library(stereo_depth_core
  "${LEGACY_SOURCE_ROOT}/StripeCenterlineExtractor.cpp"
  "${LEGACY_SOURCE_ROOT}/HikCalibrationCore.cpp"
  "${LEGACY_SOURCE_ROOT}/HikScanCore.cpp"
  "${LEGACY_SOURCE_ROOT}/stereo/core/StereoRigCalibration.cpp"
  "${LEGACY_SOURCE_ROOT}/stereo/core/StereoDepthEngine.cpp")
target_include_directories(stereo_depth_core PUBLIC "${LEGACY_SOURCE_ROOT}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_depth_core ${OpenCV_LIBS})
target_compile_options(stereo_depth_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(stereo_depth_node src/stereo_depth_node.cpp)
target_link_libraries(stereo_depth_node stereo_depth_core ${OpenCV_LIBS})
ament_target_dependencies(stereo_depth_node
  rclcpp sensor_msgs std_msgs cv_bridge message_filters welding_interfaces)

add_executable(octomap_builder_node src/octomap_builder_node.cpp)
target_link_libraries(octomap_builder_node ${OCTOMAP_LIBRARIES})
target_include_directories(octomap_builder_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(octomap_builder_node
  rclcpp sensor_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs octomap_msgs)

add_executable(pointcloud_file_publisher src/pointcloud_file_publisher.cpp)
target_include_directories(pointcloud_file_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_file_publisher ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_file_publisher rclcpp sensor_msgs pcl_conversions)

install(TARGETS stereo_depth_core stereo_depth_node octomap_builder_node pointcloud_file_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()

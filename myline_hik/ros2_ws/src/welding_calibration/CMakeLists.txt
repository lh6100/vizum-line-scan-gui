cmake_minimum_required(VERSION 3.8)
project(welding_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(welding_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(calibration_registry_core
  src/calibration_registry.cpp
)
target_include_directories(calibration_registry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(calibration_registry_core PUBLIC yaml-cpp OpenSSL::Crypto)
target_compile_options(calibration_registry_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(calibration_registry_node src/calibration_registry_node.cpp)
target_link_libraries(calibration_registry_node calibration_registry_core)
ament_target_dependencies(calibration_registry_node rclcpp welding_interfaces)

add_executable(calibration_tf_publisher_node src/calibration_tf_publisher_node.cpp)
target_link_libraries(calibration_tf_publisher_node yaml-cpp)
ament_target_dependencies(calibration_tf_publisher_node
  rclcpp welding_interfaces geometry_msgs tf2 tf2_ros)

install(TARGETS calibration_registry_core calibration_registry_node calibration_tf_publisher_node
  EXPORT export_calibration_registry_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/create_calibration_manifest.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(calibration_registry_test test/calibration_registry_test.cpp)
  target_link_libraries(calibration_registry_test calibration_registry_core)
endif()

ament_export_targets(export_calibration_registry_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(yaml_cpp_vendor OpenSSL)
ament_package()

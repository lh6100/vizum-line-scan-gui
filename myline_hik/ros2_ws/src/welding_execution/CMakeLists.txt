cmake_minimum_required(VERSION 3.8)
project(welding_execution)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(welding_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(execution_guard_core src/execution_guard.cpp)
target_include_directories(execution_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(execution_guard_core OpenSSL::Crypto)
ament_target_dependencies(execution_guard_core trajectory_msgs)
target_compile_options(execution_guard_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(robot_command_gateway_node src/robot_command_gateway_node.cpp)
target_link_libraries(robot_command_gateway_node execution_guard_core yaml-cpp)
ament_target_dependencies(robot_command_gateway_node
  rclcpp rclcpp_action welding_interfaces control_msgs moveit_msgs trajectory_msgs)

install(TARGETS execution_guard_core robot_command_gateway_node
  EXPORT export_execution_guard_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(execution_guard_test test/execution_guard_test.cpp)
  target_link_libraries(execution_guard_test execution_guard_core)
endif()

ament_export_targets(export_execution_guard_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(trajectory_msgs)
ament_package()

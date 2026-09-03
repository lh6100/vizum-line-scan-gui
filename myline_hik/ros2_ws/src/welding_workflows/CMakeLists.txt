cmake_minimum_required(VERSION 3.8)
project(welding_workflows)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(welding_interfaces REQUIRED)

add_library(workflow_state_core src/workflow_state_machine.cpp)
target_include_directories(workflow_state_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(workflow_state_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(workflow_supervisor_node src/workflow_supervisor_node.cpp)
target_link_libraries(workflow_supervisor_node workflow_state_core)
ament_target_dependencies(workflow_supervisor_node rclcpp rclcpp_action welding_interfaces)

install(TARGETS workflow_state_core workflow_supervisor_node
  EXPORT export_workflow_state_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(workflow_state_machine_test test/workflow_state_machine_test.cpp)
  target_link_libraries(workflow_state_machine_test workflow_state_core)
endif()

ament_export_targets(export_workflow_state_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()

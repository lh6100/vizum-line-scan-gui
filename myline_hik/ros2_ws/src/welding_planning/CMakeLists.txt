cmake_minimum_required(VERSION 3.8)
project(welding_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(welding_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(plan_dependency_core src/plan_dependency_validator.cpp)
target_include_directories(plan_dependency_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(plan_dependency_core welding_interfaces)
target_compile_options(plan_dependency_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(planning_scene_registry_node src/planning_scene_registry_node.cpp)
target_link_libraries(planning_scene_registry_node plan_dependency_core yaml-cpp OpenSSL::Crypto)
ament_target_dependencies(planning_scene_registry_node
  rclcpp welding_interfaces moveit_msgs octomap_msgs)

install(TARGETS plan_dependency_core planning_scene_registry_node
  EXPORT export_plan_dependency_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(plan_dependency_validator_test test/plan_dependency_validator_test.cpp)
  target_link_libraries(plan_dependency_validator_test plan_dependency_core)
endif()

ament_export_targets(export_plan_dependency_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(welding_interfaces)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(welding_data)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(welding_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(session_manager_core src/session_manager.cpp)
target_include_directories(session_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(session_manager_core PUBLIC yaml-cpp OpenSSL::Crypto)
target_compile_options(session_manager_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(session_manager_node src/session_manager_node.cpp)
target_link_libraries(session_manager_node session_manager_core)
ament_target_dependencies(session_manager_node rclcpp welding_interfaces)

install(TARGETS session_manager_core session_manager_node
  EXPORT export_session_manager_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(session_manager_test test/session_manager_test.cpp)
  target_link_libraries(session_manager_test session_manager_core)
endif()

ament_export_targets(export_session_manager_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(yaml_cpp_vendor OpenSSL)
ament_package()

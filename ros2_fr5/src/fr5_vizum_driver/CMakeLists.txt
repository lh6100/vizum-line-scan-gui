cmake_minimum_required(VERSION 3.8)
project(fr5_vizum_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fr5_vizum_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(joint_state_utils
  src/joint_state_utils.cpp
)

target_include_directories(joint_state_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(flange_pose_utils
  src/flange_pose_utils.cpp
)

target_include_directories(flange_pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(flange_pose_utils
  joint_state_utils
)

set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(DEFAULT_FAIRINO_SDK_DIR "${REPO_ROOT}/SDK/fairino-cpp-sdk-3.9.4")
if(DEFINED ENV{FAIRINO_SDK_DIR} AND NOT "$ENV{FAIRINO_SDK_DIR}" STREQUAL "")
  set(DEFAULT_FAIRINO_SDK_DIR "$ENV{FAIRINO_SDK_DIR}")
endif()
set(FAIRINO_SDK_DIR "${DEFAULT_FAIRINO_SDK_DIR}" CACHE PATH
  "Path to a Fairino C++ SDK whose robot.h and libfairino have matching versions")

# find_path()/find_library() cache their results independently.  Clear those
# derived entries on every configure so changing FAIRINO_SDK_DIR can never
# silently keep headers or a library from the previous SDK location.
unset(FAIRINO_INCLUDE_DIR CACHE)
unset(FAIRINO_LIBRARY CACHE)

find_path(FAIRINO_INCLUDE_DIR
  NAMES robot.h
  PATHS
    "${FAIRINO_SDK_DIR}/linux/libfairino/include"
    "${FAIRINO_SDK_DIR}/libfairino/include"
  NO_DEFAULT_PATH
)

find_library(FAIRINO_LIBRARY
  NAMES fairino
  PATHS
    "${FAIRINO_SDK_DIR}/libfairino/LinuxBuild/bin"
    "${FAIRINO_SDK_DIR}/linux/libfairino/lib"
    "${FAIRINO_SDK_DIR}/libfairino/lib"
  NO_DEFAULT_PATH
)

if(NOT FAIRINO_INCLUDE_DIR OR NOT FAIRINO_LIBRARY)
  message(FATAL_ERROR
    "Fairino SDK not found under '${FAIRINO_SDK_DIR}'. "
    "Set FAIRINO_SDK_DIR in the environment or pass "
    "-DFAIRINO_SDK_DIR=/absolute/path to colcon/CMake.")
endif()

message(STATUS "Fairino SDK include: ${FAIRINO_INCLUDE_DIR}")
message(STATUS "Fairino SDK library: ${FAIRINO_LIBRARY}")

add_executable(fairino_state_publisher
  src/fairino_state_publisher.cpp
)

target_include_directories(fairino_state_publisher PRIVATE
  "${FAIRINO_INCLUDE_DIR}"
)

ament_target_dependencies(fairino_state_publisher
  fr5_vizum_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(fairino_state_publisher
  flange_pose_utils
  joint_state_utils
  "${FAIRINO_LIBRARY}"
)

get_filename_component(FAIRINO_LIBRARY_DIR "${FAIRINO_LIBRARY}" DIRECTORY)
set_target_properties(fairino_state_publisher PROPERTIES
  BUILD_RPATH "${FAIRINO_LIBRARY_DIR}"
  INSTALL_RPATH "${FAIRINO_LIBRARY_DIR}"
)

set(FR5_VIZUM_INSTALL_TARGETS
  joint_state_utils
  flange_pose_utils
  fairino_state_publisher
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_joint_state_utils tests/test_joint_state_utils.cpp)
  target_link_libraries(test_joint_state_utils joint_state_utils)

  ament_add_gtest(test_flange_pose_utils tests/test_flange_pose_utils.cpp)
  target_link_libraries(test_flange_pose_utils flange_pose_utils)

endif()

install(TARGETS ${FR5_VIZUM_INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  include/fr5_vizum_driver/flange_pose_utils.hpp
  include/fr5_vizum_driver/joint_state_utils.hpp
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

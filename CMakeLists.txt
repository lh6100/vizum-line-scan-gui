cmake_minimum_required(VERSION 3.10)
project(VizumScanGUI CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree sample_consensus io)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonColor
    CommonDataModel
    FiltersCore
    FiltersSources
    GUISupportQt
    InteractionStyle
    IOPLY
    RenderingCore
    RenderingOpenGL2
)

# SDK paths. Prefer the SDK bundled in this repository, but keep the old
# sibling layout working for local development.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/SDK/VzNLSDK")
    set(VZNL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/VzNLSDK)
else()
    set(VZNL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../SDK/VzNLSDK)
endif()
set(VZNL_INC_DIR ${VZNL_SDK_DIR}/Inc)
set(VZNL_LIB_DIR ${VZNL_SDK_DIR}/Linux/x64)

set(FAIRINO_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/fairino-cpp-sdk-3.9.4)
set(FAIRINO_INC_DIR ${FAIRINO_SDK_DIR}/linux/libfairino/include)
set(FAIRINO_LIB_DIR ${FAIRINO_SDK_DIR}/libfairino/LinuxBuild/bin)
if(EXISTS "${FAIRINO_INC_DIR}/robot.h" AND EXISTS "${FAIRINO_LIB_DIR}/libfairino.so")
    set(HAVE_FAIRINO_SDK TRUE)
else()
    set(HAVE_FAIRINO_SDK FALSE)
endif()

include_directories(${VZNL_INC_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${VZNL_LIB_DIR})
if(HAVE_FAIRINO_SDK)
    include_directories(${FAIRINO_INC_DIR})
    link_directories(${FAIRINO_LIB_DIR})
endif()
add_definitions(${PCL_DEFINITIONS})

# Make the binary find SDK shared libs at runtime
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${VZNL_LIB_DIR}")
set(CMAKE_BUILD_RPATH "${VZNL_LIB_DIR}")
if(HAVE_FAIRINO_SDK)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${FAIRINO_LIB_DIR}")
    set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH};${FAIRINO_LIB_DIR}")
endif()

add_library(WeldMotionCore
    src/geometry/TransformUtils.cpp
    src/geometry/TransformUtils.h
    src/welding/WeldPathPlanner.cpp
    src/welding/WeldPathPlanner.h
    src/robot_client/FairinoRobotClient.cpp
    src/robot_client/FairinoRobotClient.h
    src/vision/PointCloudTransform.cpp
    src/vision/PointCloudTransform.h
)

target_link_libraries(WeldMotionCore
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

target_compile_definitions(WeldMotionCore PUBLIC HAVE_PCL_IO)

if(HAVE_FAIRINO_SDK)
    target_compile_definitions(WeldMotionCore PUBLIC HAVE_FAIRINO_SDK)
    target_link_libraries(WeldMotionCore fairino)
endif()

add_executable(VizumScanGUI
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
    src/RobotControlWidget.cpp
    src/RobotControlWidget.h
    src/ScanWorker.cpp
    src/ScanWorker.h
    src/WeldSeamWidget.cpp
    src/WeldSeamWidget.h
)

target_link_libraries(VizumScanGUI
    Qt5::Widgets
    -Wl,--no-as-needed
    VzNLDetect
    VzNLGraphics
    VzKernel
    VzEyeSecurityLoader-shared
    -Wl,--as-needed
    WeldMotionCore
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    pthread
)

vtk_module_autoinit(
    TARGETS VizumScanGUI
    MODULES ${VTK_LIBRARIES}
)

add_executable(CameraToBaseTest
    tests/test_camera_to_base.cpp
)

target_link_libraries(CameraToBaseTest
    WeldMotionCore
)

add_executable(FairinoWeldMoveDemo
    tests/fairino_weld_move_demo.cpp
)

target_link_libraries(FairinoWeldMoveDemo
    WeldMotionCore
)

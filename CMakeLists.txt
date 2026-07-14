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
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)
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
include_directories(${OpenCV_INCLUDE_DIRS})
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
    src/robot_capture/RobotOffsetCapturePlanner.cpp
    src/robot_capture/RobotOffsetCapturePlanner.h
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
    src/ExtrinsicCalibrationWidget.cpp
    src/ExtrinsicCalibrationWidget.h
    src/MainWindow.cpp
    src/MainWindow.h
    src/MotionTracePlotWidget.cpp
    src/MotionTracePlotWidget.h
    src/RobotControlWidget.cpp
    src/RobotControlWidget.h
    src/RgbImageLabel.cpp
    src/RgbImageLabel.h
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
    ${OpenCV_LIBS}
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

add_executable(RobotOffsetCapturePlannerTest
    tests/test_robot_offset_capture_planner.cpp
)

target_link_libraries(RobotOffsetCapturePlannerTest
    WeldMotionCore
)

add_executable(VizumEyeCaptureDemo
    tests/vizum_eye_capture_demo.cpp
)

target_link_libraries(VizumEyeCaptureDemo
    -Wl,--no-as-needed
    VzNLDetect
    VzNLGraphics
    VzKernel
    VzEyeSecurityLoader-shared
    -Wl,--as-needed
    ${OpenCV_LIBS}
    pthread
)

target_compile_definitions(VizumEyeCaptureDemo PRIVATE VIZUM_HAS_TRANSPICMODE)

add_executable(VizumDynamicProfileOnceDemo
    tests/vizum_dynamic_profile_once_demo.cpp
)

target_link_libraries(VizumDynamicProfileOnceDemo
    -Wl,--no-as-needed
    VzNLDetect
    VzNLGraphics
    VzKernel
    VzEyeSecurityLoader-shared
    -Wl,--as-needed
    ${OpenCV_LIBS}
    pthread
)

set(VZNL_MUL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/MulDetectLaser_3.6.20.110_All/SDK/VzNLSDK)
if(EXISTS "${VZNL_MUL_SDK_DIR}/Linux/x64/libVzNLDetect.so")
    set(VZNL_MUL_INC_DIR ${VZNL_MUL_SDK_DIR}/Inc)
    set(VZNL_MUL_LIB_DIR ${VZNL_MUL_SDK_DIR}/Linux/x64)

    add_executable(VizumEyeCaptureDemoMulSDK
        tests/vizum_eye_capture_demo.cpp
    )

    target_include_directories(VizumEyeCaptureDemoMulSDK BEFORE PRIVATE
        ${VZNL_MUL_INC_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(VizumEyeCaptureDemoMulSDK
        -Wl,--no-as-needed
        ${VZNL_MUL_LIB_DIR}/libVzNLDetect.so
        ${VZNL_MUL_LIB_DIR}/libVzNLGraphics.so
        ${VZNL_MUL_LIB_DIR}/libVzKernel.so
        ${VZNL_MUL_LIB_DIR}/libVzEyeSecurityLoader-shared.so
        -Wl,--as-needed
        ${OpenCV_LIBS}
        pthread
    )

    set_target_properties(VizumEyeCaptureDemoMulSDK PROPERTIES
        BUILD_RPATH "${VZNL_MUL_LIB_DIR}"
        INSTALL_RPATH "${VZNL_MUL_LIB_DIR}"
    )
endif()

set(VZNL_DETECT_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/DetectLaser_3.6.20.116_Windows&Linux&ARM/SDK/VzNLSDK)
if(EXISTS "${VZNL_DETECT_SDK_DIR}/Linux/x64/libVzNLDetect.so")
    set(VZNL_DETECT_INC_DIR ${VZNL_DETECT_SDK_DIR}/Inc)
    set(VZNL_DETECT_LIB_DIR ${VZNL_DETECT_SDK_DIR}/Linux/x64)

    add_executable(VizumEyeCaptureDemoDetectSDK
        tests/vizum_eye_capture_demo.cpp
    )

    target_compile_definitions(VizumEyeCaptureDemoDetectSDK PRIVATE VIZUM_HAS_TRANSPICMODE)

    target_include_directories(VizumEyeCaptureDemoDetectSDK BEFORE PRIVATE
        ${VZNL_DETECT_INC_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(VizumEyeCaptureDemoDetectSDK
        -Wl,--no-as-needed
        ${VZNL_DETECT_LIB_DIR}/libVzNLDetect.so
        ${VZNL_DETECT_LIB_DIR}/libVzNLGraphics.so
        ${VZNL_DETECT_LIB_DIR}/libVzKernel.so
        ${VZNL_DETECT_LIB_DIR}/libVzEyeSecurityLoader-shared.so
        -Wl,--as-needed
        ${OpenCV_LIBS}
        pthread
    )

    set_target_properties(VizumEyeCaptureDemoDetectSDK PROPERTIES
        BUILD_RPATH "${VZNL_DETECT_LIB_DIR}"
        INSTALL_RPATH "${VZNL_DETECT_LIB_DIR}"
    )
endif()

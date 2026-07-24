cmake_minimum_required(VERSION 3.10)
project(HikLineLaserCalibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(CTest)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
if(BUILD_TESTING)
    find_package(Qt5 REQUIRED COMPONENTS Test)
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d aruco)
find_package(Eigen3 REQUIRED NO_MODULE)

if(Qt5Core_DIR MATCHES "[Aa]naconda|[Cc]onda")
    message(FATAL_ERROR
        "Conda Qt was selected (${Qt5Core_DIR}) while this tool uses the system OpenCV. "
        "Run ./run_calibration_gui.sh so Qt and OpenCV use one compatible system ABI.")
endif()

set(HIK_MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation directory")
find_path(HIK_MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS "${HIK_MVS_ROOT}/include"
    NO_DEFAULT_PATH
)
find_library(HIK_MVS_LIBRARY
    NAMES MvCameraControl
    PATHS "${HIK_MVS_ROOT}/lib/64"
    NO_DEFAULT_PATH
)

add_library(LineLaserDeviceProfile
    src/LineLaserDeviceProfile.cpp
    src/LineLaserDeviceProfile.h
)
target_include_directories(LineLaserDeviceProfile PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(LineLaserDeviceProfile PUBLIC Qt5::Core)

add_library(LineLaserControl
    src/LineLaserController.cpp
    src/LineLaserController.h
)
target_include_directories(LineLaserControl PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(LineLaserControl PUBLIC Qt5::Core)

add_library(StripeCenterlineExtractor
    src/StripeCenterlineExtractor.cpp
    src/StripeCenterlineExtractor.h
)
target_include_directories(StripeCenterlineExtractor PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(StripeCenterlineExtractor PUBLIC ${OpenCV_LIBS})

add_library(HikCalibrationCore
    src/HikCalibrationCore.cpp
    src/HikCalibrationCore.h
    src/HandEyeCalibrationCore.cpp
    src/HandEyeCalibrationCore.h
    src/HikScanCore.cpp
    src/HikScanCore.h
)
target_include_directories(HikCalibrationCore PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(HikCalibrationCore PUBLIC
    StripeCenterlineExtractor
    ${OpenCV_LIBS}
)

add_library(HikSynchronizationCore
    src/HikSynchronizationCore.cpp
    src/HikSynchronizationCore.h
)
target_include_directories(HikSynchronizationCore PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(HikSynchronizationCore PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    pthread
)

add_library(HikContinuousReconstruction
    src/HikContinuousReconstruction.cpp
    src/HikContinuousReconstruction.h
)
target_include_directories(HikContinuousReconstruction PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(HikContinuousReconstruction PUBLIC
    HikCalibrationCore
    HikSynchronizationCore
    Eigen3::Eigen
    ${OpenCV_LIBS}
    pthread
)

add_library(HikCameraBackend
    src/HikCameraWorker.cpp
    src/HikCameraWorker.h
)
target_include_directories(HikCameraBackend PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(HikCameraBackend PUBLIC
    HikSynchronizationCore
    Qt5::Core
    Qt5::Gui
)

add_executable(ReplayStripeQuality
    tools/replay_stripe_quality.cpp
)
target_include_directories(ReplayStripeQuality PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ReplayStripeQuality PRIVATE
    HikCalibrationCore
    StripeCenterlineExtractor
    ${OpenCV_LIBS}
)

add_library(FairinoRobotBackend
    src/FairinoReadOnlyWorker.cpp
    src/FairinoReadOnlyWorker.h
    src/FairinoRobotSession.cpp
    src/FairinoRobotSession.h
)
target_include_directories(FairinoRobotBackend PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(FairinoRobotBackend PUBLIC
    HikSynchronizationCore
    Qt5::Core
    pthread
)

add_executable(HikLineLaserCalibration
    src/main.cpp
    src/HikCalibrationWindow.cpp
    src/HikCalibrationWindow.h
    src/ImageView.cpp
    src/ImageView.h
)
target_include_directories(HikLineLaserCalibration PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(HikLineLaserCalibration PRIVATE
    HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(HikLineLaserCalibration PRIVATE
    LineLaserDeviceProfile
    HikCalibrationCore
    HikSynchronizationCore
    HikCameraBackend
    FairinoRobotBackend
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OpenCV_LIBS}
    pthread
)

add_executable(HikConstantLaserScan
    src/scan_main.cpp
    src/HikConstantLaserScanWindow.cpp
    src/HikConstantLaserScanWindow.h
    src/ImageView.cpp
    src/ImageView.h
)
target_include_directories(HikConstantLaserScan PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(HikConstantLaserScan PRIVATE
    HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(HikConstantLaserScan PRIVATE
    LineLaserDeviceProfile
    LineLaserControl
    HikCalibrationCore
    HikContinuousReconstruction
    HikSynchronizationCore
    HikCameraBackend
    FairinoRobotBackend
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OpenCV_LIBS}
    pthread
)

if(HIK_MVS_INCLUDE_DIR AND HIK_MVS_LIBRARY)
    message(STATUS "Hikrobot MVS SDK: ${HIK_MVS_LIBRARY}")
    target_compile_definitions(HikCameraBackend PRIVATE HAVE_HIK_MVS=1)
    target_include_directories(HikCameraBackend PRIVATE "${HIK_MVS_INCLUDE_DIR}")
    target_link_libraries(HikCameraBackend PRIVATE "${HIK_MVS_LIBRARY}")
    set_target_properties(HikLineLaserCalibration HikConstantLaserScan PROPERTIES
        BUILD_RPATH "${HIK_MVS_ROOT}/lib/64"
        INSTALL_RPATH "${HIK_MVS_ROOT}/lib/64"
    )
else()
    message(WARNING "Hikrobot MVS SDK not found. The GUI will build in offline-only mode.")
endif()

set(DEFAULT_FAIRINO_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SDK/fairino-cpp-sdk-3.9.4")
if(DEFINED ENV{FAIRINO_SDK_DIR} AND NOT "$ENV{FAIRINO_SDK_DIR}" STREQUAL "")
    set(DEFAULT_FAIRINO_SDK_DIR "$ENV{FAIRINO_SDK_DIR}")
endif()
set(FAIRINO_SDK_DIR "${DEFAULT_FAIRINO_SDK_DIR}" CACHE PATH
    "Fairino C++ SDK used for FR5 pose acquisition and stop-and-shoot motion")
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
if(FAIRINO_INCLUDE_DIR AND FAIRINO_LIBRARY)
    file(READ "${FAIRINO_INCLUDE_DIR}/robot.h" FAIRINO_ROBOT_HEADER_CONTENT)
    string(FIND "${FAIRINO_ROBOT_HEADER_CONTENT}"
        "SetRobotRealtimeStateCallback" FAIRINO_CALLBACK_HEADER_POSITION)
    if(FAIRINO_CALLBACK_HEADER_POSITION EQUAL -1)
        message(FATAL_ERROR
            "The FAIRINO SDK header does not include the required per-packet "
            "20004 callback. Run tools/build_patched_fairino_sdk.sh first.")
    endif()
    if(UNIX)
        find_program(FAIRINO_NM_EXECUTABLE NAMES nm)
        if(FAIRINO_NM_EXECUTABLE)
            execute_process(
                COMMAND "${FAIRINO_NM_EXECUTABLE}" -D -C "${FAIRINO_LIBRARY}"
                OUTPUT_VARIABLE FAIRINO_EXPORTED_SYMBOLS
                ERROR_QUIET
            )
            string(FIND "${FAIRINO_EXPORTED_SYMBOLS}"
                "FRRobot::SetRobotRealtimeStateCallback"
                FAIRINO_CALLBACK_LIBRARY_POSITION)
            if(FAIRINO_CALLBACK_LIBRARY_POSITION EQUAL -1)
                message(FATAL_ERROR
                    "The FAIRINO shared library does not export the required "
                    "per-packet 20004 callback. Run "
                    "tools/build_patched_fairino_sdk.sh first.")
            endif()
        endif()
    endif()
    message(STATUS "Fairino SDK: ${FAIRINO_LIBRARY}")
    target_compile_definitions(FairinoRobotBackend PRIVATE HAVE_FAIRINO_SDK=1)
    target_include_directories(FairinoRobotBackend PRIVATE "${FAIRINO_INCLUDE_DIR}")
    target_link_libraries(FairinoRobotBackend PRIVATE "${FAIRINO_LIBRARY}")
    get_filename_component(FAIRINO_LIBRARY_DIR "${FAIRINO_LIBRARY}" DIRECTORY)
    set_property(TARGET HikLineLaserCalibration APPEND PROPERTY
        BUILD_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikConstantLaserScan APPEND PROPERTY
        BUILD_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikLineLaserCalibration APPEND PROPERTY
        INSTALL_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikConstantLaserScan APPEND PROPERTY
        INSTALL_RPATH "${FAIRINO_LIBRARY_DIR}")
else()
    message(WARNING "Fairino SDK not found. Offline/core functionality remains available, but live FR5 pose reading and stop-and-shoot scanning are disabled.")
endif()

if(OpenCV_VERSION VERSION_GREATER_EQUAL "4.6.0")
    message(FATAL_ERROR
        "This tool currently guarantees the OpenCV 4.5 legacy ChArUco layout only. "
        "OpenCV ${OpenCV_VERSION} may use a different board coordinate convention; "
        "refusing to build rather than writing mislabeled calibration files.")
endif()

if(BUILD_TESTING)
    add_executable(HikCalibrationCoreTest
        tests/test_hik_calibration_core.cpp
    )
    target_include_directories(HikCalibrationCoreTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(HikCalibrationCoreTest PRIVATE
        HikCalibrationCore
        ${OpenCV_LIBS}
    )
    add_test(NAME HikCalibrationCoreTest COMMAND HikCalibrationCoreTest)

    add_executable(StripeCenterlineExtractorTest
        tests/test_stripe_centerline_extractor.cpp
    )
    target_include_directories(StripeCenterlineExtractorTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(StripeCenterlineExtractorTest PRIVATE
        StripeCenterlineExtractor
        ${OpenCV_LIBS}
    )
    add_test(
        NAME StripeCenterlineExtractorTest
        COMMAND StripeCenterlineExtractorTest)

    add_executable(HikScanCoreTest
        tests/test_hik_scan_core.cpp
    )
    target_include_directories(HikScanCoreTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(HikScanCoreTest PRIVATE
        HikCalibrationCore
        ${OpenCV_LIBS}
    )
    add_test(NAME HikScanCoreTest COMMAND HikScanCoreTest)

    add_executable(HikSynchronizationCoreTest
        tests/test_hik_synchronization_core.cpp
    )
    target_include_directories(HikSynchronizationCoreTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(HikSynchronizationCoreTest PRIVATE
        HikSynchronizationCore
        Eigen3::Eigen
    )
    add_test(NAME HikSynchronizationCoreTest COMMAND HikSynchronizationCoreTest)

    add_executable(LineLaserDeviceProfileTest
        tests/test_line_laser_device_profile.cpp
    )
    target_link_libraries(LineLaserDeviceProfileTest PRIVATE
        LineLaserDeviceProfile
        Qt5::Core
    )
    add_test(NAME LineLaserDeviceProfileTest COMMAND LineLaserDeviceProfileTest)

    add_executable(LineLaserControllerTest
        tests/test_line_laser_controller.cpp
    )
    target_link_libraries(LineLaserControllerTest PRIVATE
        LineLaserControl
        Qt5::Core
        Qt5::Test
    )
    add_test(NAME LineLaserControllerTest COMMAND LineLaserControllerTest)

    add_executable(FairinoRobotSessionTest
        tests/test_fairino_robot_session.cpp
    )
    target_link_libraries(FairinoRobotSessionTest PRIVATE
        FairinoRobotBackend
        Qt5::Core
        Qt5::Test
    )
    add_test(NAME FairinoRobotSessionTest
             COMMAND FairinoRobotSessionTest)

    add_executable(HikContinuousReconstructionTest
        tests/test_hik_continuous_reconstruction.cpp
    )
    target_include_directories(HikContinuousReconstructionTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(HikContinuousReconstructionTest PRIVATE
        HikContinuousReconstruction
        Eigen3::Eigen
    )
    add_test(NAME HikContinuousReconstructionTest
             COMMAND HikContinuousReconstructionTest)
endif()

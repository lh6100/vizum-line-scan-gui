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

set(_qt_component_dir_variables
    Qt5_DIR
    Qt5Core_DIR
    Qt5Gui_DIR
    Qt5Widgets_DIR
)
if(BUILD_TESTING)
    list(APPEND _qt_component_dir_variables Qt5Test_DIR)
endif()
foreach(_qt_component_dir_variable IN LISTS _qt_component_dir_variables)
    if(DEFINED ${_qt_component_dir_variable}
       AND "${${_qt_component_dir_variable}}" MATCHES "[Aa]naconda|[Cc]onda")
        message(FATAL_ERROR
            "Conda Qt was selected for ${_qt_component_dir_variable} "
            "(${${_qt_component_dir_variable}}) while this tool uses the system "
            "OpenCV. Run one of the project launch scripts so every Qt component "
            "uses one compatible system ABI.")
    endif()
endforeach()
unset(_qt_component_dir_variable)
unset(_qt_component_dir_variables)

set(DEFAULT_HIK_MVS_ROOT "/opt/MVS")
if(DEFINED ENV{HIK_MVS_ROOT} AND NOT "$ENV{HIK_MVS_ROOT}" STREQUAL "")
    set(DEFAULT_HIK_MVS_ROOT "$ENV{HIK_MVS_ROOT}")
endif()
set(HIK_MVS_ROOT "${DEFAULT_HIK_MVS_ROOT}" CACHE PATH
    "Hikrobot MVS SDK installation directory")
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
    src/AutomaticCalibrationCore.cpp
    src/AutomaticCalibrationCore.h
    src/AdaptiveScanExecutor.cpp
    src/AdaptiveScanExecutor.h
    src/AdaptiveScanPlanner.cpp
    src/AdaptiveScanPlanner.h
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

add_library(HikStereoCore
    src/stereo/calibration/StereoCalibrationEngine.cpp
    src/stereo/calibration/StereoCalibrationEngine.h
    src/stereo/core/StereoRigCalibration.cpp
    src/stereo/core/StereoRigCalibration.h
    src/stereo/core/StereoDepthEngine.cpp
    src/stereo/core/StereoDepthEngine.h
    src/stereo/core/StereoFrameSynchronizer.cpp
    src/stereo/core/StereoFrameSynchronizer.h
    src/stereo/map/VoxelOccupancyMap.cpp
    src/stereo/map/VoxelOccupancyMap.h
    src/stereo/pipeline/StereoMappingPipeline.cpp
    src/stereo/pipeline/StereoMappingPipeline.h
)
target_include_directories(HikStereoCore PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(HikStereoCore PUBLIC
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

add_library(HikExposureTestStorage
    src/exposure_test/ExposureTestStorage.cpp
    src/exposure_test/ExposureTestStorage.h
)
target_include_directories(HikExposureTestStorage PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(HikExposureTestStorage PUBLIC
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
    src/Fr5PathEvaluator.h
)
target_include_directories(FairinoRobotBackend PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(FairinoRobotBackend PUBLIC
    HikCalibrationCore
    HikSynchronizationCore
    Eigen3::Eigen
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
    LineLaserControl
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

add_executable(HikStereoMapper
    src/stereo/stereo_main.cpp
    src/stereo/app/StereoCameraRig.cpp
    src/stereo/app/StereoCameraRig.h
    src/stereo/app/StereoMappingWorker.cpp
    src/stereo/app/StereoMappingWorker.h
    src/stereo/app/StereoMapperWindow.cpp
    src/stereo/app/StereoMapperWindow.h
    src/stereo/app/StereoMapperWindowUi.cpp
    src/ImageView.cpp
    src/ImageView.h
)
target_include_directories(HikStereoMapper PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(HikStereoMapper PRIVATE
    HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(HikStereoMapper PRIVATE
    LineLaserDeviceProfile
    LineLaserControl
    HikStereoCore
    HikCameraBackend
    FairinoRobotBackend
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OpenCV_LIBS}
    pthread
)

add_executable(HikStereoCalibration
    src/stereo/stereo_calibration_main.cpp
    src/stereo/app/StereoCameraRig.cpp
    src/stereo/app/StereoCameraRig.h
    src/stereo/calibration_app/StereoCalibrationWindow.cpp
    src/stereo/calibration_app/StereoCalibrationWindow.h
    src/stereo/calibration_app/StereoCalibrationWindowUi.cpp
    src/ImageView.cpp
    src/ImageView.h
)
target_include_directories(HikStereoCalibration PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(HikStereoCalibration PRIVATE
    HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(HikStereoCalibration PRIVATE
    LineLaserDeviceProfile
    LineLaserControl
    HikStereoCore
    HikCameraBackend
    FairinoRobotBackend
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OpenCV_LIBS}
    pthread
)

add_executable(HikExposureTest
    src/exposure_test_main.cpp
    src/exposure_test/ExposureTestWindow.cpp
    src/exposure_test/ExposureTestWindow.h
    src/ImageView.cpp
    src/ImageView.h
)
target_include_directories(HikExposureTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_definitions(HikExposureTest PRIVATE
    HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(HikExposureTest PRIVATE
    LineLaserDeviceProfile
    LineLaserControl
    HikCameraBackend
    HikExposureTestStorage
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    pthread
)

add_executable(HikExposureSweep
    src/exposure_sweep_main.cpp
)
target_include_directories(HikExposureSweep PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_definitions(HikExposureSweep PRIVATE
    HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(HikExposureSweep PRIVATE
    LineLaserDeviceProfile
    LineLaserControl
    HikCameraBackend
    HikExposureTestStorage
    Qt5::Core
    Qt5::Gui
    pthread
)

if(HIK_MVS_INCLUDE_DIR AND HIK_MVS_LIBRARY)
    message(STATUS "Hikrobot MVS SDK: ${HIK_MVS_LIBRARY}")
    target_compile_definitions(HikCameraBackend PRIVATE HAVE_HIK_MVS=1)
    target_include_directories(HikCameraBackend PRIVATE "${HIK_MVS_INCLUDE_DIR}")
    target_link_libraries(HikCameraBackend PRIVATE "${HIK_MVS_LIBRARY}")
    set_target_properties(HikLineLaserCalibration HikConstantLaserScan HikStereoMapper HikStereoCalibration HikExposureTest HikExposureSweep PROPERTIES
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
    set_property(TARGET HikStereoMapper APPEND PROPERTY
        BUILD_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikStereoCalibration APPEND PROPERTY
        BUILD_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikLineLaserCalibration APPEND PROPERTY
        INSTALL_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikConstantLaserScan APPEND PROPERTY
        INSTALL_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikStereoMapper APPEND PROPERTY
        INSTALL_RPATH "${FAIRINO_LIBRARY_DIR}")
    set_property(TARGET HikStereoCalibration APPEND PROPERTY
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

    add_executable(AutomaticCalibrationCoreTest
        tests/test_automatic_calibration_core.cpp
    )
    target_include_directories(AutomaticCalibrationCoreTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(AutomaticCalibrationCoreTest PRIVATE
        HikCalibrationCore
        ${OpenCV_LIBS}
    )
    add_test(NAME AutomaticCalibrationCoreTest
             COMMAND AutomaticCalibrationCoreTest)

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

    add_executable(AdaptiveScanPlannerTest
        tests/test_adaptive_scan_planner.cpp
    )
    target_include_directories(AdaptiveScanPlannerTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(AdaptiveScanPlannerTest PRIVATE
        HikCalibrationCore
        ${OpenCV_LIBS}
    )
    add_test(NAME AdaptiveScanPlannerTest
             COMMAND AdaptiveScanPlannerTest)

    add_executable(AdaptiveScanExecutorTest
        tests/test_adaptive_scan_executor.cpp
    )
    target_include_directories(AdaptiveScanExecutorTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(AdaptiveScanExecutorTest PRIVATE
        HikCalibrationCore
        ${OpenCV_LIBS}
    )
    add_test(NAME AdaptiveScanExecutorTest
             COMMAND AdaptiveScanExecutorTest)

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
    target_compile_definitions(HikSynchronizationCoreTest PRIVATE
        HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
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

    add_executable(HikStereoCoreTest
        tests/stereo/test_stereo_core.cpp
    )
    target_include_directories(HikStereoCoreTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(HikStereoCoreTest PRIVATE
        HikStereoCore
        ${OpenCV_LIBS}
    )
    target_compile_definitions(HikStereoCoreTest PRIVATE
        HIK_CALIBRATION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    add_test(NAME HikStereoCoreTest COMMAND HikStereoCoreTest)

    add_executable(HikExposureTestStorageTest
        tests/test_exposure_test_storage.cpp
    )
    target_include_directories(HikExposureTestStorageTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_link_libraries(HikExposureTestStorageTest PRIVATE
        HikExposureTestStorage
        Qt5::Core
        Qt5::Gui
    )
    add_test(NAME HikExposureTestStorageTest
             COMMAND HikExposureTestStorageTest)
endif()

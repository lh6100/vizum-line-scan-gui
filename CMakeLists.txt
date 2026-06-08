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

# SDK paths. Prefer the SDK bundled in this repository, but keep the old
# sibling layout working for local development.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/SDK/VzNLSDK")
    set(VZNL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/VzNLSDK)
else()
    set(VZNL_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../SDK/VzNLSDK)
endif()
set(VZNL_INC_DIR ${VZNL_SDK_DIR}/Inc)
set(VZNL_LIB_DIR ${VZNL_SDK_DIR}/Linux/x64)

include_directories(${VZNL_INC_DIR})
link_directories(${VZNL_LIB_DIR})

# Make the binary find SDK shared libs at runtime
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${VZNL_LIB_DIR}")
set(CMAKE_BUILD_RPATH "${VZNL_LIB_DIR}")

add_executable(VizumScanGUI
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
    src/ScanWorker.cpp
    src/ScanWorker.h
)

target_link_libraries(VizumScanGUI
    Qt5::Widgets
    -Wl,--no-as-needed
    VzNLDetect
    VzNLGraphics
    VzKernel
    VzEyeSecurityLoader-shared
    -Wl,--as-needed
    pthread
)

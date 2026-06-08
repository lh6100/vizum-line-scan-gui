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
find_package(PCL REQUIRED COMPONENTS common kdtree sample_consensus)
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

include_directories(${VZNL_INC_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${VZNL_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})

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
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    pthread
)

vtk_module_autoinit(
    TARGETS VizumScanGUI
    MODULES ${VTK_LIBRARIES}
)

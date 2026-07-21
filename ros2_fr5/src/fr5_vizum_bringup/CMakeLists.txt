cmake_minimum_required(VERSION 3.8)
project(fr5_vizum_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/calibrated_frames_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrated_frames_publisher
)

install(FILES launch/fr5_live_rviz.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibration_frames test/test_calibration_frames.py)
endif()

ament_package()

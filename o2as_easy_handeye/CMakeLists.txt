cmake_minimum_required(VERSION 2.8.3)
project(o2as_easy_handeye)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_proc
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  visp_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_calibrator
  src/calibrator.cpp
  src/calibrator.h
  src/calibrator_main.cpp
  src/aistCalibration.cpp
  src/vpMatrix.cpp
  src/names.cpp
  src/names.h
)

target_link_libraries(${PROJECT_NAME}_calibrator
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  gsl
)

add_dependencies(${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_gencpp
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS
    scripts/run_handeye_calibration.py
    scripts/go_home.py
    scripts/convert_phoxi_camera_info_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_calibrator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

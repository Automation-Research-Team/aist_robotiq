cmake_minimum_required(VERSION 2.8.3)
project(o2as_easy_handeye)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_proc
  image_transport
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  visp_bridge
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()
catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibrator
  src/calibrator/calibrator.cpp
  src/calibrator/calibrator.h
  src/calibrator/calibrator_main.cpp
  src/calibrator/aistCalibration.cpp
  src/calibrator/vpMatrix.cpp
  src/calibrator/names.cpp
  src/calibrator/names.h
)

target_link_libraries(calibrator
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  gsl
)

add_dependencies(calibrator
  ${PROJECT_NAME}_gencpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(quat2rpy
  src/quat2rpy/main.cpp
)

target_link_libraries(quat2rpy
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  gsl
)

add_dependencies(quat2rpy
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
    scripts/calibrate.py
    scripts/publish.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    calibrator
    quat2rpy
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

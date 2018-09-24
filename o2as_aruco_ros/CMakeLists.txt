cmake_minimum_required(VERSION 2.8.3)
project(o2as_aruco_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  dynamic_reconfigure
  geometry_msgs
  image_transport
  std_msgs
  std_srvs
  cv_bridge
  aruco
  aruco_ros
  tf
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
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(marker_detector
  src/marker_detector_main.cpp
)

target_link_libraries(marker_detector
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(marker_detector
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############
install(
  TARGETS
    marker_detector
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

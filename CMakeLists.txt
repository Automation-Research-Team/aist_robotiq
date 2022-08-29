cmake_minimum_required(VERSION 2.8.3)
project(aist_localization)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS_RELEASE	"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG	"-g")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(
  PhoLocalization REQUIRED
  CONFIG PATHS "$ENV{PHO_LOCALIZATION_PATH}"
)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    roslib
    genmsg
    message_generation
    actionlib_msgs
    actionlib
    std_msgs
    sensor_msgs
    geometry_msgs
    ddynamic_reconfigure
    tf
    aist_depth_filter
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PHOLOCALIZATION_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_action_files(FILES Localize.action)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
    pluginlib
    nodelet
)

add_library(
  ${PROJECT_NAME}
  src/nodelet.cpp
  src/Localization.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PHOLOCALIZATION_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

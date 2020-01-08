cmake_minimum_required(VERSION 2.8.3)
project(aist_depth_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(PkgConfig)
pkg_check_modules(LIBTIFF REQUIRED libtiff-4)

find_package(catkin REQUIRED
  COMPONENTS
    ddynamic_reconfigure
    sensor_msgs
    image_transport
)

find_package(OpenCV REQUIRED)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBTIFF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src
)

#add_message_files(
#  FILES
#    Corners.msg
#)

#generate_messages(
#  DEPENDENCIES
#    geometry_msgs
#)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
    pluginlib
    nodelet
)

add_executable(aist_depth_filter
  src/depth_filter.cpp
  src/DepthFilter.cpp
  src/oply/OrderedPly.cpp
  src/rply/rply.c
)

add_dependencies(aist_depth_filter
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aist_depth_filter
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBTIFF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(aist_depth_filter_nodelet
  src/nodelet.cpp
  src/DepthFilter.cpp
  src/oply/OrderedPly.cpp
  src/rply/rply.c
)

add_dependencies(aist_depth_filter_nodelet
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aist_depth_filter_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBTIFF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    aist_depth_filter aist_depth_filter_nodelet
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(o2as_aruco_marker_detection)

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
  aruco_msgs
  tf
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED COMPONENTS opencv_core opencv_highgui)

catkin_python_setup()

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES aruco_msgs std_msgs geometry_msgs sensor_msgs)
catkin_package()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_detector 
  src/marker_detector_node.cpp
)
target_link_libraries(marker_detector
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(marker_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    marker_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

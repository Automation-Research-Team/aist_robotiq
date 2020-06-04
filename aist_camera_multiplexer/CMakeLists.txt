cmake_minimum_required(VERSION 2.8.3)
project(aist_camera_multiplexer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(catkin REQUIRED
  COMPONENTS
    sensor_msgs
    image_transport
    message_generation
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
    SelectCamera.srv
)

generate_messages(
#  DEPENDENCIES
#    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
    pluginlib
    nodelet
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/Multiplexer.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/Multiplexer.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

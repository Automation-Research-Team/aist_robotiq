cmake_minimum_required(VERSION 2.8.3)
project(aist_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(
  PhoLocalization REQUIRED
  CONFIG PATHS "$ENV{PHO_LOCALIZATION_PATH}"
)

find_package(
  catkin REQUIRED
  roscpp
  roslib
  genmsg
  actionlib_msgs
  actionlib
  geometry_msgs
  std_msgs
  tf
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PHOLOCALIZATION_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY
    action
  FILES
    Localize.action
)

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
    pluginlib
    nodelet
)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/Localization.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${PHOLOCALIZATION_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(
  ${PROJECT_NAME}_nodelet
  src/nodelet.cpp
  src/Localization.cpp
)

add_dependencies(
  ${PROJECT_NAME}_nodelet
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_nodelet
  ${PHOLOCALIZATION_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
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

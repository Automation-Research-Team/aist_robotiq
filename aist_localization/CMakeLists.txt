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

find_package(catkin REQUIRED
  genmsg actionlib_msgs actionlib geometry_msgs tf
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PHOLOCALIZATION_INCLUDE_DIRS}
)

add_service_files(
  FILES
    LoadScene.srv
)

add_action_files(
  FILES
    Localize.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
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

add_executable(aist_localization
  src/localization.cpp
  src/Localization.cpp
)

add_dependencies(aist_localization
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aist_localization
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PHOLOCALIZATION_LIBRARIES}
)

add_library(aist_localization_nodelet
  src/nodelet.cpp
  src/Localization.cpp
)

add_dependencies(aist_localization_nodelet
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aist_localization_nodelet
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PHOLOCALIZATION_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    aist_localization aist_localization_nodelet
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

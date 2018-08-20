cmake_minimum_required(VERSION 2.8.3)
project(o2as_vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  rospy
  roscpp
  std_srvs
  phoxi_camera
  o2as_realsense_camera
  shape_msgs
  geometry_msgs
  geometric_shapes
  o2as_cad_matching_msgs
  message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
  FILES
  FindObject.srv
  FindObjects.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES o2as_vision
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SOURCES
    src/o2as_realsense_camera_interface.cpp
    src/o2as_cad_matching_interface.cpp 
    src/o2as_planning_scene_interface.cpp
    src/o2as_vision_group_interface.cpp
    src/o2as_vision.cpp
)
add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES} src/o2as_vision_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/phoxi_camera_actual_example.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(
  PROGRAMS
    scripts/cad_matching_interface.py
    scripts/planning_scene_interface.py
    scripts/realsense_camera_interface.py
    scripts/vision_group_interface.py
    scripts/vision_group.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_o2as_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

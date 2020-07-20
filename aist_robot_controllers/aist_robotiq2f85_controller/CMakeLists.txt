cmake_minimum_required(VERSION 3.10)
project(aist_robotiq2f85_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")

find_package(catkin
  REQUIRED
    angles
    control_toolbox
    robot_controllers
    robot_controllers_interface
    robotiq_msgs
    ur_modern_driver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${robotiq_msgs_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    robot_controllers
    robot_controllers_interface
  LIBRARIES
    aist_robotiq2f85_controller
  INCLUDE_DIRS
    include
)

add_library(aist_robotiq2f85_controller src/robotiq_85_gripper.cpp)
target_link_libraries(aist_robotiq2f85_controller ${catkin_LIBRARIES})

install(
  TARGETS aist_robotiq2f85_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY config include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES robotiq_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(aist_robotiq2f85_controller_node src/node.cpp)
target_link_libraries(aist_robotiq2f85_controller_node ${catkin_LIBRARIES})


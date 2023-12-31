cmake_minimum_required(VERSION 2.8.3)
project(aist_robotiq)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox
  message_generation
  xmlrpcpp
  rospy
  actionlib
  actionlib_msgs
  control_msgs
  sensor_msgs
  controller_manager
  controller_interface
)

catkin_python_setup()

add_message_files(
  FILES
    CModelCommand.msg CModelStatus.msg EPickCommand.msg
)

add_service_files(
  FILES
    SetVelocity.srv
)

add_action_files(
  FILES
    EPickCommand.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
)

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS
    gazebo_ros
    control_toolbox
    xmlrpcpp
    actionlib
    actionlib_msgs
    control_msgs
    sensor_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include
)

add_library(
  gazebo_mimic_joint_plugin
    src/mimic_joint_plugin.cpp
)
target_link_libraries(
  gazebo_mimic_joint_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

add_library(
    gazebo_disable_link_plugin
    src/disable_link_plugin.cpp
)
target_link_libraries(
  gazebo_disable_link_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

add_executable(
  ft300_driver
    src/ft300_driver.cpp
)

target_link_libraries(
  ft300_driver
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/cmodel_rtu_driver.py
    scripts/cmodel_tcp_driver.py
    scripts/cmodel_urcap_driver.py
    scripts/cmodel_controller.py
    scripts/epick_controller.py
    scripts/test_cmodel_client.py
    scripts/test_epick_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(aist_model_spawner)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    geometry_msgs
    rospy
    tf
)

catkin_python_setup()

add_service_files(
  DIRECTORY
    srv
  FILES
    Add.srv
    Delete.srv
    DeleteAll.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    rospy
)

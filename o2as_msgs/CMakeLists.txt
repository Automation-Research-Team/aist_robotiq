cmake_minimum_required(VERSION 2.8.3)
project(o2as_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    cmake_modules
    geometry_msgs
    std_msgs
    sensor_msgs
    visualization_msgs
)

# Declare the message files to be built
# add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs visualization_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs visualization_msgs)

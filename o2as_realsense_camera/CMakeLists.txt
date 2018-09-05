cmake_minimum_required(VERSION 2.8.3)
project(o2as_realsense_camera)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-std=gnu++11 -O2 -fopenmp)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    message_generation
    std_srvs
    cv_bridge
    sensor_msgs
    image_transport
    pcl_conversions
    tf2_ros
)

find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

find_package(OpenCV 3 REQUIRED
    COMPONENTS
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        opencv_highgui
    CONFIG
)
message(STATUS "OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

catkin_python_setup()

add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES sensor_msgs)
catkin_package()

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES} src/o2as_realsense_camera_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node realsense2 ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

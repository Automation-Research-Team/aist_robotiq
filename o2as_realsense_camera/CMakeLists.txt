cmake_minimum_required(VERSION 2.8.3)
project(o2as_realsense_camera)

#add_compile_options(-std=c++11)
add_compile_options(-std=gnu++11 -O2 -fopenmp)

##############
## Packages ##
##############

# Catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    dynamic_reconfigure
    message_generation
    cv_bridge
)

# OpenCV
find_package(OpenCV 3 REQUIRED
    COMPONENTS
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        opencv_highgui
    CONFIG
)
message(STATUS "OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS})

# OpenMP
find_package(OpenMP)
if(NOT OpenMP_FOUND)
    message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Realsense2
find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/CameraSetting.cfg)

#########################
## message and service ##
#########################

add_service_files(FILES 
    Connect.srv
    SaveCameraInfo.srv
    SaveFrameForCadMatching.srv
)
generate_messages(DEPENDENCIES)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SOURCES
    src/util_cam.cpp
    src/o2as_realsense_camera.cpp
)
add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES} src/o2as_realsense_camera_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node realsense2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

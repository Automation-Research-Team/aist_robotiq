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
    cv_bridge
    message_generation #genmsg
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


# Service
add_service_files(FILES 
    Connect.srv
    GetFrame.srv
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

# include
include_directories(
    includerealsense_camera_multi_node
    ${catkirealsense_camera_multi_node
    #${Openrealsense_camera_multi_node
)

# lib
set(cad_LIBRARIES
    ${PROJECT_SOURCE_DIR}/lib/libcommon.so
    #${PROJECT_SOURCE_DIR}/lib/libcad.so
    ${PROJECT_SOURCE_DIR}/lib/libcad_free_avx.so
)

# util_cam
add_library(util_cam src/util_cam.cpp)realsense_camera_multi_node
add_dependencies(util_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(util_cam realsense2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# realsense_camera_node
add_executable(realsense_camera_node src/realsense_camera_node.cpp)
add_dependencies(realsense_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(realsense_camera_node util_cam realsense2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# camera_manager
add_executable(camera_manager src/camera_manager.cpp)
add_dependencies(camera_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_manager realsense2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

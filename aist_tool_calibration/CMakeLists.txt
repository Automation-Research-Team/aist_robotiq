cmake_minimum_required(VERSION 2.8.3)
project(aist_tool_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share/cmake/TU")

find_package(TULibs REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    image_transport
    std_msgs
    std_srvs
    tf
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TULIBS_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    geometry_msgs
)

add_executable(aist_tool_calibration
  src/main.cpp
)

target_link_libraries(aist_tool_calibration
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${TUTOOLSPP_LIB}
)

add_dependencies(aist_tool_calibration
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############
install(
  TARGETS
    aist_tool_calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

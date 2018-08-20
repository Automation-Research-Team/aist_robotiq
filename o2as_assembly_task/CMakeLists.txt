cmake_minimum_required(VERSION 2.8.3)
project(o2as_assembly_task)

#add_compile_options(-std=c++11)

##############
## Packages ##
##############

# Catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    o2as_vision
    o2as_skills
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp 
)

###########
## Build ##
###########

## include
#include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

#set(${PROJECT_NAME}_SOURCES
#    src/o2as_assembly_task.cpp
#)
#add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SOURCES} src/o2as_assembly_task_node.cpp)
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

catkin_python_setup()

install(
  PROGRAMS
    scripts/assembly_task.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

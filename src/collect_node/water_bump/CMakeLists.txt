cmake_minimum_required(VERSION 3.0.2)
project(water_bump)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(INSTALL_DEVEL_PATH collect_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="water_bump")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
add_library(water_bump src/water_bump.cpp)
target_link_libraries(water_bump ${catkin_LIBRARIES} dl)
target_link_libraries(water_bump base_framework)
set_target_properties(water_bump PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})

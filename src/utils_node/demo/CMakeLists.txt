cmake_minimum_required(VERSION 3.0.2)
project(demo_utils)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(INSTALL_DEVEL_PATH utils_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="demo")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
add_library(demo_utils src/demo.cpp)
target_link_libraries(demo_utils ${catkin_LIBRARIES} dl)
target_link_libraries(demo_utils base_framework big_data)
set_target_properties(demo_utils PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})

cmake_minimum_required(VERSION 3.0.2)
project(posttunnel)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(INSTALL_DEVEL_PATH collect_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="posttunnel")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
add_library(${PROJECT_NAME} 
src/Curl.cc
src/Reporter.cc
src/PostTunnel.cc
src/MessageQ.cc
src/File.cc
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} dl rt)
target_link_libraries(${PROJECT_NAME} base_framework)
target_link_libraries(${PROJECT_NAME} curl)
set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})

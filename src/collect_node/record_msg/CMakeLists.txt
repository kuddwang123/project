cmake_minimum_required(VERSION 3.0.2)
project(record_msg)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -Wno-unused-result")
set(PROJECT_NUMBER "x9" CACHE STRING "project number, such as 2312")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(INSTALL_DEVEL_PATH collect_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp)

if(${PROJECT_NUMBER} STREQUAL "x9")
  add_definitions(-DHJ_X9)
elseif(${PROJECT_NUMBER} STREQUAL "T1pro")
  add_definitions(-DHJ_T1pro)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="record_msg")


catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
add_library(record_msg 
  src/record_msg.cpp)
target_link_libraries(record_msg ${catkin_LIBRARIES} dl rt)
target_link_libraries(record_msg base_framework)
set_target_properties(record_msg PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})

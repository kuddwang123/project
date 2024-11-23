cmake_minimum_required(VERSION 3.0.2)
set (NODE_NAME log_redirect)
project(${NODE_NAME})
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
if(${RELEASE_VER} STREQUAL "debug")

elseif(${RELEASE_VER} STREQUAL "release")
  add_definitions(-DCONFIG_ENABLE_LOG=4)
endif()
#SET( CMAKE_CXX_FLAGS "-g")
find_package(catkin REQUIRED COMPONENTS rostime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
add_definitions(-DFUNCTION_NAME="${NODE_NAME}")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
add_library(${NODE_NAME} src/log_redirect.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} dl pthread)
target_link_libraries(${NODE_NAME} base_framework)
set_target_properties(${NODE_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${NODE_NAME})

install(TARGETS ${NODE_NAME}
  LIBRARY DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/platforms/${PLATFORM}/
)

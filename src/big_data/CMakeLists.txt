cmake_minimum_required(VERSION 3.0.2)
project(big_data)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
add_definitions(-DPLATFORM=${PLATFORM})
if(${RELEASE_VER} STREQUAL "debug")

elseif(${RELEASE_VER} STREQUAL "release")
  add_definitions(-DCONFIG_ENABLE_LOG=4)
  message("minos111111")
endif()
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories("include")
link_directories(${catkin_LIBRARY_DIRS})
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")

add_library(big_data  src/big_data.cpp )
target_link_libraries(big_data ${catkin_LIBRARIES} rt base_framework)

install(DIRECTORY include/
  DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/include/
)
install(TARGETS big_data
  LIBRARY DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/platforms/${PLATFORM}/
)

cmake_minimum_required(VERSION 3.0.2)
project(base_device)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
endif()
add_definitions(-DPLATFORM=${PLATFORM})
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories("include")
link_directories(${catkin_LIBRARY_DIRS})
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

add_library(base_device src/uart.cpp src/i2c.cpp)
target_link_libraries(base_device ${catkin_LIBRARIES})

install(DIRECTORY include/
  DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/include/
)
install(TARGETS base_device
  LIBRARY DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/platforms/${PLATFORM}/
)


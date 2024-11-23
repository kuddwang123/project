cmake_minimum_required(VERSION 3.0.2)
set(NODE_NAME log_recorder)
project(${NODE_NAME})
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "..")
if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()
if(${RELEASE_VER} STREQUAL "debug")

elseif(${RELEASE_VER} STREQUAL "release")
  add_definitions(-DCONFIG_ENABLE_LOG=4)
endif()
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
if(${PLATFORM} STREQUAL "amd64")
	add_definitions(-DHJ_AMD64)
  add_definitions(-DLOG_CONFIG_PATH="${CMAKE_SOURCE_DIR}/hj_manager/config/${PLATFORM}/log_recorder.json")
elseif(${PLATFORM} STREQUAL "aarch64-linux-gnu")
  add_definitions(-DHJ_AARCH64)
  add_definitions(-DLOG_CONFIG_PATH="/home/robot/data/hj/config/log_recorder.json")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rosgraph_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${NODE_NAME} rosout.cpp)
target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
  RUNTIME DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_manager/scripts/${PLATFORM}/)

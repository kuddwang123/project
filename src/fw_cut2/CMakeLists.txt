cmake_minimum_required(VERSION 3.0.2)
project(fw_cut)
set (CMAKE_CXX_STANDARD 14)
#set(INSTALL_DEVEL_PATH collect_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp std_srvs)

catkin_package(CATKIN_DEPENDS roscpp std_srvs)
set( HJ_WORKSPACE_PATH "..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
add_definitions(-DENABLE_CSTLOG)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")

add_executable(fw_cut 
src/Utils.cc
src/Main.cc
src/Schedule.cc
src/State/Unpack.cc
src/State/Upgrade.cc
)

message("======${PLATFORM}")
#link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
message("======${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} dl rt pthread curl hjlog)

set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})

cmake_minimum_required(VERSION 3.0.2)
project(slam_lib)
set (CMAKE_CXX_STANDARD 14)
set(INSTALL_DEVEL_PATH slam_node)
set(PROJECT_NUMBER "x9" CACHE STRING "project number, such as 2312")
message("Current project number is ${PROJECT_NUMBER}")
add_definitions(-DPROJECT_NUMBER=${PROJECT_NUMBER})

set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
add_definitions(-DNODE_NAME="${PROJECT_NAME}")
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(CATKIN_DEPENDS  std_msgs)

add_custom_target(
  install_slam_lib
  COMMAND
  cp -n ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/${PLATFORM}/lib*.so ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH}
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/
)

cmake_minimum_required(VERSION 3.0.2)
project(work_directory_x9)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PROJECT_NUMBER "x9" CACHE STRING "project number, such as 2312")
message("Current project number is ${PROJECT_NUMBER}")
add_definitions(-DPROJECT_NUMBER=${PROJECT_NUMBER})

set(PLATFORM "aarch64-linux-gnu" CACHE STRING "amd64, i386, arm64_rk, etc.")

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

add_custom_target(
  create_work_directory_x9
  # copy lib files
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/collect_node/*.so ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/collect_node
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/middleware_node/*.so ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/middleware_node
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/slam_node/*.so ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/slam_node
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/planning_node/*.so ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/planning_node
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/utils_node/*.so ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/utils_node

  # copy binary files
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/collect_node/collect_node ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/bin
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/middleware_node/middleware_node ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/bin
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/slam_node/slam_node ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/bin
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/utils_node/utils_node ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/bin
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/lib/planning_node/planning_node ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/bin

  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/../src/hj_manager/${PROJECT_NUMBER}/scripts/aarch64-linux-gnu/* ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/bin

  # copy shared libraries
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/../src/hj_interface/platforms/aarch64-linux-gnu/* ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib
  COMMAND
  rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/libcore_dump.so
  COMMAND
  rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/lib/liblog_redirect.so


  # copy config files
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/../src/hj_manager/${PROJECT_NUMBER}/config/aarch64-linux-gnu/* ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/config
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/../src/collect_node/collect_node/config/${PROJECT_NUMBER}/aarch64-linux-gnu/config.json ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/config/collect_node
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/../src/slam_node/slam_node/config/${PROJECT_NUMBER}/aarch64-linux-gnu/config.json ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/config/slam_node
  COMMAND
  cp -rf ${CATKIN_DEVEL_PREFIX}/../src/planning_node/planning_node/config/${PROJECT_NUMBER}/aarch64-linux-gnu/config.json ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NUMBER}/data/hj/config/planning_node

  COMMAND
  python3 getMd5.py ${PROJECT_NUMBER}

  COMMAND
  echo "finish copy"
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/
)

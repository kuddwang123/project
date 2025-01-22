cmake_minimum_required(VERSION 3.0.2)
project(base_framework)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
add_definitions(-DPLATFORM=${PLATFORM})
set(RELEASE_VER "debug" CACHE STRING "debug or release")
if(${RELEASE_VER} STREQUAL "debug")

elseif(${RELEASE_VER} STREQUAL "release")
  add_definitions(-DCONFIG_ENABLE_LOG=4)
  set(CMAKE_BUILD_TYPE "RELEASE")
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
endif()
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include/ziplib")
include_directories("include")
link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")
# add_service_files(
#   FILES
#   HealthCheckCode.srv
# )

# generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

add_library(base_framework src/node_factory.cpp src/function_factory.cpp 
          src/log_factory.cpp src/node_cache.cpp src/shm.cpp src/hj_zip.cpp src/hj_utils.cpp)
target_link_libraries(base_framework ${catkin_LIBRARIES} rt)
target_link_libraries(base_framework libzip.so)

install(DIRECTORY include/
  DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/include/
)
install(TARGETS base_framework
  LIBRARY DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/platforms/${PLATFORM}/
)
# install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/base_framework/
#         DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/include/base_framework/)


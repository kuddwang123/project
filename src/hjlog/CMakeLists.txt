cmake_minimum_required(VERSION 3.0.2)
project(hjlog)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(INSTALL_DEVEL_PATH hjlog)
include_directories(include )
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")

add_library(hjlog
src/hjlog.cc
)

target_link_libraries(hjlog boost_filesystem)

install(DIRECTORY include/
  DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/include/
)
install(TARGETS hjlog
  LIBRARY DESTINATION ${HJ_WORKSPACE_PATH}/src/hj_interface/platforms/${PLATFORM}/
)
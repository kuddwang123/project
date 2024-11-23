cmake_minimum_required(VERSION 3.0.2)
project(iot)
set (CMAKE_CXX_STANDARD 14)
set( HJ_WORKSPACE_PATH "../..")
set(PLATFORM "amd64" CACHE STRING "amd64, i386, arm64_rk, etc.")
set(INSTALL_DEVEL_PATH collect_node)
find_package(catkin REQUIRED COMPONENTS rostime roscpp)
find_package(OpenSSL REQUIRED)

include_directories(include include/ApNetConfig ${catkin_INCLUDE_DIRS})
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include/awsiot")
include_directories("${HJ_WORKSPACE_PATH}/thirdparty/include/")
include_directories("${HJ_WORKSPACE_PATH}/hj_interface/include")

link_directories(${catkin_LIBRARY_DIRS})
link_directories("${HJ_WORKSPACE_PATH}/hj_interface/platforms/${PLATFORM}")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}/awsiot")
link_directories("${HJ_WORKSPACE_PATH}/thirdparty/platforms/${PLATFORM}")

add_definitions(-DFUNCTION_NAME="iot")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

add_library(iot 
src/Persistence.cc
src/AppDataRouter.cc
src/AwsConnectionManager.cc
src/BtConnectionManager.cc
src/IotShadowClient.cc
src/IotShadowClientLink.cc
src/IotMqttClient.cc
src/IotMqttClientLink.cc
src/error.cc
src/AppCommunication.cc
src/NetConfig.cc
src/Utils.cc
src/BuryPoint.cc
src/ApNetConfig/ApConnectionManger.cc
src/ApNetConfig/Buffer.cc
src/ApNetConfig/Poll.cc
src/ApNetConfig/TcpTransport.cc
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} dl rt)
target_link_libraries(${PROJECT_NAME} base_framework)
target_link_libraries(${PROJECT_NAME} base_device)
target_link_libraries(${PROJECT_NAME} big_data)
target_link_libraries(${PROJECT_NAME} base64)
target_link_libraries(${PROJECT_NAME} crypto)
target_link_libraries(${PROJECT_NAME} curl)
target_link_libraries(${PROJECT_NAME} 
    libaws-crt-cpp.so
    libaws-c-common.so
    libaws-c-mqtt.so
    libaws-c-s3.so
    libaws-c-event-stream.so
    libaws-c-sdkutils.so
    libaws-c-http.so
    libaws-c-io.so
    libaws-c-common.so
    libaws-c-auth.so
    libaws-checksums.so
    libaws-c-compression.so
    libaws-c-cal.so
    libs2n.so
    libIotShadow-cpp.so
    OpenSSL::Crypto
    pthread
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})

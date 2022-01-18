cmake_minimum_required(VERSION 3.17)
project(ros_bridge_client)

set(CMAKE_CXX_STANDARD 11)

find_library(ZeroMQ_LIBRARY NAMES zmq PATHS ${PC_ZeroMQ_LIBRARY_DIRS})
link_libraries(${ZeroMQ_LIBRARY})

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})
link_libraries(pthread)

add_executable(${PROJECT_NAME} main.cpp ros_bridge_client.cpp ros_bridge_client.h)
target_include_directories(${PROJECT_NAME} PUBLIC ${ZeroMQ_INCLUDE_DIR})
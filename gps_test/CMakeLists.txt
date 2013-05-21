cmake_minimum_required(VERSION 2.4.6)

project(gps)

find_package(serial REQUIRED) # used for io
find_package(Boost REQUIRED)

include_directories(${serial_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
#include_directories(util)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${serial_LIBRARIES})


add_executable(gps_test gps_test.cpp)
target_link_libraries(gps_test ${serial_LIBRARIES})


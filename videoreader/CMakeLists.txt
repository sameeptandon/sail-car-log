cmake_minimum_required(VERSION 2.6)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}
                        /opt/local/lib/cmake)

message(${CMAKE_PREFIX_PATH})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-O3)

add_library(VideoReader VideoReader.cpp)
target_link_libraries(VideoReader ${OpenCV_LIBS})

add_executable( test test.cpp )
target_link_libraries( test ${OpenCV_LIBS} VideoReader)



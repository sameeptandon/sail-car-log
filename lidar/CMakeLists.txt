cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL_PCAP)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(hdl_viewer hdl_viewer.cpp gps_hdl_grabber.cpp)
target_link_libraries(hdl_viewer ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

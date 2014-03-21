cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL_PCAP)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
set(CMAKE_BUILD_TYPE Release)

if(APPLE)
    find_package(PCL 1.7 REQUIRED COMPONENTS common io)
else(APPLE)
    find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
endif(APPLE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Pcap)
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


add_library(gps_hdl_grabber gps_hdl_grabber.cpp)

if(APPLE)
else(APPLE)
add_executable(hdl_viewer hdl_viewer.cpp)
target_link_libraries(hdl_viewer gps_hdl_grabber ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
endif(APPLE)

add_executable(LDRConverter hdl_ldr_dump.cpp)
target_link_libraries(LDRConverter gps_hdl_grabber ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCAP_LIBRARY} ${Boost_LIBRARIES})

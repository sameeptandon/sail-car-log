cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(cloudproc)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Boost COMPONENTS program_options system filesystem regex python REQUIRED)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    add_definitions( "-DHAS_BOOST" )
endif()

find_package(PCL 1.7 REQUIRED COMPONENTS common io filters visualization surface features)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(src)

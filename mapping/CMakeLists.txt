cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(mapping)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(CLUSTER OFF)

find_package(PCL 1.7.1 EXACT COMPONENTS common io filters visualization surface features segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost
    COMPONENTS program_options system filesystem regex python)
if (Boost_FOUND)
    add_definitions( "-DHAS_BOOST" )
    message("--Boost Configuration--")
    message(${Boost_LIBRARIES})
    message(${Boost_INCLUDE_DIR})
    if(CLUSTER)
        include_directories("/afs/cs.stanford.edu/u/zxie/libs/include")
    endif()
endif()


add_subdirectory(src)

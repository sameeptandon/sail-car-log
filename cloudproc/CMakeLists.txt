cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(cloudproc)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Boost COMPONENTS program_options system filesystem regex REQUIRED)
if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
    add_definitions( "-DHAS_BOOST" )
endif()

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Executables

add_executable(h5_to_pcd src/h5_to_pcd.cpp)
target_link_libraries(h5_to_pcd ${PCL_LIBRARIES} hdf5 hdf5_cpp ${Boost_LIBRARIES})

add_executable(align_clouds src/align_clouds.cpp src/utils/hdf_utils.cpp src/utils/cloud_viz.cpp)
target_link_libraries(align_clouds ${PCL_LIBRARIES} ${Boost_LIBRARIES} hdf5 hdf5_cpp)

add_executable(pairwise_align src/pairwise_align.cpp)
target_link_libraries(pairwise_align ${PCL_LIBRARIES})

add_executable(downsample_cloud src/downsample_cloud.cpp)
target_link_libraries(downsample_cloud ${PCL_LIBRARIES})

add_executable(estimate_normals src/estimate_normals.cpp)
target_link_libraries(estimate_normals ${PCL_LIBRARIES})

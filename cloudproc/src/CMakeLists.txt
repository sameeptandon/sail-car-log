# Python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Libraries

add_library(h5
    utils/hdf_utils.cpp
)
target_link_libraries(h5
    hdf5
    hdf5_cpp
)

add_library(params
    parameters.cpp
)
target_link_libraries(params
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    h5
)

add_library(cloud_utils
    utils/cloud_utils.cpp
)
target_link_libraries(cloud_utils
    ${PCL_LIBRARIES}
)

add_library(cv_utils
    utils/cv_utils.cpp
)
target_link_libraries(cv_utils
    ${OpenCV_LIBRARIES}
)

# Executables

add_executable(h5_to_pcd h5_to_pcd.cpp)
target_link_libraries(h5_to_pcd ${PCL_LIBRARIES} h5 ${Boost_LIBRARIES})

add_executable(align_clouds align_clouds.cpp)
target_link_libraries(align_clouds cloud_utils ${Boost_LIBRARIES} h5)

add_executable(pairwise_align pairwise_align.cpp)
target_link_libraries(pairwise_align ${PCL_LIBRARIES})

add_executable(downsample_cloud downsample_cloud.cpp)
target_link_libraries(downsample_cloud ${PCL_LIBRARIES})

add_executable(estimate_normals estimate_normals.cpp)
target_link_libraries(estimate_normals ${PCL_LIBRARIES})

add_executable(color_cloud color_cloud.cpp)
target_link_libraries(color_cloud ${Boost_LIBRARIES} cloud_utils h5 ${OpenCV_LIBRARIES})

add_subdirectory(videoreader)

# Octomap stuff

find_package(octomap REQUIRED)
message(${OCTOMAP_INCLUDE_DIRS})
message(${OCTOMAP_LIBRARY_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

add_subdirectory(octomap)

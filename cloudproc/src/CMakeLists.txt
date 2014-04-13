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

# Executables

add_executable(h5_to_pcd h5_to_pcd.cpp)
target_link_libraries(h5_to_pcd ${PCL_LIBRARIES} h5 ${Boost_LIBRARIES})

add_executable(align_clouds align_clouds.cpp utils/cloud_utils.cpp)
target_link_libraries(align_clouds ${PCL_LIBRARIES} ${Boost_LIBRARIES} h5)

add_executable(pairwise_align pairwise_align.cpp)
target_link_libraries(pairwise_align ${PCL_LIBRARIES})

add_executable(downsample_cloud downsample_cloud.cpp)
target_link_libraries(downsample_cloud ${PCL_LIBRARIES})

add_executable(estimate_normals estimate_normals.cpp)
target_link_libraries(estimate_normals ${PCL_LIBRARIES})

add_executable(color_cloud color_cloud.cpp utils/cloud_utils.cpp)
target_link_libraries(color_cloud ${Boost_LIBRARIES} ${PCL_LIBRARIES} h5 ${OpenCV_LIBRARIES})

add_subdirectory(videoreader)

add_executable(project_lidar_on_video project_lidar_on_video.cpp utils/cv_utils.cpp utils/cloud_utils.cpp)
target_link_libraries(project_lidar_on_video ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} VideoReader params)

add_subdirectory(octomap)

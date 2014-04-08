# Executables

add_executable(h5_to_pcd h5_to_pcd.cpp)
target_link_libraries(h5_to_pcd ${PCL_LIBRARIES} hdf5 hdf5_cpp ${Boost_LIBRARIES})

add_executable(align_clouds align_clouds.cpp utils/hdf_utils.cpp utils/cloud_utils.cpp)
target_link_libraries(align_clouds ${PCL_LIBRARIES} ${Boost_LIBRARIES} hdf5 hdf5_cpp)

add_executable(pairwise_align pairwise_align.cpp)
target_link_libraries(pairwise_align ${PCL_LIBRARIES})

add_executable(downsample_cloud downsample_cloud.cpp)
target_link_libraries(downsample_cloud ${PCL_LIBRARIES})

add_executable(estimate_normals estimate_normals.cpp)
target_link_libraries(estimate_normals ${PCL_LIBRARIES})

add_subdirectory(octomap)

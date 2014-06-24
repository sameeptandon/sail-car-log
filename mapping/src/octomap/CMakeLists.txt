add_executable(build_octomap build_octomap.cpp ../utils/path_utils.cpp)
target_link_libraries(build_octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} params)

add_executable(build_color_octomap build_color_octomap.cpp ../utils/path_utils.cpp)
target_link_libraries(build_color_octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} params)

add_executable(project_filtered_lidar_on_video project_filtered_lidar_on_video.cpp ../utils/path_utils.cpp)
target_link_libraries(project_filtered_lidar_on_video ${OCTOMAP_LIBRARIES} cloud_utils cv_utils VideoReader params)

add_executable(octomap_color octomap_color.cpp ../utils/path_utils.cpp ../cloud_server.cpp)
target_link_libraries(octomap_color cloud_utils cv_utils ${OCTOMAP_LIBRARIES} VideoReader params)

add_executable(octree_to_h5 octree_to_h5.cpp)
target_link_libraries(octree_to_h5 ${OCTOMAP_LIBRARIES} params h5 ${Boost_LIBRARIES})

add_executable(octomap_filter octomap_filter.cpp)
target_link_libraries(octomap_filter ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} params)

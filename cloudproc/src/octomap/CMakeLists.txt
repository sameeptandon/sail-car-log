find_package(octomap REQUIRED)
message(${OCTOMAP_INCLUDE_DIRS})
message(${OCTOMAP_LIBRARY_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

add_executable(build_octomap build_octomap.cpp ../utils/path_utils.cpp)
target_link_libraries(build_octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} h5)

add_executable(octomap_color octomap_color.cpp ../utils/path_utils.cpp ../cloud_server.cpp ../utils/cloud_utils.cpp ../utils/cv_utils.cpp)
target_link_libraries(octomap_color ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_IO_LIBRARIES} ${OCTOMAP_LIBRARIES} VideoReader params)

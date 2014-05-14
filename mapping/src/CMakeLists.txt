# Python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# OpenCV
if(CLUSTER)
    find_package(OpenCV REQUIRED
        HINTS "/var/tmp/build/opencv")
else()
    find_package(OpenCV REQUIRED)
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

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
    ${OpenCV_LIBS}
)

# Executables

add_executable(view_pcd view_pcd.cpp)
target_link_libraries(view_pcd ${PCL_LIBRARIES})

add_executable(transform_point_cloud transform_point_cloud.cpp)
target_link_libraries(transform_point_cloud ${PCL_LIBRARIES})

add_executable(concatenate_points_pcd concatenate_points_pcd.cpp)
target_link_libraries(concatenate_points_pcd ${PCL_LIBRARIES})

add_executable(h5_to_pcd h5_to_pcd.cpp)
target_link_libraries(h5_to_pcd ${PCL_LIBRARIES} h5 ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_executable(align_clouds align_clouds.cpp)
target_link_libraries(align_clouds cloud_utils ${Boost_PROGRAM_OPTIONS_LIBRARY} h5 params)

add_executable(pairwise_align pairwise_align.cpp)
target_link_libraries(pairwise_align ${PCL_LIBRARIES})

add_executable(downsample_cloud downsample_cloud.cpp)
target_link_libraries(downsample_cloud ${PCL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_executable(estimate_normals estimate_normals.cpp)
target_link_libraries(estimate_normals ${PCL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})

add_subdirectory(videoreader)

add_executable(color_intensity color_intensity.cpp utils/color_utils.cpp)
target_link_libraries(color_intensity ${Boost_LIBRARIES} cloud_utils h5 ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

add_executable(color_cloud color_cloud.cpp utils/color_utils.cpp)
target_link_libraries(color_cloud ${Boost_LIBRARIES} cloud_utils h5 ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

# Octomap stuff

find_package(octomap 1.6.5 EXACT)
message(${OCTOMAP_INCLUDE_DIRS})
message(${OCTOMAP_LIBRARY_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

add_subdirectory(octomap)

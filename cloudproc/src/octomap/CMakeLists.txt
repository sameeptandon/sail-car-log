find_package(octomap REQUIRED)
message(${OCTOMAP_INCLUDE_DIRS})
message(${OCTOMAP_LIBRARY_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

# Python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(build_octomap build_octomap.cpp ../utils/path_utils.cpp ../utils/hdf_utils.cpp)
target_link_libraries(build_octomap ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} hdf5 hdf5_cpp)

add_executable(octomap_color octomap_color.cpp ../utils/hdf_utils.cpp ../utils/cv_utils.cpp ../utils/cloud_utils.cpp)
target_link_libraries(octomap_color ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} VideoReader ${PYTHON_LIBRARIES} hdf5 hdf5_cpp)

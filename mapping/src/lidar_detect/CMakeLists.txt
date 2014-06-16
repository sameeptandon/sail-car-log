add_executable(detect_cars detect_cars.cpp support_plane.cpp)
target_link_libraries(detect_cars ${PCL_LIBRARIES} params cloud_utils cv_utils VideoReader)

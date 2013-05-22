cmake_minimum_required(VERSION 2.4.6)

project(gps)

find_package(serial REQUIRED) # used for io
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${serial_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_})
link_directories(${serial_LIBRARIES})
link_directories(${OpenCV_LIBRARIES})


add_executable(gps_test gps_test.cpp GPSLogger.cpp)
target_link_libraries(gps_test ${serial_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})


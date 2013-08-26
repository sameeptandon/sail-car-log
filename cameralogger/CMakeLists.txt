cmake_minimum_required(VERSION 2.4.6)

project(CameraLogger)

find_path(flycap_INCLUDE_DIRS FlyCapture2.h 
    "/usr/include/flycapture")
find_library(flycap_LIBRARIES flycapture
    "/usr/lib/")
find_path(zmq_INCLUDE_DIR zmq.h
  "/usr/local/include")
find_library(zmq_LIBRARIES zmq
  "/usr/local/lib")

find_package(serial REQUIRED) # used for io
find_package(Boost COMPONENTS program_options thread)
find_package(OpenCV REQUIRED)

include_directories(${serial_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${flycap_INCLUDE_DIRS})
include_directories(${zmq_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(CameraLogger CameraLogger.cpp GPSLogger.cpp GPSRecord.cpp)
target_link_libraries(CameraLogger ${serial_LIBRARIES} 
    ${Boost_LIBRARIES} ${OpenCV_LIBS} ${flycap_LIBRARIES}
    ${zmq_LIBRARIES})
 

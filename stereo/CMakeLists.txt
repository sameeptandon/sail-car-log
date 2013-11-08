cmake_minimum_required(VERSION 2.8.7)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


project(Stereo)

# flycap
find_path(flycap_INCLUDE_DIRS FlyCapture2.h "/usr/include/flycapture")
find_library(flycap_LIBRARIES flycapture "/usr/lib/")

# triclops
find_path(triclops triclops.h "/usr/local/include/triclops")
find_library(triclops_LIBRARIES triclops "/usr/local/lib/")

# pnmutils
find_path(pnmutils pnmutils.h "/usr/local/include/pnmutils")
find_library(pnmutils_LIBRARIES pnmutils "/usr/local/lib/")

# zmq
find_path(zmq_INCLUDE_DIR zmq.h "/usr/include")
find_library(zmq_LIBRARIES zmq "/usr/lib")

# libelas
set (LIBELAS_SRC_DIR libelas)
SET(CMAKE_CXX_FLAGS "-msse3")
FILE(GLOB LIBELAS_SRC_FILES "libelas/*.cpp")

#find_package(serial REQUIRED) # used for io
find_package(Boost COMPONENTS program_options thread)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${flycap_INCLUDE_DIRS})
include_directories(${triclops_INCLUDE_DIRS})
include_directories(${zmq_INCLUDE_DIR})
include_directories(${LIBELAS_SRC_DIR})
include_directories("../cameralogger/")

add_executable(bumblebee_grab bumblebee_grab.cpp )
target_link_libraries(bumblebee_grab ${Boost_LIBRARIES} ${OpenCV_LIBS} ${flycap_LIBRARIES}
  ${zmq_LIBRARIES})

add_executable(bumblebee_stereo bumblebee_stereo.cpp lodepng.cpp ${LIBELAS_SRC_FILES})
target_link_libraries(bumblebee_stereo ${OpenCV_LIBS} ${flycap_LIBRARIES}
#target_link_libraries(bumblebee_stereo ${OpenCV_LIBS} ${flycap_LIBRARIES}
  ${triclops_LIBRARIES} ${pnmutils_LIBRARIES} dl)

add_executable(video2png video2png.cpp)
target_link_libraries(video2png ${OpenCV_LIBS} ${flycap_LIBRARIES}
  ${zmq_LIBRARIES})









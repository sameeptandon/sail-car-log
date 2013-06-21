cmake_minimum_required(VERSION 2.6)
project(visualizer)

find_package(osg REQUIRED)
find_package(osgUtil REQUIRED)
find_package(osgViewer REQUIRED)
find_package(osgGA REQUIRED)
find_package(OpenThreads REQUIRED) 
find_package(Boost COMPONENTS program_options thread)

set(OSG_INCLUDE_DIRS  ${OSG_INCLUDE_DIR}
                      ${OSGUTIL_INCLUDE_DIR}
                      ${OSGVIEWER_INCLUDE_DIR}
                      ${OSGGA_INCLUDE_DIR}
                      ${OPENTHREADS_INCLUDE_DIR}
)

set(OSG_LIBS ${OSG_LIBRARIES}
             ${OSGUTIL_LIBRARIES}
             ${OSGVIEWER_LIBRARIES}
             ${OSGGA_LIBRARIES}
             ${OPENTHREADS_LIBRARIES}
)


include_directories(${Boost_INCLUDE_DIRS})

add_subdirectory(io)
add_subdirectory(render)

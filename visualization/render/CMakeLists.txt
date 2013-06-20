include_directories(${OSG_INCLUDE_DIRS})

add_executable(osg_demo osg_demo.cpp)
target_link_libraries(osg_demo ${OSG_LIBS})


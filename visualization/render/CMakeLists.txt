include_directories(${OSG_INCLUDE_DIRS})

add_executable(osg_demo osg_demo.cpp)
target_link_libraries(osg_demo ${OSG_LIBS})

add_executable(viewer viewer.cpp)
target_link_libraries(viewer ${OSG_LIBS} io ${Boost_LIBRARIES})

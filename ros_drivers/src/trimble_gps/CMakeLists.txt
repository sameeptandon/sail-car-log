cmake_minimum_requireD(VERSION 2.8.3)
project(trimble_gps)

find_package(catkin REQUIRED COMPONENTS 
    std_msgs
    geometry_msgs
    message_generation
    roscpp)

add_message_files(
    FILES
    GpsState.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()

add_executable(trimble_gps_publisher
  trimble_gps_publisher.cpp trimble_gps.cpp
)

add_executable(gps_converter
  gps_converter.cpp Wgs2Htz.cpp
)

#add_executable(gps_converter_BL
#  gps_converter_BL.cpp Wgs2Htz.cpp
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(dlc_svcs_node dlc_svcs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trimble_gps_publisher
   ${catkin_LIBRARIES}
)

target_link_libraries(gps_converter
   ${catkin_LIBRARIES}
)

#target_link_libraries(gps_converter_BL
#   ${catkin_LIBRARIES}
#)


## Mark executables and/or libraries for installation
install(TARGETS trimble_gps_publisher gps_converter 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2>
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

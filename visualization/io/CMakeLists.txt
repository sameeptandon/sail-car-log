
add_library(io GPSRecord.cpp)

add_executable(test_gps_parser test_gps_parser.cpp)
target_link_libraries(test_gps_parser io ${Boost_LIBRARIES}) 

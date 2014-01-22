# Installing PCL

Installing from the prebuilt binaries caused problems. I was able to fix this by installing PCL from source.

## Dependencies

    sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev libvtk5-dev

## Source

    git clone https://github.com/PointCloudLibrary/pcl.git

## Compiling

    dir build && cd build

### Default

    cmake ..
    make
    sudo make install

### Custom

    ccmake ..
    <enter> to toggle library functionality
    <c> to configure
    <g> to generate
    cmake ..
    make
    sudo make install

## Velodyne (HDL-32e)

There is an hdl viewer built into PCL. An executable will be written to build/bin/pcl\_hdl\_viewer\_simple. The source for this example is in visualization/tools/hdl\_viewer\_simple.cpp

If you want to move the hdl\_viewer\_simple.cpp out of the source folder, use this as your CMakeLists.txt:


    cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
    project(PCL_PCAP)
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeScripts)
    
    find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    add_executable(pcl_hdl_viewer_simple pcl_hdl_viewer_simple.cpp)
    target_link_libraries(pcl_hdl_viewer_simple ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
cmake_minimum_required(VERSION 2.8)
project(path_controler)

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})
generate_messages(DEPENDENCIES sensor_msgs)
find_package(PCL 1.7 REQUIRED)
catkin_package()

find_package(OpenCV)
include_directories(
include/
include ${OpenCV_INCLUDE_DIRS}
include ${PCL_INCLUDE_DIRS})


add_definitions(-std=c++11 -fPIC -pedantic -g -O0 -Wno-missing-field-initializers)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

# add the publisher example
add_executable(path_controler src/path_controler.cpp)
target_link_libraries(path_controler -lrealsense2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8)
project(imgSaver)

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs)
generate_messages(DEPENDENCIES sensor_msgs)


include_directories(${catkin_INCLUDE_DIRS})


catkin_package()
find_package(OpenCV)


include_directories(
include ${OpenCV_INCLUDE_DIRS}
)

add_definitions(-std=c++11 -fPIC -pedantic -g -O0 -Wno-missing-field-initializers)

add_executable(imgSaver src/imgsaver.cpp)
target_link_libraries(imgSaver -lrealsense2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )



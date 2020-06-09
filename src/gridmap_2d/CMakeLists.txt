cmake_minimum_required(VERSION 2.8.3)
project(gridmap_2d)

find_package(catkin REQUIRED COMPONENTS nav_msgs)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/GridMap2D.cpp)

#define some target ...
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

cmake_minimum_required(VERSION 2.8.3)
project(humanoid_planner_2d)

find_package(catkin REQUIRED geometry_msgs gridmap_2d roscpp)

## CMake config:
#find_package(sbpl REQUIRED)
## pkg-config:
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
find_package(OpenCV REQUIRED)

include_directories(${SBPL_INCLUDE_DIRS})
# not needed for SBPL with CMake config
link_directories(${SBPL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS sbpl  
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_definitions(-g -O0)
add_library(${PROJECT_NAME} src/SBPLPlanner2D.cpp)
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(sbpl_2d_planner_node src/humanoid_planner_2d.cpp)
target_link_libraries(sbpl_2d_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES})
#mylistener
#add_executable(mylistener src/mylistener.cpp)
#target_link_libraries(mylistener ${PROJECT_NAME} ${catkin_LIBRARIES})

# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS sbpl_2d_planner_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

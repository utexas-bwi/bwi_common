cmake_minimum_required(VERSION 2.8.3)
project(bwi_scavenger)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  bwi_kr_execution
  pcl_ros
  cv_bridge
  image_transport
  message_generation
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bwi_scavenger
#   src/${PROJECT_NAME}/bwi_scavenger.cpp
# )

## Declare a cpp executable

add_executable(scavenger src/scavenger.cpp src/ScavTaskColorShirt.cpp 
    src/ScavTaskFetchObject.cpp src/ScavTaskWhiteBoard.cpp src/TaskManager.cpp
    src/SearchPlanner.cpp)
target_link_libraries(scavenger ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


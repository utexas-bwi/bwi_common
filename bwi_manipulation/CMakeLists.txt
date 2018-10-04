cmake_minimum_required(VERSION 2.8.3)
project(bwi_manipulation)

set(CMAKE_CXX_STANDARD 11)

# We'll ignore any missing packages at this point and
# rely on compile errors to highlight any real problems
find_package(catkin QUIET COMPONENTS
        actionlib
        actionlib_msgs
        agile_grasp
        bwi_perception
        geometry_msgs
        pcl_ros
        pcl_conversions
        roscpp
        rospy
        std_msgs
        sensor_msgs
        tf_conversions
        )


if (agile_grasp_FOUND)
    # Set a compile define to turn on agile grasp helpers
    message("Found agile_grasp. Additional helpers will be compiled.")
    add_definitions(-DAGILE_GRASP_AVAILABLE)
endif()

find_package(PCL 1.7 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS actionlib actionlib_msgs bwi_moveit_utils bwi_perception geometry_msgs moveit_ros_planning_interface message_runtime moveit_msgs roscpp rospy std_msgs
        INCLUDE_DIRS include
        LIBRARIES manipulation_utilities
)

add_library(manipulation_utilities src/grasp_utils.cpp src/GraspCartesianCommand.cpp src/ArmPositionDB.cpp)
target_link_libraries(manipulation_utilities ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

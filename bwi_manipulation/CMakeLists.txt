cmake_minimum_required(VERSION 2.8.3)
project(bwi_manipulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        agile_grasp
        geometry_msgs
        pcl_ros
        pcl_conversions
        roscpp
        rospy
        std_msgs
        sensor_msgs
        tf_conversions
        )


find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS actionlib actionlib_msgs agile_grasp bwi_moveit_utils geometry_msgs moveit_ros_planning_interface message_runtime moveit_msgs roscpp rospy std_msgs
        INCLUDE_DIRS include
        LIBRARIES manipulation_utilities
)

add_library(manipulation_utilities src/grasp_utils.cpp src/GraspCartesianCommand.cpp src/ArmPositionDB.cpp)
target_link_libraries(manipulation_utilities ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

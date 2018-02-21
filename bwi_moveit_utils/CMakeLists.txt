cmake_minimum_required(VERSION 2.8.3)
project(bwi_moveit_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    bwi_msgs
    control_msgs
    geometry_msgs
    message_generation
    moveit_msgs
    moveit_core
    moveit_ros_planning_interface
    pluginlib
    roscpp
    rospy
    rosbag
    sensor_msgs
    std_msgs
    std_srvs
    tf
    trajectory_msgs
  )
find_package(Boost REQUIRED thread)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_service_files(
   FILES
   MoveitCartesianPose.srv
   MoveitJointPose.srv
   MoveitWaypoint.srv
)


generate_messages(
  DEPENDENCIES
  moveit_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS} )


add_executable(moveit_joint_pose_service src/moveit_joint_pose_service.cpp)
add_executable(moveit_cartesian_pose_service src/moveit_cartesian_pose_service.cpp)
add_executable(moveit_waypoint_service src/moveit_waypoint_service.cpp)

add_dependencies(moveit_joint_pose_service bwi_moveit_utils_gencpp)

add_dependencies(moveit_cartesian_pose_service bwi_moveit_utils_gencpp)
add_dependencies(moveit_waypoint_service bwi_moveit_utils_gencpp)

target_link_libraries(moveit_joint_pose_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(moveit_cartesian_pose_service ${catkin_LIBRARIES})
target_link_libraries(moveit_waypoint_service ${catkin_LIBRARIES})

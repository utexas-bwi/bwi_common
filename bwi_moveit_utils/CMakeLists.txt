cmake_minimum_required(VERSION 2.8.3)
project(bwi_moveit_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    bwi_msgs
    control_msgs
    geometry_msgs
    kinova_msgs
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
   MicoNavSafety.srv
   MicoMoveitCartesianPose.srv
   MicoMoveitJointPose.srv
   MicoMoveitWaypoint.srv
   AngularVelCtrl.srv
)

add_action_files(
   FILES
   FollowVelocityTrajectory.action
)

generate_messages(
  DEPENDENCIES
  moveit_msgs
  kinova_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS} )


add_executable(mico_moveit_joint_pose_service src/mico_moveit_joint_pose_service.cpp)
add_executable(mico_nav_safety_service src/mico_nav_safety_service.cpp)
add_executable(mico_moveit_cartesian_pose_service src/mico_moveit_cartesian_pose_service.cpp)
add_executable(mico_moveit_waypoint_service src/mico_moveit_waypoint_service.cpp)
add_executable(trajectory_record src/trajectory_record.cpp)
add_executable(position_record src/position_record.cpp)
add_executable(angular_vel_control_srv src/angular_vel_control_srv.cpp)

add_dependencies(mico_moveit_joint_pose_service bwi_moveit_utils_gencpp)
add_dependencies(mico_nav_safety_service bwi_moveit_utils_gencpp ${catkin_EXPORTED_TARGETS})
add_dependencies(mico_moveit_cartesian_pose_service bwi_moveit_utils_gencpp)
add_dependencies(mico_moveit_waypoint_service bwi_moveit_utils_gencpp)
add_dependencies(trajectory_record bwi_moveit_utils_gencpp)
add_dependencies(position_record bwi_moveit_utils_gencpp)
add_dependencies(angular_vel_control_srv bwi_moveit_utils_gencpp)

target_link_libraries(mico_moveit_joint_pose_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(mico_nav_safety_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(mico_moveit_cartesian_pose_service ${catkin_LIBRARIES})
target_link_libraries(mico_moveit_waypoint_service ${catkin_LIBRARIES})
target_link_libraries(trajectory_record ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(position_record ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(angular_vel_control_srv ${catkin_LIBRARIES})

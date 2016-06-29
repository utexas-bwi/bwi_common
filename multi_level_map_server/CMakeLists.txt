cmake_minimum_required(VERSION 2.8.3)
project(multi_level_map_server)

find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    nav_msgs
    multi_level_map_msgs
    multi_level_map_utils
    rospy
    tf
)

# install launch files, maps and the python script 
install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS nodes/multi_level_map_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

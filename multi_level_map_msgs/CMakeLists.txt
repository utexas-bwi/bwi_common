# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(multi_level_map_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs nav_msgs)

#uncomment if you have defined messages
add_message_files(
  DIRECTORY msg
  FILES LevelMetaData.msg  MultiLevelMapData.msg  MultiLevelMapLink.msg  MultiLevelMapPoint.msg
)

add_service_files(
  DIRECTORY srv
  FILES ChangeCurrentLevel.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs geometry_msgs
)

cmake_minimum_required(VERSION 2.8.3)
project(bwi_msgs)

find_package(catkin REQUIRED 
  COMPONENTS 
    actionlib_msgs
    bwi_planning_common 
    message_generation
)

add_service_files(
  DIRECTORY srv
  FILES QuestionDialog.srv
)

add_action_files(
  DIRECTORY action
  FILES LogicalNavigation.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs bwi_planning_common 
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    bwi_planning_common
    message_runtime
)

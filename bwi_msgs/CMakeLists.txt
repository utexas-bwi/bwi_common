cmake_minimum_required(VERSION 2.8.3)
project(bwi_msgs)

find_package(catkin REQUIRED 
  COMPONENTS 
    actionlib_msgs
    bwi_planning_common 
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    AvailableRobot.msg
    AvailableRobotArray.msg
    StopBaseStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    DoorHandlerInterface.srv
    QuestionDialog.srv
    ResolveChangeFloor.srv
    SemanticParser.srv
    StopBase.srv
)

add_action_files(
  DIRECTORY action
  FILES LogicalNavigation.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs bwi_planning_common geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    bwi_planning_common
    geometry_msgs
    message_runtime
    std_msgs
)

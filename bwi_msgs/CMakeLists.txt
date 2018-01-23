cmake_minimum_required(VERSION 2.8.3)
project(bwi_msgs)

find_package(catkin REQUIRED 
  COMPONENTS 
    actionlib_msgs
    bwi_planning_common 
    geometry_msgs
    message_generation
    sensor_msgs
    std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    AvailableRobot.msg
    AvailableRobotArray.msg
    LEDActionResult.msg
    LEDAnimations.msg
    LEDCameraStatus.msg
    LEDTestType.msg
    StopBaseStatus.msg
    ScavStatus.msg
    WeekTime.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    ActionUpdate.srv
    DoorHandlerInterface.srv
    GetNextImage.srv
    ImageQuestion.srv
    LEDClear.srv
    LEDSetCamera.srv
    LEDTestStrip.srv
    LocationAction.srv
    NextLocation.srv
    QuestionDialog.srv
    ResolveChangeFloor.srv
    RobotTeleporterInterface.srv
    SaveImageResponse.srv
    SemanticParser.srv
    StopBase.srv
    ScavHunt.srv
    Trigger.srv
    UploadImage.srv
    UpdateObject.srv
)

add_action_files(
  DIRECTORY action
  FILES
    LEDControl.action
    LogicalNavigation.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs bwi_planning_common geometry_msgs sensor_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    bwi_planning_common
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)

cmake_minimum_required(VERSION 2.8.3)
project(bwi_led)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roscpp
  serial
  std_msgs
)

add_definitions(-std=c++11)

add_service_files(FILES 
                  led_clear.srv
                  set_camera_off.srv
                  set_camera_on.srv
                  set_every_fifth.srv
                  set_first_five.srv
                  test_strip.srv)

add_action_files(DIRECTORY action
                 FILES LED.action)

add_message_files(DIRECTORY msg 
                  FILES led_animations.msg)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS ledcomm
  # CATKIN_DEPENDS actionlib_msgs roscpp serial std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ledcomm
)

add_executable(led_control_server src/led_control_server.cpp)
add_dependencies(led_control_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(led_control_server ${catkin_LIBRARIES})

install(TARGETS
          led_control_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
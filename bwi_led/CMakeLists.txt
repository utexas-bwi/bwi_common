cmake_minimum_required(VERSION 2.8.3)
project(bwi_led)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_definitions(-std=c++11)

add_service_files(FILES 
				  led_clear.srv
				  test_strip.srv
				  set_first_five.srv
				  set_every_fifth.srv)

add_action_files(DIRECTORY action
                 FILES LED.action)


add_message_files(DIRECTORY msg 
                  FILES led_animations.msg)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS ledcomm
  CATKIN_DEPENDS roscpp serial std_msgs actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ledcomm
)

add_executable(led_control_server src/led_control_server.cpp)
add_dependencies(led_control_server roscpp)
target_link_libraries(led_control_server ${catkin_LIBRARIES})

add_executable(test_ledcom_server src/test.cpp)
target_link_libraries(test_ledcom_server ${catkin_LIBRARIES})
add_dependencies(test_ledcom_server ${bwi_led_EXPORTED_TARGETS})
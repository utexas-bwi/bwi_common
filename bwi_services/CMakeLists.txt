cmake_minimum_required(VERSION 2.8.3)
project(bwi_services)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  message_generation 
  bwi_kr_execution
  actionlib
)

add_service_files(
  FILES
  DeliverMessage.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(deliver_message src/deliver_message.cpp)
add_dependencies(deliver_message ${catkin_EXPORTED_TARGETS})
target_link_libraries(deliver_message ${catkin_LIBRARIES})

add_executable(deliver_message_client src/deliver_message_client.cpp)
add_dependencies(deliver_message_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(deliver_message_client ${catkin_LIBRARIES})

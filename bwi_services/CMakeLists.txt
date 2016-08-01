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
  GoBesideLocation.srv
  GoToLocation.srv
  SpeakMessage.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(deliver_message_service src/deliver_message_service.cpp)
add_dependencies(deliver_message_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(deliver_message_service ${catkin_LIBRARIES})

add_executable(go_to_location_service src/go_to_location_service.cpp)
add_dependencies(go_to_location_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_to_location_service ${catkin_LIBRARIES})

add_executable(go_beside_location_service src/go_beside_location_service.cpp)
add_dependencies(go_beside_location_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_beside_location_service ${catkin_LIBRARIES})

add_executable(speak_message_service src/speak_message_service.cpp)
add_dependencies(speak_message_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(speak_message_service ${catkin_LIBRARIES})

add_executable(deliver_message_client src/deliver_message_client.cpp)
add_dependencies(deliver_message_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(deliver_message_client ${catkin_LIBRARIES})

add_executable(speak_message_client src/speak_message_client.cpp)
add_dependencies(speak_message_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(speak_message_client ${catkin_LIBRARIES})

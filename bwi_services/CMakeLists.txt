cmake_minimum_required(VERSION 2.8.3)
project(bwi_services)

find_package(catkin REQUIRED COMPONENTS
  bwi_kr_execution
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package(

)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(deliver_message src/deliver_message.cpp)
add_dependencies(deliver_message ${catkin_EXPORTED_TARGETS})
target_link_libraries(deliver_message ${catkin_LIBRARIES})

add_service_files(
  FILES
  DeliverMessage.srv
)

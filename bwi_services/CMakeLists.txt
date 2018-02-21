cmake_minimum_required(VERSION 2.8.3)
project(bwi_services)

find_package(catkin REQUIRED COMPONENTS 
  actionlib
  plan_execution
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
  DeliverMessage.srv
  GoToLocation.srv
  SpeakMessage.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    plan_execution
    message_runtime
    roscpp
    std_msgs
)

# Build
include_directories(${catkin_INCLUDE_DIRS})

add_executable(deliver_message_service src/deliver_message_service.cpp)
add_dependencies(deliver_message_service ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(deliver_message_service ${catkin_LIBRARIES})

add_executable(go_to_location_service src/go_to_location_service.cpp)
add_dependencies(go_to_location_service ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(go_to_location_service ${catkin_LIBRARIES})

add_executable(speak_message_service src/speak_message_service.cpp)
add_dependencies(speak_message_service ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(speak_message_service ${catkin_LIBRARIES})

# Install
install(TARGETS
          deliver_message_service
          go_to_location_service
          speak_message_service
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

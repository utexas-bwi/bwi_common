cmake_minimum_required(VERSION 2.8.3)
project(bwi_virtour)

find_package(catkin REQUIRED COMPONENTS 
  actionlib
  roscpp
  message_generation
  bwi_kr_execution
)

add_service_files(
  FILES
  GoToLocation.srv
  GoBesideLocation.srv
  Rotate.srv
  RequestTour.srv
  PingTour.srv
  GetTourState.srv
  LeaveTour.srv
  Authenticate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp actionlib message_generation bwi_kr_execution
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(go_to_location_service_node src/go_to_location_service.cpp)
add_dependencies(go_to_location_service_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(go_to_location_service_node ${catkin_LIBRARIES})

add_executable(rotate_service_node src/rotateService.cpp)
add_dependencies(rotate_service_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rotate_service_node ${catkin_LIBRARIES})

add_executable(tour_manager src/tour_manager.cpp)
add_dependencies(tour_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tour_manager ${catkin_LIBRARIES})

install(TARGETS go_to_location_service_node rotate_service_node tour_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY scripts/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

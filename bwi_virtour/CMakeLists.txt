cmake_minimum_required(VERSION 2.8.3)
project(bwi_virtour)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    plan_execution
    message_generation
    roscpp
    std_msgs
)

# Service generation
add_service_files(
  FILES
    Authenticate.srv
    GetTourState.srv
    GoBesideLocation.srv
    GoToLocation.srv
    LeaveTour.srv
    PingTour.srv
    RequestTour.srv
    Rotate.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    plan_execution
    message_runtime
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS})

# Build
add_executable(go_to_location_service_node src/go_to_location_service.cpp)
add_dependencies(go_to_location_service_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(go_to_location_service_node ${catkin_LIBRARIES})

add_executable(rotate_service_node src/rotateService.cpp)
add_dependencies(rotate_service_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rotate_service_node ${catkin_LIBRARIES})

add_executable(tour_manager src/tour_manager.cpp)
add_dependencies(tour_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tour_manager ${catkin_LIBRARIES})

# Install
install(TARGETS
          go_to_location_service_node
          rotate_service_node
          tour_manager
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY scripts/
        USE_SOURCE_PERMISSIONS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Test launch scripts
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

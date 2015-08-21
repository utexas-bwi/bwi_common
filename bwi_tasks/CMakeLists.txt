cmake_minimum_required(VERSION 2.8.3)
project(bwi_tasks)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_kr_execution
  roscpp
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(back_and_forth_node src/back_and_forth.cpp)
add_dependencies(back_and_forth_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_and_forth_node ${catkin_LIBRARIES})

add_executable(between_doors src/between_doors.cpp)
add_dependencies(between_doors ${catkin_EXPORTED_TARGETS})
target_link_libraries(between_doors ${catkin_LIBRARIES})

add_executable(go_to_location_node src/go_to_location.cpp)
add_dependencies(go_to_location_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_to_location_node ${catkin_LIBRARIES})

add_executable(learning_tasks_node src/learning_tasks.cpp)
add_dependencies(learning_tasks_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(learning_tasks_node ${catkin_LIBRARIES})

add_executable(look_for_person_node src/look_for_person.cpp)
add_dependencies(look_for_person_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(look_for_person_node ${catkin_LIBRARIES})

add_executable(meeting_experiment src/meeting_experiment.cpp)
add_dependencies(meeting_experiment ${catkin_EXPORTED_TARGETS})
target_link_libraries(meeting_experiment ${catkin_LIBRARIES})

add_executable(simulator_learning_tasks_node src/simulator_learning_tasks.cpp)
add_dependencies(simulator_learning_tasks_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator_learning_tasks_node ${catkin_LIBRARIES})

add_executable(visit_door_list src/visit_door_list.cpp)
add_dependencies(visit_door_list ${catkin_EXPORTED_TARGETS})
target_link_libraries(visit_door_list ${catkin_LIBRARIES})

install(TARGETS
          back_and_forth_node
          between_doors
          go_to_location_node
          learning_tasks_node
          look_for_person_node
          meeting_experiment
          simulator_learning_tasks_node
          visit_door_list
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)
project(bwi_tasks)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bwi_kr_execution
  bwi_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs bwi_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(back_and_forth_node src/back_and_forth.cpp)
add_dependencies(back_and_forth_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_and_forth_node ${catkin_LIBRARIES})

add_executable(between_doors src/between_doors.cpp)
add_dependencies(between_doors ${catkin_EXPORTED_TARGETS})
target_link_libraries(between_doors ${catkin_LIBRARIES})

add_executable(filtering_tasks src/test_filter_task.cpp)
add_dependencies(filtering_tasks ${catkin_EXPORTED_TARGETS})
target_link_libraries(filtering_tasks ${catkin_LIBRARIES})

add_executable(go_to_door_node src/go_to_door.cpp)
add_dependencies(go_to_door_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_to_door_node ${catkin_LIBRARIES})

add_executable(go_to_location_node src/go_to_location.cpp)
add_dependencies(go_to_location_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_to_location_node ${catkin_LIBRARIES})

add_executable(go_to_object_node src/go_to_object.cpp)
add_dependencies(go_to_object_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_to_object_node ${catkin_LIBRARIES})

add_executable(learning_tasks_node src/learning_tasks.cpp)
add_dependencies(learning_tasks_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(learning_tasks_node ${catkin_LIBRARIES})

add_executable(look_for_person_node src/look_for_person.cpp)
add_dependencies(look_for_person_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(look_for_person_node ${catkin_LIBRARIES})

add_executable(meeting_experiment src/meeting_experiment.cpp)
add_dependencies(meeting_experiment ${catkin_EXPORTED_TARGETS})
target_link_libraries(meeting_experiment ${catkin_LIBRARIES})

add_executable(message_delivery src/message_delivery.cpp)
add_dependencies(message_delivery ${catkin_EXPORTED_TARGETS})
target_link_libraries(message_delivery ${catkin_LIBRARIES})

add_executable(simulator_learning_tasks_node src/simulator_learning_tasks.cpp)
add_dependencies(simulator_learning_tasks_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator_learning_tasks_node ${catkin_LIBRARIES})

add_executable(visit_door_list src/visit_door_list.cpp)
add_dependencies(visit_door_list ${catkin_EXPORTED_TARGETS})
target_link_libraries(visit_door_list ${catkin_LIBRARIES})

add_executable(visit_door_list_gui src/visit_door_list_gui.cpp)
add_dependencies(visit_door_list_gui ${catkin_EXPORTED_TARGETS})
target_link_libraries(visit_door_list_gui ${catkin_LIBRARIES})

install(TARGETS
          back_and_forth_node
          between_doors
          filtering_tasks
          go_to_door_node
          go_to_location_node
          go_to_object_node
          learning_tasks_node
          look_for_person_node
          meeting_experiment
          message_delivery
          simulator_learning_tasks_node
          visit_door_list
          visit_door_list_gui
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


cmake_minimum_required(VERSION 2.8.3)
project(bwi_logical_translator)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    bwi_mapper
    bwi_msgs
    bwi_planning_common
    dynamic_reconfigure
    map_msgs
    message_filters
    move_base_msgs
    multi_level_map_msgs
    multi_level_map_utils
    nav_msgs
    roscpp
    tf
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    bwi_mapper
    bwi_msgs
    bwi_planning_common
    dynamic_reconfigure
    map_msgs
    message_filters
    move_base_msgs
    multi_level_map_msgs
    multi_level_map_utils
    nav_msgs
    roscpp
    tf
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/libbwi_logical_translator/bwi_logical_translator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(bwi_logical_navigator src/nodes/bwi_logical_navigator.cpp)
target_link_libraries(bwi_logical_navigator ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(bwi_logical_navigator ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS  bwi_logical_navigator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


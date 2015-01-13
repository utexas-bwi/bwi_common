cmake_minimum_required(VERSION 2.8.3)
project(bwi_planning_common)

find_package(catkin REQUIRED COMPONENTS bwi_mapper bwi_tools message_generation std_msgs)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

catkin_python_setup()

add_message_files(
  FILES
  PlannerAtom.msg
)

add_service_files(
  FILES
  PlannerInterface.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  DEPENDS 
    Boost
    OpenCV
  CATKIN_DEPENDS
    bwi_mapper
    bwi_tools
    message_runtime
    std_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/structures.cpp
)
target_link_libraries(${PROJECT_NAME}
  SDL
  SDL_image
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(${PROJECT_NAME}/mark_locations src/mark_locations.cpp)
set_target_properties(${PROJECT_NAME}/mark_locations PROPERTIES OUTPUT_NAME "mark_locations")
target_link_libraries(${PROJECT_NAME}/mark_locations ${PROJECT_NAME})

add_executable(${PROJECT_NAME}/merge_locations src/merge_locations.cpp)
set_target_properties(${PROJECT_NAME}/merge_locations PROPERTIES OUTPUT_NAME "merge_locations")
target_link_libraries(${PROJECT_NAME}/merge_locations ${PROJECT_NAME})

add_executable(${PROJECT_NAME}/mark_doors src/mark_doors.cpp)
set_target_properties(${PROJECT_NAME}/mark_doors PROPERTIES OUTPUT_NAME "mark_doors")
target_link_libraries(${PROJECT_NAME}/mark_doors ${PROJECT_NAME})

  
install(TARGETS 
  ${PROJECT_NAME} 
  ${PROJECT_NAME}/mark_doors 
  ${PROJECT_NAME}/mark_locations
  ${PROJECT_NAME}/merge_locations
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
 
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/logical_marker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# unit tests are enabled selectively
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

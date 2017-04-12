# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_mapper)

## Find catkin and external packages
find_package(catkin REQUIRED 
  COMPONENTS
    cv_bridge
    map_server
    nav_msgs
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS 
    OpenCV 
  CATKIN_DEPENDS 
    map_server
    nav_msgs 
  INCLUDE_DIRS 
    include
  LIBRARIES 
    bwi_mapper
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#main library
add_library(bwi_mapper 
  src/libbwi_mapper/circle_provider.cpp 
  src/libbwi_mapper/map_inflator.cpp 
  src/libbwi_mapper/map_loader.cpp
  src/libbwi_mapper/map_utils.cpp
  src/libbwi_mapper/directed_dfs.cpp
  src/libbwi_mapper/path_finder.cpp
  src/libbwi_mapper/connected_components.cpp
  src/libbwi_mapper/voronoi_approximator.cpp
  src/libbwi_mapper/topological_mapper.cpp
  src/libbwi_mapper/graph.cpp
  src/libbwi_mapper/point_utils.cpp
  src/libbwi_mapper/structures/point.cpp
  src/libbwi_mapper/structures/voronoi_point.cpp
)

target_link_libraries(bwi_mapper 
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

#binaries
set(node_targets)
foreach(node generate_graph view_graph prepare_graph) 
  add_executable(${PROJECT_NAME}_${node} src/nodes/${node}.cpp)
  set_target_properties(${PROJECT_NAME}_${node} PROPERTIES OUTPUT_NAME "${node}")
  target_link_libraries(${PROJECT_NAME}_${node} bwi_mapper)
  list(APPEND node_targets ${PROJECT_NAME}_${node})
endforeach()

#test binaries
foreach(node test_circle test_map_loader test_voronoi test_dfs test_graph)
  add_executable(${PROJECT_NAME}_${node} test/${node}.cpp)
  set_target_properties(${PROJECT_NAME}_${node} PROPERTIES OUTPUT_NAME "${node}")
  target_link_libraries(${PROJECT_NAME}_${node} bwi_mapper)
  list(APPEND node_targets ${PROJECT_NAME}_${node})
endforeach()

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

install(TARGETS
  ${PROJECT_NAME}
  ${node_targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


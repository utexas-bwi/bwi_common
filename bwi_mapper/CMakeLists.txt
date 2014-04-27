# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_mapper)

## Find catkin and external packages
find_package(catkin REQUIRED 
  COMPONENTS 
    map_server
    nav_msgs 
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

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
add_executable(generate_graph src/nodes/generate_graph.cpp)
target_link_libraries(generate_graph bwi_mapper)

add_executable(view_graph src/nodes/view_graph.cpp)
target_link_libraries(view_graph bwi_mapper)

add_executable(prepare_graph src/nodes/prepare_graph.cpp)
target_link_libraries(prepare_graph bwi_mapper)

#test binaries
add_executable(test_circle test/test_circle.cpp)
target_link_libraries(test_circle bwi_mapper)

add_executable(test_map_loader test/test_map_loader.cpp)
target_link_libraries(test_map_loader bwi_mapper)

add_executable(test_voronoi test/test_voronoi.cpp)
target_link_libraries(test_voronoi bwi_mapper)

add_executable(test_dfs test/test_dfs.cpp)
target_link_libraries(test_dfs bwi_mapper)

add_executable(test_graph test/test_graph.cpp)
target_link_libraries(test_graph bwi_mapper)

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
  bwi_mapper
  generate_graph
  view_graph
  prepare_graph
  test_circle
  test_map_loader
  test_voronoi
  test_dfs
  test_graph
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


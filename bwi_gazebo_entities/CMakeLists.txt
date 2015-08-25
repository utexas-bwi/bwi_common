# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(bwi_gazebo_entities)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED
 COMPONENTS
   bwi_mapper
   bwi_msgs
   bwi_tools
   gazebo_msgs
   geometry_msgs
   nav_msgs
)
catkin_package(
  CATKIN_DEPENDS
   bwi_mapper
   bwi_msgs
   bwi_tools
   gazebo_msgs
   geometry_msgs
   nav_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_executable(random_persons_controller src/nodes/random_persons_controller.cpp)
target_link_libraries(random_persons_controller
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#install all targets
foreach(dir entities gazebo launch persons urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

# unit tests are enabled selectively
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

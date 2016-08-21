cmake_minimum_required(VERSION 2.8.3)
project(bwi_joystick_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(joystick_teleop_node src/joystick_teleop_node.cpp)
add_dependencies(joystick_teleop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_teleop_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS joystick_teleop_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/joystick_teleop.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


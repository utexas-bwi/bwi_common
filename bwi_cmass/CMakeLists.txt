cmake_minimum_required(VERSION 2.8.3)
project(cmass)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  multi_level_map_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmass_client
#  CATKIN_DEPENDS geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(cmass_client src/client.cpp)
target_link_libraries(cmass_client ${catkin_LIBRARIES} ${OPENSSL_LIBRARIES} curl )

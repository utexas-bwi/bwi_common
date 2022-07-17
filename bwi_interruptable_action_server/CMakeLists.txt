cmake_minimum_required(VERSION 2.8.3)
project(bwi_interruptable_action_server)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_tutorials
    roscpp
    std_srvs)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES bwi_interruptable_action_server
CATKIN_DEPENDS actionlib actionlib_tutorials roscpp std_srvs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fibonacci_server src/fibonacci.cpp)

target_link_libraries(fibonacci_server ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS fibonacci_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

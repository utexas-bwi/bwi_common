cmake_minimum_required(VERSION 2.8.3)
project(bwi_wandering)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(catkin REQUIRED 
    COMPONENTS
        ar_track_alvar
        actionlib
        actionlib_msgs
        bwi_kr_execution
        bwi_msgs
        move_base_msgs
        roscpp
        std_msgs
        visualization_msgs
        tf
)


find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
    CATKIN_DEPENDS 
        actionlib
        actionlib_msgs
        bwi_kr_execution
        bwi_msgs
        message_runtime
        move_base_msgs
        roscpp
        std_msgs
        visualization_msgs
        tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(wandering src/wandering.cpp )
add_dependencies(wandering ${catkin_EXPORTED_TARGETS})
target_link_libraries(wandering
    ${catkin_LIBRARIES})


add_executable(turning src/turn.cpp )
add_dependencies(wandering ${catkin_EXPORTED_TARGETS})
target_link_libraries(turning
    ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
          wandering 
          turning
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)
project(plan_execution)


if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  message_generation
  roscpp
  bwi_msgs
  std_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    AnswerSet.msg
    AspFluent.msg
    AspRule.msg
    HriMessage.msg)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    ComputeAllPlans.srv
    ComputePlan.srv
    CurrentStateQuery.srv
    IsPlanValid.srv
    UpdateFluents.srv
    GetHriMessage.srv)

## Generate actions in the 'action' folder
add_action_files(FILES ExecutePlan.action ExecuteSmachStateMachine.action)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this42 if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include actasp/include
  LIBRARIES actasp plan_execution
  CATKIN_DEPENDS
    actionlib_msgs
    bwi_msgs
    message_runtime
    roscpp
    std_msgs
    std_srvs
    bwi_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  actasp/include
)

set(actasp_SRC)
add_subdirectory(actasp/src/)
add_library(actasp ${actasp_SRC})
target_link_libraries(actasp ${catkin_LIBRARIES})

set(plan_execution_SRC)
add_subdirectory(src/libplan_execution)
add_library(plan_execution ${plan_execution_SRC})
add_dependencies(plan_execution ${catkin_EXPORTED_TARGETS})
add_dependencies(plan_execution ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(plan_execution actasp ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS actasp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS plan_execution
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


#############
## Testing ##
#############

catkin_add_gtest(test_actasp test/actasp.cpp)
add_dependencies(test_actasp ${plan_execution_EXPORTED_TARGETS})
target_link_libraries(test_actasp actasp ${catkin_LIBRARIES})

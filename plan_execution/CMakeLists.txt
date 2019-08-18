cmake_minimum_required(VERSION 2.8.3)
project(plan_execution)


if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  message_generation
  roscpp
  bwi_msgs
  std_msgs
  std_srvs
)

include(ExternalProject)
set(CLINGO_BUILD_APPS OFF)
ExternalProject_Add(clingo
        PREFIX 3rd_party
        GIT_REPOSITORY https://github.com/potassco/clingo.git
        GIT_TAG        v5.3.0
        UPDATE_COMMAND ""
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
        )


SET(CLINGO_INCLUDE_DIR ${CATKIN_DEVEL_PREFIX}/include)
SET(CLINGO_LIB_DIR ${CATKIN_DEVEL_PREFIX}/lib)

add_library(libclingo SHARED IMPORTED)
set_target_properties(libclingo PROPERTIES IMPORTED_LOCATION ${CLINGO_LIB_DIR}/libclingo.so)

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

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  actasp/include
)

set(actasp_SRC)
add_subdirectory(actasp/src/)
add_library(actasp ${actasp_SRC})
add_dependencies(actasp clingo)
target_link_libraries(actasp libclingo ${catkin_LIBRARIES})

set(plan_execution_SRC)
add_subdirectory(src/libplan_execution)
add_library(plan_execution ${plan_execution_SRC})
add_dependencies(plan_execution ${catkin_EXPORTED_TARGETS})
add_dependencies(plan_execution ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(plan_execution actasp ${catkin_LIBRARIES})

#############
## Install ##
#############

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

catkin_add_gtest(test_actasp test/main.cpp test/asptypes.cpp test/solver.cpp test/reasoner.cpp)
add_dependencies(test_actasp ${plan_execution_EXPORTED_TARGETS})
target_link_libraries(test_actasp actasp ${catkin_LIBRARIES})

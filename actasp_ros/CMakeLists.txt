cmake_minimum_required(VERSION 2.8.3)
project(actasp_ros)


if ("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
    set(CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS
        actasp
        actionlib
        actionlib_msgs
        message_generation
        roscpp
        bwi_msgs
        std_msgs
        std_srvs
        )


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        AnswerSet.msg
        AspFluent.msg
        AspFunction.msg
        AspRule.msg)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        ComputeAllPlans.srv
        ComputePlan.srv
        CurrentStateQuery.srv
        IsPlanValid.srv)

## Generate actions in the 'action' folder
add_action_files(FILES ExecutePlan.action ExecuteSmachStateMachine.action)

catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libactasp_ros
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

)

set(libactasp_ros_SRC)
add_subdirectory(src/libactasp_ros)
add_library(libactasp_ros ${libactasp_ros_SRC})
add_dependencies(libactasp_ros ${catkin_EXPORTED_TARGETS})
add_dependencies(libactasp_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(libactasp_ros ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS libactasp_ros
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



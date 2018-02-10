cmake_minimum_required(VERSION 2.8.3)
project(bwi_kr_execution)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  bwi_msgs
  message_generation
  roscpp
  roslib
  sound_play
  std_msgs
  std_srvs
  plan_execution
)
 
catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS
    actionlib_msgs
    bwi_msgs
    message_runtime
    roscpp
    sound_play
    std_msgs
    std_srvs
    plan_execution
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${actasp_INCLUDE_DIRS}
  ${plan_execution_INCLUDE_DIRS}
)

set(bwikractions_SRC)
add_subdirectory(src/actions/)
add_library(bwikractions ${bwikractions_SRC})
add_dependencies(bwikractions ${catkin_EXPORTED_TARGETS})
target_link_libraries(bwikractions ${catkin_LIBRARIES})

foreach(dir domain domain_dialog domain_simulation launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(TARGETS bwikractions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


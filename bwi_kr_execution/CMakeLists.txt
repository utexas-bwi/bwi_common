cmake_minimum_required(VERSION 2.8.3)
project(bwi_kr_execution)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  bwi_msgs
  bwi_services
  message_generation
  plan_execution
  roscpp
  roslib
  std_msgs
  std_srvs
)
 
catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS
    actionlib_msgs
    bwi_msgs
    bwi_services
    message_runtime
    plan_execution
    roscpp
    std_msgs
    std_srvs
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


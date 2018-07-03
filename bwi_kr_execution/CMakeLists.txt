cmake_minimum_required(VERSION 2.8.3)
project(bwi_kr_execution)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs 
  bwi_msgs
  bwi_services
  knowledge_representation
  message_generation
  plan_execution
  roscpp
  roslib
  std_msgs
  std_srvs
)
 
catkin_package(
  INCLUDE_DIRS 
  LIBRARIES bwikractions
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
)

set(bwikractions_SRC)
add_subdirectory(src/actions/)
add_library(bwikractions ${bwikractions_SRC})
add_dependencies(bwikractions ${catkin_EXPORTED_TARGETS})
target_link_libraries(bwikractions ${catkin_LIBRARIES})

set(spexec_SRC)
set(anyexec_SRC src/observers.h)
set(lexec_SRC)
set(xpexec_SRC)
set(krreasoner_SRC)

add_subdirectory(src)
add_executable(plan_executor_node ${spexec_SRC})
target_link_libraries(plan_executor_node
  ${catkin_LIBRARIES} bwikractions)

#add_executable(any_plan_executor_node ${anyexec_SRC})
#target_link_libraries(any_plan_executor_node
#  ${catkin_LIBRARIES} bwikractions)

#add_executable(learning_executor_node ${lexec_SRC})
#target_link_libraries(learning_executor_node
#  ${catkin_LIBRARIES} bwikractions)

add_executable(asp_formatter src/asp_formatter.cpp)
target_link_libraries(asp_formatter ${catkin_LIBRARIES})


foreach(dir domain domain_dialog domain_simulation launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(TARGETS bwikractions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS
          asp_formatter
          plan_executor_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


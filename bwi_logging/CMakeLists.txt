cmake_minimum_required(VERSION 2.8.3)
project(bwi_logging)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    nav_msgs
    roscpp
    roslint
    sensor_msgs)

catkin_python_setup()

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_filters
    nav_msgs
    roscpp
    sensor_msgs)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_executable(frame_recording src/frame_recording.cpp)
target_link_libraries(frame_recording ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS frame_recording
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS nodes/record
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/upload
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_NODES
     RELATIVE ${PROJECT_SOURCE_DIR} nodes/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/*.py)
roslint_python(${${PROJECT_NAME}_PY_NODES}
               ${${PROJECT_NAME}_PY_SRCS})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

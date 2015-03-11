cmake_minimum_required(VERSION 2.8.3)

project(bwi_tools)

find_package(catkin REQUIRED COMPONENTS cmake_modules roslib)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(TBB REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roslib
  DEPENDS 
    OpenCV
    BOOST
  INCLUDE_DIRS include ${TBB_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_json ${TBB_LIBRARIES}
)

catkin_python_setup()

## Build
ADD_DEFINITIONS(-std=c++0x)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## The main bwi_rl library
add_library(${PROJECT_NAME}_json
  src/json/json_reader.cpp
  src/json/json_value.cpp
  src/json/json_writer.cpp
)

add_library(${PROJECT_NAME}
  src/common/Enum.cpp
  src/common/Util.cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_json
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

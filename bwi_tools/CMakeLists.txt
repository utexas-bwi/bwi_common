cmake_minimum_required(VERSION 2.8.3)
project(bwi_tools)
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
catkin_package(
  DEPENDS 
    opencv2
    boost
  INCLUDE_DIRS
    include/
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  LIBRARIES
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)
catkin_python_setup()
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

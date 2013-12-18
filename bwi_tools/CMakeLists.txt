cmake_minimum_required(VERSION 2.8.3)
project(bwi_tools)
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
  DEPENDS 
    opencv2
  INCLUDE_DIRS
    include/
    ${OpenCV_INCLUDE_DIRS}
  LIBRARIES
    ${OpenCV_LIBRARIES}
)
catkin_python_setup()
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

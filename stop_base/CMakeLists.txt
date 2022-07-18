cmake_minimum_required(VERSION 2.8.3)
project(stop_base)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS bwi_msgs geometry_msgs roslint)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS bwi_msgs geometry_msgs)

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_NODES
     RELATIVE ${PROJECT_SOURCE_DIR} nodes/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_NODES}
               ${${PROJECT_NAME}_PY_SRCS})

catkin_install_python(PROGRAMS nodes/request nodes/stop_base_controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

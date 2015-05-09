cmake_minimum_required(VERSION 2.8.3)
project(stop_base)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_python_setup()
catkin_package()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_NODES
     RELATIVE ${PROJECT_SOURCE_DIR} nodes/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS 
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS}
               ${${PROJECT_NAME}_PY_SRCS})

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

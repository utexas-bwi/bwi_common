cmake_minimum_required(VERSION 2.8.3)
project(actasp)


find_package(catkin REQUIRED COMPONENTS

)

include(ExternalProject)
set(CLINGO_BUILD_APPS OFF)
ExternalProject_Add(clingo
        PREFIX 3rd_party
        GIT_REPOSITORY https://github.com/potassco/clingo.git
        GIT_TAG        v5.3.0
        UPDATE_COMMAND ""
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
        )


SET(CLINGO_INCLUDE_DIR ${CATKIN_DEVEL_PREFIX}/include)
SET(CLINGO_LIB_DIR ${CATKIN_DEVEL_PREFIX}/lib)

add_library(libclingo SHARED IMPORTED)
set_target_properties(libclingo PROPERTIES IMPORTED_LOCATION ${CLINGO_LIB_DIR}/libclingo.so)


catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include ${CLINGO_INCLUDE_DIR}
  LIBRARIES actasp libclingo
  CATKIN_DEPENDS

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CLINGO_INCLUDE_DIR}
  include
)

set(actasp_SRC)
add_subdirectory(src/)
add_library(actasp ${actasp_SRC})
add_dependencies(actasp clingo)
target_link_libraries(actasp libclingo ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS actasp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

#############
## Testing ##
#############

catkin_add_gtest(test_actasp test/main.cpp test/asptypes.cpp test/solver.cpp test/reasoner.cpp)
add_dependencies(test_actasp actasp)
target_link_libraries(test_actasp actasp ${catkin_LIBRARIES})

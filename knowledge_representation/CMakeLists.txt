cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)


if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()
find_package(BISON)
find_package(FLEX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        bwi_perception
        roscpp
        pcl_ros
        pcl_conversions
        std_msgs
        sensor_msgs
        )

find_package(Boost REQUIRED COMPONENTS python)

find_package(PythonLibs REQUIRED)

find_package(Eigen3 REQUIRED)

find_library(mysql NAMES libmysqlcppconn8.so)
if($mysql_FOUND)
    set(DB_INCLUDES /usr/local/mysql/connector-c++-8.0/include)
    set(DB_LIBS ${mysql})
else()
    message("MySQL Connector C++ 8.0 was not found. This backend will not be available.")
endif()


link_directories(
        ${catkin_LINK_DIRS}
)

add_service_files(
        FILES
        AddObject.srv
        GetCloud.srv
        RemoveObject.srv
        ResolveObjectCorrespondences.srv
)
catkin_python_setup()

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)



catkin_package(
        INCLUDE_DIRS include ${DB_INCLUDES}
        LIBRARIES libknowledge_rep
        CATKIN_DEPENDS
        pcl_ros
        pcl_conversions
        roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ${DB_INCLUDES}
        ${Eigen3_INCLUDE_DIRS}
)

BISON_TARGET(parser src/libknowledge_rep/parser.yy ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Parser.cpp COMPILE_FLAGS --defines=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Parser.h)
FLEX_TARGET(scanner src/libknowledge_rep/lexer.l  ${CMAKE_CURRENT_BINARY_DIR}/Lexer.cpp)


ADD_FLEX_BISON_DEPENDENCY(scanner parser)

add_library(libknowledge_rep
        src/libknowledge_rep/LongTermMemoryConduit.cpp
        src/libknowledge_rep/MemoryConduit.cpp
        src/libknowledge_rep/convenience.cpp
        src/libknowledge_rep/Entity.cpp
        src/libknowledge_rep/Instance.cpp
        src/libknowledge_rep/Concept.cpp
        src/libknowledge_rep/Compiler.cpp
        ${BISON_parser_OUTPUTS}
        ${FLEX_scanner_OUTPUTS} )

add_dependencies(libknowledge_rep ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(libknowledge_rep ${DB_LIBS} ${catkin_LIBRARIES})

add_library(_libknowledge_rep_wrapper_cpp src/libknowledge_rep/python_wrapper.cpp)
add_dependencies(_libknowledge_rep_wrapper_cpp ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(_libknowledge_rep_wrapper_cpp
        libknowledge_rep
        ${DB_LIBS}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(memory_conduit_proxy_node src/memory_conduit_proxy_node.cpp)
add_dependencies(memory_conduit_proxy_node ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(memory_conduit_proxy_node libknowledge_rep ${catkin_LIBRARIES})

add_executable(show_me src/show_me.cpp)
add_dependencies(show_me ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(show_me libknowledge_rep ${catkin_LIBRARIES})

### TEST TARGETS

catkin_add_gtest(test_ltmc test/ltmc.cpp)
add_dependencies(test_ltmc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_ltmc libknowledge_rep ${catkin_LIBRARIES})

catkin_add_gtest(test_mc test/mc.cpp)
add_dependencies(test_mc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_mc libknowledge_rep ${catkin_LIBRARIES})

catkin_add_gtest(test_parser test/parser.cpp)
add_dependencies(test_parser ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_parser libknowledge_rep ${catkin_LIBRARIES})

###


set_target_properties(_libknowledge_rep_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

catkin_add_env_hooks(${PROJECT_NAME} SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(TARGETS libknowledge_rep
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS
        test_ltmc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
